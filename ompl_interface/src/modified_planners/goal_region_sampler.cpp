/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Juan David Hernandez Vega */
/* Extension of constrained_goal_sampler by: (Ioan Sucan) */

#include <moveit/ompl_interface/modified_planners/goal_region_sampler.h>
#include "moveit/ompl_interface/detail/state_validity_checker.h"
#include "moveit/ompl_interface/ompl_planning_context.h"
#include <moveit/profiler/profiler.h>
#include <tf/tf.h>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/pending/disjoint_sets.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/property_map/vector_property_map.hpp>
#include <boost/foreach.hpp>

ompl_interface::GoalRegionSampler::GoalRegionSampler(const OMPLPlanningContext* pc, const std::string& group_name,
                                                     const robot_model::RobotModelConstPtr& rm,
                                                     const planning_scene::PlanningSceneConstPtr& ps,
                                                     const std::vector<moveit_msgs::Constraints>& constrs,
                                                     const std::vector<moveit_msgs::WorkspaceGoalRegion>& wsgrs,
                                                     constraint_samplers::ConstraintSamplerManagerPtr csm,
                                                     const unsigned int max_sampled_goals)
  : ompl::base::WeightedGoalRegionSampler(pc->getOMPLSpaceInformation(),
                                          boost::bind(&GoalRegionSampler::sampleUsingConstraintSampler, this, _1, _2),
                                          max_sampled_goals, false)
  , planning_context_(pc)
  , work_state_(pc->getCompleteInitialRobotState())
  , invalid_sampled_constraints_(0)
  , warned_invalid_samples_(false)
  , verbose_display_(0)
  , planning_scene_(ps)
  , constraint_sampler_manager_(csm)
  , group_name_(group_name)
  , workspace_goal_regions_(wsgrs)
{
  // std::cout << "creating GoalRegionSampler! " << std::endl;

  for (auto& constr : constrs)
    constrs_.push_back(moveit_msgs::Constraints(constr));

  for (std::size_t i = 0; i < workspace_goal_regions_.size(); ++i)
  {
    // construct the se3 state space for sampling poses
    se3_spaces_.push_back(ompl::base::StateSpacePtr(new ompl::base::SE3StateSpace()));

    // set the bounds for the R^3 part of SE(3)
    ompl::base::RealVectorBounds bounds(3);
    bounds.setLow(0, workspace_goal_regions_[i].x.min);
    bounds.setLow(1, workspace_goal_regions_[i].y.min);
    bounds.setLow(2, workspace_goal_regions_[i].z.min);

    bounds.setHigh(0, workspace_goal_regions_[i].x.max);
    bounds.setHigh(1, workspace_goal_regions_[i].y.max);
    bounds.setHigh(2, workspace_goal_regions_[i].z.max);

    se3_spaces_[i]->as<ompl::base::SE3StateSpace>()->setBounds(bounds);
    se3_samplers_.push_back(se3_spaces_[i]->as<ompl::base::SE3StateSpace>()->allocStateSampler());

    OMPL_DEBUG("Creating SE3 workspace sampler for GoalRegion%d", i + 1);
  }

  //
  kinematic_constraint_set_.reset(new kinematic_constraints::KinematicConstraintSet(rm));

  OMPL_DEBUG("Creating PRM for Goal Regions");
  // construct the state space we are planning in
  auto prm_space = planning_context_->getOMPLStateSpace();
  // construct an instance of  space information from this state space
  auto prm_si(std::make_shared<ompl::base::SpaceInformation>(planning_context_->getOMPLStateSpace()));
  // set state validity checking for this space
  //  prm_si->setStateValidityChecker(
  //      std::make_shared<plan::ValidityChecker>(prm_si, std::make_shared<Box2DCollisionManager>(robot_)));
  // create a problem instance
  auto prm_pdef(std::make_shared<ompl::base::ProblemDefinition>(prm_si));
  // create a planner for the defined space
  prm_planner_ = std::make_shared<ompl::geometric::PRMMod>(prm_si);
  // set the problem we are trying to solve for the planner
  prm_planner_->setProblemDefinition(prm_pdef);
  // perform setup steps for the planner
  prm_planner_->setup();/*
  // Set a goal regions state sampler
  prm_planner_->getSpaceInformation()->getStateSpace()->setStateSamplerAllocator(
      std::bind(ob::newAllocStateSampler, std::placeholders::_1, this));*/

  startSampling();
  //  startGrowingRoadmap();
}

bool ompl_interface::GoalRegionSampler::checkStateValidity(ompl::base::State* new_goal,
                                                           const robot_state::RobotState& state, bool verbose) const
{
  planning_context_->copyToOMPLState(new_goal, state);
  return dynamic_cast<const StateValidityChecker*>(si_->getStateValidityChecker().get())->isValid(new_goal, verbose);
}

bool ompl_interface::GoalRegionSampler::stateValidityCallback(ompl::base::State* new_goal,
                                                              robot_state::RobotState const* state,
                                                              const robot_model::JointModelGroup* jmg,
                                                              const double* jpos, bool verbose) const
{
  // we copy the state to not change the seed state
  robot_state::RobotState solution_state(*state);
  solution_state.setJointGroupPositions(jmg, jpos);
  solution_state.update();
  return checkStateValidity(new_goal, solution_state, verbose);
}

bool ompl_interface::GoalRegionSampler::sampleUsingConstraintSampler(const ompl::base::WeightedGoalRegionSampler* gls,
                                                                     std::vector<ompl::base::State*>& sampled_states)
{
  bool success = false;

  for (std::size_t i = 0; i < workspace_goal_regions_.size(); ++i)
  {
    // Sampling an SE3 pose
    //    std::cout << "sampling !!!!!:" << std::endl;
    ompl::base::State* state = se3_spaces_[i]->as<ompl::base::SE3StateSpace>()->allocState();
    se3_samplers_[i]->sampleUniform(state);

    //    std::cout << "sampled SE3 pose:" << std::endl;
    //    se3_spaces_[i]->as<ompl::base::SE3StateSpace>()->printState(state, std::cout);

    kinematic_constraint_set_->clear();

    constrs_[i].position_constraints[0].constraint_region.primitive_poses[0].position.x =
        state->as<ompl::base::SE3StateSpace::StateType>()->getX();
    constrs_[i].position_constraints[0].constraint_region.primitive_poses[0].position.y =
        state->as<ompl::base::SE3StateSpace::StateType>()->getY();
    constrs_[i].position_constraints[0].constraint_region.primitive_poses[0].position.z =
        state->as<ompl::base::SE3StateSpace::StateType>()->getZ();

    if (workspace_goal_regions_[i].roll.free_value || workspace_goal_regions_[i].pitch.free_value ||
        workspace_goal_regions_[i].yaw.free_value)
    {
      // sampled orientation
      tf::Quaternion q_sampled = tf::Quaternion(state->as<ompl::base::SE3StateSpace::StateType>()->rotation().x,
                                                state->as<ompl::base::SE3StateSpace::StateType>()->rotation().y,
                                                state->as<ompl::base::SE3StateSpace::StateType>()->rotation().z,
                                                state->as<ompl::base::SE3StateSpace::StateType>()->rotation().w);
      tf::Matrix3x3 rotation_sampled(q_sampled);
      double roll_sampled, pitch_sampled, yaw_sampled;
      rotation_sampled.getRPY(roll_sampled, pitch_sampled, yaw_sampled);

      // initial orientation
      tf::Quaternion q_initial_goal = tf::Quaternion(
          constrs_[i].orientation_constraints[0].orientation.x, constrs_[i].orientation_constraints[0].orientation.y,
          constrs_[i].orientation_constraints[0].orientation.z, constrs_[i].orientation_constraints[0].orientation.w);
      tf::Matrix3x3 roation_initial_goal(q_initial_goal);
      double roll, pitch, yaw;
      roation_initial_goal.getRPY(roll, pitch, yaw);

      // new orientation
      tf::Quaternion q_new =
          tf::createQuaternionFromRPY(workspace_goal_regions_[i].roll.free_value ? roll_sampled : roll,
                                      workspace_goal_regions_[i].pitch.free_value ? pitch_sampled : pitch,
                                      workspace_goal_regions_[i].yaw.free_value ? yaw_sampled : yaw);

      // new orientation constraints
      constrs_[i].orientation_constraints[0].orientation.x = q_new[0];
      constrs_[i].orientation_constraints[0].orientation.y = q_new[1];
      constrs_[i].orientation_constraints[0].orientation.z = q_new[2];
      constrs_[i].orientation_constraints[0].orientation.w = q_new[3];
    }

    kinematic_constraint_set_->add(constrs_[i], planning_scene_->getTransforms());
    constraint_sampler_ = constraint_sampler_manager_->selectSampler(planning_scene_, group_name_,
                                                                     kinematic_constraint_set_->getAllConstraints());

    se3_spaces_[i]->freeState(state);

    //  moveit::Profiler::ScopedBlock
    //  sblock("GoalRegionSampler::sampleUsingConstraintSampler");

    // unsigned int max_attempts =
    // planning_context_->getMaximumGoalSamplingAttempts();
    unsigned int max_attempts = 20;
    unsigned int attempts_so_far = gls->samplingAttemptsCount();

    //    // terminate after too many attempts
    //    if (attempts_so_far >= max_attempts)
    //      continue;  // return false;

    // terminate after a maximum number of samples
    // if (gls->getStateCount() >= planning_context_->getMaximumGoalSamples())
    //    unsigned int max_goal_samples = 50;
    //    if (gls->getStateCount() >= max_goal_samples)
    //      continue;  // return false;

    // terminate the sampling thread when a solution has been found
    if (planning_context_->getOMPLProblemDefinition()->hasSolution())
      continue;  // return false;

    ompl::base::State* goal = si_->allocState();
    unsigned int max_attempts_div2 = max_attempts / 2;
    for (unsigned int a = 0; a < max_attempts && gls->isSampling(); ++a)
    {
      bool verbose = false;
      if (gls->getStateCount() == 0 && a >= max_attempts_div2)
        if (verbose_display_ < 1)
        {
          verbose = true;
          verbose_display_++;
        }

      if (constraint_sampler_)
      {
        // makes the constraint sampler also perform a validity callback
        robot_state::GroupStateValidityCallbackFn gsvcf =
            boost::bind(&ompl_interface::GoalRegionSampler::stateValidityCallback, this, goal,
                        _1,  // pointer to state
                        _2,  // const* joint model group
                        _3,  // double* of joint positions
                        verbose);
        constraint_sampler_->setGroupStateValidityCallback(gsvcf);

        unsigned int max_state_sampling_attempts = 4;
        // if (constraint_sampler_->project(work_state_,
        // planning_context_->getMaximumStateSamplingAttempts()))
        if (constraint_sampler_->project(work_state_, max_state_sampling_attempts))
        {
          work_state_.update();
          if (kinematic_constraint_set_->decide(work_state_, verbose).satisfied)
          {
            if (checkStateValidity(goal, work_state_, verbose))
            {
              ompl::base::State* new_goal = si_->allocState();
              si_->copyState(new_goal, goal);

              sampled_states.push_back(new_goal);
              WeightedGoal* weighted_state = new WeightedGoal;
              weighted_state->state_ = new_goal;
              weighted_state->weight_ = 1.0;
              weighted_state->heap_element_ = goals_priority_queue_.insert(weighted_state);
              success = true;
              break;  // return true;
            }
          }
          else
          {
            invalid_sampled_constraints_++;
            if (!warned_invalid_samples_ && invalid_sampled_constraints_ >= (attempts_so_far * 8) / 10)
            {
              warned_invalid_samples_ = true;
              //              logWarn("More than 80%% of the sampled goal states fail to satisfy "
              //                      "the constraints imposed on the goal "
              //                      "sampler. Is the constrained sampler working correctly?");
            }
          }
        }
      }
      else
      {
        default_sampler_->sampleUniform(goal);
        if (dynamic_cast<const StateValidityChecker*>(si_->getStateValidityChecker().get())->isValid(goal, verbose))
        {
          planning_context_->copyToRobotState(work_state_, goal);
          if (kinematic_constraint_set_->decide(work_state_, verbose).satisfied)
          {
            ompl::base::State* new_goal = si_->allocState();
            si_->copyState(new_goal, goal);

            WeightedGoal* weighted_state = new WeightedGoal;
            weighted_state->state_ = new_goal;
            weighted_state->weight_ = 1.0;
            weighted_state->heap_element_ = goals_priority_queue_.insert(weighted_state);
            success = true;
            break;  // return true;
          }
        }
      }
    }
    si_->freeState(goal);
  }
  if (success)
    return true;
  else
    return false;
}

void ompl_interface::GoalRegionSampler::clear()
{
  std::lock_guard<std::mutex> slock(lock_);
  WeightedGoalRegionSampler::clear();
  constrs_.clear();
  workspace_goal_regions_.clear();
  se3_samplers_.clear();
  se3_spaces_.clear();
}
