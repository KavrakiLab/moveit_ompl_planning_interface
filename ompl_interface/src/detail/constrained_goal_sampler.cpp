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

/* Author: Ioan Sucan */

#include "moveit/ompl_interface/detail/constrained_goal_sampler.h"
#include "moveit/ompl_interface/detail/state_validity_checker.h"
#include "moveit/ompl_interface/ompl_planning_context.h"
#include <moveit/profiler/profiler.h>
#include <tf/tf.h>

ompl_interface::ConstrainedGoalSampler::ConstrainedGoalSampler(
    const OMPLPlanningContext* pc, const kinematic_constraints::KinematicConstraintSetPtr& ks,
    const constraint_samplers::ConstraintSamplerPtr& cs)
  : ompl::base::GoalLazySamples(pc->getOMPLSpaceInformation(),
                                boost::bind(&ConstrainedGoalSampler::sampleUsingConstraintSampler, this, _1, _2), false)
  , planning_context_(pc)
  , kinematic_constraint_set_(ks)
  , constraint_sampler_(cs)
  , work_state_(pc->getCompleteInitialRobotState())
  , invalid_sampled_constraints_(0)
  , warned_invalid_samples_(false)
  , verbose_display_(0)
{
  if (!constraint_sampler_)
    default_sampler_ = si_->allocStateSampler();
  logDebug("Constructed a ConstrainedGoalSampler instance at address %p", this);
  startSampling();
  std::cout << "After start sampling goals: !!!!!!!!!!!!!!++++" << std::endl;
}

bool ompl_interface::ConstrainedGoalSampler::checkStateValidity(ompl::base::State* new_goal,
                                                                const robot_state::RobotState& state,
                                                                bool verbose) const
{
  planning_context_->copyToOMPLState(new_goal, state);
  return dynamic_cast<const StateValidityChecker*>(si_->getStateValidityChecker().get())->isValid(new_goal, verbose);
}

bool ompl_interface::ConstrainedGoalSampler::stateValidityCallback(ompl::base::State* new_goal,
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

bool ompl_interface::ConstrainedGoalSampler::sampleUsingConstraintSampler(const ompl::base::GoalLazySamples* gls,
                                                                          ompl::base::State* new_goal)
{
  //  moveit::Profiler::ScopedBlock
  //  sblock("ConstrainedGoalSampler::sampleUsingConstraintSampler");

  // unsigned int max_attempts =
  // planning_context_->getMaximumGoalSamplingAttempts();
  unsigned int max_attempts = 1000;
  unsigned int attempts_so_far = gls->samplingAttemptsCount();

  // terminate after too many attempts
  if (attempts_so_far >= max_attempts)
    return false;

  // terminate after a maximum number of samples
  // if (gls->getStateCount() >= planning_context_->getMaximumGoalSamples())
  unsigned int max_goal_samples = 50;
  if (gls->getStateCount() >= max_goal_samples)
    return false;

  // terminate the sampling thread when a solution has been found
  if (planning_context_->getOMPLProblemDefinition()->hasSolution())
    return false;

  unsigned int max_attempts_div2 = max_attempts / 2;
  for (unsigned int a = gls->samplingAttemptsCount(); a < max_attempts && gls->isSampling(); ++a)
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
          boost::bind(&ompl_interface::ConstrainedGoalSampler::stateValidityCallback, this, new_goal,
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
          if (checkStateValidity(new_goal, work_state_, verbose))
            return true;
        }
        else
        {
          invalid_sampled_constraints_++;
          if (!warned_invalid_samples_ && invalid_sampled_constraints_ >= (attempts_so_far * 8) / 10)
          {
            warned_invalid_samples_ = true;
            logWarn("More than 80%% of the sampled goal states fail to satisfy "
                    "the constraints imposed on the goal "
                    "sampler. Is the constrained sampler working correctly?");
          }
        }
      }
    }
    else
    {
      default_sampler_->sampleUniform(new_goal);
      if (dynamic_cast<const StateValidityChecker*>(si_->getStateValidityChecker().get())->isValid(new_goal, verbose))
      {
        planning_context_->copyToRobotState(work_state_, new_goal);
        if (kinematic_constraint_set_->decide(work_state_, verbose).satisfied)
          return true;
      }
    }
  }
  return false;
}

// ConstrainedGoalRegionSampler

ompl_interface::ConstrainedGoalRegionSampler::ConstrainedGoalRegionSampler(
    const OMPLPlanningContext* pc, const std::string& group_name, const robot_model::RobotModelConstPtr& rm,
    const planning_scene::PlanningSceneConstPtr& ps, moveit_msgs::Constraints& constr,
    const moveit_msgs::GoalRegion& gr, constraint_samplers::ConstraintSamplerManagerPtr csm,
    const constraint_samplers::ConstraintSamplerPtr& cs)
  : ompl::base::GoalLazySamples(pc->getOMPLSpaceInformation(),
                                boost::bind(&ConstrainedGoalRegionSampler::sampleUsingConstraintSampler, this, _1, _2),
                                false)
  , planning_context_(pc)
  , constraint_sampler_(cs)
  , work_state_(pc->getCompleteInitialRobotState())
  , invalid_sampled_constraints_(0)
  , warned_invalid_samples_(false)
  , verbose_display_(0)
  , goal_region_(moveit_msgs::GoalRegion(gr))
  , constr_(moveit_msgs::Constraints(constr))
  , planning_scene_(ps)
  , constraint_sampler_manager_(csm)
  , group_name_(group_name)
{
  if (!constraint_sampler_)
    default_sampler_ = si_->allocStateSampler();

  // construct the se3 state space for sampling poses
  space_ = ompl::base::StateSpacePtr(new ompl::base::SE3StateSpace());

  // set the bounds for the R^3 part of SE(3)
  ompl::base::RealVectorBounds bounds(3);
  bounds.setLow(0, goal_region_.x.min);
  bounds.setLow(1, goal_region_.y.min);
  bounds.setLow(2, goal_region_.z.min);

  bounds.setHigh(0, goal_region_.x.max);
  bounds.setHigh(1, goal_region_.y.max);
  bounds.setHigh(2, goal_region_.z.max);

  space_->as<ompl::base::SE3StateSpace>()->setBounds(bounds);

  if (!se3_sampler_)
    se3_sampler_ = space_->as<ompl::base::SE3StateSpace>()->allocStateSampler();

  //
  kinematic_constraint_set_.reset(new kinematic_constraints::KinematicConstraintSet(rm));

  logDebug("Constructed a ConstrainedGoalRegionSampler instance at address %p", this);
  startSampling();
}

bool ompl_interface::ConstrainedGoalRegionSampler::checkStateValidity(ompl::base::State* new_goal,
                                                                      const robot_state::RobotState& state,
                                                                      bool verbose) const
{
  planning_context_->copyToOMPLState(new_goal, state);
  return dynamic_cast<const StateValidityChecker*>(si_->getStateValidityChecker().get())->isValid(new_goal, verbose);
}

bool ompl_interface::ConstrainedGoalRegionSampler::stateValidityCallback(ompl::base::State* new_goal,
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

bool ompl_interface::ConstrainedGoalRegionSampler::sampleUsingConstraintSampler(const ompl::base::GoalLazySamples* gls,
                                                                                ompl::base::State* new_goal)
{
  std::cout << "sampling !!!!!:" << std::endl;
  ompl::base::State* state = space_->as<ompl::base::SE3StateSpace>()->allocState();
  se3_sampler_->sampleUniform(state);
  space_->as<ompl::base::SE3StateSpace>()->printState(state, std::cout);

  std::cout << "X: " << state->as<ompl::base::SE3StateSpace::StateType>()->getX() << std::endl;
  std::cout << "Y: " << state->as<ompl::base::SE3StateSpace::StateType>()->getY() << std::endl;
  std::cout << "Z: " << state->as<ompl::base::SE3StateSpace::StateType>()->getZ() << std::endl;

  kinematic_constraint_set_->clear();

  constr_.position_constraints[0].constraint_region.primitive_poses[0].position.x =
      state->as<ompl::base::SE3StateSpace::StateType>()->getX();
  constr_.position_constraints[0].constraint_region.primitive_poses[0].position.y =
      state->as<ompl::base::SE3StateSpace::StateType>()->getY();
  constr_.position_constraints[0].constraint_region.primitive_poses[0].position.z =
      state->as<ompl::base::SE3StateSpace::StateType>()->getZ();

  if (goal_region_.roll.free_value || goal_region_.pitch.free_value || goal_region_.yaw.free_value)
  {
    std::cout << "free value: " << std::endl;
    tf::Quaternion q_sampled = tf::Quaternion(state->as<ompl::base::SE3StateSpace::StateType>()->rotation().x,
                                              state->as<ompl::base::SE3StateSpace::StateType>()->rotation().y,
                                              state->as<ompl::base::SE3StateSpace::StateType>()->rotation().z,
                                              state->as<ompl::base::SE3StateSpace::StateType>()->rotation().w);
    tf::Matrix3x3 m_sampled(q_sampled);
    double roll_sampled, pitch_sampled, yaw_sampled;
    m_sampled.getRPY(roll_sampled, pitch_sampled, yaw_sampled);

    tf::Quaternion q = tf::Quaternion(
        constr_.orientation_constraints[0].orientation.x, constr_.orientation_constraints[0].orientation.y,
        constr_.orientation_constraints[0].orientation.z, constr_.orientation_constraints[0].orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    std::cout << "roll_sampled: " << roll_sampled << std::endl;
    std::cout << "roll: " << roll << std::endl;
    std::cout << "pitch_sampled: " << pitch_sampled << std::endl;
    std::cout << "pitch: " << pitch << std::endl;
    std::cout << "yaw_sampled: " << yaw_sampled << std::endl;
    std::cout << "yaw: " << yaw << std::endl;

    tf::Quaternion q_new = tf::createQuaternionFromRPY(goal_region_.roll.free_value ? roll_sampled : roll,
                                                       goal_region_.pitch.free_value ? pitch_sampled : pitch,
                                                       goal_region_.yaw.free_value ? yaw_sampled : yaw);

    constr_.orientation_constraints[0].orientation.x = q_new[0];
    constr_.orientation_constraints[0].orientation.y = q_new[1];
    constr_.orientation_constraints[0].orientation.z = q_new[2];
    constr_.orientation_constraints[0].orientation.w = q_new[3];
  }

  kinematic_constraint_set_->add(constr_, planning_scene_->getTransforms());
  constraint_sampler_ = constraint_sampler_manager_->selectSampler(planning_scene_, group_name_,
                                                                   kinematic_constraint_set_->getAllConstraints());

  std::cout << "kinematic constraint x: "
            << kinematic_constraint_set_->getPositionConstraints()[0].constraint_region.primitive_poses[0].position.x
            << std::endl;
  std::cout << "kinematic constraint y: "
            << kinematic_constraint_set_->getPositionConstraints()[0].constraint_region.primitive_poses[0].position.y
            << std::endl;
  std::cout << "kinematic constraint z: "
            << kinematic_constraint_set_->getPositionConstraints()[0].constraint_region.primitive_poses[0].position.z
            << std::endl;
  std::cout << "kinematic constraint qx: " << kinematic_constraint_set_->getOrientationConstraints()[0].orientation.x
            << std::endl;
  std::cout << "kinematic constraint qy: " << kinematic_constraint_set_->getOrientationConstraints()[0].orientation.y
            << std::endl;
  std::cout << "kinematic constraint qz: " << kinematic_constraint_set_->getOrientationConstraints()[0].orientation.z
            << std::endl;
  std::cout << "kinematic constraint qw: " << kinematic_constraint_set_->getOrientationConstraints()[0].orientation.w
            << std::endl;

  space_->freeState(state);

  //  moveit::Profiler::ScopedBlock
  //  sblock("ConstrainedGoalRegionSampler::sampleUsingConstraintSampler");

  // unsigned int max_attempts =
  // planning_context_->getMaximumGoalSamplingAttempts();
  unsigned int max_attempts = 1000;
  unsigned int attempts_so_far = gls->samplingAttemptsCount();

  // terminate after too many attempts
  if (attempts_so_far >= max_attempts)
    return false;

  // terminate after a maximum number of samples
  // if (gls->getStateCount() >= planning_context_->getMaximumGoalSamples())
  unsigned int max_goal_samples = 50;
  if (gls->getStateCount() >= max_goal_samples)
    return false;

  // terminate the sampling thread when a solution has been found
  if (planning_context_->getOMPLProblemDefinition()->hasSolution())
    return false;

  unsigned int max_attempts_div2 = max_attempts / 2;
  for (unsigned int a = gls->samplingAttemptsCount(); a < max_attempts && gls->isSampling(); ++a)
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
          boost::bind(&ompl_interface::ConstrainedGoalRegionSampler::stateValidityCallback, this, new_goal,
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
          if (checkStateValidity(new_goal, work_state_, verbose))
            return true;
        }
        else
        {
          invalid_sampled_constraints_++;
          if (!warned_invalid_samples_ && invalid_sampled_constraints_ >= (attempts_so_far * 8) / 10)
          {
            warned_invalid_samples_ = true;
            logWarn("More than 80%% of the sampled goal states fail to satisfy "
                    "the constraints imposed on the goal "
                    "sampler. Is the constrained sampler working correctly?");
          }
        }
      }
    }
    else
    {
      default_sampler_->sampleUniform(new_goal);
      if (dynamic_cast<const StateValidityChecker*>(si_->getStateValidityChecker().get())->isValid(new_goal, verbose))
      {
        planning_context_->copyToRobotState(work_state_, new_goal);
        if (kinematic_constraint_set_->decide(work_state_, verbose).satisfied)
          return true;
      }
    }
  }
  return false;
}
