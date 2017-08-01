/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Rice University
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
*   * Neither the name of the Rice University nor the names of its
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

/* Author: Ryan Luna */

#include "moveit/ompl_interface/geometric_planning_context.h"
#include "moveit/ompl_interface/detail/constrained_goal_sampler.h"
#include "moveit/ompl_interface/detail/constrained_sampler.h"
#include "moveit/ompl_interface/detail/goal_union.h"
#include "moveit/ompl_interface/detail/projection_evaluators.h"
#include "moveit/ompl_interface/detail/state_validity_checker.h"

#include <boost/math/constants/constants.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/kinematic_constraints/utils.h>
#include <pluginlib/class_loader.h>

#include <ompl/tools/config/SelfConfig.h>
#include <ompl/tools/multiplan/ParallelPlan.h>

#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/rrt/pRRT.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/sbl/pSBL.h>

namespace og = ompl::geometric;

using namespace ompl_interface;

GeometricPlanningContext::GeometricPlanningContext() : OMPLPlanningContext()
{
  initializePlannerAllocators();
  complete_initial_robot_state_ = nullptr;

  // Interpolate the final solution path to have a minimum number of states
  interpolate_ = true;

  // Attempt to smooth the final solution path
  simplify_ = true;

  // This context is not initialized
  initialized_ = false;

  planner_id_ = "";
}

GeometricPlanningContext::~GeometricPlanningContext()
{
  if (complete_initial_robot_state_ != nullptr)
    delete complete_initial_robot_state_;
}

std::string GeometricPlanningContext::getDescription()
{
  return "OMPL Geometric Planning";
}

void GeometricPlanningContext::initializePlannerAllocators()
{
  registerPlannerAllocator("geometric::RRT", boost::bind(&allocatePlanner<og::RRT>, _1, _2, _3));
  registerPlannerAllocator("geometric::RRTConnect", boost::bind(&allocatePlanner<og::RRTConnect>, _1, _2, _3));
  registerPlannerAllocator("geometric::LazyRRT", boost::bind(&allocatePlanner<og::LazyRRT>, _1, _2, _3));
  registerPlannerAllocator("geometric::TRRT", boost::bind(&allocatePlanner<og::TRRT>, _1, _2, _3));
  registerPlannerAllocator("geometric::EST", boost::bind(&allocatePlanner<og::EST>, _1, _2, _3));
  registerPlannerAllocator("geometric::SBL", boost::bind(&allocatePlanner<og::SBL>, _1, _2, _3));
  registerPlannerAllocator("geometric::KPIECE", boost::bind(&allocatePlanner<og::KPIECE1>, _1, _2, _3));
  registerPlannerAllocator("geometric::BKPIECE", boost::bind(&allocatePlanner<og::BKPIECE1>, _1, _2, _3));
  registerPlannerAllocator("geometric::LBKPIECE", boost::bind(&allocatePlanner<og::LBKPIECE1>, _1, _2, _3));
  registerPlannerAllocator("geometric::RRTstar", boost::bind(&allocatePlanner<og::RRTstar>, _1, _2, _3));
  registerPlannerAllocator("geometric::PRM", boost::bind(&allocatePlanner<og::PRM>, _1, _2, _3));
  registerPlannerAllocator("geometric::PRMstar", boost::bind(&allocatePlanner<og::PRMstar>, _1, _2, _3));
}

void GeometricPlanningContext::registerPlannerAllocator(const std::string& planner_id, const PlannerAllocator& pa)
{
  planner_allocators_[planner_id] = pa;
}

void GeometricPlanningContext::initialize(const std::string& ros_namespace, const PlanningContextSpecification& spec)
{
  spec_ = spec;

  interpolate_ = spec.interpolate_solution;
  simplify_ = spec.simplify_solution;

  // Erase the type and plugin fields from the configuration items
  auto it = spec_.config.find("type");
  if (it != spec_.config.end())
  {
    planner_id_ = it->second;
    spec_.config.erase(it);
  }
  else
    ROS_WARN("No planner type specified.  Using default planner configuration");

  it = spec_.config.find("plugin");
  if (it != spec_.config.end())
    spec_.config.erase(it);

  OMPLPlanningContext::initialize(ros_namespace, spec_);

  constraint_sampler_manager_ = spec_.constraint_sampler_mgr;
  if (!complete_initial_robot_state_)
    complete_initial_robot_state_ = new robot_state::RobotState(spec_.model);

  ROS_INFO("Initializing GeometricPlanningContext for '%s'", spec_.planner.c_str());

  // Initialize path constraints, if any

  const bool havePosCnst = !request_.path_constraints.position_constraints.empty();
  const bool haveOrnCnst = !request_.path_constraints.orientation_constraints.empty();
  const bool haveJntCnst = !request_.path_constraints.joint_constraints.empty();
  const bool haveVisCnst = !request_.path_constraints.visibility_constraints.empty();
  if (havePosCnst || haveOrnCnst || haveJntCnst || haveVisCnst)
  {
    path_constraints_.reset(new kinematic_constraints::KinematicConstraintSet(getRobotModel()));
    path_constraints_->add(request_.path_constraints, getPlanningScene()->getTransforms());
  }
  else
    path_constraints_.reset();

  // OMPL StateSpace
  ModelBasedStateSpaceSpecification state_space_spec(spec_.model, spec_.group);
  allocateStateSpace(state_space_spec);

  // OMPL SimpleSetup
  simple_setup_.reset(new ompl::geometric::SimpleSetup(mbss_));

  // OMPL ProjectionEvaluator
  it = spec_.config.find("projection_evaluator");
  if (it != spec_.config.end())
  {
    setProjectionEvaluator(boost::trim_copy(it->second));
    spec_.config.erase(it);
  }
  else if (planner_id_ != "")
    ROS_WARN("No projection evaluator for '%s'", planner_id_.c_str());

  // OMPL Planner
  if (planner_id_ != "")
  {
    ompl::base::PlannerPtr planner = configurePlanner(planner_id_, spec_.config);
    simple_setup_->setPlanner(planner);
  }

  // OMPL StateSampler
  mbss_->setStateSamplerAllocator(boost::bind(&GeometricPlanningContext::allocPathConstrainedSampler, this, _1));

  initialized_ = true;
}

void GeometricPlanningContext::allocateStateSpace(const ModelBasedStateSpaceSpecification& state_space_spec)
{
  bool allocated = false;

  // If there are (only) position and/or orientation constraints, make sure we
  // have a means to
  // compute IK solutions.  If so, allocate a pose model (workspace) state space
  // representation
  const bool havePositionConstraints = !request_.path_constraints.position_constraints.empty();
  const bool haveOrientationConstraints = !request_.path_constraints.orientation_constraints.empty();
  const bool haveJointConstraints = !request_.path_constraints.joint_constraints.empty();
  const bool haveVisibilityConstraints = !request_.path_constraints.visibility_constraints.empty();

  // The default is a representation based on the joint angles of the group
  ModelBasedStateSpacePtr state_space_(new ModelBasedStateSpace(state_space_spec));
  mbss_ = state_space_;
}

ompl::base::StateSamplerPtr
GeometricPlanningContext::allocPathConstrainedSampler(const ompl::base::StateSpace* ss) const
{
  if (mbss_.get() != ss)
  {
    ROS_ERROR("%s: Attempted to allocate a state sampler for an unknown state space", name_.c_str());
    return ompl::base::StateSamplerPtr();
  }

  ROS_DEBUG("%s: Allocating a new state sampler (attempts to use path constraints)", name_.c_str());

  if (path_constraints_)
  {
    constraint_samplers::ConstraintSamplerPtr cs = constraint_sampler_manager_->selectSampler(
        getPlanningScene(), getGroupName(), path_constraints_->getAllConstraints());
    if (cs)
    {
      ROS_INFO("%s: Allocating specialized state sampler for state space", name_.c_str());
      return ompl::base::StateSamplerPtr(new ConstrainedSampler(this, cs));
    }
  }
  ROS_DEBUG("%s: Allocating default state sampler for state space", name_.c_str());
  return ss->allocDefaultStateSampler();
}

void GeometricPlanningContext::clear()
{
  simple_setup_->clear();
  simple_setup_->clearStartStates();
  simple_setup_->setGoal(ompl::base::GoalPtr());
  simple_setup_->setStateValidityChecker(ompl::base::StateValidityCheckerPtr());
  goal_constraints_.clear();
}

void GeometricPlanningContext::preSolve()
{
  simple_setup_->getProblemDefinition()->clearSolutionPaths();
  const ompl::base::PlannerPtr planner = simple_setup_->getPlanner();
  if (planner)
    planner->clear();
  startGoalSampling();
  simple_setup_->getSpaceInformation()->getMotionValidator()->resetMotionCounter();
}

void GeometricPlanningContext::postSolve()
{
  stopGoalSampling();
  if (simple_setup_->getProblemDefinition()->hasApproximateSolution())
    ROS_WARN("Solution is approximate");
}

void GeometricPlanningContext::startGoalSampling()
{
  bool gls = simple_setup_->getGoal()->hasType(ompl::base::GOAL_LAZY_SAMPLES);
  if (gls)
    dynamic_cast<ompl::base::GoalLazySamples*>(simple_setup_->getGoal().get())->startSampling();
  else
    // we know this is a GoalSampleableMux by elimination
    dynamic_cast<GoalSampleableRegionMux*>(simple_setup_->getGoal().get())->startSampling();
}

void GeometricPlanningContext::stopGoalSampling()
{
  bool gls = simple_setup_->getGoal()->hasType(ompl::base::GOAL_LAZY_SAMPLES);
  if (gls)
    dynamic_cast<ompl::base::GoalLazySamples*>(simple_setup_->getGoal().get())->stopSampling();
  else
    // we know this is a GoalSampleableMux by elimination
    dynamic_cast<GoalSampleableRegionMux*>(simple_setup_->getGoal().get())->stopSampling();
}

bool GeometricPlanningContext::solve(planning_interface::MotionPlanResponse& res)
{
  if (!initialized_)
  {
    ROS_ERROR("%s: Cannot solve motion plan query.  Planning context is not "
              "initialized",
              getDescription().c_str());
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN;
    return false;
  }

  double timeout = request_.allowed_planning_time;
  double plan_time = 0.0;
  bool result = solve(timeout, request_.num_planning_attempts, plan_time);

  if (result)
  {
    // Simplifying solution
    if (simplify_)
    {
      plan_time += simplifySolution(timeout);
      if ((timeout - plan_time) > 0)
      {
        double lasttime;
        do
        {
          lasttime = plan_time;
          plan_time += simplifySolution(timeout - plan_time);
        } while ((timeout - plan_time) > 0 && plan_time - lasttime > 1e-3);
      }
    }

    ompl::geometric::PathGeometric& pg = simple_setup_->getSolutionPath();
    // Interpolating the solution
    if (interpolate_)
    {
      // The maximum length of a single segment in the solution path
      double max_segment_length =
          (spec_.max_waypoint_distance > 0.0 ? spec_.max_waypoint_distance :
                                               simple_setup_->getStateSpace()->getMaximumExtent() / 100.0);
      // Computing the total number of waypoints we want in the solution path
      unsigned int waypoint_count =
          std::max((unsigned int)floor(0.5 + pg.length() / max_segment_length), spec_.min_waypoint_count);
      interpolateSolution(pg, waypoint_count);
    }

    ROS_DEBUG("%s: Returning successful solution with %lu states", getName().c_str(), pg.getStateCount());

    res.trajectory_.reset(new robot_trajectory::RobotTrajectory(getRobotModel(), getGroupName()));

    robot_state::RobotState ks = *complete_initial_robot_state_;
    for (std::size_t i = 0; i < pg.getStateCount(); ++i)
    {
      copyToRobotState(ks, pg.getState(i));
      res.trajectory_->addSuffixWayPoint(ks, 0.0);
    }

    res.planning_time_ = plan_time;
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  }
  else
  {
    ROS_WARN("%s: Unable to solve the planning problem", getName().c_str());
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
  }

  return result;
}

bool GeometricPlanningContext::solve(planning_interface::MotionPlanDetailedResponse& res)
{
  double timeout = request_.allowed_planning_time;
  double plan_time = 0.0;
  bool result = solve(timeout, request_.num_planning_attempts, plan_time);

  if (result)
  {
    // Getting the raw solution
    ompl::geometric::PathGeometric& pg = simple_setup_->getSolutionPath();
    res.processing_time_.push_back(plan_time);
    res.description_.emplace_back("plan");

    res.trajectory_.resize(res.trajectory_.size() + 1);
    res.trajectory_.back().reset(new robot_trajectory::RobotTrajectory(getRobotModel(), getGroupName()));

    robot_state::RobotState ks = *complete_initial_robot_state_;
    for (std::size_t i = 0; i < pg.getStateCount(); ++i)
    {
      copyToRobotState(ks, pg.getState(i));
      res.trajectory_.back()->addSuffixWayPoint(ks, 0.0);
    }

    if (simplify_)
    {
        double simplify_time = plan_time;

        plan_time += simplifySolution(timeout);
        if ((timeout - plan_time) > 0)
        {
            double lasttime;
            do
            {
                lasttime = plan_time;
                plan_time += simplifySolution(timeout - plan_time);
            } while ((timeout - plan_time) > 0 && plan_time - lasttime > 1e-3);
        }

        res.processing_time_.push_back(plan_time - simplify_time);
        res.description_.emplace_back("simplify");

        pg = simple_setup_->getSolutionPath();
        res.trajectory_.resize(res.trajectory_.size() + 1);
        res.trajectory_.back().reset(new robot_trajectory::RobotTrajectory(getRobotModel(), getGroupName()));

        for (std::size_t i = 0; i < pg.getStateCount(); ++i)
        {
            copyToRobotState(ks, pg.getState(i));
            res.trajectory_.back()->addSuffixWayPoint(ks, 0.0);
        }

    }

    // Interpolating the final solution
    if (interpolate_)
    {
      pg = simple_setup_->getSolutionPath();
      // The maximum length of a single segment in the solution path
      double max_segment_length =
          (spec_.max_waypoint_distance > 0.0 ? spec_.max_waypoint_distance :
                                               simple_setup_->getStateSpace()->getMaximumExtent() / 100.0);
      // Computing the total number of waypoints we want in the solution path
      unsigned int waypoint_count =
          std::max((unsigned int)floor(0.5 + pg.length() / max_segment_length), spec_.min_waypoint_count);
      double interpolate_time = interpolateSolution(pg, waypoint_count);

      res.processing_time_.push_back(interpolate_time);
      res.description_.emplace_back("interpolate");

      ROS_DEBUG("%s: Returning successful solution with %lu states", getName().c_str(), pg.getStateCount());

      res.trajectory_.resize(res.trajectory_.size() + 1);
      res.trajectory_.back().reset(new robot_trajectory::RobotTrajectory(getRobotModel(), getGroupName()));

      for (std::size_t i = 0; i < pg.getStateCount(); ++i)
      {
        copyToRobotState(ks, pg.getState(i));
        res.trajectory_.back()->addSuffixWayPoint(ks, 0.0);
      }
    }

    res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  }
  else
  {
    ROS_INFO("%s: Unable to solve the planning problem", getName().c_str());
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
  }

  return result;
}

bool GeometricPlanningContext::solve(double timeout, unsigned int count, double& total_time)
{
  ompl::time::point start = ompl::time::now();

  preSolve();

  bool result = false;
  total_time = 0.0;
  if (count <= 1)
  {
    ompl::base::PlannerTerminationCondition ptc =
        ompl::base::timedPlannerTerminationCondition(timeout - ompl::time::seconds(ompl::time::now() - start));
    registerTerminationCondition(ptc);
    result = simple_setup_->solve(ptc) == ompl::base::PlannerStatus::EXACT_SOLUTION;
    total_time = simple_setup_->getLastPlanComputationTime();
    unregisterTerminationCondition();
  }
  else  // attempt to solve in parallel
  {
    ROS_DEBUG("Solving problem in parallel with up to %u threads", spec_.max_num_threads);
    ompl::tools::ParallelPlan pp(simple_setup_->getProblemDefinition());
    if (count <= spec_.max_num_threads)  // fewer attempts than threads
    {
      if (planner_id_.size())  // There is a planner configured
      {
        for (unsigned int i = 0; i < count; ++i)
          pp.addPlanner(configurePlanner(planner_id_, spec_.config));
      }
      else
      {
        for (unsigned int i = 0; i < count; ++i)
          pp.addPlanner(ompl::tools::SelfConfig::getDefaultPlanner(simple_setup_->getGoal()));
      }

      ompl::base::PlannerTerminationCondition ptc =
          ompl::base::timedPlannerTerminationCondition(timeout - ompl::time::seconds(ompl::time::now() - start));
      registerTerminationCondition(ptc);
      // Solve in parallel.  Hybridize the solution paths.
      result = pp.solve(ptc, count, count, true) == ompl::base::PlannerStatus::EXACT_SOLUTION;
      total_time = ompl::time::seconds(ompl::time::now() - start);
      unregisterTerminationCondition();
    }
    else  // more attempts than threads
    {
      ompl::base::PlannerTerminationCondition ptc =
          ompl::base::timedPlannerTerminationCondition(timeout - ompl::time::seconds(ompl::time::now() - start));
      registerTerminationCondition(ptc);
      int n = count / spec_.max_num_threads;
      result = true;
      for (int i = 0; i < n && !ptc(); ++i)
      {
        pp.clearPlanners();
        if (planner_id_.size())  // There is a planner configured
        {
          for (unsigned int i = 0; i < spec_.max_num_threads; ++i)
            pp.addPlanner(configurePlanner(planner_id_, spec_.config));
        }
        else
        {
          for (unsigned int i = 0; i < spec_.max_num_threads; ++i)
            pp.addPlanner(ompl::tools::SelfConfig::getDefaultPlanner(simple_setup_->getGoal()));
        }

        result &= pp.solve(ptc, count, count, true) == ompl::base::PlannerStatus::EXACT_SOLUTION;
      }

      // Do the remainder
      n = count % spec_.max_num_threads;
      if (n && !ptc())
      {
        pp.clearPlanners();
        if (planner_id_.size())  // There is a planner configured
        {
          for (unsigned int i = 0; i < spec_.max_num_threads; ++i)
            pp.addPlanner(configurePlanner(planner_id_, spec_.config));
        }
        else
        {
          for (unsigned int i = 0; i < spec_.max_num_threads; ++i)
            pp.addPlanner(ompl::tools::SelfConfig::getDefaultPlanner(simple_setup_->getGoal()));
        }

        result &= pp.solve(ptc, count, count, true) == ompl::base::PlannerStatus::EXACT_SOLUTION;
      }
      total_time = ompl::time::seconds(ompl::time::now() - start);
      unregisterTerminationCondition();
    }
  }

  postSolve();

  return result;
}

double GeometricPlanningContext::simplifySolution(double max_time)
{
  simple_setup_->simplifySolution(max_time);
  return simple_setup_->getLastSimplificationTime();
}

double GeometricPlanningContext::interpolateSolution(ompl::geometric::PathGeometric& path, unsigned int waypoint_count)
{
  ompl::time::point start = ompl::time::now();
  path.interpolate(waypoint_count);

  return ompl::time::seconds(ompl::time::now() - start);
}

void GeometricPlanningContext::registerTerminationCondition(const ompl::base::PlannerTerminationCondition& ptc)
{
  boost::mutex::scoped_lock slock(ptc_lock_);
  ptc_ = &ptc;
}

void GeometricPlanningContext::unregisterTerminationCondition()
{
  boost::mutex::scoped_lock slock(ptc_lock_);
  ptc_ = nullptr;
}

bool GeometricPlanningContext::terminate()
{
  boost::mutex::scoped_lock slock(ptc_lock_);
  if (ptc_)
    ptc_->terminate();
  return true;
}

const ompl::base::StateSpacePtr& GeometricPlanningContext::getOMPLStateSpace() const
{
  return mbss_;
}

ModelBasedStateSpace *GeometricPlanningContext::getModelBasedStateSpace()
{
    return mbss_->as<ModelBasedStateSpace>();
}

const ModelBasedStateSpace *GeometricPlanningContext::getModelBasedStateSpace() const
{
    return mbss_->as<ModelBasedStateSpace>();
}

const ompl::base::SpaceInformationPtr& GeometricPlanningContext::getOMPLSpaceInformation() const
{
  return simple_setup_->getSpaceInformation();
}

const ompl::base::ProblemDefinitionPtr& GeometricPlanningContext::getOMPLProblemDefinition() const
{
  return simple_setup_->getProblemDefinition();
}

const robot_state::RobotState& GeometricPlanningContext::getCompleteInitialRobotState() const
{
  return *complete_initial_robot_state_;
}

void GeometricPlanningContext::setCompleteInitialRobotState(const robot_state::RobotState& state)
{
  if (!complete_initial_robot_state_)
  {
    ROS_WARN("Planning context not initialized.  Not setting initial robot state");
    return;
  }

  *complete_initial_robot_state_ = state;

  // Start state
  ompl::base::ScopedState<> start_state(mbss_);
  copyToOMPLState(start_state.get(), *complete_initial_robot_state_);
  simple_setup_->setStartState(start_state);

  // State validity checker
  simple_setup_->setStateValidityChecker(ompl::base::StateValidityCheckerPtr(new StateValidityChecker(this)));
}

bool GeometricPlanningContext::setGoalConstraints(const std::vector<moveit_msgs::Constraints>& goal_constraints,
                                                  moveit_msgs::MoveItErrorCodes* error)
{
  if (goal_constraints.empty())
  {
    ROS_WARN("No goal constraints specified.  There is no problem to solve.");
    if (error)
      error->val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
    return false;
  }

  // Merge path constraints (if any) with goal constraints
  goal_constraints_.clear();
  for (const auto& goal_constraint : goal_constraints)
  {
    // NOTE: This only "intelligently" merges joint constraints .  All other
    // constraint types are simply concatenated.
    // moveit_msgs::Constraints constr =
    // kinematic_constraints::mergeConstraints(goal_constraints[i],
    // request_.path_constraints);

    // This will merge the path constraints with goal_constraints[i]
    moveit_msgs::Constraints constr;
    if (!mergeConstraints(goal_constraint, request_.path_constraints, constr))
    {
      ROS_ERROR("Failed to merge path constraints with goal constraints.  "
                "Motion plan request is invalid.");
      if (error)
        error->val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
      return false;
    }

    kinematic_constraints::KinematicConstraintSetPtr kset(
        new kinematic_constraints::KinematicConstraintSet(getRobotModel()));
    kset->add(constr, getPlanningScene()->getTransforms());
    if (!kset->empty())
      goal_constraints_.push_back(kset);
  }

  if (goal_constraints_.empty())
  {
    ROS_WARN("No goal constraints specified. There is no problem to solve.");
    if (error)
      error->val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
    return false;
  }

  // Creating constraint sampler for each constraint
  std::vector<ompl::base::GoalPtr> goals;
  for (auto& goal_constraint : goal_constraints_)
  {
    constraint_samplers::ConstraintSamplerPtr cs;
    if (constraint_sampler_manager_)
      cs = constraint_sampler_manager_->selectSampler(getPlanningScene(), getGroupName(),
                                                      goal_constraint->getAllConstraints());
    if (cs)
    {
      ompl::base::GoalPtr g = ompl::base::GoalPtr(new ConstrainedGoalSampler(this, goal_constraint, cs));
      goals.push_back(g);
    }
    else
    {
      ROS_WARN("No constraint sampler available to sample goal constraints");
    }
  }

  // Creating goal object using constraint samplers
  if (!goals.empty())
  {
    ompl::base::GoalPtr goal;
    if (goals.size() == 1)
      goal = goals[0];
    else
      goal = ompl::base::GoalPtr(new GoalSampleableRegionMux(goals));

    simple_setup_->setGoal(goal);
    return true;
  }

  OMPL_ERROR("Unable to construct goal representation");
  return false;
}

const kinematic_constraints::KinematicConstraintSetPtr& GeometricPlanningContext::getPathConstraints() const
{
  return path_constraints_;
}

const robot_model::RobotModelConstPtr& GeometricPlanningContext::getRobotModel() const
{
  return spec_.model;
}

const robot_model::JointModelGroup* GeometricPlanningContext::getJointModelGroup() const
{
  return spec_.model->getJointModelGroup(spec_.group);
}

ompl::base::PlannerPtr GeometricPlanningContext::configurePlanner(const std::string& planner_name,
                                                                  const std::map<std::string, std::string>& params)
{
  std::map<std::string, PlannerAllocator>::const_iterator it = planner_allocators_.find(planner_name);
  // Allocating planner using planner allocator
  if (it != planner_allocators_.end())
    return it->second(simple_setup_->getSpaceInformation(), spec_.name, params);

  // No planner configured by this name
  ROS_WARN("No planner allocator found with name '%s'", planner_name.c_str());
  for (auto it = planner_allocators_.begin(); it != planner_allocators_.end(); ++it)
    ROS_WARN("  %s", it->first.c_str());
  return ompl::base::PlannerPtr();
}

void GeometricPlanningContext::setProjectionEvaluator(const std::string& peval)
{
  if (!mbss_)
  {
    ROS_ERROR("No state space is configured yet");
    return;
  }
  ompl::base::ProjectionEvaluatorPtr pe = getProjectionEvaluator(peval);
  if (pe)
    mbss_->registerDefaultProjection(pe);
}

ompl::base::ProjectionEvaluatorPtr GeometricPlanningContext::getProjectionEvaluator(const std::string& peval) const
{
  if (peval.find_first_of("link(") == 0 && peval[peval.length() - 1] == ')')
  {
    std::string link_name = peval.substr(5, peval.length() - 6);
    if (getRobotModel()->hasLinkModel(link_name))
      return ompl::base::ProjectionEvaluatorPtr(new ProjectionEvaluatorLinkPose(this, link_name));
    else
      ROS_ERROR("Attempted to set projection evaluator with respect to "
                "position of link '%s', but that link is not "
                "known to the kinematic model.",
                link_name.c_str());
  }
  else if (peval.find_first_of("joints(") == 0 && peval[peval.length() - 1] == ')')
  {
    std::string joints = peval.substr(7, peval.length() - 8);
    boost::replace_all(joints, ",", " ");
    std::vector<unsigned int> j;
    std::stringstream ss(joints);
    while (ss.good() && !ss.eof())
    {
      std::string v;
      ss >> v >> std::ws;
      if (getJointModelGroup()->hasJointModel(v))
      {
        unsigned int vc = getJointModelGroup()->getJointModel(v)->getVariableCount();
        if (vc > 0)
        {
          int idx = getJointModelGroup()->getVariableGroupIndex(v);
          for (int q = 0; q < vc; ++q)
            j.push_back(idx + q);
        }
        else
          ROS_WARN("%s: Ignoring joint '%s' in projection since it has 0 DOF", name_.c_str(), v.c_str());
      }
      else
        ROS_ERROR("%s: Attempted to set projection evaluator with respect to "
                  "value of joint '%s', but that joint is "
                  "not known to the group '%s'.",
                  name_.c_str(), v.c_str(), getGroupName().c_str());
    }
    if (j.empty())
      ROS_ERROR("%s: No valid joints specified for joint projection", name_.c_str());
    else
      return ompl::base::ProjectionEvaluatorPtr(new ProjectionEvaluatorJointValue(this, j));
  }
  else
    ROS_ERROR("Unable to allocate projection evaluator based on description: '%s'", peval.c_str());
  return ompl::base::ProjectionEvaluatorPtr();
}

// Merge c2 with c1, if useful
bool GeometricPlanningContext::mergeConstraints(const moveit_msgs::Constraints& c1, const moveit_msgs::Constraints& c2,
                                                moveit_msgs::Constraints& output) const
{
  // Merge orientation constraints in c2 that have a common link with c1
  for (size_t i = 0; i < c1.orientation_constraints.size(); ++i)
  {
    bool merged = false;
    for (size_t j = 0; j < c2.orientation_constraints.size(); ++j)
    {
      // Common link
      if (c1.orientation_constraints[i].link_name == c2.orientation_constraints[j].link_name &&
          c1.orientation_constraints[i].header.frame_id ==
              c2.orientation_constraints[j].header.frame_id)  // TODO: Frame ids need not be the same.
      {
        // Check that c1 pose is compatible with c2 constraint
        Eigen::Quaterniond q1, q2;
        tf::quaternionMsgToEigen(c1.orientation_constraints[i].orientation, q1);
        tf::quaternionMsgToEigen(c2.orientation_constraints[j].orientation, q2);

        Eigen::Vector3d rpy1 = Eigen::Affine3d(q1).rotation().eulerAngles(0, 1, 2);
        Eigen::Vector3d rpy2 = Eigen::Affine3d(q2).rotation().eulerAngles(0, 1, 2);

        // Rotational difference, absolute value mod pi
        Eigen::Vector3d diff(rpy2 - rpy1);
        diff(0) = std::min(fabs(diff(0)), boost::math::constants::pi<double>() - fabs(diff(0)));
        diff(1) = std::min(fabs(diff(1)), boost::math::constants::pi<double>() - fabs(diff(1)));
        diff(2) = std::min(fabs(diff(2)), boost::math::constants::pi<double>() - fabs(diff(2)));

        // Diff will satisfy c1 by construction.  If diff also satisfies c2,
        // then we can merge the constraints
        if (diff(0) < c2.orientation_constraints[j].absolute_x_axis_tolerance &&
            diff(1) < c2.orientation_constraints[j].absolute_y_axis_tolerance &&
            diff(2) < c2.orientation_constraints[j].absolute_z_axis_tolerance)
        {
          merged = true;
          ROS_INFO("GeometricPlanningContext: Merging orientation constraints "
                   "for %s",
                   c1.orientation_constraints[i].link_name.c_str());

          moveit_msgs::OrientationConstraint or_constraint;
          or_constraint.header = c1.orientation_constraints[i].header;
          or_constraint.orientation = c1.orientation_constraints[i].orientation;
          or_constraint.link_name = c1.orientation_constraints[i].link_name;
          or_constraint.absolute_x_axis_tolerance = std::min(c1.orientation_constraints[i].absolute_x_axis_tolerance,
                                                             c2.orientation_constraints[j].absolute_x_axis_tolerance);
          or_constraint.absolute_y_axis_tolerance = std::min(c1.orientation_constraints[i].absolute_y_axis_tolerance,
                                                             c2.orientation_constraints[j].absolute_y_axis_tolerance);
          or_constraint.absolute_z_axis_tolerance = std::min(c1.orientation_constraints[i].absolute_z_axis_tolerance,
                                                             c2.orientation_constraints[j].absolute_z_axis_tolerance);
          or_constraint.weight = std::max(c1.orientation_constraints[i].weight, c2.orientation_constraints[i].weight);
          output.orientation_constraints.push_back(or_constraint);
        }
        else
        {
          ROS_ERROR("Failed to merge orientation constraints for %s", c1.orientation_constraints[i].link_name.c_str());
          ROS_ERROR("[x] %f <? %f: %s", diff(0), c2.orientation_constraints[j].absolute_x_axis_tolerance,
                    diff(0) < c2.orientation_constraints[j].absolute_x_axis_tolerance ? "TRUE" : "FALSE");
          ROS_ERROR("[y] %f <? %f: %s", diff(1), c2.orientation_constraints[j].absolute_y_axis_tolerance,
                    diff(1) < c2.orientation_constraints[j].absolute_y_axis_tolerance ? "TRUE" : "FALSE");
          ROS_ERROR("[z] %f <? %f: %s", diff(2), c2.orientation_constraints[j].absolute_z_axis_tolerance,
                    diff(2) < c2.orientation_constraints[j].absolute_z_axis_tolerance ? "TRUE" : "FALSE");
          return false;
        }
      }
    }

    if (!merged)
      output.orientation_constraints.push_back(c1.orientation_constraints[i]);
  }

  // add all orientation constraints that are in c2 but not in c1
  for (const auto& orientation_constraint : c2.orientation_constraints)
  {
    bool add = true;
    for (const auto& j : c1.orientation_constraints)
      if (orientation_constraint.link_name == j.link_name)
      {
        add = false;
        break;
      }
    if (add)
      output.orientation_constraints.push_back(orientation_constraint);
  }

  // Joint constraints.  Totally pilfered from
  // kinematic_constraints/src/utils.cpp
  // add all joint constraints that are in c1 but not in c2
  // and merge joint constraints that are for the same joint
  for (const auto& joint_constraint : c1.joint_constraints)
  {
    bool add = true;
    for (const auto& j : c2.joint_constraints)
      if (j.joint_name == joint_constraint.joint_name)
      {
        add = false;
        // now we merge
        moveit_msgs::JointConstraint m;
        const moveit_msgs::JointConstraint& a = joint_constraint;
        const moveit_msgs::JointConstraint& b = j;
        double low = std::max(a.position - a.tolerance_below, b.position - b.tolerance_below);
        double high = std::min(a.position + a.tolerance_above, b.position + b.tolerance_above);
        if (low > high)
          ROS_ERROR("Attempted to merge incompatible constraints for joint "
                    "'%s'. Discarding constraint.",
                    a.joint_name.c_str());
        else
        {
          m.joint_name = a.joint_name;
          m.position =
              std::max(low, std::min((a.position * a.weight + b.position * b.weight) / (a.weight + b.weight), high));
          m.weight = (a.weight + b.weight) / 2.0;
          m.tolerance_above = std::max(0.0, high - m.position);
          m.tolerance_below = std::max(0.0, m.position - low);
          output.joint_constraints.push_back(m);
        }
        break;
      }
    if (add)
      output.joint_constraints.push_back(joint_constraint);
  }

  // add all joint constraints that are in c2 but not in c1
  for (const auto& joint_constraint : c2.joint_constraints)
  {
    bool add = true;
    for (const auto& j : c1.joint_constraints)
      if (joint_constraint.joint_name == j.joint_name)
      {
        add = false;
        break;
      }
    if (add)
      output.joint_constraints.push_back(joint_constraint);
  }

  // Concatenate other constraints
  output.position_constraints = c1.position_constraints;
  for (const auto& position_constraint : c2.position_constraints)
    output.position_constraints.push_back(position_constraint);

  output.visibility_constraints = c1.visibility_constraints;
  for (const auto& visibility_constraint : c2.visibility_constraints)
    output.visibility_constraints.push_back(visibility_constraint);

  return true;
}

CLASS_LOADER_REGISTER_CLASS(ompl_interface::GeometricPlanningContext, ompl_interface::OMPLPlanningContext);
