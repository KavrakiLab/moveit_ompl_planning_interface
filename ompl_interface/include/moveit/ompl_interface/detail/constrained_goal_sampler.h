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

#ifndef MOVEIT_OMPL_INTERFACE_DETAIL_CONSTRAINED_GOAL_SAMPLER_
#define MOVEIT_OMPL_INTERFACE_DETAIL_CONSTRAINED_GOAL_SAMPLER_

#include <moveit/constraint_samplers/constraint_sampler.h>
#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <ompl/base/goals/GoalLazySamples.h>
#include <ompl/base/spaces/SE3StateSpace.h>

#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/GoalRegion.h>

namespace ompl_interface
{
class OMPLPlanningContext;

/** @class ConstrainedGoalSampler
 *  An interface to the OMPL goal lazy sampler*/
class ConstrainedGoalSampler : public ompl::base::GoalLazySamples
{
public:
  ConstrainedGoalSampler(
      const OMPLPlanningContext* pc, const kinematic_constraints::KinematicConstraintSetPtr& ks,
      const constraint_samplers::ConstraintSamplerPtr& cs = constraint_samplers::ConstraintSamplerPtr());

private:
  bool sampleUsingConstraintSampler(const ompl::base::GoalLazySamples* gls, ompl::base::State* new_goal);
  bool stateValidityCallback(ompl::base::State* new_goal, robot_state::RobotState const* state,
                             const robot_model::JointModelGroup*, const double*, bool verbose = false) const;
  bool checkStateValidity(ompl::base::State* new_goal, const robot_state::RobotState& state,
                          bool verbose = false) const;

  const OMPLPlanningContext* planning_context_;
  kinematic_constraints::KinematicConstraintSetPtr kinematic_constraint_set_;
  constraint_samplers::ConstraintSamplerPtr constraint_sampler_;
  ompl::base::StateSamplerPtr default_sampler_;
  robot_state::RobotState work_state_;
  unsigned int invalid_sampled_constraints_;
  bool warned_invalid_samples_;
  unsigned int verbose_display_;
};

/** @class ConstrainedGoalRegionSampler
 *  An interface to the OMPL goal lazy sampler*/
class ConstrainedGoalRegionSampler : public ompl::base::GoalLazySamples
{
public:
  ConstrainedGoalRegionSampler(
      const OMPLPlanningContext* pc, const robot_model::RobotModelConstPtr& rm,
      const planning_scene::PlanningSceneConstPtr& ps, moveit_msgs::Constraints& constr,
      const kinematic_constraints::KinematicConstraintSetPtr& ks, const moveit_msgs::GoalRegion& gr,
      const constraint_samplers::ConstraintSamplerPtr& cs = constraint_samplers::ConstraintSamplerPtr());

private:
  bool sampleUsingConstraintSampler(const ompl::base::GoalLazySamples* gls, ompl::base::State* new_goal);
  bool stateValidityCallback(ompl::base::State* new_goal, robot_state::RobotState const* state,
                             const robot_model::JointModelGroup*, const double*, bool verbose = false) const;
  bool checkStateValidity(ompl::base::State* new_goal, const robot_state::RobotState& state,
                          bool verbose = false) const;

  const OMPLPlanningContext* planning_context_;
  kinematic_constraints::KinematicConstraintSetPtr kinematic_constraint_set_;
  constraint_samplers::ConstraintSamplerPtr constraint_sampler_;
  ompl::base::StateSamplerPtr default_sampler_;
  robot_state::RobotState work_state_;
  unsigned int invalid_sampled_constraints_;
  bool warned_invalid_samples_;
  unsigned int verbose_display_;
  moveit_msgs::GoalRegion goal_region_;

  ompl::base::StateSpacePtr space_;
  ompl::base::StateSamplerPtr se3_sampler_;
  planning_scene::PlanningSceneConstPtr planning_scene_;
  moveit_msgs::Constraints constr_;
};
}

#endif
