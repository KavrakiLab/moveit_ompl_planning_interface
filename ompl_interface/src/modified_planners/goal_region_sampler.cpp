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

// MoveIt!
#include <moveit/ompl_interface/modified_planners/goal_region_sampler.h>
#include <moveit/ompl_interface/detail/state_validity_checker.h>
#include <moveit/ompl_interface/ompl_planning_context.h>
#include <moveit/profiler/profiler.h>

#include <tf/tf.h>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/pending/disjoint_sets.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/property_map/vector_property_map.hpp>
#include <boost/foreach.hpp>

ompl_interface::GoalRegionSampler::GoalRegionSampler(
    const OMPLPlanningContext* pc, const std::string& group_name, const robot_model::RobotModelConstPtr& rm,
    const planning_scene::PlanningSceneConstPtr& ps, const std::vector<moveit_msgs::Constraints>& constrs,
    const std::vector<moveit_msgs::WorkspaceGoalRegion>& wsgrs, const std::string& sort_roadmap_func_str,
    constraint_samplers::ConstraintSamplerManagerPtr csm, const unsigned int max_sampled_goals)
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
  , sort_roadmap_func_str_(sort_roadmap_func_str)
  , robot_model_loader_("robot_description")
{
  // Kinematics robot information
  kinematic_model_ = robot_model_loader_.getModel();
  kinematic_state_ = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model_));
  kinematic_state_->setToDefaultValues();
  joint_model_group_ = kinematic_model_->getJointModelGroup(planning_context_->getGroupName());

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

  startSampling();

  if (!sort_roadmap_func_str_.empty())
  {
    OMPL_DEBUG("Creating PRM for Goal Regions");
    // construct the state space we are planning in
    ModelBasedStateSpacePtr prm_space(new ModelBasedStateSpace(
        planning_context_->getOMPLStateSpace()->as<ModelBasedStateSpace>()->getSpecification()));
    // construct an instance of  space information from this state space
    auto prm_si(std::make_shared<ompl::base::SpaceInformation>(prm_space));
    // set state validity checking for this space
    prm_si->setStateValidityChecker(
        ompl::base::StateValidityCheckerPtr(new ompl_interface::StateValidityChecker(planning_context_)));
    // create a problem instance
    auto prm_pdef(std::make_shared<ompl::base::ProblemDefinition>(prm_si));
    // create a planner for the defined space
    prm_planner_ = std::make_shared<ompl::geometric::PRMMod>(prm_si);
    // set the problem we are trying to solve for the planner
    prm_planner_->setProblemDefinition(prm_pdef);
    // perform setup steps for the planner
    prm_planner_->setup();
    // Set a goal regions state sampler
    prm_planner_->getSpaceInformation()->getStateSpace()->setStateSamplerAllocator(
        std::bind(ob::newAllocGoalRegionStateSampler, std::placeholders::_1, this));
    startGrowingRoadmap();
  }
}

std::string ompl_interface::GoalRegionSampler::getSortRoadmapFuncStr()
{
  return sort_roadmap_func_str_;
}

double ompl_interface::GoalRegionSampler::distanceGoal(const ompl::base::State* st) const
{
  // Solving FK
  std::vector<double> joint_values;
  for (unsigned int i = 0; i < si_->getStateDimension(); i++)
    joint_values.push_back(st->as<ompl::base::RealVectorStateSpace::StateType>()->values[i]);

  kinematic_state_->setJointGroupPositions(joint_model_group_, joint_values);
  const Eigen::Affine3d& ee_pose = kinematic_state_->getGlobalLinkTransform("gripper_link");

  // Distances to the goal regions
  for (std::size_t i = 0; i < workspace_goal_regions_.size(); ++i)
  {
    const Eigen::Isometry3d wsgr_tf = planning_scene_->getFrameTransform(workspace_goal_regions_[i].header.frame_id);
    Eigen::Matrix3d rot(wsgr_tf.rotation());
    Eigen::Vector3d wsgr_min(workspace_goal_regions_[i].x.min, workspace_goal_regions_[i].y.min,
                             workspace_goal_regions_[i].z.min);
    wsgr_min = rot * wsgr_min;
    Eigen::Vector3d wsgr_max(workspace_goal_regions_[i].x.max, workspace_goal_regions_[i].y.max,
                             workspace_goal_regions_[i].z.max);
    wsgr_max = rot * wsgr_max;

    double wsgr_min_x, wsgr_max_x, wsgr_min_y, wsgr_max_y, wsgr_min_z, wsgr_max_z;
    if (wsgr_min.x() < wsgr_max.x())
    {
      wsgr_min_x = wsgr_min.x();
      wsgr_max_x = wsgr_max.x();
    }
    else
    {
      wsgr_min_x = wsgr_max.x();
      wsgr_max_x = wsgr_min.x();
    }
    if (wsgr_min.y() < wsgr_max.y())
    {
      wsgr_min_y = wsgr_min.y();
      wsgr_max_y = wsgr_max.y();
    }
    else
    {
      wsgr_min_y = wsgr_max.y();
      wsgr_max_y = wsgr_min.y();
    }
    if (wsgr_min.z() < wsgr_max.z())
    {
      wsgr_min_z = wsgr_min.z();
      wsgr_max_z = wsgr_max.z();
    }
    else
    {
      wsgr_min_z = wsgr_max.z();
      wsgr_max_z = wsgr_min.z();
    }

    if ((abs(ee_pose.translation().x() - wsgr_tf.translation().x() - wsgr_min_x) < 0.002 ||
         ee_pose.translation().x() >= (wsgr_tf.translation().x() + wsgr_min_x)) &&
        (abs(ee_pose.translation().x() - wsgr_tf.translation().x() - wsgr_max_x) < 0.002 ||
         ee_pose.translation().x() <= (wsgr_tf.translation().x() + wsgr_max_x)) &&
        (abs(ee_pose.translation().y() - wsgr_tf.translation().y() - wsgr_min_y) < 0.002 ||
         ee_pose.translation().y() >= (wsgr_tf.translation().y() + wsgr_min_y)) &&
        (abs(ee_pose.translation().y() - wsgr_tf.translation().y() - wsgr_max_y) < 0.002 ||
         ee_pose.translation().y() <= (wsgr_tf.translation().y() + wsgr_max_y)) &&
        (abs(ee_pose.translation().z() - wsgr_tf.translation().z() - wsgr_min_z) < 0.002 ||
         ee_pose.translation().z() >= (wsgr_tf.translation().z() + wsgr_min_z)) &&
        (abs(ee_pose.translation().z() - wsgr_tf.translation().z() - wsgr_max_z) < 0.002 ||
         ee_pose.translation().z() <= (wsgr_tf.translation().z() + wsgr_max_z)))
    {
      if (workspace_goal_regions_[i].roll.free_value && workspace_goal_regions_[i].pitch.free_value &&
          workspace_goal_regions_[i].yaw.free_value)
      {
        return 0.0;
      }

      // orientation constraints
      tf::Quaternion quaternion_constraints(
          constrs_[i].orientation_constraints[0].orientation.x, constrs_[i].orientation_constraints[0].orientation.y,
          constrs_[i].orientation_constraints[0].orientation.z, constrs_[i].orientation_constraints[0].orientation.w);
      tf::Matrix3x3 roation_constraints(quaternion_constraints);
      double constr_roll, constr_pitch, constr_yaw;
      roation_constraints.getRPY(constr_roll, constr_pitch, constr_yaw);

      // pose orientation
      Eigen::Matrix3d pose_roation = ee_pose.rotation();
      Eigen::Quaterniond pose_quaternion(pose_roation);
      tf::Quaternion pose_quaternion_tf(pose_quaternion.x(), pose_quaternion.y(), pose_quaternion.z(),
                                        pose_quaternion.w());

      tf::Matrix3x3 pose_roation_(pose_quaternion_tf);
      double pose_roll, pose_pitch, pose_yaw;
      pose_roation_.getRPY(pose_roll, pose_pitch, pose_yaw);

      bool meet_orientation_const = true;
      if (!workspace_goal_regions_[i].roll.free_value && abs(constr_roll - pose_roll) > 0.02)
        meet_orientation_const = false;
      if (!workspace_goal_regions_[i].pitch.free_value && abs(constr_pitch - pose_pitch) > 0.02)
        meet_orientation_const = false;
      if (!workspace_goal_regions_[i].yaw.free_value && abs(constr_yaw - pose_yaw) > 0.02)
        meet_orientation_const = false;

      if (meet_orientation_const)
      {
        return 0.0;
      }
    }
  }

  return GoalStates::distanceGoal(st);
}

double ompl_interface::GoalRegionSampler::getTerminalCost(const ompl::base::State* st) const
{
  // Solving FK
  std::vector<double> joint_values;
  for (unsigned int i = 0; i < si_->getStateDimension(); i++)
    joint_values.push_back(st->as<ompl::base::RealVectorStateSpace::StateType>()->values[i]);

  kinematic_state_->setJointGroupPositions(joint_model_group_, joint_values);
  const Eigen::Affine3d& ee_pose = kinematic_state_->getGlobalLinkTransform("gripper_link");

  // Distances to the goal regions
  for (std::size_t i = 0; i < workspace_goal_regions_.size(); ++i)
  {
    const Eigen::Isometry3d wsgr_tf = planning_scene_->getFrameTransform(workspace_goal_regions_[i].header.frame_id);
    Eigen::Matrix3d rot(wsgr_tf.rotation());
    Eigen::Vector3d wsgr_min(workspace_goal_regions_[i].x.min, workspace_goal_regions_[i].y.min,
                             workspace_goal_regions_[i].z.min);
    wsgr_min = rot * wsgr_min;
    Eigen::Vector3d wsgr_max(workspace_goal_regions_[i].x.max, workspace_goal_regions_[i].y.max,
                             workspace_goal_regions_[i].z.max);
    wsgr_max = rot * wsgr_max;
    double wsgr_min_x, wsgr_max_x, wsgr_min_y, wsgr_max_y, wsgr_min_z, wsgr_max_z;
    if (wsgr_min.x() < wsgr_max.x())
    {
      wsgr_min_x = wsgr_min.x();
      wsgr_max_x = wsgr_max.x();
    }
    else
    {
      wsgr_min_x = wsgr_max.x();
      wsgr_max_x = wsgr_min.x();
    }
    if (wsgr_min.y() < wsgr_max.y())
    {
      wsgr_min_y = wsgr_min.y();
      wsgr_max_y = wsgr_max.y();
    }
    else
    {
      wsgr_min_y = wsgr_max.y();
      wsgr_max_y = wsgr_min.y();
    }
    if (wsgr_min.z() < wsgr_max.z())
    {
      wsgr_min_z = wsgr_min.z();
      wsgr_max_z = wsgr_max.z();
    }
    else
    {
      wsgr_min_z = wsgr_max.z();
      wsgr_max_z = wsgr_min.z();
    }

    if ((abs(ee_pose.translation().x() - wsgr_tf.translation().x() - wsgr_min_x) < 0.002 ||
         ee_pose.translation().x() >= (wsgr_tf.translation().x() + wsgr_min_x)) &&
        (abs(ee_pose.translation().x() - wsgr_tf.translation().x() - wsgr_max_x) < 0.002 ||
         ee_pose.translation().x() <= (wsgr_tf.translation().x() + wsgr_max_x)) &&
        (abs(ee_pose.translation().y() - wsgr_tf.translation().y() - wsgr_min_y) < 0.002 ||
         ee_pose.translation().y() >= (wsgr_tf.translation().y() + wsgr_min_y)) &&
        (abs(ee_pose.translation().y() - wsgr_tf.translation().y() - wsgr_max_y) < 0.002 ||
         ee_pose.translation().y() <= (wsgr_tf.translation().y() + wsgr_max_y)) &&
        (abs(ee_pose.translation().z() - wsgr_tf.translation().z() - wsgr_min_z) < 0.002 ||
         ee_pose.translation().z() >= (wsgr_tf.translation().z() + wsgr_min_z)) &&
        (abs(ee_pose.translation().z() - wsgr_tf.translation().z() - wsgr_max_z) < 0.002 ||
         ee_pose.translation().z() <= (wsgr_tf.translation().z() + wsgr_max_z)))
    {
      if (workspace_goal_regions_[i].roll.free_value && workspace_goal_regions_[i].pitch.free_value &&
          workspace_goal_regions_[i].yaw.free_value)
      {
        if (sort_roadmap_func_str_.compare("Distance2Center") == 0)
          return distanceToCenterOfGoalRegion(ee_pose);
        else if (sort_roadmap_func_str_.compare("+x-y") == 0 || sort_roadmap_func_str_.compare("-x-y") == 0)
          return distanceToCornerOfGoalRegion(ee_pose);
        else if (sort_roadmap_func_str_.compare("+x") == 0 || sort_roadmap_func_str_.compare("+y") == 0 ||
                 sort_roadmap_func_str_.compare("-x") == 0 || sort_roadmap_func_str_.compare("-y") == 0)
          return distanceToEdgeOfGoalRegion(ee_pose);
      }

      // orientation constraints
      tf::Quaternion quaternion_constraints(
          constrs_[i].orientation_constraints[0].orientation.x, constrs_[i].orientation_constraints[0].orientation.y,
          constrs_[i].orientation_constraints[0].orientation.z, constrs_[i].orientation_constraints[0].orientation.w);
      tf::Matrix3x3 roation_constraints(quaternion_constraints);
      double constr_roll, constr_pitch, constr_yaw;
      roation_constraints.getRPY(constr_roll, constr_pitch, constr_yaw);

      // pose orientation
      Eigen::Matrix3d pose_roation = ee_pose.rotation();
      Eigen::Quaterniond pose_quaternion(pose_roation);
      tf::Quaternion pose_quaternion_tf(pose_quaternion.x(), pose_quaternion.y(), pose_quaternion.z(),
                                        pose_quaternion.w());

      tf::Matrix3x3 pose_roation_(pose_quaternion_tf);
      double pose_roll, pose_pitch, pose_yaw;
      pose_roation_.getRPY(pose_roll, pose_pitch, pose_yaw);

      bool meet_orientation_const = true;
      if (!workspace_goal_regions_[i].roll.free_value && abs(constr_roll - pose_roll) > 0.02)
        meet_orientation_const = false;
      if (!workspace_goal_regions_[i].pitch.free_value && abs(constr_pitch - pose_pitch) > 0.02)
        meet_orientation_const = false;
      if (!workspace_goal_regions_[i].yaw.free_value && abs(constr_yaw - pose_yaw) > 0.02)
        meet_orientation_const = false;

      if (meet_orientation_const)
      {
        if (sort_roadmap_func_str_.compare("Distance2Center") == 0)
          return distanceToCenterOfGoalRegion(ee_pose);
        else if (sort_roadmap_func_str_.compare("+x-y") == 0 || sort_roadmap_func_str_.compare("-x-y") == 0)
          return distanceToCornerOfGoalRegion(ee_pose);
        else if (sort_roadmap_func_str_.compare("+x") == 0 || sort_roadmap_func_str_.compare("+y") == 0 ||
                 sort_roadmap_func_str_.compare("-x") == 0 || sort_roadmap_func_str_.compare("-y") == 0)
          return distanceToEdgeOfGoalRegion(ee_pose);
      }
    }
  }

  return GoalStates::distanceGoal(st);
}

void ompl_interface::GoalRegionSampler::addState(const ompl::base::State* st)
{
  ompl::base::State* new_goal = si_->allocState();
  si_->copyState(new_goal, st);

  WeightedGoal* weighted_state = new WeightedGoal;
  weighted_state->state_ = new_goal;
  weighted_state->weight_ = 1.0;
  weighted_state->heap_element_ = goals_priority_queue_.insert(weighted_state);

  ompl::base::WeightedGoalRegionSampler::addState(st);
}

const std::vector<ompl::base::State*> ompl_interface::GoalRegionSampler::getGoalSamples() const
{
  return states_;
}

void ompl_interface::GoalRegionSampler::getBetterSolution(ompl::base::PathPtr solution_path)
{
  OMPL_INFORM("Getting better solution from goal regions roadmap");
  auto graph = prm_planner_->as<ompl::geometric::PRMMod>()->getRoadmap();

  ompl::base::State* start_state_roadmap = solution_path->as<ompl::geometric::PathGeometric>()->getStates().back();

  // Create the list of states
  std::list<std::tuple<double, ompl::geometric::PRMMod::Vertex>> lst;

  // Create list of roadmap nodes and distances to the center of the goal region
  ompl::geometric::PRMMod::Vertex start_vertex;
  bool start_vertex_found(false);
  double start_distance;
  BOOST_FOREACH (ompl::geometric::PRMMod::Vertex v, boost::vertices(graph))
  {
    // Solving FK
    std::vector<double> joint_values;
    for (unsigned int i = 0; i < si_->getStateDimension(); i++)
      joint_values.push_back(prm_planner_->as<ompl::geometric::PRMMod>()
                                 ->stateProperty_[v]
                                 ->as<ompl::base::RealVectorStateSpace::StateType>()
                                 ->values[i]);

    kinematic_state_->setJointGroupPositions(joint_model_group_, joint_values);
    const Eigen::Affine3d& ee_pose = kinematic_state_->getGlobalLinkTransform("gripper_link");

    /* Print end-effector pose. Remember that this is in the model frame */
    //    ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
    //    ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());

    // Distances to the goal regions
    double distance = std::numeric_limits<double>::infinity();
    if (sort_roadmap_func_str_.compare("Distance2Center") == 0)
      distance = distanceToCenterOfGoalRegion(ee_pose);
    else if (sort_roadmap_func_str_.compare("+x-y") == 0 || sort_roadmap_func_str_.compare("-x-y") == 0)
      distance = distanceToCornerOfGoalRegion(ee_pose);
    else if (sort_roadmap_func_str_.compare("+x") == 0 || sort_roadmap_func_str_.compare("+y") == 0 ||
             sort_roadmap_func_str_.compare("-x") == 0 || sort_roadmap_func_str_.compare("-y") == 0)
      distance = distanceToEdgeOfGoalRegion(ee_pose);

    // std::cout << "ee_pose.translation(): " << ee_pose.translation() << std::endl;
    // std::cout << "distance: " << distance << std::endl;

    // Adding the shortest distance and vertex to the list
    lst.push_back(std::make_tuple(distance, v));

    // Finding start vertex
    if (si_->getStateSpace()->equalStates(start_state_roadmap,
                                          prm_planner_->as<ompl::geometric::PRMMod>()->stateProperty_[v]))
    {
      start_vertex_found = true;
      start_distance = distance;
      start_vertex = v;
      break;
    }
  }

  if (start_vertex_found)
  {
    lst.sort();
    // Find an internal connection to a lower cost node
    ompl::base::PathPtr roadmap_internal_path = nullptr;
    for (auto& element : lst)
    {
      if (std::get<0>(element) < start_distance &&
          prm_planner_->as<ompl::geometric::PRMMod>()->sameComponent(start_vertex, std::get<1>(element)))
      {
        if (!si_->getStateSpace()->equalStates(
                prm_planner_->as<ompl::geometric::PRMMod>()->stateProperty_[start_vertex],
                prm_planner_->as<ompl::geometric::PRMMod>()->stateProperty_[std::get<1>(element)]))
        {
          roadmap_internal_path =
              prm_planner_->as<ompl::geometric::PRMMod>()->constructSolution(start_vertex, std::get<1>(element));
        }
        break;
      }
    }

    if (roadmap_internal_path)
    {
      // Concatenate internal path to the solution path
      std::cout << "Concatenating: " << roadmap_internal_path->as<ompl::geometric::PathGeometric>()->getStateCount() - 1
                << std::endl;
      for (unsigned int i = 1; i < roadmap_internal_path->as<ompl::geometric::PathGeometric>()->getStateCount(); i++)
      {
        solution_path->as<ompl::geometric::PathGeometric>()->append(
            roadmap_internal_path->as<ompl::geometric::PathGeometric>()->getState(i));
      }
    }
  }
}

double ompl_interface::GoalRegionSampler::distanceToCenterOfGoalRegion(const Eigen::Affine3d& ee_pose) const
{
  // Distances to the goal regions
  double distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < workspace_goal_regions_.size(); ++i)
  {
    const Eigen::Isometry3d wsgr_tf = planning_scene_->getFrameTransform(workspace_goal_regions_[i].header.frame_id);

    double gr_distance =
        sqrt(pow(ee_pose.translation().x() - wsgr_tf.translation().x(), 2.0) +
             pow(ee_pose.translation().y() - wsgr_tf.translation().y(), 2.0)); /* +
                                                                     pow(ee_pose.translation().z() -
                                                                     wsgr_tf.translation().z(), 2.0));*/

    if (gr_distance < distance)
      distance = gr_distance;
  }
  return distance;
}

double ompl_interface::GoalRegionSampler::distanceToEdgeOfGoalRegion(const Eigen::Affine3d& ee_pose) const
{
  // Distances to the goal regions
  double distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < workspace_goal_regions_.size(); ++i)
  {
    const Eigen::Isometry3d wsgr_tf = planning_scene_->getFrameTransform(workspace_goal_regions_[i].header.frame_id);
    Eigen::Matrix3d rot(wsgr_tf.rotation());
    Eigen::Vector3d wsgr_min(workspace_goal_regions_[i].x.min, workspace_goal_regions_[i].y.min,
                             workspace_goal_regions_[i].z.min);
    wsgr_min = rot * wsgr_min;
    Eigen::Vector3d wsgr_max(workspace_goal_regions_[i].x.max, workspace_goal_regions_[i].y.max,
                             workspace_goal_regions_[i].z.max);
    wsgr_max = rot * wsgr_max;
    double wsgr_min_x, wsgr_max_x, wsgr_min_y, wsgr_max_y, wsgr_min_z, wsgr_max_z;
    if (wsgr_min.x() < wsgr_max.x())
    {
      wsgr_min_x = wsgr_min.x();
      wsgr_max_x = wsgr_max.x();
    }
    else
    {
      wsgr_min_x = wsgr_max.x();
      wsgr_max_x = wsgr_min.x();
    }
    if (wsgr_min.y() < wsgr_max.y())
    {
      wsgr_min_y = wsgr_min.y();
      wsgr_max_y = wsgr_max.y();
    }
    else
    {
      wsgr_min_y = wsgr_max.y();
      wsgr_max_y = wsgr_min.y();
    }
    if (wsgr_min.z() < wsgr_max.z())
    {
      wsgr_min_z = wsgr_min.z();
      wsgr_max_z = wsgr_max.z();
    }
    else
    {
      wsgr_min_z = wsgr_max.z();
      wsgr_max_z = wsgr_min.z();
    }

    double gr_distance = std::numeric_limits<double>::infinity();

    if (sort_roadmap_func_str_.compare("+x") == 0)
    {
      //      std::cout << "(wsgr_tf.translation().x() + wsgr_max_x) - ee_pose.translation().x()" << std::endl;
      //      std::cout << "( " << wsgr_tf.translation().x() << " + " << wsgr_max_x << " ) - " <<
      //      ee_pose.translation().x()
      //                << std::endl;
      gr_distance = (wsgr_tf.translation().x() + wsgr_max_x) - ee_pose.translation().x();
    }
    else if (sort_roadmap_func_str_.compare("-x") == 0)
    {
      //      std::cout << "ee_pose.translation().x() - (wsgr_tf.translation().x() + wsgr_min_x)" << std::endl;
      //      std::cout << ee_pose.translation().x() << " - ( " << wsgr_tf.translation().x() << " + " << wsgr_min_x <<
      //      ")"
      //                << std::endl;
      gr_distance = ee_pose.translation().x() - (wsgr_tf.translation().x() + wsgr_min_x);
    }
    else if (sort_roadmap_func_str_.compare("+y") == 0)
      gr_distance = (wsgr_tf.translation().y() + wsgr_max_y) - ee_pose.translation().y();
    else if (sort_roadmap_func_str_.compare("-y") == 0)
      gr_distance = ee_pose.translation().y() - (wsgr_tf.translation().y() + wsgr_min_y);

    if (gr_distance < 0)
    {
      std::cout << "ERROR!!!!" << std::endl;
      exit(0);
    }

    if (gr_distance < distance)
      distance = gr_distance;
  }
  return distance;
}

double ompl_interface::GoalRegionSampler::distanceToCornerOfGoalRegion(const Eigen::Affine3d& ee_pose) const
{
  // Distances to the goal regions
  double distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < workspace_goal_regions_.size(); ++i)
  {
    const Eigen::Isometry3d wsgr_tf = planning_scene_->getFrameTransform(workspace_goal_regions_[i].header.frame_id);
    Eigen::Matrix3d rot(wsgr_tf.rotation());
    Eigen::Vector3d wsgr_min(workspace_goal_regions_[i].x.min, workspace_goal_regions_[i].y.min,
                             workspace_goal_regions_[i].z.min);
    wsgr_min = rot * wsgr_min;
    Eigen::Vector3d wsgr_max(workspace_goal_regions_[i].x.max, workspace_goal_regions_[i].y.max,
                             workspace_goal_regions_[i].z.max);
    wsgr_max = rot * wsgr_max;
    double wsgr_min_x, wsgr_max_x, wsgr_min_y, wsgr_max_y, wsgr_min_z, wsgr_max_z;
    if (wsgr_min.x() < wsgr_max.x())
    {
      wsgr_min_x = wsgr_min.x();
      wsgr_max_x = wsgr_max.x();
    }
    else
    {
      wsgr_min_x = wsgr_max.x();
      wsgr_max_x = wsgr_min.x();
    }
    if (wsgr_min.y() < wsgr_max.y())
    {
      wsgr_min_y = wsgr_min.y();
      wsgr_max_y = wsgr_max.y();
    }
    else
    {
      wsgr_min_y = wsgr_max.y();
      wsgr_max_y = wsgr_min.y();
    }
    if (wsgr_min.z() < wsgr_max.z())
    {
      wsgr_min_z = wsgr_min.z();
      wsgr_max_z = wsgr_max.z();
    }
    else
    {
      wsgr_min_z = wsgr_max.z();
      wsgr_max_z = wsgr_min.z();
    }

    double gr_distance = std::numeric_limits<double>::infinity();

    if (sort_roadmap_func_str_.compare("+x-y") == 0)
    {
      //      std::cout << "(wsgr_tf.translation().x() + wsgr_max_x) - ee_pose.translation().x()" << std::endl;
      //      std::cout << "( " << wsgr_tf.translation().x() << " + " << wsgr_max_x << " ) - " <<
      //      ee_pose.translation().x()
      //                << std::endl;
      gr_distance = sqrt(pow((wsgr_tf.translation().x() + wsgr_max_x) - ee_pose.translation().x(), 2.0) +
                         pow(ee_pose.translation().y() - (wsgr_tf.translation().y() + wsgr_min_y), 2.0));
    }
    else if (sort_roadmap_func_str_.compare("-x-y") == 0)
    {
      //      std::cout << "ee_pose.translation().x() - (wsgr_tf.translation().x() + wsgr_min_x)" << std::endl;
      //      std::cout << ee_pose.translation().x() << " - ( " << wsgr_tf.translation().x() << " + " << wsgr_min_x <<
      //      ")"
      //                << std::endl;
      gr_distance = sqrt(pow(ee_pose.translation().x() - (wsgr_tf.translation().x() + wsgr_min_x), 2.0) +
                         pow(ee_pose.translation().y() - (wsgr_tf.translation().y() + wsgr_min_y), 2.0));
    }

    if (gr_distance < 0)
    {
      std::cout << "ERROR!!!!" << std::endl;
      exit(0);
    }

    if (gr_distance < distance)
      distance = gr_distance;
  }
  return distance;
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
    ompl::base::State* state = se3_spaces_[i]->as<ompl::base::SE3StateSpace>()->allocState();
    se3_samplers_[i]->sampleUniform(state);

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
    unsigned int max_attempts = 2;
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

        unsigned int max_state_sampling_attempts = 2;
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

//-----------

ompl_interface::GoalRegionChecker::GoalRegionChecker(
    const std::vector<ompl::base::State*> goal_samples, const OMPLPlanningContext* pc, const std::string& group_name,
    const robot_model::RobotModelConstPtr& rm, const planning_scene::PlanningSceneConstPtr& ps,
    const std::vector<moveit_msgs::Constraints>& constrs, const std::vector<moveit_msgs::WorkspaceGoalRegion>& wsgrs,
    const std::string& sort_roadmap_func_str, constraint_samplers::ConstraintSamplerManagerPtr csm)
  : ompl::base::RandomGoalRegionSampler(pc->getOMPLSpaceInformation(),
                                        boost::bind(&GoalRegionChecker::sampleUsingConstraintSampler, this, _1, _2),
                                        false)
  , planning_context_(pc)
  , work_state_(pc->getCompleteInitialRobotState())
  , invalid_sampled_constraints_(0)
  , warned_invalid_samples_(false)
  , verbose_display_(0)
  , planning_scene_(ps)
  , constraint_sampler_manager_(csm)
  , group_name_(group_name)
  , workspace_goal_regions_(wsgrs)
  , sort_roadmap_func_str_(sort_roadmap_func_str)
  , robot_model_loader_("robot_description")
{
  for (auto& goal_sample_state : goal_samples)
    addState(goal_sample_state);
  // Kinematics robot information
  kinematic_model_ = robot_model_loader_.getModel();
  kinematic_state_ = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model_));
  kinematic_state_->setToDefaultValues();
  joint_model_group_ = kinematic_model_->getJointModelGroup(planning_context_->getGroupName());

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

  startSampling();

  OMPL_DEBUG("Creating GoalRegionChecker");
}

ompl_interface::GoalRegionChecker::GoalRegionChecker(const OMPLPlanningContext* pc, const std::string& group_name,
                                                     const robot_model::RobotModelConstPtr& rm,
                                                     const planning_scene::PlanningSceneConstPtr& ps,
                                                     const std::vector<moveit_msgs::Constraints>& constrs,
                                                     const std::vector<moveit_msgs::WorkspaceGoalRegion>& wsgrs,
                                                     const std::string& sort_roadmap_func_str,
                                                     constraint_samplers::ConstraintSamplerManagerPtr csm)
  : ompl::base::RandomGoalRegionSampler(pc->getOMPLSpaceInformation(),
                                        boost::bind(&GoalRegionChecker::sampleUsingConstraintSampler, this, _1, _2),
                                        false)
  , planning_context_(pc)
  , work_state_(pc->getCompleteInitialRobotState())
  , invalid_sampled_constraints_(0)
  , warned_invalid_samples_(false)
  , verbose_display_(0)
  , planning_scene_(ps)
  , constraint_sampler_manager_(csm)
  , group_name_(group_name)
  , workspace_goal_regions_(wsgrs)
  , sort_roadmap_func_str_(sort_roadmap_func_str)
  , robot_model_loader_("robot_description")
{
  // Kinematics robot information
  kinematic_model_ = robot_model_loader_.getModel();
  kinematic_state_ = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model_));
  kinematic_state_->setToDefaultValues();
  joint_model_group_ = kinematic_model_->getJointModelGroup(planning_context_->getGroupName());

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

  startSampling();

  OMPL_DEBUG("Creating GoalRegionChecker");
}

double ompl_interface::GoalRegionChecker::distanceGoal(const ompl::base::State* st) const
{
  // Solving FK
  std::vector<double> joint_values;
  for (unsigned int i = 0; i < si_->getStateDimension(); i++)
    joint_values.push_back(st->as<ompl::base::RealVectorStateSpace::StateType>()->values[i]);

  kinematic_state_->setJointGroupPositions(joint_model_group_, joint_values);
  const Eigen::Affine3d& ee_pose = kinematic_state_->getGlobalLinkTransform("gripper_link");

  // Distances to the goal regions
  for (std::size_t i = 0; i < workspace_goal_regions_.size(); ++i)
  {
    const Eigen::Isometry3d wsgr_tf = planning_scene_->getFrameTransform(workspace_goal_regions_[i].header.frame_id);
    Eigen::Matrix3d rot(wsgr_tf.rotation());
    Eigen::Vector3d wsgr_min(workspace_goal_regions_[i].x.min, workspace_goal_regions_[i].y.min,
                             workspace_goal_regions_[i].z.min);
    wsgr_min = rot * wsgr_min;
    Eigen::Vector3d wsgr_max(workspace_goal_regions_[i].x.max, workspace_goal_regions_[i].y.max,
                             workspace_goal_regions_[i].z.max);
    wsgr_max = rot * wsgr_max;

    double wsgr_min_x, wsgr_max_x, wsgr_min_y, wsgr_max_y, wsgr_min_z, wsgr_max_z;
    if (wsgr_min.x() < wsgr_max.x())
    {
      wsgr_min_x = wsgr_min.x();
      wsgr_max_x = wsgr_max.x();
    }
    else
    {
      wsgr_min_x = wsgr_max.x();
      wsgr_max_x = wsgr_min.x();
    }
    if (wsgr_min.y() < wsgr_max.y())
    {
      wsgr_min_y = wsgr_min.y();
      wsgr_max_y = wsgr_max.y();
    }
    else
    {
      wsgr_min_y = wsgr_max.y();
      wsgr_max_y = wsgr_min.y();
    }
    if (wsgr_min.z() < wsgr_max.z())
    {
      wsgr_min_z = wsgr_min.z();
      wsgr_max_z = wsgr_max.z();
    }
    else
    {
      wsgr_min_z = wsgr_max.z();
      wsgr_max_z = wsgr_min.z();
    }

    if ((abs(ee_pose.translation().x() - wsgr_tf.translation().x() - wsgr_min_x) < 0.002 ||
         ee_pose.translation().x() >= (wsgr_tf.translation().x() + wsgr_min_x)) &&
        (abs(ee_pose.translation().x() - wsgr_tf.translation().x() - wsgr_max_x) < 0.002 ||
         ee_pose.translation().x() <= (wsgr_tf.translation().x() + wsgr_max_x)) &&
        (abs(ee_pose.translation().y() - wsgr_tf.translation().y() - wsgr_min_y) < 0.002 ||
         ee_pose.translation().y() >= (wsgr_tf.translation().y() + wsgr_min_y)) &&
        (abs(ee_pose.translation().y() - wsgr_tf.translation().y() - wsgr_max_y) < 0.002 ||
         ee_pose.translation().y() <= (wsgr_tf.translation().y() + wsgr_max_y)) &&
        (abs(ee_pose.translation().z() - wsgr_tf.translation().z() - wsgr_min_z) < 0.002 ||
         ee_pose.translation().z() >= (wsgr_tf.translation().z() + wsgr_min_z)) &&
        (abs(ee_pose.translation().z() - wsgr_tf.translation().z() - wsgr_max_z) < 0.002 ||
         ee_pose.translation().z() <= (wsgr_tf.translation().z() + wsgr_max_z)))
    {
      if (workspace_goal_regions_[i].roll.free_value && workspace_goal_regions_[i].pitch.free_value &&
          workspace_goal_regions_[i].yaw.free_value)
      {
        return 0.0;
      }

      // orientation constraints
      tf::Quaternion quaternion_constraints(
          constrs_[i].orientation_constraints[0].orientation.x, constrs_[i].orientation_constraints[0].orientation.y,
          constrs_[i].orientation_constraints[0].orientation.z, constrs_[i].orientation_constraints[0].orientation.w);
      tf::Matrix3x3 roation_constraints(quaternion_constraints);
      double constr_roll, constr_pitch, constr_yaw;
      roation_constraints.getRPY(constr_roll, constr_pitch, constr_yaw);

      // pose orientation
      Eigen::Matrix3d pose_roation = ee_pose.rotation();
      Eigen::Quaterniond pose_quaternion(pose_roation);
      tf::Quaternion pose_quaternion_tf(pose_quaternion.x(), pose_quaternion.y(), pose_quaternion.z(),
                                        pose_quaternion.w());

      tf::Matrix3x3 pose_roation_(pose_quaternion_tf);
      double pose_roll, pose_pitch, pose_yaw;
      pose_roation_.getRPY(pose_roll, pose_pitch, pose_yaw);

      bool meet_orientation_const = true;
      if (!workspace_goal_regions_[i].roll.free_value && abs(constr_roll - pose_roll) > 0.02)
        meet_orientation_const = false;
      if (!workspace_goal_regions_[i].pitch.free_value && abs(constr_pitch - pose_pitch) > 0.02)
        meet_orientation_const = false;
      if (!workspace_goal_regions_[i].yaw.free_value && abs(constr_yaw - pose_yaw) > 0.02)
        meet_orientation_const = false;

      if (meet_orientation_const)
      {
        return 0.0;
      }
    }
  }

  return GoalStates::distanceGoal(st);
}

double ompl_interface::GoalRegionChecker::getTerminalCost(const ompl::base::State* st, bool debug_flag) const
{
  // Solving FK
  std::vector<double> joint_values;
  for (unsigned int i = 0; i < si_->getStateDimension(); i++)
    joint_values.push_back(st->as<ompl::base::RealVectorStateSpace::StateType>()->values[i]);

  kinematic_state_->setJointGroupPositions(joint_model_group_, joint_values);
  const Eigen::Affine3d& ee_pose = kinematic_state_->getGlobalLinkTransform("gripper_link");

  // Distances to the goal regions
  for (std::size_t i = 0; i < workspace_goal_regions_.size(); ++i)
  {
    const Eigen::Isometry3d wsgr_tf = planning_scene_->getFrameTransform(workspace_goal_regions_[i].header.frame_id);
    Eigen::Matrix3d rot(wsgr_tf.rotation());
    Eigen::Vector3d wsgr_min(workspace_goal_regions_[i].x.min, workspace_goal_regions_[i].y.min,
                             workspace_goal_regions_[i].z.min);
    wsgr_min = rot * wsgr_min;
    Eigen::Vector3d wsgr_max(workspace_goal_regions_[i].x.max, workspace_goal_regions_[i].y.max,
                             workspace_goal_regions_[i].z.max);
    wsgr_max = rot * wsgr_max;
    double wsgr_min_x, wsgr_max_x, wsgr_min_y, wsgr_max_y, wsgr_min_z, wsgr_max_z;
    if (wsgr_min.x() < wsgr_max.x())
    {
      wsgr_min_x = wsgr_min.x();
      wsgr_max_x = wsgr_max.x();
    }
    else
    {
      wsgr_min_x = wsgr_max.x();
      wsgr_max_x = wsgr_min.x();
    }
    if (wsgr_min.y() < wsgr_max.y())
    {
      wsgr_min_y = wsgr_min.y();
      wsgr_max_y = wsgr_max.y();
    }
    else
    {
      wsgr_min_y = wsgr_max.y();
      wsgr_max_y = wsgr_min.y();
    }
    if (wsgr_min.z() < wsgr_max.z())
    {
      wsgr_min_z = wsgr_min.z();
      wsgr_max_z = wsgr_max.z();
    }
    else
    {
      wsgr_min_z = wsgr_max.z();
      wsgr_max_z = wsgr_min.z();
    }

    if ((abs(ee_pose.translation().x() - wsgr_tf.translation().x() - wsgr_min_x) < 0.002 ||
         ee_pose.translation().x() >= (wsgr_tf.translation().x() + wsgr_min_x)) &&
        (abs(ee_pose.translation().x() - wsgr_tf.translation().x() - wsgr_max_x) < 0.002 ||
         ee_pose.translation().x() <= (wsgr_tf.translation().x() + wsgr_max_x)) &&
        (abs(ee_pose.translation().y() - wsgr_tf.translation().y() - wsgr_min_y) < 0.002 ||
         ee_pose.translation().y() >= (wsgr_tf.translation().y() + wsgr_min_y)) &&
        (abs(ee_pose.translation().y() - wsgr_tf.translation().y() - wsgr_max_y) < 0.002 ||
         ee_pose.translation().y() <= (wsgr_tf.translation().y() + wsgr_max_y)) &&
        (abs(ee_pose.translation().z() - wsgr_tf.translation().z() - wsgr_min_z) < 0.002 ||
         ee_pose.translation().z() >= (wsgr_tf.translation().z() + wsgr_min_z)) &&
        (abs(ee_pose.translation().z() - wsgr_tf.translation().z() - wsgr_max_z) < 0.002 ||
         ee_pose.translation().z() <= (wsgr_tf.translation().z() + wsgr_max_z)))
    {
      if (workspace_goal_regions_[i].roll.free_value && workspace_goal_regions_[i].pitch.free_value &&
          workspace_goal_regions_[i].yaw.free_value)
      {
        if (sort_roadmap_func_str_.compare("Distance2Center") == 0)
          return distanceToCenterOfGoalRegion(ee_pose);
        else if (sort_roadmap_func_str_.compare("+x-y") == 0 || sort_roadmap_func_str_.compare("-x-y") == 0)
          return distanceToCornerOfGoalRegion(ee_pose, debug_flag);
        else if (sort_roadmap_func_str_.compare("+x") == 0 || sort_roadmap_func_str_.compare("+y") == 0 ||
                 sort_roadmap_func_str_.compare("-x") == 0 || sort_roadmap_func_str_.compare("-y") == 0)
          return distanceToEdgeOfGoalRegion(ee_pose, debug_flag);
      }

      // orientation constraints
      tf::Quaternion quaternion_constraints(
          constrs_[i].orientation_constraints[0].orientation.x, constrs_[i].orientation_constraints[0].orientation.y,
          constrs_[i].orientation_constraints[0].orientation.z, constrs_[i].orientation_constraints[0].orientation.w);
      tf::Matrix3x3 roation_constraints(quaternion_constraints);
      double constr_roll, constr_pitch, constr_yaw;
      roation_constraints.getRPY(constr_roll, constr_pitch, constr_yaw);

      // pose orientation
      Eigen::Matrix3d pose_roation = ee_pose.rotation();
      Eigen::Quaterniond pose_quaternion(pose_roation);
      tf::Quaternion pose_quaternion_tf(pose_quaternion.x(), pose_quaternion.y(), pose_quaternion.z(),
                                        pose_quaternion.w());

      tf::Matrix3x3 pose_roation_(pose_quaternion_tf);
      double pose_roll, pose_pitch, pose_yaw;
      pose_roation_.getRPY(pose_roll, pose_pitch, pose_yaw);

      bool meet_orientation_const = true;
      if (!workspace_goal_regions_[i].roll.free_value && abs(constr_roll - pose_roll) > 0.02)
        meet_orientation_const = false;
      if (!workspace_goal_regions_[i].pitch.free_value && abs(constr_pitch - pose_pitch) > 0.02)
        meet_orientation_const = false;
      if (!workspace_goal_regions_[i].yaw.free_value && abs(constr_yaw - pose_yaw) > 0.02)
        meet_orientation_const = false;

      if (meet_orientation_const)
      {
        if (sort_roadmap_func_str_.compare("Distance2Center") == 0)
          return distanceToCenterOfGoalRegion(ee_pose);
        else if (sort_roadmap_func_str_.compare("+x-y") == 0 || sort_roadmap_func_str_.compare("-x-y") == 0)
          return distanceToCornerOfGoalRegion(ee_pose, debug_flag);
        else if (sort_roadmap_func_str_.compare("+x") == 0 || sort_roadmap_func_str_.compare("+y") == 0 ||
                 sort_roadmap_func_str_.compare("-x") == 0 || sort_roadmap_func_str_.compare("-y") == 0)
          return distanceToEdgeOfGoalRegion(ee_pose, debug_flag);
      }
    }
  }

  return GoalStates::distanceGoal(st);
}

double ompl_interface::GoalRegionChecker::distanceToCenterOfGoalRegion(const Eigen::Affine3d& ee_pose) const
{
  // Distances to the goal regions
  double distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < workspace_goal_regions_.size(); ++i)
  {
    const Eigen::Isometry3d wsgr_tf = planning_scene_->getFrameTransform(workspace_goal_regions_[i].header.frame_id);

    double gr_distance =
        sqrt(pow(ee_pose.translation().x() - wsgr_tf.translation().x(), 2.0) +
             pow(ee_pose.translation().y() - wsgr_tf.translation().y(), 2.0)); /* +
                                                                     pow(ee_pose.translation().z() -
                                                                     wsgr_tf.translation().z(), 2.0));*/

    if (gr_distance < distance)
      distance = gr_distance;
  }
  return distance;
}

double ompl_interface::GoalRegionChecker::distanceToEdgeOfGoalRegion(const Eigen::Affine3d& ee_pose,
                                                                     bool debug_flag) const
{
  // Distances to the goal regions
  double distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < workspace_goal_regions_.size(); ++i)
  {
    const Eigen::Isometry3d wsgr_tf = planning_scene_->getFrameTransform(workspace_goal_regions_[i].header.frame_id);
    Eigen::Matrix3d rot(wsgr_tf.rotation());
    Eigen::Vector3d wsgr_min(workspace_goal_regions_[i].x.min, workspace_goal_regions_[i].y.min,
                             workspace_goal_regions_[i].z.min);
    wsgr_min = rot * wsgr_min;
    Eigen::Vector3d wsgr_max(workspace_goal_regions_[i].x.max, workspace_goal_regions_[i].y.max,
                             workspace_goal_regions_[i].z.max);
    wsgr_max = rot * wsgr_max;
    double wsgr_min_x, wsgr_max_x, wsgr_min_y, wsgr_max_y, wsgr_min_z, wsgr_max_z;
    if (wsgr_min.x() < wsgr_max.x())
    {
      wsgr_min_x = wsgr_min.x();
      wsgr_max_x = wsgr_max.x();
    }
    else
    {
      wsgr_min_x = wsgr_max.x();
      wsgr_max_x = wsgr_min.x();
    }
    if (wsgr_min.y() < wsgr_max.y())
    {
      wsgr_min_y = wsgr_min.y();
      wsgr_max_y = wsgr_max.y();
    }
    else
    {
      wsgr_min_y = wsgr_max.y();
      wsgr_max_y = wsgr_min.y();
    }
    if (wsgr_min.z() < wsgr_max.z())
    {
      wsgr_min_z = wsgr_min.z();
      wsgr_max_z = wsgr_max.z();
    }
    else
    {
      wsgr_min_z = wsgr_max.z();
      wsgr_max_z = wsgr_min.z();
    }

    double gr_distance = std::numeric_limits<double>::infinity();

    if (sort_roadmap_func_str_.compare("+x") == 0)
    {
      if (debug_flag)
      {
        std::cout << "(wsgr_tf.translation().x() + wsgr_max_x) - ee_pose.translation().x()" << std::endl;
        std::cout << "( " << wsgr_tf.translation().x() << " + " << wsgr_max_x << " ) - " << ee_pose.translation().x()
                  << std::endl;
      }
      gr_distance = (wsgr_tf.translation().x() + wsgr_max_x) - ee_pose.translation().x();
    }
    else if (sort_roadmap_func_str_.compare("-x") == 0)
    {
      if (debug_flag)
      {
        std::cout << "ee_pose.translation().x() - (wsgr_tf.translation().x() + wsgr_min_x)" << std::endl;
        std::cout << ee_pose.translation().x() << " - ( " << wsgr_tf.translation().x() << " + " << wsgr_min_x << ")"
                  << std::endl;
      }
      gr_distance = ee_pose.translation().x() - (wsgr_tf.translation().x() + wsgr_min_x);
    }

    if (gr_distance < 0)
    {
      std::cout << "ERROR!!!!" << std::endl;
      exit(0);
    }

    if (gr_distance < distance)
      distance = gr_distance;
  }
  return distance;
}

double ompl_interface::GoalRegionChecker::distanceToCornerOfGoalRegion(const Eigen::Affine3d& ee_pose,
                                                                       bool debug_flag) const
{
  // Distances to the goal regions
  double distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < workspace_goal_regions_.size(); ++i)
  {
    const Eigen::Isometry3d wsgr_tf = planning_scene_->getFrameTransform(workspace_goal_regions_[i].header.frame_id);
    Eigen::Matrix3d rot(wsgr_tf.rotation());
    Eigen::Vector3d wsgr_min(workspace_goal_regions_[i].x.min, workspace_goal_regions_[i].y.min,
                             workspace_goal_regions_[i].z.min);
    wsgr_min = rot * wsgr_min;
    Eigen::Vector3d wsgr_max(workspace_goal_regions_[i].x.max, workspace_goal_regions_[i].y.max,
                             workspace_goal_regions_[i].z.max);
    wsgr_max = rot * wsgr_max;
    double wsgr_min_x, wsgr_max_x, wsgr_min_y, wsgr_max_y, wsgr_min_z, wsgr_max_z;
    if (wsgr_min.x() < wsgr_max.x())
    {
      wsgr_min_x = wsgr_min.x();
      wsgr_max_x = wsgr_max.x();
    }
    else
    {
      wsgr_min_x = wsgr_max.x();
      wsgr_max_x = wsgr_min.x();
    }
    if (wsgr_min.y() < wsgr_max.y())
    {
      wsgr_min_y = wsgr_min.y();
      wsgr_max_y = wsgr_max.y();
    }
    else
    {
      wsgr_min_y = wsgr_max.y();
      wsgr_max_y = wsgr_min.y();
    }
    if (wsgr_min.z() < wsgr_max.z())
    {
      wsgr_min_z = wsgr_min.z();
      wsgr_max_z = wsgr_max.z();
    }
    else
    {
      wsgr_min_z = wsgr_max.z();
      wsgr_max_z = wsgr_min.z();
    }

    double gr_distance = std::numeric_limits<double>::infinity();

    if (sort_roadmap_func_str_.compare("+x-y") == 0)
    {
      if (debug_flag)
      {
        std::cout << "(wsgr_tf.translation().x() + wsgr_max_x) - ee_pose.translation().x()" << std::endl;
        std::cout << "( " << wsgr_tf.translation().x() << " + " << wsgr_max_x << " ) - " << ee_pose.translation().x()
                  << std::endl;
      }
      gr_distance = sqrt(pow((wsgr_tf.translation().x() + wsgr_max_x) - ee_pose.translation().x(), 2.0) +
                         pow(ee_pose.translation().y() - (wsgr_tf.translation().y() + wsgr_min_y), 2.0));
    }
    else if (sort_roadmap_func_str_.compare("-x-y") == 0)
    {
      if (debug_flag)
      {
        std::cout << "ee_pose.translation().x() - (wsgr_tf.translation().x() + wsgr_min_x)" << std::endl;
        std::cout << ee_pose.translation().x() << " - ( " << wsgr_tf.translation().x() << " + " << wsgr_min_x << ")"
                  << std::endl;
      }
      gr_distance = sqrt(pow(ee_pose.translation().x() - (wsgr_tf.translation().x() + wsgr_min_x), 2.0) +
                         pow(ee_pose.translation().y() - (wsgr_tf.translation().y() + wsgr_min_y), 2.0));
    }

    if (gr_distance < 0)
    {
      std::cout << "ERROR!!!!" << std::endl;
      exit(0);
    }

    if (gr_distance < distance)
      distance = gr_distance;
  }
  return distance;
}

bool ompl_interface::GoalRegionChecker::checkStateValidity(ompl::base::State* new_goal,
                                                           const robot_state::RobotState& state, bool verbose) const
{
  planning_context_->copyToOMPLState(new_goal, state);
  return dynamic_cast<const StateValidityChecker*>(si_->getStateValidityChecker().get())->isValid(new_goal, verbose);
}

bool ompl_interface::GoalRegionChecker::stateValidityCallback(ompl::base::State* new_goal,
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

bool ompl_interface::GoalRegionChecker::sampleUsingConstraintSampler(const ompl::base::RandomGoalRegionSampler* gls,
                                                                     std::vector<ompl::base::State*>& sampled_states)
{
  // std::cout << "inside sampleUsingConstraintSampler ++++++: " << std::endl;
  bool success = false;

  for (std::size_t i = 0; i < workspace_goal_regions_.size(); ++i)
  {
    // Sampling an SE3 pose
    ompl::base::State* state = se3_spaces_[i]->as<ompl::base::SE3StateSpace>()->allocState();
    se3_samplers_[i]->sampleUniform(state);

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
    unsigned int max_attempts = 2;
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
            boost::bind(&ompl_interface::GoalRegionChecker::stateValidityCallback, this, goal,
                        _1,  // pointer to state
                        _2,  // const* joint model group
                        _3,  // double* of joint positions
                        verbose);
        constraint_sampler_->setGroupStateValidityCallback(gsvcf);

        unsigned int max_state_sampling_attempts = 2;
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

            success = true;
            break;  // return true;
          }
        }
      }
    }
    si_->freeState(goal);
  }
  if (success)
  {
    return true;
  }
  else
    return false;
}

void ompl_interface::GoalRegionChecker::clear()
{
  std::lock_guard<std::mutex> slock(lock_);
  constrs_.clear();
  workspace_goal_regions_.clear();
  se3_samplers_.clear();
  se3_spaces_.clear();
}
