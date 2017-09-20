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

#include "moveit/ompl_interface/trajopt_planning_context.h"
#include "moveit/ompl_interface/geometric_planning_context.h"
#include "moveit/ompl_interface/detail/constrained_goal_sampler.h"
#include "moveit/ompl_interface/detail/constrained_sampler.h"
#include "moveit/ompl_interface/detail/goal_union.h"
#include "moveit/ompl_interface/detail/projection_evaluators.h"
#include "moveit/ompl_interface/detail/state_validity_checker.h"

#include <boost/math/constants/constants.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>

#include <pluginlib/class_list_macros.h>

#include <ompl/tools/config/SelfConfig.h>
#include <ompl/tools/multiplan/ParallelPlan.h>
#include <ompl/base/objectives/CollisionEvaluator.h>
#include <ompl/base/objectives/ObstacleConstraint.h>
#include <ompl/base/objectives/JointDistanceObjective.h>

#include <ompl/geometric/planners/trajopt/TrajOpt.h>
#include <iostream>

namespace og = ompl::geometric;

using namespace ompl_interface;

TrajOptPlanningContext::TrajOptPlanningContext() : GeometricPlanningContext()
{
  // Don't smooth the solution path, TrajOpt should already return a smoothed path.
  simplify_ = false;

  registerPlannerAllocator("geometric::TrajOpt", boost::bind(&ompl_interface::allocatePlanner<og::TrajOpt>, _1, _2, _3));
}

std::string TrajOptPlanningContext::getDescription()
{
  return "OMPL+TrajOpt Geometric Planning";
}

Eigen::MatrixXd MoveItApiWrapper::jacobianAtPoint(
        std::vector<double> configuration,
        Eigen::Vector3d point,
        std::string link_name)
{
  auto start_time = std::chrono::high_resolution_clock::now();
  Eigen::MatrixXd jacobian;
  kinematic_state_->setJointGroupPositions(joint_model_group_, configuration);
  kinematic_state_->getJacobian(joint_model_group_, kinematic_state_->getLinkModel(link_name),
    point, jacobian);
  auto end_time = std::chrono::high_resolution_clock::now();
  jaco_ms = jaco_ms + (end_time - start_time);
  return jacobian;
}

bool MoveItApiWrapper::extraCollisionInformation(
        std::vector<double> configuration,
        std::vector<double>& signedDists,
        std::vector<Eigen::Vector3d>& points,
        std::vector<std::string>& link_names,
        std::vector<Eigen::Vector3d>& normals)
{
  auto start_time = std::chrono::high_resolution_clock::now();
  kinematic_state_->setJointGroupPositions(joint_model_group_, configuration);
  bool inCollision = planning_scene_->isStateColliding(*kinematic_state_, joint_model_group_->getName());
  if (inCollision) { // || planning_scene_->distanceToCollision(*kinematic_state_) < safeDist_) {

    // Get more in depth information.
    collision_detection::CollisionRequest request;
    request.group_name = joint_model_group_->getName();
    request.distance = true;
    request.contacts = true;
    collision_detection::CollisionResult result;
    planning_scene_->checkCollision(request, result, *kinematic_state_);
    if (!result.collision) {
      // WTF! Why is this part not in collision, but the other is?
      ROS_WARN("Call to MoveIt collision detector is different for same position??");
    }
    const std::vector<std::string> group_names = joint_model_group_->getLinkModelNames();
    //ROS_INFO("Result: collision: %s, distance: %f", (result.collision) ? "true" : "false", result.distance);
    //std::cout << "Configuration: ";
    //for (auto val : configuration) {
    //    std::cout << val << ", ";
    //}
    //std::cout << std::endl;
    for (auto entry : result.contacts) {
      collision_detection::Contact contact = entry.second[0];
      for (int i = 0; i < entry.second.size(); i++) {
        if (std::abs(entry.second[i].depth) > std::abs(contact.depth)) {
            contact = entry.second[i];
        }
      }
      ROS_INFO("contact %s+%s: depth: %f", contact.body_name_1.c_str(), contact.body_name_2.c_str(), contact.depth);
      if (result.distance > 0.0) {
          signedDists.push_back(result.distance);
      } else {
          signedDists.push_back(contact.depth);
      }
      // If it's in the joint model group, then use it. Otherwise, get the closest link to that body,
      // use forward kinematics from the closest link in the group to get the vector to that point, use
      // that for the new point vector.
      if (contact.body_type_1 == collision_detection::BodyTypes::ROBOT_LINK) {
        if (std::find(group_names.begin(), group_names.end(), contact.body_name_1) != group_names.end()) {
          link_names.push_back(contact.body_name_1);
          normals.push_back(-contact.normal);
        } else { // not in joint model group, get closest link to this body.
            // TODO: finish
          //kinematic_state_->getLinkModel(contact.body_name_1);
          ROS_WARN("NOT IN JOINT GROUP!");
        }
      } else if (contact.body_type_2 == collision_detection::BodyTypes::ROBOT_LINK &&
          std::find(group_names.begin(), group_names.end(), contact.body_name_2) != group_names.end()) {
        link_names.push_back(contact.body_name_2);
        normals.push_back(contact.normal);
        break;
      } else {
        ROS_WARN("Collisions between two attached or world objects?");
      }
      // Change the contacts to be relative to the link.
      Eigen::Affine3d trans = kinematic_state_->getGlobalLinkTransform(link_names.back());
      printf("point before trans: %f, %f, %f\n", contact.pos[0], contact.pos[1], contact.pos[2]);
      points.push_back(trans.inverse() * contact.pos);
      printf("Collided point: %f, %f, %f\n", points.back()[0], points.back()[1], points.back()[2]);
    }
  }
  auto end_time = std::chrono::high_resolution_clock::now();
  coll_ms = coll_ms + (end_time - start_time);
  // TODO get signedDistance if it's very close?
  return inCollision;
}

void TrajOptPlanningContext::initialize(const std::string& ros_namespace, const PlanningContextSpecification& spec)
{
  printf("Calling initilize!!!\n");
  fflush(stdout);
  // long and annoying copy to a mutable object.
  PlanningContextSpecification specMod;
  specMod.name = spec.name;
  specMod.group = spec.group;
  specMod.planner = spec.planner;
  specMod.config = spec.config;
  // Always use the TrajOpt planner.
  specMod.config["type"] = "geometric::TrajOpt";
  specMod.simplify_solution = false;
  specMod.interpolate_solution = spec.interpolate_solution;
  specMod.min_waypoint_count = spec.min_waypoint_count;
  specMod.max_waypoint_distance = spec.max_waypoint_distance;
  specMod.max_num_threads = 1;
  specMod.model = spec.model;
  specMod.constraint_sampler_mgr = spec.constraint_sampler_mgr;

  GeometricPlanningContext::initialize(ros_namespace, specMod);
  // TODO: setup the optimization objectives here.
  const ompl::base::SpaceInformationPtr &si = simple_setup_->getSpaceInformation();
  ompl::base::MultiConvexifiableOptimizationPtr bare_bones = std::make_shared<ompl::base::MultiConvexifiableOptimization>(si);
  bare_bones->addObjective(std::make_shared<ompl::base::JointDistanceObjective>(si));

  // Collision objective. Need a callback for Jacobians, a callback for collision info, and statespace.
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(getRobotModel()));
  const robot_state::JointModelGroup *joint_model_group = getJointModelGroup();
  const planning_scene::PlanningSceneConstPtr planning_scene = getPlanningScene();
  auto it = spec_.config.find("safety_distance");
  double safety_distance = 0.0;
  if (it != spec_.config.end()) {
    safety_distance = atof(it->second.c_str());
  }
  if (safety_distance == 0.0) { // isn't in config, or config is wrong.
    safety_distance = 0.3; // default.
  }
  wrapper = new ompl_interface::MoveItApiWrapper(kinematic_state, joint_model_group, planning_scene, safety_distance);

  ompl::base::JacobianFn jacobian = [this](std::vector<double> configuration, Eigen::Vector3d point, std::string link_name) {
      return wrapper->jacobianAtPoint(configuration, point, link_name);
  };

  ompl::base::WorkspaceCollisionFn collisions = [this](std::vector<double> configuration, std::vector<double>& signedDist, std::vector<Eigen::Vector3d>& point, std::vector<std::string>& link_name, std::vector<Eigen::Vector3d>& normal) {
      return wrapper->extraCollisionInformation(configuration, signedDist, point, link_name, normal);
  };
  bare_bones->addObjective(std::make_shared<ompl::base::ObstacleConstraint>(si, safety_distance, collisions, jacobian));
  printf("BareBones Objective is %p\n", bare_bones);
  simple_setup_->setOptimizationObjective(bare_bones);
}

bool TrajOptPlanningContext::solve(double timeout, unsigned int count, double& total_time)
{
  ROS_INFO("Check again Column? %s", getPlanningScene()->getObjectType("column").db.c_str());
  printf("At solve time, pdf's objective is %p\n", simple_setup_->getPlanner()->getProblemDefinition()->getOptimizationObjective());
  ompl::time::point start = ompl::time::now();
  preSolve();

  bool result = false;
  total_time = 0.0;
  // Always use one solver, it's deterministic!
  //if (count <= 1)
  {
    ompl::base::PlannerTerminationCondition ptc =
        ompl::base::timedPlannerTerminationCondition(timeout - ompl::time::seconds(ompl::time::now() - start));
    registerTerminationCondition(ptc);
    result = simple_setup_->solve(ptc) == ompl::base::PlannerStatus::EXACT_SOLUTION;
    total_time = simple_setup_->getLastPlanComputationTime();
    unregisterTerminationCondition();
    std::cout << "Time spent in callbacks: " << std::endl <<
                 "Jacobians: " << wrapper->getJacoMs().count() << std::endl <<
                 "\tCollision: " << wrapper->getCollMs().count() << std::endl <<
                 "\tBoth:      " << wrapper->getBothMs().count() << std::endl;
  }
  /*else  // attempt to solve in parallel
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
  */

  postSolve();

  return result;
}

PLUGINLIB_EXPORT_CLASS(ompl_interface::TrajOptPlanningContext, ompl_interface::GeometricPlanningContext)
PLUGINLIB_EXPORT_CLASS(ompl_interface::TrajOptPlanningContext, ompl_interface::OMPLPlanningContext)
