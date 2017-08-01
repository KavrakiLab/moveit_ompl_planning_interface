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
#include <ompl/geometric/planners/trajopt/TrajOpt.h>

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
  Eigen::MatrixXd jacobian;
  kinematic_state_->setJointGroupPositions(joint_model_group_, configuration);
  kinematic_state_->getJacobian(joint_model_group_, kinematic_state_->getLinkModel(link_name),
    point, jacobian);
  return jacobian;
}

bool MoveItApiWrapper::extraCollisionInformation(
        std::vector<double> configuration,
        double& signedDist,
        Eigen::Vector3d& point,
        std::string& link_name,
        Eigen::Vector3d& normal)
{
  bool inCollision = planning_scene_->isStateColliding(*kinematic_state_, joint_model_group_->getName());
  if (inCollision) {
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
    collision_detection::Contact contact = result.contacts.begin()->second[0];
    point = contact.pos;
    normal = contact.normal;
    signedDist = std::min(result.distance, contact.depth);
    if (contact.body_type_1 == collision_detection::BodyTypes::ROBOT_LINK) {
      link_name = contact.body_name_1;
    } else if (contact.body_type_2 == collision_detection::BodyTypes::ROBOT_LINK) {
      link_name = contact.body_name_2;
    } else {
      ROS_WARN("Collisions between two attached or world objects?");
    }
  }
  // TODO get signedDistance if it's very close?
  return inCollision;
}

void TrajOptPlanningContext::initialize(const std::string& ros_namespace, PlanningContextSpecification& spec)
{
  // Don't simplify the solution, and always use the TrajOpt Planner.
  spec.simplify_solution = false;
  spec.config["type"] = "geometric::TrajOpt";
  GeometricPlanningContext::initialize(ros_namespace, spec);
  // TODO: setup the optimization objectives here.
  const ompl::base::SpaceInformationPtr &si = simple_setup_->getSpaceInformation();
  ompl::base::MultiConvexifiableOptimizationPtr bare_bones = std::make_shared<ompl::base::MultiConvexifiableOptimization>(si);
  bare_bones->addObjective(std::make_shared<ompl::base::JointDistanceObjective>(si));

  // Collision objective. Need a callback for Jacobians, a callback for collision info, and statespace.
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(getRobotModel()));
  const robot_state::JointModelGroup *joint_model_group = getJointModelGroup();
  const planning_scene::PlanningSceneConstPtr planning_scene = getPlanningScene();
  MoveItApiWrapper wrapper(kinematic_state, joint_model_group, planning_scene);

  ompl::base::JacobianFn jacobian = [&wrapper](std::vector<double> configuration, Eigen::Vector3d point, std::string link_name) {
      return wrapper.jacobianAtPoint(configuration, point, link_name);
  };

  ompl::base::WorkspaceCollisionFn collisions = [&wrapper](std::vector<double> configuration, double& signedDist, Eigen::Vector3d& point, std::string& link_name, Eigen::Vector3d& normal) {
      return wrapper.extraCollisionInformation(configuration, signedDist, point, link_name, normal);
  };
  auto it = spec_.config.find("safety_distance");
  double safety_distance = 0.0;
  if (it != spec_.config.end()) {
    safety_distance = atof(it->second.c_str());
  }
  if (safety_distance == 0.0) { // isn't in config, or config is wrong.
    safety_distance = 0.3; // default.
  }
  bare_bones->addObjective(std::make_shared<ompl::base::ObstacleConstraint>(si, safety_distance));
  simple_setup_->setOptimizationObjective(bare_bones);
}

bool TrajOptPlanningContext::solve(double timeout, unsigned int count, double& total_time)
{
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
