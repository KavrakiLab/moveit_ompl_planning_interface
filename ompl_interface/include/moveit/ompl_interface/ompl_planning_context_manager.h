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

#ifndef MOVEIT_OMPL_INTERFACE_OMPL_PLANNING_CONTEXT_MANAGER_
#define MOVEIT_OMPL_INTERFACE_OMPL_PLANNING_CONTEXT_MANAGER_

#include <boost/scoped_ptr.hpp>
#include <dynamic_reconfigure/server.h>
#include <moveit/constraint_sampler_manager_loader/constraint_sampler_manager_loader.h>
#include <moveit/constraint_samplers/constraint_sampler_manager.h>
#include <moveit/ompl_interface/ompl_planning_context.h>
#include <moveit/planning_interface/planning_interface.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
//#include <moveit_planners_ompl/OMPLDynamicReconfigureConfig.h>
#include <moveit_ompl_planning_interface/OMPLDynamicReconfigureConfig.h>

namespace ompl_interface
{
class OMPLPlanningContextManager : public planning_interface::PlannerManager
{
public:
  OMPLPlanningContextManager();

  /// \brief Initialize the planner manager for the given robot.
  /// All ROS functionalities are namespaced by \e ns
  virtual bool initialize(const robot_model::RobotModelConstPtr& model, const std::string& ns) override;

  /// A short string that identifies the planning interface
  virtual std::string getDescription() const;

  /// \brief Get the names of the known planning algorithms (values that can be
  /// filled as planner_id in the planning request)
  virtual void getPlanningAlgorithms(std::vector<std::string>& algs) const;

  /// \brief Construct a planning context given the current scene and a
  /// planning request. If a problem is encountered, error code is set
  /// and empty ptr is returned.  The context returned  is clean -- the
  /// motion planner will start from scratch every time a context is
  /// constructed.
  /// \param planning_scene A const planning scene to use for planning
  /// \param req The representation of the planning request
  /// \param error_code This is where the error is set if constructing the
  /// planning context fails
  virtual planning_interface::PlanningContextPtr
  getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                     const planning_interface::MotionPlanRequest& req, moveit_msgs::MoveItErrorCodes& error_code) const;

  /// \brief Determine whether this plugin instance is able to represent this
  /// planning request
  virtual bool canServiceRequest(const planning_interface::MotionPlanRequest& req) const;

protected:
  /// \brief Retrieve an instance of a planning context given the configuration
  /// settings
  std::shared_ptr<OMPLPlanningContext>
  getPlanningContext(const planning_interface::PlannerConfigurationSettings& config) const;

  /// \brief Read planning context information from the ROS param server
  void configurePlanningContexts();

  /// \brief Read planning group context parameters from the ROS param server
  void getGroupSpecificParameters(const std::string& group_name,
                                  std::map<std::string, std::string>& specific_group_params);

private:
  /// \brief Callback for the dynamic reconfigure server options of this node
  void dynamicReconfigureCallback(moveit_ompl_planning_interface::OMPLDynamicReconfigureConfig& config, uint32_t level);

  /// \brief kinematic model of the robot to plan for
  robot_model::RobotModelConstPtr kmodel_;

  /// \brief The ROS namespace this plugin operates in
  std::string ns_;

  /// \brief The ROS node handle
  ros::NodeHandle nh_;

  /// \brief The plugin loader for the planning context plugins
  std::shared_ptr<pluginlib::ClassLoader<OMPLPlanningContext>> ompl_planner_loader_;

  constraint_sampler_manager_loader::ConstraintSamplerManagerLoaderPtr constraint_sampler_manager_loader_;
  constraint_samplers::ConstraintSamplerManagerPtr constraint_sampler_manager_;

  boost::scoped_ptr<dynamic_reconfigure::Server<moveit_ompl_planning_interface::OMPLDynamicReconfigureConfig>>
      dynamic_reconfigure_server_;
  bool simplify_;
  bool interpolate_;
  unsigned int min_waypoint_count_;
  double max_waypoint_distance_;
  unsigned int max_num_threads_;
};
}

#endif
