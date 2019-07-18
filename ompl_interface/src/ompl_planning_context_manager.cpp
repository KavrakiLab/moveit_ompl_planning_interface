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

#include <moveit/ompl_interface/ompl_planning_context_manager.h>

// For backward compatibility with older .yaml files.
#define DEFAULT_OMPL_PLANNING_PLUGIN "ompl_interface/GeometricPlanningContext"

using namespace ompl_interface;

OMPLPlanningContextManager::OMPLPlanningContextManager() : planning_interface::PlannerManager()
{
  ROS_WARN("BASE *******");
  constraint_sampler_manager_.reset(new constraint_samplers::ConstraintSamplerManager());
  constraint_sampler_manager_loader_.reset(
      new constraint_sampler_manager_loader::ConstraintSamplerManagerLoader(constraint_sampler_manager_));
}

/// \brief Initialize the planner manager for the given robot
/// Assumed that any ROS functionalities are namespaced by ns
bool OMPLPlanningContextManager::initialize(const robot_model::RobotModelConstPtr& model, const std::string& ns)
{
  kmodel_ = model;
  ns_ = ns;

  nh_ = ros::NodeHandle(ns);

  dynamic_reconfigure_server_.reset(
      new dynamic_reconfigure::Server<moveit_ompl_planning_interface::OMPLDynamicReconfigureConfig>(
          ros::NodeHandle(nh_, ns.empty() ? "ompl_context_mgr" : ns + "/ompl_context_mgr")));
  dynamic_reconfigure_server_->setCallback(
      boost::bind(&OMPLPlanningContextManager::dynamicReconfigureCallback, this, _1, _2));

  // Initialize planning context plugin loader
  try
  {
    ompl_planner_loader_.reset(new pluginlib::ClassLoader<OMPLPlanningContext>("moveit_ompl_planning_interface", "ompl_"
                                                                                                                 "inter"
                                                                                                                 "face:"
                                                                                                                 ":OMPL"
                                                                                                                 "Plann"
                                                                                                                 "ingCo"
                                                                                                                 "ntex"
                                                                                                                 "t"));
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
    return false;
  }

  // read in planner configurations and group information from param server
  configurePlanningContexts();

  return planning_interface::PlannerManager::initialize(model, ns);
}

std::string OMPLPlanningContextManager::getDescription() const
{
  return "OMPL+LUNA";
}

void OMPLPlanningContextManager::getPlanningAlgorithms(std::vector<std::string>& algs) const
{
  algs.clear();
  algs.reserve(config_settings_.size());
  for (const auto& config_setting : config_settings_)
    algs.push_back(config_setting.first);
}

planning_interface::PlanningContextPtr OMPLPlanningContextManager::getPlanningContext(
    const planning_scene::PlanningSceneConstPtr& planning_scene, const planning_interface::MotionPlanRequest& req,
    moveit_msgs::MoveItErrorCodes& error_code) const
{
  if (req.group_name.empty())
  {
    ROS_ERROR("No group specified to plan for");
    error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
    return planning_interface::PlanningContextPtr();
  }

  error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;

  if (!planning_scene)
  {
    ROS_ERROR("No planning scene supplied");
    return planning_interface::PlanningContextPtr();
  }

  // Initialize an (empty) planner configuration
  planning_interface::PlannerConfigurationSettings config;

  // Create a default configuration if no planner was explicitly requested
  if (req.planner_id.empty())
  {
    config.group = req.group_name;
    config.config["plugin"] = DEFAULT_OMPL_PLANNING_PLUGIN;
  }
  else
  {
    // identify the correct planning configuration
    auto pc = config_settings_.end();

    // The user can specify either "planner_name" or "group_name[planner_name]"
    pc = config_settings_.find(req.planner_id.find(req.group_name) == std::string::npos ?
                                   req.group_name + "[" + req.planner_id + "]" :
                                   req.planner_id);
    if (pc == config_settings_.end())
    {
      ROS_ERROR("No planning configuration for group '%s' using planner '%s'", req.group_name.c_str(),
                req.planner_id.c_str());
      return planning_interface::PlanningContextPtr();
    }

    config = pc->second;
  }

  std::shared_ptr<OMPLPlanningContext> context = getPlanningContext(config);

  if (context)
  {
    context->setPlanningScene(planning_scene);
    context->setMotionPlanRequest(req);

    // Creating a generic planning context for this planner
    PlanningContextSpecification spec;
    spec.name = config.name;
    spec.group = config.group;
    spec.planner = req.planner_id;
    spec.config = config.config;
    spec.model = kmodel_;
    spec.constraint_sampler_mgr = constraint_sampler_manager_;

    spec.simplify_solution = simplify_;
    spec.interpolate_solution = interpolate_;
    spec.min_waypoint_count = min_waypoint_count_;
    spec.max_waypoint_distance = max_waypoint_distance_;
    spec.max_num_threads = max_num_threads_;

    context->initialize(nh_.getNamespace(), spec);

    const moveit_msgs::WorkspaceParameters& wparams = req.workspace_parameters;
    if (wparams.min_corner.x == wparams.max_corner.x && wparams.min_corner.x == 0.0 &&
        wparams.min_corner.y == wparams.max_corner.y && wparams.min_corner.y == 0.0 &&
        wparams.min_corner.z == wparams.max_corner.z && wparams.min_corner.z == 0.0)
      ROS_WARN("The planning volume was not specified.");

    ROS_DEBUG("Setting planning volume (affects SE2 & SE3 joints only) to x = "
              "[%f, %f], y = [%f, %f], z = [%f, %f]",
              wparams.min_corner.x, wparams.max_corner.x, wparams.min_corner.y, wparams.max_corner.y,
              wparams.min_corner.z, wparams.max_corner.z);

    context->getModelBasedStateSpace()->setPlanningVolume(wparams.min_corner.x, wparams.max_corner.x,
                                                          wparams.min_corner.y, wparams.max_corner.y,
                                                          wparams.min_corner.z, wparams.max_corner.z);

    // Set the start and goal states for this query
    robot_state::RobotStatePtr start_state = planning_scene->getCurrentStateUpdated(req.start_state);
    context->setCompleteInitialRobotState(start_state);

    if (!context->setGoalConstraints(req.goal_constraints, req.goal_regions, &error_code))
      return planning_interface::PlanningContextPtr();

    try
    {
      std::shared_ptr<GeometricPlanningContext> context_geom =
          std::dynamic_pointer_cast<GeometricPlanningContext>(context);

      context_geom->configure();
      ROS_DEBUG_NAMED("planning_context_manager", "%s: New planning context is set.", context->getName().c_str());
      error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    }
    catch (ompl::Exception& ex)
    {
      ROS_ERROR_NAMED("planning_context_manager", "OMPL encountered an error: %s", ex.what());
      //      context.reset();
    }
  }
  return context;
}

/// \brief Determine whether this plugin instance is able to represent this
/// planning request
bool OMPLPlanningContextManager::canServiceRequest(const planning_interface::MotionPlanRequest& req) const
{
  // Trajectory constraints are not supported by OMPL planners
  return req.trajectory_constraints.constraints.empty();
}

std::shared_ptr<OMPLPlanningContext>
OMPLPlanningContextManager::getPlanningContext(const planning_interface::PlannerConfigurationSettings& config) const
{
  // TODO: Cache contexts we have created before?
  auto config_it = config.config.find("plugin");

  boost::shared_ptr<OMPLPlanningContext> ptr;
  if (config_it != config.config.end())
  {
    ptr = ompl_planner_loader_->createInstance(config_it->second);
  }
  else
  {
    ROS_WARN("No plugin specified for planner configuration '%s'.  Assuming "
             "default plugin.",
             config.name.c_str());
    ptr = ompl_planner_loader_->createInstance(DEFAULT_OMPL_PLANNING_PLUGIN);
  }

  return std::shared_ptr<OMPLPlanningContext>(ptr.get(), [ptr](OMPLPlanningContext*) mutable { ptr.reset(); });
}

void OMPLPlanningContextManager::configurePlanningContexts()
{
  const std::vector<std::string>& group_names = kmodel_->getJointModelGroupNames();
  planning_interface::PlannerConfigurationMap pconfig;

  // Looking for each group in the planner configurations
  for (const auto& group_name : group_names)
  {
    // get parameters specific for the robot planning group
    std::map<std::string, std::string> specific_group_params;
    getGroupSpecificParameters(group_name, specific_group_params);

    // Retrieve the parameters for each planner configured for this group
    XmlRpc::XmlRpcValue config_names;
    if (nh_.getParam(group_name + "/planner_configs", config_names))
    {
      if (config_names.getType() != XmlRpc::XmlRpcValue::TypeArray)
      {
        ROS_ERROR("Expected a list of planner configurations for group '%s'", group_name.c_str());
        continue;
      }

      for (size_t j = 0; j < config_names.size(); ++j)
      {
        if (config_names[j].getType() != XmlRpc::XmlRpcValue::TypeString)
        {
          ROS_ERROR("Expected a list of strings for the planner configurations "
                    "of group '%s'",
                    group_name.c_str());
          continue;
        }

        std::string planner_config = static_cast<std::string>(config_names[j]);
        XmlRpc::XmlRpcValue xml_config;
        if (nh_.getParam("planner_configs/" + planner_config, xml_config))
        {
          if (xml_config.getType() == XmlRpc::XmlRpcValue::TypeStruct)
          {
            planning_interface::PlannerConfigurationSettings pc;
            pc.name = group_name + "[" + planner_config + "]";
            pc.group = group_name;
            // inherit parameters from the group (which can be overriden)
            pc.config = specific_group_params;

            // read parameters specific for this configuration
            for (auto& it : xml_config)
            {
              switch (it.second.getType())
              {
                case XmlRpc::XmlRpcValue::TypeString:
                  pc.config[it.first] = static_cast<std::string>(it.second);
                  break;
                case XmlRpc::XmlRpcValue::TypeDouble:
                  pc.config[it.first] = boost::lexical_cast<std::string>(static_cast<double>(it.second));
                  break;
                case XmlRpc::XmlRpcValue::TypeInt:
                  pc.config[it.first] = std::to_string(static_cast<int>(it.second));
                  break;
                case XmlRpc::XmlRpcValue::TypeBoolean:
                  pc.config[it.first] = boost::lexical_cast<std::string>(static_cast<bool>(it.second));
                  break;
              }
            }
            std::string actual_name = group_name + "[" + planner_config + "]";
            pconfig[actual_name] = pc;
          }
        }
      }
    }
  }

  for (auto it = pconfig.begin(); it != pconfig.end(); ++it)
  {
    ROS_DEBUG_STREAM_NAMED("parameters", "Parameters for configuration '" << it->first << "'");
    for (std::map<std::string, std::string>::const_iterator config_it = it->second.config.begin();
         config_it != it->second.config.end(); ++config_it)
      ROS_DEBUG_STREAM_NAMED("parameters", " - " << config_it->first << " = " << config_it->second);
  }
  setPlannerConfigurations(pconfig);
}

void OMPLPlanningContextManager::getGroupSpecificParameters(const std::string& group_name,
                                                            std::map<std::string, std::string>& specific_group_params)
{
  // the set of planning parameters that can be specific for the group
  // (inherited by configurations of that group)
  static const std::string KNOWN_GROUP_PARAMS[] = { "projection_evaluator", "longest_valid_segment_fraction" };

  for (const auto& k : KNOWN_GROUP_PARAMS)
  {
    if (nh_.hasParam(group_name + "/" + k))
    {
      std::string value;
      if (nh_.getParam(group_name + "/" + k, value))
      {
        if (!value.empty())
          specific_group_params[k] = value;
      }
      else
      {
        double value_d;
        if (nh_.getParam(group_name + "/" + k, value_d))
          specific_group_params[k] = boost::lexical_cast<std::string>(value_d);
        else
        {
          int value_i;
          if (nh_.getParam(group_name + "/" + k, value_d))
            specific_group_params[k] = std::to_string(value_i);
          else
          {
            bool value_b;
            if (nh_.getParam(group_name + "/" + k, value_b))
              specific_group_params[k] = boost::lexical_cast<std::string>(value_b);
          }
        }
      }
    }
  }
}

void OMPLPlanningContextManager::dynamicReconfigureCallback(
    moveit_ompl_planning_interface::OMPLDynamicReconfigureConfig& config, uint32_t /*level*/)
{
  simplify_ = config.simplify_solutions;
  interpolate_ = config.minimum_waypoint_count > 2;
  min_waypoint_count_ = config.minimum_waypoint_count;
  max_waypoint_distance_ = config.maximum_waypoint_distance;
  max_num_threads_ = config.maximum_number_threads;
}

CLASS_LOADER_REGISTER_CLASS(ompl_interface::OMPLPlanningContextManager, planning_interface::PlannerManager);
