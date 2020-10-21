// MoveIt!
#include <moveit/ompl_interface/modified_planners/transition_region_sampler.h>
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

#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <utils_library/dmp_utils.h>

ompl_interface::TransitionRegionSampler::TransitionRegionSampler(
    const OMPLPlanningContext* pc, const std::string& group_name, const robot_model::RobotModelConstPtr& rm,
    const planning_scene::PlanningSceneConstPtr& ps, const std::vector<moveit_msgs::Constraints>& constrs,
    const moveit_msgs::DMPSimulationInformation& dmp_information, constraint_samplers::ConstraintSamplerManagerPtr csm,
    const bool use_max_sampled_goals, const unsigned int max_sampled_goals)
  : ompl::base::WeightedGoalRegionSampler(pc->getOMPLSpaceInformation(),
                                          boost::bind(&TransitionRegionSampler::sampleGoalsOnline, this, _1, _2),
                                          use_max_sampled_goals, max_sampled_goals, false)
  , planning_context_(pc)
  , work_state_(pc->getCompleteInitialRobotState())
  , invalid_sampled_constraints_(0)
  , warned_invalid_samples_(false)
  , verbose_display_(0)
  , planning_scene_(ps)
  , constraint_sampler_manager_(csm)
  , group_name_(group_name)
  , dmp_information_(dmp_information)
  , robot_model_loader_("robot_description")
{
  learnt_dmp_ = dmp_utils::loadDMP(dmp_information.dmp_name);
  dmp_utils::makeSetActive(learnt_dmp_.dmp_list, nh_);
  ROS_INFO("Transition Region Sampler: PourWS Set as active");
  robot_ = std::make_shared<robowflex::ParamRobot>();
  robot_->loadKinematics("arm_with_torso");
  scene_ = std::make_shared<robowflex::Scene>(robot_);
  move_group_.pullScene(scene_);
  move_group_.pullState(robot_);
  
  kinematic_model_ = std::make_shared<robot_model::RobotModel>(rm->getURDF(), rm->getSRDF());
  kinematic_state_ = robot_state::RobotStatePtr(new robot_state::RobotState(pc->getCompleteInitialRobotState()));

  template_plan_ = dmp_utils::getTemplatePlan(dmp_information.dmp_name, learnt_dmp_, nh_);
  
  dmp_cost_ = std::make_shared<ompl_interface::DMPCost>(template_plan_, *kinematic_state_, group_name_);

  joint_model_group_ = kinematic_model_->getJointModelGroup(planning_context_->getGroupName());

  dmp_source_constraint_set_.reset(new kinematic_constraints::KinematicConstraintSet(rm));
  dmp_sink_constraint_set_.reset(new kinematic_constraints::KinematicConstraintSet(rm));
  rng_ = ompl::RNG();

  marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
  marker_id_ = 0;
  startSampling();
}

double ompl_interface::TransitionRegionSampler::distanceGoal(const ompl::base::State* st) const
{
  return GoalStates::distanceGoal(st);
}

double ompl_interface::TransitionRegionSampler::getTerminalCost(const ompl::base::State* st) const
{
  return GoalStates::distanceGoal(st);
}

const std::vector<ompl::base::State*> ompl_interface::TransitionRegionSampler::getGoalSamples() const
{
  return states_;
}

bool ompl_interface::TransitionRegionSampler::checkStateValidity(ompl::base::State* new_goal,
                                                                 const robot_state::RobotState& state,
                                                                 bool verbose) const
{
  planning_context_->copyToOMPLState(new_goal, state);
  return dynamic_cast<const StateValidityChecker*>(si_->getStateValidityChecker().get())->isValid(new_goal, verbose);
}

bool ompl_interface::TransitionRegionSampler::stateValidityCallback(ompl::base::State* new_goal,
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

bool ompl_interface::TransitionRegionSampler::sampleState(robot_state::RobotStatePtr rstate,
                                                          std::vector<double>& ee_state, int max_sample_attempts,
                                                          constraint_samplers::ConstraintSamplerPtr sampler)
{
  rstate->update();
  ee_state.clear();
  int n = 0;
  while (n < max_sample_attempts)
  {
    bool succ = sampler->sample(*rstate);
    if (succ && planning_scene_->isStateValid(*rstate, group_name_))
    {
      auto ee_pose = rstate->getGlobalLinkTransform("gripper_link");
      Eigen::Quaterniond qq(ee_pose.rotation());
      ee_state.push_back(ee_pose.translation().x());
      ee_state.push_back(ee_pose.translation().y());
      ee_state.push_back(ee_pose.translation().z());
      ee_state.push_back(qq.x());
      ee_state.push_back(qq.y());
      ee_state.push_back(qq.z());
      ee_state.push_back(qq.w());
      return true;
    }
    n++;
  }
  return false;
}

std::vector<double> ompl_interface::TransitionRegionSampler::samplePourSink(moveit_msgs::Constraints sink_constraints)
{
  // Sample SE3 Pose
  double zrot = rng_.uniformReal(-1, 1);
  Eigen::Matrix3f m1;
  if (dmp_information_.dmp_name == "pour")
  {
    m1 = Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(0*M_PI, Eigen::Vector3f::UnitY()) *
         Eigen::AngleAxisf(0*M_PI, Eigen::Vector3f::UnitZ());
  }
  else if (dmp_information_.dmp_name == "pick" || dmp_information_.dmp_name == "place")
  {
    m1 = Eigen::AngleAxisf(0*M_PI, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(0*M_PI, Eigen::Vector3f::UnitY()) *
         Eigen::AngleAxisf(0*M_PI, Eigen::Vector3f::UnitZ());
  }

  Eigen::Matrix3f m2;
  m2 = Eigen::AngleAxisf(0*M_PI, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(0*M_PI, Eigen::Vector3f::UnitY()) *
       Eigen::AngleAxisf(zrot*M_PI, Eigen::Vector3f::UnitZ());

  auto total_rotation = m1 * m2;
  Eigen::Quaternionf rotq(total_rotation);

  std::vector<double> state;
  state.push_back(sink_constraints.position_constraints[0].constraint_region.primitive_poses[0].position.x);
  state.push_back(sink_constraints.position_constraints[0].constraint_region.primitive_poses[0].position.y);
  state.push_back(sink_constraints.position_constraints[0].constraint_region.primitive_poses[0].position.z);
  state.push_back(rotq.x());
  state.push_back(rotq.y());
  state.push_back(rotq.z());
  state.push_back(rotq.w());
  // constraints.orientation_constraints;
  return state;
}

std::vector<double> ompl_interface::TransitionRegionSampler::robotStateToEEVectorPose(robot_state::RobotState &state)
{
  auto base_tf = state.getGlobalLinkTransform("base_link");
  auto target_tf = state.getGlobalLinkTransform("gripper_link");
  auto ee_tf = base_tf.inverse() * target_tf;
  auto q = Eigen::Quaterniond(ee_tf.linear());
  std::vector<double> ee_vec = {
    ee_tf.translation().x(), ee_tf.translation().y(), ee_tf.translation().z(), q.x(), q.y(), q.z(), q.w()
  };
  return ee_vec;
}

double ompl_interface::TransitionRegionSampler::cartesianToJointPath(dmp::GetDMPPlanAvoidObstaclesResponse &dmp_plan, robot_trajectory::RobotTrajectoryPtr trajPtr)
{
    const auto &gsvcf =
        [this](robot_state::RobotState* state, const moveit::core::JointModelGroup* jmg,
                           const double* values) 
        { 
          state->setJointGroupPositions(jmg, values);
          state->updateCollisionBodyTransforms();
          if (!this->scene_->getScene()->isStateValid(*state, "arm_with_torso"))
            ROS_ERROR("Collision");
          return this->scene_->getScene()->isStateValid(*state, "arm_with_torso");
        }; 

    std::vector<robot_state::RobotStatePtr> trajectory;

    auto jmg_ = robot_->getScratchState()->getJointModelGroup("arm_with_torso");
    auto link_model = robot_->getScratchState()->getLinkModel("gripper_link");

    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Transform<double, 3, 1>>> waypoints;

    for (unsigned int i = 0; i < dmp_plan.plan.points.size(); i++)
    {
      Eigen::Isometry3d point;
      auto positions = dmp_plan.plan.points[i].positions;
      point.translation() = Eigen::Vector3d(positions[0], positions[1], positions[2]);
      point.linear() = Eigen::Quaterniond(positions[6], positions[3], positions[4], positions[5]).toRotationMatrix();
      waypoints.push_back(point);
    }

    moveit::core::JumpThreshold jt(0.5, 0);
    double val = robot_->getScratchState()->computeCartesianPath(jmg_, trajectory, link_model, waypoints, true, 5, jt, gsvcf);

    ROS_WARN("Compute cartesian path: %f", val);
    
    for (auto &point : trajectory)
      trajPtr->addSuffixWayPoint(*point, 0.0);

    return val;
}

void ompl_interface::TransitionRegionSampler::downsampleDMP(dmp::GetDMPPlanAvoidObstaclesResponse &dmp_plan, int factor)
{
  auto copy = dmp_plan;
  copy.plan.points.clear();
  for (unsigned int i = 0; i < dmp_plan.plan.points.size(); i++) {
    if (i % factor == 0) {
      copy.plan.points.push_back(dmp_plan.plan.points[i]);
    }
  }
  dmp_plan = copy;
}

void ompl_interface::TransitionRegionSampler::moveitTrajectoryToOMPLPath(robot_trajectory::RobotTrajectoryPtr traj, ompl::geometric::PathGeometric& ompl_path)
{
  ROS_WARN("Inside moveitTrajectoryToOMPLPath");
  ROS_WARN("Num waypoints: %d", traj->getWayPointCount());
  
  for (int i = 0; i < traj->getWayPointCount(); i++)
  {
      //robot = traj->getWayPoint(i);
      ompl::base::State* currState = si_->allocState();
      planning_context_->copyToOMPLState(currState, traj->getWayPoint(i));
      ompl_path.append(currState);
  }
}

double ompl_interface::TransitionRegionSampler::WSPathtoOMPLPath(dmp::GetDMPPlanAvoidObstaclesResponse& dmpPlan,
                                                               ompl::geometric::PathGeometric& ompl_path,
                                                               robot_state::RobotState robot)
{
  move_group_.pullScene(scene_);
  auto sc = scene_;
  const auto &gsvcf =
      [this](robot_state::RobotState* state, const moveit::core::JointModelGroup* jmg,
                         const double* values) 
      { 
        state->setJointGroupPositions(jmg, values);
        state->updateCollisionBodyTransforms();
        return this->planning_scene_->isStateValid(*state, this->group_name_);
      }; 

  auto start = std::chrono::high_resolution_clock::now();
  ompl::base::State* last_state = si_->allocState();
  double max  = 0;
  for (unsigned i=0; i < dmpPlan.plan.points.size(); i=i+5)
  {
    Eigen::Isometry3d ee_pose = Eigen::Isometry3d::Identity();
    ee_pose.translation() =
        Eigen::Vector3d(dmpPlan.plan.points[i].positions[0],
                        dmpPlan.plan.points[i].positions[1],
                        dmpPlan.plan.points[i].positions[2]);
    auto q = Eigen::Quaterniond(dmpPlan.plan.points[i].positions[6],
                                dmpPlan.plan.points[i].positions[3],
                                dmpPlan.plan.points[i].positions[4],
                                dmpPlan.plan.points[i].positions[5]);
    q.normalize();
    ee_pose.linear() = q.toRotationMatrix();

    auto curr_ee_pose = robot.getGlobalLinkTransform("gripper_link");
    auto curr_to_new = curr_ee_pose.inverse() * ee_pose;
    Eigen::VectorXd twist(6);
    Eigen::AngleAxisd aa(curr_to_new.linear());
    Eigen::Vector3d translational = curr_to_new.translation();
    Eigen::Vector3d angular = aa.angle() * aa.axis();
    twist << translational, angular;

    if (robot.setFromDiffIK(robot.getJointModelGroup("arm_with_torso"), twist, "gripper_link", 1, gsvcf))
    {
      ompl::base::State* currState = si_->allocState();
      planning_context_->copyToOMPLState(currState, robot);
      // Check Jump
      if (i > 0)
      {
        double d = si_->distance(currState, last_state);
        if (d > 5.0)
          return -1; // discontinuous
        max = std::max(d, max);
      }
      si_->copyState(last_state, currState);
      ompl_path.append(currState);
    }
    else
      return -2; // collision
  }
  //ROS_ERROR("Returning Max: %f", max);
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> time = end - start;
  ROS_ERROR("Time to trace DMP: %f", time.count());
  return max;
}

bool ompl_interface::TransitionRegionSampler::publishMarker(robot_state::RobotState &state)
{
  
  // Publish Source Marker
  marker_id_++;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time::now();
  marker.ns = "transition";
  marker.id = marker_id_; 
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  auto gripper_tf = state.getGlobalLinkTransform("gripper_link");

  marker.pose.position.x = gripper_tf.translation().x();
  marker.pose.position.y = gripper_tf.translation().y();
  marker.pose.position.z = gripper_tf.translation().z();
  marker.pose.orientation.x = Eigen::Quaterniond(gripper_tf.rotation()).x();
  marker.pose.orientation.y = Eigen::Quaterniond(gripper_tf.rotation()).y();
  marker.pose.orientation.z = Eigen::Quaterniond(gripper_tf.rotation()).z();
  marker.pose.orientation.w = Eigen::Quaterniond(gripper_tf.rotation()).w();

  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 0.5;

  marker.lifetime = ros::Duration();
  marker_array_.markers.push_back(marker);
  while (marker_pub_.getNumSubscribers() < 1)
  {
    if (!ros::ok())
    {
      return false;
    }
    ROS_WARN_ONCE("Please create a subscriber to marker array topic");
    sleep(1);
  }
  marker_pub_.publish(marker_array_);
  return true;
}

void ompl_interface::TransitionRegionSampler::interpolate(robot_trajectory::RobotTrajectoryPtr traj)
{
  ROS_INFO("Interpolating path");
  while (traj->getWayPointCount() < 100)
  {
    robot_trajectory::RobotTrajectory tmp = *traj;
    traj->clear();
    for (unsigned int i = 0; i < tmp.getWayPointCount()-1; i++)
    {
      robot_state::RobotState curr(tmp.getWayPoint(i));
      robot_state::RobotState next(tmp.getWayPoint(i+1));
      robot_state::RobotState state(robot_->getModel());
      curr.interpolate(next, 0.5, state);

      traj->addSuffixWayPoint(curr, 0.0);
      traj->addSuffixWayPoint(state, 0.0);

      if (i == tmp.getWayPointCount()-2)
        traj->addSuffixWayPoint(next, 0.0);
    }
  }
}

bool ompl_interface::TransitionRegionSampler::sampleGoalsOnline(const ompl::base::WeightedGoalRegionSampler* gls,
                                                                std::vector<ompl::base::State*>& sampled_states)
{
  ROS_INFO("Inside Sample Goals Online");
  bool success = false;
  auto start = std::chrono::high_resolution_clock::now();
  int num_sampled = 0;

  //for (unsigned int i = 0; i < 1; i++)

  if (planning_context_->getOMPLProblemDefinition()->hasSolution())
  {
    ROS_INFO("Solution already found");
    return false;
  }
 
  ROS_INFO("Setting up the samplers");
  // Setup the DMP Source Sampler
  dmp_source_constraints_ = dmp_information_.dmp_source_constraints;
  dmp_source_constraints_.position_constraints[0].weight = 1.0;
  dmp_source_constraint_set_->clear();
  dmp_source_constraint_set_->add(dmp_source_constraints_, planning_scene_->getTransforms());
  dmp_source_sampler_ = constraint_sampler_manager_->selectSampler(planning_scene_, group_name_,
                                                                   dmp_source_constraint_set_->getAllConstraints());
  // Setup Sink Sampler
  dmp_sink_constraints_ = dmp_information_.dmp_sink_constraints;
  dmp_sink_constraints_.position_constraints[0].weight = 1.0;
  dmp_sink_constraint_set_->clear();
  dmp_sink_constraint_set_->add(dmp_sink_constraints_, planning_scene_->getTransforms());
  dmp_sink_sampler_ = constraint_sampler_manager_->selectSampler(planning_scene_, group_name_,
                                                                   dmp_sink_constraint_set_->getAllConstraints());
  ROS_INFO("Setup the samplers");
  if (!dmp_source_sampler_ || !dmp_sink_sampler_)
  {
    ROS_ERROR("Error setting up the source and sink samplers.");
    return false;
  }

  ROS_INFO("About to sample sink and source");
  // Sample Sink  this should just be a 7dof SE3 point reachable by the robot.
  robot_state::RobotState sampled_dmp_sink(*kinematic_state_);

  auto sample_start = std::chrono::high_resolution_clock::now();
  while (1)
  {
    if (!dmp_sink_sampler_->sample(sampled_dmp_sink))
      continue;
    break;
  }
  ROS_INFO("Sink Sampled");

  // Sample Source 
  robot_state::RobotState sampled_dmp_source(*kinematic_state_);
  while (1)
  {
    if (!dmp_source_sampler_->sample(sampled_dmp_source))
      continue;
    break;
  }
  ROS_INFO("Source Sampled");
  auto sample_end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> sampling_time = sample_start - sample_end;
  ROS_INFO("Sampling Time: %f", sampling_time.count());


  double source_sink_distance = sampled_dmp_source.distance(sampled_dmp_sink, sampled_dmp_source.getJointModelGroup("arm_with_torso"));
  ROS_INFO("Distance between source and sink: %f", source_sink_distance);

  //publishMarker(sampled_dmp_source);

  // Convert to Vector Representation
  std::vector<double> sampled_dmp_source_vector;
  sampled_dmp_source_vector = robotStateToEEVectorPose(sampled_dmp_source);
  std::vector<double> sampled_dmp_sink_vector;
  sampled_dmp_sink_vector = robotStateToEEVectorPose(sampled_dmp_sink);

  // Simulate DMP in W-Space (with obstacle avoidance)
  //auto dmp_simulate_start = std::chrono::high_resolution_clock::now();
  dmp::GetDMPPlanAvoidObstaclesResponse planResp = dmp_utils::simulateDMPAvoidObstacles(sampled_dmp_source_vector, sampled_dmp_sink_vector, learnt_dmp_, scene_, nh_);
  //auto dmp_simulate_stop = std::chrono::high_resolution_clock::now();
  //std::chrono::duration<double> dmp_simulation_time = dmp_simulate_start - dmp_simulate_stop;
  //ROS_INFO("DMP Simulation Time: %f", dmp_simulation_time.count());

  dmp::DMPTraj dmp_traj = planResp.plan;


  // Method 1: Convert DMP Path to OMPL Path
  //ompl::geometric::PathGeometric ompl_path(si_);
  //auto dmp_conversion_start = std::chrono::high_resolution_clock::now();
  //double max_jump = WSPathtoOMPLPath(planResp, ompl_path, sampled_dmp_source); // Also Converts to IK
  //auto dmp_conversion_end = std::chrono::high_resolution_clock::now();
  //std::chrono::duration<double> dmp_conversion_time = dmp_conversion_end - dmp_conversion_start;
  //ROS_INFO("DMP Conversion Time: %f", dmp_conversion_time);
  //if (max_jump == -1)
  //{
    //ROS_ERROR("Path discontinuous");
    //return false;
  //}
  //else if (max_jump == -2)
  //{
    //ROS_ERROR("Unable to find collision free IK");
    //return false;
  //}

  ////Method 1: Score the path -> It compares the W-Space Path using DTW, and also continuity/clearance costs.
  //ROS_INFO("Sampled Valid Goal");
  //double similarity_cost = 0.002 * dmp_cost_->getCost(planResp);
  //double discontinuity_cost =  max_jump;
  //double total_cost = similarity_cost + discontinuity_cost;
  //double score = 10.0 / total_cost;
  //num_sampled++;
  //ROS_INFO("Similarity Cost: %f  Discontinuity Cost: %f", similarity_cost, discontinuity_cost);
  //ROS_INFO("Total Score: %f", score);

  // Method 2
  //move_group_.pullScene(scene_);
  //move_group_.pullState(robot_);
  
  // Set robot_state to sampled source.
  std::vector<double> source;
  sampled_dmp_source.copyJointGroupPositions("arm_with_torso", source);
  robot_->setGroupState("arm_with_torso", source);

  downsampleDMP(planResp, 15);

  ompl::geometric::PathGeometric ompl_path(si_);
  auto moveit_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(robot_->getModel(), "arm_with_torso");
  double completion = cartesianToJointPath(planResp, moveit_trajectory);
  ROS_WARN("Completion: %f", completion);
  if (completion < 0.7) return false;

  moveitTrajectoryToOMPLPath(moveit_trajectory, ompl_path);
  ompl_path.interpolate();

  // Check OMPL Path
  robot_state::RobotState rs(robot_->getModel());
  for (auto& os : ompl_path.getStates())
  {
    planning_context_->copyToRobotState(rs, os);
    if (!scene_->getScene()->isStateValid(rs, "arm_with_torso"))
    {
      ROS_ERROR("Path in collision");
      return false;
    }
  }

  // Method 2: Score
  double similarity_cost = 0.002 * dmp_cost_->getCost(planResp);
  double partial_penalty = 5 / completion;
  double score = 10 / (similarity_cost + partial_penalty);
  ROS_INFO("Similarity Cost: %f  Partial Cost: %f", similarity_cost, partial_penalty);
  ROS_INFO("Total Score: %f", score);
  num_sampled++;
  
  // Insert in heap
  ompl::base::State* new_goal = si_->allocState();
  planning_context_->copyToOMPLState(new_goal, sampled_dmp_source);
  sampled_states.push_back(new_goal);
  WeightedGoal* weighted_state = new WeightedGoal;
  weighted_state->state_ = new_goal;
  weighted_state->weight_ = score;
  auto new_dmp_path = std::make_shared<ompl::geometric::PathGeometric>(ompl_path);
  weighted_state->dmp_path_ = new_dmp_path;
  weighted_state->heap_element_ = goals_priority_queue_.insert(weighted_state);

  success = true;

  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;

  ROS_INFO("SAMPLING TIME: %f", elapsed.count());

  if (success)
    return true;
  else
    return false;
}

void ompl_interface::TransitionRegionSampler::clear()
{
  std::lock_guard<std::mutex> slock(lock_);
  WeightedGoalRegionSampler::clear();
  constrs_.clear();
}
