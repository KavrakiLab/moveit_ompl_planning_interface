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

bool ompl_interface::TransitionRegionSampler::sampleGoalsOnline(const ompl::base::WeightedGoalRegionSampler* gls,
                                                                std::vector<ompl::base::State*>& sampled_states)
{
  ROS_INFO("Inside Sample Goals Online");
  bool success = false;
  auto start = std::chrono::high_resolution_clock::now();
  int num_sampled = 0;
  int batch_sample_size = 5;

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
  // Sample Sink
  robot_state::RobotState sampled_dmp_sink(*kinematic_state_);
  while (1)
  {
    if (!dmp_sink_sampler_->sample(sampled_dmp_sink))
      continue;
    if (!planning_scene_->isStateValid(sampled_dmp_sink, group_name_))
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
    if (!planning_scene_->isStateValid(sampled_dmp_source, group_name_))
      continue;
    break;
  }
  ROS_INFO("Source Sampled");

  marker_id_++;
  // Publish Source Marker
  visualization_msgs::Marker marker;
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time::now();
  marker.ns = "transition";
  marker.id = marker_id_; 
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  auto gripper_tf = sampled_dmp_source.getGlobalLinkTransform("gripper_link");

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
      return 0;
    }
    ROS_WARN_ONCE("Please create a subscriber to marker array topic");
    sleep(1);
  }
  marker_pub_.publish(marker_array_);

  // Convert to Vector Representation
  std::vector<double> sampled_dmp_source_vector;
  sampled_dmp_source.copyJointGroupPositions(group_name_, sampled_dmp_source_vector);
  std::vector<double> sampled_dmp_sink_vector;
  sampled_dmp_sink.copyJointGroupPositions(group_name_, sampled_dmp_sink_vector);

  // Simulate DMP
  dmp::GetDMPPlanResponse planResp = dmp_utils::simulateDMP(sampled_dmp_source_vector, sampled_dmp_sink_vector, learnt_dmp_, nh_);

  dmp::DMPTraj dmp_traj = planResp.plan;
  robot_state::RobotState dmpState(*kinematic_state_);
  ompl::geometric::PathGeometric ompl_path(si_);
  for (size_t i = 0; i < dmp_traj.points.size(); i++)
  {
    std::vector<double> joint_pos = dmp_traj.points[i].positions;
    dmpState.setJointGroupPositions(group_name_, joint_pos);
    ompl::base::State* currState = si_->allocState();
    planning_context_->copyToOMPLState(currState, dmpState);
    ompl_path.append(currState);
  }

  ompl_path.interpolate();

  ROS_INFO("About to check DMP OMPL Path for validity");
  // Score the path
  double score = 1.0;
  //double clearance = ompl_path.clearance();
  //int len = ompl_path.getStateCount();
  //ROS_INFO("DMP Path Length: %d", len);
  double clearance = 10000;
  for (int s=0; s < ompl_path.getStateCount(); s=s+50)
  {
    double c = si_->getStateValidityChecker()->clearance(ompl_path.getState(s));
    ROS_INFO("Clearance: %f", c);
    if (c<clearance)
      clearance = c;
  }

  bool collissionFree = true;
  if (clearance <= 0)
    collissionFree = false;

  // Score based on clearance.
  if (collissionFree)
  {
    ROS_INFO("Sampled Valid Goal");
    double cost = dmp_cost_->getCost(planResp);
    //score = 500 - cost;
    score = cost;
    num_sampled++;
  }
  else
  {
    ROS_INFO("DMP Path in Collision");
    return false;  // This DMP doesn't work. Sample more.
  }

  ROS_INFO("This DMP has a cost of: %f", score);
  ROS_INFO("Template Path Length: %d   DMP Path Length: %d", template_plan_.plan.points.size(), planResp.plan.points.size());
  
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
  //continue;  // return true;
  

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
