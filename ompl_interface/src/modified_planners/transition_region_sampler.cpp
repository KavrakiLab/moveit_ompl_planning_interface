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
  ROS_INFO("Transition Region Sampler: DMP Loaded");
  dmp_utils::makeSetActive(learnt_dmp_.dmp_list, nh_);

  template_plan_ = dmp_utils::getTemplatePlan(dmp_information.dmp_name, learnt_dmp_, nh_);
  dmp_cost_ = std::make_shared<ompl_interface::DMPCost>(template_plan_);

  kinematic_model_ = std::make_shared<robot_model::RobotModel>(rm->getURDF(), rm->getSRDF());
  kinematic_state_ = robot_state::RobotStatePtr(new robot_state::RobotState(pc->getCompleteInitialRobotState()));

  joint_model_group_ = kinematic_model_->getJointModelGroup(planning_context_->getGroupName());

  dmp_sink_constraint_set_.reset(new kinematic_constraints::KinematicConstraintSet(rm));
  dmp_source_constraint_set_.reset(new kinematic_constraints::KinematicConstraintSet(rm));
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
  // robot_state::RobotStatePtr rstate = std::make_shared<robot_state::RobotState>(*kinematic_state_);
  rstate->update();
  int n = 0;
  while (n < max_sample_attempts)
  {
    bool succ = sampler->sample(*rstate);
    if (succ && planning_scene_->isStateValid(*rstate, group_name_))
    {
      // rstate->copyJointGroupPositions(group_name_, state);
      auto ee_pose = rstate->getGlobalLinkTransform("wrist_roll_link");
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

bool ompl_interface::TransitionRegionSampler::sampleGoalsOnline(const ompl::base::WeightedGoalRegionSampler* gls,
                                                                std::vector<ompl::base::State*>& sampled_states)
{
  bool success = false;
  auto start = std::chrono::high_resolution_clock::now();
  int num_sampled = 0;
  int batch_sample_size = 5;

  //while (num_sampled < batch_sample_size)

  for (unsigned int i = 0; i < 5; i++)
  {
    if (planning_context_->getOMPLProblemDefinition()->hasSolution())
      return false;

    // Setup the DMP Source Sampler
    dmp_source_constraints_ = dmp_information_.dmp_source_constraints;
    dmp_source_constraints_.position_constraints[0].weight = 1.0;
    dmp_source_constraint_set_->clear();
    dmp_source_constraint_set_->add(dmp_source_constraints_, planning_scene_->getTransforms());
    dmp_source_sampler_ = constraint_sampler_manager_->selectSampler(planning_scene_, group_name_,
                                                                     dmp_source_constraint_set_->getAllConstraints());

    ROS_INFO("Set Source Sampler");
    // Setup the DMP Sink Sampler
    dmp_sink_constraints_ = dmp_information_.dmp_sink_constraints;
    dmp_sink_constraints_.position_constraints[0].weight = 1.0;
    dmp_sink_constraint_set_->clear();
    dmp_sink_constraint_set_->add(dmp_sink_constraints_, planning_scene_->getTransforms());
    dmp_sink_sampler_ = constraint_sampler_manager_->selectSampler(planning_scene_, group_name_,
                                                                   dmp_sink_constraint_set_->getAllConstraints());

    ROS_INFO("Set Sink Sampler");
    if (!dmp_source_sampler_ || !dmp_sink_sampler_)
    {
      ROS_ERROR("Error setting up the source and sink samplers.");
      return false;
    }

    ROS_INFO("Now Sampling");
    // Sample Sink
    robot_state::RobotStatePtr sink_state_r = std::make_shared<robot_state::RobotState>(*kinematic_state_);
    std::vector<double> sink_state_ee;
    if (!sampleState(sink_state_r, sink_state_ee, 5, dmp_sink_sampler_))
      continue;

    // Sample Source this needs to be pose not joint positions
    robot_state::RobotStatePtr source_state_r = std::make_shared<robot_state::RobotState>(*kinematic_state_);
    std::vector<double> source_state_ee;
    if (!sampleState(source_state_r, source_state_ee, 3, dmp_source_sampler_))
      continue;

    // Simulate DMP
    dmp::GetDMPPlanResponse planResp = dmp_utils::simulateDMP(source_state_ee, sink_state_ee, learnt_dmp_, nh_);

    // Convert to IK
    std::vector<moveit::core::RobotStatePtr> traj;
    double val = dmp_utils::toCartesianPath(traj, planResp, kinematic_state_, group_name_, "wrist_roll_link");
    ompl::geometric::PathGeometric ompl_path(si_);

    ROS_INFO("Converted DMP Path to IK: val %f", val);

    if (traj.size() == 0)
    {
      ROS_INFO("No valid cartesian path exists.");
      continue;
    }
    // Convert to ompl path
    for (int i = 0; i < traj.size(); i++)
    {
      ompl::base::State* currState = si_->allocState();
      planning_context_->copyToOMPLState(currState, *traj[i]);
      ompl_path.append(currState);
    }

    ROS_INFO("Coverted to OMPL Path");

    ompl_path.interpolate();

    // Score the path
    double score = 1.0;
    if (ompl_path.check() && val > 0.80)
    {
      ROS_INFO("Sampled Valid Goal");

      //double cost = dmp_cost_->getCost(planResp);
      //score = 1 - cost;
      //double smoothness = ompl_path.smoothness();
      //double length = ompl_path.getStateCount();
      //ROS_INFO("Euclidean Cost: %f", cost);
      //ROS_INFO("Smoothness: %f", smoothness);
      //ROS_INFO("Length: %f", length);

      num_sampled++;
    }
    else
      continue;  // This DMP doesn't work. Sample more.

    // Insert in heap
    ROS_INFO("This DMP has a score of: %f", score);
    ompl::base::State* new_goal = si_->allocState();

    planning_context_->copyToOMPLState(new_goal, *source_state_r);

    sampled_states.push_back(new_goal);
    WeightedGoal* weighted_state = new WeightedGoal;
    weighted_state->state_ = new_goal;
    weighted_state->weight_ = score;
    auto new_dmp_path = std::make_shared<ompl::geometric::PathGeometric>(ompl_path);
    weighted_state->dmp_path_ = new_dmp_path;
    weighted_state->heap_element_ = goals_priority_queue_.insert(weighted_state);

    success = true;
    continue;  // return true;
  }

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
