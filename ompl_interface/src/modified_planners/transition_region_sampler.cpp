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
  // Load DMP
  learnt_dmp_ = loadDMP(dmp_information.dmp_name);

  // Set DMP as Active
  makeSetActiveRequest(learnt_dmp_.dmp_list, nh_);

  template_plan_ = getTemplatePlan(dmp_information.dmp_name, nh_);
  dmp_cost_ = std::make_shared<ompl_interface::DMPCost>(template_plan_);

  sphere_size_ = 0.5;

  kinematic_model_ = std::make_shared<robot_model::RobotModel>(rm->getURDF(), rm->getSRDF());
  kinematic_state_ = robot_state::RobotStatePtr(new robot_state::RobotState(pc->getCompleteInitialRobotState()));

  joint_model_group_ = kinematic_model_->getJointModelGroup(planning_context_->getGroupName());

  ompl::base::StateSpacePtr ssptr = planning_context_->getOMPLStateSpace();

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

double** ompl_interface::TransitionRegionSampler::deserialize(std::string demo_name, int& rows, int& cols)
{
  std::string fullpath = ros::package::getPath("hybrid_planner") + "/demos/" + demo_name + ".txt";
  // check if exists!!
  std::ifstream file(fullpath);
  if (file.fail())
  {
    ROS_INFO("%s demo file not found", demo_name);
    exit(0);
  }
  file >> rows; // 8
  file >> cols; // 22
  double** arr = new double*[rows];
  for (int i = 0; i < rows; i++)
  {
    arr[i] = new double[cols];
    for (int j = 0; j < cols; j++)
      file >> arr[i][j];
  }
  return arr;
}

dmp::GetDMPPlanResponse ompl_interface::TransitionRegionSampler::getTemplatePlan(std::string dmp_name,
                                                                                 ros::NodeHandle& n)
{
  dmp::GetDMPPlanResponse template_plan;
  // Simulate the learnt DMP from original start and goal
  int rows, cols;
  double** traj = deserialize(dmp_name, rows, cols);
  std::vector<double> start;
  std::vector<double> end;
  for (int d=0; d<rows; d++)
  {
    start.push_back(traj[d][0]);
    end.push_back(traj[d][cols-1]);
  }
  template_plan = simulateDMP(start, end, learnt_dmp_, n);
  return template_plan;
}

dmp::LearnDMPFromDemoResponse ompl_interface::TransitionRegionSampler::loadDMP(std::string dmp_name)
{
  std::string fullpath = ros::package::getPath("hybrid_planner") + "/DMPs/" + dmp_name + ".txt";
  dmp::LearnDMPFromDemoResponse resp;
  std::ifstream file(fullpath);

  if (file.is_open())
  {
    int dims, numF;
    double tau;
    file >> dims;
    file >> tau;
    resp.tau = tau;
    for (int i = 0; i < dims; i++)
    {
      dmp::DMPData dmp;
      file >> dmp.d_gain >> dmp.k_gain;
      file >> numF;
      dmp.f_domain.resize(numF);
      dmp.f_targets.resize(numF);
      for (int j = 0; j < numF; j++)
      {
        file >> dmp.f_domain[j] >> dmp.f_targets[j];
      }
      int numWeights;
      file >> numWeights;
      dmp.weights.resize(numWeights);
      for (int k = 0; i < numWeights; k++)
      {
        file >> dmp.weights[k];
      }
      resp.dmp_list.push_back(dmp);
    }
    ROS_INFO("%s DMP Loaded", dmp_name.c_str());
  }
  else
  {
    ROS_INFO("DMP NOT FOUND!");
    // exit(0);
  }
  return resp;
}

dmp::GetDMPPlanResponse ompl_interface::TransitionRegionSampler::makePlanRequest(
    std::vector<double> x_0, std::vector<double> x_dot_0, double t_0, std::vector<double> goal,
    std::vector<double> goalThresh, int segLength, double tau, double dt, int integrateIter, ros::NodeHandle& n)
{
  std::cout << "Making a planning request" << std::endl;
  ros::ServiceClient client = n.serviceClient<dmp::GetDMPPlan>("get_dmp_plan");
  dmp::GetDMPPlan srv;

  dmp::GetDMPPlanResponse resp;
  srv.request.x_0 = x_0;
  srv.request.x_dot_0 = x_dot_0;
  srv.request.t_0 = t_0;
  srv.request.goal = goal;
  srv.request.goal_thresh = goalThresh;
  srv.request.seg_length = segLength;
  srv.request.tau = tau;
  srv.request.dt = dt;
  srv.request.integrate_iter = integrateIter;
  if (client.call(srv))
  {
    resp = srv.response;
  }
  else
  {
    ROS_INFO("Failed to call service get_dmp_plan");
  };
  return resp;
}

void ompl_interface::TransitionRegionSampler::makeSetActiveRequest(std::vector<dmp::DMPData> dmpList,
                                                                   ros::NodeHandle& n)
{
  ros::ServiceClient client = n.serviceClient<dmp::SetActiveDMP>("set_active_dmp");
  dmp::SetActiveDMP srv;
  srv.request.dmp_list = dmpList;
  if (client.call(srv))
  {
    bool succ = srv.response.success;
    ROS_INFO("DMP Set as active!");
  }
  else
  {
    ROS_INFO("Failed to call service set_active_dmp");
  }
}

dmp::GetDMPPlanResponse ompl_interface::TransitionRegionSampler::simulateDMP(std::vector<double>& startPose,
                                                                             std::vector<double>& goalPose,
                                                                             dmp::LearnDMPFromDemoResponse& dmp,
                                                                             ros::NodeHandle& n)
{
  // Get plan reponse.
  double t_0 = 0;
  int seglength = -1;
  double tau = dmp.tau;
  int integrate_iter = 5;
  double dt = 1.0;
  std::vector<double> goal_thresh(8, 0.04);
  std::vector<double> x_dot_0{ 0, 0, 0, 0, 0, 0, 0, 0 };

  dmp::GetDMPPlanResponse planResp =
      makePlanRequest(startPose, x_dot_0, t_0, goalPose, goal_thresh, seglength, tau, dt, integrate_iter, n);
  std::cout << "DMP Goal Reached: " << unsigned(planResp.at_goal) << std::endl;

  return planResp;
}

bool ompl_interface::TransitionRegionSampler::sampleGoalsOnline(const ompl::base::WeightedGoalRegionSampler* gls,
                                                                std::vector<ompl::base::State*>& sampled_states)
{
  bool success = false;
  auto start = std::chrono::high_resolution_clock::now();

  for (unsigned int i = 0; i < 5; i++)
  {
    if (planning_context_->getOMPLProblemDefinition()->hasSolution())
      return false;

    // Setup the DMP Source Sampler
    dmp_source_constraints_ = dmp_information_.dmp_sink_constraints;
    dmp_source_constraints_.position_constraints[0].constraint_region.primitives[0].dimensions[0] = sphere_size_;
    dmp_source_constraints_.orientation_constraints[0].absolute_x_axis_tolerance = 360;
    dmp_source_constraints_.orientation_constraints[0].absolute_y_axis_tolerance = 360;
    dmp_source_constraints_.orientation_constraints[0].absolute_z_axis_tolerance = 360;
    dmp_source_constraints_.position_constraints[0].weight = 1.0;
    dmp_source_constraint_set_->clear();
    dmp_source_constraint_set_->add(dmp_source_constraints_, planning_scene_->getTransforms());
    dmp_source_sampler_ = constraint_sampler_manager_->selectSampler(planning_scene_, group_name_,
                                                                     dmp_source_constraint_set_->getAllConstraints());

    // Setup the DMP Sink Sampler
    dmp_sink_constraints_ = dmp_information_.dmp_sink_constraints;
    dmp_sink_constraint_set_->clear();
    dmp_sink_constraints_.position_constraints[0].weight = 1.0;

    dmp_sink_constraint_set_->add(dmp_sink_constraints_, planning_scene_->getTransforms());
    dmp_sink_sampler_ = constraint_sampler_manager_->selectSampler(planning_scene_, group_name_,
                                                                   dmp_sink_constraint_set_->getAllConstraints());

    if (!dmp_source_sampler_ || !dmp_sink_sampler_)
      return false;

    // Sample Sink
    robot_state::RobotState sampled_dmp_sink(*kinematic_state_);
    int max_dmp_sink_sample_attempts = 5;
    int k = 0;
    bool dmp_sink_sampled = false;
    while (k < max_dmp_sink_sample_attempts)
    {
      k++;
      bool s = dmp_sink_sampler_->sample(sampled_dmp_sink);
      if (planning_scene_->isStateValid(sampled_dmp_sink, group_name_) && s)
      {
        ROS_INFO("Good DMP Sink Sampled");
        dmp_sink_sampled = true;
        break;
      }
    }
    if (!dmp_sink_sampled)
      continue;
    std::vector<double> sampled_dmp_sink_vector;
    sampled_dmp_sink.copyJointGroupPositions(group_name_, sampled_dmp_sink_vector);

    // Sample Source
    robot_state::RobotState sampled_dmp_source(*kinematic_state_);
    int max_dmp_source_sample_attempts = 3;
    int h = 0;
    bool dmp_source_sampled = false;
    while (h < max_dmp_source_sample_attempts)
    {
      h++;
      bool s = dmp_source_sampler_->sample(sampled_dmp_source);
      if (planning_scene_->isStateValid(sampled_dmp_source, group_name_) && s)
      {
        ROS_INFO("Good DMP Source Sampled");
        dmp_source_sampled = true;
        break;
      }
    }
    if (!dmp_source_sampled)
      continue;
    std::vector<double> sampled_dmp_source_vector;
    sampled_dmp_source.copyJointGroupPositions(group_name_, sampled_dmp_source_vector);

    // Simulate DMP
    dmp::GetDMPPlanResponse planResp =
        simulateDMP(sampled_dmp_source_vector, sampled_dmp_sink_vector, learnt_dmp_, nh_);
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
    double score = 1.0;
    if (ompl_path.check())
    {
      ROS_INFO("Sampled Valid Goal");

      double cost = dmp_cost_->getCost(planResp); 
      score = 1 - cost;
      double smoothness = ompl_path.smoothness();
      double length = ompl_path.getStateCount();
      ROS_INFO("Euclidean Cost: %f", cost);
      ROS_INFO("Smoothness: %f", smoothness);
      ROS_INFO("Length: %f", length);
    }
    else
    {
      continue;  // This DMP doesn't work. Sample more.
    }

    ROS_INFO("This DMP has a score of: %f", score);
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
