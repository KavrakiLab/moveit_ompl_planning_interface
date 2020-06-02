
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
    const moveit_msgs::DMPSimulationInformation& dmp_information,
    constraint_samplers::ConstraintSamplerManagerPtr csm, 
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
  makeSetActiveRequest(learnt_dmp_.dmp_list, nh_); // I need to start the DMP server!
  
  dmp_end_ = dmp_information.dmp_end; 

  //
  

  sphere_size_ = 1.0;
  ompl::base::RealVectorBounds bounds(3);
  bounds.setLow(0, double(dmp_information_.center_point[0]) - 0.2);
  bounds.setLow(1, double(dmp_information_.center_point[1]) - 0.2);
  bounds.setLow(2, double(dmp_information_.center_point[2]) - 0.2);

  bounds.setHigh(0, double(dmp_information_.center_point[0]) + 0.2);
  bounds.setHigh(1, double(dmp_information_.center_point[1]) + 0.2);
  bounds.setHigh(2, double(dmp_information_.center_point[2]) + 0.2);

  se3_space_ = ompl::base::StateSpacePtr(new ompl::base::SE3StateSpace());
  se3_space_->as<ompl::base::SE3StateSpace>()->setBounds(bounds); 

  transition_sampler_ = se3_space_->as<ompl::base::SE3StateSpace>()->allocStateSampler();
  center_state_ = se3_space_->as<ompl::base::SE3StateSpace>()->allocState();

  center_state_->as<ompl::base::SE3StateSpace::StateType>()->setXYZ(
                                  double(dmp_information_.center_point[0]), 
                                  double(dmp_information_.center_point[1]),
                                  double(dmp_information_.center_point[2]));
  
  center_state_->as<ompl::base::SE3StateSpace::StateType>()->rotation().x = dmp_information_.center_rotation[0];
  center_state_->as<ompl::base::SE3StateSpace::StateType>()->rotation().y = dmp_information_.center_rotation[1];
  center_state_->as<ompl::base::SE3StateSpace::StateType>()->rotation().z = dmp_information_.center_rotation[2];
  center_state_->as<ompl::base::SE3StateSpace::StateType>()->rotation().w = dmp_information_.center_rotation[3];
  
  // 

  kinematic_model_ = std::make_shared<robot_model::RobotModel>(rm->getURDF(), rm->getSRDF());
  kinematic_state_ = robot_state::RobotStatePtr(new robot_state::RobotState(pc->getCompleteInitialRobotState()));

  joint_model_group_ = kinematic_model_->getJointModelGroup(planning_context_->getGroupName());

  for (auto& constr : constrs)
    constrs_.push_back(moveit_msgs::Constraints(constr));

  ompl::base::StateSpacePtr ssptr = planning_context_->getOMPLStateSpace();

  kinematic_constraint_set_.reset(new kinematic_constraints::KinematicConstraintSet(rm));
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
                                                           const robot_state::RobotState& state, bool verbose) const
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
  // Set that DMP as active.
  // makeSetActiveRequest(dmp.dmp_list, n);

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
  /*
  * The initial idea is to sample about 4 goals each iteration.
  * Sample SE3 Poses within a sphere around the object requested.
  * Using SE3 Poses, sample IK using constraint sampler.
  * Simulate and score DMPs.
  * Threshold these points.
  * Add the good points along with weight to sampled_states and heap.
  */

  bool success = false;

  auto start = std::chrono::high_resolution_clock::now();

  for (unsigned int i = 0; i < 1; i++)
  {

    // Sample SE3 Pose
    ompl::base::State* state = se3_space_->as<ompl::base::SE3StateSpace>()->allocState();
    transition_sampler_->sampleUniformNear(state, center_state_->as<ompl::base::SE3StateSpace::StateType>(), sphere_size_); // The angle not distant.
    // transition_sampler_->sampleUniform(state);

    kinematic_constraint_set_->clear();
    
    constrs_.clear();
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "world";
    pose.pose.position.x = state->as<ompl::base::SE3StateSpace::StateType>()->getX();
    pose.pose.position.y = state->as<ompl::base::SE3StateSpace::StateType>()->getY();
    pose.pose.position.z = state->as<ompl::base::SE3StateSpace::StateType>()->getZ();

    pose.pose.orientation.x = state->as<ompl::base::SE3StateSpace::StateType>()->rotation().x;
    pose.pose.orientation.y = state->as<ompl::base::SE3StateSpace::StateType>()->rotation().y;
    pose.pose.orientation.z = state->as<ompl::base::SE3StateSpace::StateType>()->rotation().z;
    pose.pose.orientation.w = state->as<ompl::base::SE3StateSpace::StateType>()->rotation().w;

    // ROS_INFO("Sampled Position: %f, %f, %f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    // ROS_INFO("Sampled Orientation: %f, %f, %f, %f", pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);

    moveit_msgs::Constraints c = kinematic_constraints::constructGoalConstraints("wrist_roll_link", pose);
    constrs_.push_back(c);

    kinematic_constraint_set_->add(constrs_[0], planning_scene_->getTransforms());
    constraint_sampler_ = constraint_sampler_manager_->selectSampler(planning_scene_, group_name_,
                                                                     kinematic_constraint_set_->getAllConstraints());

    se3_space_->freeState(state);

    unsigned int max_attempts = 2;
    unsigned int attempts_so_far = gls->samplingAttemptsCount();

    ompl::base::State* goal = si_->allocState();
    unsigned int max_attempts_div2 = max_attempts / 2;

    if (planning_context_->getOMPLProblemDefinition()->hasSolution())
      continue;  // return false;

    for (unsigned int a = 0; a < max_attempts && gls->isSampling(); ++a)
    {
      bool verbose = false;
      if (gls->getStateCount() == 0 && a >= max_attempts_div2)
      {
        if (verbose_display_ < 1)
        {
          verbose = true;
          verbose_display_++;
        }
      }

      if (constraint_sampler_)
      {
        // ROS_INFO("Constraint Sampler Good");
        robot_state::GroupStateValidityCallbackFn gsvcf =
            boost::bind(&ompl_interface::TransitionRegionSampler::stateValidityCallback, this, goal,
                        _1,  // pointer to state
                        _2,  // const* joint model group
                        _3,  // double* of joint positions
                        verbose);
        constraint_sampler_->setGroupStateValidityCallback(gsvcf);

        unsigned int max_state_sampling_attempts = 2;

        if (constraint_sampler_->project(work_state_, max_state_sampling_attempts))
        {
          work_state_.update();
          if (kinematic_constraint_set_->decide(work_state_, verbose).satisfied)
          {
            if (checkStateValidity(goal, work_state_, verbose))
            {
              // Monte-Carlo Simulation
              double score = 1.0;
              std::vector<double> joint_pos_transition_point;

              robot_state::RobotState transition_point_rs(*kinematic_state_);
              planning_context_->copyToRobotState(transition_point_rs, goal);
              transition_point_rs.copyJointGroupPositions(group_name_, joint_pos_transition_point);

              dmp::GetDMPPlanResponse planResp = simulateDMP(joint_pos_transition_point, dmp_end_, learnt_dmp_, nh_);
              dmp::DMPTraj dmp_traj = planResp.plan;        

              //
              robot_state::RobotState dmpState(*kinematic_state_);
              ompl::geometric::PathGeometric ompl_path(si_);

              // auto dmp_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(kinematic_model_, group_name_);
              // moveit_msgs::RobotTrajectory traj_msg;
              // moveit_msgs::DisplayTrajectory out;
              // out.model_id = kinematic_model_->getName();

              for (size_t i = 0; i < dmp_traj.points.size(); i++)
              {
                std::vector<double> joint_pos = dmp_traj.points[i].positions;
                dmpState.setJointGroupPositions(group_name_, joint_pos);
                ompl::base::State* currState = si_->allocState();
                planning_context_->copyToOMPLState(currState, dmpState);
                ompl_path.append(currState);
                // dmp_trajectory->insertWayPoint(i, dmpState, 0.1);
              }

              // dmp_trajectory->getRobotTrajectoryMsg(traj_msg);
              // out.trajectory.push_back(traj_msg);
              // moveit::core::robotStateToRobotStateMsg(dmp_trajectory->getFirstWayPoint(), out.trajectory_start);

              // si_->setStateValidityCheckingResolution(0.05);
              ompl_path.interpolate();

              if (ompl_path.check())
              {
                // publish the traj
                // ros::Publisher traj_pub_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("robowflex/trajectory", 1000);
                // traj_pub_.publish(out);
                
                ROS_INFO("Sampled Valid Goal");

                double smoothness = ompl_path.smoothness();
                ROS_INFO("Smoothness: %f", smoothness);
                        

                // for (int j = 0; j < si_->getStateDimension(); j++)
                //   ROS_INFO("Joint pos %f", goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[j]);

                // ros::Duration(15.0).sleep();
                // ROS_INFO("Clearance: %f", ompl_path.clearance());
              }
              else
              {
                // ROS_INFO("Invalid DMP");
                return false;
              }
              
          

              // SCORE THIS DMP
              // for (size_t i = 0; i < dmp_traj.points.size(); i++)
              // {                
              //   // Put the robot in that state
              //   std::vector<double> jointPos = dmp_traj.points[i].positions;
              //   kinematic_state_->setJointGroupPositions(joint_model_group_, jointPos); 
              //   ompl::base::State* currState = si_->allocState();
              //   planning_context_->copyToOMPLState(currState, *kinematic_state_);

              //   bool valid = dynamic_cast<const StateValidityChecker*>(si_->getStateValidityChecker().get())->isValid(currState);
      
              //   // This takes time
              //   double clearance = dynamic_cast<const StateValidityChecker*>(si_->getStateValidityChecker().get())->clearance(currState);
              //   ROS_INFO("Clearance: %f", clearance);
              //   if (!valid)
              //   {
              //     ROS_INFO("State Not Valid"); 
              //     std::vector<std::string> colliding_links;
              //     planning_scene_->getCollidingLinks(colliding_links, *kinematic_state_);
              //     for (auto& link : colliding_links)
              //       ROS_INFO("Colliding Link: %s", link.c_str());
              //     // score = score - 0.2;
              //     return false;
              //   }
              // }




              ROS_INFO("This DMP has a score of: %f", score);
              ompl::base::State* new_goal = si_->allocState();
              si_->copyState(new_goal, goal);
              sampled_states.push_back(new_goal);
              WeightedGoal* weighted_state = new WeightedGoal;
              weighted_state->state_ = new_goal;
              weighted_state->weight_ = score;
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
            }
          }
        }

      }
      else
      {
        ROS_INFO("Something up with constrained sampler");
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


