
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
    const moveit_msgs::TransitionRegion& transition_region, const moveit_msgs::DMPSimulationInformation& dmp_information,
    constraint_samplers::ConstraintSamplerManagerPtr csm, 
    const bool use_max_sampled_goals, const unsigned int max_sampled_goals)
  : ompl::base::WeightedGoalRegionSampler(pc->getOMPLSpaceInformation(),
                                          boost::bind(&TransitionRegionSampler::sampleGoalsFromTransitionRegion, this, _1, _2),
                                          use_max_sampled_goals, max_sampled_goals, false)
  , planning_context_(pc)
  , work_state_(pc->getCompleteInitialRobotState())
  , invalid_sampled_constraints_(0)
  , warned_invalid_samples_(false)
  , verbose_display_(0)
  , planning_scene_(ps)
  , constraint_sampler_manager_(csm)
  , group_name_(group_name)
  , transition_region_(transition_region)
  , dmp_information_(dmp_information)
{
  // I need to add action and object information here.
  
  ompl::base::RealVectorBounds bounds(3);
  bounds.setLow(0, dmp_information_.center_point[0] - 1);
  bounds.setLow(0, dmp_information_.center_point[1] - 1);
  bounds.setLow(0, dmp_information_.center_point[2] - 1);

  bounds.setHigh(0, dmp_information_.center_point[0] + 1);
  bounds.setHigh(0, dmp_information_.center_point[1] + 1);
  bounds.setHigh(0, dmp_information_.center_point[2] + 1);

  se3_space_->as<ompl::base::SE3StateSpace>()->setBounds(bounds);
  transition_sampler_ = se3_space_->as<ompl::base::SE3StateSpace>()->allocStateSampler();
  center_state_ = se3_space_->as<ompl::base::SE3StateSpace>()->allocState();
  center_state_->as<ompl::base::SE3StateSpace::StateType>()->setXYZ(
                                  dmp_information_.center_point[0], 
                                  dmp_information_.center_point[1],
                                  dmp_information_.center_point[2]);
  // 

  kinematic_model_ = std::make_shared<robot_model::RobotModel>(rm->getURDF(), rm->getSRDF());

  kinematic_state_ = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model_));
  kinematic_state_->setToDefaultValues();
  joint_model_group_ = kinematic_model_->getJointModelGroup(planning_context_->getGroupName());

  for (auto& constr : constrs)
    constrs_.push_back(moveit_msgs::Constraints(constr));

  ompl::base::StateSpacePtr ssptr = planning_context_->getOMPLStateSpace();

 
  for (auto &rstatemsg : transition_region_.transition_states)
  {
    ompl::base::State* ompl_state = si_->allocState();
    robot_state::RobotState rstate(rm); 
    moveit::core::robotStateMsgToRobotState(rstatemsg.state, rstate);
    planning_context_->copyToOMPLState(ompl_state, rstate);

    weights_M.push_back(double(rstatemsg.score));
    states_M.push_back(ompl_state);
  }
  
  ROS_INFO("Transition Region Sampler: Number of States: %d", states_M.size());

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

// void ompl_interface::TransitionRegionSampler::addState(const ompl::base::State* st) // Maybe add weight as argument!
// {
//   // This should not be called
//   ompl::base::State* new_goal = si_->allocState();
//   si_->copyState(new_goal, st);

//   WeightedGoal* weighted_state = new WeightedGoal;
//   weighted_state->state_ = new_goal;
//   weighted_state->weight_ = 1.0;  
//   weighted_state->heap_element_ = goals_priority_queue_.insert(weighted_state);
//   ompl::base::WeightedGoalRegionSampler::addState(st);

// }

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


  // terminate the sampling thread when a solution has been found
  // if (planning_context_->getOMPLProblemDefinition()->hasSolution())
  //   return false;
  
  for (unsigned int i = 0; i < 4; i++)
  {
    // Sample SE3 Pose
    ompl::base::State* state = se3_space_->as<ompl::base::SE3StateSpace>()->allocState();
    transition_sampler_->sampleUniformNear(state, center_state_ ,1);

    kinematic_constraint_set_->clear();

    // IK Stuff
    constrs_[0].position_constraints[0].constraint_region.primitive_poses[0].position.x =
      state->as<ompl::base::SE3StateSpace::StateType>()->getX();
    
    constrs_[0].position_constraints[0].constraint_region.primitive_poses[0].position.y =
      state->as<ompl::base::SE3StateSpace::StateType>()->getY();
    
    constrs_[0].position_constraints[0].constraint_region.primitive_poses[0].position.z =
      state->as<ompl::base::SE3StateSpace::StateType>()->getZ();

    
    tf::Quaternion q_sampled = tf::Quaternion(state->as<ompl::base::SE3StateSpace::StateType>()->rotation().x,
                                              state->as<ompl::base::SE3StateSpace::StateType>()->rotation().y,
                                              state->as<ompl::base::SE3StateSpace::StateType>()->rotation().z,
                                              state->as<ompl::base::SE3StateSpace::StateType>()->rotation().w);


    
    constrs_[0].orientation_constraints[0].orientation.x = state->as<ompl::base::SE3StateSpace::StateType>()->rotation().x;
    constrs_[0].orientation_constraints[0].orientation.y = state->as<ompl::base::SE3StateSpace::StateType>()->rotation().y;
    constrs_[0].orientation_constraints[0].orientation.z = state->as<ompl::base::SE3StateSpace::StateType>()->rotation().z;
    constrs_[0].orientation_constraints[0].orientation.w = state->as<ompl::base::SE3StateSpace::StateType>()->rotation().w;

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
              // This is a valid state. Now I need to simulate this bad boy.
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

bool ompl_interface::TransitionRegionSampler::sampleGoalsFromTransitionRegion(const ompl::base::WeightedGoalRegionSampler* gls,
                                                                        std::vector<ompl::base::State*>& sampled_states)
{
  bool success = false;
  ompl::base::State* goal = si_->allocState();
  // ompl::base::State* goalSample = si_->allocState();
  kinematic_constraint_set_->clear();

  unsigned int max_attempts = 2;
  unsigned int attempts_so_far = gls->samplingAttemptsCount();

  // terminate the sampling thread when a solution has been found
  if (planning_context_->getOMPLProblemDefinition()->hasSolution())
    return false;

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


    int r = rng_.uniformInt(0, states_M.size()-1);
    double w = weights_M[r];

    // There are 5 constraints!!

    // Add sampled state to CONSTR_
    // for (unsigned int i = 0; i < si_->getStateDimension(); i++)
    // {
    //   constrs_[0].joint_constraints[i].joint_name = joint_model_group_->getVariableNames()[i];
    //   constrs_[0].joint_constraints[i].position = states_M[r]->as<ompl::base::RealVectorStateSpace::StateType>()->values[i];
    //   constrs_[0].joint_constraints[i].tolerance_below = 0.01;
    //   constrs_[0].joint_constraints[i].tolerance_above = 0.01; 
    // }
      
    kinematic_constraint_set_->add(constrs_[0], planning_scene_->getTransforms());
    constraint_sampler_ = constraint_sampler_manager_->selectSampler(planning_scene_, group_name_,
                                                                     kinematic_constraint_set_->getAllConstraints());

    if (constraint_sampler_)
    {
      // makes the constraint sampler also perform a validity callback
        robot_state::GroupStateValidityCallbackFn gsvcf =
            boost::bind(&ompl_interface::TransitionRegionSampler::stateValidityCallback, this, goal,
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
              ROS_INFO("Sampled Collision Free Goal");
              ompl::base::State* new_goal = si_->allocState();
              si_->copyState(new_goal, goal);
              si_->freeState(goal);
              sampled_states.push_back(new_goal);
              WeightedGoal* weighted_state = new WeightedGoal;
              weighted_state->state_ = new_goal;
              weighted_state->weight_ = w;
              weighted_state->heap_element_ = goals_priority_queue_.insert(weighted_state);
              success = true;
              break;  // return true;
            }
          }
          else
          {
            ROS_INFO("Shouldn't be hitting this");
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
      ROS_INFO("Constraint Sampler is failing");
      default_sampler_->sampleUniform(goal);
      if (dynamic_cast<const StateValidityChecker*>(si_->getStateValidityChecker().get())->isValid(goal, verbose))
      {
        planning_context_->copyToRobotState(work_state_, goal);
        if (kinematic_constraint_set_->decide(work_state_, verbose).satisfied)
        {
          ompl::base::State* new_goal = si_->allocState();
          si_->copyState(new_goal, goal);
          si_->freeState(goal);
          sampled_states.push_back(new_goal);
          WeightedGoal* weighted_state = new WeightedGoal;
          weighted_state->state_ = new_goal;
          weighted_state->weight_ = w;
          weighted_state->heap_element_ = goals_priority_queue_.insert(weighted_state);
          success = true;
          break;  // return true;
        }
      }
    }
  }

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


