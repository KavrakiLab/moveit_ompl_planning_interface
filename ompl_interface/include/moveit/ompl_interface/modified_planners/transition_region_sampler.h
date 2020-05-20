
#ifndef MOVEIT_OMPL_INTERFACE_DETAIL_TRANSITION_REGION_SAMPLER_
#define MOVEIT_OMPL_INTERFACE_DETAIL_TRANSITION_REGION_SAMPLER_

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit/constraint_samplers/constraint_sampler.h>
#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <moveit/ompl_interface/detail/constrained_sampler.h>
#include <ompl/base/goals/GoalLazySamples.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/PrecomputedStateSampler.h>
#include <ompl/util/RandomNumbers.h>

#include <moveit/ompl_interface/modified_planners/PRMMod.h>
#include <moveit/ompl_interface/modified_planners/goal_regions_state_sampler.h>
#include <moveit/ompl_interface/modified_planners/weighted_goal_region_sampler.h>
#include <moveit/robot_state/conversions.h>

#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/WorkspaceGoalRegion.h>


namespace ompl_interface
{
class OMPLPlanningContext;

/** @class GoalRegionSampler
 *  An interface to the goal region sampler*/
class TransitionRegionSampler : public ompl::base::WeightedGoalRegionSampler
{
public:
  TransitionRegionSampler(const OMPLPlanningContext* pc, const std::string& group_name,
                        const robot_model::RobotModelConstPtr& rm, const planning_scene::PlanningSceneConstPtr& ps,
                        const std::vector<moveit_msgs::Constraints>& constrs,
                        const moveit_msgs::TransitionRegion& transition_region, 
                        constraint_samplers::ConstraintSamplerManagerPtr csm,
                        const bool use_max_sampled_goals = true, const unsigned int max_sampled_goals = 10);

  void addState(const ompl::base::State* st) override;

  const std::vector<ompl::base::State*> getGoalSamples() const;

  void clear() override;

private:
  bool sampleGoalsFromTransitionRegion(const ompl::base::WeightedGoalRegionSampler* gls,
                                       std::vector<ompl::base::State*>& sampled_states);
  bool stateValidityCallback(ompl::base::State* new_goal, robot_state::RobotState const* state,
                             const robot_model::JointModelGroup*, const double*, bool verbose = false) const;
  bool checkStateValidity(ompl::base::State* new_goal, const robot_state::RobotState& state,
                          bool verbose = false) const;

  const OMPLPlanningContext* planning_context_;
  kinematic_constraints::KinematicConstraintSetPtr kinematic_constraint_set_;
  constraint_samplers::ConstraintSamplerPtr constraint_sampler_;
  ompl::base::StateSamplerPtr default_sampler_;
  robot_state::RobotState work_state_;
  unsigned int invalid_sampled_constraints_;
  bool warned_invalid_samples_;
  unsigned int verbose_display_;

  planning_scene::PlanningSceneConstPtr planning_scene_;
//   ompl::base::StateSamplerPtr discrete_sampler_;
  std::vector<moveit_msgs::Constraints> constrs_;
  moveit_msgs::TransitionRegion transition_region_;
  constraint_samplers::ConstraintSamplerManagerPtr constraint_sampler_manager_;
  const std::string& group_name_;

//   std::map<const ompl::base::State*, double> state_weight_map_;
  std::vector<double> weights_M;
  std::vector<ompl::base::State*> states_M;

  ompl::RNG rng_;;


  // Kinematics
  robot_model_loader::RobotModelLoader robot_model_loader_;
  robot_model::RobotModelPtr kinematic_model_;
  robot_state::RobotStatePtr kinematic_state_;
  const robot_state::JointModelGroup* joint_model_group_;

};

}

#endif