
#ifndef MOVEIT_OMPL_INTERFACE_DETAIL_TRANSITION_REGION_SAMPLER_
#define MOVEIT_OMPL_INTERFACE_DETAIL_TRANSITION_REGION_SAMPLER_

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit/constraint_samplers/constraint_sampler.h>
#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <moveit/ompl_interface/detail/constrained_sampler.h>
#include <moveit/ompl_interface/detail/constrained_valid_state_sampler.h>
#include <ompl/base/goals/GoalLazySamples.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/PrecomputedStateSampler.h>
#include <ompl/util/RandomNumbers.h> 
#include <ompl/geometric/PathGeometric.h> 
#include <ompl/base/OptimizationObjective.h>

#include <moveit/ompl_interface/modified_planners/PRMMod.h>
#include <moveit/ompl_interface/modified_planners/goal_regions_state_sampler.h>
#include <moveit/ompl_interface/modified_planners/weighted_goal_region_sampler.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/ompl_interface/modified_planners/dmp_cost.h>

#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/WorkspaceGoalRegion.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <ros/package.h>

// DMP
#include <dmp/DMPData.h>
#include <dmp/DMPPoint.h>
#include <dmp/DMPTraj.h>
#include <dmp/GetDMPPlan.h>
#include <dmp/LearnDMPFromDemo.h>
#include <dmp/SetActiveDMP.h>

#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <chrono>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// Robowflex
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_movegroup/services.h>
//#include <robowflex_library/io/visualization.h>

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
                          const moveit_msgs::DMPSimulationInformation& dmp_information,
                          constraint_samplers::ConstraintSamplerManagerPtr csm, const bool use_max_sampled_goals = true,
                          const unsigned int max_sampled_goals = 10);

  double distanceGoal(const ompl::base::State* st) const override;

  double getTerminalCost(const ompl::base::State* st) const;

  const std::vector<ompl::base::State*> getGoalSamples() const;

  void clear() override;

private:
  bool sampleGoalsOnline(const ompl::base::WeightedGoalRegionSampler* gls,
                         std::vector<ompl::base::State*>& sampled_states);

  bool stateValidityCallback(ompl::base::State* new_goal, robot_state::RobotState const* state,
                             const robot_model::JointModelGroup*, const double*, bool verbose = false) const;
  bool checkStateValidity(ompl::base::State* new_goal, const robot_state::RobotState& state,
                          bool verbose = false) const;

  bool sampleState(robot_state::RobotStatePtr rstate, std::vector<double>& ee_state, int max_sample_attempts,
                   constraint_samplers::ConstraintSamplerPtr sampler);

  std::vector<double> samplePourSink(moveit_msgs::Constraints sink_constraints);

  std::vector<double> robotStateToEEVectorPose(robot_state::RobotState &state);
  void WSPathtoOMPLPath(dmp::GetDMPPlanAvoidObstaclesResponse& dmpPlan, ompl::geometric::PathGeometric& ompl_path,
                        robot_state::RobotState& robot_state);

  const OMPLPlanningContext* planning_context_;
  kinematic_constraints::KinematicConstraintSetPtr kinematic_constraint_set_;
  constraint_samplers::ConstraintSamplerPtr constraint_sampler_;
  ompl::base::StateSamplerPtr default_sampler_;
  robot_state::RobotState work_state_;
  unsigned int invalid_sampled_constraints_;
  bool warned_invalid_samples_;
  unsigned int verbose_display_;

  // DMP Stuff
  dmp::LearnDMPFromDemoResponse learnt_dmp_;
  dmp::GetDMPPlanResponse template_plan_;
  std::shared_ptr<ompl_interface::DMPCost> dmp_cost_;

  moveit_msgs::Constraints dmp_sink_constraints_;
  moveit_msgs::Constraints dmp_source_constraints_;
  constraint_samplers::ConstraintSamplerPtr dmp_source_sampler_;
  constraint_samplers::ConstraintSamplerPtr dmp_sink_sampler_;
  kinematic_constraints::KinematicConstraintSetPtr dmp_source_constraint_set_;
  kinematic_constraints::KinematicConstraintSetPtr dmp_sink_constraint_set_;

  moveit_msgs::DMPSimulationInformation dmp_information_;
  ros::NodeHandle nh_;

  planning_scene::PlanningSceneConstPtr planning_scene_;
  std::vector<moveit_msgs::Constraints> constrs_;
  moveit_msgs::TransitionRegion transition_region_;
  constraint_samplers::ConstraintSamplerManagerPtr constraint_sampler_manager_;
  const std::string& group_name_;

  ompl::RNG rng_;

  // Kinematics
  robot_model_loader::RobotModelLoader robot_model_loader_;
  robot_model::RobotModelPtr kinematic_model_;
  robot_state::RobotStatePtr kinematic_state_;
  const robot_state::JointModelGroup* joint_model_group_;

  // Publish RVIZ Markers
  ros::Publisher marker_pub_;
  visualization_msgs::MarkerArray marker_array_;
  int marker_id_;
  std::vector<double> start_template_;
  std::vector<double> end_template_;

  // Roboflex and Movegroup
  robowflex::ScenePtr scene_;
  robowflex::movegroup::MoveGroupHelper move_group_;
};

}  // namespace ompl_interface

#endif
