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

#ifndef MOVEIT_OMPL_INTERFACE_GEOMETRIC_PLANNING_CONTEXT_
#define MOVEIT_OMPL_INTERFACE_GEOMETRIC_PLANNING_CONTEXT_

#include "moveit/ompl_interface/ompl_planning_context.h"
#include <ros/ros.h>
//#include "moveit/ompl_interface/constraints_library.h"
#include <boost/thread/mutex.hpp>
#include <ompl/geometric/SimpleSetup.h>

namespace ompl_interface
{
/// \brief Definition of a geometric planning context.  This context plans in
/// the space
/// of joint angles for a given group.  This context is NOT thread safe.
class GeometricPlanningContext : public OMPLPlanningContext
{
public:
  GeometricPlanningContext();

  virtual ~GeometricPlanningContext();

  virtual std::string getDescription();

  virtual void initialize(const std::string& ros_namespace, const PlanningContextSpecification& spec);

  /// \brief Clear all data structures used by the planner
  virtual void clear();

  /// \brief Solve the motion planning problem and store the result in \e res.
  /// This function should not clear data structures before computing. The
  /// constructor
  /// and clear() do that.
  virtual bool solve(planning_interface::MotionPlanResponse& res);

  /// \brief Solve the motion planning problem and store the detailed result in
  /// \e res.
  /// This function should not clear data structures before computing. The
  /// constructor
  /// and clear() do that.
  virtual bool solve(planning_interface::MotionPlanDetailedResponse& res);

  /// \brief Stop planning
  virtual bool terminate();

  virtual const ompl::base::StateSpacePtr& getOMPLStateSpace() const override;

  virtual ModelBasedStateSpace* getModelBasedStateSpace();
  virtual const ModelBasedStateSpace* getModelBasedStateSpace() const override;

  virtual void copyToRobotState(robot_state::RobotState& rstate, const ompl::base::State* state) const override
  {
    getModelBasedStateSpace()->copyToRobotState(rstate, state);
  }

  virtual void copyToOMPLState(ompl::base::State* state, const robot_state::RobotState& rstate) const override
  {
    getModelBasedStateSpace()->copyToOMPLState(state, rstate);
  }

  virtual const ompl::base::SpaceInformationPtr& getOMPLSpaceInformation() const;

  virtual const ompl::base::ProblemDefinitionPtr& getOMPLProblemDefinition() const;

  virtual const robot_state::RobotState& getCompleteInitialRobotState() const;

  /// \brief Set the initial (start) state of the robot
  virtual void setCompleteInitialRobotState(const robot_state::RobotState& state);

  /// \brief Set the set of constraints that encapsulate the set of valid goal
  /// states
  /// These constraints are merged with any path constraints that are specified
  /// in the
  /// motion plan request.
  virtual bool setGoalConstraints(const std::vector<moveit_msgs::Constraints>& goal_constraints,
                                  moveit_msgs::MoveItErrorCodes* error);

  /// \brief Return the set of constraints that must be satisfied along the
  /// entire path
  virtual const kinematic_constraints::KinematicConstraintSetPtr& getPathConstraints() const;

protected:
  /// \brief Merge constraints c1 and c2, storing the result in output.
  /// If the constraints cannot be merged, false is returned.
  virtual bool mergeConstraints(const moveit_msgs::Constraints& c1, const moveit_msgs::Constraints& c2,
                                moveit_msgs::Constraints& output) const;

  /// \brief Simplify the solution path (in simple setup).  Use no more than
  /// max_time seconds.
  virtual double simplifySolution(double max_time);

  /// \brief Ensure that the given path has at least waypoint_count waypoints.
  virtual double interpolateSolution(ompl::geometric::PathGeometric& path, unsigned int waypoint_count);

  /// \brief Definition of a PlannerAllocator function.  This function takes the
  /// OMPL SpaceInformation
  /// object, the new name of the planner (if any), and a map containing planner
  /// configuration items.
  typedef boost::function<ompl::base::PlannerPtr(const ompl::base::SpaceInformationPtr&, const std::string&,
                                                 const std::map<std::string, std::string>&)>
      PlannerAllocator;

  /// \brief Allocate the StateSpace for the given specification.  This will
  /// initialize the
  /// \e mbss_ member.
  virtual void allocateStateSpace(const ModelBasedStateSpaceSpecification& state_space_spec);

  /// \brief Allocate a (possibly constrained) state sampler.  If there are no
  /// path constraints, the
  /// sampler is the default from OMPL.  Otherwise, a custom sampler is created
  /// to sample states from
  /// the constraints specified in the motion plan request.
  virtual ompl::base::StateSamplerPtr allocPathConstrainedSampler(const ompl::base::StateSpace* ss) const;

  /// \brief A method that is invoked immediately before every call to solve()
  virtual void preSolve();

  /// \brief A method that is invoked immediately after every call to solve()
  virtual void postSolve();

  /// \brief The solve method that actually does all of the solving
  /// Solve the problem \e count times or until \e timeout seconds elapse.
  /// The total time taken by this call is returned in \e total_time.
  virtual bool solve(double timeout, unsigned int count, double& total_time);

  /// \brief Begin the goal sampling thread
  void startGoalSampling();

  /// \brief Stop the goal sampling thread
  void stopGoalSampling();

  /// \brief Set the currently running termination condition.  Used for
  /// terminate()
  void registerTerminationCondition(const ompl::base::PlannerTerminationCondition& ptc);

  /// \brief Clear the currently running termination condition.  Used for
  /// terminate()
  void unregisterTerminationCondition();

  /// \brief Return the complete robot model
  const robot_model::RobotModelConstPtr& getRobotModel() const;

  /// \brief Return the robot model for the group being planned
  const robot_model::JointModelGroup* getJointModelGroup() const;

  /// \brief Initialize all of the planner allocators for planners this context
  /// is aware of
  void initializePlannerAllocators();

  /// \brief Associate the given planner_id string with the given planner
  /// allocator
  void registerPlannerAllocator(const std::string& planner_id, const PlannerAllocator& pa);

  /// \brief Return an instance of the given planner_name configured with the
  /// given parameters
  virtual ompl::base::PlannerPtr configurePlanner(const std::string& planner_name,
                                                  const std::map<std::string, std::string>& params);

  /// \brief Configure a new projection evaluator given the string encoding
  virtual ompl::base::ProjectionEvaluatorPtr getProjectionEvaluator(const std::string& peval) const;

  /// \brief Register a projection evaluator for the OMPL state space given the
  /// string encoding
  virtual void setProjectionEvaluator(const std::string& peval);

  /// \brief Pointer to the OMPL SimpleSetup object
  ompl::geometric::SimpleSetupPtr simple_setup_;

  /// \brief Pointer to the (derived) OMPL StateSpace object
  ompl::base::StateSpacePtr mbss_;

  /// \brief Robot state containing the initial position of all joints
  robot_state::RobotState* complete_initial_robot_state_;

  /// \brief The set of goal constraints to achieve
  std::vector<kinematic_constraints::KinematicConstraintSetPtr> goal_constraints_;

  /// \brief The (possibly empty) set of constraints that must be satisfied
  /// along the entire path.
  kinematic_constraints::KinematicConstraintSetPtr path_constraints_;

  /// \brief The constraint sampler factory.
  constraint_samplers::ConstraintSamplerManagerPtr constraint_sampler_manager_;

  /// \brief The specification parameters for this context
  PlanningContextSpecification spec_;

  /// \brief The id of the planner this context is configured for
  std::string planner_id_;

  /// \brief The set of planner allocators that have been registered
  std::map<std::string, PlannerAllocator> planner_allocators_;

  /// \brief The currently registered planner termination condition
  const ompl::base::PlannerTerminationCondition* ptc_;
  /// \brief Mutex around ptc_ for thread safety.
  boost::mutex ptc_lock_;

  /// \brief If true, the solution path will be interpolated (after
  /// simplification, if simplify_ is true).
  bool interpolate_;

  /// \brief If true, the solution path will be shortened after discovery.
  bool simplify_;

  ros::NodeHandle nh_;

  /// \brief True when the context is properly initialized
  bool initialized_;
};
}

#endif
