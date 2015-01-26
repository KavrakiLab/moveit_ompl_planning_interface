/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Rice University
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

#ifndef MOVEIT_OMPL_INTERFACE_OMPL_PLANNING_CONTEXT_
#define MOVEIT_OMPL_INTERFACE_OMPL_PLANNING_CONTEXT_

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/constraint_samplers/constraint_sampler_manager.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include "moveit_ompl_planning_interface/ModelBasedStateSpace.h"

namespace moveit_ompl_interface
{

struct PlanningContextSpecification
{
    std::string name;                           // name of the context configuration
    std::string group;                          // name of the group to plan for
    std::string planner;                        // id of the planner to use
    std::map<std::string, std::string> config;  // planning context parameters

    bool simplify_solution;                     // If true, solution path should be simplified
    bool interpolate_solution;                  // If true, solution path should contain a minimum number of waypoints (after simplification)
    unsigned int min_waypoint_count;            // The minimum number of waypoints (for interpolation)
    double max_waypoint_distance;               // The maximum allowed distance between waypoints (for interpolation)
    unsigned int max_num_threads;               // The maximum number of threads allowed for calls to solve()

    robot_model::RobotModelConstPtr model;      // the robot model
    constraint_samplers::ConstraintSamplerManagerPtr constraint_sampler_mgr; // Constraint sampler loaders
};

/// \brief Definition of an OMPL-specific planning context.  This context is
/// responsible for setting up and executing the OMPL planning environment
class OMPLPlanningContext : public planning_interface::PlanningContext
{
public:
    OMPLPlanningContext() : planning_interface::PlanningContext("UNINITIALIZED", "NO_GROUP") {}

    virtual ~OMPLPlanningContext() {}

    /// \brief Return a unique identification string for this context
    virtual std::string getDescription() = 0;

    /// \brief Initialize this planning context.  Give the context a name and
    /// the name of a group to plan for
    virtual void initialize(const PlanningContextSpecification& spec)
    {
        name_ = spec.name;
        group_ = spec.group;
    }

    /// \brief Solve the motion planning problem and store the result in \e res.
    /// This function should not clear data structures before computing. The constructor
    /// and clear() do that.
    virtual bool solve(planning_interface::MotionPlanResponse &res) = 0;

    /// \brief Solve the motion planning problem and store the detailed result in \e res.
    /// This function should not clear data structures before computing. The constructor
    /// and clear() do that.
    virtual bool solve(planning_interface::MotionPlanDetailedResponse &res) = 0;

    /// \brief If solve() is running, terminate the computation. Return false if termination
    /// not possible. No-op if solve() is not running (returns true).
    virtual bool terminate() = 0;

    /// \brief Clear the data structures used by the planner
    virtual void clear() = 0;

    /// \brief Return a pointer to the OMPL state space
    virtual const ModelBasedStateSpacePtr& getOMPLStateSpace() const = 0;

    /// \brief Return a pointer to the OMPL space information object
    virtual const ompl::base::SpaceInformationPtr& getOMPLSpaceInformation() const = 0;

    /// \brief Return a pointer to the OMPL problem definition object
    virtual const ompl::base::ProblemDefinitionPtr& getOMPLProblemDefinition() const = 0;

    /// \brief Return a reference to the initial state of the robot
    virtual const robot_state::RobotState& getCompleteInitialRobotState() const = 0;

    /// \brief Set the values of the initial state of the robot
    virtual void setCompleteInitialRobotState(const robot_state::RobotStatePtr& state) = 0;

    /// \brief Set the goal state of the robot via a set of constraints
    virtual bool setGoalConstraints(const std::vector<moveit_msgs::Constraints> &goal_constraints,
                                    moveit_msgs::MoveItErrorCodes *error) = 0;
};

}

#endif