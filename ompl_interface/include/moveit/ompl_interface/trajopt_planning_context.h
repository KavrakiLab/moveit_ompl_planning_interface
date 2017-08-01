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

/* Author: Bryce Willey */

#ifndef MOVEIT_OMPL_INTERFACE_TRAJOPT_PLANNING_CONTEXT_
#define MOVEIT_OMPL_INTERFACE_TRAJOPT_PLANNING_CONTEXT_

#include <ros/ros.h>
#include "moveit/ompl_interface/ompl_planning_context.h"
#include "moveit/ompl_interface/geometric_planning_context.h"
#include <boost/thread/mutex.hpp>
#include <ompl/geometric/SimpleSetup.h>

namespace ompl_interface
{

class MoveItApiWrapper
{
public:
    MoveItApiWrapper(robot_state::RobotStatePtr kinematic_state,
                    const moveit::core::JointModelGroup * jointModelGroup,
                    const planning_scene::PlanningSceneConstPtr planningScene) :
            kinematic_state_(kinematic_state), joint_model_group_(jointModelGroup),
            planning_scene_(planningScene)
    {}

    Eigen::MatrixXd jacobianAtPoint(std::vector<double> configuration,
                               Eigen::Vector3d point,
                               std::string link_name);

    bool extraCollisionInformation(std::vector<double> configuration,
                                   double& signedDist,
                                   Eigen::Vector3d& point,
                                   std::string& link_name,
                                   Eigen::Vector3d& normal);

private:
    robot_state::RobotStatePtr kinematic_state_;
    const moveit::core::JointModelGroup *joint_model_group_;
    const planning_scene::PlanningSceneConstPtr planning_scene_;
};

/// \brief Definition of a TrajOpt planning context.  This context plans in
/// the space of joint angles for a given group.  This context is NOT thread
/// safe.
class TrajOptPlanningContext : public GeometricPlanningContext
{
public:
  TrajOptPlanningContext();
  virtual ~TrajOptPlanningContext() {}
  virtual std::string getDescription();

  // TODO: this might call GeometricPlanningContext's constructor? IDK how exactly it works.
  virtual void initialize(
      const std::string& ros_namespace,
      PlanningContextSpecification& spec);

protected:
  /// \brief The solve method that actually does all of the solving
  /// Solve the problem \e count times or until \e timeout seconds elapse.
  /// The total time taken by this call is returned in \e total_time.
  virtual bool solve(double timeout, unsigned int count, double& total_time) override;
};
}

#endif
