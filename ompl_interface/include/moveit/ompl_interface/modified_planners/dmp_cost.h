#ifndef DMP_COST_H
#define DMP_COST_H

// DMP
#include <dmp/DMPData.h>
#include <dmp/DMPPoint.h>
#include <dmp/DMPTraj.h>
#include <dmp/GetDMPPlan.h>
#include <dmp/LearnDMPFromDemo.h>
#include <dmp/SetActiveDMP.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

namespace ompl_interface
{
class DMPCost
{
public:
  DMPCost(dmp::GetDMPPlanResponse& template_path, robot_state::RobotState &robot, std::string group);
  double getCost(dmp::GetDMPPlanResponse& path);
  double getCSpaceCost(dmp::GetDMPPlanResponse &dmp_path);

private:
  std::vector<std::vector<double>> equalizePaths(std::vector<std::vector<double>>& dmp_path);
  void downsample(std::vector<std::vector<double>> &dmp_path);
  double euclideanDistance(std::vector<std::vector<double>>& dmp_path);
  double pointDistance(std::vector<double> &p1, std::vector<double> &p2);
  double dtwDistance(std::vector<std::vector<double>>& dmp_path);
  std::vector<std::vector<double>> projectPath(std::vector<std::vector<double>> &dmp_path);
  std::vector<std::vector<double>> toCartesianPath(std::vector<std::vector<double>> joint_path);
  std::vector<std::vector<double>> toVector(dmp::GetDMPPlanResponse &dmp_path);
  std::vector<std::vector<double>> normalize(std::vector<std::vector<double>> &cart_path);

  std::vector<std::vector<double>> template_path_;
  robot_state::RobotStatePtr robot_;
};

}  // namespace ompl_interface

#endif

