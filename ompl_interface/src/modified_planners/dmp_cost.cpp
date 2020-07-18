#include <moveit/ompl_interface/modified_planners/dmp_cost.h>

ompl_interface::DMPCost::DMPCost(dmp::GetDMPPlanResponse& template_path, robot_state::RobotState &robot, std::string group)
{
  robot_ = std::make_shared<robot_state::RobotState>(robot);
  template_path_ = toVector(template_path);
}

std::vector<std::vector<double>> ompl_interface::DMPCost::toVector(dmp::GetDMPPlanResponse &dmp_path)
{
  std::vector<std::vector<double>> traj;
  for (auto &point : dmp_path.plan.points)
  {
    std::vector<double> point_vec = point.positions;
    traj.push_back(point_vec);
  }
  return traj;
}

std::vector<std::vector<double>> ompl_interface::DMPCost::toCartesianPath(std::vector<std::vector<double>> joint_path)
{
  std::vector<std::vector<double>> cart_path;
  for (auto pt : joint_path)
  {
    robot_->setJointGroupPositions("arm_with_torso", pt);
    auto gripper_pose = robot_->getGlobalLinkTransform("gripper_link");

    std::vector<double> cart_pt;
    cart_pt.push_back(gripper_pose.translation().x());
    cart_pt.push_back(gripper_pose.translation().y());
    cart_pt.push_back(gripper_pose.translation().z());
    cart_pt.push_back(Eigen::Quaterniond(gripper_pose.rotation()).x());
    cart_pt.push_back(Eigen::Quaterniond(gripper_pose.rotation()).y());
    cart_pt.push_back(Eigen::Quaterniond(gripper_pose.rotation()).z());
    cart_pt.push_back(Eigen::Quaterniond(gripper_pose.rotation()).w());

    cart_path.push_back(cart_pt);
  }
  return cart_path;
}

double ompl_interface::DMPCost::getCost(dmp::GetDMPPlanResponse &dmp_path)
{
  auto dmp_path_vec = toVector(dmp_path);
  double total_cost;
  double euclidean_cost = euclideanDistance(dmp_path_vec);
  total_cost = euclidean_cost;
  return total_cost;
}

std::vector<std::vector<double>>  ompl_interface::DMPCost::equalizePaths(std::vector<std::vector<double>>& dmp_path)
{
  std::vector<std::vector<double>>::iterator it;
  std::vector<double> point(8,0.0);
  if (dmp_path.size() > template_path_.size())
  {
    it = template_path_.end();
    int diff = dmp_path.size() - template_path_.size();
    template_path_.insert(it, diff, point);
  }
  else if (dmp_path.size() < template_path_.size())
  {
    it = dmp_path.end();
    int diff = template_path_.size() - dmp_path.size();
    dmp_path.insert(it, diff, point);
  }
  return dmp_path;

}

double ompl_interface::DMPCost::euclideanDistance(std::vector<std::vector<double>>& dmp_path)
{
  dmp_path = equalizePaths(dmp_path);
  double total = 0.0;
  for (unsigned int i = 0; i < dmp_path.size(); i++)
  {
    total = total + pointDistance(dmp_path[i], template_path_[i]);
  }
  return total;
}

double ompl_interface::DMPCost::dtwDistance(std::vector<std::vector<double>>& dmp_path)
{

}

double ompl_interface::DMPCost::pointDistance(std::vector<double> &p1, std::vector<double> &p2)
{
  Eigen::Quaterniond q1(p1[6], p1[3], p1[4], p1[5]);
  Eigen::Quaterniond q2(p2[6], p2[3], p2[4], p2[5]);

  Eigen::Vector3d t1(p1[0], p1[1], p1[2]);
  Eigen::Vector3d t2(p2[0], p2[1], p2[2]);

  double angular_dist = q1.angularDistance(q2);
  double translation_dist = (t1-t2).squaredNorm();
  return 0.5*translation_dist + 0.5*angular_dist;
}

