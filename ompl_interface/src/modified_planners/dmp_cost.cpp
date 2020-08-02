#include <moveit/ompl_interface/modified_planners/dmp_cost.h>
#include <fast_dtw/DTW.h>
#include <fast_dtw/FastDTW.h>
#include <fast_dtw/EuclideanDistance.h>
#include <fast_dtw/SE3Distance.h>
#include <fast_dtw/SearchWindow.h>
#include <algorithm>

ompl_interface::DMPCost::DMPCost(dmp::GetDMPPlanResponse& template_path, robot_state::RobotState &robot, std::string group)
{
  robot_ = std::make_shared<robot_state::RobotState>(robot);
  template_path_ = toVector(template_path);
  downsample(template_path_);
  //template_path_ = toCartesianPath(template_path_);
  template_path_ = projectPath(template_path_);
  template_path_ = normalize(template_path_);
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

std::vector<std::vector<double>> ompl_interface::DMPCost::toVectorAvoidObstacles(dmp::GetDMPPlanAvoidObstaclesResponse &dmp_path)
{
  std::vector<std::vector<double>> traj;
  for (auto &point : dmp_path.plan.points)
  {
    std::vector<double> point_vec = point.positions;
    traj.push_back(point_vec);
  }
  return traj;
}

void ompl_interface::DMPCost::downsample(std::vector<std::vector<double>> &dmp_path)
{
  for (int i=0; i < dmp_path.size(); i=i+2)
    dmp_path.erase(dmp_path.begin() + i);
  for (int i=0; i < dmp_path.size(); i=i+2)
    dmp_path.erase(dmp_path.begin() + i);
  for (int i=0; i < dmp_path.size(); i=i+2)
    dmp_path.erase(dmp_path.begin() + i);
}

std::vector<std::vector<double>> ompl_interface::DMPCost::normalize(std::vector<std::vector<double>> &cart_path)
{
  std::vector<std::vector<double>> outpath = cart_path;
  for (int d=0; d<2; d++)
  {
    std::vector<double> dim;
    for (int n=0; n < cart_path.size(); n++)
      dim.push_back(cart_path[n][d]);

    const auto [min, max] = std::minmax_element(dim.begin(), dim.end());
    for (auto& e : dim)
      e = (e - *min) / (*max-*min);

    for (int n=0; n < cart_path.size(); n++)
    {
      outpath[n][d] = dim[d];
    }
  }
  return outpath;
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

    Eigen::Quaterniond q(gripper_pose.linear());
    q.normalize();
    cart_pt.push_back(q.x());
    cart_pt.push_back(q.y());
    cart_pt.push_back(q.z());
    cart_pt.push_back(q.w());

    cart_path.push_back(cart_pt);
  }
  return cart_path;
}

double ompl_interface::DMPCost::getCost(dmp::GetDMPPlanAvoidObstaclesResponse &dmp_path)
{
  std::vector<std::vector<double>> dmp_path_vec = toVectorAvoidObstacles(dmp_path);
  downsample(dmp_path_vec);
  //dmp_path_vec = toCartesianPath(dmp_path_vec);
  dmp_path_vec = projectPath(dmp_path_vec);
  dmp_path_vec = normalize(dmp_path_vec);

  fastdtw::TimeSeries<double, 6> ts1;
  fastdtw::TimeSeries<double, 6> ts2;

  for (int point = 0; point < dmp_path_vec.size(); point++)
  {
    ts1.addLast(point, fastdtw::TimeSeriesPoint<double, 6>(dmp_path_vec[point].data()));
  }

  for (int point = 0; point < template_path_.size(); point++)
  {
    ts2.addLast(point, fastdtw::TimeSeriesPoint<double, 6>(template_path_[point].data()));
  }
  fastdtw::TimeWarpInfo<double> info =  fastdtw::STRI::getWarpInfoBetween(ts1,ts2, fastdtw::EuclideanDistance());
  return info.getDistance();
}

double ompl_interface::DMPCost::getCSpaceCost(dmp::GetDMPPlanResponse &dmp_path)
{
  std::vector<std::vector<double>> dmp_path_vec = toVector(dmp_path);
  dmp_path_vec = normalize(dmp_path_vec);
  double total_cost = 0;

  fastdtw::TimeSeries<double, 8> ts1;
  fastdtw::TimeSeries<double, 8> ts2;

  for (int point = 0; point < dmp_path_vec.size(); point++)
  {
    ts1.addLast(point, fastdtw::TimeSeriesPoint<double, 8>(dmp_path_vec[point].data()));
  }
  for (int point = 0; point < template_path_.size(); point++)
  {
    ts2.addLast(point, fastdtw::TimeSeriesPoint<double, 8>(template_path_[point].data()));
  }
  fastdtw::TimeWarpInfo<double> info =  fastdtw::FAST::getWarpInfoBetween(ts1,ts2, fastdtw::EuclideanDistance());
  return info.getDistance();
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
  ROS_INFO("Inside DTW Distance: %d", dmp_path.size());
  if (dmp_path.size() == 0)
  {
    ROS_ERROR("DMP path empty!");
    exit(1);
  }
  fastdtw::TimeSeries<double, 3> ts1;
  fastdtw::TimeSeries<double, 3> ts2;
  ROS_INFO("Initialized TSs");
  //int ii=0;
  for (int i=0; i < dmp_path.size(); i++) // Need to fix this by downsampling
  {
    ts1.addLast(i, fastdtw::TimeSeriesPoint<double, 3>(dmp_path[i].data()));
  }
  for (int i=0; i<template_path_.size(); i++)
  {
    ts2.addLast(i, fastdtw::TimeSeriesPoint<double, 3>(template_path_[i].data()));
  }

  //fastdtw::SearchWindow sw(200, 200);
  int search_radius = 10; // Default is one
  ROS_INFO("About to call warp");
  fastdtw::TimeWarpInfo<double> info =  fastdtw::FAST::getWarpInfoBetween(ts1,ts2,search_radius,fastdtw::SE3Distance());
  ROS_INFO("Called Warp");
  return info.getDistance();
}

std::vector<std::vector<double>> ompl_interface::DMPCost::projectPath(std::vector<std::vector<double>> &dmp_path)
{
  std::vector<std::vector<double>> new_path;

  auto compare = dmp_path.back();
  Eigen::Vector2d p2(compare[0], compare[1]);
  //Eigen::Quaterniond q2(compare[6], compare[3], compare[4], compare[5]);
  //Eigen::Quaterniond q2(1, 0, 0, 0);
  double h2 = compare[2];
  for (auto &point : dmp_path)
  {
    std::vector<double> new_point; // r, height, angular distance from end
    Eigen::Vector2d p1(point[0], point[1]);
    double h1 = point[2];
    double r = (p1-p2).squaredNorm();
    double h = h1-h2;

    Eigen::Quaterniond q1(point[6], point[3], point[4], point[5]);
    //double theta = q1.angularDistance(q2);

    new_point.push_back(r);
    new_point.push_back(h);
    new_point.push_back(q1.x());
    new_point.push_back(q1.y());
    new_point.push_back(q1.z());
    new_point.push_back(q1.w());
    new_path.push_back(new_point);
  }
  return new_path;
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

