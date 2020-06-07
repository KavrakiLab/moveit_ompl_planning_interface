#include <moveit/ompl_interface/modified_planners/dmp_cost.h>

ompl_interface::DMPCost::DMPCost(dmp::GetDMPPlanResponse& template_path)
{
  template_path_ = template_path;
}

double ompl_interface::DMPCost::getCost(dmp::GetDMPPlanResponse& path)
{
  double total_cost;
  double euclidean_cost = euclideanDistance(path);
  total_cost = (0.02)*euclidean_cost;
  return total_cost;
}

dmp::GetDMPPlanResponse ompl_interface::DMPCost::equalizePaths(dmp::GetDMPPlanResponse& path)
{
  std::vector<dmp::DMPPoint>::iterator it;
  dmp::DMPPoint point;
  int dim = path.plan.points[0].positions.size();
  point.positions = std::vector<double>(dim, 0.0);
  point.velocities = std::vector<double>(dim, 0.0);

  if (path.plan.points.size() > template_path_.plan.points.size())
  {
    it = template_path_.plan.points.end();
    int diff = path.plan.points.size() - template_path_.plan.points.size();
    template_path_.plan.points.insert(it, diff, point);
    return path;
  }
  else if (path.plan.points.size() < template_path_.plan.points.size())
  {
    it = path.plan.points.end();
    int diff = template_path_.plan.points.size() - path.plan.points.size();
    path.plan.points.insert(it, diff, point);
  }
  return path;
}

double ompl_interface::DMPCost::euclideanDistance(dmp::GetDMPPlanResponse& path)
{
  path = equalizePaths(path);
  double total = 0.0;
  for (unsigned int i = 0; i < path.plan.points.size(); i++)
  {
    for (unsigned int d = 0; d < path.plan.points[0].positions.size(); d++)
    {
      total = total + pow((path.plan.points[i].positions[d] - template_path_.plan.points[i].positions[d]), 2);
      total = total + pow((path.plan.points[i].velocities[d] - template_path_.plan.points[i].velocities[d]), 2);
    }
  }
  return sqrt(total);
}

double ompl_interface::DMPCost::dtwDistance(dmp::GetDMPPlanResponse& path)
{
  return 0;
}

double ompl_interface::DMPCost::clearance(dmp::GetDMPPlanResponse& path)
{
  return 0;
}
