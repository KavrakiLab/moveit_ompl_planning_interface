#ifndef DMP_COST_H
#define DMP_COST_H

// DMP
#include <dmp/DMPData.h>
#include <dmp/DMPPoint.h>
#include <dmp/DMPTraj.h>
#include <dmp/GetDMPPlan.h>
#include <dmp/LearnDMPFromDemo.h>
#include <dmp/SetActiveDMP.h>

namespace ompl_interface
{
class DMPCost
{
public:
  DMPCost(dmp::GetDMPPlanResponse& template_path);
  double getCost(dmp::GetDMPPlanResponse& path);

private:
  dmp::GetDMPPlanResponse equalizePaths(dmp::GetDMPPlanResponse& path);
  double euclideanDistance(dmp::GetDMPPlanResponse& path);
  double dtwDistance(dmp::GetDMPPlanResponse& path);
  double clearance(dmp::GetDMPPlanResponse& path);

  dmp::GetDMPPlanResponse template_path_;
};

}  // namespace ompl_interface

#endif

