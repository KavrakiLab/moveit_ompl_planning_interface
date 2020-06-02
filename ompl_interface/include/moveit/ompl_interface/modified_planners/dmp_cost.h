#ifndef DMP_COST_H
#define DMP_COST_H


#include <ompl/base/State.h>
#include <ompl/base/Cost.h>
#include <ompl/base/OptimizationObjective.h>


class DMPCost : public ompl::base::OptimizationObjective
{
public:
    DMPCost(const ompl::base::SpaceInformationPtr &si) :
        ompl::base::OptimizationObjective(si) {}

    virtual ompl::base::Cost stateCost(const ompl::base::State* s) const;
    virtual ompl::base::Cost motionCost(const ompl::base::State *s1, const ompl::base::State *s2) const;  // This is the main bad boy
    virtual ompl::base::Cost combineCosts(ompl::base::Cost c1, ompl::base::Cost c2) const;
    virtual ompl::base::Cost identityCost() const;
    virtual ompl::base::Cost infiniteCost() const;
};


#endif

