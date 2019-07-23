/* Author: Juan D. Hernández Vega */

#ifndef OMPL_BASE_GOAL_REGIONS_STATE_SAMPLER_
#define OMPL_BASE_GOAL_REGIONS_STATE_SAMPLER_

#include <functional>

#include <ompl/base/StateSpace.h>
#include <boost/thread/mutex.hpp>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/planners/PlannerIncludes.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include <moveit/ompl_interface/modified_planners/weighted_goal_region_sampler.h>

// Eigen
#include <Eigen/Dense>

// OMPL namespaces
namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;

namespace ompl
{
namespace base
{
class WeightedGoalRegionSampler;

ob::StateSamplerPtr newAllocStateSampler(const ob::StateSpace* space,
                                         const ob::WeightedGoalRegionSampler* weighted_goal_region_sampler);

class GoalRegionsStateSampler;
/** \brief Goal sampling function. Returns false when no further calls should be made to it.
            Fills its second argument (the state) with the sampled goal state. This function need
   not be thread safe. */
typedef std::function<bool(const GoalRegionsStateSampler*, std::vector<base::State*>&)> GoalRegionsSamplingFn;

/** \brief Extended state sampler to use with the CForest planning algorithm. It wraps the
   user-specified state sampler.*/
class GoalRegionsStateSampler : public ob::StateSampler
{
public:
  /** \brief Constructor */
  GoalRegionsStateSampler(const ob::StateSpace* space, ob::StateSamplerPtr sampler,
                          const ob::WeightedGoalRegionSampler* weighted_goal_region_sampler)
    : ob::StateSampler(space)
    , sampler_(sampler)
    , sampled_goal_states_index_(0)
    , weighted_goal_region_sampler_(weighted_goal_region_sampler)
  {
  }

  /** \brief Destructor */
  ~GoalRegionsStateSampler()
  {
    clear();
  }

  /** \brief It will sample the next state of the vector StatesToSample_. If this is empty,
      it will call the sampleUniform() method of the specified sampler. */
  virtual void sampleUniform(ob::State* state);

  /** \brief It will sample the next state of the vector StatesToSample_. If this is empty,
      it will call the sampleUniformNear() method of the specified sampler. */
  virtual void sampleUniformNear(ob::State* state, const ob::State* near, const double distance);

  /** \brief It will sample the next state of the vector StatesToSample_. If this is empty,
      it will call the sampleGaussian() method of the specified sampler. */
  virtual void sampleGaussian(ob::State* state, const ob::State* mean, const double stdDev);

  const ob::StateSpace* getStateSpace() const
  {
    return space_;
  }

  void clear();

protected:
  /** \brief Extracts the next sample when statesToSample_ is not empty. */
  void getNextSample(ob::State* state);

  /** \brief States to be sampled */
  std::vector<ob::State*> statesToSample_;

  /** \brief Underlying, user-specified state sampler. */
  ob::StateSamplerPtr sampler_;

  /** \brief Index to keep track of the goal states that has been sampled. */
  unsigned int sampled_goal_states_index_;

  /** \brief Pointer to the weighted_goal_region_sampler that contains the goal states. */
  const ob::WeightedGoalRegionSampler* weighted_goal_region_sampler_;
};
};  // namespace base
};  // namespace ompl

#endif
