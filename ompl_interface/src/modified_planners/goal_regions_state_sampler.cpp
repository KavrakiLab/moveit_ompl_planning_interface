/* Author: Juan D. Hernández Vega */

#include <moveit/ompl_interface/modified_planners/goal_regions_state_sampler.h>

ob::StateSamplerPtr ob::newAllocGoalRegionStateSampler(
    const ob::StateSpace* space, const ob::WeightedGoalRegionSampler* weighted_goal_region_sampler)
{
  ob::GoalRegionsStateSampler* sampler =
      new ob::GoalRegionsStateSampler(space, space->allocDefaultStateSampler(), weighted_goal_region_sampler);
  // std::cout << "start_states.size(): " << start_states.size() << std::endl;
  // std::cout << "bias: " << planner->as<og::RRTstarMod>()->getGoalBias() << std::endl;
  // sampler->setStatesToSample(start_states);
  return ob::StateSamplerPtr(sampler);
}

void ob::GoalRegionsStateSampler::sampleUniform(ob::State* state)
{
  while (true)
  {
    {
      if (!weighted_goal_region_sampler_->isGrowingRoadmap())
        break;
      std::lock_guard<std::mutex> slock(weighted_goal_region_sampler_->lock_);
      if (weighted_goal_region_sampler_->sampled_goal_states_.size() > sampled_goal_states_index_)
      {
        //        std::cout << "***sampled_goal_states_.size(): " <<
        //        weighted_goal_region_sampler_->sampled_goal_states_.size()
        //                  << std::endl;
        //        std::cout << "+++sampled_goal_states_index: " << sampled_goal_states_index_ << std::endl;
        getNextSample(state);
        break;
      }
    }
  }
}

void ob::GoalRegionsStateSampler::sampleUniformNear(ob::State* state, const ob::State* near, const double distance)
{
  if (!statesToSample_.empty())
    getNextSample(state);
  else
    sampler_->sampleUniformNear(state, near, distance);
}

void ob::GoalRegionsStateSampler::sampleGaussian(ob::State* state, const ob::State* mean, const double stdDev)
{
  if (!statesToSample_.empty())
    getNextSample(state);
  else
    sampler_->sampleGaussian(state, mean, stdDev);
}

void ob::GoalRegionsStateSampler::getNextSample(ob::State* state)
{
  space_->copyState(state, weighted_goal_region_sampler_->sampled_goal_states_[sampled_goal_states_index_]);
  sampled_goal_states_index_++;
}

void ob::GoalRegionsStateSampler::clear()
{
  for (size_t i = 0; i < statesToSample_.size(); ++i)
    space_->freeState(statesToSample_[i]);
  statesToSample_.clear();
  sampler_.reset();
}
