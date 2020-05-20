/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Rice University
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

/* Author: Ioan Sucan */
/* Modified by: Juan David Hernandez Vega */

#ifndef OMPL_BASE_GOALS_WEIGHTED_GOAL_REGION_SAMPLER_
#define OMPL_BASE_GOALS_WEIGHTED_GOAL_REGION_SAMPLER_

#include <functional>
#include <limits>
#include <mutex>
#include <thread>

#include <ompl/base/State.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/datastructures/BinaryHeap.h>
#include <ompl/base/Planner.h>

#include <moveit/ompl_interface/modified_planners/PRMMod.h>
#include <moveit/ompl_interface/modified_planners/goal_regions_state_sampler.h>
namespace ompl
{
namespace base
{
class WeightedGoalRegionSampler;

/** \brief Goal sampling function. Returns false when no further calls should be made to it.
    Fills its second argument (the state) with the sampled goal state. This function need not
    be thread safe. */ 
typedef std::function<bool(const WeightedGoalRegionSampler*, std::vector<base::State*>&)> WeightedGoalRegionSamplingFn;

/** \brief Definition of a goal region that can be sampled,
 but the sampling process can be slow.  This class allows
 sampling the happen in a separate thread, and the number of
 goals may increase, as the planner is running, in a
 thread-safe manner.


 \todo The Python bindings for WeightedGoalRegionSampler class are still broken.
 The OMPL C++ code creates a new thread from which you should be able
 to call a python Goal sampling function. Acquiring the right threads
 and locks and messing around with the Python Global Interpreter Lock
 (GIL) is very tricky. See ompl/py-bindings/generate_bindings.py for
 an initial attempt to make this work.
 */
class WeightedGoalRegionSampler : public GoalStates
{
public:
  struct WeightedGoal;

  /** \brief When new samples are generated and added to the
      list of possible samples, a callback can be
      called. This type specifies the signature of that callback */
  typedef std::function<void(const base::State*)> NewStateCallbackFn;

  /** \brief Create a goal region that can be sampled in a
      lazy fashion. A function (\e samplerFunc) that
      produces samples from that region needs to be passed
      to this constructor. The sampling thread is
      automatically started if \e autoStart is true. The
      sampling function is not called in parallel by
      OMPL. Hence, the function is not required to be thread
      safe, unless the user issues additional calls in
      parallel. The instance of WeightedGoalRegionSampler remains
      thread safe however.

      The function \e samplerFunc returns a truth value. If
      the return value is true, further calls to the
      function can be made. If the return is false, no more
      calls should be made. The function takes two
      arguments: the instance of WeightedGoalRegionSampler making the
      call and the state to fill with a goal state. For
      every state filled in by \e samplerFunc,
      addStateIfDifferent() is called.  A state computed by
      the sampling thread is added if it is "sufficiently
      different" from previously added states. A state is
      considered "sufficiently different" if it is at least
      \e minDist away from previously added states.  */
  WeightedGoalRegionSampler(const SpaceInformationPtr& si, WeightedGoalRegionSamplingFn samplerFunc,
                            const bool use_max_sampled_goals = true, const unsigned int num_sampled_goals = 10,
                            bool autoStart = true, double minDist = std::numeric_limits<double>::epsilon());

  ~WeightedGoalRegionSampler() override;

  void sampleGoal(State* st) const override;

  double distanceGoal(const State* st) const override;

  void addState(const State* st) override;

  /** \brief Start the goal sampling thread */
  void startSampling();

  /** \brief Stop the goal sampling thread */
  void stopSampling();

  /** \brief Return true if the sampling thread is active */
  bool isSampling() const;

  /** \brief Start the roadmap growing thread */
  void startGrowingRoadmap();

  /** \brief Stop the roadmap growing thread */
  void stopGrowingRoadmap();

  /** \brief Return true if the growing thread is active */
  bool isGrowingRoadmap() const;

  void useMaxSampledGoals(bool use_max_sampled_goals);

  /** \brief Set the minimum distance that a new state returned by the sampling thread needs to be away from
      previously added states, so that it is added to the list of goal states. */
  void setMinNewSampleDistance(double dist)
  {
    minDist_ = dist;
  }

  /** \brief Get the minimum distance that a new state returned by the sampling thread needs to be away from
      previously added states, so that it is added to the list of goal states. */
  double getMinNewSampleDistance() const
  {
    return minDist_;
  }

  /** \brief The number of times the sampling function was called and it returned true */
  unsigned int samplingAttemptsCount() const
  {
    return samplingAttempts_;
  }

  /** \brief Set the callback function to be called when a new state is added to the list of possible samples.
     This function
      is not required to be thread safe, as calls are made one at a time. */
  void setNewStateCallback(const NewStateCallbackFn& callback);

  /** \brief Add a state \e st if it further away that \e minDistance from previously added states. Return
   * true if the state was added. */
  bool addStateIfDifferent(const State* st, double minDistance);

  /** \brief Return true if GoalStates::couldSample() is true or if the sampling thread is active, as in this
   * case it is possible a sample can be produced at some point. */
  bool couldSample() const override;

  bool hasStates() const override;
  const State* getState(unsigned int index) const override;
  std::size_t getStateCount() const override;
  unsigned long int getRoadmapEdgeCount() const;

  void penalizeWeightedGoal(WeightedGoal& weighted_goal);

  void rewardWeightedGoal(WeightedGoal& weighted_goal);

  void sampleWeightedGoal(WeightedGoal& weighted_goal);

  bool sampleGoals(const ompl::base::WeightedGoalRegionSampler* gls,
                           std::vector<ompl::base::State*>& sampled_states); 

  void sampleConsecutiveGoal(WeightedGoal& weighted_goal);

  bool addWeightedState(State* st, double weight);

  void clear() override;

  unsigned int maxSampleCount() const override;

  struct WeightedGoalCompare
  {
    bool operator()(WeightedGoal* goal1, WeightedGoal* goal2) const
    {
      // highest weight means highest score
      return goal1->weight_ > goal2->weight_;
    }
  };  // namespace base

  struct WeightedGoal
  {
  public:
    WeightedGoal()
    {
    }

    State* state_{ nullptr };
    double weight_{ 1.0 };
    ompl::BinaryHeap<WeightedGoal*, WeightedGoalCompare>::Element* heap_element_{ nullptr };
  };

  std::vector<const State*> sampled_goal_states_;

  /** \brief Lock for updating the set of states */
  mutable std::mutex lock_;

protected:
  /** \brief The function that samples goals by calling \e samplerFunc_ in a separate thread */
  void goalSamplingThread();

  /** \brief The function that grows the roadmap in a separate thread */
  void roadmapGrowingThread();

  /** \brief Function that produces samples */
  WeightedGoalRegionSamplingFn samplerFunc_;

  /** \brief Flag used to notify the sampling thread to terminate sampling */
  bool terminateSamplingThread_;

  /** \brief Flag used to notify the roadmap growing thread to terminate growing */
  bool terminateGrowingRoadmapThread_;

  /** \brief Additional thread for sampling goal states */
  std::thread* samplingThread_;

  /** \brief Additional thread for growing the roadmap of goal states */
  std::thread* growingRoadmapThread_;

  /** \brief The number of times the sampling function was called and it returned true */
  unsigned int samplingAttempts_;

  /** \brief Samples returned by the sampling thread are added to the list of states only if
      they are at least minDist_ away from already added samples. */
  double minDist_;

  /** \brief If defined, this function is called when a new state is added to the list of possible samples */
  NewStateCallbackFn callback_;

  // Maximum number of sampled goals per goal region
  unsigned int max_sampled_goals_;

  bool sample_goals_;

  // Maximum number of sampled goals per goal region
  unsigned int num_sampled_goals_;
  bool use_max_sampled_goals_;

  ompl::BinaryHeap<WeightedGoal*, WeightedGoalCompare> goals_priority_queue_;
  ompl::base::PlannerPtr prm_planner_;
};  // namespace ompl

//----------------------------------------
class RandomGoalRegionSampler;

/** \brief Goal sampling function. Returns false when no further calls should be made to it.
    Fills its second argument (the state) with the sampled goal state. This function need not
    be thread safe. */
typedef std::function<bool(const RandomGoalRegionSampler*, std::vector<base::State*>&)> RandomGoalRegionSamplingFn;

/** \brief Definition of a goal region that can be sampled,
 but the sampling process can be slow.  This class allows
 sampling the happen in a separate thread, and the number of
 goals may increase, as the planner is running, in a
 thread-safe manner.


 \todo The Python bindings for RandomGoalRegionSampler class are still broken.
 The OMPL C++ code creates a new thread from which you should be able
 to call a python Goal sampling function. Acquiring the right threads
 and locks and messing around with the Python Global Interpreter Lock
 (GIL) is very tricky. See ompl/py-bindings/generate_bindings.py for
 an initial attempt to make this work.
 */
class RandomGoalRegionSampler : public GoalStates
{
public:
  /** \brief When new samples are generated and added to the
      list of possible samples, a callback can be
      called. This type specifies the signature of that callback */
  typedef std::function<void(const base::State*)> NewStateCallbackFn;

  /** \brief Create a goal region that can be sampled in a
      lazy fashion. A function (\e samplerFunc) that
      produces samples from that region needs to be passed
      to this constructor. The sampling thread is
      automatically started if \e autoStart is true. The
      sampling function is not called in parallel by
      OMPL. Hence, the function is not required to be thread
      safe, unless the user issues additional calls in
      parallel. The instance of RandomGoalRegionSampler remains
      thread safe however.

      The function \e samplerFunc returns a truth value. If
      the return value is true, further calls to the
      function can be made. If the return is false, no more
      calls should be made. The function takes two
      arguments: the instance of RandomGoalRegionSampler making the
      call and the state to fill with a goal state. For
      every state filled in by \e samplerFunc,
      addStateIfDifferent() is called.  A state computed by
      the sampling thread is added if it is "sufficiently
      different" from previously added states. A state is
      considered "sufficiently different" if it is at least
      \e minDist away from previously added states.  */
  RandomGoalRegionSampler(const SpaceInformationPtr& si, RandomGoalRegionSamplingFn samplerFunc, bool autoStart = true,
                          double minDist = std::numeric_limits<double>::epsilon());

  ~RandomGoalRegionSampler() override;

  void sampleGoal(State* st) const override;

  double distanceGoal(const State* st) const override;

  void addState(const State* st) override;

  /** \brief Start the goal sampling thread */
  void startSampling();

  /** \brief Stop the goal sampling thread */
  void stopSampling();

  /** \brief Return true if the sampling thread is active */
  bool isSampling() const;

  /** \brief Set the minimum distance that a new state returned by the sampling thread needs to be away from
      previously added states, so that it is added to the list of goal states. */
  void setMinNewSampleDistance(double dist)
  {
    minDist_ = dist;
  }

  /** \brief Get the minimum distance that a new state returned by the sampling thread needs to be away from
      previously added states, so that it is added to the list of goal states. */
  double getMinNewSampleDistance() const
  {
    return minDist_;
  }

  /** \brief The number of times the sampling function was called and it returned true */
  unsigned int samplingAttemptsCount() const
  {
    return samplingAttempts_;
  }

  /** \brief Set the callback function to be called when a new state is added to the list of possible samples.
     This function
      is not required to be thread safe, as calls are made one at a time. */
  void setNewStateCallback(const NewStateCallbackFn& callback);

  /** \brief Add a state \e st if it further away that \e minDistance from previously added states. Return
   * true if the state was added. */
  bool addStateIfDifferent(const State* st, double minDistance);

  /** \brief Return true if GoalStates::couldSample() is true or if the sampling thread is active, as in this
   * case it is possible a sample can be produced at some point. */
  bool couldSample() const override;

  bool hasStates() const override;
  const State* getState(unsigned int index) const override;
  std::size_t getStateCount() const override;

  void clear() override;

  unsigned int maxSampleCount() const override;

  std::vector<const State*> sampled_goal_states_;

  /** \brief Lock for updating the set of states */
  mutable std::mutex lock_;

protected:
  /** \brief The function that samples goals by calling \e samplerFunc_ in a separate thread */
  void goalSamplingThread();

  /** \brief Function that produces samples */
  RandomGoalRegionSamplingFn samplerFunc_;

  /** \brief Flag used to notify the sampling thread to terminate sampling */
  bool terminateSamplingThread_;

  /** \brief Additional thread for sampling goal states */
  std::thread* samplingThread_;

  /** \brief The number of times the sampling function was called and it returned true */
  unsigned int samplingAttempts_;

  /** \brief Samples returned by the sampling thread are added to the list of states only if
      they are at least minDist_ away from already added samples. */
  double minDist_;

  /** \brief If defined, this function is called when a new state is added to the list of possible samples */
  NewStateCallbackFn callback_;

  bool sample_goals_;

  // Maximum number of sampled goals per goal region
  unsigned int num_sampled_goals_;
};

}  // namespace base
}  // namespace ompl

#endif
