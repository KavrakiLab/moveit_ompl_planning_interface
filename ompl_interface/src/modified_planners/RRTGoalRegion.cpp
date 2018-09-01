/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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

#include <moveit/ompl_interface/modified_planners/RRTGoalRegion.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/tools/config/SelfConfig.h>
#include <limits>

ompl::geometric::RRTGoalRegion::RRTGoalRegion(const base::SpaceInformationPtr& si) : base::Planner(si, "RRTGoalRegion")
{
  specs_.approximateSolutions = true;
  specs_.directed = true;

  Planner::declareParam<double>("range", this, &RRTGoalRegion::setRange, &RRTGoalRegion::getRange, "0.:1.:"
                                                                                                   "10000.");
  Planner::declareParam<double>("goal_bias", this, &RRTGoalRegion::setGoalBias, &RRTGoalRegion::getGoalBias, "0.:.05:"
                                                                                                             "1.");
}

ompl::geometric::RRTGoalRegion::~RRTGoalRegion()
{
  freeMemory();
}

void ompl::geometric::RRTGoalRegion::clear()
{
  Planner::clear();
  sampler_.reset();
  freeMemory();
  if (nn_)
    nn_->clear();
  lastGoalMotion_ = nullptr;
}

void ompl::geometric::RRTGoalRegion::setup()
{
  Planner::setup();
  tools::SelfConfig sc(si_, getName());
  sc.configurePlannerRange(maxDistance_);

  if (!nn_)
    nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(this));
  nn_->setDistanceFunction([this](const Motion* a, const Motion* b) { return distanceFunction(a, b); });
}

void ompl::geometric::RRTGoalRegion::freeMemory()
{
  if (nn_)
  {
    std::vector<Motion*> motions;
    nn_->list(motions);
    for (auto& motion : motions)
    {
      if (motion->state != nullptr)
        si_->freeState(motion->state);
      delete motion;
    }
  }
}

ompl::base::PlannerStatus ompl::geometric::RRTGoalRegion::solve(const base::PlannerTerminationCondition& ptc)
{
  checkValidity();
  base::Goal* goal = pdef_->getGoal().get();
  auto* goal_region = dynamic_cast<base::WeightedGoalRegionSamples*>(goal);

  while (const base::State* st = pis_.nextStart())
  {
    auto* motion = new Motion(si_);
    si_->copyState(motion->state, st);
    nn_->add(motion);
  }

  if (nn_->size() == 0)
  {
    OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
    return base::PlannerStatus::INVALID_START;
  }

  if (!sampler_)
    sampler_ = si_->allocStateSampler();

  OMPL_INFORM("%s: Starting planning with %u states already in data structure", getName().c_str(), nn_->size());

  Motion* solution = nullptr;
  Motion* approxsol = nullptr;
  double approxdif = std::numeric_limits<double>::infinity();
  auto* rmotion = new Motion(si_);
  base::State* rstate = rmotion->state;
  base::State* xstate = si_->allocState();

  base::WeightedGoalRegionSamples::WeightedGoal weighted_goal;
  bool expansion_toward_goal;

  // maxDistance_ = 3.0;
  //goalBias_ = 0.5;

  while (!ptc)
  {
    expansion_toward_goal = false;
    /* sample random state (with goal biasing) */
    if ((goal_region != nullptr) && rng_.uniform01() < goalBias_ && goal_region->canSample())
    {
      expansion_toward_goal = true;

      weighted_goal.state_ = rstate;
      goal_region->sampleWeightedGoal(weighted_goal);

      //      std::cout << "goal" << std::endl;
      //      si_->printState(rstate);
    }
    else
    {
      sampler_->sampleUniform(rstate);

      //      std::cout << "random" << std::endl;
      //      si_->printState(rstate);
    }

    /* find closest state in the tree */
    Motion* nmotion = nn_->nearest(rmotion);
    base::State* dstate = rstate;

    /* find state to add */
    double d = si_->distance(nmotion->state, rstate);
    if (d > maxDistance_)
    {
      si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
      dstate = xstate;
    }

    std::pair<base::State*, double> last_valid;

    bool expansion_result = si_->checkMotion(nmotion->state, dstate, last_valid);

    if (expansion_toward_goal && !expansion_result)
    {
      if (last_valid.second > 0.0)
      {
        // std::cout << "Expanding towards the goal: should NOT penalize!! " << std::endl;
        goal_region->rewardWeightedGoal(weighted_goal);

        si_->getStateSpace()->interpolate(nmotion->state, dstate, last_valid.second, xstate);
        dstate = xstate;

        /* create a motion */
        auto* motion = new Motion(si_);
        si_->copyState(motion->state, dstate);
        motion->parent = nmotion;

        nn_->add(motion);
        double dist = 0.0;
        bool sat = goal->isSatisfied(motion->state, &dist);
        if (sat)
        {
          approxdif = dist;
          solution = motion;

          break;
        }
        if (dist < approxdif)
        {
          approxdif = dist;
          approxsol = motion;
        }
      }
      else
      {
        // std::cout << "Expanding towards the goal: should penalize " << std::endl;
        goal_region->penalizeWeightedGoal(weighted_goal);
      }
    }
    else if (expansion_result)
    {
      if (expansion_toward_goal)
      {
        // std::cout << "Expanding towards the goal: should NOT penalize " << std::endl;
        goal_region->rewardWeightedGoal(weighted_goal);
      }

      /* create a motion */
      auto* motion = new Motion(si_);
      si_->copyState(motion->state, dstate);
      motion->parent = nmotion;

      nn_->add(motion);
      double dist = 0.0;
      bool sat = goal->isSatisfied(motion->state, &dist);
      if (sat)
      {
        approxdif = dist;
        solution = motion;
        break;
      }
      if (dist < approxdif)
      {
        approxdif = dist;
        approxsol = motion;
      }
    }
  }

  bool solved = false;
  bool approximate = false;
  if (solution == nullptr)
  {
    solution = approxsol;
    approximate = true;
  }

  if (solution != nullptr)
  {
    lastGoalMotion_ = solution;

    /* construct the solution path */
    std::vector<Motion*> mpath;
    while (solution != nullptr)
    {
      mpath.push_back(solution);
      solution = solution->parent;
    }

    /* set the solution path */
    auto path(std::make_shared<PathGeometric>(si_));
    for (int i = mpath.size() - 1; i >= 0; --i)
      path->append(mpath[i]->state);
    pdef_->addSolutionPath(path, approximate, approxdif, getName());
    solved = true;
  }

  si_->freeState(xstate);
  if (rmotion->state != nullptr)
    si_->freeState(rmotion->state);
  delete rmotion;

  OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

  return base::PlannerStatus(solved, approximate);
}

void ompl::geometric::RRTGoalRegion::getPlannerData(base::PlannerData& data) const
{
  Planner::getPlannerData(data);

  std::vector<Motion*> motions;
  if (nn_)
    nn_->list(motions);

  if (lastGoalMotion_ != nullptr)
    data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

  for (auto& motion : motions)
  {
    if (motion->parent == nullptr)
      data.addStartVertex(base::PlannerDataVertex(motion->state));
    else
      data.addEdge(base::PlannerDataVertex(motion->parent->state), base::PlannerDataVertex(motion->state));
  }
}
