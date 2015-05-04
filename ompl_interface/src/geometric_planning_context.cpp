/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Rice University
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

/* Author: Ryan Luna */

#include "moveit/ompl_interface/geometric_planning_context.h"
#include "moveit/ompl_interface/parameterization/joint_space/joint_model_state_space.h"
#include "moveit/ompl_interface/parameterization/work_space/pose_model_state_space.h"
#include "moveit/ompl_interface/detail/state_validity_checker.h"
#include "moveit/ompl_interface/detail/projection_evaluators.h"
#include "moveit/ompl_interface/detail/constrained_goal_sampler.h"
#include "moveit/ompl_interface/detail/goal_union.h"
#include "moveit/ompl_interface/detail/constrained_sampler.h"

#include <pluginlib/class_loader.h>
#include <moveit/kinematic_constraints/utils.h>

#include <ompl/tools/multiplan/ParallelPlan.h>
#include <ompl/tools/config/SelfConfig.h>

#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/pRRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/sbl/pSBL.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>

namespace og = ompl::geometric;

using namespace ompl_interface;

GeometricPlanningContext::GeometricPlanningContext() : OMPLPlanningContext()
{
    initializePlannerAllocators();
    complete_initial_robot_state_ = NULL;

    // Interpolate the final solution path to have a minimum number of states
    interpolate_ = true;

    // Attempt to smooth the final solution path
    simplify_ = true;

    planner_id_ = "";
}

GeometricPlanningContext::~GeometricPlanningContext()
{
    if (complete_initial_robot_state_)
        delete complete_initial_robot_state_;
}

std::string GeometricPlanningContext::getDescription()
{
    return "OMPL Geometric Planning";
}

template<typename T>
static ompl::base::PlannerPtr allocatePlanner(const ompl::base::SpaceInformationPtr &si,
                                              const std::string &new_name, const std::map<std::string, std::string>& params)
{
  ompl::base::PlannerPtr planner(new T(si));
  if (!new_name.empty())
    planner->setName(new_name);
  planner->params().setParams(params, true);
  return planner;
}

void GeometricPlanningContext::initializePlannerAllocators()
{
    registerPlannerAllocator("geometric::RRT", boost::bind(&allocatePlanner<og::RRT>, _1, _2, _3));
    registerPlannerAllocator("geometric::RRTConnect", boost::bind(&allocatePlanner<og::RRTConnect>, _1, _2, _3));
    registerPlannerAllocator("geometric::LazyRRT", boost::bind(&allocatePlanner<og::LazyRRT>, _1, _2, _3));
    registerPlannerAllocator("geometric::TRRT", boost::bind(&allocatePlanner<og::TRRT>, _1, _2, _3));
    registerPlannerAllocator("geometric::EST", boost::bind(&allocatePlanner<og::EST>, _1, _2, _3));
    registerPlannerAllocator("geometric::SBL", boost::bind(&allocatePlanner<og::SBL>, _1, _2, _3));
    registerPlannerAllocator("geometric::KPIECE", boost::bind(&allocatePlanner<og::KPIECE1>, _1, _2, _3));
    registerPlannerAllocator("geometric::BKPIECE", boost::bind(&allocatePlanner<og::BKPIECE1>, _1, _2, _3));
    registerPlannerAllocator("geometric::LBKPIECE", boost::bind(&allocatePlanner<og::LBKPIECE1>, _1, _2, _3));
    registerPlannerAllocator("geometric::RRTstar", boost::bind(&allocatePlanner<og::RRTstar>, _1, _2, _3));
    registerPlannerAllocator("geometric::PRM", boost::bind(&allocatePlanner<og::PRM>, _1, _2, _3));
    registerPlannerAllocator("geometric::PRMstar", boost::bind(&allocatePlanner<og::PRMstar>, _1, _2, _3));
}

void GeometricPlanningContext::registerPlannerAllocator(const std::string &planner_id, const PlannerAllocator &pa)
{
    planner_allocators_[planner_id] = pa;
}

ConstraintsLibraryPtr GeometricPlanningContext::getConstraintsLibrary() const
{
    return constraints_library_;
}

void GeometricPlanningContext::initialize(const std::string& ros_namespace, const PlanningContextSpecification& spec)
{
    nh_ = ros::NodeHandle(ros_namespace);
    spec_ = spec;

    simplify_ = spec.simplify_solution;
    interpolate_ = spec.interpolate_solution;

    // Erase the type and plugin fields from the configuration items
    std::map<std::string, std::string>::iterator it = spec_.config.find("type");
    if (it != spec_.config.end())
    {
        planner_id_ = it->second;
        spec_.config.erase(it);
    }
    else
        ROS_WARN("No planner type specified.  Using default planner configuration");

    it = spec_.config.find("plugin");
    if (it != spec_.config.end())
        spec_.config.erase(it);

    OMPLPlanningContext::initialize(ros_namespace, spec_);

    constraint_sampler_manager_ = spec_.constraint_sampler_mgr;
    if (!complete_initial_robot_state_)
        complete_initial_robot_state_ = new robot_state::RobotState(spec_.model);

    ROS_INFO("Initializing GeometricPlanningContext for '%s'", spec_.planner.c_str());

    // OMPL StateSpace
    ModelBasedStateSpaceSpecification state_space_spec(spec_.model, spec_.group);
    allocateStateSpace(state_space_spec);

    // OMPL SimpleSetup
    simple_setup_.reset(new ompl::geometric::SimpleSetup(mbss_));

    // OMPL ProjectionEvaluator
    it = spec_.config.find("projection_evaluator");
    if (it != spec_.config.end())
    {
        setProjectionEvaluator(boost::trim_copy(it->second));
        spec_.config.erase(it);
    }
    else if (planner_id_ != "")
        ROS_WARN("No projection evaluator for '%s'", planner_id_.c_str());

    // OMPL Planner
    if (planner_id_ != "")
    {
        ompl::base::PlannerPtr planner = configurePlanner(planner_id_, spec_.config);
        simple_setup_->setPlanner(planner);
    }

    // Initialize path constraints, if any
    if (!request_.path_constraints.position_constraints.empty() ||
        !request_.path_constraints.orientation_constraints.empty() ||
        !request_.path_constraints.visibility_constraints.empty() ||
        !request_.path_constraints.joint_constraints.empty())
    {
        path_constraints_.reset(new kinematic_constraints::KinematicConstraintSet(getRobotModel()));
        path_constraints_->add(request_.path_constraints, getPlanningScene()->getTransforms());
    }
    else
    {
        path_constraints_.reset();
    }

    // Library of constraints
    constraints_library_.reset(new ConstraintsLibrary(this, constraint_sampler_manager_));
    std::string cpath;
    if (nh_.getParam("constraint_approximations_path", cpath))
    {
        constraints_library_->loadConstraintApproximations(cpath);
        std::stringstream ss;
        constraints_library_->printConstraintApproximations(ss);
        ROS_INFO_STREAM(ss.str());
    }

    // OMPL StateSampler
    mbss_->setStateSamplerAllocator(boost::bind(&GeometricPlanningContext::allocPathConstrainedSampler, this, _1));


}

void GeometricPlanningContext::allocateStateSpace(const ModelBasedStateSpaceSpecification& state_space_spec)
{
    bool allocated = false;

    // If there are (only) position and/or orientation constraints, make sure we have a means to
    // compute IK solutions.  If so, allocate a pose model (workspace) state space representation
    if ((!request_.path_constraints.position_constraints.empty() || !request_.path_constraints.orientation_constraints.empty()) &&
         request_.path_constraints.joint_constraints.empty() && request_.path_constraints.visibility_constraints.empty())
    {
        const robot_model::JointModelGroup *jmg = state_space_spec.joint_model_group_;
        if (jmg)
        {
            const std::pair<robot_model::JointModelGroup::KinematicsSolver, robot_model::JointModelGroup::KinematicsSolverMap>& slv = jmg->getGroupKinematics();
            bool ik = false;
            // check that we have a direct means to compute IK
            if (slv.first)
                ik = jmg->getVariableCount() == slv.first.bijection_.size();
            else if (!slv.second.empty())
            {
                // or an IK solver for each of the subgroups
                unsigned int vc = 0;
                unsigned int bc = 0;
                for (robot_model::JointModelGroup::KinematicsSolverMap::const_iterator jt = slv.second.begin() ; jt != slv.second.end() ; ++jt)
                {
                    vc += jt->first->getVariableCount();
                    bc += jt->second.bijection_.size();
                }
                if (vc == jmg->getVariableCount() && vc == bc)
                    ik = true;
            }

            if (ik)
            {
                PoseModelStateSpacePtr state_space_(new PoseModelStateSpace(state_space_spec));
                mbss_ = boost::static_pointer_cast<ModelBasedStateSpace>(state_space_);
                allocated = true;
            }
        }
    }

    // The default is a representation based on the joint angles of the group
    if (!allocated)
    {
        JointModelStateSpacePtr state_space_(new JointModelStateSpace(state_space_spec));
        mbss_ = boost::static_pointer_cast<ModelBasedStateSpace>(state_space_);
    }
}

ompl::base::StateSamplerPtr GeometricPlanningContext::allocPathConstrainedSampler(const ompl::base::StateSpace* ss) const
{
    if (mbss_.get() != ss)
    {
        ROS_ERROR("%s: Attempted to allocate a state sampler for an unknown state space", name_.c_str());
        return ompl::base::StateSamplerPtr();
    }

    ROS_DEBUG("%s: Allocating a new state sampler (attempts to use path constraints)", name_.c_str());

    if (path_constraints_ && constraints_library_)
    {
        const ConstraintApproximationPtr &ca = constraints_library_->getConstraintApproximation(request_.path_constraints);
        if (ca)
        {
            ompl::base::StateSamplerAllocator c_ssa = ca->getStateSamplerAllocator(request_.path_constraints);
            if (c_ssa)
            {
                ompl::base::StateSamplerPtr res = c_ssa(ss);
                if (res)
                {
                    logInform("%s: Using precomputed state sampler (approximated constraint space) for constraint '%s'", name_.c_str(), request_.path_constraints.name.c_str());
                    return res;
                }
            }
        }

        constraint_samplers::ConstraintSamplerPtr cs = constraint_sampler_manager_->selectSampler(getPlanningScene(), getGroupName(), path_constraints_->getAllConstraints());
        if (cs)
        {
            ROS_INFO("%s: Allocating specialized state sampler for state space", name_.c_str());
            return ompl::base::StateSamplerPtr(new ConstrainedSampler(this, cs));
        }
    }
    ROS_DEBUG("%s: Allocating default state sampler for state space", name_.c_str());
    return ss->allocDefaultStateSampler();
}

void GeometricPlanningContext::clear()
{
    simple_setup_->clear();
    simple_setup_->clearStartStates();
    simple_setup_->setGoal(ompl::base::GoalPtr());
    simple_setup_->setStateValidityChecker(ompl::base::StateValidityCheckerPtr());
    goal_constraints_.clear();
}

void GeometricPlanningContext::preSolve()
{
    simple_setup_->getProblemDefinition()->clearSolutionPaths();
    const ompl::base::PlannerPtr planner = simple_setup_->getPlanner();
    if(planner)
        planner->clear();
    startGoalSampling();
    simple_setup_->getSpaceInformation()->getMotionValidator()->resetMotionCounter();
}

void GeometricPlanningContext::postSolve()
{
    stopGoalSampling();
    if (simple_setup_->getProblemDefinition()->hasApproximateSolution())
        ROS_WARN("Solution is approximate");
}

void GeometricPlanningContext::startGoalSampling()
{
  bool gls = simple_setup_->getGoal()->hasType(ompl::base::GOAL_LAZY_SAMPLES);
  if (gls)
    static_cast<ompl::base::GoalLazySamples*>(simple_setup_->getGoal().get())->startSampling();
  else
    // we know this is a GoalSampleableMux by elimination
    static_cast<GoalSampleableRegionMux*>(simple_setup_->getGoal().get())->startSampling();
}

void GeometricPlanningContext::stopGoalSampling()
{
  bool gls = simple_setup_->getGoal()->hasType(ompl::base::GOAL_LAZY_SAMPLES);
  if (gls)
    static_cast<ompl::base::GoalLazySamples*>(simple_setup_->getGoal().get())->stopSampling();
  else
    // we know this is a GoalSampleableMux by elimination
    static_cast<GoalSampleableRegionMux*>(simple_setup_->getGoal().get())->stopSampling();
}

bool GeometricPlanningContext::solve(planning_interface::MotionPlanResponse& res)
{
    double timeout = request_.allowed_planning_time;
    double plan_time = 0.0;
    bool result = solve(timeout, request_.num_planning_attempts, plan_time);

    if (result)
    {
        // Simplifying solution
        if (simplify_ && (timeout - plan_time) > 0)
        {
            plan_time += simplifySolution(timeout - plan_time);
        }

        ompl::geometric::PathGeometric &pg = simple_setup_->getSolutionPath();
        // Interpolating the solution
        if (interpolate_)
        {
            // The maximum length of a single segment in the solution path
            double max_segment_length = (spec_.max_waypoint_distance > 0.0 ? spec_.max_waypoint_distance : simple_setup_->getStateSpace()->getMaximumExtent() / 100.0);
            // Computing the total number of waypoints we want in the solution path
            unsigned int waypoint_count = std::max((unsigned int)floor(0.5 + pg.length() / max_segment_length), spec_.min_waypoint_count);
            interpolateSolution(pg, waypoint_count);
        }

        ROS_DEBUG("%s: Returning successful solution with %lu states", getName().c_str(),
                   pg.getStateCount());

        res.trajectory_.reset(new robot_trajectory::RobotTrajectory(getRobotModel(), getGroupName()));

        robot_state::RobotState ks = *complete_initial_robot_state_;
        for (std::size_t i = 0 ; i < pg.getStateCount() ; ++i)
        {
            mbss_->copyToRobotState(ks, pg.getState(i));
            res.trajectory_->addSuffixWayPoint(ks, 0.0);
        }

        res.planning_time_ = plan_time;
        res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    }
    else
    {
        ROS_INFO("%s: Unable to solve the planning problem", getName().c_str());
        res.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
    }

    return result;
}

bool GeometricPlanningContext::solve(planning_interface::MotionPlanDetailedResponse& res)
{
    double timeout = request_.allowed_planning_time;
    double plan_time = 0.0;
    bool result = solve(timeout, request_.num_planning_attempts, plan_time);

    if (result)
    {
        // Getting the raw solution
        ompl::geometric::PathGeometric &pg = simple_setup_->getSolutionPath();
        res.processing_time_.push_back(plan_time);
        res.description_.push_back("plan");

        res.trajectory_.resize(res.trajectory_.size() + 1);
        res.trajectory_.back().reset(new robot_trajectory::RobotTrajectory(getRobotModel(), getGroupName()));

        robot_state::RobotState ks = *complete_initial_robot_state_;
        for (std::size_t i = 0 ; i < pg.getStateCount() ; ++i)
        {
            mbss_->copyToRobotState(ks, pg.getState(i));
            res.trajectory_.back()->addSuffixWayPoint(ks, 0.0);
        }


        // Simplifying solution
        if (simplify_ && (timeout - plan_time) > 0)
        {
            double simplify_time = simplifySolution(timeout - plan_time);

            res.processing_time_.push_back(simplify_time);
            res.description_.push_back("simplify");

            pg = simple_setup_->getSolutionPath();
            res.trajectory_.resize(res.trajectory_.size() + 1);
            res.trajectory_.back().reset(new robot_trajectory::RobotTrajectory(getRobotModel(), getGroupName()));

            for (std::size_t i = 0 ; i < pg.getStateCount() ; ++i)
            {
                mbss_->copyToRobotState(ks, pg.getState(i));
                res.trajectory_.back()->addSuffixWayPoint(ks, 0.0);
            }
        }

        // Interpolating the final solution
        if (interpolate_)
        {
            pg = simple_setup_->getSolutionPath();
            // The maximum length of a single segment in the solution path
            double max_segment_length = (spec_.max_waypoint_distance > 0.0 ? spec_.max_waypoint_distance : simple_setup_->getStateSpace()->getMaximumExtent() / 100.0);
            // Computing the total number of waypoints we want in the solution path
            unsigned int waypoint_count = std::max((unsigned int)floor(0.5 + pg.length() / max_segment_length), spec_.min_waypoint_count);
            double interpolate_time = interpolateSolution(pg, waypoint_count);

            res.processing_time_.push_back(interpolate_time);
            res.description_.push_back("interpolate");

            ROS_DEBUG("%s: Returning successful solution with %lu states", getName().c_str(),
                       pg.getStateCount());

            res.trajectory_.resize(res.trajectory_.size() + 1);
            res.trajectory_.back().reset(new robot_trajectory::RobotTrajectory(getRobotModel(), getGroupName()));

            for (std::size_t i = 0 ; i < pg.getStateCount() ; ++i)
            {
                mbss_->copyToRobotState(ks, pg.getState(i));
                res.trajectory_.back()->addSuffixWayPoint(ks, 0.0);
            }
        }

        res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    }
    else
    {
        ROS_INFO("%s: Unable to solve the planning problem", getName().c_str());
        res.error_code_.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
    }

    return result;
}

bool GeometricPlanningContext::solve(double timeout, unsigned int count, double& total_time)
{
    ompl::time::point start = ompl::time::now();

    preSolve();

    bool result = false;
    total_time = 0.0;
    if (count <= 1)
    {
        ompl::base::PlannerTerminationCondition ptc = ompl::base::timedPlannerTerminationCondition(timeout - ompl::time::seconds(ompl::time::now() - start));
        registerTerminationCondition(ptc);
        result = simple_setup_->solve(ptc) == ompl::base::PlannerStatus::EXACT_SOLUTION;
        total_time = simple_setup_->getLastPlanComputationTime();
        unregisterTerminationCondition();
    }
    else // attempt to solve in parallel
    {
        ROS_DEBUG("Solving problem in parallel with up to %u threads", spec_.max_num_threads);
        ompl::tools::ParallelPlan pp(simple_setup_->getProblemDefinition());
        if (count <= spec_.max_num_threads) // fewer attempts than threads
        {
            if (planner_id_.size()) // There is a planner configured
            {
                for(unsigned int i = 0; i < count; ++i)
                    pp.addPlanner(configurePlanner(planner_id_, spec_.config));
            }
            else
            {
                for (unsigned int i = 0 ; i < count; ++i)
                    pp.addPlanner(ompl::tools::SelfConfig::getDefaultPlanner(simple_setup_->getGoal()));
            }

            ompl::base::PlannerTerminationCondition ptc = ompl::base::timedPlannerTerminationCondition(timeout - ompl::time::seconds(ompl::time::now() - start));
            registerTerminationCondition(ptc);
            // Solve in parallel.  Hybridize the solution paths.
            result = pp.solve(ptc, 1, count, true) == ompl::base::PlannerStatus::EXACT_SOLUTION;
            total_time = ompl::time::seconds(ompl::time::now() - start);
            unregisterTerminationCondition();
        }
        else // more attempts than threads
        {
            ompl::base::PlannerTerminationCondition ptc = ompl::base::timedPlannerTerminationCondition(timeout - ompl::time::seconds(ompl::time::now() - start));
            registerTerminationCondition(ptc);
            int n = count / spec_.max_num_threads;
            result = true;
            for (int i = 0; i < n && !ptc(); ++i)
            {
                pp.clearPlanners();
                if (planner_id_.size()) // There is a planner configured
                {
                    for(unsigned int i = 0; i < spec_.max_num_threads; ++i)
                        pp.addPlanner(configurePlanner(planner_id_, spec_.config));
                }
                else
                {
                    for (unsigned int i = 0 ; i < spec_.max_num_threads; ++i)
                        pp.addPlanner(ompl::tools::SelfConfig::getDefaultPlanner(simple_setup_->getGoal()));
                }

                result &= pp.solve(ptc, 1, count, true) == ompl::base::PlannerStatus::EXACT_SOLUTION;
            }

            // Do the remainder
            n = count % spec_.max_num_threads;
            if (n && !ptc())
            {
                pp.clearPlanners();
                if (planner_id_.size()) // There is a planner configured
                {
                    for(unsigned int i = 0; i < spec_.max_num_threads; ++i)
                        pp.addPlanner(configurePlanner(planner_id_, spec_.config));
                }
                else
                {
                    for (unsigned int i = 0 ; i < spec_.max_num_threads; ++i)
                        pp.addPlanner(ompl::tools::SelfConfig::getDefaultPlanner(simple_setup_->getGoal()));
                }

                result &= pp.solve(ptc, 1, count, true) == ompl::base::PlannerStatus::EXACT_SOLUTION;
            }
            total_time = ompl::time::seconds(ompl::time::now() - start);
            unregisterTerminationCondition();
        }
    }

    postSolve();

    return result;
}

double GeometricPlanningContext::simplifySolution(double max_time)
{
    simple_setup_->simplifySolution(max_time);
    return simple_setup_->getLastSimplificationTime();
}

double GeometricPlanningContext::interpolateSolution(ompl::geometric::PathGeometric &path, unsigned int waypoint_count)
{
    ompl::time::point start = ompl::time::now();
    path.interpolate(waypoint_count);
    return ompl::time::seconds(ompl::time::now() - start);
}

void GeometricPlanningContext::registerTerminationCondition(const ompl::base::PlannerTerminationCondition &ptc)
{
    boost::mutex::scoped_lock slock(ptc_lock_);
    ptc_ = &ptc;
}

void GeometricPlanningContext::unregisterTerminationCondition()
{
    boost::mutex::scoped_lock slock(ptc_lock_);
    ptc_ = NULL;
}


bool GeometricPlanningContext::terminate()
{
    boost::mutex::scoped_lock slock(ptc_lock_);
    if (ptc_)
        ptc_->terminate();
    return true;
}

const ModelBasedStateSpacePtr& GeometricPlanningContext::getOMPLStateSpace() const
{
    return mbss_;
}

const ompl::base::SpaceInformationPtr& GeometricPlanningContext::getOMPLSpaceInformation() const
{
    return simple_setup_->getSpaceInformation();
}

const ompl::base::ProblemDefinitionPtr& GeometricPlanningContext::getOMPLProblemDefinition() const
{
    return simple_setup_->getProblemDefinition();
}

const robot_state::RobotState& GeometricPlanningContext::getCompleteInitialRobotState() const
{
    return *complete_initial_robot_state_;
}

void GeometricPlanningContext::setCompleteInitialRobotState(const robot_state::RobotState& state)
{
    if (!complete_initial_robot_state_)
    {
        ROS_WARN("Planning context not initialized.  Not setting initial robot state");
        return;
    }

    *complete_initial_robot_state_ = state;

    // Start state
    ompl::base::ScopedState<> start_state(mbss_);
    mbss_->copyToOMPLState(start_state.get(), *complete_initial_robot_state_);
    simple_setup_->setStartState(start_state);

    // State validity checker
    simple_setup_->setStateValidityChecker(ompl::base::StateValidityCheckerPtr(new StateValidityChecker(this)));
}

bool GeometricPlanningContext::setGoalConstraints(const std::vector<moveit_msgs::Constraints> &goal_constraints,
                                                  moveit_msgs::MoveItErrorCodes *error)
{
    if (goal_constraints.empty())
    {
        ROS_WARN("No goal constraints specified.  There is no problem to solve.");
        if (error)
            error->val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
        return false;
    }

    // Translating goal constraints
    goal_constraints_.clear();
    for(size_t i = 0; i < goal_constraints.size(); ++i)
    {
        moveit_msgs::Constraints constr = kinematic_constraints::mergeConstraints(goal_constraints[i], request_.path_constraints);
        kinematic_constraints::KinematicConstraintSetPtr kset(new kinematic_constraints::KinematicConstraintSet(getRobotModel()));
        kset->add(constr, getPlanningScene()->getTransforms());
        if (!kset->empty())
            goal_constraints_.push_back(kset);
    }

    if (goal_constraints_.empty())
    {
        ROS_WARN("No goal constraints specified. There is no problem to solve.");
        if (error)
            error->val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
        return false;
    }

    // Creating constraint sampler for each constraint
    std::vector<ompl::base::GoalPtr> goals;
    for (std::size_t i = 0 ; i < goal_constraints_.size() ; ++i)
    {
        constraint_samplers::ConstraintSamplerPtr cs;
        if (constraint_sampler_manager_)
            cs = constraint_sampler_manager_->selectSampler(getPlanningScene(), getGroupName(), goal_constraints_[i]->getAllConstraints());
        if (cs)
        {
            ompl::base::GoalPtr g = ompl::base::GoalPtr(new ConstrainedGoalSampler(this, goal_constraints_[i], cs));
            goals.push_back(g);
        }
        else
        {
            ROS_WARN("No constraint sampler available to sample goal constraints");
        }
    }

    // Creating goal object using constraint samplers
    if (!goals.empty())
    {
        ompl::base::GoalPtr goal;
        if (goals.size() == 1)
            goal = goals[0];
        else
            goal = ompl::base::GoalPtr(new GoalSampleableRegionMux(goals));

        simple_setup_->setGoal(goal);
        return true;
    }

    OMPL_ERROR("Unable to construct goal representation");
    return false;
}

const robot_model::RobotModelConstPtr& GeometricPlanningContext::getRobotModel() const
{
    return mbss_->getRobotModel();
}

const robot_model::JointModelGroup* GeometricPlanningContext::getJointModelGroup() const
{
    return mbss_->getJointModelGroup();
}

ompl::base::PlannerPtr GeometricPlanningContext::configurePlanner(const std::string& planner_name, const std::map<std::string, std::string>& params)
{
    std::map<std::string, PlannerAllocator>::const_iterator it = planner_allocators_.find(planner_name);
    // Allocating planner using planner allocator
    if (it != planner_allocators_.end())
        return it->second(simple_setup_->getSpaceInformation(), spec_.name, params);

    // No planner configured by this name
    ROS_WARN("No planner allocator found with name '%s'", planner_name.c_str());
    for(std::map<std::string, PlannerAllocator>::const_iterator it = planner_allocators_.begin(); it != planner_allocators_.end(); ++it)
        ROS_WARN("  %s", it->first.c_str());
    return ompl::base::PlannerPtr();
}

void GeometricPlanningContext::setProjectionEvaluator(const std::string &peval)
{
    if (!mbss_)
    {
        ROS_ERROR("No state space is configured yet");
        return;
    }
    ompl::base::ProjectionEvaluatorPtr pe = getProjectionEvaluator(peval);
    if (pe)
        mbss_->registerDefaultProjection(pe);
}

ompl::base::ProjectionEvaluatorPtr GeometricPlanningContext::getProjectionEvaluator(const std::string &peval) const
{
  if (peval.find_first_of("link(") == 0 && peval[peval.length() - 1] == ')')
  {
    std::string link_name = peval.substr(5, peval.length() - 6);
    if (getRobotModel()->hasLinkModel(link_name))
      return ompl::base::ProjectionEvaluatorPtr(new ProjectionEvaluatorLinkPose(this, link_name));
    else
      ROS_ERROR("Attempted to set projection evaluator with respect to position of link '%s', but that link is not known to the kinematic model.", link_name.c_str());
  }
  else
    if (peval.find_first_of("joints(") == 0 && peval[peval.length() - 1] == ')')
    {
      std::string joints = peval.substr(7, peval.length() - 8);
      boost::replace_all(joints, ",", " ");
      std::vector<unsigned int> j;
      std::stringstream ss(joints);
      while (ss.good() && !ss.eof())
      {
        std::string v; ss >> v >> std::ws;
        if (getJointModelGroup()->hasJointModel(v))
        {
          unsigned int vc = getJointModelGroup()->getJointModel(v)->getVariableCount();
          if (vc > 0)
          {
            int idx = getJointModelGroup()->getVariableGroupIndex(v);
            for (int q = 0 ; q < vc ; ++q)
              j.push_back(idx + q);
          }
          else
            ROS_WARN("%s: Ignoring joint '%s' in projection since it has 0 DOF", name_.c_str(), v.c_str());
        }
        else
          ROS_ERROR("%s: Attempted to set projection evaluator with respect to value of joint '%s', but that joint is not known to the group '%s'.",
                   name_.c_str(), v.c_str(), getGroupName().c_str());
      }
      if (j.empty())
        ROS_ERROR("%s: No valid joints specified for joint projection", name_.c_str());
      else
        return ompl::base::ProjectionEvaluatorPtr(new ProjectionEvaluatorJointValue(this, j));
    }
    else
      ROS_ERROR("Unable to allocate projection evaluator based on description: '%s'", peval.c_str());
  return ompl::base::ProjectionEvaluatorPtr();
}

CLASS_LOADER_REGISTER_CLASS(ompl_interface::GeometricPlanningContext, ompl_interface::OMPLPlanningContext);