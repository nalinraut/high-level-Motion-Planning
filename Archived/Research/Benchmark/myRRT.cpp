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

#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/datastructures/NearestNeighborsGNAT.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/tools/config/MagicConstants.h"
#include "ompl/geometric/PathSimplifier.h"

#include "myRRT.h"
#include "utility.h"
#include <limits>
#include <stdio.h>
#include <iostream>
using namespace std;

ompl::geometric::myRRT::myRRT::myRRT(const base::SpaceInformationPtr &si, int iterationLimit) : base::Planner(si, "myRRT")
{
    specs_.approximateSolutions = true;

	iterationCounter_=0;
    goalBias_ = 0.05;
    maxDistance_ = 0.0;

    this->iterationLimit_ = iterationLimit;
    t_cc_ = 0;
    t_shortcut_ = 0;
    do_shortcut = false;
    best_path = NULL;
}

void ompl::geometric::myRRT::clear(void)
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
}

void ompl::geometric::myRRT::setup(void)
{
    Planner::setup();
	tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);
    if (!nn_)
        nn_.reset(new NearestNeighborsGNAT<Motion*>());
    nn_->setDistanceFunction(boost::bind(&myRRT::distanceFunction, this, _1, _2));
}

void ompl::geometric::myRRT::freeMemory(void)
{
    if (nn_)
    {
        std::vector<Motion*> motions;
        nn_->list(motions);
        for (unsigned int i = 0 ; i < motions.size() ; ++i)
        {
            if (motions[i]->state)
                si_->freeState(motions[i]->state);
            delete motions[i];
        }
    }
}

ompl::base::PlannerStatus ompl::geometric::myRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal                 *goal   = pdef_->getGoal().get();
    base::GoalSampleableRegion *goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);


    while (const base::State *st = pis_.nextStart())
    {
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        nn_->add(motion);
    }

    if (nn_->size() == 0)
    {
        printf("There are no valid initial states!\n");
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();


    Motion *solution  = NULL;
    double  approxdif = std::numeric_limits<double>::infinity();
    Motion *rmotion   = new Motion(si_);
    base::State *rstate = rmotion->state;
    base::State *xstate = si_->allocState();
//    Motion *goalmotion   = new Motion(si_);
//    goal_s->sampleGoal(goalmotion->state);
//	si_->printState(goalmotion->state,cout);
//	base::State *goalstate = goalmotion->state;

    iterationCounter_ = 0;
    double init_time = elapsedTime();
    double used_time = 0;
//    while (iterationCounter_ < iterationLimit_)
    while(used_time < 60)
    {
		iterationCounter_++;
        /* sample random state (with goal biasing) */
        if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);

//        si_->printState(rstate);
        /* find closest state in the tree */
        Motion *nmotion = nn_->nearest(rmotion);
        base::State *dstate = rstate;

        /* find state to add */
        double d = si_->distance(nmotion->state, rstate);
        if (d > maxDistance_)
        {
            si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
            dstate = xstate;
        }
		double t1 = elapsedTime();
        bool resCheckMotion = si_->checkMotion(nmotion->state, dstate);
		double t2 = elapsedTime();
		t_cc_ += t2 - t1;

        if (resCheckMotion)
        {
            /* create a motion */
            Motion *motion = new Motion(si_);
            si_->copyState(motion->state, dstate);
            motion->parent = nmotion;

            nn_->add(motion);
            double dist = 0.0;
            bool solved = goal->isSatisfied(motion->state, &dist);
            if (solved)
            {
                solution = motion;

                /* construct the solution path */
                std::vector<Motion*> mpath;
                while (solution != NULL)
                {
                    mpath.push_back(solution);
                    solution = solution->parent;
                }

                /* set the solution path */
                PathGeometric *path = new PathGeometric(si_);
			   for (int i = mpath.size() - 1 ; i >= 0 ; --i){
				   path->getStates().push_back(si_->cloneState(mpath[i]->state));
			   }
			   if(pdef_->hasSolution()){
				   double old_length = (pdef_->getSolutionPath())->length();
				   if(path->length() < old_length){
					   pdef_->clearSolutionPaths();
					   pdef_->addSolutionPath(base::PathPtr(path), true, dist);
					   best_path = path;
				   }else
					   delete path;
			   }else{
				   pdef_->addSolutionPath(base::PathPtr(path), true, dist);
				   best_path = path;
			   }
                break;
            }else{
//            	cout<<"there is no solve case!"<<endl;
            }
        }
        used_time = elapsedTime() - init_time;
    }

    si_->freeState(xstate);
    if (rmotion->state)
        si_->freeState(rmotion->state);
    delete rmotion;

    if(pdef_->hasSolution())
    	return base::PlannerStatus::APPROXIMATE_SOLUTION;
    return base::PlannerStatus::TIMEOUT;
}

//void ompl::geometric::myRRT::set_bestpath(const PathGeometric* path){
//	   if(best_path)
//		   delete best_path;
//	   best_path = new PathGeometric(si_);
//	   for(int i = 0; i < path->getStateCount(); i++){
//		   best_path->getStates().push_back(si_->cloneState(path->getState(i)));
//	   }
//}

void ompl::geometric::myRRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion*> motions;
    if (nn_)
        nn_->list(motions);

    for (unsigned int i = 0 ; i < motions.size() ; ++i)
//        data.recordEdge(motions[i]->parent ? motions[i]->parent->state : NULL, motions[i]->state);
        data.addEdge(motions[i]->parent ? motions[i]->parent->state : NULL, motions[i]->state);
}

