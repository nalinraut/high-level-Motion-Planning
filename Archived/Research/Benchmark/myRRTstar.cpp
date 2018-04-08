/*
 * myRRTstar.cpp
 *
 *  Created on: Feb 19, 2012
 *      Author: Weiran
 */

#include "myRRTstar.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/datastructures/NearestNeighborsGNAT.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/goals/GoalState.h"
#include <limits>

#include "utility.h"


void ompl::geometric::myRRTstar::clear(void)
{
	Planner::clear();
	sampler_.reset();
	freeMemory();
	if (nn_)
		nn_->clear();
}

void ompl::geometric::myRRTstar::setup(void)
{
	Planner::setup();
	tools::SelfConfig sc(si_, getName());
	sc.configurePlannerRange(maxDistance_);

	if (!nn_)
		nn_.reset(new NearestNeighborsGNAT<Motion*>());
	nn_->setDistanceFunction(boost::bind(&myRRTstar::distanceFunction, this, _1, _2));
}

void ompl::geometric::myRRTstar::freeMemory(void)
{
	if (nn_) {
		std::vector<Motion*> motions;
		nn_->list(motions);
		for (unsigned int i = 0; i < motions.size(); ++i) {
			if (motions[i]->state)
				si_->freeState(motions[i]->state);
			delete motions[i];
		}
	}
}

void ompl::geometric::myRRTstar::getPlannerData(base::PlannerData &data) const
{
	Planner::getPlannerData(data);
	std::vector<Motion*> motions;
	if (nn_)
		nn_->list(motions);
	for (unsigned int i = 0; i < motions.size(); ++i)
//		data.recordEdge(motions[i]->parent ? motions[i]->parent->state : NULL, motions[i]->state);
		data.addEdge(motions[i]->parent ? motions[i]->parent->state : NULL, motions[i]->state);
}
//internal function for finding the size of radius of search neighbor hood.

double ompl::geometric::myRRTstar::getRadius(void)
{
	int numDimension = si_->getStateSpace()->getDimension();
	int dataSz = nn_->size();
	double radius = gamma * pow(log((double) (dataSz + 1.0)) / ((double) (dataSz + 1.0)),
		1.0 / (double) (numDimension));
	return radius;
}

//this is the function for priority Queue used for finding best parent, it will put smaller cost state in front!

int compareCost(std::pair<ompl::geometric::myRRTstar::Motion*, double> i, std::pair<ompl::geometric::myRRTstar::Motion*, double> j)
{
	return(i.second < j.second);
}

bool ompl::geometric::myRRTstar::findBestParent(Motion* rndMotion, std::vector<Motion*> &nbh, Motion*& bestParent)
{
	double t1=elapsedTime();
	//if the nearest neighbor is empty
	if (nbh.size() == 0) {
		//get the nearest one
		bestParent = nn_->nearest(rndMotion);
		//if no nearest neighbor
		if (bestParent == NULL)
		{
			double t2=elapsedTime();
			this->t_nnq+=t2-t1;
			return false;
		}
		//else check the validity between these 2 states
		double time1 = elapsedTime();
		bool resCheckMotion = si_->checkMotion(rndMotion->state, bestParent->state);
		double time2 = elapsedTime();
		t_cc_ += time2 - time1;
		if (!resCheckMotion)
		{
			double t2=elapsedTime();
			this->t_nnq+=t2-t1;
			return false;
		}
	}//or find the best parent among neighbor hood
	else
	{
		int numNBH = nbh.size();
		//cost and neighbor pair
		std::vector<std::pair<Motion*, double> > stateCostPair(numNBH);
		//an index iterator
		int i = 0;
		//cost of motion
		double trajCost;
		for (std::vector<Motion*>::iterator iter = nbh.begin(); iter != nbh.end(); iter++)
		{
			stateCostPair[i].first = *iter;
			trajCost = distanceFunction((*iter), rndMotion);
			stateCostPair[i].second = (*iter)->costFromRoot + trajCost;
			i++;
		}
		// Sort vertices according to cost
		std::sort(stateCostPair.begin(), stateCostPair.end(), compareCost);
		// Try out each extension according to increasing cost
		i = 0;
		for (std::vector< std::pair<Motion*, double> >::iterator iter = stateCostPair.begin();
			iter != stateCostPair.end(); iter++)
		{

			Motion* motionCurr = iter->first;

			// Extend the current vertex towards stateIn (and this time check for collision with obstacles)
			double time1 = elapsedTime();
			bool resCheckMotion = si_->checkMotion(motionCurr->state, rndMotion->state);
			double time2 = elapsedTime();
			t_cc_ += time2 - time1;
			if (resCheckMotion)
			{
				bestParent = motionCurr;
				double t2 = elapsedTime();
				this->t_nnq += t2 - t1;
				return true;
			}
		}
		
		double t2=elapsedTime();
		this->t_nnq+=t2-t1;
		return false;
	}
	return true;
}

void ompl::geometric::myRRTstar::checkBetterMotion(Motion* motion, base::Goal* goal)
{
	if (goal->isSatisfied(motion->state))
	{
		double costCurr = motion->costFromRoot;
		if ((lowerBoundMotion == NULL) || ((lowerBoundMotion != NULL) && (costCurr < lowerBoundCost)))
		{
			lowerBoundMotion = motion;
			lowerBoundCost = costCurr;
		}
	}
}

//This is the function for updating children costs during rewiring process!

void ompl::geometric::myRRTstar::updateBranchCost(Motion* motion, int level, base::Goal* goal)
{
	for (std::set< Motion* >::iterator iter = motion->children.begin(); iter != motion->children.end(); iter++)
	{
		Motion* motionCurr = *iter;
		motionCurr->costFromRoot = motion->costFromRoot + motionCurr->costFromParent;
		checkBetterMotion(motionCurr, goal);
		updateBranchCost(motionCurr, level + 1, goal);
	}
}

bool ompl::geometric::myRRTstar::prepareSolve()
{
	checkValidity();

	while (const base::State * st = pis_.nextStart())
	{
		Motion *motion = new Motion(si_);
		si_->copyState(motion->state, st);
		nn_->add(motion);
	}

	if (nn_->size() == 0)
	{
		printf("There are no valid initial states!\n");
		return false;
	}

	if (!sampler_)
		sampler_ = si_->allocStateSampler();

	//msg_.inform("Starting with %u states", nn_->size());

	return true;
}

void ompl::geometric::myRRTstar::sampleAState(ompl::base::State* rndState)
{
	base::Goal *goal = pdef_->getGoal().get();
	base::GoalSampleableRegion *goal_s = dynamic_cast<base::GoalSampleableRegion*> (goal);
	/* sample random state (with goal biasing) */
	if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
		goal_s->sampleGoal(rndState);
	else
		sampler_->sampleUniform(rndState);
}

bool ompl::geometric::myRRTstar::shouldInsert(Motion* rndMotion)
{
	//decide whether or not to insert the state
	if (lowerBoundMotion != NULL)
	{
		double cost2goal = DBL_MAX;
		base::State* tmpState;
		base::Goal *goal = pdef_->getGoal().get();
		base::GoalSampleableRegion *goal_s = dynamic_cast<base::GoalSampleableRegion*> (goal);
		if (goal_s)
		{
			ompl::base::GoalState *tmpGoal = goal_s->as<ompl::base::GoalState > ();
			tmpState = tmpGoal->getState();
			cost2goal = si_->distance(tmpState, rndMotion->state) - goal_s->getThreshold();
			if (lowerBoundCost < rndMotion->costFromRoot + cost2goal)
				return false;
		}
	}
	return true;
}

void ompl::geometric::myRRTstar::insertMotion(Motion* bestParent, Motion*& addedMotion, Motion* rndMotion)
{
	//We have to allocate space for this addedMotion first!
	addedMotion = new Motion(si_);
	si_->copyState(addedMotion->state, rndMotion->state);
	addedMotion->parent = bestParent;
	double costFromParent = 0.0;
	costFromParent = distanceFunction(addedMotion, bestParent);
	addedMotion->costFromParent = costFromParent;
	addedMotion->costFromRoot = bestParent->costFromRoot + costFromParent;
	base::Goal *goal = pdef_->getGoal().get();
	checkBetterMotion(addedMotion, goal);
	nn_->add(addedMotion);
	//do something about parent
	bestParent->children.insert(addedMotion);
}

void ompl::geometric::myRRTstar::rewire(std::vector<Motion*>& nbh, Motion* addedMotion)
{
	double t1 = elapsedTime();
	if (nbh.size() > 0)
	{
		for (std::vector<Motion* >::iterator iter = nbh.begin(); iter != nbh.end(); iter++)
		{
			Motion* motionCurr = *iter;
			double cost = distanceFunction(addedMotion, motionCurr);
			if (addedMotion->costFromRoot + cost < motionCurr->costFromRoot)
			{
				double t1 = elapsedTime();
				bool resCheckMotion = si_->checkMotion(addedMotion->state, motionCurr->state);
				double t2 = elapsedTime();
				t_cc_ += t2 - t1;
				if (resCheckMotion)
				{
					if (motionCurr->parent)
						motionCurr->parent->children.erase(motionCurr);
					motionCurr->parent = addedMotion;
					motionCurr->costFromParent = cost;
					motionCurr->costFromRoot = addedMotion->costFromRoot + cost;
					addedMotion->children.insert(motionCurr);
					base::Goal *goal = pdef_->getGoal().get();
					updateBranchCost(motionCurr, 0, goal);
					checkBetterMotion(motionCurr, goal);
				} else
					continue;
			}

		}
	}
	double t2 = elapsedTime();
	this->t_rw += t2 - t1;
}

void ompl::geometric::myRRTstar::storePath()
{
	base::Goal *goal = pdef_->getGoal().get();
	double approxdif = std::numeric_limits<double>::max();
	;
	/* construct the solution path */
	std::vector<Motion*> mpath;
	Motion* tmp = lowerBoundMotion;
	while (tmp != NULL)
	{
		mpath.push_back(tmp);
		tmp = tmp->parent;
	}

	/* set the solution path */
	PathGeometric *path = new PathGeometric(si_);
	for (int i = mpath.size() - 1; i >= 0; --i)
	{
		if (!mpath[i]->state)
			std::cout << "got you in writing path" << std::endl;
		path->getStates().push_back(si_->cloneState(mpath[i]->state));
	}
//	goal->addSolutionPath(base::PathPtr(path));
	pdef_->addSolutionPath(base::PathPtr(path));
}

void ompl::geometric::myRRTstar::shouldInterpolate(Motion* bestParent, Motion* rndMotion)
{
	double d = si_->distance(bestParent->state, rndMotion->state);
	if (d > maxDistance_)
	{
		base::State *tmpState = si_->allocState();
		si_->getStateSpace()->interpolate(bestParent->state, rndMotion->state, maxDistance_ / d, tmpState);
		//copy tmp state back to rndMotion
		si_->copyState(rndMotion->state, tmpState);
		//free the state
		si_->freeState(tmpState);
	}


}

ompl::base::PlannerStatus ompl::geometric::myRRTstar::solve(const base::PlannerTerminationCondition &ptc)
{
	if (!prepareSolve()) return base::PlannerStatus::INVALID_START;

	Motion *rndMotion = new Motion(si_);
	base::State *rndState = rndMotion->state;
	Motion *bestParent = NULL;
	Motion *addedMotion = NULL;
	//nearest neighbor within r
	std::vector<Motion*> nbh;
	double radius;
	int iterationCount = 0;
	while (iterationCount < iterationLimit)
	{
		iterationCount += 1;
		//step 1 sample a random state
		sampleAState(rndState);

		if(si_->isValid(rndState) == false)
			break;

		//step 2 compute near neighbor
		radius = getRadius();
		nn_->nearestR(rndMotion, radius, nbh);

		//step 3 find the best parent
		if (!findBestParent(rndMotion, nbh, bestParent))
			continue;
		//decide whether or not should interpolate
		shouldInterpolate(bestParent, rndMotion);

		//decide whether to insert the state
		if (!shouldInsert(rndMotion))
			continue;
		else
			insertMotion(bestParent, addedMotion, rndMotion);

		//step 4 rewiring
		rewire(nbh, addedMotion);
	}

	if (lowerBoundMotion != NULL)
	{
		//std::cout << "we found a path" << std::endl;
		storePath();
	} else
		//std::cout << "didn't find a solution!" << std::endl;

		//clean temporary states
		if (rndMotion->state)
		si_->freeState(rndMotion->state);
	delete rndMotion;

//	return pdef_->getGoal().get()->isAchieved();
    if(pdef_->hasSolution())
    	return base::PlannerStatus::APPROXIMATE_SOLUTION;
    return base::PlannerStatus::TIMEOUT;

}
