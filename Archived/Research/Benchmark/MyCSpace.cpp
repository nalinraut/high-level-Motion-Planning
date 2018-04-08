/*
 * MyCSpace.cpp
 *
 *  Created on: Jan 22, 2014
 *      Author: iuiml
 */




#include "MyCSpace.h"
#include <planning/EdgePlanner.h>
#include <math/random.h>
#include "PlannerSettings.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>

MyCSpace::MyCSpace(int n, ob::StateValidityChecker* checker, const ob::SpaceInformationPtr& si) {
	nD = n;
	qMin.resize(nD);
	qMax.resize(nD);
	qMin.set(-Inf);
	qMax.set(Inf);
	stateChecker = checker;
	resolution = 0.001;
	si_ = si;
}

void MyCSpace::SetSpaceBounds(const Config& qmin, const Config& qmax) {
	qMin = qmin;
	qMax = qmax;
}

void MyCSpace::SetSpaceBounds(const double& qmin, const double& qmax) {
	qMin.set(qmin);
	qMax.set(qmax);
}

void MyCSpace::Sample(Config& x) {
	x.resize(nD);
	for(int i = 0; i < nD; i++)
		x[i] = Rand(qMin[i], qMax[i]);
}

bool MyCSpace::IsFeasible(const Config& x) {
	ob::State *omplstate = ((si_->getStateSpace())->as<ob::RealVectorStateSpace>()->allocState());
	for (int i = 0; i < nD; i++) {
		(omplstate->as<ob::RealVectorStateSpace::StateType>())->values[i] = x[i];
	}
//	cout<<"config:"<<x<<endl;
//	cout<<"omplstate:"<<endl;
//	si_->printState(omplstate);getchar();
	return stateChecker->isValid(omplstate);
}

EdgePlanner* MyCSpace::LocalPlanner(const Config& x, const Config& y) {
	return new StraightLineEpsilonPlanner(this, x, y, resolution);
}
