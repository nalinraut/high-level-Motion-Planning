/*
 * MyCSpace.h
 *
 *  Created on: Jan 22, 2014
 *      Author: iuiml
 */

#ifndef MYCSPACE_H_
#define MYCSPACE_H_


#include <math/vector.h>
#include <planning/CSpace.h>
//#include "PolyhedronStateChecker.h"
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/MotionValidator.h>
namespace ob = ompl::base;

class MyCSpace : public CSpace
{
public:
	MyCSpace(int n, ob::StateValidityChecker* checker, const ob::SpaceInformationPtr& si);
	~MyCSpace() {};
	void SetSpaceBounds(const Config& qmin, const Config& qmax);
	void SetSpaceBounds(const double& qmin, const double& qmax);
	virtual void Sample(Config& x);
	virtual bool IsFeasible(const Config& x);
	virtual EdgePlanner* LocalPlanner(const Config& x,const Config& y);

    ob::SpaceInformationPtr       si_;
	ob::StateValidityChecker* stateChecker;
	Config qMin, qMax;
	int nD;
	double resolution;
};

#endif /* MYCSPACE_H_ */

