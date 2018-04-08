/*
 * LinkageStateChecker.h
 *
 *  Created on: Jan 25, 2014
 *      Author: iuiml
 */

#ifndef LINKAGESTATECHECKER_H_
#define LINKAGESTATECHECKER_H_


#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <math/vector.h>
#include "Linkage.h"

#include "utility.h"

namespace ob = ompl::base;

class LinkageStateChecker : public ob::StateValidityChecker
{
public:
	LinkageStateChecker(const ob::SpaceInformationPtr &si):ob::StateValidityChecker(si)
    {
        this->numD = si->getStateDimension();
        linkage = new Linkage(numD);
    }

	~LinkageStateChecker(){
		if(linkage)
			delete linkage;
	}
	void setLinkageLength(vector<double> length){
		linkage->setLinkLength(length);
	}

    virtual bool isValid(const ob::State *state) const
    {
        const ob::RealVectorStateSpace::StateType *relstate = state->as<ob::RealVectorStateSpace::StateType>();
        Math::Vector v(numD,relstate->values);
//        if(isValid(v) == false){
//        cout<<"v:"<<v<<endl;
//    	cout<<"linkage checker:"<<endl;getchar();
//        }
        return isValid(v);
    }

    bool isValid(const Vector &v) const
    {
    	linkage->updateConfig(v);
    	return !(linkage->selfCollision());
    }

private:
    Linkage *linkage;
    int numD;
};

#endif /* LINKAGESTATECHECKER_H_ */
