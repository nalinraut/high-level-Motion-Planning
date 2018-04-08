/* 
 * File:   utils.h
 * Author: Weiran
 *
 * Created on March 17, 2012, 4:16 PM
 */

#ifndef UTILITY_H
#define	UTILITY_H

#include <iostream>
#include <stdio.h>
#include <fstream>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#include <time.h>

#include <math/matrix.h>
#include <math/vector.h>
#include <vector>

#include <ompl/base/PlannerData.h>
#include <ompl/base/ScopedState.h>

using namespace std;
using namespace Math;
namespace ob = ompl::base;

typedef Vector Config;

bool createDir(char* dir);

double determinantTriangleMatrix(const Math::Matrix &matrix);

void state_to_config(const ob::ScopedState<>& state, Config& config);

//namespace ob = ompl::base;
//
//class tgfPlannerData : public ob::PlannerData {
//    //this class only works for R^2 for now!!!
//public:
//    void print(std::ostream &out = std::cout) const;
//    tgfPlannerData & operator=(const tgfPlannerData &other);
//
//};


#endif	/* UTILITY_H */


