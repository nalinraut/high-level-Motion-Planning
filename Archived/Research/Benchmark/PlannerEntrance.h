/*
 * PlannerEntrance.h
 *
 *  Created on: Jan 15, 2014
 *      Author: iuiml
 */

#ifndef PLANNERENTRANCE_H_
#define PLANNERENTRANCE_H_

#include "PolyhedronStateChecker.h"
#include "SetupEnvironment.h"
#include "PlannerSettings.h"
#include <ompl/base/ProblemDefinition.h>
#include <stdio.h>

using namespace std;

namespace ob = ompl::base;
namespace og = ompl::geometric;

void openIOFiles(char* pLengthDir, int runs, FILE* SolutionInfoFile, ofstream *SolutionPathFile, ofstream *PlannerDataFile);
void closeIOFiles(FILE* SolutionInfoFile, ofstream *SolutionPathFile, ofstream *PlannerDataFile);

double getCutoffSeconds(int nD);

void testLinkage();

double RRTPlan(char* pLengthDir, int runs, ob::ProblemDefinitionPtr &pdef, bool do_shortcut);

void RRTStarPlan(char* pLengthDir, int runs, ob::ProblemDefinitionPtr &pdef);

double CEPlan(char* pLengthDir, int runs, ob::ProblemDefinitionPtr &pdef, ob::ScopedState<> &start, ob::ScopedState<> &goal, ob::StateValidityChecker* myChecker);

void PRMStarPlan(ScenarioIndex scenarioIndex, char* pLengthDir, int runs, ob::ProblemDefinitionPtr &pdef, ob::ScopedState<> &start, ob::ScopedState<> &goal, ob::StateValidityChecker* myChecker, bool lazy);

void FMMPlan(char* pLengthDir, int runs, ob::ProblemDefinitionPtr &pdef, ob::ScopedState<> &start, ob::ScopedState<> &goal, ob::StateValidityChecker* myChecker, double thin, bool do_offset);

//void FMMwithPerturbationPlan(ScenarioIndex scenario, int nD, double width, double thin, char* pLengthDir, int runs);

#endif /* PLANNERENTRANCE_H_ */
