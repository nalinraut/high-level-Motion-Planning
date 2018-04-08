/*
 * PlannerEntrance.cpp
 *
 *  Created on: Jan 15, 2014
 *      Author: iuiml
 */

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/PlannerData.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
//#include <ompl/contrib/rrt_star/RRTstar.h>
#include <ompl/base/PlannerTerminationCondition.h>

#include "ompl/datastructures/NearestNeighbors.h"

#include <math/random.h>
#include <ompl/config.h>
#include <iostream>
#include <math.h>
#include <fstream>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#include <Timer.h>

#include "planning/OptimalMotionPlanner.h"
#include "ompl/geometric/PathSimplifier.h"

#include "PlannerEntrance.h"
#include "myRRT.h"
#include "myRRTstar.h"
#include "CEntropy.h"
#include "FMM.h"
#include "SetupEnvironment.h"
#include "Polyhedron.h"
#include "utility.h"
#include "MyCSpace.h"
#include "Linkage.h"

using namespace std;

namespace ob = ompl::base;
namespace og = ompl::geometric;

void openIOFiles(char* pLengthDir, int runs, FILE* SolutionInfoFile, ofstream *SolutionPathFile, ofstream *PlannerDataFile){
	char pLengthName[100];
	sprintf(pLengthName, "%s/pathLength%04d.txt", pLengthDir, runs);
	SolutionInfoFile = fopen(pLengthName, "w");
	if(recordPath){
		sprintf(pLengthName, "%s/path%04d.txt", pLengthDir, runs);
		SolutionPathFile->open(pLengthName);
	}
	if(recordPlannerData){
		sprintf(pLengthName, "%s/plannerdata%04d.txt", pLengthDir, runs);
		PlannerDataFile->open(pLengthName);
	}
}

double getCutoffSeconds(int nD){
	if(nD == 2)
		return cutoff_seconds_2D;
	if(nD == 3)
		return cutoff_seconds_3D;
	return cutoff_seconds_HD;
}

void closeIOFiles(FILE* SolutionInfoFile, ofstream *SolutionPathFile, ofstream *PlannerDataFile){
	fclose(SolutionInfoFile);
	SolutionPathFile->close();
	PlannerDataFile->close();
}

void testLinkage(){
	int nD = 8;
	Config config(nD);
//	double joint_start[] = {150, -90, -50, -80, -90, -100, -120};
	for(int i = 0; i < nD; i++)
		config[i] = joint_start_8D[i]*Math::Pi/180.0;
	vector<double> linklength(1,0);
	linklength.resize(nD);
	for(int i = 0; i < nD; i++)
		linklength[i] = 1;
	Linkage linkage(config, linklength);
	cout<<"config:"<<config<<endl;
	cout<<"joint positions:"<<endl;
	linkage.printJointPos(cout);
	if(linkage.selfCollision()){
		cout<<"In Collision!"<<endl;
	}else
		cout<<"No Collision!"<<endl;
}

double RRTPlan(char* pLengthDir, int runs, ob::ProblemDefinitionPtr &pdef, bool do_shortcut) {
	//============prepare files to output=============
	FILE* SolutionInfoFile;
	ofstream SolutionPathFile;
	ofstream PlannerDataFile;
	char pLengthName[100];
	sprintf(pLengthName, "%s/pathLength%04d.txt", pLengthDir, runs);
	SolutionInfoFile = fopen(pLengthName, "w");
	if(recordPath){
		sprintf(pLengthName, "%s/path%04d.txt", pLengthDir, runs);
		SolutionPathFile.open(pLengthName);
	}
	if(recordPlannerData){
		sprintf(pLengthName, "%s/plannerdata%04d.txt", pLengthDir, runs);
		PlannerDataFile.open(pLengthName);
	}
	fprintf(SolutionInfoFile, "%%iterations\ttotalTime\tCCTime\tpathLength\tShortcutTime\t\n");
	//=============prepare to run====================
	int nD = pdef->getSpaceInformation()->getStateDimension();
	double cutoff_seconds = getCutoffSeconds(nD);
	//elapse time
	double timeSpentTotal = 0.0;
	double timeSpentCC = 0.0;
	double timeShortcut = 0;
	//best path found
	double bestPathCost = std::numeric_limits<double>::max();
	//total iteration counter
	int totalIteration = 0;
	og::PathGeometric *bestPath = NULL;
	while(true){
		//===========set up planner========================
		og::myRRT * planner = new og::myRRT(pdef->getSpaceInformation(), rrt_rrtstar_prm_samples*nD);
		pdef->clearSolutionPaths();
		// set the problem we are trying to solve for the planner
		planner->setProblemDefinition(pdef);
		// perform setup steps for the planner
		planner->setup();
		planner->set_shortcut(do_shortcut);
		//======solve=============
		double t1 = elapsedTime();
		ob::PlannerStatus solved = planner->solve(PTC);
		double t2 = elapsedTime();
		timeSpentCC += planner->getCCTime();
		timeShortcut += planner->getShortcutTime();
		timeSpentTotal += t2 - t1;
		if (solved) {
			ob::ProblemDefinitionPtr pdef = planner->getProblemDefinition();
			ob::PathPtr path = pdef->getSolutionPath();

		   double pathLength = planner->best_path->length();
			if (pathLength < bestPathCost) //do the update
			{
				if(bestPath != NULL){
					delete bestPath;
				}
				bestPath = new og::PathGeometric(pdef->getSpaceInformation());
				for (int i = 0; i < planner->best_path->getStateCount(); i++){
				   bestPath->getStates().push_back(pdef->getSpaceInformation()->cloneState(planner->best_path->getState(i)));
			    }
			}
		} else {
//			std::cout << "My code shouldn't run here normally" << std::endl;
		}
		if(bestPath != NULL){
		   //short cut the best path
		   if(do_shortcut){
			   double t1 = elapsedTime();
			   og::PathSimplifier pathsimplifier(pdef->getSpaceInformation());
			   pathsimplifier.shortcutPath(*(bestPath));
			   timeShortcut += elapsedTime() - t1;
		   }
		   double newpathlength = bestPath->length();
		   if(newpathlength < bestPathCost-0.0001){
			   //record result
				bestPathCost = newpathlength;
				totalIteration = planner->getIterationCounter();
				fprintf(SolutionInfoFile, "%05d\t%.8e\t%.8e\t%.8e\t%.8e\n",
						totalIteration, timeSpentTotal, timeSpentCC, bestPathCost, timeShortcut);
				fflush(SolutionInfoFile);
				//path cost
				if(recordPath){
					bestPath->print(SolutionPathFile);
					SolutionPathFile.flush();
				}

				if(recordPlannerData){
					ob::PlannerData Data(pdef->getSpaceInformation());
					planner->getPlannerData(Data);
					Data.printGraphviz(PlannerDataFile);
				}
		   }
		}

		if (timeSpentTotal > cutoff_seconds){
			if(!(solved)){
				totalIteration = planner->getIterationCounter();
				fprintf(SolutionInfoFile, "%05d\t%.8e\t%.8e\t%.8e\t%.8e\n",
						totalIteration, timeSpentTotal, timeSpentCC, bestPathCost, timeShortcut);
				fflush(SolutionInfoFile);
			}
			break;
		}
		delete planner;
	}

	if(bestPath != NULL){
		delete bestPath;
	}

	fclose(SolutionInfoFile);
	SolutionPathFile.close();
	PlannerDataFile.close();

	return bestPathCost;
}

void RRTStarPlan(char* pLengthDir, int runs, ob::ProblemDefinitionPtr &pdef) {
	//record data tree?
	bool record = false;
	pdef->clearSolutionPaths();
	//=============prepare out put files===========
	FILE* SolutionInfoFile;
	ofstream SolutionPathFile;
	ofstream PlannerDataFile;
	char pLengthName[100];
	sprintf(pLengthName, "%s/pathLength%04d.txt", pLengthDir, runs);
	SolutionInfoFile = fopen(pLengthName, "w");
	if(recordPath){
		sprintf(pLengthName, "%s/path%04d.txt", pLengthDir, runs);
		SolutionPathFile.open(pLengthName);
	}
	if(recordPlannerData){
		sprintf(pLengthName, "%s/plannerdata%04d.txt", pLengthDir, runs);
		PlannerDataFile.open(pLengthName);
	}
	fprintf(SolutionInfoFile, "%%iterations\t"
			"totalTime\t"
			"CCTime\t"
			"RewireTime\t"
			"NNQtime\t"
			"pathLength\n");
	int nD = pdef->getSpaceInformation()->getStateDimension();
	double cutoff_seconds = getCutoffSeconds(nD);
	//===========set up planner========================
	og::myRRTstar * planner = new og::myRRTstar(pdef->getSpaceInformation(), 1);
//	planner->setGama(pdef->getSpaceInformation()->getMaximumExtent()/2.0);
	// set the problem we are trying to solve for the planner
	planner->setProblemDefinition(pdef);
	// perform setup steps for the planner
	planner->setup();
	//========prepare to run=======================
	//elapse time
	double timeSpentTotal = 0.0;
	double timeSpentCC = 0.0;
	double timeSpentNNQ = 0.0;
	double timeSpentRW = 0.0;

	double bestPathCost = std::numeric_limits<double>::max();
	int iter = 0;
	while(true){
		//solve the problem
		double t1 = elapsedTime();
		ob::PlannerStatus solved = planner->solve(PTC);
		//record time
		double t2 = elapsedTime();
		timeSpentTotal += t2 - t1;
		//different from rrtMultiple
		timeSpentCC = planner->getCCTime();
		timeSpentNNQ = planner->getNNQTime();
		timeSpentRW = planner->getRWTime();
		if (solved) {
//            ob::PathPtr path = pdef->getGoal()->getSolutionPath();
			ob::PathPtr path = pdef->getSolutionPath();
			double pathlength = path->length();
			if(pathlength < bestPathCost){
				fprintf(SolutionInfoFile, "%05d\t%.8e\t%.8e\t%.8e\t%.8e\t%.8e\n",
						iter  * rrt_rrtstar_prm_samples, timeSpentTotal, timeSpentCC, timeSpentRW,
						timeSpentNNQ, pathlength);
				fflush(SolutionInfoFile);
				if(recordPath){
					path->print(SolutionPathFile);
					SolutionPathFile.flush();
				}
				bestPathCost = pathlength;
			}
		} else {
//			double max = -1;
//			fprintf(SolutionInfoFile, "%05d\t%.8e\t%.8e\t%.8e\t%.8e\t%.8e\n",
//					iter, timeSpentTotal, timeSpentCC, timeSpentRW,
//					timeSpentNNQ, bestPathCost);
//			fflush(SolutionInfoFile);
		}
		if (timeSpentTotal > cutoff_seconds){
//			if(!(solved)){
				fprintf(SolutionInfoFile, "%05d\t%.8e\t%.8e\t%.8e\t%.8e\t%.8e\n",
						iter  * rrt_rrtstar_prm_samples, timeSpentTotal, timeSpentCC, timeSpentRW,
						timeSpentNNQ, bestPathCost);
				fflush(SolutionInfoFile);
//			}
			break;
		}
		iter++;
	}

	if(recordPlannerData){
		ob::PlannerData Data(pdef->getSpaceInformation());
		planner->getPlannerData(Data);
		Data.printGraphviz(PlannerDataFile);
	}

	delete planner;

	fclose(SolutionInfoFile);
	SolutionPathFile.close();
	PlannerDataFile.close();
}

double CEPlan(char* pLengthDir, int runs, ob::ProblemDefinitionPtr &pdef) {
	pdef->clearSolutionPaths();
	//=============prepare out put files===========
	FILE* SolutionInfoFile;
	ofstream SolutionPathFile;
	ofstream PlannerDataFile;
	char pLengthName[100];
	sprintf(pLengthName, "%s/pathLength%04d.txt", pLengthDir, runs);
	SolutionInfoFile = fopen(pLengthName, "w");
	if(recordPath){
		sprintf(pLengthName, "%s/path%04d.txt", pLengthDir, runs);
		SolutionPathFile.open(pLengthName);
	}
	if(recordPlannerData){
		sprintf(pLengthName, "%s/plannerdata%04d.txt", pLengthDir, runs);
		PlannerDataFile.open(pLengthName);
	}
	fprintf(SolutionInfoFile, "%%"
			"totalTime\t"
			"CCTime_planner\t"
			"pathLength\t"
			"InitTime\n");
	int nD = pdef->getSpaceInformation()->getStateDimension();
	double cutoff_seconds = getCutoffSeconds(nD);
	//===========set up planner========================
	CEntropy * planner = new CEntropy(pdef);
	// set the problem we are trying to solve for the planner
	planner->setProblemDefinition(pdef);
	// perform setup steps for the planner
	Timer timer;
	planner->setup();
	double timeSpentTotal = timer.ElapsedTime();
	double bestPathCost = std::numeric_limits<double>::max();
	while(true){
		if (planner->flag != -1){
			break;
		}
		//solve the problem
		timer.Reset();
		ob::PlannerStatus solvestatus = planner->solve(PTC);
		timeSpentTotal += timer.ElapsedTime();

		double timeSpentCC_planner = planner->getColCheckTime();
		if (solvestatus) {
			double pathLength = planner->solution.length(planner->start, planner->goal);
			if(pathLength < bestPathCost){
				fprintf(SolutionInfoFile, "%.8e\t%.8e\t%.8e\t%.8e\n", timeSpentTotal,
						timeSpentCC_planner,
						pathLength, planner->initTime);
				fflush(SolutionInfoFile);
				if(recordPath){
					planner->printSolution(SolutionPathFile);
					SolutionPathFile.flush();
				}
				bestPathCost = pathLength;
			}
		} else {
			fprintf(SolutionInfoFile, "%.8e\t%.8e\t%.8e\t%.8e\n", timeSpentTotal,
					timeSpentCC_planner,
					bestPathCost, planner->initTime);
			fflush(SolutionInfoFile);

		}
		if (timeSpentTotal > cutoff_seconds){
			if(!(solvestatus)){
				fprintf(SolutionInfoFile, "%.8e\t%.8e\t%.8e\t%.8e\n", timeSpentTotal,
						timeSpentCC_planner,
						bestPathCost, planner->initTime);
				fflush(SolutionInfoFile);
			}
			break;
		}
	}
//	cout<<"# of states:"<<planner->nStatesPath<<endl;
//	cout<<"# of paths:"<<planner->nPaths<<endl;
//	cout<<"rho:"<<planner->rho<<endl;
	delete planner;

	fclose(SolutionInfoFile);
	SolutionPathFile.close();
	PlannerDataFile.close();
	return bestPathCost;
}

void PRMStarPlan(ScenarioIndex scenarioIndex, char* pLengthDir, int runs, ob::ProblemDefinitionPtr &pdef, ob::ScopedState<> &start, ob::ScopedState<> &goal, ob::StateValidityChecker* myChecker, bool lazy) {
	//=============prepare out put files===========
	FILE* SolutionInfoFile;
	ofstream SolutionPathFile;
	ofstream PlannerDataFile;
	char pLengthName[100];
	sprintf(pLengthName, "%s/pathLength%04d.txt", pLengthDir, runs);
	SolutionInfoFile = fopen(pLengthName, "w");
	if(recordPath){
		sprintf(pLengthName, "%s/path%04d.txt", pLengthDir, runs);
		SolutionPathFile.open(pLengthName);
	}
	if(recordPlannerData){
		sprintf(pLengthName, "%s/plannerdata%04d.txt", pLengthDir, runs);
		PlannerDataFile.open(pLengthName);
	}
	fprintf(SolutionInfoFile, "%%"
			"TotalTime\tCCTime\tKnnTime\tConnectTime\tLazyTime\tpathLength\n");
	//===========set up planner========================
	int nD = pdef->getSpaceInformation()->getStateDimension();
	double cutoff_seconds = getCutoffSeconds(nD);
	const ob::SpaceInformationPtr si_ = pdef->getSpaceInformation();
	double low, high;
	getScenarioBounds(scenarioIndex,low, high);
	MyCSpace space(nD, myChecker, si_);
	space.SetSpaceBounds(low, high);
	space.resolution = si_->getStateValidityCheckingResolution()*si_->getMaximumExtent();
//	cout<<"resolution :"<<space.resolution <<endl;
	PRMStarPlanner* planner = new PRMStarPlanner(&space);
	Config startConfig, goalConfig;
	state_to_config(nD, start, startConfig);
	state_to_config(nD, goal, goalConfig);
	planner->lazy = lazy;
	planner->Init(startConfig, goalConfig);

	Timer timer;
	double timeSpentTotal = 0;
	double bestPathCost = std::numeric_limits<double>::max();
	while(true){
		timer.Reset();
		for(int i = 0; i < rrt_rrtstar_prm_samples; i++)
			planner->PlanMore();
		timeSpentTotal += timer.ElapsedTime();
		MilestonePath path;
		bool solvestatus = planner->GetPath(path);
		if (solvestatus && ( (planner->lazy == true && planner->CheckPath(planner->start, planner->goal)) || lazy == false)) {
			double pathLength = path.Length();
			if(pathLength < bestPathCost){
				fprintf(SolutionInfoFile, "%.8e\t%.8e\t%.8e\t%.8e\t%.8e\t%.8e\n", timeSpentTotal,
						planner->tCheck,planner->tKnn,planner->tConnect,planner->tLazy,
						pathLength);
				fflush(SolutionInfoFile);
				if(recordPath){
					path.Save(SolutionPathFile);
					SolutionPathFile.flush();
				}
				bestPathCost = pathLength;
			}
		} else if(timeSpentTotal > cutoff_seconds){
			fprintf(SolutionInfoFile, "%.8e\t%.8e\t%.8e\t%.8e\t%.8e\t%.8e\n", timeSpentTotal,
					planner->tCheck,planner->tKnn,planner->tConnect,planner->tLazy,
					bestPathCost);
			fflush(SolutionInfoFile);
		}
		if (timeSpentTotal > cutoff_seconds)
			break;
	}

	delete planner;

	fclose(SolutionInfoFile);
	SolutionPathFile.close();
	PlannerDataFile.close();
}

void FMMPlan(char* pLengthDir, int runs, ob::ProblemDefinitionPtr &pdef, ob::ScopedState<> &start, ob::ScopedState<> &goal, ob::StateValidityChecker* myChecker, double thin, bool do_offset) {
	pdef->clearSolutionPaths();
	//===========set up planner========================
	FMM * planner = new FMM(pdef->getSpaceInformation(), start, goal);
	// set the problem we are trying to solve for the planner
	planner->setProblemDefinition(pdef);
	planner->setStateChecker(myChecker);
	double gridoffset = Math::Rand(FMMOffsetRangeMin,FMMOffsetRangeMax);
//	double gridoffset = 0.0146;
	if(do_offset){
		planner->setLowerBoundOffset(-gridoffset);
		planner->thin_of_narrow_passage = thin;
	}else
		gridoffset = 0;
	// perform setup steps for the planner
	planner->setup();
	//=============prepare out put files===========
	FILE* SolutionInfoFile;
	ofstream SolutionPathFile;
	ofstream PlannerDataFile;
	char pLengthName[100];
	sprintf(pLengthName, "%s/pathLength%04d.txt", pLengthDir, runs);
	SolutionInfoFile = fopen(pLengthName, "w");
	if(recordPath){
		sprintf(pLengthName, "%s/path%04d.txt", pLengthDir, runs);
		SolutionPathFile.open(pLengthName);
	}
	if(recordPlannerData){
		sprintf(pLengthName, "%s/plannerdata%04d.txt", pLengthDir, runs);
		PlannerDataFile.open(pLengthName);
	}

	fprintf(SolutionInfoFile, "%%"
			"totalTime\t"
			"CCTime_planner\t"
			"numGrid\t"
			"PathValid\t"
			"pathLength\t"
			"offset\t"
			"initTime"
			"estimateTime\t"
			"propagateTime\t"
			"overheadTime\n");
	sprintf(pLengthName,"%s/collisionpoints%04d.txt",pLengthDir,runs);
	ofstream collisionpointFile;collisionpointFile.open(pLengthName);

	int nD = pdef->getSpaceInformation()->getStateDimension();
	double cutoff_seconds = getCutoffSeconds(nD);
	double bestPathCost = std::numeric_limits<double>::max();
	bool bestPathValidFlag = false;
	//========prepare to run=======================
	//elapse time
	double timeSpentTotal = 0;
	while(true){
		//solve the problem
		Timer timer;
		ob::PlannerStatus solved = planner->solve(PTC);
		//record time
		timeSpentTotal += timer.ElapsedTime();

		double timeSpentCC_planner = planner->getColCheckTime();
		if (solved) {
			double pathLength = getSolutionPathLength(planner->solution);
			fprintf(SolutionInfoFile, "%.8e\t%.8e\t%d\t%d\t%.8e\t%.8e\t%.8e\t%.8e\t%.8e\t%.8e\n", timeSpentTotal,
					timeSpentCC_planner, planner->numGrid, planner->validflag,
					pathLength,gridoffset,planner->initTime,planner->estimateTime, planner->propagateTime,planner->overheadTime);
			fflush(SolutionInfoFile);
			if(recordPath){
				planner->printSolution(SolutionPathFile);
				SolutionPathFile.flush();
			}
			if(pathLength < bestPathCost){
				bestPathCost = pathLength;
				bestPathValidFlag = planner->validflag;
			}

			collisionpointFile << planner->collision_points.size()<<endl;
			for(int i = 0; i < planner->collision_points.size(); i++){
				collisionpointFile <<planner->collision_points[i]<<endl;
			}
			collisionpointFile <<endl;
			cout<<planner->numGrid<<endl;getchar();
		} else {
//			double pathLength = getSolutionPathLength(planner->solution);
//						fprintf(SolutionInfoFile, "%.8e\t%.8e\t%d\t%d\t%.8e\t%.8e\t%.8e\t%.8e\t%.8e\t%.8e\n", timeSpentTotal,
//								timeSpentCC_planner, planner->numGrid, -1,
//								pathLength,gridoffset,planner->initTime,planner->estimateTime, planner->propagateTime,planner->overheadTime);
//						fflush(SolutionInfoFile);
		}
		if (timeSpentTotal > cutoff_seconds){
			if(!(solved)){
				fprintf(SolutionInfoFile, "%.8e\t%.8e\t%d\t%d\t%.8e\t%.8e\t%.8e\t%.8e\t%.8e\t%.8e\n", timeSpentTotal,
						timeSpentCC_planner, planner->numGrid, bestPathValidFlag,
						bestPathCost,gridoffset,planner->initTime,planner->estimateTime, planner->propagateTime,planner->overheadTime);
				fflush(SolutionInfoFile);
			}
			break;
		}
	}

	delete planner;

	collisionpointFile.close();

	fclose(SolutionInfoFile);
	SolutionPathFile.close();
	PlannerDataFile.close();
}

//void FMMwithPerturbationPlan(ScenarioIndex scenario, int nD, double width, double thin, char* pLengthDir, int runs) {
//	//=============prepare out put files===========
//	FILE* SolutionInfoFile;
//	ofstream SolutionPathFile;
//	ofstream PlannerDataFile;
//	char pLengthName[100];
//	sprintf(pLengthName, "%s/pathLength%04d.txt", pLengthDir, runs);
//	SolutionInfoFile = fopen(pLengthName, "w");
//	if(recordPath){
//		sprintf(pLengthName, "%s/path%04d.txt", pLengthDir, runs);
//		SolutionPathFile.open(pLengthName);
//	}
//	if(recordPlannerData){
//		sprintf(pLengthName, "%s/plannerdata%04d.txt", pLengthDir, runs);
//		PlannerDataFile.open(pLengthName);
//	}
//	fprintf(SolutionInfoFile, "%%"
//			"totalTime\t"
//			"CCTime_planner\t"
//			"numGrid\t"
//			"PathValid\t"
//			"perturbValue\t"
//			"initTime\t"
//			"estimateTime\t"
//			"propagateTime\t"
//			"overheadTime\t"
//			"pathLength\n");
//	//=================construct space=================================
//	ob::StateSpacePtr space(new ob::RealVectorStateSpace(nD));
//	// set the bounds for the R^2
//	ob::RealVectorBounds bounds(nD);
//	double low,high;
//	getScenarioBounds(scenario, low, high);
//	bounds.setLow(low);
//	bounds.setHigh(high);
//	space->as<ob::RealVectorStateSpace>()->setBounds(bounds);
//	// construct an instance of  space information from this state space
//	ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
//	// set state validity checking for this space
//	ob::StateValidityChecker* myChecker = setupNarrowPassage(scenario, si, nD, width, thin);
//
//	// create a start state
//	ob::ScopedState<> start(space);
//	ob::ScopedState<> goal(space);
//	setupStartandGoal(scenario, nD, thin, start, goal);
//
//	si->setStateValidityChecker(ob::StateValidityCheckerPtr(myChecker));
//	si->setStateValidityCheckingResolution(
//			stateCheckResolution / si->getStateSpace()->getMaximumExtent());
//	si->setup();
//
//	ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
//	// set the start and goal states
//	//goal threshold
//	pdef->setStartAndGoalStates(start, goal);
//	//===========set up planner========================
//	FMM * planner = new FMM(si, start, goal);
//	// set the problem we are trying to solve for the planner
//	planner->setProblemDefinition(pdef);
//	// perform setup steps for the planner
//	planner->setup();
//	double gridoffset = Math::Rand(0,FMMOffsetRange);
//	planner->setLowerBoundOffset(gridoffset);
//	//========prepare to run=======================
//	double cutoff_seconds = getCutoffSeconds(nD);
//	double timeSpentTotal = 0;
//	double timeSpentCC = 0.0;
//	int lastGrid = 0;
//	while(true){
//
//		//solve the problem
//		Timer timer;
//		planner->lastnumGrid = lastGrid;
//		ob::PlannerStatus solved = planner->solve(PTC);
//		//record time
//		timeSpentTotal += timer.ElapsedTime();
//		timeSpentCC += planner->getColCheckTime();
//		double onesecond = onesecond_global;
//		if (solved) {
//			fprintf(SolutionInfoFile,
//					"%.8e\t%.8e\t%d\t%d\t%.8e\t%.8e\t%.8e\t%.8e\t%.8e\t%.8e\n",
//					timeSpentTotal, timeSpentCC, planner->numGrid,
//					planner->validflag, gridoffset, planner->initTime,
//					planner->estimateTime, planner->propagateTime,
//					planner->overheadTime,
//					getSolutionPathLength(planner->solution));
//			fflush(SolutionInfoFile);
//			if(recordPath){
//				planner->printSolution(SolutionPathFile);
//				SolutionPathFile.flush();
//			}
//		} else {
//		}
//
//		lastGrid = planner->numGrid;
//
//		if (timeSpentTotal > cutoff_seconds){
//			if(!(solved)){
//				fprintf(SolutionInfoFile,
//						"%.8e\t%.8e\t%d\t%d\t%.8e\t%.8e\t%.8e\t%.8e\t%.8e\t%.8e\n",
//						timeSpentTotal, timeSpentCC, planner->numGrid,
//						planner->validflag, gridoffset, planner->initTime,
//						planner->estimateTime, planner->propagateTime,
//						planner->overheadTime,
//						getSolutionPathLength(planner->solution));
//				fflush(SolutionInfoFile);
//				if(recordPath)
//					planner->printSolution(SolutionPathFile);
//			}
//			break;
//		}
//	}
//
//	delete planner;
//
//	fclose(SolutionInfoFile);
//	SolutionPathFile.close();
//	PlannerDataFile.close();
//}



