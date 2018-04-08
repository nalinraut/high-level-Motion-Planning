/*
 * benchmark.cpp
 *
 *  Created on: Jan 14, 2014
 *      Author: Jingru
 */

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
//#include <ompl/contrib/rrt_star/RRTstar.h>
#include <ompl/base/PlannerTerminationCondition.h>

#include "ompl/datastructures/NearestNeighbors.h"

#include <ompl/config.h>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <fstream>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#include <Timer.h>

#include "myRRT.h"
#include "myRRTstar.h"
#include "CEntropy.h"
#include "FMM.h"
#include "PolyhedronStateChecker.h"
#include "SetupEnvironment.h"
#include "Polyhedron.h"
#include "utility.h"
#include "PlannerEntrance.h"
#include "PlannerSettings.h"

#ifdef Run_on_Odin
#include <mpi.h>
#endif

using namespace std;

namespace ob = ompl::base;
namespace og = ompl::geometric;

void writeExperimentInfo(char *experimentDir, ScenarioIndex scenarioIndex) {
	char costFile[100];
	sprintf(costFile, "%s/best-path-cost.txt", experimentDir);
	ofstream file;
	file.open(costFile);
	file << "nD\twidth\tthin\tbest-path-length\n";
	for (int i = 0; i < num_nD_to_test; i++) {
		int nD = nD_to_test[i];
		for (int j = 0; j < num_width_to_test; j++) {
			double width = width_to_test[j];
			for (int k = 0; k < num_thin_to_test; k++) {
				double thin = thin_to_test[k];

				double bestpathcost = getBestPathCost(scenarioIndex, nD, width,	thin);
				file << nD << "\t" << width << "\t" << thin << "\t"	<< bestpathcost << endl;
			}
		}
	}
	file.close();

	char summaryfilename[100];
	sprintf(summaryfilename, "%s/summary.txt", experimentDir);
	file.open(summaryfilename);
	file << "cutoff_time_2D:" << cutoff_seconds_2D << endl;
	file << "cutoff_time_3D:" << cutoff_seconds_3D << endl;
	file << "cutoff_time_HD:" << cutoff_seconds_HD << endl;
	file << "nRuns:" << nRuns << endl;

	file << "num_nD_to_test:" << num_nD_to_test << endl;
	for (int i = 0; i < num_nD_to_test; i++) {
		file << nD_to_test[i] << ";";
	}
	file << endl;
	file << "num_width_to_test:" << num_width_to_test << endl;
	for (int i = 0; i < num_width_to_test; i++) {
		file << width_to_test[i] << ";";
	}
	file << endl;
	file << "num_thin_to_test:" << num_thin_to_test << endl;
	for (int i = 0; i < num_thin_to_test; i++) {
		file << thin_to_test[i] << ";";
	}
	file << endl;

	file << "planners that are running:"<<endl;
	file << "runMRRT:"<<runMRRT<<endl;
	file << "runMRRT_shortcut:"<<runMRRT_shortcut<<endl;
	file << "runRRTStar:"<<runRRTStar<<endl;
	file << "runPRMStar:"<<runPRMStar<<endl;
	file << "runPRMStar_Lazy:"<<runPRMStar_Lazy<<endl;
	file << "runCE:"<<runCE<<endl;
	file << "runFMM:"<<runFMM<<endl;
	file << "runFMMOffset:"<<runFMMOffset<<endl;
	file << "runFMMPerturbation:"<<runFMMPerturbation<<endl;

	file << "FMMOffsetRangeMax:"<<FMMOffsetRangeMax<<endl;
	file << "FMMOffsetRangeMin:"<<FMMOffsetRangeMin<<endl;
	file << "MaxRank:"<<MaxRank<<endl;

	if (scenarioIndex == NarrowPassage_2_homotopy)
		file << "scenarioindex:NarrowPassage_2_homotopy" << endl;
	else if (scenarioIndex == NarrowPassage_1_homotopy)
		file << "scenarioindex:NarrowPassage_1_homotopy" << endl;
	else if(scenarioIndex == PlanaryLinkage)
		file << "scenarioindex:PlanaryLinkage"<<endl;
	else if(scenarioIndex == NarrowKink_1_homotopy)
		file << "scenarioindex:NarrowKink_1_homotopy"<<endl;

	file << "rrt_rrtstar_samples:" << rrt_rrtstar_prm_samples << endl;
	file << "FMMOffsetRange:" << FMMOffsetRangeMax << endl;
	file << "maxCEInitTime:" << maxCEInitTime << endl;
	file << "maxCEGeneratePathTime:" << maxCEGeneratePathTime << endl;
	file << "nFMMRuns:" << nFMMRuns << endl;
	if(scenarioIndex == PlanaryLinkage){
		file << "joint_start_5D:"<<endl;
		for(int i = 0; i < 5; i++)
			file << joint_start_5D[i]<<";";
		file<<endl;

		file << "joint_start_6D:"<<endl;
		for(int i = 0; i < 6; i++)
			file << joint_start_6D[i]<<";";
		file<<endl;

		file << "joint_start_7D:"<<endl;
		for(int i = 0; i < 7; i++)
			file << joint_start_7D[i]<<";";
		file<<endl;

		file << "joint_start_8D:"<<endl;
		for(int i = 0; i < 8; i++)
			file << joint_start_8D[i]<<";";
		file<<endl;
	}

	file.close();

}

bool createTopDirs(char *experimentDir) {
	if(createDir(experimentDir) == false)
		return false;

	char topDir[100];
	sprintf(topDir, "%s/%s", experimentDir, mRRTFolder.c_str());
	if(createDir(topDir) == false)
		return false;
	sprintf(topDir, "%s/%s", experimentDir, mRRTFolder_shortcut.c_str());
	if(createDir(topDir) == false)
		return false;
	sprintf(topDir, "%s/%s", experimentDir, RRTStarFolder.c_str());
	if(createDir(topDir) == false)
		return false;
	sprintf(topDir, "%s/%s", experimentDir, CEFolder.c_str());
	if(createDir(topDir) == false)
		return false;
	sprintf(topDir, "%s/%s", experimentDir, PRMStarFolder.c_str());
	if(createDir(topDir) == false)
		return false;
	sprintf(topDir, "%s/%s", experimentDir, PRMStarFolder_Lazy.c_str());
	if(createDir(topDir) == false)
		return false;
	sprintf(topDir, "%s/%s", experimentDir, FMMFolder.c_str());
	if(createDir(topDir) == false)
		return false;
	sprintf(topDir, "%s/%s", experimentDir, FMMOffsetFolder.c_str());
	if(createDir(topDir) == false)
		return false;
	sprintf(topDir, "%s/%s", experimentDir, FMMPerturbationFolder.c_str());
	if(createDir(topDir) == false)
		return false;
	return true;
}

#ifdef Run_on_Odin

int main(int argc, char *argv[]) {
	int rank, size;
	char hname[HOST_NAME_MAX + 1];
	bool empty_task = false;

	gethostname(hname, HOST_NAME_MAX);
	MPI_Init(&argc, &argv);
	MPI_Comm_rank(MPI_COMM_WORLD, &rank); /* get this processor's unique ID               */
	MPI_Comm_size(MPI_COMM_WORLD, &size); /* get total number of processors available     */

//	if (rank > FMMPerturbationRank) {
//		printf("No task for Process rank %d of size %d on hname %s\n", rank, size, hname);
//		MPI_Finalize();
//	}
	double t1 = elapsedTime();

	assert(size >= MaxRank);

	if (rank > MaxRank) {
		cout << "No Task for Process hostname=" << hname << " rank=" << rank << endl;
		empty_task = true;
	}

	int myRankIndex = 0;

	srand(time(NULL));

	for(int l = 0; l < num_scenarios_to_test; l++){
		ScenarioIndex scenarioIndex = scenarios_to_test[l];

		//===================create folders and write experiment info=================
		char experimentDir[100];
		time_t t = time(0); // get time now
		struct tm * now = localtime(&t);
		sprintf(experimentDir, "Exp-Scen-%d-%d-%d-%d", scenarioIndex, now->tm_year + 1900,
				now->tm_mon + 1, now->tm_mday);

		if(createTopDirs(experimentDir) == false){
			empty_task = true;
		}
		if (rank == 0) {
			writeExperimentInfo(experimentDir,scenarioIndex);
		}

		if (empty_task == true) {
			break;
		}

		for (int i = 0; i < num_nD_to_test; i++) {
			int nD = nD_to_test[i];
			for (int j = 0; j < num_width_to_test; j++) {
				double width = width_to_test[j];
				for (int k = 0; k < num_thin_to_test; k++) {
					double thin = thin_to_test[k];
//						---------------set up problem definition--------------
					//construct space
					ob::StateSpacePtr space(new ob::RealVectorStateSpace(nD));
					// set the bounds for the R^2
					double low, high;
					getScenarioBounds(scenarioIndex,low, high);
					ob::RealVectorBounds bounds(nD);
					bounds.setLow(low);
					bounds.setHigh(high);
					space->as<ob::RealVectorStateSpace>()->setBounds(bounds);
					// construct an instance of  space information from this state space
					ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
					// set state validity checking for this space
					ob::StateValidityChecker* myChecker = setupNarrowPassage(scenarioIndex, si, nD, width, thin);

					// create a start state
					ob::ScopedState<> start(space);
					ob::ScopedState<> goal(space);
					setupStartandGoal(scenarioIndex, nD, thin, start, goal);

					si->setStateValidityChecker(
							ob::StateValidityCheckerPtr(myChecker));
					si->setStateValidityCheckingResolution(
							stateCheckResolution
									/ si->getStateSpace()->getMaximumExtent());
					si->setup();

					ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
					// set the start and goal states, goal region radius is set to zero
					pdef->setStartAndGoalStates(start, goal);

					if(myRankIndex == rank){
						cout<<myRankIndex<<" is running nD="<<nD<<", width="<<width<<",thin="<<thin<<",scenario="<<scenarioIndex<<"..."<<endl;

						char pLength[100];
						if(runMRRT){
							sprintf(pLength,"%s/%s/nD-%d-w-%.3f-t-%.3f",experimentDir, mRRTFolder.c_str(), nD, width, thin);
							if(createDir(pLength) == false)
								continue;
							for (int i = 1; i <= nRuns; i++) {
								RRTPlan(pLength, i, pdef, false);
							}
						}

						if(runMRRT_shortcut){
							sprintf(pLength,"%s/%s/nD-%d-w-%.3f-t-%.3f",experimentDir, mRRTFolder_shortcut.c_str(), nD, width, thin);
							if(createDir(pLength) == false)
								continue;
							for (int i = 1; i <= nRuns; i++) {
								RRTPlan(pLength, i, pdef, true);
							}
						}

						if(runRRTStar){
							sprintf(pLength,"%s/%s/nD-%d-w-%.3f-t-%.3f",experimentDir, RRTStarFolder.c_str(), nD, width, thin);
							if(createDir(pLength) == false)
								continue;
							for (int i = 1; i <= nRuns; i++) {
								RRTStarPlan(pLength, i, pdef);
							}
						}

						if(runPRMStar){
							sprintf(pLength,"%s/%s/nD-%d-w-%.3f-t-%.3f",experimentDir, PRMStarFolder.c_str(), nD, width, thin);
							if(createDir(pLength) == false)
								continue;
							for (int i = 1; i <= nRuns; i++) {
								PRMStarPlan(scenarioIndex, pLength, i, pdef, start, goal, myChecker, false);
							}
						}

						if(runPRMStar_Lazy){
							sprintf(pLength,"%s/%s/nD-%d-w-%.3f-t-%.3f",experimentDir, PRMStarFolder_Lazy.c_str(), nD, width, thin);
							if(createDir(pLength) == false)
								continue;
							for (int i = 1; i <= nRuns; i++) {
								PRMStarPlan(scenarioIndex, pLength, i, pdef, start, goal, myChecker, true);
							}
						}

						if(runCE){
							sprintf(pLength,"%s/%s/nD-%d-w-%.3f-t-%.3f",experimentDir, CEFolder.c_str(), nD, width, thin);
							if(createDir(pLength) == false)
								continue;
							for (int i = 1; i <= nRuns; i++) {
								CEPlan(pLength, i, pdef, start, goal, myChecker);
							}
						}
						if(runFMM){
							sprintf(pLength,"%s/%s/nD-%d-w-%.3f-t-%.3f",experimentDir, FMMFolder.c_str(), nD, width, thin);
							if(createDir(pLength) == false)
								continue;
							for (int i = 1; i <= nFMMRuns; i++) {
								bool do_offset = false;
								FMMPlan(pLength,i, pdef, start, goal, myChecker,thin,do_offset);							}
						}

						if(runFMMOffset){
							sprintf(pLength,"%s/%s/nD-%d-w-%.3f-t-%.3f",experimentDir, FMMOffsetFolder.c_str(), nD, width, thin);
							if(createDir(pLength) == false)
								continue;
							for (int i = 1; i <= nFMMRuns; i++) {
								bool do_offset = true;
								FMMPlan(pLength,i, pdef, start, goal, myChecker,thin,do_offset);							}
						}

//						if(runFMMPerturbation){
//							sprintf(pLength,"%s/%s/nD-%d-w-%.3f-t-%.3f",experimentDir, FMMPerturbationFolder.c_str(), nD, width, thin);
//							if(createDir(pLength) == false)
//								continue;
//							for (int i = 1; i <= nFMMRuns; i++) {
//								FMMwithPerturbationPlan(scenarioIndex, nD, width, thin, pLength, i);
//							}
//						}
					}
					myRankIndex++;
				}
			}
		}
	}

	double t2 = elapsedTime();
	cout << "Process hostname=" << hname << " rank=" << rank << " used time:" << t2 - t1 << "(seconds) = " << (t2 - t1) / 3600.0 << "(hours)" << endl;
	cout << "=====================Main Done!=====================" << endl;
	MPI_Finalize();
}

#else
int main() {
//	testLinkage();return 0;
	//===================create directories=================
	srand(time(NULL));
	double t1 = elapsedTime();

	for(int l = 0; l < num_scenarios_to_test; l++){
		ScenarioIndex scenarioIndex = scenarios_to_test[l];
		time_t t = time(0);// get time now
		struct tm * now = localtime( & t );
		char experimentDir[100];
		sprintf(experimentDir, "Experiments-%d-%d-%d",now->tm_year+1900, now->tm_mon+1, now->tm_mday);
		createTopDirs(experimentDir);
		//===================write experiment info=================
		writeExperimentInfo(experimentDir,scenarioIndex);
		//===================start experiment=================
		for (int i = 0; i < num_nD_to_test; i++) {
			int nD = nD_to_test[i];
			for (int j = 0; j < num_width_to_test; j++) {
				double width = width_to_test[j];
				for (int k = 0; k < num_thin_to_test; k++) {
					double thin = thin_to_test[k];
					//---------------set up problem definition--------------
					//construct space
					ob::StateSpacePtr space(new ob::RealVectorStateSpace(nD));
					// set the bounds for the R^2
					double low, high;
					getScenarioBounds(scenarioIndex,low, high);
					ob::RealVectorBounds bounds(nD);
					bounds.setLow(low);
					bounds.setHigh(high);
					space->as<ob::RealVectorStateSpace>()->setBounds(bounds);
					// construct an instance of  space information from this state space
					ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
					// set state validity checking for this space
//					ob::StateValidityChecker* myChecker = setupNarrowPassage(scenarioIndex, si, nD, width, thin);
					double perturbation = 0.125;
					ob::StateValidityChecker* myChecker = setupNarrowPassage(scenarioIndex, si, nD, width, thin,perturbation);

					// create a start state
					ob::ScopedState<> start(space);
					ob::ScopedState<> goal(space);
					setupStartandGoal(scenarioIndex, nD, thin, start, goal);

					si->setStateValidityChecker(ob::StateValidityCheckerPtr(myChecker));
					si->setStateValidityCheckingResolution(
							stateCheckResolution / si->getStateSpace()->getMaximumExtent());
					si->setup();

					ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
					// set the start and goal states, goal region radius is set to zero
					pdef->setStartAndGoalStates(start, goal);

					//////////////////////////////////////////////////////////////////////
					char pLength[100];
//					//---------------rrt multiple---------------------------
//					sprintf(pLength,"%s/%s/nD-%d-w-%.3f-t-%.3f",experimentDir, mRRTFolder.c_str(), nD, width, thin);
//					createDir(pLength);
//					double avg = 0;
//					for (int i = 1; i <= nRuns; i++) {
//						std::cout<<"in multiple-rrt, this is runs:"<<i<<"; nD:"<<nD<<";width:"<<width<<";thin:"<<thin<<std::endl;
//						avg += RRTPlan(pLength,i, pdef, true);
//					}
//					cout<<"avg:"<<avg/(double)nRuns<<endl;
//					//---------------rrt star ---------------------------
//					sprintf(pLength,"%s/%s/nD-%d-w-%.3f-t-%.3f",experimentDir, RRTStarFolder.c_str(), nD, width, thin);
//					createDir(pLength);
//					for (int i = 1; i <= nRuns; i++) {
//						std::cout<<"in rrt-star, this is runs:"<<i<<"; nD:"<<nD<<";width:"<<width<<";thin:"<<thin<<std::endl;
//						RRTStarPlan(pLength, i, pdef);
//					}
//					//----------------Cross Entropy-------------------------
//					sprintf(pLength,"%s/%s/nD-%d-w-%.3f-t-%.3f",experimentDir, CEFolder.c_str(), nD, width, thin);
//					createDir(pLength);
//					double avg = 0;
//					for (int i = 1; i <= nRuns; i++) {
//						std::cout<<"in cross entropy, this is runs:"<<i<<"; nD:"<<nD<<";width:"<<width<<";thin:"<<thin<<std::endl;
//						avg += CEPlan(pLength, i, pdef, start, goal, myChecker);
//						cout<<"avg:"<<avg/(double)i<<endl;
//					}
//					cout<<"avg:"<<avg/(double)nRuns<<endl;
				//----------------PRM star-------------------------
					sprintf(pLength,"%s/%s/nD-%d-w-%.3f-t-%.3f",experimentDir, PRMStarFolder.c_str(), nD, width, thin);
					createDir(pLength);
					for (int i = 1; i <= nRuns; i++) {
						std::cout<<"in prm-star, this is runs:"<<i<<"; nD:"<<nD<<";width:"<<width<<";thin:"<<thin<<std::endl;
						PRMStarPlan(scenarioIndex, pLength, i, pdef, start, goal, myChecker,false);
					}
//					//----------------PRM star-------------------------
//					sprintf(pLength,"%s/%s/nD-%d-w-%.3f-t-%.3f",experimentDir, PRMStarFolder_Lazy.c_str(), nD, width, thin);
//					createDir(pLength);
//					for (int i = 1; i <= nRuns; i++) {
//						std::cout<<"in prm-star-lazy, this is runs:"<<i<<"; nD:"<<nD<<";width:"<<width<<";thin:"<<thin<<std::endl;
//						PRMStarPlan(scenarioIndex, pLength, i, pdef, start, goal, myChecker,true);
//					}
//					//----------------FMM without offset-------------------------
//					sprintf(pLength,"%s/%s/nD-%d-w-%.3f-t-%.3f",experimentDir, FMMFolder.c_str(), nD, width, thin);
//					createDir(pLength);
//					for (int i = 1; i <= 1; i++) {
//						std::cout<<"in fmm, this is runs:"<<i<<"; nD:"<<nD<<";width:"<<width<<";thin:"<<thin<<std::endl;
//						FMMPlan(pLength,i, pdef, start, goal, myChecker,thin,false);
//					}
//					//----------------FMM with offset-------------------------
//					sprintf(pLength,"%s/%s/nD-%d-w-%.3f-t-%.3f",experimentDir, FMMOffsetFolder.c_str(), nD, width, thin);
//					createDir(pLength);
//					for (int i = 1; i <= nFMMRuns; i++) {
//						std::cout<<"in fmm-offset, this is runs:"<<i<<"; nD:"<<nD<<";width:"<<width<<";thin:"<<thin<<std::endl;
//						bool do_offset = true;
//						if(scenarioIndex == PlanaryLinkage)
//							do_offset = false;
//						FMMPlan(pLength,i, pdef, start, goal, myChecker,thin,do_offset);
//					}
				}
			}
		}
	}
	double t2 = elapsedTime();
	cout<<"Used time:"<<t2-t1<<"(seconds) = "<<(t2-t1)/3600.0<<"(hours)"<<endl;
	cout<<"=====================All Done!====================="<<endl;
	return 0;
}
#endif


