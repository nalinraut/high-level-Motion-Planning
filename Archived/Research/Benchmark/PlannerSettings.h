/*
 * PlannerSettings.h
 *
 *  Created on: Jan 20, 2014
 *      Author: iuiml
 */

#ifndef PLANNERSETTINGS_H_
#define PLANNERSETTINGS_H_

#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include "SetupEnvironment.h"

namespace ob = ompl::base;

//#define Run_on_Odin

//termination condition to comply with ompl function
static ob::PlannerTerminationCondition PTC = ob::plannerAlwaysTerminatingCondition();
static const int MaxRank = 4;

#ifdef Run_on_Odin
///////////////////////////////////////////////////////////////////
//number of runs for each planner
static const int nRuns = 20;

static const double cutoff_seconds_2D = 60;
static const double cutoff_seconds_3D = 60;
static const double cutoff_seconds_HD = 60;
//static const int nRuns = 2;
//
//static const double cutoff_seconds_2D = 10;
//static const double cutoff_seconds_3D = 10;
//static const double cutoff_seconds_HD = 600;

static const int num_nD_to_test = 4;
static const int nD_to_test[] = {2,3,4,5};//{2,3,4,5};//{5,6,7,8};
static const int num_width_to_test = 1;//4;//2;
static const double width_to_test[] = {0.15};//{0.005, 0.01, 0.05, 0.1};//{0.1,0.15,0.2,0.24};//{0.05, 0.1};//{0.005};
static const int num_thin_to_test = 1;
static const double thin_to_test[] = {0.5};//{0.01, 0.1, 0.25, 0.5};//{0.3};//{0.25, 0.5};

static const int num_scenarios_to_test = 1;
static const ScenarioIndex scenarios_to_test[] = {NarrowKink_1_homotopy};//{NarrowKink_1_homotopy,PlanaryLinkage, NarrowPassage_2_homotopy, NarrowPassage_1_homotopy};

///////////////////////////////////////////////////////////////////

////maximum time of seconds allowed for each planner to run
//static const double cutoff_seconds = 10;
//
////number of runs for each planner
//static const int nRuns = 1;
//
//static const int num_nD_to_test = 1;
//static const int nD_to_test[] = {2};
//static const int num_width_to_test = 1;
//static const double width_to_test[] = {0.005};
//static const int num_thin_to_test = 1;
//static const double thin_to_test[] = {0.01};
//
//static const int num_scenarios_to_test = 2;
//static const ScenarioIndex scenarios_to_test[] = {NarrowKink_1_homotopy,NarrowPassage_2_homotopy, NarrowPassage_1_homotopy};

///////////////////////////////////////////////////////////////////

#else

//maximum time of seconds allowed for each planner to run
static const double cutoff_seconds_2D = 60;
static const double cutoff_seconds_3D = 60;
static const double cutoff_seconds_HD = 60;

//number of runs for each planner
static const int nRuns = 1;

static const int num_nD_to_test = 1;
static const int nD_to_test[] = {2};
static const int num_width_to_test = 1;
static const double width_to_test[] = {0.005};
static const int num_thin_to_test = 1;
static const double thin_to_test[] = {0.1};
static const int num_scenarios_to_test = 1;
static const ScenarioIndex scenarios_to_test[] = {NarrowPassage_1_homotopy};

#endif

static const bool runMRRT = true;
static const bool runMRRT_shortcut = true;
static const bool runRRTStar = false;
static const bool runPRMStar = false;
static const bool runPRMStar_Lazy = false;
static const bool runCE = false;
static const bool runFMM = false;
static const bool runFMMOffset = false;
static const bool runFMMPerturbation = false;

static const int nFMMRuns = 20;
static const bool recordPath = true;
static const bool recordPlannerData = false;

static const string mRRTFolder = "multipleRRT";
static const string mRRTFolder_shortcut = "multipleRRT_shortcut";
static const string RRTStarFolder = "RRTStar";
static const string CEFolder = "CrossEntropy";
static const string PRMStarFolder = "PRMStar";
static const string PRMStarFolder_Lazy = "PRMStar_Lazy";
static const string FMMFolder = "FMM";
static const string FMMPerturbationFolder = "FMMPerturbation";
static const string FMMOffsetFolder = "FMMOffset";

static const double FMMOffsetRangeMax = 0.05;
static const double FMMOffsetRangeMin = 0.01;

//maximum num of samples allowed for each RRT and RRTstar solve
static const int rrt_rrtstar_prm_samples = 500;

//maximum time allowed for CE to initialize valid path
static const double maxCEInitTime = 60;
//maximum time allowed for CE to generate valid path
static const double maxCEGeneratePathTime = 60;
////maximum iterations for Mutiple-RRT shortcut path
//static const int maxMRRTShortcut = 10;

//default link length for linkage scenario
static const double default_link_length = 1;

////maximum grids are allowed for each axis for FMM
//static const int maxFMMNumGrid = 4000;

////for PRM planner who has a straight line local planner with an epsilon value
//static const double prm_epsilon_loclalplanner = 0.0001;

#endif /* PLANNERSETTINGS_H_ */

