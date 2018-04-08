/*
 * File:   SetupEnvironment.h
 * Author: Jingru
 *
 * Created on March 13, 2012, 11:35 AM
 */

#ifndef READENVIRONMENT_H
#define	READENVIRONMENT_H
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstring>

#include <ompl/base/ScopedState.h>
#include <math/vector.h>
#include "Polyhedron.h"
#include "CEState.h"

namespace ob = ompl::base;

static const double lowbound = 0;
static const double highbound = 0;

//int nD_global = 2;
//int mD_global = 2*nD_global;
static const double width_global = 0.2;
static const double thin_global = 0.1;
static const double onefourth_global = 0.25;
static const double onesecond_global = 0.5;

static const double joint_start_5D[] = {120, -90, -80, -110, -120};
static const double joint_goal_5D[] = {179, 0, 0, 0, 0};

static const double joint_start_6D[] = {120, -90, -60, -110, -80, -130};
static const double joint_goal_6D[] = {179, 0, 0, 0, 0, 0};

static const double joint_start_7D[] = {150, -90, -50, -80, -90, -100, -130};
static const double joint_goal_7D[] = {179, 0, 0, 0, 0, 0, 0};

static const double joint_start_8D[] = {150, -90, -50, -80, -80, -80, -100, -125};
static const double joint_goal_8D[] = {179, 0, 0, 0, 0, 0, 0, 0};

static const int num_config_best_path_kink = 8;

enum ScenarioIndex{
	NarrowPassage_2_homotopy,
	NarrowPassage_1_homotopy,
	PlanaryLinkage,
	NarrowKink_1_homotopy
};
    
const double stateCheckResolution = 0.001;

double getBestPathCost(ScenarioIndex scenario, int nD, double width, double thin, double valuePerturb = 0);

double getBestPathCost_Linkage(int nD);

double getBestPathCost_1_Kink(int nD, double width, double thin, double valuePerturb = 0);

double getBestPathCost_2_homotopy(int nD, double width, double thin, double valuePerturb = 0);

double getBestPathCost_1_homotopy(int nD, double width, double thin, double valuePerturb = 0);

void getScenarioBounds(ScenarioIndex scenario, double &low, double &high);

ob::StateValidityChecker* setupNarrowPassage(ScenarioIndex scenario, ob::SpaceInformationPtr &si, const int &nD, const double &width, const double &thin, double valuePerturb = 0);

//width is the width of narrow passage, thin is the horizontal length of the horizontal narrow passage
vector<Polyhedron> setupNarrowPassage_1_Kink(const int &nD, const double &width, double thin, double valuePerturb = 0);

vector<Polyhedron> setupNarrowPassage_2_homotopy(const int &nD, const double &width, const double &thin, double valuePerturb = 0);

vector<Polyhedron> setupNarrowPassage_1_homotopy(const int &nD, const double &width, const double &thin, double valuePerturb = 0);

vector<Polyhedron> readNarrowPassage(const char* filename, CEState &sp, CEState &gp, int &D);

bool readFile(const char* filename, int &nD, double &width, double &thin);

void setupStartandGoal(ScenarioIndex scenario, const int &nD, const double &thin, ob::ScopedState<> &start, ob::ScopedState<> &goal);

#endif	/* READENVIRONMENT_H */

