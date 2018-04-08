/* 
 * File:   CEntropy.h
 * Author: Jingru
 *
 * Created on February 22, 2012, 12:31 PM
 */

#ifndef CENTROPY_H
#define	CENTROPY_H
#include <planning/CSpace.h>
#include <statistics/GaussianMixtureModel.h>
#include "CEPath.h"
#include <time.h>

class CrossEntropyPlanner {
public:
    CrossEntropyPlanner(CSpace* cspace);
    virtual ~CrossEntropyPlanner(void) {}

    void setTrajPara(int numTrajs, int numStatesInTraj);
    void setStartandGoalStates(Config &s, Config &g);
    void setGMM(int k, int d);
    void setStandardGMM();

    bool checkStartandGoal();
    void initializePara();
    
    bool solve(int maxIter = 1, double tol = 1e-4);
    
    bool isPathValid(const CEPath &traj);

    void sampleState(Config &s);
    void sampleRandomState(Config &s);
    double pathCost(CEPath &traj);
    void getEliteSet();
    bool trainGMM();
    bool isGaussianToZero();
    bool generatePaths();

    void computeMeanCov();

    double getColCheckTime();
    CEPath getSolution();
    
    // atrributes     
    int flag; //solution converges according to the termination conditions if flag != -1

    CSpace* space;
    Config start; //start state
    Config goal; //goal state
    int nStatesPath; //number of states in a trajectory
    int nPaths; //number of trajectories
    int K; //number of GMM components
    int maxGMMIter; //max number of iterations for GMM EM training
    int maxIter; //max number of iterations for CE method
    double tol; //det tolerance for CE method termination 
    
    double rho; //constant for computing the elite set 
    double gama; //rare event level Cost(rho*N) = gama
    double minGama; //termination when a solution of cost less than minGama is found
    Statistics::GaussianMixtureModel GMM; //GaussianMixtureModel

    double colcheckTime, initTime;

    vector<CEPath> paths; //set of path sampled
    vector<CEPath*> eliteset; //elite set
    CEPath bestPath;
};

#endif	/* CENTROPY_H */

