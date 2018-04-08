/* 
 * File:   CEntropy.cpp
 * Author: Jingru
 * 
 * Created on February 22, 2012, 12:31 PM
 */

#include "CEntropy.h"
#include "CEState.h"
#include "CEPath.h"
#include <math/vector.h>
#include <cstdlib>
#include "utility.h"
#include <limits>
#include <time.h>
#include <math/vector.h>
#include<math.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include "PolyhedronStateChecker.h"
#include "PlannerSettings.h"
#include <Timer.h>

using namespace std;

CEntropy::CEntropy(CSpace* space) 
{
    K = 1;
    numD = ob::Planner::si_->getStateDimension();
    //for scenario NarrowPassage_2_homotopy
//    nStatesPath = 3;
//    rho = 0.05;
//    nPaths = (int)(2*numD*nStatesPath*K/rho);
    //for scenario NarrowPassage_1_homotopy
//    nStatesPath = 3;
//    rho = 0.5;
//    nPaths = 10;//(numD*nStatesPath*K/rho);
    //for NarrowKink_1_homotopy
    nStatesPath = 6;
    rho = 0.8;
    nPaths = 5;

    gama = 0;
    minGama = 0;
    maxGMMIter = 100;
    resolution = 0.001;
    maxIter = 1;
    flag = -1;
    tol = 1.0e-20;
    GMM.Resize(K, nStatesPath*numD);
    colcheckTime = 0;
    initTime = 0;

    initSuccess = false;
}

CEPath CEntropy::getSolutionPath()
{
    CEPath cepath(2+this->nStatesPath);
    cepath.states[0] = this->start;
    for(int i = 0; i < this->nStatesPath; i++)
    {
        cepath.states[i+1] = bestPath.states[i];
    }
    cepath.states[nStatesPath+1] = this->goal;
    return cepath;
}


bool CEntropy::solve(int mIter,double tol)
{    
  if(nPaths != paths.size())
    if(flag != -1)
    {
        cout<<"CE converge after maxIter="<<maxIter<<endl;
        cout<<"current solution is stored."<<endl;
        return true;
    }         

    double lastCost = numD*numD;
    for(int i = 0; i < mIter; i++)
    {
//        cout<<"*************iter:"<<i<<"*************"<<endl;
        getEliteSet();
        const CEPath& solution = *eliteset[0];
        ///////////////////////////////use solution diff < tol for termination
        double curCost = this->pathCost(solution);
        if(abs(lastCost - curCost) < tol)
        {
            cout<<"find a solution with difference less than tolerance "<<tol<<endl;
            flag = 0;
            break;
        }
        lastCost = curCost;
        ///////////////////////////////
//        trainGMM();
        computeMeanCov();
        ///////////////////////////////        
//        if(gama <= minGama)
//        {
//            cout<<"find a solution with cost less than minGama."<<endl;
//            cout<<"gama="<<gama<<" <= minGama="<<minGama<<endl;
//            flag = 1;
//            break;
//        }

        /////////////////////////////// count the no. of zeros in diagonal of covariance       
        if(isGaussianToZero())
        {
            flag = 2;
//			cout<<"converged isGaussianToZero"<<endl;
            break;
        }
        ///////////////////////////////use determinant of covariance for termination       
//        double det = GMM.gaussians[0].L.diagonalProduct();
//        if(det <= tol)
//        {
//            cout<<"Determinant of covariance drops within tolerance tol="<<tol<<endl;
//            break;
//        }
        if(generatePaths() == false)
        	return false;
    }

//    cout<<"found solution of cost="<<pathCost(solution)<<endl;
//    solution.print();
    return true;
}

bool CEntropy::isGaussianToZero()
{
        int nzero = 0;
        for(int j=0;j<numD*nStatesPath;j++) 
        {
            if(GMM.gaussians[0].L(j,j) <= 1e-3) 
            {
              nzero++;
              GMM.gaussians[0].L(j,j) = 1e-3;            }
        }
        if(nzero == numD*nStatesPath) 
        {
//            printf("Gaussian dropped to zero\n");
            return true;
        } 
        return false;
}
    
void CEntropy::initializePara()
{
//    cout<<"rho:"<<rho<<"; nStatesPath:"<<nStatesPath<<"; nPath:"<<nPaths<<"
bool CEntropy::initializePaths()
{
    assert(nPaths > 0);
    paths.clear();
    Timer timer;
    while(paths.size() < nPaths){
    	double usedtime = timer.ElapsedTime();
    	if(usedtime > maxCEInitTime && paths.size() < nPaths){
    		initTime += usedtime;
    		return false;
    	}
		CEPath path(nStatesPath);
		for(int k = 0; k < nStatesPath; k++)
		{
		  for(int i=0;i<50;i++) {
		    space->Sample(path.states[k]);
		    if(space->IsFeasible(path.states[k])) break;
		}
		if(this->isPathValid(path))
			paths.push_back(path);
    }
    initTime += timer.ElapsedTime();
    return true;
}

void CEntropy::setEnvironment(int nd, double l, double h)
{
    numD = nd;
    low = l;
    high = h;
}

void CEntropy::setTrajPara(int numTrajs, int numStatesInTraj)
{
    nPaths = numTrajs;
    nStatesPath = numStatesInTraj;
}

void CEntropy::setStartandGoalStates(CEState &s, CEState &g)
{
    assert(s.q.size() == numD);
    assert(s.q.size() == g.q.size());
    start = s;
    goal = g;
}

void CEntropy::setStartandGoalStates(const ob::ScopedState<> &s, const ob::ScopedState<> &g)
{
    start.q = s.reals();
    goal.q = g.reals();
}

void CEntropy::setGMM(int k, int d)
{
    GMM.Resize(k, d);
}

void CEntropy::setStandardGMM()
{
    Math::Matrix sigma(numD*nStatesPath,numD*nStatesPath);
    sigma.setIdentity();
    for(int i = 0; i < K; i++)
    {
        GMM.gaussians[i].setCovariance(sigma, 1);
    }
}


bool CEntropy::isPathValid(CEPath& path)
{
  Timer timer;
    int size = path.states.size();
    for(int i = 0; i < size; i++)
    {
        if(!space->IsFeasible(path.states[i]))
        {
            this->colcheckTime += timer.ElapsedTime();
            return false;
        }
    }
    CEState* last = &start;
    for(int i=0;i<size; i++) {
      SmartPointer<EdgePlanner> e = space->LocalPlanner(*last,path.states[i]);
      if(!e->IsVisible()) {
	this->colcheckTime += timer.ElapsedTime();
	return false;
      }
      last = &path.states[i];
    }
    SmartPointer<EdgePlanner> e = space->LocalPlanner(*last,goal);
    if( !e->IsVisible())   {
      this->colcheckTime += timer.ElapsedTime();
      return false;
    }
    this->colcheckTime += timer.ElapsedTime();
    return true;
}

double CEntropy::getColCheckTime()
{
    return colcheckTime;
}

//use the distance of the path as cost temporarily
double CEntropy::pathCost(CEPath& traj)
{
  return traj.length(start,goal);
}

void CEntropy::getEliteSet()
{
    int eliteSize = (int)(rho*nPaths) + 2;
    double costs[eliteSize];
    eliteset.clear();
    eliteset.resize(eliteSize);
    CEPath* pt = &paths[0];
    eliteset[0] = pt;
    costs[0] = pathCost(*pt);
    for(int i = 1; i < nPaths; i++)
    {
        double cost = pathCost(paths[i]);
        int last = (i > eliteSize) ? eliteSize:i;
        CEPath* pt1 = &paths[i];
        int j;
        for(j = last-1; j >= 0; j--)
        {           
            if(cost < costs[j])
            {
                if(j+1 < eliteSize)
                {
                    eliteset[j+1] = eliteset[j];
                    costs[j+1] = costs[j];
                    if(j == 0)
                    {
                        eliteset[0] = pt1;
                        costs[0] = cost;
                    }
                }
            }else{
                if(j+1 < eliteSize)
                {
                    eliteset[j+1] = pt1;
                    costs[j+1] = cost; 
                }
                break;
            }   
        }
    }
    gama = costs[eliteSize-1];
//    cout<<"gama="<<gama<<endl;
//    cout<<"out"<<endl;
//        cout<<"trajs size="<<trajs.size()<<endl;
//    for(int i = 0; i < trajs.size(); i++)
//    {
//        cout<<i<<":"<<trajCost(trajs[i])<<endl;
//        trajs[i].print();
//    }
//    cout<<"elitset size="<<elitset.size()<<endl;
//    for(int i = 0; i < elitSize; i++)
//    {
//        cout<<i<<":"<<trajCost(*elitset[i])<<endl;
//    }
}

bool CEntropy::trainGMM()
{
    //  bool TrainEM(const std::vector<Vector>& examples,Real& tol,int maxIters,int verbose=0);
    vector<Math::Vector> examples;
    int eliteSize = eliteset.size();
    for(int i = 0; i < eliteSize; i++)
    {
        Math::Vector v;
        eliteset[i]->getPathtoVector(v);
        examples.push_back(v);
    }

    GMM.Resize(K,numD*nStatesPath);
    Math::Matrix sigma(numD*nStatesPath,numD*nStatesPath);
    sigma.setIdentity();
    for(int i = 0; i < K; i++)
    {
        GMM.gaussians[i].setCovariance(sigma, 1);
    }
    Math::Real tol = 0.1;
    bool res = GMM.TrainEM(examples,tol,10,1);
//    cout<<"TrainEM"<<endl;
//    cout<<"mean"<<endl;
//    Math::Vector mean;
//    GMM.GetMean(mean);
//    cout<<mean<<endl;
//    Math::Matrix cov;
//    GMM.GetCovariance(cov);
//    cout<<"covariance"<<endl;
//    cout<<cov<<endl;
    return res;
}

void CEntropy::computeMeanCov()
{
    int dim = numD*nStatesPath;
    Math::Vector mean(dim);
    mean.setZero();
    int eliteSize = eliteset.size();
//    cout<<"eliteSize="<<eliteSize<<endl;
    vector<Math::Vector> examples;
    for(int i = 0; i < eliteSize; i++)
    { 
        Math::Vector v;
        eliteset[i]->getPathtoVector(v);
        examples.push_back(v);
        mean += v;
    }
    mean /= eliteSize;
    
    Math::Matrix cov(dim,dim);
    cov.setZero();
    for(int i = 0; i < eliteSize; i++)
    {
        Math::Vector v = examples[i];
        Math::Vector tmp;
        tmp = v - mean;
        Math::Matrix m(dim,dim);
        for(int r = 0; r < dim; r++)
        {
            for(int c = r; c < dim; c++)
            {
                m(r,c) = tmp[r]*tmp[c];
                m(c,r) = m(r,c);
            }
        }
        cov += m;
    }
    cov /= eliteSize;
    
    GMM.Resize(K,numD*nStatesPath);
    assert(GMM.gaussians.size() == 1);
    GMM.gaussians[0].setMean(mean);
    GMM.gaussians[0].setCovariance(cov);
}
//void CEntropy::generatePaths()
//{
//    paths.clear();
//    paths.resize(nPaths);
//    for(int i = 0; i < nPaths; i++)
//    {
////        cout<<"generate traj i="<<i<<endl;
//        Math::Vector sample(numD*nStatesPath);
//        GMM.Generate(sample);
//        CEPath pt;
//        pt.setPathfromVector(sample,nStatesPath);
//        while(!this->isPathValid(pt))
//        {
//            GMM.Generate(sample);
//            pt.setPathfromVector(sample,nStatesPath);
//        }
//        paths[i] = pt;
//    }
//}

bool CEntropy::generatePaths()
{
	Timer timer;
    paths.clear();
    while(paths.size() < nPaths){
    	if(timer.ElapsedTime() > maxCEGeneratePathTime)
    		return false;
        Math::Vector sample(numD*nStatesPath);
        GMM.Generate(sample);
        CEPath pt;
        pt.setPathfromVector(sample,nStatesPath);
        if(this->isPathValid(pt)){
        	paths.push_back(pt);
        }
    }
    return true;
}
