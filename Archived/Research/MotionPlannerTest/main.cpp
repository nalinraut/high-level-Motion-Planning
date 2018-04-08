#include "MotionPlannerProgram.h"
#include <KrisLibrary/math3d/Circle2D.h>
#include <KrisLibrary/math/random.h>
#include <KrisLibrary/math/sample.h>
#include <KrisLibrary/math/SelfTest.h>
#include <KrisLibrary/planning/Grid2DCSpace.h>
#include <KrisLibrary/optimization/BlackBoxOptimization.h>
#include <KrisLibrary/math/vectorfunction.h>
//#include <planning/CliqueFMM.h>
#include "XmlReader.h"
#include "Callbacks.h"
//#include "CSpaceAnalysisProgram.h"
//#include "SeedSamplingTest.h"
//#include "MDPProgram.h"
#include "CarProgram.h"
#include "PendulumProgram.h"
#include "DoubleIntegratorProgram.h"
#include "TreePlannerProgram.h"
//#include "ClosedChainPlannerProgram.h"
//#include "MultiModalProgram.h"
//#include "EndgamePlannerProgram.h"
//#include "PRMPlannerProgram.h"
#include "ExplainingPlannerProgram.h"
//#include "DisplacementPlannerProgram.h"
//#include "SafestPathPlannerProgram.h"
//#include "POProgram.h"
#include "OptimalPlannerProgram.h"
//#include "JumpPlannerProgram.h"
#include <KrisLibrary/math/interpolate.h>
#include <KrisLibrary/Timer.h>
#include <iostream>
using namespace std;

/*

//#include <AI/FuzzyPlanning.h>

struct InvCoshFuzzyPlanner : public FuzzyPlannerBase
{
  InvCoshFuzzyPlanner(Real c,Real max=1.0):coeff(c),maxVal(max) {}
  virtual Real ProbabilityPerIters(int iters) { return maxVal*(1.0-1.0/cosh(coeff*iters)); }
    
  Real coeff,maxVal;
};

struct ExpFuzzyPlanner : public FuzzyPlannerBase
{
  ExpFuzzyPlanner(Real c,Real p=1.0,Real max=1.0):coeff(c),power(p),maxVal(max) {}
  virtual Real ProbabilityPerIters(int iters) { return maxVal*(1.0-Exp(-coeff*Pow(Real(iters),power))); }
    
  Real coeff,power,maxVal;
};


void FuzzyPlanningTest()
{
  //CoinFlipFuzzyPlanner p1(0.5),p2(0.5);
  ExpFuzzyPlanner p1(0.004,2.0,0.8);
  CoinFlipFuzzyPlanner p2(0.03);
  FuzzyPlanningORScheduler sched;
  sched.planners.resize(2);
  sched.planners[0] = &p1;
  sched.planners[1] = &p2;
  sched.Init();
  for(int iters=0;iters<100;iters++) {
    pair<int,int> res=sched.ChooseOptimal();
    cout<<"****** Iter "<<iters+1<<":              ********"<<endl;
    cout<<"****** choice: "<<res.first<<", iters "<<res.second<<"*******"<<endl;
    cout<<"Evals: ";
    for(size_t i=0;i<sched.profileEvalCounts.size();i++)
      cout<<sched.profileEvalCounts[i]<<" ";
    cout<<endl;
    getchar();
    bool succ=sched.ExecuteTest(res.first,res.second);
  }
}
*/

//const Real narrowPassageLength = 0.3;
const Real narrowPassageLength = 0.45;
const Real narrowPassageWidth = 0.02;
//const Real narrowPassageWidth = 0.01;
//const Real narrowPassageWidth = 0.005;
//const Real narrowPassageWidth = 0.1;
const Real kinkLength = 0.1;
const Real bypassWidth = 0.0;
const bool boxObstacle = true;
const bool triangular = false;
const bool circular = false;
const bool circleField = false;
const bool borderL = false;
const bool kink = false;
const bool windy = false;
const bool passage = false;
const int numWinds = 3;
const int numCircles = 4;

inline void SetupObstacles(Geometric2DCollection& obstacles)
{
  const Real border = 0.02;
  AABB2D temp;
  temp.bmin.set(0,0);
  temp.bmax.set(border,1);
  obstacles.Add(temp);
  temp.bmin.set(0,0);
  temp.bmax.set(1,border);
  obstacles.Add(temp);
  temp.bmin.set(1-border,0);
  temp.bmax.set(1,1);
  obstacles.Add(temp);
  temp.bmin.set(0,1-border);
  temp.bmax.set(1,1);
  obstacles.Add(temp);
  
  if(boxObstacle) {
    temp.bmin.set(0.5-0.5*narrowPassageLength,0.5-0.5*narrowPassageWidth);
    temp.bmax.set(0.5+0.5*narrowPassageLength,0.5+0.5*narrowPassageWidth);
    obstacles.Add(temp);
  }
  else if(triangular) {
    Triangle2D t;
    t.a.set(0.5+narrowPassageLength*0.5,0.0);
    t.b.set(0.5,0.5-narrowPassageWidth*0.5);
    t.c.set(0.5-narrowPassageLength*0.5,0.0);
    obstacles.Add(t);
    t.a.set(0.5-narrowPassageLength*0.5,1);
    t.b.set(0.5,0.5+narrowPassageWidth*0.5);
    t.c.set(0.5+narrowPassageLength*0.5,1);
    obstacles.Add(t);
  }
  else if(circular) {
    Circle2D c;
    c.center.set(0.5,0.5-narrowPassageWidth*0.5-narrowPassageLength*0.5);
    c.radius = narrowPassageLength*0.5;
    obstacles.Add(c);
    c.center.set(0.5,0.5+narrowPassageWidth*0.5+narrowPassageLength*0.5);
    c.radius = narrowPassageLength*0.5;
    obstacles.Add(c);
    
    temp.bmin.set(0.5-0.5*narrowPassageLength,0);
    temp.bmax.set(0.5+0.5*narrowPassageLength,0.5-0.5*narrowPassageWidth-narrowPassageLength*0.5);
    obstacles.Add(temp);
    temp.bmin.set(0.5-0.5*narrowPassageLength,0.5+0.5*narrowPassageWidth+0.5*narrowPassageLength);
    temp.bmax.set(0.5+0.5*narrowPassageLength,1);
    obstacles.Add(temp);
  }
  else if(circleField) {
    Circle2D c;
    Real width = (1.0-2.0*narrowPassageWidth-2.0*border)/numCircles;
    Real start = border+width*0.5+narrowPassageWidth;
    c.radius = narrowPassageLength*width;
    for(int i=0;i<numCircles;i++) {
      if(i%2==0) {
	for(int j=0;j+1<numCircles;j++) {
	  c.center.set(width*(Real(j)+0.5)+start,width*Real(i)+start);
	  obstacles.Add(c);
	}
      }
      else {
	for(int j=0;j<numCircles;j++) {
	  c.center.set(width*Real(j)+start,width*Real(i)+start);
	  obstacles.Add(c);
	}
      }
    }
  }
  else if(kink) {
    //bottom left of kink
    temp.bmin.set(0.5-0.5*narrowPassageLength,0);
    temp.bmax.set(0.5-0.5*narrowPassageWidth,0.5-0.5*narrowPassageWidth+kinkLength*0.5);
    obstacles.Add(temp);
    //bottom right of kink
    temp.bmin.set(0.5-0.5*narrowPassageWidth,0);
    temp.bmax.set(0.5+0.5*narrowPassageLength,0.5-0.5*narrowPassageWidth-kinkLength*0.5);
    obstacles.Add(temp);
    //top left of kink
    temp.bmin.set(0.5-0.5*narrowPassageLength,0.5+0.5*narrowPassageWidth+kinkLength*0.5);
    temp.bmax.set(0.5+0.5*narrowPassageWidth,1-bypassWidth);
    obstacles.Add(temp);
    //top right of kink
    temp.bmin.set(0.5+0.5*narrowPassageWidth,0.5+0.5*narrowPassageWidth-kinkLength*0.5);
    temp.bmax.set(0.5+0.5*narrowPassageLength,1-bypassWidth);
    obstacles.Add(temp);
  }
  else if(windy) {
    Real width = 1.0/Real(numWinds+1);
    for(int i=0;i<numWinds;i++) {
      Real center = Real(i+1)/Real(numWinds+1);
      if(i%2 == 1) {
	temp.bmin.set(center-width*narrowPassageLength,0);
	temp.bmax.set(center+width*narrowPassageLength,1.0-border-narrowPassageWidth);
      }
      else {
	temp.bmin.set(center-width*narrowPassageLength,border+narrowPassageWidth);
	temp.bmax.set(center+width*narrowPassageLength,1.0);
      }
      obstacles.Add(temp);
    }
  }
  else if(borderL) {
    temp.bmin.set(border+narrowPassageWidth,border+narrowPassageWidth);
    temp.bmax.set(1,1);
    obstacles.Add(temp);
  }
  else if(passage) {
    temp.bmin.set(0.5-0.5*narrowPassageLength,0);
    //temp.bmax.set(0.5+0.5*narrowPassageLength,0.5-0.5*narrowPassageWidth);
    temp.bmax.set(0.5+0.5*narrowPassageLength,0.5-narrowPassageWidth);
    obstacles.Add(temp);
    //temp.bmin.set(0.5-0.5*narrowPassageLength,0.5+0.5*narrowPassageWidth);
    temp.bmin.set(0.5-0.5*narrowPassageLength,0.5);
    temp.bmax.set(0.5+0.5*narrowPassageLength,1-bypassWidth);
    obstacles.Add(temp);
  }
}

inline void SetupSimpleCSpace(Geometric2DCSpace& cspace)
{
  cspace.euclideanSpace = EUCLIDEAN_SPACE;
  cspace.domain.bmin.set(0,0);
  cspace.domain.bmax.set(1,1);
  SetupObstacles(cspace);  
  cspace.InitConstraints();
}

inline bool LoadCSpace(Geometric2DCSpace& cspace,const char* file)
{
  XmlDocument doc;
  if(!doc.Load(file)) {
    fprintf(stderr,"Error loading XML document %s\n",file);
    return false;
  }
  if(0!=strcmp(doc.RootElement()->Value(),"point2d_cspace")) {
    fprintf(stderr,"Warning, root element is not a point2d_cspace\n");
  }
  if(!XmlParse(doc.RootElement(),cspace)) {
    fprintf(stderr,"Error reading geometric 2d cspace from document %s\n",file);
    return false;
  }
  return true;
}

inline bool LoadCSpace(DrawableCSpace& cspace,const char* file)
{
  XmlDocument doc;
  if(!doc.Load(file)) {
    fprintf(stderr,"Error loading XML document %s\n",file);
    return false;
  }
  return cspace.ReadFromXml(doc.RootElement());
}

struct LengthCost
{
  const vector<Vector>& nodes;
  LengthCost(const vector<Vector>& _nodes)
    :nodes(_nodes)
  {}
  Real operator()(int & e,int s,int t) { return nodes[s].distance(nodes[t]); }
};


/*
void PrintCliqueFMMTestHeader(FILE* out=stdout)
{
  fprintf(out,"d,N,r,trial,V,E,SP time,SP cost,FMM time,FMM cost,FMM #propagate,FMM propagate time,FMM overhead time,FMM newton time\n");
}

void DoCliqueFMMTest(int d,int N,Real r,int numTrials=10,FILE* out=stdout)
{
  Vector bmin(d,0.0),bmax(d,1.0);
  Vector start(d,0.1),goal(d,0.9);
  for(int trial=0;trial<numTrials;trial++) {
    Real dijkstraTime,dijkstraCost,fmmTime,fmmCost;
    CliqueFMM fmm;
    fmm.points.resize(2);
    fmm.points[0] = start;
    fmm.points[1] = goal;
    fmm.start = 0;
    fmm.goal = 1;
    for(int i=0;i<N;i++) {
      Vector x(d);
      for(int j=0;j<d;j++)
	x[j] = Rand(bmin[j],bmax[j]);
      fmm.points.push_back(x);
    }
    fmm.MakeBallGraph(r);
    fmm.pointLocation = new KDTreePointLocation(fmm.points);
    Graph::UndirectedGraph<Vector,int> G;
    for(size_t i=0;i<fmm.points.size();i++)
      G.AddNode(fmm.points[i]);
    for(size_t i=0;i<fmm.edges.size();i++) {
      for(set<int>::iterator j=fmm.edges[i].begin();j!=fmm.edges[i].end();j++)
	if(i < *j)
	  G.AddEdge(i,*j);
    }
    Timer timer;
    Graph::ShortestPathProblem<Vector,int> spp(G);
    spp.InitializeSource(0);
    spp.FindPath_Undirected(1,LengthCost(G.nodes));
    dijkstraTime = timer.ElapsedTime();
    bool spSuccess = (spp.p[1] >= 0);
    if(spSuccess) {
      dijkstraCost = spp.d[1];
    }
    else
      dijkstraCost = Inf;

    timer.Reset();
    bool fmmSuccess = fmm.Search();
    fmmTime = timer.ElapsedTime();
    fmmCost=Inf;
    if(fmmSuccess) {
      vector<Vector> path;
      bool hasPath = fmm.GetPath(r,1,path);
      if(hasPath) {
	fmmCost = 0;
	for(size_t i=0;i+1<path.size();i++)
	  fmmCost += path[i].distance(path[i+1]);
	if(fmmCost > dijkstraCost) {
	  printf("Extracted path cost exceeds dijkstra cost\n");
	  printf("FMM estimate %g, cost %g, dijkstra %g\n",fmm.distances[1],fmmCost,dijkstraCost);
	  getchar();
	}
      }
      else {
	printf("Warning, did not successfully get path... using distance\n");
	//getchar();
	fmmCost = fmm.distances[1];
      }
    }
    int numEdges=0;
    for(size_t i=0;i<fmm.edges.size();i++)
      numEdges += fmm.edges[i].size();
    numEdges /= 2;
    fprintf(out,"%d,%d,%g,%d,%d,%d,%g,%g,%g,%g,%d,%g,%g,%g\n",d,N,r,trial,fmm.points.size(),numEdges,dijkstraTime,dijkstraCost,fmmTime,fmmCost,fmm.numPropagateSteps,fmm.propagateTime,fmm.newtonTime,fmm.overheadTime);
    fflush(out);
  }
}

void CliqueFMMTest()
{
  PrintCliqueFMMTestHeader();
  DoCliqueFMMTest(2,100,0.2);
  DoCliqueFMMTest(2,100,0.3);
  DoCliqueFMMTest(2,100,0.4);
  DoCliqueFMMTest(2,100,0.5);

  DoCliqueFMMTest(3,100,0.4);
  DoCliqueFMMTest(3,100,0.5);
  DoCliqueFMMTest(3,100,0.6);
  DoCliqueFMMTest(3,100,0.7);

  DoCliqueFMMTest(2,300,0.1);
  DoCliqueFMMTest(2,300,0.2);
  DoCliqueFMMTest(2,300,0.3);

  DoCliqueFMMTest(3,300,0.2);
  DoCliqueFMMTest(3,300,0.3);
  DoCliqueFMMTest(3,300,0.4);

  DoCliqueFMMTest(2,1000,0.05);
  DoCliqueFMMTest(2,1000,0.1);
  DoCliqueFMMTest(2,1000,0.15);

  DoCliqueFMMTest(3,1000,0.1);
  DoCliqueFMMTest(3,1000,0.2);
  DoCliqueFMMTest(3,1000,0.3);
}
*/

int gBlackBoxTestPredicateCount = 0;

bool BlackBoxTestPredicate(const Vector& x)
{
  gBlackBoxTestPredicateCount ++;
  if(Abs(x(0)) > 10.0) return false;
  if(Abs(x(1)) > 10.0) return false;
  if(10*x(0) - x(1) > 10) return false;
  if(x(0) + x(1) > 9) return false;
  return true;
}

void BlackBoxOptimizationTest()
{
  //NOTE: THIS WORKS POORLY WHEN THE SLOPE OF THE CONSTRAINT IS OBLIQUE W.R.T. THE OBJECTIVE GRADIENT
  Vector c(2);
  c(0) = -1;
  c(1) = 0;
  Optimization::NonlinearProgram nlp(new LinearScalarFieldFunction(c,0.0));
  Optimization::BlackBoxConstraintOptimizer opt(nlp,BlackBoxTestPredicate);
  opt.x.resize(2);
  opt.x.setZero();
  int iters=100;
  ConvergenceResult res = opt.Solve(iters);
  printf("Convergence result %d, %d iters\n",(int)res,iters);
  cout<<"Solution "<<opt.x<<endl;
  printf("Evaluated %d points, stored %d points\n",gBlackBoxTestPredicateCount,opt.points.size());
}

int main(int argc,char** argv)
{
  //BlackBoxOptimizationTest();
  
  //CliqueFMMTest();
  //FuzzyPlanningTest();
  //return 0;
  Srand(time(NULL));
  //CSpaceAnalysisProgram program;
  //TreePlannerProgram program;
  //MDPProgram program;
  //ClosedChainPlannerProgram program;
  //SeedSamplingProgram program;
  //MultiModalPlannerProgram program;
  //EndgamePlannerProgram program;
  //PRMPlannerProgram program;
  //CarPlannerProgram program;
  //PendulumPlannerProgram program;
  DoubleIntegratorPlannerProgram program;
  //POProximityPlannerProgram program;
  //POPursuitPlannerProgram program;
  //POPegPlannerProgram program;
  //MCRPlannerProgram program;
  //DisplacementPlannerProgram program;
  //SafestPathPlannerProgram program;
  //OptimalPlannerProgram program;
  //JumpPlannerProgram program;

  if(argc < 2) {
    program.cspace.pointSpace = new Geometric2DCSpace();
    SetupSimpleCSpace(*program.cspace.pointSpace);
  }
  else {
    if(!LoadCSpace(program.cspace,argv[1])) {
      printf("Error reading cspace from %s\n",argv[1]);
      return 1;
    }
  }
  return program.Run("Motion Planning Test");
}

