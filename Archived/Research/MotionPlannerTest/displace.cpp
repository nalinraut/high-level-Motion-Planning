#include "XmlReader.h"
#include <KrisLibrary/math/random.h>
#include <KrisLibrary/Timer.h>
#include <KrisLibrary/utils/StatCollector.h>
#include <KrisLibrary/planning/MCRPlanner.h>
#include "displace.h"
#include <KrisLibrary/Timer.h>
#include <iostream>
using namespace std;

bool gUseGreedy = false;
//Vector a(2,0.0025);
//Vector b(2,0.9975);
Vector a(2,0.1);
Vector b(2,0.9);
Real delta = 0.166;

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


int TestPlannerRRT(CSpace* space,int maxIters)
{
  RRTPlanner planner(space);
  planner.AddMilestone(a);
  planner.delta = 0.066;
  planner.connectionThreshold = 0.066;
  for(int iters=0;iters<maxIters;iters++) {
    RRTPlanner::Node* n=planner.Extend();
    if(!n) continue;
    if(space->Distance(n->x,b) < planner.connectionThreshold) {
      EdgePlanner* e=IsVisible(space,n->x,b);
      if(e) {
	delete e;
	printf("RRT done on iteration %d\n",iters);
	return iters;
      }
    }
  }
  return maxIters;
}

int TestPlannerBiRRT(CSpace* space,int maxIters)
{
  BidirectionalRRTPlanner planner(space);
  planner.Init(a,b);
  planner.delta = delta;
  planner.connectionThreshold = delta;
  for(int iters=0;iters<maxIters;iters++) {
    if(planner.Plan()) {
      printf("BiRRT done on iteration %d\n",iters);
      return iters;
    }
  }
  return maxIters;
}

int TestPlannerPRM(CSpace* space,int maxIters,int k)
{
  RoadmapPlanner planner(space);
  int start=planner.AddMilestone(a);
  int goal=planner.AddMilestone(b);
  Config x;
  for(int iters=0;iters<maxIters;iters++) {
    space->Sample(x);
    if(space->IsFeasible(x)) {
      int n = planner.AddMilestone(x);
      planner.ConnectToNearestNeighbors(n,k);
      if(planner.AreConnected(start,goal)) {
	printf("%d-PRM done on iteration %d\n",k,iters);
	return iters;
      }
    }
  }
  return maxIters;
}

int TestPlannerPRMStar(CSpace* space,int maxIters)
{
  RoadmapPlanner planner(space);
  int start=planner.AddMilestone(a);
  int goal=planner.AddMilestone(b);
  Real kconst = (1.0+1.0/a.n)*E;
  Config x;
  for(int iters=0;iters<maxIters;iters++) {
    space->Sample(x);
    if(space->IsFeasible(x)) {
      int n = planner.AddMilestone(x);
      int k=(int)(kconst*Log(Real(planner.roadmap.nodes.size())));
      planner.ConnectToNearestNeighbors(n,k);
      if(planner.AreConnected(start,goal)) {
	printf("PRM* done on iteration %d\n",iters);
	return iters;
      }
    }
  }
  return maxIters;
}


size_t TestMCRIncrementalLimit(ExplicitCSpace* space,int maxIters)
{
  MCRPlanner eplanner(space);
  eplanner.updatePathsComplete = !gUseGreedy;
  eplanner.Init(a,b);
  eplanner.expandDistance = delta;
  eplanner.goalConnectThreshold = delta;
  vector<int> expansionSchedule;
  Subset origCover;
  eplanner.Completion(0,0,1,origCover);
  for(int i=0;i<=(int)origCover.size();i++) {
    expansionSchedule.push_back(maxIters*i/origCover.size());
  }
  vector<int> bestPath,optimalPath;
  Subset cover,optimalCover;
  eplanner.Plan(0,expansionSchedule,bestPath,cover);

  printf("Time NN %g, refine %g, explore %g, update %g, overhead %g\n",eplanner.timeNearestNeighbors,eplanner.timeRefine,eplanner.timeExplore,eplanner.timeUpdatePaths,eplanner.timeOverhead);
  printf("Update paths calls: %d, iterations %d\n",eplanner.numUpdatePaths,eplanner.numUpdatePathsIterations);
  /*
  //check difference between this greedy path and the optimal path
  bool res=(eplanner.OptimalPath(0,1,optimalPath,optimalCover));
  printf("Greedy cover size: %d, optimal: %d\n",cover.size(),optimalCover.size());
  if(!res) return cover.size();
  return optimalCover.size();
  */
  return cover.size();
}


void RunRRTTests(CSpace* space,int numTrials,int maxIters)
{
  StatCollector rrtTime,birrtTime,prmTime,prmStarTime;
  Timer timer;
  for(int i=0;i<numTrials;i++) {
    timer.Reset();
    TestPlannerRRT(space,maxIters);
    rrtTime.collect(timer.ElapsedTime());
  }
  for(int i=0;i<numTrials;i++) {
    timer.Reset();
    TestPlannerBiRRT(space,maxIters);
    birrtTime.collect(timer.ElapsedTime());
  }
  for(int i=0;i<numTrials;i++) {
    timer.Reset();
    TestPlannerPRM(space,maxIters,10);
    prmTime.collect(timer.ElapsedTime());
  }
  for(int i=0;i<numTrials;i++) {
    timer.Reset();
    TestPlannerPRMStar(space,maxIters);
    prmStarTime.collect(timer.ElapsedTime());
  }
  cout<<"RRT time: "; rrtTime.Print(cout); cout<<endl;
  cout<<"BiRRT time: "; birrtTime.Print(cout); cout<<endl;
  cout<<"PRM time: "; prmTime.Print(cout); cout<<endl;
  cout<<"PRM* time: "; prmStarTime.Print(cout); cout<<endl;
}

}

void RunDisplacementTests(Geometric2DCSpace* space,int numTrials,int maxIters)
{
  StatCollector planTime;
  cout<<"Begin displacement plan test"<<endl;
  GeometricDisplacementCSpace displacementSpace(space);
  for(int trial = 0; trial < numTrials; trial++) {
    int numDisplacementSamples = 100;
    DisplacementPlanner planner(&displacementSpace);
    planner.Init(a,b);
    Timer timer;
    PlanDisplacement(planner,maxIters,numDisplacementSamples);
    planTime.collect(timer.ElapsedTime());
    getchar();
  } 
  cout<<"Displacement plan time: "; planTime.Print(cout); cout<<endl;
}

int main(int argc,char** argv)
{
  if(argc < 2) {
    printf("Usage: displace space.xml\n");
    return 0;
  }

  Srand(time(NULL));
  Geometric2DCSpace space;
  if(!LoadCSpace(space,argv[1])) 
    return 1;

  RunDisplacementTests(&space,10,10000);
  return 0;
}
