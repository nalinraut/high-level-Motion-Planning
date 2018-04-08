#include "XmlReader.h"
#include <KrisLibrary/math/random.h>
#include <KrisLibrary/Timer.h>
#include <KrisLibrary/utils/StatCollector.h>
#include <KrisLibrary/planning/ExplainingPlanner.h>
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

int TestExplainPRM(ExplicitCSpace* space,int maxIters,int k)
{
  Timer timer;
  double rawTime = 0.0;

  ErrorExplainingPlanner eplanner(space);
  ErrorExplainingPlanner::Roadmap& roadmap=eplanner.roadmap;
  eplanner.updatePathsComplete = !gUseGreedy;
  eplanner.Init(a,b);
  eplanner.expandDistance = delta;
  eplanner.goalConnectThreshold = delta;

  Subset bestCover;
  eplanner.Completion(0,0,1,bestCover);
  Real bestCoverTime = 0;
  int bestCoverIters = 0;

  Config x;
  vector<pair<Real,size_t> > distances;
  for(int iters=0;iters<maxIters;iters++) {
    timer.Reset();

    space->Sample(x);
    int i=eplanner.AddNode(x);

    distances.resize(0);
    distances.reserve(roadmap.nodes.size()-1);
    for(size_t j=0;j<roadmap.nodes.size();j++) {
      if(i==(int)j) continue;
      distances.resize(distances.size()+1);
      distances.back().first = space->Distance(roadmap.nodes[i].q,roadmap.nodes[j].q);
      distances.back().second = j;
    }
    sort(distances.begin(),distances.end());
    assert(i >= 0 && i < (int)roadmap.nodes.size());
    bool edgeAdded = false;
    for(int j=0;j<Min(k,int(distances.size()));j++) {
      assert(distances[j].second >= 0 && distances[j].second < (int)roadmap.nodes.size());
      if(eplanner.AddEdge(i,distances[j].second))
	edgeAdded = true;
    }
    rawTime += timer.ElapsedTime();

    if(edgeAdded) {
      if(gUseGreedy) {
	timer.Reset();
	eplanner.UpdatePathsGreedy();
	rawTime += timer.ElapsedTime();
      }
      else
	eplanner.UpdatePathsComplete();
      int gmode=roadmap.nodes[1].mode;
      if(eplanner.modeGraph.nodes[gmode].minCover < bestCover.count()) {
	bestCoverTime = rawTime;
	bestCoverIters = iters;
	for(size_t j=0;j<eplanner.modeGraph.nodes[gmode].pathCovers.size();j++)
	  if(eplanner.modeGraph.nodes[gmode].pathCovers[j].count() < bestCover.count())
	    bestCover = eplanner.modeGraph.nodes[gmode].pathCovers[j];
	if(bestCover.count()==0) break;
	printf("cover %d iters %d time %g\n",bestCover.count(),bestCoverIters,bestCoverTime);
      }
    }
  }

  printf("cover %d iters %d time %g\n",bestCover.count(),bestCoverIters,bestCoverTime);
	
  return maxIters;
}



size_t TestPlannerIncrementalLimit(ExplicitCSpace* space,int maxIters)
{
  ErrorExplainingPlanner eplanner(space);
  eplanner.updatePathsComplete = !gUseGreedy;
  eplanner.Init(a,b);
  eplanner.expandDistance = delta;
  eplanner.goalConnectThreshold = delta;
  vector<int> expansionSchedule;
  Subset origCover;
  eplanner.Completion(0,0,1,origCover);
  for(int i=0;i<=(int)origCover.count();i++) {
    expansionSchedule.push_back(maxIters*i/origCover.count());
  }
  vector<int> bestPath,optimalPath;
  Subset cover,optimalCover;
  eplanner.Plan(0,expansionSchedule,bestPath,cover);

  printf("Time NN %g, refine %g, explore %g, update %g, overhead %g\n",eplanner.timeNearestNeighbors,eplanner.timeRefine,eplanner.timeExplore,eplanner.timeUpdatePaths,eplanner.timeOverhead);
  printf("Update paths calls: %d, iterations %d\n",eplanner.numUpdatePaths,eplanner.numUpdatePathsIterations);
  /*
  //check difference between this greedy path and the optimal path
  bool res=(eplanner.OptimalPath(0,1,optimalPath,optimalCover));
  printf("Greedy cover size: %d, optimal: %d\n",cover.count(),optimalCover.count());
  if(!res) return cover.count();
  return optimalCover.count();
  */
  return cover.count();
}




size_t TestPlannerMaxLimit(ExplicitCSpace* space,int maxIters)
{
  int n=space->NumObstacles();
  ErrorExplainingPlanner eplanner(space);
  eplanner.updatePathsComplete = !gUseGreedy;
  eplanner.Init(a,b);
  eplanner.expandDistance = delta;
  eplanner.goalConnectThreshold = delta;
  vector<int> expansionSchedule(1,maxIters);
  vector<int> bestPath,optimalPath;
  Subset cover,optimalCover;
  eplanner.Plan(n,expansionSchedule,bestPath,cover);

  /*
  //check difference between this greedy path and the optimal path
  bool res=(eplanner.OptimalPath(0,1,optimalPath,optimalCover));
  printf("Greedy cover size: %d, optimal: %d\n",cover.count(),optimalCover.count());
  if(!res) return cover.count();
  return optimalCover.count();
  */
  return cover.count();
}



size_t TestPlannerFixedLimit(ExplicitCSpace* space,int maxIters,int limit)
{
  ErrorExplainingPlanner eplanner(space);
  eplanner.updatePathsComplete = !gUseGreedy;
  eplanner.Init(a,b);
  eplanner.expandDistance = delta;
  eplanner.goalConnectThreshold = delta;
  vector<int> expansionSchedule(1,maxIters);
  vector<int> bestPath,optimalPath;

  Subset cover,optimalCover;
  eplanner.Plan(limit,expansionSchedule,bestPath,cover);

  //printf("Time NN %g, refine %g, explore %g, update %g, overhead %g\n",eplanner.timeNearestNeighbors,eplanner.timeRefine,eplanner.timeExplore,eplanner.timeUpdatePaths,eplanner.timeOverhead);
  /*
  //check difference between this greedy path and the optimal path
  bool res=(eplanner.OptimalPath(0,1,optimalPath,optimalCover));
  printf("Greedy cover size: %d, optimal: %d\n",cover.count(),optimalCover.count());
  if(!res) return cover.count();
  return optimalCover.count();
  */
  return cover.count();
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

void RunExpansionTests(ExplicitCSpace* space,int numTrials,int maxIters)
{
  size_t best = (size_t)space->NumObstacles();
  StatCollector incrementalTimes,maxLimitTimes,optimalTimes;
  Timer timer;

  /*
  printf("* Raw PRM test: *\n");
  for(int i=0;i<numTrials;i++) {
    TestExplainPRM(space,maxIters,10);
  }
  return;
  */

  //incremental
  printf("* Incremental test: *\n");
  for(int i=0;i<numTrials;i++) {
    timer.Reset();
    size_t res=TestPlannerIncrementalLimit(space,maxIters);
    //printf("Resulting cover: %d\n",res);
    if(res < best) best=res;
    incrementalTimes.collect(timer.ElapsedTime());
  }

  //fixed at max
  printf("* Max limit test: *\n");
  for(int i=0;i<numTrials;i++) {
    timer.Reset();
    size_t res=TestPlannerMaxLimit(space,maxIters);
    //printf("Resulting cover: %d\n",res);
    if(res < best) best=res;
    maxLimitTimes.collect(timer.ElapsedTime());
  }

  //optimal
  printf("* Optimal test (limit %d): *\n",best);
  for(int i=0;i<numTrials;i++) {
    timer.Reset();
    size_t res=TestPlannerFixedLimit(space,maxIters,best);
    //printf("Resulting cover: %d\n",res);
    if(res < best) best=res;
    optimalTimes.collect(timer.ElapsedTime());
  }
  cout<<"Incremental time: "; incrementalTimes.Print(cout); cout<<endl;
  cout<<"Max limit time: "; maxLimitTimes.Print(cout); cout<<endl;
  cout<<"Optimal time: "; optimalTimes.Print(cout); cout<<endl;

  if(best == 0) {
    RunRRTTests(space,numTrials,maxIters);    
  }
}


int main(int argc,char** argv)
{
  if(argc < 2) {
    printf("Usage: explain space.xml\n");
    return 0;
  }

  Srand(time(NULL));
  Geometric2DCSpace space;
  if(!LoadCSpace(space,argv[1])) 
    return 1;

  RunExpansionTests(&space,10,10000);
  //RunRRTTests(&space,10,20000);
  return 0;
}
