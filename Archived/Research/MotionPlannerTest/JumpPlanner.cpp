#include "JumpPlanner.h"
#include <KrisLibrary/Timer.h>
#include <KrisLibrary/math/random.h>
#include <KrisLibrary/structs/FastFindHeap.h>
#include <set>
using namespace std;

#define DO_TIMING 1

const static int numCosts = 2;
const static int pathCostIndex = 0;
const static int marginCostIndex = 1;

inline bool FuzzyVectorEquals(const Vector& a,const Vector& b,Real eps=Epsilon)
{
  if(a.n != b.n) return false;
  for(int i=0;i<a.n;i++)
    if(!FuzzyEquals(a[i],b[i],eps)) return false;
  return true;
}

JumpPlanner::JumpPlanner(KinodynamicCSpace* _space)
  :space(_space),randomSeedProbability(0.0),goalBiasProbability(0.1),goalNode(-1),goalNodeFrontierIndex(-1),goalCost(Inf),
   extendTime(0),rewireTime(0),nearestNeighborTime(0),propagateTime(0),collisionCheckTime(0),simulateTime(0),visibilityCheckTime(0),
   numBiasedSampleControls(0),numSimulates(0),numVisibilityChecks(0)
{}

void JumpPlanner::Init(const State& x0)
{
  roadmap.Cleanup();
  goalNode=-1;
  goalCost=Inf;
  extendTime=rewireTime=nearestNeighborTime=propagateTime=collisionCheckTime=simulateTime=visibilityCheckTime=0;
  numBiasedSampleControls=numSimulates=numVisibilityChecks=0;

  int res=AddMilestone(x0);
  Assert(res == 0);
  Vector costVector(2);
  costVector[0] = 0;
  costVector[1] = 0;
  roadmap.nodes[res].paretoOptimalCosts.resize(1);
  roadmap.nodes[res].paretoOptimalCosts[0] = costVector; 
  roadmap.nodes[res].paretoOptimalParents.resize(1);
  roadmap.nodes[res].paretoOptimalParents[0] = -1;
}

int JumpPlanner::AddMilestone(const State& x)
{
  Node n;
  n.x = x;
  Timer timer;
  n.obstacleDistance = ObstacleDistance(x);
  //Sanity check
  Assert(space->IsFeasible(x) == (n.obstacleDistance >= 0));
  n.goalSetMargin = GoalSetMargin(x);
  collisionCheckTime += timer.ElapsedTime();
  return roadmap.AddNode(n);
}

int JumpPlanner::SimulateAndAddEdge(int n,const ControlInput& u)
{
  Assert(n >= 0 && n < (int)roadmap.nodes.size());
  vector<State> path;
  space->Simulate(roadmap.nodes[n].x,u,path);
  EdgePlanner* ep = space->TrajectoryChecker(path);
  return AddEdge(n,u,path,ep);
}

int JumpPlanner::SimulateAndAddFeasibleEdge(int n,const ControlInput& u)
{
  Assert(n >= 0 && n < (int)roadmap.nodes.size());
  vector<State> path;
  Timer timer;
  space->Simulate(roadmap.nodes[n].x,u,path);
  simulateTime += timer.ElapsedTime();
  timer.Reset();
  EdgePlanner* ep = space->TrajectoryChecker(path);
  bool visible = ep->IsVisible();
  visibilityCheckTime += timer.ElapsedTime();
  numVisibilityChecks += 1;
  if(visible)
    return AddEdge(n,u,path,ep);
  delete ep;
  return -1;
}

int JumpPlanner::AddEdge(int n,const ControlInput& u,const std::vector<State>& path,const SmartPointer<EdgePlanner>& ep)
{
  Edge e;
  e.isJump = false;
  e.jumpDistance = 0;
  e.u = u;
  e.path = path;
  e.e = ep;
  int c = AddMilestone(path.back());
  roadmap.AddEdge(n,c,e);
  UpdateCosts(n,c);
  return c;
}

bool JumpPlanner::UpdateCost(int nIndex,const Vector& c,int parent)
{
  Node& n=roadmap.nodes[nIndex];
  /*
  Assert(roadmap.FindEdge(parent,nIndex)!=NULL);
  for(size_t k=0;k<n.paretoOptimalParents.size();k++) {
    if(n.paretoOptimalParents[k] >=0)
      Assert(roadmap.FindEdge(n.paretoOptimalParents[k],nIndex)!=NULL);
  }
  */

  bool changed=false;
  bool cdominated = false;
  /*
  cout<<"Updating pareto frontier of node "<<nIndex<<endl;
  printf("Current: ");
  for(size_t j=0;j<n.paretoOptimalCosts.size();j++) 
    printf("(%g,%g), ",n.paretoOptimalCosts[j][0],n.paretoOptimalCosts[j][1]);
  printf("\n");
  cout<<"Cost from parent "<<parent<<": "<<c[0]<<","<<c[1]<<endl;
  */
  for(size_t i=0;i<n.paretoOptimalCosts.size();i++) {
    //better: c is strictly better than existing frontier point on at least one axis 
    //dominated: c dominates the frontier point c >= p with domination on some axis
    bool better=false, dominates=true;
    for(int k=0;k<numCosts;k++) {
      if(c[k] < n.paretoOptimalCosts[i][k]) {
	better = true;
      }
      else if(c[k] > n.paretoOptimalCosts[i][k]) {
	dominates = false;
      }
    }
    //cout<<"better: "<<better<<", dominated: "<<dominated<<endl;
    if(!better) { cdominated = true; break; }
    if(dominates && better) {
      n.paretoOptimalCosts[i] = n.paretoOptimalCosts.back();
      n.paretoOptimalParents[i] = n.paretoOptimalParents.back();
      n.paretoOptimalCosts.resize(n.paretoOptimalCosts.size()-1);
      n.paretoOptimalParents.resize(n.paretoOptimalParents.size()-1);
      /*
      for(size_t k=0;k<n.paretoOptimalParents.size();k++)
	Assert(roadmap.FindEdge(n.paretoOptimalParents[k],nIndex)!=NULL);
      */
      i--;
      changed = true;
    }
  }
  if(changed) {
    if(cdominated) FatalError("Sanity check: Looks like pareto optimal frontier wasn't pareto optimal?");
    Assert(!cdominated);
  }
  if(!cdominated) {
    n.paretoOptimalCosts.push_back(c);
    n.paretoOptimalParents.push_back(parent);
    changed = true;
  }
  return changed;
}

bool JumpPlanner::UpdateCosts(int n,int c)
{
  Edge* e = roadmap.FindEdge(n,c);
  Assert(e != NULL);
  Node& nn = roadmap.nodes[n];
  Node& nc = roadmap.nodes[c];
  bool changed = false;
  Real ec = EdgeCost(nn.x,e->u);
  for(size_t i=0;i<nn.paretoOptimalCosts.size();i++) {
    Vector ci = nn.paretoOptimalCosts[i];
    ci[pathCostIndex] += ec;
    ci[marginCostIndex] = PropagateDistance(ci[marginCostIndex],e->u)+e->jumpDistance;
    //cout<<"propagation: "<<n<<" -> "<<c<<": "<<ci<<endl;
    //if we cannot guarantee feasibility, don't use this
    if(ci[marginCostIndex] > nc.obstacleDistance || ci[marginCostIndex] > GoalSetRadius()) {
      continue;
    }
    if(UpdateCost(c,ci,n))
      changed=true;
  }
  if(changed) {
    Node& nc = roadmap.nodes[c];
    if(c == goalNode) {
      //may have changed the frontier index by deleting a suboptimal point
      for(size_t i=0;i<nc.paretoOptimalCosts.size();i++) {
	const Vector& ci = nc.paretoOptimalCosts[i];
	if(ci[marginCostIndex] <= nc.goalSetMargin && ci[pathCostIndex] <= goalCost) {
	  goalNodeFrontierIndex = i;
	}
      }
    }
    //check for improved path to goal
    for(size_t i=0;i<nc.paretoOptimalCosts.size();i++) {
      const Vector& ci = nc.paretoOptimalCosts[i];
      if(ci[marginCostIndex] <= nc.goalSetMargin && ci[pathCostIndex] < goalCost) {
	cout<<"Improved cost to goal, path cost "<<ci[0]<<", margin "<<ci[1]<<endl;
	goalNode = c;
	goalNodeFrontierIndex = i;
	goalCost = ci[pathCostIndex];
      }
    }
  }
  return changed;
}

bool JumpPlanner::UpdateCosts(int n,int c,const Vector& ncost)
{
  Edge* e = roadmap.FindEdge(n,c);
  Assert(e != NULL);
  Node& nn = roadmap.nodes[n];
  Node& nc = roadmap.nodes[c];
  bool changed = false;
  Vector ci = ncost;
  ci[pathCostIndex] += EdgeCost(nn.x,e->u);
  ci[marginCostIndex] = PropagateDistance(ci[marginCostIndex],e->u)+e->jumpDistance;
  //cout<<"propagation: "<<n<<" -> "<<c<<": "<<ci<<endl;
  //if we cannot guarantee feasibility, don't use this
  if(ci[marginCostIndex] > nc.obstacleDistance || ci[marginCostIndex] > GoalSetRadius()) {
    return false;
  }
  if(UpdateCost(c,ci,n))
    changed=true;

  if(changed) {
    Node& nc = roadmap.nodes[c];
    if(c == goalNode) {
      //may have changed the frontier index by deleting a suboptimal point
      for(size_t i=0;i<nc.paretoOptimalCosts.size();i++) {
	const Vector& ci = nc.paretoOptimalCosts[i];
	if(ci[marginCostIndex] <= nc.goalSetMargin && ci[pathCostIndex] <= goalCost) {
	  goalNodeFrontierIndex = i;
	}
      }
    }
    //check for improved path to goal
    for(size_t i=0;i<nc.paretoOptimalCosts.size();i++) {
      const Vector& ci = nc.paretoOptimalCosts[i];
      if(ci[marginCostIndex] <= nc.goalSetMargin && ci[pathCostIndex] < goalCost) {
	cout<<"Improved cost to goal, path cost "<<ci[0]<<", margin "<<ci[1]<<endl;
	goalNode = c;
	goalNodeFrontierIndex = i;
	goalCost = ci[pathCostIndex];
      }
    }
  }
  return changed;
}

struct EdgePropagation
{
  EdgePropagation()
    :source(-1),target(-1)
  {}
  EdgePropagation(int _source,int _target,const Vector& _cost)
    :source(_source),target(_target),sourceCost(_cost)
  {}
  bool operator < (const EdgePropagation& rhs) const {
    if(source < rhs.source) return true;
    else if(source > rhs.source) return false;
    if(target < rhs.target) return true;
    else if(target > rhs.target) return false; 
    for(int i=0;i<sourceCost.n;i++) {
      if(sourceCost[i] < rhs.sourceCost[i]) return true;
      else if(sourceCost[i] > rhs.sourceCost[i]) return false;
    }
    return false;
  }
  int source,target;
  Vector sourceCost;
};

void JumpPlanner::PropagateCosts(int n,int c)
{
  Timer timer;
  FastFindHeap<EdgePropagation,Real> q;
  if(n==-1) {
    //just propagate to children of root
    Graph::EdgeIterator<Edge> ei;
    for(roadmap.Begin(c,ei);!ei.end();ei++) {   
      q.push(EdgePropagation(ei.source(),ei.target(),roadmap.nodes[c].paretoOptimalCosts[0]),0);
    }
  }
  else {
    //start with n,c
    for(size_t i=0;i<roadmap.nodes[n].paretoOptimalCosts.size();i++) {
      const Vector& ci = roadmap.nodes[n].paretoOptimalCosts[i];
      q.push(EdgePropagation(n,c,ci),ci[pathCostIndex]);
    }
  }
  int count = 0;
  while(!q.empty()) {
    count++;
    EdgePropagation edge = q.top(); q.pop();
    n = edge.source;
    c = edge.target;
    bool changed = UpdateCosts(n,c,edge.sourceCost);

    if(changed) {
      Node& nc = roadmap.nodes[c];
      //propagate more
      Graph::EdgeIterator<Edge> ei;
      for(roadmap.Begin(c,ei);!ei.end();ei++) {
	q.adjust(EdgePropagation(ei.source(),ei.target(),nc.paretoOptimalCosts.back()),nc.paretoOptimalCosts.back()[pathCostIndex]);
      }
    }
  }
  propagateTime += timer.ElapsedTime();
}

void JumpPlanner::GetChildCost(int n,const Vector& nCost,Edge* e,Vector& childCost)
{
  childCost = nCost;
  childCost[pathCostIndex] += EdgeCost(roadmap.nodes[n].x,e->u);
  childCost[marginCostIndex] = PropagateDistance(nCost[marginCostIndex],e->u)+e->jumpDistance;
}

JumpPlanner::Edge* JumpPlanner::AddJump(int n,int c)
{
  Edge e;
  e.isJump = true;
  //e.jumpDistance = space->Distance(roadmap.nodes[n].x,roadmap.nodes[c].x);
  space->BiasedSampleControl(roadmap.nodes[n].x,roadmap.nodes[c].x,e.u);
  numBiasedSampleControls++;
  space->Simulate(roadmap.nodes[n].x,e.u,e.path);
  numSimulates++;
  e.e = space->TrajectoryChecker(e.path);
  e.jumpDistance = space->Distance(e.path.back(),roadmap.nodes[c].x);
  Edge* res=&roadmap.AddEdge(n,c,e);
  PropagateCosts(n,c);
  return res;
}

JumpPlanner::Edge* JumpPlanner::TestAndAddJump(int n,int c)
{
  //test if any existing cost is pareto optimal without propagation
  bool shouldAdd = false;
  for(size_t i=0;i<roadmap.nodes[n].paretoOptimalCosts.size();i++) {
    const Vector& ci = roadmap.nodes[n].paretoOptimalCosts[i];
    //check if ci is a pareto optimal point
    bool cdominated = false;
    for(size_t j=0;j<roadmap.nodes[c].paretoOptimalCosts.size();j++) {
      const Vector& cj=roadmap.nodes[c].paretoOptimalCosts[j];
      bool better = false;
      for(int k=0;k<numCosts;k++) 
	if(ci[k] < cj[k]) { better=true; break; }
      if(!better) { cdominated = true; break; }
    }
    //if it is pareto optimal, we should add it
    if(!cdominated) {
      shouldAdd = true;
      break;
    }
  }
  if(!shouldAdd) return NULL;

  Timer timer;
  Edge e;
  e.isJump = true;
  space->BiasedSampleControl(roadmap.nodes[n].x,roadmap.nodes[c].x,e.u);
  numBiasedSampleControls++;
  Vector temp;
  space->SimulateEndpoint(roadmap.nodes[n].x,e.u,temp);
  numSimulates++;
  simulateTime += timer.ElapsedTime();
  e.jumpDistance = space->Distance(temp,roadmap.nodes[c].x);

  //test if any propagated cost is pareto optimal
  shouldAdd = false;
  for(size_t i=0;i<roadmap.nodes[n].paretoOptimalCosts.size();i++) {
    Vector ci=roadmap.nodes[n].paretoOptimalCosts[i];
    ci[pathCostIndex] += EdgeCost(roadmap.nodes[n].x,e.u);
    ci[marginCostIndex] = PropagateDistance(ci[marginCostIndex],e.u)+e.jumpDistance;
    if(ci[marginCostIndex] > roadmap.nodes[c].obstacleDistance || ci[marginCostIndex] > GoalSetRadius()) continue;
    //check if ci is a pareto optimal point
    bool cdominated = false;
    for(size_t j=0;j<roadmap.nodes[c].paretoOptimalCosts.size();j++) {
      const Vector& cj=roadmap.nodes[c].paretoOptimalCosts[j];
      bool better = false;
      for(int k=0;k<numCosts;k++) 
	if(ci[k] < cj[k]) { better=true; break; }
      if(!better) { cdominated = true; break; }
    }
    //if it is pareto optimal, we should add it
    if(!cdominated) {
      shouldAdd = true;
      break;
    }
  }
  if(!shouldAdd) return NULL;

  timer.Reset();
  if(!space->IsFeasible(temp)) {
    collisionCheckTime += timer.ElapsedTime();
    return NULL;
  }
  collisionCheckTime += timer.ElapsedTime();
  timer.Reset();
  space->Simulate(roadmap.nodes[n].x,e.u,e.path);
  simulateTime += timer.ElapsedTime();
  timer.Reset();
  e.e = space->TrajectoryChecker(e.path);
  if(!e.e->IsVisible()) {
    visibilityCheckTime += timer.ElapsedTime();
    numVisibilityChecks += 1;
    return NULL;
  }
  visibilityCheckTime += timer.ElapsedTime();
  numVisibilityChecks += 1;
  
  Edge* res=&roadmap.AddEdge(n,c,e);
  PropagateCosts(n,c);
  /*
  cout<<"New frontier: ";
  for(size_t j=0;j<roadmap.nodes[c].paretoOptimalCosts.size();j++) 
    printf("(%g,%g), ",roadmap.nodes[c].paretoOptimalCosts[j][0],roadmap.nodes[c].paretoOptimalCosts[j][1]);
  printf("\n");
  */
  return res;
}


bool JumpPlanner::CalculateCosts()
{
  //clear existing costs
  for(size_t i=0;i<roadmap.nodes.size();i++) {
    roadmap.nodes[i].paretoOptimalCosts.resize(0); 
    roadmap.nodes[i].paretoOptimalParents.resize(0); 
  }
  goalNode = -1;
  goalCost = Inf;
  //initialize start node
  Vector costVector(2);
  costVector[0] = 0;
  costVector[1] = 0;
  roadmap.nodes[0].paretoOptimalCosts.resize(1);
  roadmap.nodes[0].paretoOptimalCosts[0] = costVector; 
  roadmap.nodes[0].paretoOptimalParents.resize(1);
  roadmap.nodes[0].paretoOptimalParents[0] = -1;

  PropagateCosts(-1,0);
}

bool JumpPlanner::GetPath(KinodynamicMilestonePath& kpath)
{
  if(goalNode < 0) return false;
  vector<int> path;
  vector<Edge*> edges;
  int n=goalNode;
  int frontierIndex=goalNodeFrontierIndex;
  Assert(frontierIndex >= 0 && frontierIndex < roadmap.nodes[n].paretoOptimalParents.size());
  while(n != 0) {
    path.push_back(n);
    Assert(frontierIndex >= 0 && frontierIndex < roadmap.nodes[n].paretoOptimalParents.size());
    int p = roadmap.nodes[n].paretoOptimalParents[frontierIndex];
    edges.push_back(roadmap.FindEdge(p,n));
    Assert(edges.back() != NULL);

    //find index of p's pareto frontier that gave rise to n
    const Vector& cn = roadmap.nodes[n].paretoOptimalCosts[frontierIndex];
    frontierIndex = -1;
    Vector temp;
    for(size_t i=0;i<roadmap.nodes[p].paretoOptimalCosts.size();i++) {
      GetChildCost(p,roadmap.nodes[p].paretoOptimalCosts[i],edges.back(),temp);
      if(FuzzyVectorEquals(temp,cn))
	frontierIndex = (int)i;
    }
    if(frontierIndex < 0) {
      FatalError("Costs not propagated properly?");
    }
    Assert(frontierIndex >= 0);
    n = p;
  }
  path.push_back(0);
  reverse(path.begin(),path.end());
  reverse(edges.begin(),edges.end());
  kpath.Clear();
  kpath.milestones.push_back(roadmap.nodes[0].x);
  for(size_t i=0;i<edges.size();i++)
    //if(!edges[i]->isJump)
      kpath.Append(edges[i]->u,space);
  return true;
}

bool JumpPlanner::GetPath(int n,int frontierIndex,vector<int>& path,vector<int>& frontierIndices)
{
  path.resize(0);
  frontierIndices.resize(0);
  while(n != 0) {
    path.push_back(n);
    frontierIndices.push_back(frontierIndex);
    Assert(frontierIndex >= 0 && frontierIndex < roadmap.nodes[n].paretoOptimalParents.size());
    int p = roadmap.nodes[n].paretoOptimalParents[frontierIndex];
    Edge* e=roadmap.FindEdge(p,n);
    Assert(e != NULL);

    //find index of p's pareto frontier that gave rise to n
    const Vector& cn = roadmap.nodes[n].paretoOptimalCosts[frontierIndex];
    frontierIndex = -1;
    Vector temp;
    for(size_t i=0;i<roadmap.nodes[p].paretoOptimalCosts.size();i++) {
      GetChildCost(p,roadmap.nodes[p].paretoOptimalCosts[i],e,temp);
      if(FuzzyVectorEquals(temp,cn))
	frontierIndex = (int)i;
    }
    if(frontierIndex < 0) {
      FatalError("Costs not propagated properly?");
    }
    Assert(frontierIndex >= 0);
    n = p;
  }
  path.push_back(0);
  frontierIndices.push_back(frontierIndex);
  reverse(path.begin(),path.end());
  reverse(frontierIndices.begin(),frontierIndices.end());
  return true;
}


void JumpPlanner::PlanMore(int numIters)
{
  Timer timer;
  for(int iters=0;iters<numIters;iters++) { 
    timer.Reset();
    int n=Extend();
    extendTime += timer.ElapsedTime();
    timer.Reset();
    if(n >= 0) Rewire(n);
    rewireTime += timer.ElapsedTime();
    /*
      //Sanity check
      for(size_t i=0;i<roadmap.nodes.size();i++) {
	Assert(roadmap.nodes[i].paretoOptimalParents.size() == roadmap.nodes[i].paretoOptimalCosts.size());
	for(size_t j=0;j<roadmap.nodes[i].paretoOptimalParents.size();j++) 
	  if(roadmap.nodes[i].paretoOptimalParents[j] >= 0) {
	    Assert(roadmap.FindEdge(roadmap.nodes[i].paretoOptimalParents[j],i)!=NULL);
	  }
      }
    */
  }
}

int JumpPlanner::Extend()
{
  State x;
  if(RandBool(randomSeedProbability)) {
    space->Sample(x);
    if(space->IsFeasible(x))
      return AddMilestone(x);
    return -1;
  }
  if(RandBool(goalBiasProbability)) {
    if(!SampleGoalSet(x))
      space->Sample(x);
  }
  else
    space->Sample(x);
  return ExtendToward(x);
}

int JumpPlanner::ExtendToward(const State& x)
{
  Timer timer;
  int n = FindClosestNode(x);
  nearestNeighborTime += timer.ElapsedTime();
  ControlInput u;
  space->BiasedSampleControl(roadmap.nodes[n].x,x,u);
  numBiasedSampleControls++;
  int c = SimulateAndAddFeasibleEdge(n,u);
  return c;
}

bool JumpPlanner::Rewire(int n)
{
  Real r=roadmap.nodes[n].obstacleDistance;
  const Config& x = roadmap.nodes[n].x;
  int kmax = int(((1.0+1.0/x.n)*E)*Log(Real(roadmap.nodes.size())));
  return Rewire(n,Inf,kmax);
}

bool JumpPlanner::Rewire(int n,Real r,int k)
{
  Timer timer;
  //compute K-nearest neighbors within radius r
  vector<int> neighbors(k);
  set<pair<Real,int> > knn;
  Real dmax = r;
  for(int i=0;i<roadmap.nodes.size();i++) {
    Real d=space->Distance(roadmap.nodes[i].x,roadmap.nodes[n].x);
    if(d > 0 && d < dmax) {
      pair<Real,int> idx(d,i);
      knn.insert(idx);
      if((int)knn.size() > k)
	knn.erase(--knn.end());
      dmax = (--knn.end())->first;
    }
  }
  neighbors.resize(0);
  for(set<pair<Real,int> >::const_iterator j=knn.begin();j!=knn.end();j++) {
    neighbors.push_back(j->second);
  }
  nearestNeighborTime += timer.ElapsedTime();

  for(size_t i=0;i<neighbors.size();i++) {
    if(!roadmap.FindEdge(n,neighbors[i])) {
      TestAndAddJump(n,neighbors[i]);
    }
    if(!roadmap.FindEdge(neighbors[i],n)) {
      TestAndAddJump(neighbors[i],n);
    }
  }
}

int JumpPlanner::PickRandomNode() const
{
  return RandInt(roadmap.nodes.size());
}

int JumpPlanner::FindClosestNode(const State& x) const
{
  int res = -1;
  Real dmin = Inf;
  Real d;
  for(size_t i=0;i<roadmap.nodes.size();i++) {
    d = space->Distance(x,roadmap.nodes[i].x);
    if(d < dmin) {
      dmin = d;
      res = (int)i;
    }
  }
  return res;
}

int JumpPlanner::ApproximateRandomClosestNode(const State& x,int numIters) const
{
  int res = -1;
  Real dmin = Inf;
  Real d;
  for(int iters=0;iters<numIters;iters++) {
    int i = RandInt(roadmap.nodes.size());
    d = space->Distance(x,roadmap.nodes[i].x);
    if(d < dmin) {
      dmin = d;
      res = (int)i;
    }
  }
  return res;
}


LipschitzJumpPlanner::LipschitzJumpPlanner(KinodynamicCSpace* space,Real lipschitzConstant)
  :JumpPlanner(space),lengthLipschitzConstant(lipschitzConstant)
{}

LearningLipschitzJumpPlanner::LearningLipschitzJumpPlanner(KinodynamicCSpace* space)
  :LipschitzJumpPlanner(space)
{}

void LearningLipschitzJumpPlanner::PlanMore(int numIters)
{
  Real origGoalCost = goalCost;
  for(int iters=0;iters<numIters;iters++) {
    int n=Extend(); 
    if(n >= 0) {
      Rewire(n);
      if(goalCost < origGoalCost) {
	LearnLipschitzConstant();
	origGoalCost = goalCost;
      }
    }
  }
}

void LearningLipschitzJumpPlanner::LearnLipschitzConstant()
{
  printf("Learning lipschitz constant from simulation...\n");
  Real origLipschitzConstant = lengthLipschitzConstant;
  Assert(goalNode >= 0);
  vector<int> path,frontierIndices;
  GetPath(goalNode,goalNodeFrontierIndex,path,frontierIndices);
  KinodynamicMilestonePath kpath;
  kpath.milestones.push_back(roadmap.nodes[0].x);
  for(size_t i=0;i+1<path.size();i++) {
    int n=path[i],c=path[i+1];
    int frontierIndex=frontierIndices[i];
    Edge* e=roadmap.FindEdge(n,c);
    /*if(!e->isJump)*/ {
      kpath.Append(e->u,space);
      Real R = roadmap.nodes[n].paretoOptimalCosts[frontierIndex][marginCostIndex];
      Real lenu = ControlLength(e->u);
      Real Rc = space->Distance(kpath.milestones.back(),roadmap.nodes[c].x);
      //Rc <= R*(1 + lipschitzConstant*len(u))
      if(lenu > Epsilon) {
	Real constant = (Rc/R - 1.0) / lenu;
	if(constant > lengthLipschitzConstant)
	  lengthLipschitzConstant = constant;
      }
    }
  }
  if(lengthLipschitzConstant > origLipschitzConstant) {
    printf("Lipschitz constant updated to %g\n",lengthLipschitzConstant);
    CalculateCosts();
  }
}
