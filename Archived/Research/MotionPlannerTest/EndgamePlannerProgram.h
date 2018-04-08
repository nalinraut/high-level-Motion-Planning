#include <KrisLibrary/planning/SBL2.h>
#include "MotionPlannerProgram.h"
#include <KrisLibrary/planning/VisibilityGraphPlanner.h>
#include <KrisLibrary/statistics/Histogram.h>
#include <fstream>
#include <sstream>
using namespace std;

const static bool gFocusedGoalSampling = true;

class EndgamePlanner
{
public:
  typedef TreeRoadmapPlanner::Node Node;

  EndgamePlanner(CSpace*);
  ~EndgamePlanner();
  void Init(const Config& start,CSpace* goalSet);
  void Init(const Config& start,const Config& goal,Real goalRadius);
  Node* ExtendForward();
  Node* SampleGoal();
  Node* ExtendBackward();
  bool ConnectTrees(Node* fwdNode,Node* bwdNode);
  bool PlanForward(int maxIters);
  bool PlanBidirectional(int maxIters);
  void CreatePath(MilestonePath& path);
  Real EvalCost(Real configCost,Real pathCost) const;

  CSpace* space;
  CSpace* goalSet;
  RRTPlanner planner;
  Node* fwdTree;
  vector<Node*> bwdTrees;
  Real expandBackwardFraction;
  Real sampleGoalFraction;

  //temp
  int numForwardExtends;
  int numGoalSamples;
  int numBackwardExtends;
  int numConnectionAttempts;
};

struct NeighborhoodGoalSet : public CSpace
{
  NeighborhoodGoalSet(CSpace* _base,const Config& _center,Real _radius)
    :base(_base),center(_center),radius(_radius),sampleRadius(Inf)
  {}
  virtual void Sample(Config& x) { 
    if(IsInf(sampleRadius)) base->Sample(x);
    else base->SampleNeighborhood(center,sampleRadius,x);
  }
  virtual EdgePlanner* LocalPlanner(const Config& a,const Config& b) { return NULL; }
  virtual bool IsFeasible(const Config& x) { return base->Distance(x,center) <= radius; }

  CSpace* base;
  Config center;
  Real radius;

  Real sampleRadius;
};


EndgamePlanner::EndgamePlanner(CSpace* _space)
  :space(_space),goalSet(NULL),planner(_space),fwdTree(NULL)
{
  numForwardExtends = numBackwardExtends = numGoalSamples = numConnectionAttempts = 0;
  planner.delta = 0.1;
  expandBackwardFraction = 0.5;
  sampleGoalFraction = 0;
}

EndgamePlanner::~EndgamePlanner()
{
  SafeDelete(goalSet);
}

Real EndgamePlanner::EvalCost(Real configCost,Real pathCost) const
{
  Real fwdTreeCost = Real(numForwardExtends)*(configCost+pathCost);
  Real bwdTreeCost = Real(numBackwardExtends)*(configCost+pathCost);
  Graph::CountCallback<Node*> callback;
  fwdTree->DFS(callback);
  return fwdTreeCost + Real(callback.count)*configCost + bwdTreeCost + numConnectionAttempts*pathCost;
}

void EndgamePlanner::Init(const Config& start,CSpace* _goalSet)
{
  SafeDelete(goalSet);
  goalSet = _goalSet;
  planner.Cleanup();
  fwdTree = planner.AddMilestone(start);
  Assert(fwdTree);
  bwdTrees.resize(0);

  numForwardExtends = numBackwardExtends = numGoalSamples = numConnectionAttempts = 0;
}

void EndgamePlanner::Init(const Config& start,const Config& goal,Real goalRadius)
{
  NeighborhoodGoalSet* s=new NeighborhoodGoalSet(space,goal,goalRadius);
  /*
  s->sampleRadius = goalRadius*10;
  if(s->sampleRadius > 1) s->sampleRadius = Inf;
  */
  Init(start,s);
}

TreeRoadmapPlanner::Node* EndgamePlanner::ExtendForward()
{
  numForwardExtends++;
  Config x,dest;
  space->Sample(dest);
  Node* closest = planner.ClosestMilestoneInSubtree(fwdTree,dest);
  Real dist=space->Distance(closest->x,dest);
  if(dist > planner.delta)
    space->Interpolate(closest->x,dest,planner.delta/dist,x);
  else
    x=dest;
  Node* newNode = planner.TryExtend(closest,x);
  return newNode;
}

TreeRoadmapPlanner::Node* EndgamePlanner::SampleGoal()
{
  numGoalSamples++;
  if(!goalSet) return NULL;
  Config x;
  goalSet->Sample(x);
  if(goalSet->IsFeasible(x)) {
    Node* n = planner.AddMilestone(x);
    if(n) bwdTrees.push_back(n);
    return n;
  }
  return NULL;
}

TreeRoadmapPlanner::Node* EndgamePlanner::ExtendBackward()
{
  if(bwdTrees.empty()) return SampleGoal();

  numBackwardExtends++;
  Config x,dest;
  space->Sample(dest);
  Node* closest = NULL;
  Real closestDist = Inf;
  for(size_t i=0;i<bwdTrees.size();i++) {
    Node* n=planner.ClosestMilestoneInSubtree(bwdTrees[i],dest);
    Real dist=space->Distance(n->x,dest);
    if(dist < closestDist) {
      closest = n;
      closestDist = dist;
    }
  }
  if(closestDist > planner.delta)
    space->Interpolate(closest->x,dest,planner.delta/closestDist,x);
  else
    x=dest;
  Node* newNode = planner.TryExtend(closest,x);
  return newNode;
}

bool EndgamePlanner::ConnectTrees(Node* fwdNode,Node* bwdNode)
{
  numConnectionAttempts++;
  EdgePlanner* e = planner.TryConnect(fwdNode,bwdNode);
  return (e != NULL);
}

bool EndgamePlanner::PlanForward(int maxIters)
{
  for(int iters=0;iters<maxIters;iters++) {
    Node* n = ExtendForward();
    if(n) {
      if(goalSet->IsFeasible(n->x)) return true;
    }
  }
  return false;
}

bool EndgamePlanner::PlanBidirectional(int maxIters)
{
  Node* lastForwardNode=fwdTree;
  Node* lastBackwardNode=NULL;
  if(!bwdTrees.empty()) lastBackwardNode=bwdTrees[RandInt(bwdTrees.size())];
  for(int iters=0;iters<maxIters;iters++) {
    if(!RandBool(expandBackwardFraction)) {
      Node* n = ExtendForward();
      if(n) {
	lastForwardNode = n;
	if(goalSet->IsFeasible(n->x)) {
	  return true;
	}
	if(lastBackwardNode) {
	  if(space->Distance(n->x,lastBackwardNode->x) < planner.delta) {
	    if(ConnectTrees(n,lastBackwardNode)) return true;
	  }
	}
      }
    }
    else {
      Node* n = ExtendBackward();
      if(n) lastBackwardNode = n;
    }
  }
  return false;
}

void EndgamePlanner::CreatePath(MilestonePath& path)
{
  for(size_t i=0;i<planner.milestones.size();i++) 
    if(fwdTree->connectedComponent == planner.milestones[i]->connectedComponent && goalSet->IsFeasible(planner.milestones[i]->x)) {
      planner.CreatePath(fwdTree,planner.milestones[i],path);
      return;
    }
  path.edges.clear();
}

/*
struct DrawTreeCallback : public SBLTree::Node::Callback
{
  bool ForwardEdge(SBLTree::Node* i,SBLTree::Node* j)
  {
    //const Config& x=*i, &y=*j;
    const SmartPointer<EdgePlanner>& e=j->edgeFromParent();
    edgeColor.setCurrentGL();
    glBegin(GL_LINES);
    glVertex2v(i->x);
    glVertex2v(j->x);
    glEnd();
    return true;
  }

  void Visit(SBLTree::Node* node)
  {
    nodeColor.setCurrentGL();
    glBegin(GL_POINTS);
    glVertex2v(node->x);
    glEnd();
  }

  GLColor nodeColor,edgeColor;
};
*/

struct EndgamePlannerProgram : public MotionPlannerProgram
{
  bool hasStart,hasGoal,hasPath;
  Config start,goal;
  Real goalRadius;
  MilestonePath path;
  EndgamePlanner planner;
  VisibilityGraphPlanner vgraph;

  EndgamePlannerProgram()
    :planner(&cspace),vgraph(&cspace)
  {
    hasStart=hasGoal=hasPath=false;
    goalRadius = 0.01;
  }

  virtual bool Initialize() {
    if(!MotionPlannerProgram::Initialize()) return false;
    vgraph.Init();
    return true;
  }

  virtual void Handle_Display() {
    SetupDisplay();
    DrawCSpace();

    DrawTreeCallback callback;
    callback.nodeColor.set(1,0,1);
    callback.edgeColor.set(0.5,0,0.5);
    glPointSize(3.0);
    glLineWidth(2.0);
    if(planner.fwdTree)
      planner.fwdTree->DFS(callback);
    for(size_t i=0;i<planner.bwdTrees.size();i++) {
      planner.bwdTrees[i]->DFS(callback);
    }
    if(hasPath) {
      glColor3f(0,0,1);
      glLineWidth(4.0);
      glBegin(GL_LINE_STRIP);
      for(size_t i=0;i<path.NumMilestones();i++)
	glVertex2v(path.GetMilestone(i));
      glEnd();
    }
    if(hasStart) {
      glPointSize(7.0);
      glColor3f(0,1,0);
      glBegin(GL_POINTS);
      glVertex2v(start);
      glEnd();
    }
    if(hasGoal) {
      glPointSize(7.0);
      glColor3f(1,0,0);
      glBegin(GL_POINTS);
      glVertex2v(goal);
      glEnd();
    }
    glutSwapBuffers();
  }

  bool SampleConfig(Config& q,int maxIters=100)
  {
    for(int i=0;i<maxIters;i++) {
      cspace.Sample(q);
      if(cspace.IsFeasible(q)) return true;
    }
    return false;
  }

  bool SampleProblem()
  {
    if(!SampleConfig(start)) return false;
    if(!SampleConfig(goal)) return false;

    planner.Init(start,goal,goalRadius);
    hasStart = true;
    hasGoal = true;
    if(planner.PlanForward(100)) {
      planner.CreatePath(path);
      hasPath = true;
      return true;
    }
    return false;
  }

  virtual void Handle_Keypress(unsigned char key,int x,int y)
  {
    if(key == 't') {
      if(!hasStart) {
	if(!SampleConfig(start)) return;
	hasStart=true;
      }
      if(!hasGoal) {
	if(!SampleConfig(goal)) return;
	hasGoal=true;
      }
      int expandIters = 5000;
      map<Real,vector<int> > stats;
      for(int iters=0;iters<8;iters++) {
	goalRadius = Pow(0.5,iters+1);
	stats[goalRadius].resize(expandIters,0);
	printf("Testing fwd planning goal radius %g...\n",goalRadius);
	for(int test=0;test<100;test++) {
	  planner.Init(start,goal,goalRadius);
	  bool res=planner.PlanForward(expandIters);
	  if(res && (int)planner.EvalCost(0.1,1) < expandIters)
	    stats[goalRadius][(int)planner.EvalCost(0.1,1)]++;
	}
      }
      for(map<Real,vector<int> >::iterator i=stats.begin();i!=stats.end();i++) {
	int accum=0;
	for(size_t j=0;j<i->second.size();j++) {
	  i->second[j] += accum;
	  accum = i->second[j];
	}
      }
      {
      cout<<"Saving to endgame_stats.txt"<<endl;
      ofstream out("endgame_stats.txt");
      out<<"#iters\tr=";
      for(map<Real,vector<int> >::const_iterator i=stats.begin();i!=stats.end();i++) 
	out<<i->first<<"\t";
      out<<endl;
      for(int iters=0;iters<expandIters;iters+=10) {
	out<<iters<<"\t";
	for(map<Real,vector<int> >::const_iterator i=stats.begin();i!=stats.end();i++) 
	  out<<i->second[iters]<<"\t";
	out<<endl;
      }
      out.close();
      }
      {
      for(map<Real,vector<int> >::const_iterator i=stats.begin();i!=stats.end();i++) {
	vector<pair<Real,Real> > profile;
	profile.resize(i->second.size());
	Real expectedCost = 0;
	Real pSuccess = 0;
	for(size_t j=0;j<i->second.size();j++) {
	  expectedCost += (1.0-pSuccess);
	  pSuccess = Real(i->second[j])/Real(100);
	  profile[j] = pair<Real,Real>(expectedCost,pSuccess);
	}
	stringstream ss;
	ss<<"endgame_profile_"<<i->first<<".txt";
	ofstream out(ss.str().c_str());
	cout<<"Saving profile to "<<ss.str()<<endl;
	out<<"#cost\tpSuccess"<<endl;
	for(size_t j=0;j<profile.size();j++)
	  out<<profile[j].first<<"\t"<<profile[j].second<<endl;
      }
      }
    
      goalRadius = 0.01;
      Refresh();
    }
    else if(key == ' ') {
      if(!hasStart) {
	if(!SampleConfig(start)) return;
	hasStart=true;
      }
      if(!hasGoal) {
	if(!SampleConfig(goal)) return;
	hasGoal=true;
      }
      planner.Init(start,goal,goalRadius);
      if(planner.PlanBidirectional(5000)) {
	planner.CreatePath(path);
	hasPath = true;
      }
      cout<<"# of forward iters: "<<planner.numForwardExtends<<endl;
      cout<<"# of backward iters: "<<planner.numBackwardExtends<<endl;
      cout<<"# of goal samples: "<<planner.numGoalSamples<<endl;

      Refresh();
    }
    else if(key == 'T') {
      //plan bidirectional
      if(!hasStart) {
	if(!SampleConfig(start)) return;
	hasStart=true;
      }
      if(!hasGoal) {
	if(!SampleConfig(goal)) return;
	hasGoal=true;
      }
      int expandIters = 5000;
      map<Real,vector<int> > stats;
      for(int iters=0;iters<8;iters++) {
	goalRadius = Pow(0.5,iters+1);
	stats[goalRadius].resize(expandIters,0);
	printf("Testing bidirectional planning goal radius %g...\n",goalRadius);
	for(int test=0;test<100;test++) {
	  planner.Init(start,goal,goalRadius);
	  if(gFocusedGoalSampling) {
	    NeighborhoodGoalSet* gs = dynamic_cast<NeighborhoodGoalSet*>(planner.goalSet);
	    gs->sampleRadius = goalRadius*5;
	    if(gs->sampleRadius > 1) gs->sampleRadius = Inf;
	  }
	  bool res=planner.PlanBidirectional(expandIters*2);
	  if(res && (int)planner.EvalCost(0.1,1) < expandIters)
	    stats[goalRadius][(int)planner.EvalCost(0.1,1)]++;
	}
      }
      for(map<Real,vector<int> >::iterator i=stats.begin();i!=stats.end();i++) {
	int accum=0;
	for(size_t j=0;j<i->second.size();j++) {
	  i->second[j] += accum;
	  accum = i->second[j];
	}
      }
      {
      cout<<"Saving to endgame_bidir_stats.txt"<<endl;
      ofstream out;
      if(gFocusedGoalSampling) 
	out.open("endgame_bidir2_stats.txt");
      else
	out.open("endgame_bidir_stats.txt");
      out<<"#iters\tr=";
      for(map<Real,vector<int> >::const_iterator i=stats.begin();i!=stats.end();i++) 
	out<<i->first<<"\t";
      out<<endl;
      for(int iters=0;iters<expandIters;iters+=10) {
	out<<iters<<"\t";
	for(map<Real,vector<int> >::const_iterator i=stats.begin();i!=stats.end();i++) 
	  out<<i->second[iters]<<"\t";
	out<<endl;
      }
      out.close();
      }
      {
      for(map<Real,vector<int> >::const_iterator i=stats.begin();i!=stats.end();i++) {
	vector<pair<Real,Real> > profile;
	profile.resize(i->second.size());
	Real expectedCost = 0;
	Real pSuccess = 0;
	for(size_t j=0;j<i->second.size();j++) {
	  expectedCost += (1.0-pSuccess);
	  pSuccess = Real(i->second[j])/Real(100);
	  profile[j] = pair<Real,Real>(expectedCost,pSuccess);
	}
	stringstream ss;
	if(gFocusedGoalSampling) 
	  ss<<"endgame_bidir2_profile_"<<i->first<<".txt";
	else
	  ss<<"endgame_bidir_profile_"<<i->first<<".txt";
      
	ofstream out(ss.str().c_str());
	cout<<"Saving profile to "<<ss.str()<<endl;
	out<<"#cost\tpSuccess"<<endl;
	for(size_t j=0;j<profile.size();j++)
	  out<<profile[j].first<<"\t"<<profile[j].second<<endl;
      }
      }
    
      goalRadius = 0.01;
      Refresh();      
    }
    else if(key == 'b') {
      //plan bidirectional
      if(!hasStart) {
	if(!SampleConfig(start)) return;
	hasStart=true;
      }
      if(!hasGoal) {
	if(!SampleConfig(goal)) return;
	hasGoal=true;
      }
      int expandIters = 2000;
      vector<vector<int> > successes(expandIters+1);
      vector<vector<int> > counts(expandIters+1);
      for(size_t i=0;i<=expandIters;i++) {
	successes[i].resize(expandIters+1,0);
	counts[i].resize(expandIters+1,0);
      }
      for(int i=0;i<=10;i++) {
	planner.expandBackwardFraction = Real(i)/20.0;
	printf("Testing bidir, backward fraction %g\n",planner.expandBackwardFraction);
	Real b = planner.expandBackwardFraction/(1.0-planner.expandBackwardFraction);
	Real nadjust = Real(expandIters)*2.0*Sqrt(1.0+Sqr(b));
	printf("Num iters %g\n",nadjust);
	int numFailed=0;
	for(int test=0;test<100;test++) {
	  planner.Init(start,goal,goalRadius);
	  planner.bwdTrees.push_back(planner.planner.AddMilestone(goal));
	  bool res=planner.PlanBidirectional((int)nadjust);
	  for(int j=0;j<=Min(planner.numForwardExtends,expandIters);j++)
	    for(int k=0;k<=Min(planner.numBackwardExtends,expandIters);k++) {
	      counts[j][k]++;
	      counts[k][j]++;
	    }
	  if(res) {
	    for(int j=planner.numForwardExtends;j<=expandIters;j++) {
	      for(int k=planner.numBackwardExtends;k<=expandIters;k++) {
		counts[j][k]++;
		counts[k][j]++;
		successes[j][k]++;
		successes[k][j]++;
	      }
	    }
	    //eliminate double counting
	    if(planner.numForwardExtends <= expandIters && planner.numBackwardExtends <= expandIters) {
	      counts[planner.numForwardExtends][planner.numBackwardExtends]--;
	      counts[planner.numBackwardExtends][planner.numForwardExtends]--;
	    }
	  }
	  else numFailed++;
	}
	printf("%d failed\n",numFailed);
      }
      printf("Saving to bidir_stats.txt\n");
      ofstream out("bidir_stats.txt");
      for(size_t i=0;i<successes.size();i+=100) {
	for(size_t j=0;j<successes.size();j+=100) {
	  if(counts[i][j] == 0)
	    out<<"0 ";
	  else
	    out<<Real(successes[i][j])/Real(counts[i][j])<<" ";
	}
	out<<endl;
      }
      printf("Saving to bidir_stats2.txt\n");
      vector<vector<Real> > cols(11);
      for(int i=0;i<=10;i++) {
	planner.expandBackwardFraction = Real(i)/20.0;
	cols[i].push_back(planner.expandBackwardFraction);
	Real b = planner.expandBackwardFraction/(1.0-planner.expandBackwardFraction);
	Real adjust = Sqrt(1.0+Sqr(b));
	for(int iters=0;iters<=expandIters;iters+=100) {
	  int x = (int)(Real(iters)/adjust);
	  int y = (int)(b*Real(iters)/adjust);
	  if(counts[x][y] == 0) cols[i].push_back(0);
	  else cols[i].push_back(Real(successes[x][y])/Real(counts[x][y]));
	}
      }
      {
	ofstream out("bidir_stats2.txt");
	out<<"#iters\t";
	for(size_t i=0;i<cols.size();i++)  out<<cols[i][0]<<"\t";
	out<<endl;
	for(size_t j=1;j<cols[0].size();j++) {
	  out<<(j-1)*100<<"\t";
	  for(size_t i=0;i<cols.size();i++)
	    out<<cols[i][j]<<"\t";
	  out<<endl;
	}
      }
      printf("Done\n");
      planner.expandBackwardFraction = 0.5;
      Refresh(); 
    }
    else if(key == 's') {
      if(!hasPath) {
	if(!SampleProblem()) return;
      }
    }
    else if(key == 'a') {
      Config q(2),q2(2);
      Statistics::Histogram hist;
      hist.Resize(100,0,0.2);
      for(int sample=0;sample<100;sample++) {
	printf("Sample %d...\n",sample);
	SampleConfig(q);
	Real d0 = vgraph.Distance(q,goal);
	int numSamples=1000;
	vector<Real> gap(numSamples);
	Real d,cost;
	for(int n=0;n<numSamples;n++) {
	  cspace.SampleNeighborhood(q,0.1,q2);
	  cost = cspace.Distance(q,q2);
	  if(!cspace.IsFeasible(q2)) 
	    d=Inf;
	  else
	    d=vgraph.Distance(q2,goal);
	  gap[n] = d+cost - d0;
	}
	for(size_t i=0;i<gap.size();i++) hist.AddBucket(gap[i]);
      }
      ofstream out("dSample.txt");
      out<<"#bucket    PDF    CDF"<<endl;
      Real sum=0;
      for(size_t i=0;i<hist.divs.size();i++) {
	sum += hist.buckets[i+1];
	out<<hist.divs[i]<<"    "<<hist.buckets[i+1]<<" "<<sum<<endl;
      }
      out.close();
    }
  }

  virtual void Handle_Click(int button,int state,int x,int y) {
    if(state == GLUT_DOWN) {
      Config q;
      ClickToConfig(x,y,q);
      if(!hasStart) {
	start=q;
	hasStart=true;
      }
      else if(!hasGoal) {
	goal=q;
	hasGoal=true;
	planner.Init(start,goal,goalRadius);
	hasPath = false;
      }
      else if(!hasPath) {
	if(planner.PlanForward(1000)) {
	  planner.CreatePath(path);
	  hasPath = true;
	}
      }
      else if(hasPath) {
	hasStart=false;
	hasGoal=false;
	hasPath=false;
      }
      Refresh();
    }
  }
};
