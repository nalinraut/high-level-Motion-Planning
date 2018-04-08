#include <KrisLibrary/utils/StatCollector.h>
#include <KrisLibrary/utils/ObjectPrinter.h>
#include <KrisLibrary/AI/CSpaceMDP.h>
#include <KrisLibrary/AI/MDPPolicy.h>
#include <KrisLibrary/AI/FixedMDP.h>
#include <KrisLibrary/planning/KinodynamicMotionPlanner.h>
#include "MotionPlannerProgram.h"
#include "CarCSpace.h"
#include <KrisLibrary/Timer.h>
using namespace std;
using namespace AI;

typedef ObjectPrinter<StatCollector> StatPrinter;


class CarCSpaceAdaptor : public CarCSpace
{
public:
  CarCSpaceAdaptor(CSpace* _obstacleSpace,bool _obstacle2d=true)
    :obstacleSpace(_obstacleSpace),obstacle2d(_obstacle2d)
  {}

  virtual bool IsFeasible(const Config& x) {
    if(x(0) < bmin(0) || x(0) > bmax(0)) return false;
    if(x(1) < bmin(1) || x(1) > bmax(1)) return false;
    if(obstacleSpace) {
      if(obstacle2d) {
	Config x2(2);
	x2(0)=x(0);
	x2(1)=x(1);
	return obstacleSpace->IsFeasible(x2);
      }
      else {
	return obstacleSpace->IsFeasible(x);
      }
    }
    return true;
  }

  CSpace* obstacleSpace;
  bool obstacle2d;
};

struct DrawCarCallback : public KinodynamicTree::Node::Callback
{
  bool ForwardEdge(KinodynamicTree::Node* i,KinodynamicTree::Node* j)
  {
    //const Config& x=*i, &y=*j;
    const KinodynamicTree::EdgeData& e=j->edgeFromParent();
    edgeColor.setCurrentGL();
    glBegin(GL_LINE_STRIP);
    for(size_t k=0;k<e.path.size();k++)
      glVertex2v(e.path[k]);
    glEnd();
    return true;
  }

  void Visit(KinodynamicTree::Node* node)
  {
    nodeColor.setCurrentGL();
    glBegin(GL_POINTS);
    glVertex2v(*node);
    glEnd();
  }

  GLColor nodeColor,edgeColor;
};



///Assists with selective overriding of another cspace's methods
class PiggybackCSpace : public CSpace
{
public:
  PiggybackCSpace(CSpace* _baseSpace=NULL)
    :baseSpace(_baseSpace)
  {}
  virtual void Sample(Config& x) {
    Assert(baseSpace!=NULL);
    baseSpace->Sample(x);
  }
  virtual void SampleNeighborhood(const Config& c,Real r,Config& x) {
    if(baseSpace!=NULL) baseSpace->SampleNeighborhood(c,r,x);
    else CSpace::SampleNeighborhood(c,r,x);
  }
  virtual EdgePlanner* LocalPlanner(const Config& a,const Config& b) {
    Assert(baseSpace != NULL);
    return baseSpace->LocalPlanner(a,b);
  }
  virtual bool IsFeasible(const Config& x) {
    if(baseSpace) return baseSpace->IsFeasible(x);
    else return true;
  }
  virtual Real Distance(const Config& x, const Config& y) {
    if(baseSpace) return baseSpace->Distance(x,y);
    else return CSpace::Distance(x,y);
  }
  virtual void Interpolate(const Config& x,const Config& y,Real u,Config& out) {
    if(baseSpace) return baseSpace->Interpolate(x,y,u,out);
    else return CSpace::Interpolate(x,y,u,out);
  }
  virtual void Midpoint(const Config& x,const Config& y,Config& out) {
    if(baseSpace) return baseSpace->Midpoint(x,y,out);
    else return CSpace::Midpoint(x,y,out);
  }

  CSpace* baseSpace;
};

class NeighborhoodCSpace : public PiggybackCSpace
{
public:
  NeighborhoodCSpace()
    :radius(0)
  {}
  NeighborhoodCSpace(const Config& _center,Real _radius)
    :center(_center),radius(_radius)
  {}
  NeighborhoodCSpace(CSpace* _baseSpace,const Config& _center,Real _radius)
    :PiggybackCSpace(_baseSpace),center(_center),radius(_radius)
  {}
  virtual void Sample(Config& x) { SampleNeighborhood(center,radius,x); }
  virtual bool IsFeasible(const Config& x) { 
    return (Distance(x,center) < radius); 
  }

  Config center;
  Real radius;
};

class TargetCSpaceMDP : public CSpaceMDP
{
public:
  TargetCSpaceMDP(CSpace* cspace,CSpace* _goalSet)
    :CSpaceMDP(&adaptor),adaptor(cspace),goalSet(_goalSet)
  {
  }
  virtual Value Reward(const State& s) {
    if(IsTerminal(s)) return 0;
    FatalError("Reward called without a state-action pair");
    return 0;
  }
  virtual Value Reward(const State& s,const Action& a) {
    if(IsTerminal(s)) return 0;
    return CSpaceMDP::Reward(s,a);
  }
  virtual bool IsTerminal(const Config& s) {
    return goalSet->IsFeasible(s);
  }

  KinematicCSpaceAdaptor adaptor;
  CSpace* goalSet;
};

class GuidedKinodynamicPlanner
{
public:
  typedef KinodynamicTree::Node Node;

  GuidedKinodynamicPlanner(KinodynamicCSpace* _space)
    :space(_space),
     numSampleControlIters(20),pickGreedyControl(false),nonGreedyCutoff(0.2),
     softmaxExponent(Inf),
     tree(_space)
  {}

  virtual Real Value(const State& x)=0;
  virtual Real QValue(const State& x,const ControlInput& u)=0;
  virtual Real Cost(const State& x,const ControlInput& u,const State& x2)=0;
  //Two possible models:
  //1) successor update.  Change the successor function to reflect the new
  //   successor distribution, new cost of the action.
  //2) value update.  Change the value function to have the new value at
  //   qnext and some neighborhood around it
  virtual void UpdateModel(const Node* n,
			   const ControlInput& u,
			   const State& qnext,
			   Real value)=0;

  virtual void Clear() 
  {
    tree.Clear();
    nodeIndex.clear();
    candidateNodes.clear();
    expandedNodes.clear();
    createdNodes.clear();
    failedNodes.clear();
    candidateExpansions.clear();
  }

  void Init(const State& x) {
    tree.Init(x);
    AddNode(tree.root);
  }

  Real PlanningCost(Real time) const { return time*10.0+0.05; }

  Node* PickNode() {
    Node* best=NULL;
    Real bestValue = -Inf;
    for(size_t i=0;i<nodeIndex.size();i++) {
      Real val=Value(*nodeIndex[i]);
      if(val > bestValue) {
	best = nodeIndex[i];
	bestValue = val;
      }
    }
    return best;
  }

  Real MaxValueNodes(vector<Node*>& best)
  {
    best.resize(0);
    Real bestValue = -Inf;
    for(size_t i=0;i<nodeIndex.size();i++) {
      Real val=Value(*nodeIndex[i]);
      if(val > bestValue) {
	best.resize(1);
	best[0] = nodeIndex[i];
	bestValue = val;
      }
      else if(val == bestValue) {
	best.push_back(nodeIndex[i]);
      }
    }
    return bestValue;
  }

  Real PickControl(Node* n,ControlInput& ubest)
  {
    ControlInput u;
    Real vn = Value(*n),vbest=-Inf;
    for(int i=0;i<numSampleControlIters;i++) {
      space->SampleControl(*n,u);
      Real vnext = QValue(*n,u);
      if(vnext > vbest) {
	vbest = vnext;
	ubest = u;
      }
    }
    return vbest;
  }

  Node* PickNodeAndControl(ControlInput& ubest) {
    candidateNodes.resize(0);
    candidateNodes.reserve(20);
    candidateExpansions.resize(0);
    candidateExpansions.reserve(numSampleControlIters);

    if(pickGreedyControl) {
      Real maxValue = MaxValueNodes(candidateNodes);
      Node* nbest = NULL;
      Real bestValue = -Inf;
      ControlInput u;
      for(int iters=0;iters<numSampleControlIters;iters++) {
	Node* n=candidateNodes[RandInt(candidateNodes.size())];
	space->SampleControl(*n,u);
	candidateExpansions.push_back(pair<Node*,ControlInput>(n,u));
	Real val = QValue(*n,u);
	if(val > bestValue) {
	  nbest = n;
	  ubest = u;
	  bestValue = val;
	}
      }
      if(nbest) {
	//HACKAGE!
	if(bestValue < maxValue - 0.1) {
	  //cout<<"Best motion decreased signficantly"<<endl;
	}
	else {
	  State xnew;
	  space->SimulateEndpoint(*nbest,ubest,xnew);
	  if(Value(xnew) == maxValue) {
	    //cout<<"Estimate didn't move?"<<endl;
	  }
	  else {
	    return nbest;
	  }
	}
      }
    }

    //Timer timer;
    Node* best=NULL;
    Real bestValue = -Inf;
    for(size_t i=0;i<nodeIndex.size();i++) {
      Real val=Value(*nodeIndex[i]);
      if(val > bestValue) {
	best = nodeIndex[i];
	bestValue = val;
      }
    }
    //cout<<"Time to pick best value: "<<timer.ElapsedTime()<<endl;
    //timer.Reset();
    for(size_t i=0;i<nodeIndex.size();i++) {
      Real val=Value(*nodeIndex[i]);
      if(val > bestValue - nonGreedyCutoff) {
	candidateNodes.push_back(nodeIndex[i]);
      }
    }
    //cout<<"Time to pick best candidates: "<<timer.ElapsedTime()<<endl;
    //timer.Reset();

    ControlInput u;
    if(IsInf(softmaxExponent)) {
      best=NULL;
      bestValue = -Inf;
      for(int i=0;i<numSampleControlIters;i++) {
	Node* n=candidateNodes[RandInt(candidateNodes.size())];
	space->SampleControl(*n,u);
	candidateExpansions.push_back(pair<Node*,ControlInput>(n,u));
	Real vnext=QValue(*n,u);
	if(vnext > bestValue) {
	  best = n;
	  bestValue = vnext;
	  ubest = u;
	}
      }
    }
    else {
      bestValue = -Inf;
      vector<Real> weights(numSampleControlIters);
      for(int i=0;i<numSampleControlIters;i++) {
	Node* n=candidateNodes[RandInt(candidateNodes.size())];
	space->SampleControl(*n,u);
	candidateExpansions.push_back(pair<Node*,ControlInput>(n,u));
	Real vnext=QValue(*n,u);
	weights[i] = vnext;
	if(vnext > bestValue) bestValue = vnext;
      }
      for(int i=0;i<numSampleControlIters;i++) 
	weights[i] = Exp(softmaxExponent*(weights[i]-bestValue));
      Real sumweight = 0;
      for(int i=0;i<numSampleControlIters;i++) 
	sumweight += weights[i];
      Real pickedVal = Rand()*sumweight;
      for(int i=0;i<numSampleControlIters;i++) {
	pickedVal -= weights[i];
	if(pickedVal < 0) {
	  ubest = candidateExpansions[i].second;
	  return candidateExpansions[i].first;
	}
      }
      //uh... some kind of error?
      ubest = candidateExpansions[0].second;
      return candidateExpansions[0].first;
    }
    //cout<<"Time to pick control: "<<timer.ElapsedTime()<<endl;
    //timer.Reset();
    return best;
  }

  void AddNode(Node* n) {
    nodeIndex.push_back(n);
  }

  void Extend() {
    Timer timer;
    ControlInput u;
    Node* n=PickNodeAndControl(u);
    if(!n) return;

    timer.Reset();
    if(n != NULL) {
      vector<State> path;
      space->Simulate(*n,u,path);
      EdgePlanner* e = space->TrajectoryChecker(path);
      if(e->IsVisible()) {
	Node* c=tree.AddMilestone(n,u,path,e);
	Assert(*c == path.back());
	Real planningTime = timer.ElapsedTime();
	Real value = Value(*c) - Cost(*n,u,*c);
	
	//cout<<"Updating value at "<<(*n)[0]<<","<<(*n)[1]<<" from "<<Value(*n)<<" to "<<value<<", planning time "<<planningTime<<endl;
	//cout<<"  Value at end of path "<<(*c)[0]<<","<<(*c)[1]<<" is "<<Value(*c)<<", cost of path is "<<Cost(*n,u,*c)<<" planning cost "<<PlanningCost(planningTime)<<endl;
	
	UpdateModel(n,u,*c,value-PlanningCost(planningTime));
	AddNode(c);

	expandedNodes.push_back(n);
	createdNodes.push_back(c);
	return;
      }
    }

    failedNodes.push_back(n);
    Real planningTime = timer.ElapsedTime();
    Real vn = Value(*n);
    //cout<<"Infeasible! reducing value by "<<PlanningCost(planningTime)<<endl;
    UpdateModel(n,u,*n,vn-PlanningCost(planningTime));
  }

  KinodynamicCSpace* space;
  int numSampleControlIters;
  bool pickGreedyControl;
  Real nonGreedyCutoff;
  Real softmaxExponent;
  vector<Node*> nodeIndex;
  KinodynamicTree tree;

  //temp: for visual debugging
  vector<Node*> candidateNodes;
  vector<Node*> expandedNodes;
  vector<Node*> createdNodes;
  vector<Node*> failedNodes;
  vector<pair<Node*,ControlInput> > candidateExpansions;
};

class GuidedCarPlanner : public GuidedKinodynamicPlanner
{
public:
  GuidedCarPlanner(KinodynamicCSpace* cspace)
    :GuidedKinodynamicPlanner(cspace),
     mdp(NULL),values(NULL),solver(NULL),
     updateChunk(1),
     curUpdateChunk(0),
     updateFraction(0.1)
  {}

  virtual void Clear() {
    GuidedKinodynamicPlanner::Clear();
    updateDeltas.clear();
    curUpdateChunk=0;
  }

  void GetIndex(const State& x,Grid::Index& index) const {
    State y;
    ReduceState(x,y);
    values->grid.PointToIndex(y,index);
  }

  void ReduceState(const State& x,State& y) const
  {
    y.resize(2);
    y(0)=x(0);
    y(1)=x(1);
  }

  void ReduceAction(const State& x,const ControlInput& u,ControlInput& v) const
  {
    State y;
    space->SimulateEndpoint(x,u,y);
    v.resize(2);
    v(0)=y(0);
    v(1)=y(1);
  }

  virtual Real Value(const State& x) {
    Grid::Index i;
    GetIndex(x,i);
    Real val = values->table[i];
    return val;
  }
  virtual Real QValue(const State& x,const ControlInput& u) {
    State xnext;
    if(!space->NextState(x,u,xnext)) return -Inf;
    //space->SimulateEndpoint(x,u,xnext);
    //if(!space->IsFeasible(xnext)) return -Inf;
    return Value(xnext) - Cost(x,u,xnext);

    /*
    State y,y2;
    ControlInput v;
    ReduceState(x,y);
    ReduceAction(x,u,v);
    mdp->DeterministicTransition(y,v,y2);
    if(y2 == y) return -Inf;
    Real val;
    values->QueryV(y2,val);
    val += mdp->Reward(y,v);

    //adjust by the delta
    Grid::Index index;
    values->grid.PointToIndex(y,index);
    Real adj = 0;
    UpdateStorage::const_iterator i=updateDeltas.find(index);
    if(i != updateDeltas.end()) adj = -i->second;

    return val + adj;
    */
  }
  virtual Real Cost(const State& x,const ControlInput& u,const State& x2) {
    return Abs(u(1));
  }
  virtual void UpdateModel(const Node* n,
			   const ControlInput& u,
			   const State& qnext,
			   Real value)
  {
    /*
    //TD(0)
    Grid::Index i;
    GetIndex(*n,i);
    if(updateDeltas.find(i) == updateDeltas.end()) 
      updateDeltas[i] = 0;
    Real oldValue = values->table[i];
    updateDeltas[i] += updateFraction*(value-oldValue);
    values->table[i] += updateFraction*(value-oldValue);
    //cout<<"Updating "<<VectorPrinter(q)<<" from "<<oldValue<<" to "<<value<<endl;

    curUpdateChunk += Abs(value-oldValue);
    if(curUpdateChunk > updateChunk) {
      RerunLearning();
      curUpdateChunk = 0;
    }
    */
    //TD(lambda)
    Grid::Index i;
    GetIndex(*n,i);
    Real oldValue = values->table[i];
    Real lambda=0.9;
    Real factor = 1.0;
    Real sumDelta = 0;
    while(n != NULL) {
      GetIndex(*n,i);
      if(updateDeltas.find(i) == updateDeltas.end()) 
	updateDeltas[i] = 0;
      Real oldValue = values->table[i];
      updateDeltas[i] += factor*(value-oldValue);
      values->table[i] += factor*(value-oldValue);
      sumDelta += factor*(value-oldValue);

      factor *= lambda;
      n = n->getParent();
      if(factor < 1e-3) break;
    }
    /*
    curUpdateChunk += Abs(sumDelta);
    if(curUpdateChunk > updateChunk) {
      RerunLearning();
      curUpdateChunk = 0;
    }
    */

  }

  struct TreeBackupCallback : public Graph::CallbackBase<KinodynamicTree::Node*>
  {
    GuidedCarPlanner* planner;
    
    virtual void VisitNode(KinodynamicTree::Node* n) {
      KinodynamicTree::Node* c=n->getFirstChild();
      Real vmax = -Inf;
      if(c==NULL) vmax = planner->Value(*n);  //no children
      while(c != NULL) {
	Real vc = planner->Value(*c) - planner->Cost(*n,c->edgeFromParent().u,*c);
	if(vc > vmax) vmax=vc;
	c=c->getNextSibling();
      }
      Grid::Index i;
      planner->GetIndex(*n,i);
      planner->values->table[i] = vmax;
    }
  };

  void UpdateValuesFromTree()
  {
    cout<<"Updating the values from the planning tree"<<endl;
    //back up from leaf nodes, if it increases the utility value
    TreeBackupCallback backupCallback;
    backupCallback.planner = this;
    tree.root->DFS(backupCallback);
  }

  void RerunLearning()
  {
    cout<<"Rerunning the learning algorithm, making "<<updateDeltas.size()<<" updates..."<<endl;

    //run value iteration

    int maxValueIters=5;
    for(int iters=0;iters<maxValueIters;iters++) {

    Config s;
    for(CSpaceMDPGridValueStorage::ValueTable::const_iterator i=values->table.begin();i!=values->table.end();i++) {
      s.resize(2);
      bool found=false;
      for(int iters=0;iters<50;iters++) {
	s(0) = (Real(i->first[0])+Rand())*values->grid.h(0);
	s(1) = (Real(i->first[1])+Rand())*values->grid.h(1);
	if(mdp->space->IsFeasible(s)) {
	  found=true;
	  break;
	}
      }
      if(!found) continue;
      
      solver->ValueIteration(s);
      if(updateDeltas.find(i->first)!=updateDeltas.end())
	values->table[i->first] += updateDeltas[i->first];
    }


      /*
    CSpaceMDPGridValueStorage::ValueTable newTable = values->table;

    Config s;
    //for(int iters=0;iters<1000;iters++) {
    for(CSpaceMDPGridValueStorage::ValueTable::const_iterator i=values->table.begin();i!=values->table.end();i++) {
      //mdp->SampleState(s);
      s.resize(2);
      bool found=false;
      for(int iters=0;iters<50;iters++) {
	s(0) = (Real(i->first[0])+Rand())*values->grid.h(0);
	s(1) = (Real(i->first[1])+Rand())*values->grid.h(1);
	if(mdp->space->IsFeasible(s)) {
	  found=true;
	  break;
	}
      }
      if(!found) continue;

      if(updateDeltas.find(i->first) != updateDeltas.end())
	values->table[i->first] -= updateDeltas[i->first];
      Real oldValue = values->table[i->first];
      solver->ValueIteration(s);
      newTable[i->first]=values->table[i->first];
      values->table[i->first]=oldValue;
    }

    for(UpdateStorage::const_iterator k=updateDeltas.begin();k!=updateDeltas.end();k++) {
      //if(k->second < 0)
	newTable[k->first] += k->second;
    }

    values->table = newTable;
      */

    }

    //UpdateValuesFromTree();
  }

  CSpaceMDP* mdp;
  CSpaceMDPGridValueStorage* values;
  CSpaceMDPSolver* solver;

  typedef map<Grid::Index,Real> UpdateStorage;
  UpdateStorage updateDeltas;
  Real updateChunk;  //the absolute value of changes before a value update is made
  Real curUpdateChunk;  //the current amount of changes
  Real updateFraction;  //the fraction of the way the value update is performed
};

struct MDPProgram : public MotionPlannerProgram
{
  CarCSpaceAdaptor carCspace;
  TargetCSpaceMDP mdp;
  CSpaceMDPGridValueStorage valueStorage;
  CSpaceMDPSolver solver;
  NeighborhoodCSpace goalSet;
  NeighborhoodCSpace carGoalSet;
  RRTKinodynamicPlanner planner;
  GuidedCarPlanner guidedPlanner;
  FixedMDP fixedmdp;
  vector<vector<Config> > fixedactions;

  vector<vector<Config> > traces;
  int curQView;

  MDPProgram()
    :mdp(&cspace,&goalSet),valueStorage(mdp),
     solver(mdp,valueStorage),
     carCspace(&mdp.adaptor),
     planner(&carCspace),guidedPlanner(&carCspace)
  {
    Vector bmin(2,cspace.domain.bmin);
    Vector bmax(2,cspace.domain.bmax-Vector2(1e-5));
    //division of grid
    valueStorage.SetDivisionSize(0.05);
    valueStorage.Initialize(bmin,bmax,0);
    valueStorage.defaultValue = -10;
    valueStorage.qtables.resize(8);
    goalSet.baseSpace = &cspace;
    goalSet.center.resize(2);
    //goal position and radius
    goalSet.center(0) = 0.9;
    goalSet.center(1) = 0.1;
    goalSet.radius = 0.05;
    carGoalSet.baseSpace = &carCspace;
    carGoalSet.center.resize(3);
    carGoalSet.center(0) = goalSet.center(0);
    carGoalSet.center(1) = goalSet.center(1);
    carGoalSet.center(2) = 0;
    carGoalSet.radius = goalSet.radius;

    //radius of MDP sampling neighborhood
    mdp.adaptor.maxNeighborhoodRadius = 0.05;

    //max forward/backward movement of car
    carCspace.dmin = -0.1;
    carCspace.dmax = 0.1;
    //max left/right turn radius of car
    carCspace.turnmin = -20;
    carCspace.turnmax = 20;
    carCspace.angleWeight = 0.1;
    //RRT config
    planner.goalSet = &carGoalSet;
    planner.goalSeekProbability = 0.1;
    //start config
    Config initialState(3);
    initialState(0) = 0.1;
    initialState(1) = 0.1;
    initialState(2) = 0;
    planner.Init(initialState);
    guidedPlanner.mdp = &mdp;
    guidedPlanner.values = &valueStorage;
    guidedPlanner.solver = &solver;
    guidedPlanner.Init(initialState);

    mdp.rewardConstant = -0.05;

    curQView=-1;
    InitDefaultValues();
  }

  void InitDefaultValues()
  {
    Config bmin,bmax;
    for(CSpaceMDPGridValueStorage::ValueTable::iterator i=valueStorage.table.begin();i!=valueStorage.table.end();i++) {
      valueStorage.grid.CellBounds(i->first,bmin,bmax);
      bmin.inplaceMul(0.5);
      bmin.madd(bmax,0.5);
      //if(cspace.IsFeasible(bmin) || !IsInf(i->second)) {
	i->second = -cspace.Distance(goalSet.center,bmin);
	//}
    }
  }

  virtual void Handle_Display() {
    SetupDisplay();
    DrawCSpace();

    glColor3f(1,0,0);
    drawCircle2D(Vector2(goalSet.center(0),goalSet.center(1)),goalSet.radius);
    
    CSpaceMDPGridValueStorage::ValueTable::const_iterator i;
    CSpaceMDPGridValueStorage::ValueTable& table=(curQView<0?valueStorage.table:valueStorage.qtables[curQView]);
    Real minValue=Inf,maxValue=-Inf;
    for(i=table.begin();i!=table.end();i++) {
      if(IsFinite(i->second)) {
	if(i->second < minValue) minValue=i->second;
	if(i->second > maxValue) maxValue=i->second;
      }
    }
    char buf[256];
    sprintf(buf,"Action %d, Min value: %g, max value %g",curQView,minValue,maxValue);
    glColor3f(0,0,0);
    glRasterPos2f(0.1,0.9);
    void* fontface = GLUT_BITMAP_HELVETICA_10;
    int n=strlen(buf);
    for(int i=0;i<n;i++) 
      glutBitmapCharacter(fontface,buf[i]);

    //draw sampled actions
    if(!fixedactions.empty()) {
      glColor3f(0.5,0.5,1);
      glBegin(GL_LINES);
      int fixedstate=0;
      for(map<Grid::Index,Real>::const_iterator i=valueStorage.table.begin();i!=valueStorage.table.end();i++,fixedstate++) {
	Vector cmin,cmax;
	valueStorage.grid.CellBounds(i->first,cmin,cmax);
	cmin *= 0.5;
	cmin.madd(cmax,0.5);
	for(size_t j=0;j<fixedactions[fixedstate].size();j++) {
	  glVertex2v(cmin);
	  glVertex2v(fixedactions[fixedstate][j]);
	}
      }
      glEnd();
    }

    //draw best actions
    glColor3f(0,0,1);
    glBegin(GL_LINES);
    for(map<Grid::Index,ControlInput>::const_iterator i=valueStorage.bestActions.begin();i!=valueStorage.bestActions.end();i++) {
      Vector cmin,cmax;
      valueStorage.grid.CellBounds(i->first,cmin,cmax);
      cmin *= 0.5;
      cmin.madd(cmax,0.5);
      glVertex2v(cmin);
      glVertex2v(i->second);
    }
    glEnd();

    
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    glBegin(GL_QUADS);
    for(i=table.begin();i!=table.end();i++) {
      Grid::Index index=i->first;
      Vector cmin,cmax;
      valueStorage.grid.CellBounds(index,cmin,cmax);
      Real value=(i->second-minValue)/(maxValue-minValue);
      GLColor col;
      if(!IsFinite(value))
	col.set(0.5,0.5,0.5,0.5);
      else if(value < 0)
	col.set(1,1,1,0.5);
      else {
	col.setHSV(value*180,1,1);
	col.rgba[3]=0.5;
      }
      col.setCurrentGL();
      glVertex2f(cmin(0),cmin(1));
      glVertex2f(cmax(0),cmin(1));
      glVertex2f(cmax(0),cmax(1));
      glVertex2f(cmin(0),cmax(1));
    }
    glEnd();
    glDisable(GL_BLEND);

    for(GuidedCarPlanner::UpdateStorage::const_iterator k=guidedPlanner.updateDeltas.begin();k!=guidedPlanner.updateDeltas.end();k++) {
      Vector cmin,cmax;
      valueStorage.grid.CellBounds(k->first,cmin,cmax);
      Real value=k->second;
      GLColor col;
      if(!IsFinite(value))
	col.set(0.5,0.5,0.5,0.5);
      else {
	if(value < 0)
	  col.set(1,0,0);
	else
	  col.set(0,0,1);
      }
      col.setCurrentGL();

      glMatrixMode(GL_MODELVIEW);
      glPushMatrix();
      glTranslatef(cmin(0),cmin(1),0);
      glScalef(cmax(0)-cmin(0),cmax(1)-cmin(1),1);
      {
	glBegin(GL_LINES);
	if(value < 0) { //draw down arrow
	  Real endpt = 0.5+0.5*value*10.0;
	  glVertex2f(0.2,0.5);
	  glVertex2f(0.2,endpt);
	  glVertex2f(0.1,0.1+endpt);
	  glVertex2f(0.2,endpt);
	  glVertex2f(0.3,0.1+endpt);
	  glVertex2f(0.2,endpt);
	}
	else {  //draw up arrow
	  Real endpt = 0.5+0.5*value*10.0;
	  glVertex2f(0.2,0.5);
	  glVertex2f(0.2,endpt);
	  glVertex2f(0.1,-0.1+endpt);
	  glVertex2f(0.2,endpt);
	  glVertex2f(0.3,-0.1+endpt);
	  glVertex2f(0.2,endpt);
	}
	glEnd();
      }
      glPopMatrix();
    }


    for(size_t i=0;i<traces.size();i++) {
      glColor3f(0,0,0);
      glPointSize(3.0);
      glBegin(GL_POINTS);
      for(size_t j=0;j<traces[i].size();j++) 
	glVertex2f(traces[i][j](0),traces[i][j](1));
      glEnd();
      glBegin(GL_LINE_STRIP);
      for(size_t j=0;j<traces[i].size();j++) 
	glVertex2f(traces[i][j](0),traces[i][j](1));
      glEnd();
    }

    //draw RRT planner tree
    DrawCarCallback callback;
    callback.nodeColor.set(0,1,0);
    callback.edgeColor.set(0,0.5,0);
    glPointSize(5.0);
    glLineWidth(2.0);
    if(planner.tree.root)
      planner.tree.root->DFS(callback);

    //draw guided planner tree
    callback.nodeColor.set(0,1,1);
    callback.edgeColor.set(0,0.5,0.5);
    if(guidedPlanner.tree.root)
      guidedPlanner.tree.root->DFS(callback);
    glLineWidth(1.0);

    //draw expansion feedback
    //candidate expansion nodes
    glPointSize(5.0);
    glColor3f(0,0.5,1);
    glBegin(GL_POINTS);
    for(size_t i=0;i<guidedPlanner.candidateNodes.size();i++)
      glVertex2v(*guidedPlanner.candidateNodes[i]);
    glEnd();
    //candidate expansion paths
    glColor3f(0,0,0);
    for(size_t i=0;i<guidedPlanner.candidateExpansions.size();i++) { 
      vector<State> path;
      carCspace.Simulate(*guidedPlanner.candidateExpansions[i].first,guidedPlanner.candidateExpansions[i].second,path);
      glBegin(GL_LINE_STRIP);
      for(size_t j=0;j<path.size();j++)
	glVertex2v(path[j]);
      glEnd();
    }
    //expanded, created, failed nodes
    glColor3f(0,0,1);
    glPointSize(7.0);
    glBegin(GL_POINTS);
    glColor3f(1,0,1);
    for(size_t i=0;i<guidedPlanner.expandedNodes.size();i++)
      glVertex2v(*guidedPlanner.expandedNodes[i]);
    glColor3f(0,0,1);
    for(size_t i=0;i<guidedPlanner.createdNodes.size();i++)
      glVertex2v(*guidedPlanner.createdNodes[i]);
    glColor3f(1,0,0);
    for(size_t i=0;i<guidedPlanner.failedNodes.size();i++)
      glVertex2v(*guidedPlanner.failedNodes[i]);
    glEnd();

    glutSwapBuffers();
  }

  virtual void Handle_Reshape(int w,int h) {
    GLUTProgramBase::Handle_Reshape(w,h);
    glViewport(0,0,(GLsizei)width,(GLsizei)height);
    glutPostRedisplay();
  }

  virtual void Handle_Click(int button,int state,int x,int y) {
    if(state == GLUT_DOWN) {
    }
  }
  virtual void Handle_Keypress(unsigned char c,int x, int y) {
    c = tolower(c);
    if(c=='v') {
      curQView=-1;
      traces.clear();
      Config s;
      map<Grid::Index,int> gridToIndex;
      int numcells = 0;
      for(CSpaceMDPGridValueStorage::ValueTable::const_iterator i=valueStorage.table.begin();i!=valueStorage.table.end();i++) {
	gridToIndex[i->first] = numcells;
	numcells++;
      }
      mdp.defaultNumActionSamples = 30;
      fixedmdp.isTerminal.resize(numcells);
      fixedmdp.successors.resize(numcells);
      fixedmdp.probabilities.resize(numcells);
      fixedmdp.actionBasedReward.resize(numcells);
      fill(fixedmdp.isTerminal.begin(),fixedmdp.isTerminal.end(),false);
      FixedMDPValueStorage fixedvalues(fixedmdp);
      fixedvalues.Q.clear();
      Timer timer;
      fixedactions.resize(numcells);
      //for(int iters=0;iters<1000;iters++) {
      static map<Grid::Index,Config> foundConfigs;
      for(CSpaceMDPGridValueStorage::ValueTable::const_iterator i=valueStorage.table.begin();i!=valueStorage.table.end();i++) {
	//mdp.SampleState(s);
	s.resize(2);
	bool found=false;
	for(int iters=0;iters<50;iters++) {
	  s(0) = (Real(i->first[0])+Rand())*valueStorage.grid.h(0);
	  s(1) = (Real(i->first[1])+Rand())*valueStorage.grid.h(1);
	  if(cspace.IsFeasible(s)) {
	    found=true;
	    break;
	  }
	}
	if(!found) {
	  continue;
	}

	//DEBUG: view the sample centers
	//traces.resize(traces.size()+1);
	//traces.back().push_back(s);
	//solver.ValueIteration(s);

	int index=gridToIndex[i->first];
	fixedactions[index].resize(0);
	if(mdp.IsTerminal(s)) {
	  fixedmdp.isTerminal[index] = true;
	  //do nothing
	}
	else {
	  mdp.GetActions(s,fixedactions[index]);
	  if(valueStorage.bestActions.find(i->first)!=valueStorage.bestActions.end())
	    fixedactions[index].push_back(valueStorage.bestActions[i->first]);
	}
	fixedmdp.actionBasedReward[index].resize(fixedactions[index].size());
	fixedmdp.successors[index].resize(fixedactions[index].size());
	fixedmdp.probabilities[index].resize(fixedactions[index].size());
	for(size_t j=0;j<fixedactions[index].size();j++) {
	  fixedmdp.actionBasedReward[index][j]=mdp.Reward(s,fixedactions[index][j]);
	  Assert(!mdp.IsTerminal(s));

	  vector<Config> successors;
	  mdp.GetSuccessors(s,fixedactions[index][j],successors,fixedmdp.probabilities[index][j]);
	  fixedmdp.successors[index][j].resize(successors.size());
	  for(size_t k=0;k<successors.size();k++) {
	    Grid::Index cell;
	    valueStorage.grid.PointToIndex(successors[k],cell);
	    int sindex=gridToIndex[cell];
	    fixedmdp.successors[index][j][k]=sindex;
	  }
	  bool valid = false;
	  for(size_t k=0;k<successors.size();k++) {
	    if(fixedmdp.successors[index][j][k] != index) valid=true;
	  }
	  if(!valid) {
	    fixedactions[index].erase(fixedactions[index].begin()+j);
	    fixedmdp.actionBasedReward[index].erase(fixedmdp.actionBasedReward[index].begin()+j);
	    fixedmdp.successors[index].erase(fixedmdp.successors[index].begin()+j);
	    fixedmdp.probabilities[index].erase(fixedmdp.probabilities[index].begin()+j);
	    j--;
	  }
	}
	fixedvalues.V[index] = i->second;
      }
      cout<<"Time to fix successors: "<<timer.ElapsedTime()<<endl;
      timer.Reset();

      vector<int> order(fixedmdp.successors.size());
      IdentityPermutation(order);
      for(int iters=0;iters<50;iters++) {
	RandomlyPermute(order);
	for(size_t i=0;i<order.size();i++) {
	  int s=order[i];
	  int aopt;
	  Real qnext;
	  if(!fixedmdp.successors[s].empty()) {
	    fixedvalues.PickOptimalActionV(s,aopt,qnext);
	    fixedvalues.V[s] = qnext;
	  }
	}
      }
      cout<<"Time to VI: "<<timer.ElapsedTime()<<endl;
      Real rate=1.0;
      for(CSpaceMDPGridValueStorage::ValueTable::iterator i=valueStorage.table.begin();i!=valueStorage.table.end();i++) {
	int s=gridToIndex[i->first];
	if(!fixedmdp.successors[s].empty()) {
	  int aopt;
	  Real qnext;
	  fixedvalues.PickOptimalActionV(s,aopt,qnext);
	  i->second += rate*(qnext-i->second);
	  valueStorage.bestActions[i->first]=fixedactions[s][aopt];
	}
	else {
	  i->second += rate*(fixedvalues.V[s]-i->second);
	}
      }
      
      Refresh();
    }
    else if(c=='q') {
      if(curQView<0) curQView=0;
      traces.clear();
      Config s,s2;
      Vector a;
      Real temp;
      mdp.SampleState(s);
      traces.resize(1);
      MDPEpsilonGreedyPolicy<Config,Vector,Real> policy(valueStorage);
      policy.epsilon = 0.0;
      for(int iters=0;iters<1000;iters++) {
	traces.back().push_back(s);
	if(mdp.IsTerminal(s)) {
	  Grid::Index sindex;
	  valueStorage.grid.PointToIndex(s,sindex);
	  for(size_t i=0;i<valueStorage.qtables.size();i++)
	    valueStorage.qtables[i][sindex] = mdp.Reward(s);

	  //break;
	  traces.resize(traces.size()+1);
	  mdp.SampleState(s);
	}
	else if(!cspace.IsFeasible(s)) {
	  traces.resize(traces.size()+1);
	  mdp.SampleState(s);
	}
	else {
	  policy.Eval(s,a,temp);
	  solver.QLearningIteration(s,a,0.6);
	  mdp.Simulate(s,a,s2);
	  s = s2;
	}
      }
      Refresh();
    }
    else if(c=='1') {
      curQView++;
      if(curQView>=(int)valueStorage.qtables.size())
	curQView=-1;
      Refresh();
    }
    else if(c=='r') {
      Assert(planner.tree.root != NULL);
      Config initialState = *planner.tree.root;
      guidedPlanner.Clear();
      InitDefaultValues();
      guidedPlanner.Init(initialState);
      planner.Init(initialState);
      Refresh();
    }
    else if(c=='p') {
      planner.Plan(1000);
      Refresh();
    }
    else if(c=='g') {
      guidedPlanner.candidateNodes.resize(0);
      guidedPlanner.candidateExpansions.resize(0);
      guidedPlanner.createdNodes.resize(0);
      guidedPlanner.failedNodes.resize(0);
      guidedPlanner.expandedNodes.resize(0);

      //for(int i=0;i<10;i++)
	guidedPlanner.Extend();

      for(size_t j=0;j<guidedPlanner.createdNodes.size();j++) {
        Config x=*guidedPlanner.createdNodes[j];
        x.n--;
	if(goalSet.IsFeasible(x)) {   //done! 
	  Graph::CountCallback<KinodynamicTree::Node*> callback;
	  guidedPlanner.tree.root->DFS(callback);
	  cout<<"Done at "<<callback.count<<" nodes"<<endl;
	}
      }
      Refresh();
    }
    else if(c=='m') {
      cout<<"Transferring V values to Q"<<endl;
      valueStorage.TransferVToQ();
      if(curQView < 0) curQView=0;
      Refresh();
    }
    else if(c=='n') {
      cout<<"Transferring Q values to V"<<endl;
      valueStorage.TransferQToV();
      curQView=-1;
      Refresh();
    }
    else if(c=='c') {
      CollectStats();
    }
  }

  Real PathCost(KinodynamicTree::Node* n) const
  {
    Real cost=0;
    KinodynamicTree::Node* p=n->getParent();
    while(p != NULL) {
      cost += Abs(n->edgeFromParent().u(1));

      n=p;
      p=n->getParent();
    }
    return cost;
  }

  void CollectStats()
  {
    const int numRuns = 20;
    const int maxIters = 100000;
    StatCollector planningTime,numIters,pathCost;
    /*
    cout<<"Calculating stats on RRT..."<<endl;
    for(int run=0;run<numRuns;run++) {
      cout<<"RRT run "<<run+1<<"..."<<endl;
      Timer timer;
      Assert(planner.tree.root != NULL);
      Config initialState = *planner.tree.root;
      planner.Init(initialState);
      int iters;
      for(iters=0;iters<maxIters;iters++) {
	RRTKinodynamicPlanner::Node* n=planner.Extend();
	if(n) {
	  Config x=*n; x.n--;
	  if(goalSet.IsFeasible(x)) {  //done!
	    planner.goalNode = n;
	    break;
	  }
        }
      }
      if(iters != maxIters) {
	cout<<"RRT succeeded in "<<iters<<" iters"<<endl;
	planningTime << timer.ElapsedTime();
	numIters << iters;
	pathCost << PathCost(planner.goalNode);

	cout<<"  Iterations: "<<numIters<<endl;
	cout<<"  Time: "<<planningTime<<endl;
	cout<<"  Cost: "<<pathCost<<endl;
      }
      else {
	cout<<"RRT failed after "<<maxIters<<" iters, "<<timer.ElapsedTime()<<"s"<<endl;
      }
    }
    cout<<"RRT stats: "<<endl;
    cout<<"  Failures: "<<numRuns-numIters.number()<<endl;
    cout<<"  Iterations: "<<numIters<<endl;
    cout<<"  Time: "<<planningTime<<endl;
    cout<<"  Cost: "<<pathCost<<endl;
    */

    numIters.clear();
    planningTime.clear();
    pathCost.clear();

    for(int run=0;run<numRuns;run++) {
      cout<<"Guided run "<<run+1<<"..."<<endl;
      Timer timer;
      Assert(planner.tree.root != NULL);
      Config initialState = *planner.tree.root;
      guidedPlanner.Clear();
      InitDefaultValues();
      guidedPlanner.RerunLearning();
      guidedPlanner.Init(initialState);
      KinodynamicTree::Node* goalNode=NULL;
      int iters;
      const int maxIters = 1000;
      for(iters=0;iters<maxIters;iters++) {
	guidedPlanner.createdNodes.clear();
	guidedPlanner.candidateExpansions.clear();
	guidedPlanner.expandedNodes.clear();
	guidedPlanner.candidateNodes.clear();
	guidedPlanner.failedNodes.clear();
	guidedPlanner.Extend();
	bool done=false;
	for(size_t j=0;j<guidedPlanner.createdNodes.size();j++) {
	  Config x=*guidedPlanner.createdNodes[j]; x.n--;
	  if(goalSet.IsFeasible(x)) {   //done! 
	    goalNode = guidedPlanner.createdNodes[j];
	    done=true;
	    break;
	  }
	}
	if(done) break;
      }
      if(iters != maxIters) {
	planningTime << timer.ElapsedTime();
	numIters << iters;
	pathCost << PathCost(goalNode);

	cout<<"Guided succeed after "<<iters<<" iters"<<endl;
	cout<<"  Iterations: "<<StatPrinter(numIters)<<endl;
	cout<<"  Time: "<<StatPrinter(planningTime)<<endl;
	cout<<"  Cost: "<<StatPrinter(pathCost)<<endl;
      }
      else {
	cout<<"Guided failed after "<<maxIters<<" iters, "<<timer.ElapsedTime()<<"s"<<endl;
      }
    }
    cout<<"Guided stats: "<<endl;
    cout<<"  Failures: "<<numRuns-numIters.number()<<endl;
    cout<<"  Iterations: "<<StatPrinter(numIters)<<endl;
    cout<<"  Time: "<<StatPrinter(planningTime)<<endl;
    cout<<"  Cost: "<<StatPrinter(pathCost)<<endl;
  }
};
