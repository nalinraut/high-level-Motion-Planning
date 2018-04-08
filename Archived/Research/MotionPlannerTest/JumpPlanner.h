#ifndef JUMP_PLANNER_H
#define JUMP_PLANNER_H

#include <KrisLibrary/planning/KinodynamicCSpace.h>
#include <KrisLibrary/planning/KinodynamicPath.h>
#include <KrisLibrary/graph/Graph.h>

class JumpPlanner
{
 public:
  struct Node
  {
    State x;
    Real obstacleDistance,goalSetMargin;
    ///Pareto frontier of path cost and margin cost.
    ///Stored in lexicograhical order.
    std::vector<Vector> paretoOptimalCosts;
    std::vector<int> paretoOptimalParents;
  };
  struct Edge
  {
    ///if isJump = true, then only jumpDistance is filled out.
    ///otherwise, u, path, and e are filled out
    bool isJump;
    ControlInput u;
    std::vector<State> path;
    SmartPointer<EdgePlanner> e;
    Real jumpDistance;
  };
  typedef Graph::Graph<Node,Edge> Roadmap;

  JumpPlanner(KinodynamicCSpace* space);
  ///Must be overridden -- return obstacle distance to x in C-space
  virtual Real ObstacleDistance(const State& x) = 0;
  ///Must be overridden --- return the margin by which x is inside the goal region. <= 0 indicates x is not in the goal
  virtual Real GoalSetMargin(const State& x) = 0;
  ///May be overridden --- return the maximum margin of any state inside the goal set
  virtual Real GoalSetRadius() { return Inf; }
  ///May be overridden --- sample a state inside the goal set
  virtual bool SampleGoalSet(State& x) { return false; }
  ///Must be overridden -- this is the cost function
  virtual Real EdgeCost(const State& x0,const ControlInput& u) = 0;
  ///Must be overridden --- return an upper bound on distance between the
  ///forward evolution d(f(x0,u),f(x,u)) if d(x0,x)<=R 
  virtual Real PropagateDistance(Real R,ControlInput& u)=0;
  
  void Init(const State& x0);
  int AddMilestone(const State& x);
  ///Adds an extension from node n, returns the child node index
  int SimulateAndAddEdge(int n,const ControlInput& u);
  ///Adds a feasible extension from node n.  Returns the resulting node
  ///index or -1 if not feasible
  int SimulateAndAddFeasibleEdge(int n,const ControlInput& u);
  ///Returns the resulting node index
  int AddEdge(int n,const ControlInput& u,const std::vector<State>& path,const SmartPointer<EdgePlanner>& e);
  ///Creates a jump edge
  Edge* AddJump(int n,int c);
  ///Creates a jump edge if it would improve pareto optimality, returns 0 otherwise
  Edge* TestAndAddJump(int n,int c);
  ///Calculates all costs on the roadmap
  bool CalculateCosts();
  ///Recursively propagates a cost update from node n to c
  void PropagateCosts(int n,int c);
  ///Propagates costs from node n to c, does not recurse.  Returns true if changed.
  bool UpdateCosts(int n,int c);
  ///Propagates a single frontier cost from node n to c, does not recurse.  Returns true if changed.
  bool UpdateCosts(int n,int c,const Vector& nCost);
  ///Updates the pareto optimal frontier at node n with a new cost from
  ///the indicated parent. Returns true if the frontier was changed
  bool UpdateCost(int n,const Vector& c,int parent);
  ///Propagates the accumulated cost from node n to a child through edge e
  void GetChildCost(int n,const Vector& nCost,Edge* e,Vector& childCost);
  int PickRandomNode() const;
  int FindClosestNode(const State& x) const;
  int ApproximateRandomClosestNode(const State& x,int numIters) const;
  bool GetPath(KinodynamicMilestonePath& path);
  bool GetPath(int node,int frontierIndex,std::vector<int>& path,std::vector<int>& frontierIndices);

  ///Plans for numIters iterations
  virtual void PlanMore(int numIters);
  ///Returns index of extended node or -1 if not extended.  Default uses
  ///ExtendToward(xrand) for a random state xrand
  virtual int Extend();
  ///Returns index of extended node or -1 if not extended.  Default uses
  ///space->BiasedSampleControl() to find a good control
  virtual int ExtendToward(const State& x);
  ///Rewires using jumps from the given node
  virtual bool Rewire(int node);
  ///Rewires using jumps from the given node, with radius and k-nearest
  ///neighbors specified
  virtual bool Rewire(int node,Real radius,int knn);

  KinodynamicCSpace* space;
  Roadmap roadmap;
  Real randomSeedProbability,goalBiasProbability;

  //solution information
  int goalNode,goalNodeFrontierIndex;
  Real goalCost;

  //stats
  Real extendTime,rewireTime,nearestNeighborTime,propagateTime,collisionCheckTime,simulateTime,visibilityCheckTime;
  int numBiasedSampleControls,numSimulates,numVisibilityChecks;
};

class LipschitzJumpPlanner : public JumpPlanner 
{
 public:
  LipschitzJumpPlanner(KinodynamicCSpace* space,Real lipschitzConstant=0);
  virtual Real ControlLength(const ControlInput& u) =0;
  virtual Real PropagateDistance(Real R,ControlInput& u) { return R*(1+ControlLength(u)*lengthLipschitzConstant); }
  virtual Real EdgeCost(const State& x0,const ControlInput& u) { return ControlLength(u); }

  Real lengthLipschitzConstant;
};

class LearningLipschitzJumpPlanner : public LipschitzJumpPlanner 
{
 public:
  LearningLipschitzJumpPlanner(KinodynamicCSpace* space);
  virtual void PlanMore(int numIters);
  void LearnLipschitzConstant();
};

#endif
