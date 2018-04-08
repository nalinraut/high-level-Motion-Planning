#include <KrisLibrary/utils/StatCollector.h>
#include "JumpPlanner.h"
#include "MotionPlannerProgram.h"
#include "CarCSpace.h"
#include <KrisLibrary/Timer.h>
using namespace std;

class CarJumpPlanner : public LipschitzJumpPlanner
{
 public:
  CarJumpPlanner(CSpace* obstacleSpace,const Config& start,const Config& goal)
    :LipschitzJumpPlanner(&carCspace,0.3),carCspace(obstacleSpace)
  {
    Assert(goal.n==2 || goal.n==3);
    goalSet.baseSpace = obstacleSpace;
    goalSet.center.resize(2);
    //goal position and radius
    goalSet.center(0) = goal(0);
    goalSet.center(1) = goal(1);
    goalSet.radius = 0.1;
    carGoalSet.baseSpace = &carCspace;
    carGoalSet.center.resize(3);
    carGoalSet.center(0) = goal(0);
    carGoalSet.center(1) = goal(1);
    carGoalSet.center(2) = (goal.n == 3 ? goal(2) : 0);
    carGoalSet.radius = goalSet.radius;

    //max forward/backward movement of car
    //carCspace.dmin = -0.1;
    carCspace.dmin = 0;
    carCspace.dmax = 0.2;
    //max left/right turn radius of car
    carCspace.turnmin = -20;
    carCspace.turnmax = 20;
    carCspace.angleWeight = 0.03;

    Init(start);
  }
  virtual Real ControlLength(const ControlInput& u) { return Abs(u[1]); }
  virtual Real ObstacleDistance(const State& x) {
    return carCspace.ObstacleDistance(x);
  }
  virtual Real GoalSetMargin(const State& x) {
    return carGoalSet.radius - carCspace.Distance(carGoalSet.center,x);
  }
  virtual Real GoalSetRadius() {
    return carGoalSet.radius;
  }
  virtual bool SampleGoalSet(State& x) { 
    carGoalSet.Sample(x);
    return true;
  }

  CSpace* obstacleSpace;
  CarCSpaceAdaptor carCspace;
  NeighborhoodCSpace goalSet;
  NeighborhoodCSpace carGoalSet;
};

struct JumpPlannerProgram : public MotionPlannerProgram
{
  SmartPointer<CarJumpPlanner> planner;

  JumpPlannerProgram()
  {
  }

  bool Initialize()
  {
    Reset();
  }

  void Reset()
  {
    //start config
    Config initialState(3);
    initialState(0) = 0.1;
    initialState(1) = 0.1;
    initialState(2) = 0;
    Config goalState(3);
    goalState(0) = 0.9;
    goalState(1) = 0.9;
    goalState(2) = 0;
    planner = new CarJumpPlanner(cspace,initialState,goalState);
  }

  virtual void Handle_Display() {
    SetupDisplay();
    DrawCSpace();

    glColor3f(1,0,0);
    drawCircle2D(Vector2(planner->goalSet.center(0),planner->goalSet.center(1)),planner->goalSet.radius);
    

    //draw planner graph
    DrawCarCallback callback;
    GLColor nodeColor(0,1,0);
    GLColor edgeColor(0,0.5,0);
    GLColor jumpEdgeColor(0.5,0,0.5);
    GLColor jumpConnectionColor(1,0.5,1);
    glPointSize(5.0);
    glLineWidth(1.0);
    glBegin(GL_LINES);
    for(size_t i=0;i<planner->roadmap.nodes.size();i++) {
      Graph::EdgeIterator<JumpPlanner::Edge> ei;
      for(planner->roadmap.Begin(i,ei);!ei.end();ei++) {
	int t = ei.target();
	if(ei->isJump) {
	  glColor3f(1,1,1);
	  glVertex2d(planner->roadmap.nodes[i].x(0),planner->roadmap.nodes[i].x(1));
	  jumpEdgeColor.setCurrentGL();
	  glVertex2d(ei->path.back()(0),ei->path.back()(1));
	  jumpConnectionColor.setCurrentGL();
	  glVertex2d(ei->path.back()(0),ei->path.back()(1));
	  glVertex2d(planner->roadmap.nodes[t].x(0),planner->roadmap.nodes[t].x(1));
	}
	else {
	  edgeColor.setCurrentGL();
	  glVertex2d(planner->roadmap.nodes[t].x(0),planner->roadmap.nodes[t].x(1));
	  glColor3f(1,1,1);
	  glVertex2d(planner->roadmap.nodes[i].x(0),planner->roadmap.nodes[i].x(1));
	}
      }
    }
    glEnd();
    nodeColor.setCurrentGL();
    glBegin(GL_POINTS);
    for(size_t i=0;i<planner->roadmap.nodes.size();i++) 
      glVertex2d(planner->roadmap.nodes[i].x(0),planner->roadmap.nodes[i].x(1));
    glEnd();

    KinodynamicMilestonePath path;
    if(planner->GetPath(path)) {
      glColor3f(0,0,1);
      glLineWidth(2.0);
      glBegin(GL_LINE_STRIP);
      for(size_t i=0;i<path.paths.size();i++) {
	for(size_t j=0;j<path.paths[i].size();j++) {
	  glVertex2d(path.paths[i][j](0),path.paths[i][j](1));
	}
      }
      glEnd();
    }
         
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
    if(c=='r') {
      Reset();
      Refresh();
    }
    else if(c=='p') {
      planner->PlanMore(1000);
      printf("%d nodes, %d edges\n",planner->roadmap.nodes.size(),planner->roadmap.NumEdges());
      size_t np = 0;
      for(size_t i=0;i<planner->roadmap.nodes.size();i++)
	np += planner->roadmap.nodes[i].paretoOptimalCosts.size();
      printf("%g average paths in pareto frontier\n",Real(np)/Real(planner->roadmap.nodes.size()));
      printf("Timing: %g extend, %g rewire\n",planner->extendTime,planner->rewireTime);
      printf("  %g NN, %g propagate, %g collision check, %g simulate, %g visibility check\n",planner->nearestNeighborTime,planner->propagateTime,planner->collisionCheckTime,planner->simulateTime,planner->visibilityCheckTime);
      printf("  %d biased sample controls, %d simulates, %d visibility checks\n",planner->numBiasedSampleControls,planner->numSimulates,planner->numVisibilityChecks);
      Refresh();
    }
    else if(c==' ') {
      planner->PlanMore(1);
      Refresh();
    }
  }
};
