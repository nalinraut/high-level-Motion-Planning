#include <KrisLibrary/utils/StatCollector.h>
#include <KrisLibrary/planning/CSetHelpers.h>
#include <KrisLibrary/planning/KinodynamicMotionPlanner.h>
#include <KrisLibrary/planning/KinodynamicOptimizer.h>
#include <KrisLibrary/planning/AOMetaPlanner.h>
#include "MotionPlannerProgram.h"
#include "PendulumCSpace.h"
#include <KrisLibrary/Timer.h>
using namespace std;


class PendulumPlannerProgram : public MotionPlannerProgram
{
public:
  PendulumKinodynamicSpace pendulumCspace;
  State xstart;
  NeighborhoodSet goalSet;
  SmartPointer<KinodynamicPlannerBase> planner;
  SmartPointer<ObjectiveFunctionalBase> objectiveFn;
  SmartPointer<KinodynamicLocalOptimizer> optimizer;

  KinodynamicMilestonePath path;

  PendulumPlannerProgram()
    :pendulumCspace(),
    goalSet(pendulumCspace.GetStateSpace(),Vector(),0.0)
  {
    objectiveFn = new ControlTimeObjectiveFunction;

    goalSet.center.resize(2);
    //goal position and radius
    goalSet.center(0) = Pi*0.5;
    goalSet.center(1) = 0;
    goalSet.radius = 0.05;
    
    //max velocity / acceleratino of pendulum
    pendulumCspace.SetVelocityLimits(-10,10);
    pendulumCspace.SetAccelerationLimits(-4,4);

    //AO-RRT config
    /*
    CostSpaceRRTPlanner* crrt = new CostSpaceRRTPlanner(&pendulumCspace,objectiveFn);
    crrt->goalSeekProbability = 0.1;
    //crrt->tree.EnablePointLocation("kdtree");
    crrt->lazy = true;
    planner = crrt;
    */

    //AO-EST config
    CostSpaceESTPlanner* cest = new CostSpaceESTPlanner(&pendulumCspace,objectiveFn);
    cest->SetDensityEstimatorResolution(0.4);
    planner = cest;
    /*
    //RRT config
    RRTKinodynamicPlanner* rrt = new RRTKinodynamicPlanner(&pendulumCspace);
    planner = rrt;
    */
  }

  virtual bool Initialize() {
    if(!MotionPlannerProgram::Initialize()) return false;
    //start config
    xstart.resize(2);
    xstart(0) = -Pi/2;
    xstart(1) = 0;
    planner->Init(xstart,&goalSet);
    glClearColor(1,1,1,1);
    return true;
  }

  virtual void Handle_Display() {
    SetupDisplay(-Pi,Pi,-Pi,Pi);
    //DrawCSpace();

    glColor3f(1,0,0);
    drawCircle2D(Vector2(goalSet.center(0),goalSet.center(1)),goalSet.radius);

    //draw RRT planner tree
    DrawPendulumCallback callback;
    callback.nodeColor.set(0,1,0);
    callback.edgeColor.set(0,0.5,0);
    glPointSize(3.0);
    glLineWidth(1.0);
    RRTKinodynamicPlanner* rrt = dynamic_cast<RRTKinodynamicPlanner*>(&*planner);
    if(rrt && rrt->tree.root)
      rrt->tree.root->DFS(callback);    
    ESTKinodynamicPlanner* est = dynamic_cast<ESTKinodynamicPlanner*>(&*planner);
    if(est && est->tree.root)
      est->tree.root->DFS(callback); 

    if(!path.milestones.empty()) {
      DrawPendulumCallback cb;
      glColor3f(0,0,0);
      glPointSize(5.0);
      glLineWidth(2.0);
      cb.DrawPath(path);
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
      planner->Init(xstart,&goalSet);
      Refresh();
    }
    else if(c==' ') {
      planner->Plan(1);
      if(planner->Done()) {
        if(path.Empty())
          printf("Found a path!\n");
        planner->GetPath(path);
      }
    }
    else if(c=='p') {
      planner->Plan(1000);
      if(planner->Done()) {
        if(path.Empty())
          printf("Found a path!\n");
        planner->GetPath(path);
      }
      PropertyMap props;
      planner->GetStats(props);
      cout<<"Stats: ";
      props.Print(cout);
      Refresh();
    }
    else if(c=='o') {
      if(!path.Empty()) {
        if(!optimizer) {
          optimizer = new KinodynamicLocalOptimizer(&pendulumCspace,objectiveFn);
          optimizer->Init(path,&goalSet);
        }
        optimizer->Plan(10);
        optimizer->GetPath(path);
        Refresh();
      }
    }
  }
};


