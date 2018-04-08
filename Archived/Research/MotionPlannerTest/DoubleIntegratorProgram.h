#include <KrisLibrary/utils/StatCollector.h>
#include <KrisLibrary/planning/CSetHelpers.h>
#include <KrisLibrary/planning/KinodynamicMotionPlanner.h>
#include <KrisLibrary/planning/KinodynamicOptimizer.h>
#include <KrisLibrary/planning/DoubleIntegrator.h>
#include <KrisLibrary/planning/AOMetaPlanner.h>
#include "MotionPlannerProgram.h"
#include <KrisLibrary/Timer.h>
using namespace std;

typedef DrawCarCallback DrawDoubleIntegratorCallback;


Config gDIHeuristicGoal;
Real gDIHeuristicRadius;

Real DIHeuristic(const Config& x)
{
  Real tmax = 0;
  tmax = Max(tmax,Abs(x[0]-gDIHeuristicGoal[0])/2 - gDIHeuristicRadius/2);
  tmax = Max(tmax,Abs(x[1]-gDIHeuristicGoal[1])/2 - gDIHeuristicRadius/2);
  tmax = Max(tmax,Abs(x[2])/4 - gDIHeuristicRadius/4);
  tmax = Max(tmax,Abs(x[3])/4 - gDIHeuristicRadius/4);
  return tmax;
}

class DoubleIntegratorPlannerProgram : public MotionPlannerProgram
{
public:
  DoubleIntegratorKinodynamicSpace diCspace;
  State xstart;
  NeighborhoodSet goalSet;
  SmartPointer<KinodynamicPlannerBase> planner;
  SmartPointer<ObjectiveFunctionalBase> objectiveFn;
  SmartPointer<KinodynamicLocalOptimizer> optimizer;

  KinodynamicMilestonePath path;

  DoubleIntegratorPlannerProgram()
    :diCspace((CSpace*)cspace,new BoxCSpace(Vector(2,-2.0),Vector(2,2.0)),new BoxSet(Vector(2,-4.0),Vector(2,4.0)),0.25),
    goalSet(diCspace.GetStateSpace(),Vector(),0.0)
  {
    objectiveFn = new ControlTimeObjectiveFunction;

    goalSet.center.resize(4);
    //goal position and radius
    goalSet.center(0) = 0.9;
    goalSet.center(1) = 0.9;
    goalSet.center(2) = 0;
    goalSet.center(3) = 0;
    goalSet.radius = 0.1;

    gDIHeuristicGoal = goalSet.center;
    gDIHeuristicRadius = goalSet.radius;

    diCspace.SetVisibilityEpsilon(0.001);
  }

  virtual bool Initialize() {
    if(!MotionPlannerProgram::Initialize()) return false;

    dynamic_cast<MultiCSpace*>(&*diCspace.GetStateSpace())->components[0] = (CSpace*)cspace;

    //AO-RRT config
    CostSpaceRRTPlanner* crrt = new CostSpaceRRTPlanner(&diCspace,objectiveFn);
    crrt->goalSeekProbability = 0.1;
    crrt->tree.EnablePointLocation("kdtree");
    crrt->SetHeuristic(DIHeuristic);
    crrt->delta = 0.5;
    crrt->lazy = false;
    planner = crrt;

    //AO-EST config
    /*
    CostSpaceESTPlanner* cest = new CostSpaceESTPlanner(&diCspace,objectiveFn);
    cest->SetDensityEstimatorResolution(0.1);
    cest->SetHeuristic(DIHeuristic);
    planner = cest;
    */
    
    /*
    //RRT config
    RRTKinodynamicPlanner* rrt = new RRTKinodynamicPlanner(&diCspace);
    planner = rrt;
    */

    //start config
    xstart.resize(4);
    xstart(0) = 0.1;
    xstart(1) = 0.1;
    xstart(2) = 0;
    xstart(3) = 0;
    planner->Init(xstart,&goalSet);


    glClearColor(1,1,1,1);
    return true;
  }

  virtual void Handle_Display() {
    SetupDisplay();
    DrawCSpace();

    glColor3f(1,0,0);
    drawCircle2D(Vector2(goalSet.center(0),goalSet.center(1)),goalSet.radius);

    //draw RRT planner tree
    DrawDoubleIntegratorCallback callback;
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
      glColor3f(0,0,0);
      glPointSize(5.0);
      glLineWidth(2.0);
      callback.DrawPath(path);
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
      Refresh();
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
      cout<<endl;
      /*
      //calculate closest
      KinodynamicTree* tree = &dynamic_cast<CostSpaceESTPlanner*>(&*planner)->tree;
      Graph::TopologicalSortCallback<KinodynamicTree::Node*> cb;
      tree->root->DFS(cb);
      Real dmin = Inf;
      Vector x(4);
      for(list<KinodynamicTree::Node*>::iterator i=cb.list.begin();i!=cb.list.end();i++) {
        (*i)->getSubVectorCopy(0,x);
        Real d = diCspace.GetStateSpace()->Distance(x,goalSet.center);
        dmin = Min(dmin,d);
      }
      cout<<"Minimum distance to goal "<<dmin<<endl;
      */
      Refresh();
    }
    else if(c=='o') {
      if(!path.Empty()) {
        if(!optimizer) {
          optimizer = new KinodynamicLocalOptimizer(&diCspace,objectiveFn);
          optimizer->Init(path,&goalSet);
        }
        optimizer->Plan(10);
        optimizer->GetPath(path);
        Refresh();
      }
    }
  }
};


