#include <KrisLibrary/utils/StatCollector.h>
#include <KrisLibrary/planning/CSetHelpers.h>
#include <KrisLibrary/planning/KinodynamicMotionPlanner.h>
#include <KrisLibrary/planning/KinodynamicOptimizer.h>
#include <KrisLibrary/planning/AOMetaPlanner.h>
#include "MotionPlannerProgram.h"
#include "CarCSpace.h"
#include <KrisLibrary/Timer.h>
using namespace std;

Config gCarHeuristicGoal;

Real CarHeuristic(const Config& x)
{
  Real aerr = Sqr(0.1)*Sqr(AngleDiff(x[2],gCarHeuristicGoal[2]));
  return Max(0.0,Sqrt(Sqr(x[0]-gCarHeuristicGoal[0]) + Sqr(x[1]-gCarHeuristicGoal[1]) + aerr) - 0.05);
}

class CarPlannerProgram : public MotionPlannerProgram
{
public:
  CarKinodynamicSpace carCspace;
  State xstart;
  NeighborhoodSet goalSet;
  NeighborhoodSet carGoalSet;
  SmartPointer<KinodynamicPlannerBase> planner;
  SmartPointer<ObjectiveFunctionalBase> objectiveFn;
  SmartPointer<KinodynamicLocalOptimizer> optimizer;

  KinodynamicMilestonePath path;

  CarPlannerProgram()
    :carCspace(),
    goalSet((CSpace*)cspace,Vector(),0.0),
    carGoalSet(carCspace.GetStateSpace(),Vector(),0.0)
  {
    objectiveFn = new CarLengthObjectiveFunction;

    goalSet.center.resize(2);
    //goal position and radius
    goalSet.center(0) = 0.9;
    goalSet.center(1) = 0.1;
    goalSet.radius = 0.05;
    carGoalSet.center.resize(3);
    carGoalSet.center(0) = goalSet.center(0);
    carGoalSet.center(1) = goalSet.center(1);
    carGoalSet.center(2) = Pi/2;
    carGoalSet.radius = goalSet.radius;
    gCarHeuristicGoal = carGoalSet.center;

    //max forward/backward movement of car
    carCspace.SetSteeringLimits(-20,20,-0.1,0.1);
    carCspace.SetAngleWeight(0.1);
  }

  virtual bool Initialize() {
    if(!MotionPlannerProgram::Initialize()) return false;
    carCspace.SetObstacles(cspace);

    //AO-RRT config
    CostSpaceRRTPlanner* crrt = new CostSpaceRRTPlanner(&carCspace,objectiveFn);
    crrt->goalSeekProbability = 0.1;
    crrt->tree.EnablePointLocation("kdtree");
    crrt->SetHeuristic(CarHeuristic);
    crrt->lazy = false;
    planner = crrt;
    
    /*
    //AO-EST config
    CostSpaceESTPlanner* cest = new CostSpaceESTPlanner(&carCspace,objectiveFn);
    cest->SetHeuristic(CarHeuristic);
    planner = cest;
    */
    /*
    //RRT config
    RRTKinodynamicPlanner* rrt = new RRTKinodynamicPlanner(&carCspace);
    rrt->goalSeekProbability = 0.1;
    rrt->tree.EnablePointLocation("kdtree");
    planner = rrt;
    */

    //start config
    xstart.resize(3);
    xstart(0) = 0.1;
    xstart(1) = 0.1;
    xstart(2) = 0;
    planner->Init(xstart,&carGoalSet);
    return true;
  }

  virtual void Handle_Display() {
    SetupDisplay();
    DrawCSpace();

    glColor3f(1,0,0);
    drawCircle2D(Vector2(goalSet.center(0),goalSet.center(1)),goalSet.radius);

    //draw RRT planner tree
    DrawCarCallback callback;
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
      planner->Init(xstart,&carGoalSet);
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
      Refresh();
    }
    else if(c=='o') {
      if(!path.Empty()) {
        if(!optimizer) {
          optimizer = new KinodynamicLocalOptimizer(&carCspace,objectiveFn);
          optimizer->Init(path,&carGoalSet);
        }
        optimizer->Plan(10);
        optimizer->GetPath(path);
        Refresh();
      }
    }
  }
};
