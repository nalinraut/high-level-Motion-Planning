#include <KrisLibrary/planning/MotionPlanner.h>
#include <KrisLibrary/planning/OptimalMotionPlanner.h>
#include <KrisLibrary/planning/PointLocation.h>
#include "MotionPlannerProgram.h"
#include <fstream>
#include <sstream>
using namespace std;


struct OptimalPlannerProgram : public MotionPlannerProgram
{
  bool hasStart,hasGoal,hasPath;
  Config start,goal;
  MilestonePath path;
  PRMStarPlanner planner;
  Real cumulativeTime;

  OptimalPlannerProgram()
    :planner(cspace)
  {
    //approximate shortest paths
    planner.suboptimalityFactor = 0.0;
    planner.pointLocator = new KDTreePointLocation(planner.roadmap.nodes);
    planner.connectByRadius = true;
    //planner.lazyCheckThreshold = 0.05;
    hasStart=hasGoal=hasPath=false;
  }

  virtual bool Initialize() {
    if(!MotionPlannerProgram::Initialize()) return false;
    if(cspace.planningQueries.empty()) {
      start.resize(2);
      start[0] = 0.3;
      start[1] = 0.5;
      goal.resize(2);
      goal[0] = 0.7;
      goal[1] = 0.5;
    }
    else {
      start = cspace.planningQueries[0].first;
      goal = cspace.planningQueries[0].second;
    }
    hasStart = hasGoal = true;
    InitPlanner();
    return true;
  }

  void InitPlanner()
  {
    cumulativeTime = 0;
    planner.space = cspace;
    planner.Init(start,goal);
    hasPath = false;
  }

  void PlanStep()
  {
    planner.PlanMore();
  }

  void PlanMore(int num=1000,FILE* csvout=NULL)
  {
    Assert(hasStart&&hasGoal);
    Timer timer;
    for(int i=0;i<num;i++) 
      PlanStep();

    //extract minimum cost path
    hasPath = planner.GetPath(path);
    if(hasPath) {
      /*
      //TODO: fix this problem
      if(planner.suboptimalityFactor == 0)
	Assert(path.IsFeasible());
      */
    }
    cumulativeTime += timer.ElapsedTime();
    if(csvout == NULL) {
      printf("Plan step %d, time %g, resulting cost %g\n",planner.numPlanSteps,cumulativeTime,planner.spp.d[planner.goal]);
      printf("Time config check %g, knn %g, connect %g, lazy %g (check %g), %d edge checks, %d edge prechecks\n",planner.tCheck,planner.tKnn,planner.tConnect,planner.tLazy,planner.tLazyCheck,planner.numEdgeChecks,planner.numEdgePrechecks);
    }
    else {
      fprintf(csvout,"%d,%g,%g,%g,%g,%g,%g,%d\n",planner.numPlanSteps,cumulativeTime,planner.spp.d[planner.goal],planner.tCheck,planner.tKnn,planner.tConnect,planner.tLazy,planner.numEdgeChecks);
    }
  }

  virtual void Handle_Display() {
    SetupDisplay();
    DrawCSpace();

    DrawGraphCallback callback(planner.roadmap,&cspace);
    callback.nodeColor.set(1,0,1);
    callback.edgeColor.set(0.5,0,0.5,0.5);
    callback.doLazy = true;
    callback.lazyEdgeColor.set(0.5,0.5,0.5,0.25);
    glPointSize(3.0);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    callback.Draw();
    glDisable(GL_BLEND);
    if(hasPath) {
      glColor3f(0,0,1);
      glLineWidth(4.0);
      glBegin(GL_LINE_STRIP);
      for(size_t i=0;i<path.NumMilestones();i++)
	glVertex2v(path.GetMilestone(i));
      glEnd();
      glLineWidth(1.0);
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
      cspace->Sample(q);
      if(cspace->IsFeasible(q)) return true;
    }
    return false;
  }

  bool SampleProblem()
  {
    if(!SampleConfig(start)) return false;
    if(!SampleConfig(goal)) return false;

    hasStart = true;
    hasGoal = true;
    hasPath = false;
    InitPlanner();
    return true;
  }

  void BatchTest(const char* name,const char* filename="",int numTrials=10,int maxIters=100000,int printIncrement=1000,Real tmax = 10)
  {
    printf("Testing %s, saving to %s:\n",name,filename);
    FILE* f = NULL;
    if(filename) {
      f = fopen(filename,"w");
      fprintf(f,"trial,plan iters,plan time,best cost,time configuration test,time KNN,time connect,time lazy,edge checks\n");
    }
    for(int trials=0;trials<numTrials;trials++) {
      InitPlanner();
      Timer timer;
      for(int i=0;i<maxIters/printIncrement;i++) {
	if(f)
	  fprintf(f,"%d,",trials);
	PlanMore(1000,f);
	if(timer.ElapsedTime() > tmax) break;
      }
    }
    fclose(f);
  }

  virtual void Handle_Keypress(unsigned char key,int x,int y)
  {
    if(key == '1') {
      PlanMore(1);
    }
    else if(key == ' ') {
      PlanMore();
    }
    else if(key == 's') {
      if(!SampleProblem()) return;
    }
    else if(key == 'o') {
      planner.suboptimalityFactor = 0.2;
      printf("Setting suboptimal planner, parameter %g\n",planner.suboptimalityFactor);
    }
    else if(key == 'l') {
      planner.lazy = !planner.lazy;
      if(planner.lazy) printf("Toggled to lazy\n");
      else printf("Toggled to non-lazy\n");
    }
    else if(key == 'p') {
      planner.rrg = !planner.rrg;
      if(planner.rrg) printf("Toggled to RRG* expansion strategy\n");
      else printf("Toggled to PRM* expansion strategy\n");
    }
    else if(key == 'r') { 
      hasStart=false;
      hasGoal=false;
      hasPath=false;
    }
    else if(key == 't') {
      int n=10;
      //standard PRM*
      planner.lazy = false;
      planner.rrg = false;
      planner.suboptimalityFactor = 0.0;
      BatchTest("PRM*","results/prm.csv");

      //standard RRT*
      planner.lazy = false;
      planner.rrg = true;
      planner.suboptimalityFactor = 0.0;
      BatchTest("RRT*","results/rrt.csv");

      planner.lazy = false;
      planner.rrg = true;
      planner.suboptimalityFactor = 0.1;
      BatchTest("epsilon-RRT*, epsilon=0.1","results/rrt_subopt0.1.csv");
      planner.suboptimalityFactor = 0.2;
      BatchTest("epsilon-RRT*, epsilon=0.2","results/rrt_subopt0.2.csv");

      planner.lazy = true;
      planner.rrg = false;
      planner.suboptimalityFactor = 0.0;
      BatchTest("Lazy PRM*","results/lazyprm.csv");

      planner.lazy = true;
      planner.rrg = true;
      planner.suboptimalityFactor = 0.0;
      BatchTest("Lazy RRG*","results/lazyrrg.csv");

      /*
      planner.lazy = true;
      planner.rrg = false;
      planner.suboptimalityFactor = 0.1;
      BatchTest("epsilon-Lazy PRM*, epsilon=0.1","results/lazyprm_subopt0.1.csv");
      planner.suboptimalityFactor = 0.2;
      BatchTest("epsilon-Lazy PRM*, epsilon=0.2","results/lazyprm_subopt0.2.csv");
      */

      planner.lazy = true;
      planner.rrg = true;
      planner.suboptimalityFactor = 0.1;
      BatchTest("epsilon-Lazy RRG*, epsilon=0.2","results/lazyrrg_subopt0.1.csv");
      planner.suboptimalityFactor = 0.2;
      BatchTest("epsilon-Lazy RRG*, epsilon=0.2","results/lazyrrg_subopt0.2.csv");
    }
    Refresh();
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
	InitPlanner();
      }
      else {
	PlanMore();
      }
      Refresh();
    }
  }
};
