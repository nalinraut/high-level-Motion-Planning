#include <planning/MultiModalPlanner.h>
#include <planning/MotionPlanner.h>
#include "MotionPlannerProgram.h"
#include <fstream>
#include <sstream>
using namespace std;

struct MultiModalPlannerProgram : public MotionPlannerProgram
{
  bool hasStart,hasGoal,hasPath;
  Config start,goal;
  MilestonePath path;
  MultiModalPRM mmp;
  ExplicitMMCSpace mmpSpace;
  SBLPlanner sbl;

  MultiModalPlannerProgram()
    :mmp(&mmpSpace),sbl(&cspace)
  {
    hasStart=hasGoal=hasPath=false;

    mmp.numExpandModeSamples = 20;
    mmp.numExpandTransSamples = 1;
    mmpSpace.modeGraph.Resize(3);
    Geometric2DCSpace* r1 = new Geometric2DCSpace(cspace);
    Geometric2DCSpace* r2 = new Geometric2DCSpace(cspace);
    Geometric2DCSpace* r3 = new Geometric2DCSpace(cspace);
    Geometric2DCSpace* t1 = new Geometric2DCSpace(cspace);
    Geometric2DCSpace* t2 = new Geometric2DCSpace(cspace);
    r1->domain.bmax.x = 0.5;
    r2->domain.bmin.x = 0.5;
    Real xshed = 0.1;
    Real yshed = 0.1;
    r3->domain.bmin.x = 0.5-0.5*narrowPassageLength-xshed;
    r3->domain.bmax.x = 0.5+0.5*narrowPassageLength+xshed;
    r3->domain.bmin.y = 0.5-0.5*narrowPassageWidth-yshed;
    r3->domain.bmax.y = 0.5+0.5*narrowPassageWidth+yshed;
    t1->domain = r1->domain;
    t1->domain.setIntersection(r3->domain);
    t2->domain = r2->domain;
    t2->domain.setIntersection(r3->domain);
    mmpSpace.modeGraph.nodes[0] = r1;
    mmpSpace.modeGraph.nodes[1] = r2;
    mmpSpace.modeGraph.nodes[2] = r3;
    mmpSpace.modeGraph.AddEdge(0,2,t1);
    mmpSpace.modeGraph.AddEdge(1,2,t2);
  }

  virtual void Handle_Display() {
    SetupDisplay();
    DrawCSpace();

    DrawSBLTreeCallback callback;
    //draw SBL tree
    callback.nodeColor.set(1,0,1);
    callback.edgeColor.set(0.5,0,0.5);
    glPointSize(2.0);
    glLineWidth(1.0);
    if(sbl.tStart)
      sbl.tStart->root->DFS(callback);
    if(sbl.tGoal)
      sbl.tGoal->root->DFS(callback);
    //draw transitions
    glColor3f(0,0.5,1);
    glPointSize(5.0);
    glBegin(GL_POINTS);
    for(size_t i=0;i<mmp.planningGraph.nodes.size();i++) {
      MultiModalPRM::PlanningGraph::Iterator e;
      for(mmp.planningGraph.Begin(i,e);!e.end();++e) {
	for(size_t k=0;k<e->transitions.size();k++)
	  glVertex2v(e->transitions[k]);
      }
    }
    glEnd();
    //draw MMP trees
    callback.nodeColor.set(0,0,0);
    callback.edgeColor.set(0,0,0);
    glPointSize(3.0);
    glLineWidth(2.0);
    for(size_t i=0;i<mmp.planningGraph.nodes.size();i++) {
      MotionPlannerInterface* planner=mmp.planningGraph.nodes[i].planner;
      RoadmapPlanner prm(mmp.space->GetModeCSpace(i));
      planner->GetRoadmap(prm);
      //DrawSBLPRTCallback prtCallback(prm);
      DrawGraphCallback callback(prm.roadmap);
      glPointSize(3.0);
      callback.nodeColor.set(0,1,0.5);
      callback.edgeColor.set(0,1,1);
      prm.roadmap.DFS(callback);
      /*
      prtCallback.nodeColor.set(0,1,0.5);
      prtCallback.edgeColor.set(0,1,1);
      mmp.planningGraph.nodes[i].prm->roadmap.DFS(prtCallback);
      glPointSize(2.0);
      glLineWidth(1.0);
      for(size_t j=0;j<mmp.planningGraph.nodes[i].prm->roadmap.nodes.size();j++)
	mmp.planningGraph.nodes[i].prm->roadmap.nodes[i]->root->DFS(callback);
      */
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

  virtual void Handle_Keypress(unsigned char key,int x,int y)
  {
    if(key == 's') {
      if(!hasStart || !hasGoal) return;
      sbl.Cleanup();
      sbl.Init(start,goal);
      for(int iters=0;iters<10000;iters++) {
	if(sbl.IsDone()) {
	  printf("SBL finished on iteration %d\n",iters);
	  sbl.CreatePath(path);
	  hasPath = true;
	  break;
	}
	sbl.Extend();
      }
      Refresh();
    }
    else if(key == 'm') {
      if(!hasStart || !hasGoal) return;
      mmp.InitializeExplicit(&mmpSpace);
      if(start[0] < 0.5) mmp.SetStart(start,0);
      else mmp.SetStart(start,1);
      if(goal[0] < 0.5) mmp.SetGoal(goal,0);
      else mmp.SetGoal(goal,1);
      for(int iters=0;iters<100;iters++) {
	mmp.ExpandAll();
	if(mmp.IsStartAndGoalConnected()) {
	  printf("MMP finished on iteration %d\n",iters);
	  break;
	}
      }
    }
    Refresh();
  }

  virtual void Handle_Click(int button,int state,int x,int y) {
    if(state == GLUT_DOWN) {
      Config q;
      ClickToConfig(x,y,q);
      if(!hasStart || hasGoal) {
	start=q;
	hasStart=true;
	hasGoal = false;
      }
      else if(!hasGoal) {
	goal=q;
	hasGoal=true;
	hasPath = false;
      }
      Refresh();
    }
  }
};
