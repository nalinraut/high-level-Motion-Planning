#include <KrisLibrary/graph/IO.h>
#include "MotionPlannerProgram.h"
#include <KrisLibrary/planning/CSetHelpers.h>
#include <KrisLibrary/planning/CSpaceHelpers.h>
#include <KrisLibrary/planning/MCRPlanner.h>
#include <KrisLibrary/planning/MCRPlannerGoalSet.h>
#include <fstream>
#include <sstream>
#include <map>
using namespace std;

bool blockBBs = true;


Subset Violations(CSpace* space,const Config& q);
Subset Violations(CSpace* space,const Config& a,const Config& b);

struct MCRPlannerProgram : public MotionPlannerProgram
{
  bool hasStart,hasGoal,hasPath;
  Config start,goal;
  FiniteSet goalSet;
  //MCRPlanner planner;
  MCRPlannerGoalSet planner;
  Real maxExplanationCost;
  vector<int> path;
  Subset pathCover;
  vector<int> milestoneLabels;
  bool drawOutlines,drawRoadmap;

  MCRPlannerProgram()
    :planner(cspace.pointSpace),goalSet(goal)
  {
    hasStart=hasGoal=hasPath=false;
    maxExplanationCost = 0;

    drawOutlines = false;
    drawRoadmap = true;
  }

  virtual bool Initialize() {
    planner.space = cspace.pointSpace;
    if(planner.space == NULL) {
      planner.space = cspace.translatingSpace;
      if(planner.space == NULL) {
	planner.space = cspace.rigidSpace;
	if(planner.space == NULL) 
	  return false;
      }
    }

    if(blockBBs) {
      planner.obstacleWeights.resize(planner.space->NumConstraints(),1.0);
      for(int i=0;i<planner.space->NumConstraints();i++) {
	string s = planner.space->ConstraintName(i);
	if(s.substr(0,4) == "aabb") {
	  printf("Blocking %d\n",i);
	  planner.obstacleWeights[i] = Inf;
	}
      }
    }

    if(!MotionPlannerProgram::Initialize()) return false;
    return true;
  }

  virtual void Handle_Display() {
    SetupDisplay();

    //draw CSpace
    glDisable(GL_LIGHTING);
    glLineWidth(1.0);
    //blank out background (light yellow)
    //glColor3f(1,1,0.5);
    //blank out background (white)
    glColor3f(1,1,1);
    glBegin(GL_QUADS);
    glVertex2f(0,0);
    glVertex2f(1,0);
    glVertex2f(1,1);
    glVertex2f(0,1);
    glEnd();
    vector<vector<Vector2> > polys;
    if(cspace.pointSpace)
      cspace.pointSpace->ToPolygons(polys);
    else if(cspace.translatingSpace)
      cspace.translatingSpace->obstacles.ToPolygons(polys);
    else if(cspace.rigidSpace)
      cspace.rigidSpace->obstacles.ToPolygons(polys);
    if(!drawOutlines) {
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    }
    GLColor ocolors[8] = {GLColor(0,0,0),GLColor(1,0,0),GLColor(0,1,0),GLColor(0,0,1),GLColor(0.5,0.5,0.5),GLColor(0,0.5,0.5),GLColor(0.5,0,0.5),GLColor(0.5,0.5,0) };
    for(size_t i=0;i<polys.size();i++) {
      if(drawOutlines) {
	ocolors[i%8].setCurrentGL();
	glBegin(GL_LINE_LOOP);
	for(size_t j=0;j<polys[i].size();j++)
	  glVertex2v(polys[i][j]);
	glEnd();
      }
      else {
	int ofs = (cspace.pointSpace? 4:1);
	if(find(pathCover.items.begin(),pathCover.items.end(),i+ofs) == pathCover.items.end()) {
	  if(planner.obstacleWeights.empty() || !IsInf(planner.obstacleWeights[i+ofs]))
	    glColor4f(0,0,0,0.6);
	  else
	    glColor4f(0,0,0,1);
	}
	else 
	  glColor4f(1,0.5,0,0.6);
	glBegin(GL_TRIANGLE_FAN);
	for(size_t j=0;j<polys[i].size();j++) 
	  glVertex2v(polys[i][j]);
	glEnd();
      }
    }
    if(!drawOutlines) {
      glDisable(GL_BLEND);
    }
    //DrawCSpace();

    if(drawRoadmap) {
      glDisable(GL_DEPTH_TEST);
      glDisable(GL_LIGHTING);
      //DrawMCRCallback callback(planner,&cspace);
      DrawMCRGoalSetCallback callback(planner,&cspace);
      callback.nodeColor.resize(5);
      callback.nodeColor[0].set(1,0,1);
      callback.nodeColor[1].set(0,0,1);
      callback.nodeColor[2].set(0,1,1);
      callback.nodeColor[3].set(0,1,0);
      callback.nodeColor[4].set(0.5,0.5,0.5);
      callback.edgeColor.set(0,0,0,0.25);
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
      glPointSize(5.0);
      glLineWidth(1.0);
      callback.Draw();
      const static GLColor colors[8] = {GLColor(1,1,0),GLColor(0,1,1),GLColor(1,0,1),GLColor(1,0.5,0),GLColor(0,1,0.5),GLColor(0.5,0,1),GLColor(0.5,1,0),GLColor(0,0.5,1)};
      for(size_t i=0;i<milestoneLabels.size();i++) {
	if(i >= planner.roadmap.nodes.size()) break;
	if(milestoneLabels[i] > 0) {
	  colors[milestoneLabels[i]%8].setCurrentGL();
	  glPointSize(5.0);
	  cspace.DrawRoadmapNode(planner.roadmap.nodes[i].q,2);
	}
      }
    }

    //draw the path
    if(hasPath) {
      glColor3f(0,0,1);
      glLineWidth(4.0);
      for(size_t i=0;i+1<path.size();i++)
	cspace.DrawRoadmapEdge(planner.roadmap.nodes[path[i]].q,planner.roadmap.nodes[path[i+1]].q);
    }

    //draw the start and goal
    if(hasStart) {
      glPointSize(7.0);
      glColor3f(0,1,0);
      cspace.DrawConfiguration(start,1);
    }
    if(hasGoal) {
      glPointSize(7.0);
      glColor3f(1,0,0);
      cspace.DrawConfiguration(goal,2);
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

    //planner.Init(start,goal);
    goalSet.items[0] = goal;
    planner.Init(start,&goalSet);
    for(int i=0;i<1000;i++) {
      vector<int> newnodes;
      planner.Expand2(10,newnodes);
    }
    //planner.GreedyPath(0,1,path,pathCover);
    planner.GreedyPath(path,pathCover);
    return false;
  }

  virtual void Handle_Keypress(unsigned char key,int x,int y)
  {
    if(key == 's') {
      if(!SampleProblem()) return;
      Refresh();
    }
    else if(key == 'o') {
      drawOutlines = !drawOutlines;
      Refresh();
    }
    else if(key == 'p') {
      //print planner stats
      printf("Expands: %d, refines %d (%d successful), explores %d\n",planner.numExpands,planner.numRefinementAttempts,planner.numRefinementSuccesses,planner.numExplorationAttempts);
      printf("Config checks %d, edge checks %d\n",planner.numConfigChecks,planner.numEdgeChecks);
      printf("Path cover updates %d, iterations %d\n",planner.numUpdatePaths,planner.numUpdatePathsIterations);
    }
    else if(key == 't') {
      Real maxExplanationCost=0;
      Subset bestPathCover;
      Real bestCost;
      double pathLength=-1;
      Timer timer;
      double expandTime=0,pathTime=0;
      //compute initial path: start->goal
      path.resize(0);
      planner.GreedyPath(path,bestPathCover);
      bestCost = planner.Cost(bestPathCover);

      Subset lowerCover = Violations(planner.space,start);
      lowerCover=lowerCover+Violations(planner.space,goal);
      Real lowerCost = planner.Cost(lowerCover);
      /*
      cout<<"Straight line cover: ";
      for(set<int>::iterator i=bestPathCover.items.begin();i!=bestPathCover.items.end();i++)
	cout<<*i<<" ";
      cout<<endl;
      */
      pathLength = planner.space->Distance(planner.roadmap.nodes[0].q,planner.roadmap.nodes[1].q);
      //start planning
      printf("Planning... (saving to plan_results.csv)\n");
      ofstream out("plan_results.csv",ofstream::out);
      out<<"Iter,planTime,expl_limit,path_cover_size,path_length"<<endl;
      int maxIters=10000;
      for(int iters=0;iters<maxIters;iters++) {
	if((iters+1)%500==0) {
	  maxExplanationCost+=1.0;
	}
	if(bestCost >= 0 && maxExplanationCost >= bestCost)
	  maxExplanationCost = bestCost-1;
	if(maxExplanationCost < lowerCost) maxExplanationCost = lowerCost;
	if((iters)%100 == 0) 
	  out<<iters<<","<<expandTime<<","<<maxExplanationCost<<","<<bestCost<<","<<pathLength<<endl;

	//expand step
	timer.Reset();
	vector<int> newnodes;
	planner.Expand2(maxExplanationCost,newnodes);
	expandTime += timer.ElapsedTime();
	timer.Reset();

	//test for new path
	if((iters+1)%100 != 0) continue;
	if(!planner.modeGraph.nodes[planner.roadmap.nodes[1].mode].pathCovers.empty()) {
	  //if(planner.GreedyPath(0,1,path,bestPathCover)) {
	  if(planner.GreedyPath(path,bestPathCover)) {
	    if(planner.Cost(bestPathCover) < bestCost)
	      printf("Improved cover: iter %d, limit %g, cost %g\n",iters,maxExplanationCost,planner.Cost(bestPathCover));
	    bestCost = planner.Cost(bestPathCover);
	    double oldLength = pathLength;
	    pathLength = 0;
	    for(size_t i=0;i+1<path.size();i++)
	      pathLength += planner.space->Distance(planner.roadmap.nodes[path[i]].q,planner.roadmap.nodes[path[i+1]].q);
	    if(oldLength != pathLength)
	      printf("Changed path length: iter %d, %g -> %g\n",iters,oldLength,pathLength);
	  }
	  else
	    printf("Warning, start and goal are connected but GreedyPath returned false?\n");
	}
	pathTime += timer.ElapsedTime();
      }
      out<<maxIters<<","<<expandTime<<","<<maxExplanationCost<<","<<bestCost<<","<<pathLength<<endl;
      out.close();

      printf("Expanding took time %g, path search took time %g\n",expandTime,pathTime);
      printf("Computing true optimal cover\n");
      planner.OptimalPath(0,1,path,bestPathCover);
      printf("%g\n",planner.Cost(bestPathCover));
      if(!path.empty()) {
	hasPath = true;
      }
      pathCover = bestPathCover;
      Refresh();
    }
    else if(key == '-') {
      maxExplanationCost -= 1.0;
      printf("Max explanation cost: %g\n",maxExplanationCost);
    }
    else if(key == '=') {
      maxExplanationCost += 1.0;
      printf("Max explanation cost: %g\n",maxExplanationCost);
    }
    else if(key == 'r') {
      drawRoadmap = !drawRoadmap;
      Refresh();
    }
  }

  virtual void Handle_Click(int button,int state,int x,int y) {
    if(state == GLUT_DOWN && button == GLUT_LEFT_BUTTON) {
      Config q;
      ClickToConfig(x,y,q);
      if(!hasStart) {
	if(cspace.pointSpace || cspace.translatingSpace) {
	  start=q;
	  hasStart=true;
	}
	else if(cspace.rigidSpace) {
	  start.resize(3);
	  start(0)=q(0);
	  start(1)=q(1);
	  start(2)=0;
	  hasStart=true;
	}
	else if(cspace.multiSpace) {
	  Vector oldstart=start;
	  int stride=(cspace.multiSpace->allowRotation?3:2);
	  start.resize(oldstart.n + stride);
	  if(!oldstart.empty())
	    start.copySubVector(0,oldstart);
	  start(oldstart.n) = q(0);
	  start(oldstart.n+1) = q(1);
	  if(stride==3)
	    start(oldstart.n+2) = 0;
	  if(start.n == cspace.multiSpace->robots.size()*stride)
	    hasStart=true;
	}
      }
      else if(!hasGoal) {
	if(cspace.pointSpace || cspace.translatingSpace) {
	  goal=q;
	  hasGoal=true;
	}
	else if(cspace.rigidSpace) {
	  goal.resize(3);
	  goal(0)=q(0);
	  goal(1)=q(1);
	  goal(2)=0;
	  hasGoal=true;
	}
	else if(cspace.multiSpace) {
	  Vector oldgoal=goal;
	  int stride=(cspace.multiSpace->allowRotation?3:2);
	  goal.resize(oldgoal.n + stride);
	  if(!oldgoal.empty())
	    goal.copySubVector(0,oldgoal);
	  goal(oldgoal.n) = q(0);
	  goal(oldgoal.n+1) = q(1);
	  if(stride==3)
	    goal(oldgoal.n+2) = 0;
	  if(goal.n == cspace.multiSpace->robots.size()*stride)
	    hasGoal=true;
	}
	if(hasStart && hasGoal) {
	  //planner.Init(start,goal);
	  goalSet.items[0] = goal;
	  planner.Init(start,&goalSet);
	  hasPath = false;
	}
      }
      else {
	printf("Expanding 100...\n");
	for(int iters=0;iters<100;iters++) {
	  vector<int> newnodes;
	  planner.Expand2(maxExplanationCost,newnodes);
	}
	printf("Computing greedy path\n");
	//planner.GreedyPath(0,1,path,pathCover);
	planner.GreedyPath(path,pathCover);
	if(!path.empty()) {
	  hasPath = true;
	}
      }
      Refresh();
    }
  }
};
