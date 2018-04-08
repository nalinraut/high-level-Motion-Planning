#include <Graph/IO.h>
#include "MotionPlannerProgram.h"
#include <planning/MCRPlanner.h>
#include <planning/PerturbationCSpace.h>
#include <fstream>
#include <sstream>
#include <map>
using namespace std;

Real uncertaintyScale = 2.0;

struct RigidPerturbationCSpace : public PerturbationCSpace
{
  Config start;

  RigidPerturbationCSpace(CSpace* baseSpace,const vector<Vector>& _perturbations,const Config& _start)
    :PerturbationCSpace(baseSpace,_perturbations),start(_start)
  {
  }
  virtual Config Perturb(const Config& q,const Vector& perturbation)
  {
    assert(q.n==3);
    assert(start.n==3);
    Config res(3);
    res[2] = q[2] + perturbation[2];
    Matrix2 rot;
    rot.setRotate(perturbation[2]);
    Vector2 ofs(q[0]-start[0],q[1]-start[1]);
    Vector2 newpos = rot*ofs + Vector2(start[0]+perturbation[0],start[1]+perturbation[1]);
    res[0] = newpos[0];
    res[1] = newpos[1];
    //cout<<res<<endl;
    return res;
  }
};

struct SafestPathPlannerProgram : public MotionPlannerProgram
{
  bool hasStart,hasGoal,hasPath;
  Config start,goal;
  SmartPointer<PerturbationCSpace> perturbationSpace;
  MCRPlanner planner;
  int maxExplanationSize;
  vector<int> path;
  Subset pathCover;
  vector<int> milestoneLabels;
  bool drawRoadmap;
  bool drawEnvironmentLocal;

  SafestPathPlannerProgram()
    :planner(NULL)
  {
    hasStart=hasGoal=hasPath=false;
    maxExplanationSize = 0;
    drawRoadmap = true;
    drawEnvironmentLocal = true;
  }

  virtual bool Initialize() {
    if(!cspace) return false;
    if(!MotionPlannerProgram::Initialize()) return false;
    return true;
  }

  void SetupPlanner(int numSamples=100)
  {
    //now sample offsets
    Config perturbationSize;
    cspace->Sample(perturbationSize);
    perturbationSize.set(0.01);
    perturbationSize(2) = 0.1;
    perturbationSize *= uncertaintyScale;
    vector<Config> perturbations(numSamples);
    for(int sample=0;sample<numSamples;sample++) {
      perturbations[sample].resize(perturbationSize.n);
      for(int i=0;i<perturbationSize.n;i++)
	perturbations[sample][i] = Rand(-perturbationSize(i),perturbationSize(i));
    }
    //if it's a rigid space, make the rotations about the start configuration only
    if(cspace.rigidSpace) {
      perturbationSpace = new RigidPerturbationCSpace(cspace,perturbations,start);
    }
    else {
      perturbationSpace = new PerturbationCSpace(cspace,perturbations);
    }
    planner.space = perturbationSpace;
    planner.Init(start,goal);
    planner.goalConnectThreshold = 5.0;
    planner.expandDistance = 0.1;

    Subset lowerCover;
    vector<bool> violations;
    planner.space->CheckObstacles(start,violations);
    lowerCover=Subset(violations);
    printf("Start is covered by %d constraints\n",lowerCover.count());
    for(size_t i=0;i<violations.size();i++)
      if(violations[i])
	printf("  %s\n",planner.space->ObstacleName(i).c_str());
    planner.space->CheckObstacles(goal,violations);
    printf("Goal is covered by %d constraints\n",Subset(violations).count());
    for(size_t i=0;i<violations.size();i++)
      if(violations[i])
	printf("  %s\n",planner.space->ObstacleName(i).c_str());
    lowerCover=lowerCover+Subset(violations);
    printf("Start and goal are covered by %d constraints\n",lowerCover.count());
    Subset origCover;
    planner.Completion(0,0,1,origCover);
    printf("Straight line violates %d constraints\n",origCover.count());
  }

  void DrawOutline(const vector<vector<Vector2> >& polys,const Config& q)
  {
    glPushMatrix();
    glTranslated(q[0],q[1],0);
    glRotated(RtoD(q[2]),0,0,1);
    for(size_t i=0;i<polys.size();i++) {
      glBegin(GL_LINE_LOOP);
      for(size_t j=0;j<polys[i].size();j++) 
	glVertex2v(polys[i][j]);
      glEnd();
    }
    glPopMatrix();
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
    if(drawEnvironmentLocal && perturbationSpace) {
      vector<vector<Vector2> > polys;
      cspace.rigidSpace->obstacles.ToPolygons(polys);
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

      size_t npert = Min(perturbationSpace->perturbations.size(),size_t(100));
      for(size_t p=0;p<npert;p++) {
	glPushMatrix();
	Config q = perturbationSpace->perturbations[p];
	glTranslated(q[0]+start[0],q[1]+start[1],0);
	glRotated(RtoD(q[2]),0,0,1);
	glTranslated(-start[0],-start[1],0);

	for(size_t i=0;i<polys.size();i++) {
	  glColor4f(0,0,0,0.02);
	  glBegin(GL_TRIANGLE_FAN);
	  for(size_t j=0;j<polys[i].size();j++)
	    glVertex2v(polys[i][j]);
	  glEnd();
	}
	glPopMatrix();
      }
    }
    else {
      DrawCSpace();
    }

    if(perturbationSpace && !drawEnvironmentLocal) {
      glPointSize(7.0);
      vector<vector<Vector2> > polys;
      if(cspace.rigidSpace)
	cspace.rigidSpace->robot.ToPolygons(polys);
      size_t npert = Min(perturbationSpace->perturbations.size(),size_t(20));

      if(hasStart) {
	glColor4f(0,1,0,1.0/float(npert));
	for(size_t i=0;i<npert;i++) 
	  cspace.DrawConfiguration(perturbationSpace->Perturb(start,perturbationSpace->perturbations[i]),1);
	glColor4f(0,0.5,0,0.5);
	for(size_t i=0;i<npert;i++) 
	  DrawOutline(polys,perturbationSpace->Perturb(start,perturbationSpace->perturbations[i]));
      }
      if(hasGoal) {
	glColor4f(1,0,0,1.0/float(npert));
	for(size_t i=0;i<npert;i++) 
	  cspace.DrawConfiguration(perturbationSpace->Perturb(goal,perturbationSpace->perturbations[i]),2);
	glColor4f(0.5,0,0,0.5);
	for(size_t i=0;i<npert;i++) 
	  DrawOutline(polys,perturbationSpace->Perturb(goal,perturbationSpace->perturbations[i]));
      }
    }

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    if(drawRoadmap) {
      DrawExplainingCallback callback(planner,&cspace);
      callback.nodeColor.resize(1);
      callback.nodeColor[0].set(0,0,0);
      callback.edgeColor.set(0,0,0,0.25);
      if(perturbationSpace && cspace.rigidSpace) {
	callback.nodeColor.resize(perturbationSpace->perturbations.size());
	for(size_t i=0;i<perturbationSpace->perturbations.size();i++)
	  callback.nodeColor[i].set(0,0,0,1.0-double(i)/perturbationSpace->perturbations.size());
      }
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
      glPointSize(5.0);
      glLineWidth(1.0);
      callback.Draw();
    }
    
    if(hasPath) {
      glColor3f(0,0,1);
      glLineWidth(4.0);
      for(size_t i=0;i+1<path.size();i++)
	cspace.DrawRoadmapEdge(planner.roadmap.nodes[path[i]].q,planner.roadmap.nodes[path[i+1]].q);
      if(cspace.rigidSpace) {
	vector<vector<Vector2> > polys;
	cspace.rigidSpace->robot.ToPolygons(polys);
	MilestonePath mpath;
	planner.GetMilestonePath(path,mpath);
	vector<Real> pathLen(mpath.NumMilestones());
	pathLen[0] = 0;
	for(int i=0;i<mpath.edges.size();i++)
	  pathLen[i+1] = pathLen[i] + cspace->Distance(mpath.edges[i]->Start(),mpath.edges[i]->Goal());
	
	Real t=0.0;
	Real dt = 0.1;
	glLineWidth(1.0);
	for(size_t i=0;i<path.size();i++) {
	  while(pathLen[i+1] > t + dt) {
	    t += dt;
	    Real u=(t-pathLen[i])/(pathLen[i+1]-pathLen[i]);
	    Config q;
	    mpath.edges[i]->Eval(u,q);
	    glColor4f(0,0,1,0.5);
	    cspace.DrawConfiguration(q,3);
	    glColor4f(0,0,0.5,1);
	    DrawOutline(polys,q);
	  }
	}
      }
    }

    if(!perturbationSpace || drawEnvironmentLocal) {
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

  virtual void Handle_Keypress(unsigned char key,int x,int y)
  {
    if(key == 'p') {
      //print planner stats
      printf("Expands: %d, refines %d (%d successful), explores %d\n",planner.numExpands,planner.numRefinementAttempts,planner.numRefinementSuccesses,planner.numExplorationAttempts);
      printf("Config checks %d, edge checks %d\n",planner.numConfigChecks,planner.numEdgeChecks);
      printf("Path cover updates %d, iterations %d\n",planner.numUpdatePaths,planner.numUpdatePathsIterations);
    }
    else if(key == 't') {
      int maxExplanationSize=0;
      Subset bestPathCover;
      int bestPath=-1;
      double pathLength=-1;
      Timer timer;
      double expandTime=0,pathTime=0;
      //compute initial path: start->goal
      path.resize(2);
      path[0] = 0; path[1] = 1;
      planner.Completion(0,0,1,bestPathCover);
      bestPath = bestPathCover.count();

      Subset lowerCover;
      vector<bool> violations;
      planner.space->CheckObstacles(start,violations);
      lowerCover=Subset(violations);
      planner.space->CheckObstacles(goal,violations);
      lowerCover=lowerCover+Subset(violations);
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
	  maxExplanationSize += (int)Ceil(double(bestPath-lowerCover.count())/double((maxIters-iters)/500));
	}
	if(bestPath >= 0 && maxExplanationSize >= bestPath)
	  maxExplanationSize = bestPath-1;
	if(maxExplanationSize < lowerCover.count()) maxExplanationSize = lowerCover.count();
	if((iters)%100 == 0) 
	  out<<iters<<","<<expandTime<<","<<maxExplanationSize<<","<<bestPath<<","<<pathLength<<endl;

	//expand step
	timer.Reset();
	vector<int> newnodes;
	planner.Expand2(maxExplanationSize,newnodes);
	expandTime += timer.ElapsedTime();
	timer.Reset();

	//test for new path
	if((iters+1)%100 != 0) continue;
	if(planner.modeGraph.nodes[planner.roadmap.nodes[1].mode].minCover < bestPath) {
	  if(planner.GreedyPath(0,1,path,bestPathCover)) {
	    if(bestPathCover.count() < bestPath)
	      printf("Improved cover: iter %d, limit %d, size %d\n",iters,maxExplanationSize,bestPathCover.count());
	    bestPath = bestPathCover.count();
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
      out<<maxIters<<","<<expandTime<<","<<maxExplanationSize<<","<<bestPath<<","<<pathLength<<endl;
      out.close();

      printf("Expanding took time %g, path search took time %g\n",expandTime,pathTime);
      printf("Computing true optimal cover\n");
      planner.OptimalPath(0,1,path,bestPathCover);
      printf("%d\n",bestPathCover.count());
      if(!path.empty()) {
	hasPath = true;
      }
      Refresh();
    }
    else if(key == '-') {
      maxExplanationSize --;
      printf("Max explanation size: %d\n",maxExplanationSize);
    }
    else if(key == '=') {
      maxExplanationSize ++;
      printf("Max explanation size: %d\n",maxExplanationSize);
    }
    else if (key == 'r') {
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
	  SetupPlanner();
	  hasPath = false;
	}
      }
      else {
	printf("Expanding...\n");
	for(int iters=0;iters<100;iters++) {
	  vector<int> newnodes;
	  planner.Expand2(maxExplanationSize,newnodes);
	}
	planner.GreedyPath(0,1,path,pathCover);
	if(!path.empty()) {
	  hasPath = true;
	  printf("Greedy path has cover %d\n",pathCover.count());
	}
      }
      Refresh();
    }
  }
};
