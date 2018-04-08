
#include "displace.h"
#include "MotionPlannerProgram.h"
#include <fstream>
#include <sstream>
#include <limits>
#include <map>
using namespace std;

//true if aabb's in the space should be considered immovable obstacles
const static bool blockBBs = true;
//true if aabb's in the space should be considered rotational obstacles
const static bool rotateBBs = false;

struct Stats
{
  char lastOp; //'n' = none, 'e' = expand, 'd' = displacement sample, 'o' = local optimization
  int numIters;
  Real totalTime;
  Real solutionCost,pathLength;
  int numNodes,numDispSamples;
};

struct DisplacementPlannerProgram : public MotionPlannerProgram
{
  bool hasStart,hasGoal,hasPath;
  Config start,goal;
  SmartPointer<ObstacleDisplacementCSpace> displacementSpace;
  DisplacementPlanner planner;
  Real timeAccum;
  Real maxExplanationCost;
  vector<int> path;
  vector<int> pathAssignment;
  vector<int> milestoneLabels;
  vector<Stats> stats;
  bool drawOutlines,drawRoadmap,drawSamples;

  DisplacementPlannerProgram()
    :planner(NULL)
  {
    hasStart=hasGoal=hasPath=false;
    maxExplanationCost = 0;

    drawOutlines = false;
    drawRoadmap = true;
    drawSamples = false;
  }

  void StoreCurrentStats(char lastOp='n')
  {
    Stats s;
    s.lastOp = lastOp;
    s.numIters = planner.numExpands;
    s.totalTime = timeAccum;
    s.solutionCost = planner.OptimalCost(1);
    if(planner.OptimalPathTo(1)!=NULL) 
      s.pathLength = planner.OptimalPathTo(1)->pathLength;
    else
      s.pathLength = 0;
    s.numNodes = (int)planner.roadmap.nodes.size();
    s.numDispSamples = 0;
    for(size_t i=0;i<planner.displacementSamples.size();i++)
      s.numDispSamples += (int)planner.displacementSamples[i].size();
    stats.push_back(s);
  }

  void SaveStats(const char* fn) 
  {
    ofstream out(fn,ios::out);
    if(!out) {
      fprintf(stderr,"%s could not be opened for writing\n",fn);
      return;
    }
    out<<"op,#iters,time,#nodes,#displacements,solution cost,path length"<<endl;
    for(size_t i=0;i<stats.size();i++)
      out<<stats[i].lastOp<<","<<stats[i].numIters<<","<<stats[i].totalTime<<","<<stats[i].numNodes<<","<<stats[i].numDispSamples<<","<<stats[i].solutionCost<<","<<stats[i].pathLength<<endl;
    out.close();
    printf("Wrote stats to %s\n",fn);
  }

  virtual bool Initialize() 
  {
    if(cspace.pointSpace!=NULL) {
      GeometricDisplacementCSpace* dspace = new GeometricDisplacementCSpace(cspace.pointSpace);
      for(int i=0;i<dspace->NumObstacles();i++) {
	string s = dspace->ObstacleName(i);
	if(s.substr(0,4) == "aabb") {
	  if(blockBBs) {
	    printf("Blocking %d\n",i);
	    dspace->motionType[i] = GeometricDisplacementCSpace::Fixed;
	  }
	  else if(rotateBBs) {
	    printf("Rotating %d\n",i);
	    dspace->motionType[i] = GeometricDisplacementCSpace::Rotation;
	  }
	}
      }
      displacementSpace = dspace;
    }
    else if(cspace.translatingSpace!=NULL) {
      TranslatingDisplacementCSpace* dspace = new TranslatingDisplacementCSpace(cspace.translatingSpace);
      for(int i=0;i<dspace->NumObstacles();i++) {
	string s = dspace->ObstacleName(i);
	if(s.substr(0,4) == "aabb") {
	  if(blockBBs) {
	    printf("Blocking %d\n",i);
	    dspace->motionType[i] = TranslatingDisplacementCSpace::Fixed;
	  }
	  else if(rotateBBs) {
	    printf("Rotating %d\n",i);
	    dspace->motionType[i] = TranslatingDisplacementCSpace::Rotation;
	  }
	}
      }
      displacementSpace = dspace;
    }
    else if(cspace.rigidSpace != NULL) {
      RigidDisplacementCSpace* dspace = new RigidDisplacementCSpace(cspace.rigidSpace);
      for(int i=0;i<dspace->NumObstacles();i++) {
	string s = dspace->ObstacleName(i);
	if(s.substr(0,4) == "aabb") {
	  if(blockBBs) {
	    printf("Blocking %d\n",i);
	    dspace->motionType[i] = RigidDisplacementCSpace::Fixed;
	  }
	  else if(rotateBBs) {
	    printf("Rotating %d\n",i);
	    dspace->motionType[i] = RigidDisplacementCSpace::Rotation;
	  }
	}
      }
      displacementSpace = dspace;
    }
    else {
      FatalError("Don't have displacement version of multi-robot spaces\n");
    }
    planner.space = displacementSpace;
    planner.pathCostWeight = 0.1;
    //planner.pathCostWeight = 1.0;
    planner.updatePathsComplete = true;
    planner.numConnections = -1;
    planner.initialDisplacementCosts.resize(displacementSpace->NumObstacles(),0.1);

    pathAssignment.resize(displacementSpace->NumObstacles());
    fill(pathAssignment.begin(),pathAssignment.end(),0);

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
    //draw polygons
    GLColor ocolors[8] = {GLColor(0,0,0),GLColor(1,0,0),GLColor(0,1,0),GLColor(0,0,1),GLColor(0.5,0.5,0.5),GLColor(0,0.5,0.5),GLColor(0.5,0,0.5),GLColor(0.5,0.5,0) };
    int ofs = (cspace.pointSpace? 4:1);
    for(size_t i=0;i<polys.size();i++) {
      if(hasPath && pathAssignment[i+ofs]!=0) {
	//draw original obstacle
	ocolors[i%8].setCurrentGL();
	glLineWidth(2.0);
	glBegin(GL_LINE_LOOP);
	for(size_t j=0;j<polys[i].size();j++)
	  glVertex2v(polys[i][j]);
	glEnd();
	glLineWidth(1.0);

	//draw displaced obstacle
	Vector disp = planner.displacementSamples[i+ofs][pathAssignment[i+ofs]];
	if(disp.n == 0) continue;
	if(disp.n == 1) {
	  //consider it to be a rotation displacement
	  AABB2D bb;
	  bb.setPoint(polys[i][0]);
	  for(size_t j=0;j<polys[i].size();j++) 
	    bb.expand(polys[i][j]);
	  Vector2 c=0.5*(bb.bmin+bb.bmax);
	  RigidTransform2D T;
	  T.R.setRotate(disp(0));
	  T.t = c-T.R*c;
	  glColor4f(0,0,0,0.6);   
	  glBegin(GL_TRIANGLE_FAN);
	  for(size_t j=0;j<polys[i].size();j++) 
	    glVertex2v(T*polys[i][j]);
	  glEnd();
	}
	else {
	  //consider it to be a translation displacement
	  Assert(disp.n==2);
	  Vector2 disp2(disp[0],disp[1]);
	  glColor4f(0,0,0,0.6);   
	  glBegin(GL_TRIANGLE_FAN);
	  for(size_t j=0;j<polys[i].size();j++) 
	    glVertex2v(polys[i][j]+disp2);
	  glEnd();
	}
      }
      else {
	if(drawOutlines) {
	  ocolors[i%8].setCurrentGL();
	  glBegin(GL_LINE_LOOP);
	  for(size_t j=0;j<polys[i].size();j++)
	    glVertex2v(polys[i][j]);
	  glEnd();
	}
	else {
	  if(!planner.space->displacementSpaces.empty()&&planner.space->displacementSpaces[i+ofs]!=NULL)
	    glColor4f(1,0.5,0,0.6);
	  else
	    glColor4f(0.2,0.2,0.2,1);
	  glBegin(GL_TRIANGLE_FAN);
	  for(size_t j=0;j<polys[i].size();j++) 
	    glVertex2v(polys[i][j]);
	  glEnd();
	}
      }
    }

    if(drawSamples) {
      for(size_t i=0;i<polys.size();i++) {
	if(planner.space->displacementSpaces[i+ofs]==NULL) continue;
	Vector2 centroid = polys[i][0];
	for(size_t j=1;j<polys[i].size();j++)
	  centroid += polys[i][j];
	centroid /= polys[i].size();
	glColor3f(0,1,0);
	if(planner.displacementSamples[i+ofs][0].n==1) {
	  //assume rotational displacement
	  AABB2D bb;
	  bb.setPoint(polys[i][0]);
	  for(size_t j=0;j<polys[i].size();j++) 
	    bb.expand(polys[i][j]);
	  Vector2 c=0.5*(bb.bmin+bb.bmax);

	  glLineWidth(3.0);
	  glBegin(GL_LINES);
	  for(size_t j=0;j<planner.displacementSamples[i+ofs].size();j++) {
	    RigidTransform2D T;
	    T.R.setRotate(planner.displacementSamples[i+ofs][j](0));
	    T.t = c-T.R*c;
	    glVertex2v(polys[i][2]);
	    glVertex2v(T*polys[i][2]);
	  }
	  glEnd();
	  glLineWidth(1.0);
	}
	else {
	  glLineWidth(3.0);
	  glBegin(GL_LINES);
	  for(size_t j=0;j<planner.displacementSamples[i+ofs].size();j++) {
	    glVertex2v(centroid);
	    glVertex2v(centroid+Vector2(planner.displacementSamples[i+ofs][j]));
	  }
	  glEnd();
	  glLineWidth(1.0);
	}
      }
    }

    if(!drawOutlines) {
      glDisable(GL_BLEND);
    }
    //DrawCSpace();
    if(drawRoadmap) {
      glDisable(GL_DEPTH_TEST);
      glDisable(GL_LIGHTING);
      DrawDisplacementCallback callback(planner,&cspace);
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

    planner.Init(start,goal);
    maxExplanationCost = Max(Epsilon,planner.pathCostWeight*planner.space->Distance(start,goal));
    fill(pathAssignment.begin(),pathAssignment.end(),0);
    if(PlanDisplacement(planner,1000,100)) {
      UpdateSolution();
    }
    return false;
  }

  void AutoPlan(int numIters,int numExpandsPerDisp,int numLocalOptimize,Real expandLimitStep)
  {
      if(expandLimitStep <= 0) {
	printf("Invalid expand limit step %g\n",expandLimitStep);
	return;
      }
      DisplacementPlanner::PathSearchNode* n=planner.OptimalPathTo(1);
      Real bestCost = planner.OptimalCost(1);
      Real t0 = timeAccum;
      Timer timer;
      for(int iters=0;iters<numIters;iters++) {
	if(n == NULL) {
	  //printf("Increasing explanation cost from %g by %g to %g.\n",maxExplanationCost,expandLimitStep,maxExplanationCost+expandLimitStep);
	  maxExplanationCost += expandLimitStep;
	}
	if((iters+1) % numExpandsPerDisp == 0) {
	  printf("Displacement sample iter %d...\n",iters);
	  planner.AddNewDisplacement(maxExplanationCost);
	  timeAccum = timer.ElapsedTime()+t0;
	  StoreCurrentStats('d');
	}
	vector<int> newnodes;
	planner.Expand(maxExplanationCost,newnodes);
	timeAccum = timer.ElapsedTime()+t0;
	StoreCurrentStats('e');
	if(planner.OptimalPathTo(1) != n) {
	  if(planner.OptimalCost(1) < bestCost) {
	    printf("Got a new path, optimizing\n");
	    if(numLocalOptimize>0) {
	      planner.RefineGoalPathAndDisplacements(numLocalOptimize,0.5,1.0);
	      planner.ShortcutGoalPath(1,5);
	    }

	    UpdateSolution();
	    n = planner.OptimalPathTo(1);
	    bestCost = planner.OptimalCost(1);
	    timeAccum = timer.ElapsedTime()+t0;
	    StoreCurrentStats('o');
	  }
	}
      }
  }

  virtual void Handle_Keypress(unsigned char key,int x,int y)
  {
    if(key=='h') {
      printf("[space] - Expand roadmap by 1 sample.\n");
      printf("a - Plan auto.\n");
      printf("d - Sample displacement.\n");
      printf("g - Refine goal path with 100 samples.\n");
      printf("p - Print planner stats.\n");
      printf("t - Do a plan according to some heuristic strategy?\n");
      printf("= - Increase max explanation size by 1.\n");
      printf("- - Reduce max explanation size by 1.\n");
      printf("r - Toggle draw roadmap.\n");
      printf("o - Toggle draw obstacle outlines.\n");
      printf("q - Toggle draw displacement samples.\n");
      printf("s - Save stats to stats.csv.\n");
    }
    else if(key == 'a') {
      int numTrials;
      int numIters,numExpandsPerDisp,numLocalOptimize;
      Real expandLimitStep;
      cout<<"Number of trials > "; cout.flush();
      cin >> numTrials;
      if(!cin) {
	printf("Error reading number\n");
	cin.clear();
	return;
      }
      cin.ignore( numeric_limits<streamsize>::max(), '\n' );

      cout<<"Number of expand iterations to perform > "; cout.flush();
      cin >> numIters;
      if(!cin) {
	printf("Error reading number\n");
	cin.clear();
	return;
      }
      cin.ignore( numeric_limits<streamsize>::max(), '\n' );

      cout<<"Number of expands per displacement sample > "; cout.flush();
      cin >> numExpandsPerDisp;
      if(!cin) {
	printf("Error reading number\n");
	cin.clear();
	return;
      }  
      cin.ignore( numeric_limits<streamsize>::max(), '\n' );

      cout<<"Number of local optimize iters > "; cout.flush();
      cin >> numLocalOptimize;
      if(!cin) {
	printf("Error reading number\n");
	cin.clear();
	return;
      }      
      cin.ignore( numeric_limits<streamsize>::max(), '\n' );
      cout<<"Expand limit growth > "; cout.flush();
      cin >> expandLimitStep;
      if(!cin) {
	printf("Error reading number\n");
	cin.clear();
	return;
      }      
      cin.ignore( numeric_limits<streamsize>::max(), '\n' );

      //debug
      //printf("%d iters %d expand per disp %d optimize %g expand step\n",numIters,numExpandsPerDisp,numLocalOptimize,expandLimitStep);
      //getchar();

      if(numTrials == 0) {
	//just go from the current status
	AutoPlan(numIters,numExpandsPerDisp,numLocalOptimize,expandLimitStep);
	Refresh();
      }
      else {
	for(int trial=0;trial<numTrials;trial++) {
	  planner.Init(start,goal);
	  timeAccum = 0;
	  stats.resize(0);
	  StoreCurrentStats('n');

	  AutoPlan(numIters,numExpandsPerDisp,numLocalOptimize,expandLimitStep);

	  char buf[64];
	  sprintf(buf,"stats%d.csv",trial+1);
	  SaveStats(buf);
	}
      }
      Refresh();
    }
    else if(key == 's') {
      //if(!SampleProblem()) return;
      SaveStats("stats.csv");
      Refresh();
    }
    else if(key == ' ') {
      vector<int> newnodes;
      Timer timer;
      planner.Expand(maxExplanationCost,newnodes);
      timeAccum += timer.ElapsedTime();
      UpdateSolution();
      StoreCurrentStats('e');
      Refresh();
    }
    else if(key == 'd') {
      Timer timer;
      planner.AddNewDisplacement(maxExplanationCost);
      timeAccum += timer.ElapsedTime();
      UpdateSolution();
      StoreCurrentStats('d');
      Refresh();
    }
    else if(key == 'g') {
      Timer timer;
      planner.RefineGoalDisplacements(100,0.5);
      planner.ShortcutGoalPath(1,5);
      timeAccum += timer.ElapsedTime();
      UpdateSolution();
      StoreCurrentStats('o');
      Refresh();
    }
    else if(key == 'b') {
      Timer timer;
      planner.RefineGoalPathAndDisplacements(100,0.5,1.0);
      //planner.ShortcutGoalPath(1,5);
      timeAccum += timer.ElapsedTime();
      UpdateSolution();
      StoreCurrentStats('o');
      Refresh();
    }
    else if(key == 'o') {
      drawOutlines = !drawOutlines;
      Refresh();
    }
    else if(key == 'q') {
      drawSamples = !drawSamples;
      Refresh();
    }
    else if(key == 'p') {
      //print planner stats
      printf("Expands: %d, refines %d (%d successful), explores %d\n",planner.numExpands,planner.numRefinementAttempts,planner.numRefinementSuccesses,planner.numExplorationAttempts);
      printf("Config checks %d, edge checks %d\n",planner.numConfigChecks,planner.numEdgeChecks);
      printf("Path cover updates %d, iterations %d\n",planner.numUpdateCovers,planner.numUpdateCoversIterations);
      printf("Time NN %g, refine %g, explore %g, update in %g, update out %g, overhead %g\n",planner.timeNearestNeighbors,planner.timeRefine,planner.timeExplore,planner.timeUpdateCoversIn,planner.timeUpdateCoversOut,planner.timeOverhead);
    }
    else if(key == 't') {
      Real maxExplanationCost=0;
      Real bestCost=Inf;
      double pathLength=-1;
      Timer timer;
      double expandTime=0,pathTime=0;
      path.resize(0);
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
	if(bestCost >= 0 && maxExplanationCost > bestCost)
	  maxExplanationCost = bestCost;
	if((iters)%100 == 0) 
	  out<<iters<<","<<expandTime<<","<<maxExplanationCost<<","<<bestCost<<","<<pathLength<<endl;

	//expand step
	timer.Reset();
	vector<int> newnodes;
	planner.Expand(maxExplanationCost,newnodes);
	expandTime += timer.ElapsedTime();
	timer.Reset();

	//test for new path
	if((iters+1)%100 != 0) continue;
	DisplacementPlanner::PathSearchNode* n=planner.OptimalPathTo(1);
	if(n) {
	  if(planner.OptimalCost(1) < bestCost) {
	    printf("Improved cover: iter %d, limit %g, cost %g\n",iters,maxExplanationCost,planner.OptimalCost(1));
	    bestCost = planner.OptimalCost(1);
	    Real oldLength = pathLength;
	    pathLength = n->pathLength;
	    if(oldLength != pathLength)
	      printf("Changed path length: iter %d, %g -> %g\n",iters,oldLength,pathLength);
	  }
	}
	maxExplanationCost += 1.0;
	pathTime += timer.ElapsedTime();
      }
      out<<maxIters<<","<<expandTime<<","<<maxExplanationCost<<","<<bestCost<<","<<pathLength<<endl;
      out.close();

      printf("Expanding took time %g, path search took time %g\n",expandTime,pathTime);
      printf("Computing true optimal cover\n");
      UpdateSolution();
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

  void UpdateSolution()
  {
    if(planner.OptimalPathTo(1)!=NULL) {
      maxExplanationCost = planner.OptimalCost(1);
      hasPath = true;
      DisplacementPlanner::PathSearchNode* n=planner.OptimalPathTo(1);
      pathAssignment = n->assignment;
      path.resize(0);
      while(n!=NULL) {
	path.push_back(n->vertex);
	n = n->parent;
      }
      reverse(path.begin(),path.end());
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
	  planner.Init(start,goal);
	  timeAccum = 0;
	  stats.resize(0);
	  StoreCurrentStats('n');
	  maxExplanationCost = Max(Epsilon,planner.pathCostWeight*planner.space->Distance(start,goal));
	  hasPath = false;
	}
      }
      else {
	printf("Expanding 20...\n");
	Timer timer;
	for(int iters=0;iters<20;iters++) {
	  vector<int> newnodes;
	  planner.Expand(maxExplanationCost,newnodes);
	}
	timeAccum += timer.ElapsedTime();
	UpdateSolution();
	StoreCurrentStats('e');
      }
      Refresh();
    }
  }
};
