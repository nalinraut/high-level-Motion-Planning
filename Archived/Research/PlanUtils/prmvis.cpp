#include <GLdraw/GLUTProgram.h>
#include <GLdraw/drawextra.h>
#include <GLdraw/GL.h>
#include <GLdraw/GLColor.h>
#include <GL/glut.h>
#include <planning/AnyMotionPlanner.h>
#include <graph/IO.h>
#include "DrawableSpace.h"
#include <math/random.h>
#include <math/sample.h>
#include "XmlReader.h"
#include <Timer.h>
#include <iostream>
#include <fstream>
#include <sstream>
using namespace std;
using namespace GLDraw;

struct DrawGraphCallback : public Graph::CallbackBase<int>
{
  DrawGraphCallback(RoadmapPlanner::Roadmap& _prm,DrawableCSpace* _space=NULL)
    :space(_space),prm(_prm) {}

  void Draw() {
    if(space) {
      edgeColor.setCurrentGL();
      for(size_t i=0;i<prm.nodes.size();i++) {
	RoadmapPlanner::Roadmap::Iterator e;
	for(prm.Begin(i,e);!e.end();e++) {
	  space->DrawRoadmapEdge(prm.nodes[e.source()],prm.nodes[e.target()]);
	}
      }
      nodeColor.setCurrentGL();
      for(size_t i=0;i<prm.nodes.size();i++) 
	space->DrawRoadmapNode(prm.nodes[i]);
    }
    else {
      edgeColor.setCurrentGL();
      glBegin(GL_LINES);
      for(size_t i=0;i<prm.nodes.size();i++) {
	RoadmapPlanner::Roadmap::Iterator e;
	for(prm.Begin(i,e);!e.end();e++) {
	  glVertex2v(prm.nodes[e.source()]);
	  glVertex2v(prm.nodes[e.target()]);
	}
      }
      glEnd();
      nodeColor.setCurrentGL();
      glBegin(GL_POINTS);
      for(size_t i=0;i<prm.nodes.size();i++) 
	glVertex2v(prm.nodes[i]);
      glEnd();
    }
  }

  bool ForwardEdge(int i,int j)
  {
    edgeColor.setCurrentGL();
    glBegin(GL_LINES);
    glVertex2v(prm.nodes[i]);
    glVertex2v(prm.nodes[j]);
    glEnd();
    return true;
  }

  void BackEdge(int i,int j)
  {
    edgeColor.setCurrentGL();
    glBegin(GL_LINES);
    glVertex2v(prm.nodes[i]);
    glVertex2v(prm.nodes[j]);
    glEnd();
  }

  void CrossEdge(int i,int j)
  {
    edgeColor.setCurrentGL();
    glBegin(GL_LINES);
    glVertex2v(prm.nodes[i]);
    glVertex2v(prm.nodes[j]);
    glEnd();
  }

  void Visit(int i)
  {
    nodeColor.setCurrentGL();
    glBegin(GL_POINTS);
    glVertex2v(prm.nodes[i]);
    glEnd();
  }

  DrawableCSpace* space;
  RoadmapPlanner::Roadmap& prm;
  GLColor nodeColor,edgeColor;
};


struct MotionPlannerProgram : public GLUTProgramBase
{
  DrawableCSpace cspace;

  MotionPlannerProgram()
    :GLUTProgramBase(600,600)
  {
    //TODO: the main program, or the child program must setup the cspace
  }

  void SetupDisplay() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_POINT_SMOOTH);

    //setup grid -> screen camera frame
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0,width,0,height,-10,10);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    //glScalef(width/Real(grid.m),height/Real(grid.n),One);
    glScalef(width,height,One);
  }
  void DrawCSpace() {
    cspace.DrawStatic();
  }

  virtual void Handle_Display() {
    SetupDisplay();
    DrawCSpace();
    glutSwapBuffers();
  }

  virtual void Handle_Reshape(int w,int h) {
    GLUTProgramBase::Handle_Reshape(w,h);
    glViewport(0,0,(GLsizei)width,(GLsizei)height);
    glutPostRedisplay();
  }

  void ClickToConfig(int x,int y,Config& q) const
  {
    q.resize(2);
    q(0) = Real(x)/Real(width);
    q(1) = 1.0-Real(y)/Real(height);
  }
};


struct PRMPlannerProgram : public MotionPlannerProgram
{
  bool hasStart,hasGoal,hasPath;
  Config start,goal;
  MilestonePath path;
  MotionPlannerFactory factory;
  MotionPlannerInterface* planner;
  RoadmapPlanner prm;
  vector<int> milestoneLabels;

  bool useNearestNeighbors;
  int connectionNeighbors;
  Real connectionThreshold;

  PRMPlannerProgram()
    :planner(NULL),prm(NULL)
  {
    hasStart=hasGoal=hasPath=false;

    useNearestNeighbors = false;
    connectionNeighbors = 10;
    connectionThreshold = Inf;
  }

  virtual bool Initialize() {
    if(!MotionPlannerProgram::Initialize()) return false;
    return true;
  }

  virtual void Handle_Display() {
    SetupDisplay();
    DrawCSpace();

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    DrawGraphCallback callback(prm.roadmap,&cspace);
    callback.nodeColor.set(1,0,1);
    callback.edgeColor.set(0.5,0,0.5);
    glPointSize(3.0);
    glLineWidth(1.0);
    //prm.roadmap.DFS(callback);
    callback.Draw();
    const static GLColor colors[8] = {GLColor(1,1,0),GLColor(0,1,1),GLColor(1,0,1),GLColor(1,0.5,0),GLColor(0,1,0.5),GLColor(0.5,0,1),GLColor(0.5,1,0),GLColor(0,0.5,1)};
    for(size_t i=0;i<milestoneLabels.size();i++) {
      if(i >= prm.roadmap.nodes.size()) break;
      if(milestoneLabels[i] > 0) {
	colors[milestoneLabels[i]%8].setCurrentGL();
	glPointSize(5.0);
	cspace.DrawRoadmapNode(prm.roadmap.nodes[i],2);
      }
    }
    glBegin(GL_POINTS);
    glEnd();
    if(hasPath) {
      glColor3f(0,0,1);
      glLineWidth(4.0);
      for(size_t i=0;i+1<path.NumMilestones();i++)
	cspace.DrawRoadmapEdge(path.GetMilestone(i),path.GetMilestone(i+1),1);
    }
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

    SafeDelete(planner);
    planner = factory.Create(cspace);
    int istart = planner->AddMilestone(start);
    int igoal = planner->AddMilestone(goal);
    Assert(istart == 0);
    Assert(igoal == 1);
    for(int i=0;i<1000;i++) {
      planner->PlanMore();
      if(planner->IsConnected(0,1)) {
	planner->GetPath(0,1,path);
	hasPath = true;
	hasStart = true;
	hasGoal = true;
	planner->GetRoadmap(prm);
	return true;
      }
    }
    return false;
  }

  //attempts a complete plan, for n iters
  //returns the number of used iterations
  int PlanIters(int n)
  {
    SafeDelete(planner);
    planner = factory.Create(cspace);
    int istart=planner->AddMilestone(start);
    int igoal=planner->AddMilestone(goal);
    for(int i=0;i<n;i++) {
      if(planner->IsConnected(istart,igoal)) return i;
      planner->PlanMore();
    }
    return n;
  }

  void BuildExhaustive(int n)
  {
    SafeDelete(planner);
    MotionPlannerFactory f;
    f.knn = 0;
    f.connectionThreshold = Inf;
    planner = f.Create(cspace);
    //int istart=planner->AddMilestone(start);
    //int igoal=planner->AddMilestone(goal);
    planner->PlanMore(n);
  }

  bool LoadRoadmap(const char* fn)
  {
      ifstream in(fn);
      Graph::Graph<string,string> Gstr;
      if(!Graph::Load_TGF(in,Gstr)) {
	printf("Failed to read TGF file %s\n",fn);
	return false;
      }
      else {
	printf("Loaded TGF file %s\n",fn);
	if(!Graph::NodesFromStrings(Gstr,prm.roadmap)) {
	  printf("Failed to parse configs from %s\n",fn);
	  return false;
	}
	//reconstruct edges
	for(size_t i=0;i<prm.roadmap.nodes.size();i++) {
	  Graph::EdgeIterator<SmartPointer<EdgePlanner> > e;
	  for(prm.roadmap.Begin(i,e);!e.end();++e) {
	    *e = cspace->LocalPlanner(prm.roadmap.nodes[e.source()],prm.roadmap.nodes[e.target()]);
	  }
	}
      }
      in.close();
      return true;

  }

  virtual void Handle_Keypress(unsigned char key,int x,int y)
  {
    if(key == 'S') {
      Graph::Graph<string,string> Gstr;
      Graph::NodesToStrings(prm.roadmap,Gstr);
      ofstream out("roadmap.tgf");
      Graph::Save_TGF(out,Gstr);
      out.close();
    }
    else if(key == 'L') {
      if(LoadRoadmap("roadmap.tgf")) {
	Refresh();
      }
    }
    else if(key == 'l') {
      //load labels
      ifstream in("roadmap.labels");
      vector<int> labels;
      while(in) {
	int val;
	in>>val;
	if(in) labels.push_back(val);
      }
      milestoneLabels.resize(prm.roadmap.nodes.size(),0);
      copy(labels.begin(),labels.begin()+Min(labels.size(),milestoneLabels.size()),milestoneLabels.begin());
    }
    else if(key == 's') {
      if(!SampleProblem()) return;
      Refresh();
    }
    else if(key == 't') {
      if(!hasStart) {
	if(!SampleConfig(start)) return;
      }
      else if(!hasGoal) {
	if(!SampleConfig(goal)) return;
      }

      printf("Saving timing results to prm_trials.txt\n");
      ofstream out("prm_trials.txt");
      int maxIters=1000;
      for(int trials=0;trials<1000;trials++) {
	printf("%d ",trials); fflush(stdout);
	int res=PlanIters(maxIters);
	out<<res<<endl;
      }
      printf("\n");
      out.close();
      printf("Done.\n");
    }
    /*
    else if(key == 'e') {
      prm.Cleanup();
      BuildExhaustive(1000);
      planner->GetRoadmap(prm);
      
      //ofstream out("visibility_epsilon.txt");
      //for(size_t i=0;i<prm.roadmap.nodes.size();i++) {
//	out<<i<<" "<<CalcEpsilon(prm,i)<<endl;
  //    }
    //  out.close();
      
      pair<Real,Real> ab;
      ab=CalcAlphaBetaGreedy(prm,100,&highlight);
      printf("AB(100) = %g,%g\n",ab.first,ab.second);
      ab=CalcAlphaBetaGreedy(prm,200,&highlight);
      printf("AB(200) = %g,%g\n",ab.first,ab.second);
      ab=CalcAlphaBetaGreedy(prm,500,&highlight);
      printf("AB(500) = %g,%g\n",ab.first,ab.second);
      ab=CalcAlphaBetaGreedy(prm,1000,&highlight);
      printf("AB(1000) = %g,%g\n",ab.first,ab.second);
      Refresh();
    }
    */
    else if(key == 'v') {
      BuildExhaustive(100);
      planner->GetRoadmap(prm);
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
	  SafeDelete(planner);
	  planner = factory.Create(cspace);
	  int istart = planner->AddMilestone(start);
	  int igoal = planner->AddMilestone(goal);
	  Assert(istart==0 && igoal==1);
	  hasPath = false;
	}
      }
      else if(!hasPath) {
	for(int iters=0;iters<100;iters++) {
	  planner->PlanMore();
	  if(planner->IsConnected(0,1)) {
	    planner->GetPath(0,1,path);
	    planner->GetRoadmap(prm);
	    hasPath = true;
	    break;
	  }
	}
	planner->GetRoadmap(prm);
      }
      else if(hasPath) {
	hasStart=false;
	hasGoal=false;
	hasPath=false;
	start.resize(0);
	goal.resize(0);
	SafeDelete(planner);
	prm.Cleanup();
      }
      Refresh();
    }
    /*
    else if(state == GLUT_DOWN && button == GLUT_RIGHT_BUTTON) {
      Config q;
      ClickToConfig(x,y,q);
      Real closestDist = Inf;
      int closest=-1;
      for(size_t i=0;i<prm.roadmap.nodes.size();i++) {
	Real d=cspace->Distance(prm.roadmap.nodes[i],q);
	if(d < closestDist) {
	  closestDist = d;
	  closest = (int)i;
	}
      }
      if(closest >= 0) {
	if(highlight.count(closest)==0)
	  highlight.insert(closest);
	else
	  highlight.erase(highlight.find(closest));
      }
      pair<Real,Real> ab=ComputeAlphaBeta(highlight,prm.roadmap.nodes.size(),true);
      printf("AB = %g,%g\n",ab.first,ab.second);
      Refresh();
    }
    */
  }
};




int main(int argc,char** argv)
{
  Srand(time(NULL));
  if(argc <= 1) {
    printf("Must specify an XML file describing the C-space\n");
    return 1;
  }
  PRMPlannerProgram program;
  for(int i=1;i<argc;i++) {
    if(0 == strcmp(FileExtension(argv[i]),"xml")) {
      XmlDocument doc;
      if(!doc.Load(argv[i])) {
	printf("Error loading XML document %s\n",argv[i]);
	return 1;
      }
      if(0==strcmp(doc.RootElement()->Value(),"planner")) {
	if(!XmlParse(doc.RootElement(),program.factory)) {
	  printf("Error reading planner from XML document %s\n",argv[i]);
	  return 1;
	}
      }
      else {
	if(!program.cspace.ReadFromXml(doc.RootElement())) {
	  printf("Error reading cspace from XML document %s\n",argv[i]);
	  return 1;
	}
      }
    }
    else if(0 == strcmp(FileExtension(argv[i]),"tgf")) {
      if(!program.LoadRoadmap(argv[i])) 
	return 1;
    }
  }
  return program.Run("PRM Planner Test");
}
