#include "gcp.h"
#include <stdlib.h>
#include <stdio.h>
#include <KrisLibrary/Timer.h>
#include <KrisLibrary/GLdraw/GLUTProgram.h>
#include <KrisLibrary/GLdraw/GL.h>
#include <GL/glut.h>
#include <KrisLibrary/GLdraw/drawextra.h>

using namespace GCP;

void MakeFlatTerrain(PolygonalTerrain& terrain,double x0,double x1,double y,int numdivs)
{
  terrain.segments.resize(numdivs);
  for(int i=0;i<numdivs;i++) {
    Real u0 = Real(i)/Real(numdivs);
    Real u1 = Real(i+1)/Real(numdivs);
    terrain.segments[i].a.x = x0 + u0*(x1-x0);
    terrain.segments[i].a.y = y;
    terrain.segments[i].b.x = x0 + u1*(x1-x0);
    terrain.segments[i].b.y = y;
  }
}

void MakeStaircaseTerrain(PolygonalTerrain& terrain,double x0,double x1,double y0,double y1,int numsteps)
{
  terrain.segments.resize(numsteps*2);
  for(int i=0;i<numsteps;i++) {
    Real u0 = Real(i)/Real(numsteps);
    Real u1 = Real(i+1)/Real(numsteps);
    terrain.segments[2*i].a.x = x0 + u0*(x1-x0);
    terrain.segments[2*i].a.y = y0 + u0*(y1-y0);
    terrain.segments[2*i].b.x = x0 + u0*(x1-x0);
    terrain.segments[2*i].b.y = y0 + u1*(y1-y0);
    terrain.segments[2*i+1].a = terrain.segments[2*i].b;
    terrain.segments[2*i+1].b.x = x0 + u1*(x1-x0);
    terrain.segments[2*i+1].b.y = y0 + u1*(y1-y0);
  }
}

void DrawArc(const Vector2& x,const Vector2& v,Real a,Real xmax)
{
  glBegin(GL_LINE_STRIP);
  Real slope = v.y / v.x;
  Real dx = 0.05;
  Real u = 0;
  while(u < xmax - x.x) {
    glVertex2d(x.x+u,x.y+u*slope + 0.5*u*u*a);  
    u += dx;
  }
  u = xmax-x.x;
  glVertex2d(xmax,x.y+u*slope + 0.5*u*u*a);  
  glEnd();
}

#define ModeEditTerrain 0
#define ModePathCurvature 1
#define ModeCapturePoint 2
#define NumModes 3

class CapturePointGUI : public GLUTProgramBase
{
public:
  int mode;
  Vector2 viewMin,viewMax;
  Vector2 lastClickPoint;
  bool makingPolyline;
  Problem problem;

  bool doSolve;
  Real recomputeTime;
  bool validSolution;
  std::vector<Real> pathCurvatures;
  std::vector<Vector2> capturePoints;
  bool showTrace;
  std::vector<Vector2> xtrace,vtrace;
  int traceCount;

  CapturePointGUI()
    :GLUTProgramBase(640,640)
  {
    mode = ModeEditTerrain;
    makingPolyline = false;
    doSolve = true;
    recomputeTime = 0;
    validSolution = false;
    showTrace = false;
    traceCount = 0;

    //set up the initial conditions
    Real h = 1.0;
    Real vx0 = 1.0;
    problem.terrain.segments.resize(1);
    problem.terrain.segments[0].a = Vector2(0,0);
    problem.terrain.segments[0].b = Vector2(1,0);
    problem.SetInitialConditions(Vector2(0,h),Vector2(vx0,0));
    problem.SetFriction(1.5);
    problem.Lmin = 0.33;
    problem.Lmax = 2;

    viewMin.x = -0.5;
    viewMin.y = -0.5;
    viewMax.x = 1.5;
    viewMax.y = 1.5;
  }

  void RenderWorld() {
    glLineWidth(2.0);
    glBegin(GL_LINES);
    glColor3f(0,0,0);
    for(size_t i=0;i<problem.terrain.segments.size();i++) {
      glVertex2f(problem.terrain.segments[i].a.x,problem.terrain.segments[i].a.y);
      glVertex2f(problem.terrain.segments[i].b.x,problem.terrain.segments[i].b.y);
    }
    glEnd();
    if(makingPolyline) {
      glEnable(GL_POINT_SMOOTH);
      glPointSize(10.0);
      glBegin(GL_POINTS);
      glVertex2f(lastClickPoint.x,lastClickPoint.y);
      glEnd();
    }   

    //if the trace should be drawn, draw it
    if(showTrace) {
      //draw a faint trace
      glColor3f(1,0.5,0.5);
      for(size_t i=0;i<xtrace.size();i++)
	GLDraw::drawCircle2D(Math3D::Vector2(&xtrace[i].x),0.025);
      //draw the current state
      glColor3f(1,0,0);
      GLDraw::drawCircle2D(Math3D::Vector2(&xtrace[traceCount].x),0.05);
      glColor3f(1,0.5,0);
      glLineWidth(3.0);
      glBegin(GL_LINES);
      glVertex2d(xtrace[traceCount].x,xtrace[traceCount].y);
      Real h=1;
      glVertex2d(xtrace[traceCount].x+h*vtrace[traceCount].x,xtrace[traceCount].y+h*vtrace[traceCount].y);
      glEnd();
      glLineWidth(1.0);
      return;
    }

    glColor3f(1,0,0);
    GLDraw::drawCircle2D(Math3D::Vector2(&problem.x0.x),0.05);
    glColor3f(1,0.5,0);
    glLineWidth(3.0);
    glBegin(GL_LINES);
    glVertex2d(problem.x0.x,problem.x0.y);
    Real h=1;
    glVertex2d(problem.x0.x+h*problem.v0.x,problem.x0.y+h*problem.v0.y);
    glEnd();
    glLineWidth(1.0);

    if(mode == ModeCapturePoint) {
      if(validSolution) { //draw curvature
	glLineWidth(3.0);
	glColor3f(1,0.5,0);
      }
      else {
	glColor3f(1,0.75,0.5);
	glLineWidth(1.0);
      }
      DrawArc(problem.x0,problem.v0,problem.pathCurvature,problem.capturePoint.x);
      glColor3f(0,1,0);
      GLDraw::drawCircle2D(Math3D::Vector2(&problem.capturePoint.x),0.04);
    }
    else if(mode == ModePathCurvature) {
      float rad = 0.02;
      if(validSolution) { //draw capture point
	rad = 0.04;
	glColor3f(0,1,0);
      }
      else {
	glColor3f(0.5,1,0.5);
      }
      GLDraw::drawCircle2D(Math3D::Vector2(&problem.capturePoint.x),rad);
      glLineWidth(3.0);
      glColor3f(1,0.5,0);
      DrawArc(problem.x0,problem.v0,problem.pathCurvature,problem.x0.x + 5);
    }
    else {
      //draw all points / curvatures
      for(size_t i=0;i<capturePoints.size();i++) {
	Real u=Real(i+1)/Real(capturePoints.size());
	glColor3f(0,u,1);
	GLDraw::drawCircle2D(Math3D::Vector2(&capturePoints[i].x),0.02);
      }
      for(size_t i=0;i<pathCurvatures.size();i++) {
	Real u=Real(i+1)/Real(pathCurvatures.size());      
	glColor3f(1,u,0);
	DrawArc(problem.x0,problem.v0,pathCurvatures[i],capturePoints[i].x);
      }
    }
  }
  virtual bool Initialize() {
    if(!GLUTProgramBase::Initialize()) return false;
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glClearColor(1,1,1,1);
    glClearDepth(1);
    return true;
  }
  virtual void Handle_Display() {
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(viewMin.x,viewMax.x,viewMin.y,viewMax.y,-100,100);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    RenderWorld();
    glutSwapBuffers();
  }
  virtual void Handle_Keypress(unsigned char key,int x,int y)
  {
    if(key == ' ') { 
      mode = (mode+1)%NumModes;
      makingPolyline = false;
      Recompute();
      Refresh();
    }
    else if(key == 'r') {
      problem.terrain.segments.resize(0);
      problem.pathCurvature = 0;
      problem.capturePoint = Vector2(0,0);
      problem.capturePointNormal = Vector2(0,1);
      makingPolyline = false;
      Recompute();
      Refresh();
    }
    else if(key == 'a') {
      showTrace = !showTrace;
      if(showTrace) {
	traceCount = 0;
	problem.SimulationTrace(problem.timeStep,1000,xtrace,vtrace);
	SleepIdleCallback(0);
	if(xtrace.size()==0) showTrace=false;
      }
      Refresh();
    }
    else if(key == 's') {
      doSolve = !doSolve;
      if(doSolve) printf("Solve enabled\n");
      else printf("Solve disabled\n");
    }
  }
  virtual void Handle_Special(int key,int x,int y) {
    switch(key) {
    case GLUT_KEY_UP:
      problem.v0.y += 0.1;
      Recompute();
      Refresh();
      break;
    case GLUT_KEY_DOWN:
      problem.v0.y -= 0.1;
      Recompute();
      Refresh();
      break;
    case GLUT_KEY_RIGHT:
      problem.v0.x += 0.1;
      Recompute();
      Refresh();
      break;
    case GLUT_KEY_LEFT:
      problem.v0.x -= 0.1;
      if(problem.v0.x < 0) problem.v0.x = 0;
      Recompute();
      Refresh();
      break;
    }
  }
  Vector2 ClickPoint(float x,float y) {
    float ux = x/float(width);
    float uy = 1.0-y/float(height);
    return Vector2(viewMin.x+ux*(viewMax.x-viewMin.x),
		   viewMin.y+uy*(viewMax.y-viewMin.y));
  }

  void DoMouse(int x,int y) {
    if(mode == ModeEditTerrain) {
      Vector2 newClickPoint = ClickPoint(x,y);
      if(makingPolyline) {
	problem.terrain.segments.back().b = newClickPoint;
      }
      lastClickPoint = newClickPoint;
    }
    else if(mode == ModeCapturePoint) {
      Vector2 pt = ClickPoint(x,y);
      Real dmin = 1e300;
      //find closest point on terrain
      for(size_t i=0;i<problem.terrain.segments.size();i++) {
	Real d = problem.terrain.segments[i].Distance(pt);
	if(d < dmin) {
	  problem.capturePoint = problem.terrain.segments[i].ClosestPoint(pt);
	  problem.capturePointNormal = problem.terrain.segments[i].Normal();
	  dmin = d;
	}
      }
    }
    else { //setting path curvature
      Vector2 pt = ClickPoint(x,y);
      if(pt.x <= problem.x0.x) return;
      if(problem.v0.x == 0) return;
      Real dx = pt.x - problem.x0.x;
      Real dy = pt.y - problem.x0.y;
      Real slope0 = problem.v0.y/problem.v0.x;
      //y(dx)-y0 = dx*slope0 + 0.5*dx^2*curvature = dy
      problem.pathCurvature = (dy - slope0*dx)/(dx*dx*0.5);
    }
  }

  virtual void Handle_Drag(int x,int y) {
    DoMouse(x,y);
    Recompute();
    Refresh();
  }

  virtual void Handle_Click(int button,int state,int x,int y)
  {
    if(mode == ModeEditTerrain) {
      if(state == GLUT_DOWN) {
	if(!makingPolyline)
	  lastClickPoint = ClickPoint(x,y);
	for(size_t i=0;i<problem.terrain.segments.size();i++)
	  problem.terrain.segments[i].Sort();
	problem.terrain.segments.resize(problem.terrain.segments.size()+1);
	problem.terrain.segments.back().a = lastClickPoint;
	problem.terrain.segments.back().b = lastClickPoint;
      }
      if(state == GLUT_UP) {
	if(button == GLUT_LEFT_BUTTON) {
	  //continue on starting a new polyline segment
	  makingPolyline = true;
	}
	else //segment finished
	  makingPolyline = false;
      }
    }
    DoMouse(x,y);
    Recompute();
    Refresh();
  }

  virtual void Handle_Idle() {
    if(showTrace) {
      traceCount = (traceCount+1)%xtrace.size();
      Refresh();
      SleepIdleCallback((int)(1000*problem.timeStep));
    }
    else
      SleepIdleCallback();
  }

  void Recompute() {
    if(!doSolve) return;
    Timer timer;
    if(mode == ModeEditTerrain) {
      problem.SolveAllCurvatureCPs(1e-2,pathCurvatures,capturePoints);
    }
    else if(mode == ModeCapturePoint) {
      validSolution = problem.SolveCurvatureFromCP(problem.capturePoint,problem.capturePointNormal);
    }
    else { //path curvature => capture point
      validSolution = problem.SolveCPFromCurvature(problem.pathCurvature);
    }
    recomputeTime = timer.ElapsedTime();
    if(showTrace) {
      problem.SimulationTrace(problem.timeStep,1000,xtrace,vtrace);
      SleepIdleCallback(0);
      if(xtrace.size()==0) showTrace=false;
      else
	traceCount = traceCount % xtrace.size();
    }
  }
};

int main(int argc,char** argv)
{
  CapturePointGUI gui;
  return gui.Run("Capture Point GUI");
}
