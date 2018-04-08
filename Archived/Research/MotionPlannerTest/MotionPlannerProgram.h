#ifndef MOTION_PLANNER_PROGRAM_H
#define MOTION_PLANNER_PROGRAM_H

#include <KrisLibrary/GLdraw/GLUTProgram.h>
#include <KrisLibrary/GLdraw/drawextra.h>
#include <KrisLibrary/GLdraw/GL.h>
#include <KrisLibrary/GLdraw/GLColor.h>
#include <GL/glut.h>
#include <KrisLibrary/planning/MotionPlanner.h>
#include "DrawableSpace.h"
#include "Callbacks.h"
using namespace std;
using namespace GLDraw;

#define EUCLIDEAN_SPACE 1


struct MotionPlannerProgram : public GLUTProgramBase
{
  DrawableCSpace cspace;

  MotionPlannerProgram()
    :GLUTProgramBase(600,600)
  {
    //TODO: the main program, or the child program must setup the cspace
  }

  void SetupDisplay(Real xmin=0,Real xmax=1,Real ymin=0,Real ymax=1) {
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
    glScalef(width/(xmax-xmin),height/(ymax-ymin),One);
    glTranslatef(-xmin,-ymin,0);
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

#endif
