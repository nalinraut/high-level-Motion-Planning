#include <GLdraw/GLUTProgram.h>
#include <GLdraw/drawextra.h>
#include <GLdraw/GL.h>
#include <GLdraw/GLColor.h>
#include <stdio.h>
#include <GLdraw/GLUTString.h>
#include <GL/glut.h>
#include <utils/stringutils.h>
#include <math/SVDecomposition.h>
#include <Timer.h>
#include "FreeformTaskPredictor.h"
#include <fstream>
#include <sstream>
using namespace std;
using namespace GLDraw;
using namespace Math;



struct MousePredictionProgram : public GLUTProgramBase
{
  FreeformTaskPredictor predictor;
  bool clicked;
  Timer clickTimer;
  bool predDirty;
  int sleeptime;

  int predictionHorizon;

  MousePredictionProgram()
    :GLUTProgramBase(800,600)
  {
    clicked = false;
    predictionHorizon = 0;
    predDirty = true;
  }

  bool LoadConfig(const char* fn)
  {
    if(!predictor.Init(fn)) return false;
    predictor.Reset(0,0);

    sleeptime = INT_MAX;
    for(size_t i=0;i<predictor.filters.size();i++)
      if(predictor.filterInfo[i].timestep!=0) 
	sleeptime = Min(sleeptime,int(predictor.filterInfo[i].timestep*1000));
    return true;
  }

  virtual bool Initialize()
  {
    GLUTProgramBase::Initialize();
    SleepIdleCallback();
    return true;
  }

  void SetupDisplay()
  {
    glClearColor(1,1,1,1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glEnable(GL_POINT_SMOOTH);

    //setup grid -> screen camera frame
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0,width,0,height,-10,10);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    //glScalef(width,height,One);
  }

  void DrawCovariance(int i,const Vector& mean,const Matrix& cov,Real p,bool mousePred)
  {
    const static GLColor colors[8]={GLColor(0,0,0),GLColor(1,0,0),GLColor(0,1,0),GLColor(0,0,1),GLColor(0.5,0.5,0.5),GLColor(0.5,0.5,0),GLColor(0.5,0,0.5),GLColor(0,0.5,0.5)};
    const static GLColor white(1,1,1);
    GLColor col=colors[i%8];
    col.setCurrentGL();
    if(mousePred)
      col.blend(col,colors[0],0.5);

    glPointSize(5.0);
    glBegin(GL_POINTS);
    glVertex2d(mean(0),height-mean(1));
    glEnd();

    RobustSVD<Real> svd;  //can't use pre/postcondition, the singular values aren't the same
    svd.preMultiply = svd.postMultiply = false;
    if(!svd.set(cov)) return;
    Matrix R;
    Vector c;
    R.resize(cov.m,cov.m,Zero);
    c.resize(cov.m,Zero);
    for(int k=0;k<cov.m;k++) {
      int index;
      c[k] = svd.svd.W.maxAbsElement(&index);
      assert(c[k] >= 0.0);
      Vector temp;
      svd.svd.U.getColRef(index,temp);
      R.copyCol(k,temp);

      svd.svd.W(index) = 0;
    }
    Matrix4 m(Zero);
    for(int i=0;i<2;i++) {
      for(int j=0;j<2;j++)
	m(i,j) = R(i,j)*Sqrt(c(j));
      m(3,3) = 1.0;
      m(i,3) = mean[i];
    }
    m(1,0) = -m(1,0);
    m(1,1) = -m(1,1);
    m(0,3) = m(0,3);
    m(1,3) = height -  m(1,3);
    m(3,3)=1;
    if(m.determinant() < 0) {
      //swap cols 0 and 1
      for(int i=0;i<3;i++)
	std::swap(m(i,0),m(i,1));
      assert(m.determinant() > 0);
    }
    glPushMatrix();
    glMultMatrix(m);
    if(mousePred) {
      glLineWidth(3.0);
      drawWireCircle2D(Vector2(0.0,0.0),1,32);
    }
    else {
      col.rgba[3] = p;
      col.setCurrentGL();
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
      drawCircle2D(Vector2(0.0,0.0),1,32);
      glDisable(GL_BLEND);
    }
    glPopMatrix();
  }

  void DrawDistribution(int i,const GaussianMixtureModelRaw& gmm,Real p,bool mousePred)
  {
    for(size_t j=0;j<gmm.means.size();j++) {
      DrawCovariance(i,gmm.means[j],gmm.covariances[j],p*gmm.phi[j],mousePred);
    }
  }

  virtual void Handle_Display() {
    SetupDisplay();
    glColor3f(0,0,0);
    /*
    if(clicked) {
      glPushMatrix();
      glTranslatef(lastx,height-lasty,0);
      drawWireCircle2D(Vector2(0.0,0.0),5.0,32);
      //drawWireCircle2D(Vector2(lastx,lasty),5.0,32);
      glPopMatrix();
    }
    */

    if(predDirty) {
      predictor.UpdatePredictions(predictionHorizon);
      predDirty = false;
    }

    int x=20,y=20;
    int h=15;
    for(size_t i=0;i<predictor.predGoal.size();i++) {
      DrawDistribution(i,predictor.predGoal[i],predictor.pfilter[i]*0.5,false);

      //draw trace of prediction
      glColor3f(1,0.5,0);
      glBegin(GL_LINE_STRIP);
      for(size_t j=0;j<predictor.predGoalTrace[i].size();j++) {
	Vector2 v = predictor.predGoalTrace[i][j];
	v.y = height - v.y;
	glVertex2v(v);
      }
      glEnd();

      glColor3f(0,0,0);
      char buf[256];
      sprintf(buf,"%s: %g",predictor.filterInfo[i].name.c_str(),predictor.pfilter[i]);
      glRasterPos2i(x,height-y);
      glutBitmapString(GLUT_BITMAP_HELVETICA_12,buf);
      y += h;
    }
    for(size_t i=0;i<predictor.predMouse.size();i++) {
      DrawDistribution(i,predictor.predMouse[i],1,true);
    }
    y += h;
    glColor3f(0,0,0);
    char buf[256];
    for(size_t i=0;i<predictor.filters.size();i++) {
      Vector mean;
      Matrix cov;
      predictor.predMouse[i].GetMean(mean);
      predictor.predMouse[i].GetCovariance(cov);
      for(size_t j=0;j<predictor.filterInfo[i].observationNames.size();j++) {
	sprintf(buf,"%s: %g (%g)\n",predictor.filterInfo[i].observationNames[j].c_str(),mean(j),Sqrt(cov(j,j)));
	glRasterPos2i(x,height-y);
	glutBitmapString(GLUT_BITMAP_HELVETICA_12,buf);
	y += h;
      }

      GaussianMixtureModelRaw gmm;
      predictor.filters[i]->GetPrediction(predictionHorizon,gmm);
      Vector homean,hmean;
      Matrix hocov,hcov;
      gmm.GetMean(homean);
      gmm.GetCovariance(hocov);
      hmean.setRef(homean,0,1,(int)predictor.filterInfo[i].hiddenVars.size());
      hcov.setRef(hocov,0,0,1,1,(int)predictor.filterInfo[i].hiddenVars.size(),(int)predictor.filterInfo[i].hiddenVars.size());

      for(int j=0;j<hmean.n;j++) {
	int k=predictor.filterInfo[i].hiddenVars[j];
	hmean(j) = hmean(j)*predictor.filterInfo[i].scale[k]+predictor.filterInfo[i].offset[k];
	hcov(j,j) = hcov(j,j)*Sqr(predictor.filterInfo[i].scale[k]);
      }
      for(size_t j=0;j<predictor.filterInfo[i].hiddenNames.size();j++) {
	sprintf(buf,"%s: %g (%g)\n",predictor.filterInfo[i].hiddenNames[j].c_str(),hmean(j),Sqrt(hcov(j,j)));
	glRasterPos2i(x,height-y);
	glutBitmapString(GLUT_BITMAP_HELVETICA_12,buf);
	y += h;
      }
    }
    glutSwapBuffers();
  }

  virtual void Handle_Reshape(int w,int h) {
    GLUTProgramBase::Handle_Reshape(w,h);
    glViewport(0,0,(GLsizei)width,(GLsizei)height);
    Refresh();
  }

  virtual void Handle_Keypress(unsigned char key,int x,int y) {
    if(key == '=' || key == '+') {
      predictionHorizon ++;
      printf("Prediction horizon: %d\n",predictionHorizon);
      Refresh();
    }
    else if(key == '-' || key == '_') {
      predictionHorizon --;
      if(predictionHorizon < 0) predictionHorizon=0;
      printf("Prediction horizon: %d\n",predictionHorizon);
      Refresh();
    }
    else if(key == 'r') {
      predictor.RunReachTrials("Old/processed_data/reach_test.csv","Old/processed_data/reach_test_predict.csv");
    }
    else if(key == 't') {
      predictor.RunTrajTrials("Old/processed_data/traj_test.csv","Old/processed_data/traj_test_predict.csv");
    }
  }

  virtual void Handle_Click(int button,int state,int x,int y) {
    if(state == GLUT_DOWN && button == GLUT_LEFT_BUTTON) {
      predictor.Reset(x,y);
      printf("%d\n",predictor.predGoal.size());
      predDirty = true;
      clicked=true;
      clickTimer.Reset();

      SleepIdleCallback(sleeptime);
    }
    else if(state == GLUT_UP) {
      clicked = false;
      SleepIdleCallback();
    }
  }

  virtual void Handle_Drag(int x,int y) {
    if(clicked) {
      Real t = clickTimer.ElapsedTime();
      if(predictor.MouseInput(t,x,y))
	predDirty = true;
      Refresh();
    }
  }

  void Handle_Idle() {
    if(clicked) {
      bool updated=false;
      Real t = clickTimer.ElapsedTime();
      if(predictor.IdleInput(t)) {
	predDirty = true;
	updated = true;
      }
      if(updated) Refresh();
      SleepIdleCallback(sleeptime);
    }
    else
      SleepIdleCallback();
  }    
};


bool DoUnivariateGMR(const string& fn)
{
  GaussianMixtureRegression gmr;
  ifstream in(fn.c_str(),ios::in);
  if(!in) {
    printf("Unable to open gmm file %s\n",fn.c_str());
    return false;
  }
  in >> gmr.joint;
  in.close();

  Vector xymean,ymean;
  Matrix xycov,ycov;
  gmr.joint.GetMean(xymean);
  gmr.joint.GetCovariance(xycov);

  GaussianMixtureModelRaw ygmm;
  vector<int> xindices(1,0);
  gmr.SetXIndices(xindices);
  int numdivs=1000;
  printf("x,ymean,ystd,mean+std,mean-std\n");
  for(int i=0;i<numdivs;i++) {
    Vector x(1);
    x(0) = xymean(0) + (Real(i)/Real(numdivs)-0.5)*8.0*xycov(0,0);
    gmr.GetY(x,ygmm);
    ygmm.GetMean(ymean);
    ygmm.GetCovariance(ycov);
    printf("%g,%g,%g,%g,%g\n",x(0),ymean(0),Sqrt(ycov(0,0)),ymean(0)+Sqrt(ycov(0,0)),ymean(0)-Sqrt(ycov(0,0)));
  }
  return true;
}

int main(int argc,char** argv)
{
  if(argc <= 1) {
    printf("Must specify a setup files\n");
    return 1;
  }
  /* TEMP: just a test
  DoUnivariateGMR(argv[1]);
  return 0;
  */
  MousePredictionProgram program;
  for(int i=1;i<argc;i++) {
    if(!program.LoadConfig(argv[i])) exit(-1);
  }
  return program.Run("Mouse Prediction Test");
}
