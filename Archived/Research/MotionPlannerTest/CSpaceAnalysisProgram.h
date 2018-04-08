#include <KrisLibrary/planning/CSpaceAnalysis.h>
#include <KrisLibrary/planning/MotionPlanner.h>
#include "MotionPlannerProgram.h"
#include "Clusterize.h"
using namespace std;


//defined in planning/CSpaceAnalysis.cpp
void KNearestNeighbors(const Config& x,const vector<Config>& pts,int k,CSpace* cspace,vector<int>& nn,vector<Real>& dist);
void KNearestNeighbors(int query_pt,const vector<Config>& pts,int k,CSpace* cspace,vector<int>& nn,vector<Real>& dist);
void KGoodNeighbors(int query_pt,const vector<Config>& pts,int k,CSpace* cspace,vector<int>& neighbors,vector<Real>& dist,Real factor);

void KNearestNeighborGraph(const vector<Config>& pts,int k,CSpace* cspace,vector<vector<int> >& neighbors)
{
  vector<Real> dist;
  neighbors.resize(pts.size());
  for(size_t i=0;i<pts.size();i++) {
    KNearestNeighbors(i,pts,k,cspace,neighbors[i],dist);
  }
}

void KGoodNeighborGraph(const vector<Config>& pts,int k,CSpace* cspace,vector<vector<int> >& neighbors,Real factor)
{
  vector<Real> dist;
  neighbors.resize(pts.size());
  for(size_t i=0;i<pts.size();i++) {
    KGoodNeighbors(i,pts,k,cspace,neighbors[i],dist,factor);
  }
}

//functor that returns true if two configurations are within distance d
struct InNeighborhood
{
  InNeighborhood(CSpace* _space,Real _d) : space(_space),d(_d) {}
  bool operator () (const Config& a,const Config& b) const
  {
    Assert(a.n == 2 && b.n == 2);
    return space->Distance(a,b) < d;
  }

  CSpace* space;
  Real d;
};


struct CSpaceAnalysisProgram : public MotionPlannerProgram
{
  CSpaceAnalysis analysis;
  vector<Vector2> graph;

  CSpaceAnalysisProgram()
  {
  }

  virtual void Handle_Display() {
    SetupDisplay();
    DrawCSpace();

    //draw the neighbors
    vector<vector<int> > neighbors;
    KGoodNeighborGraph(analysis.space.points,4,&cspace,neighbors,0.7);
    //KNearestNeighborGraph(analysis.space.points,8,&cspace,neighbors);
    glColor3f(0,0,0);
    glBegin(GL_LINES);
    Assert(neighbors.size() == analysis.space.points.size());
    for(size_t i=0;i<neighbors.size();i++) {
      for(size_t j=0;j<neighbors[i].size();j++) {
	Assert(neighbors[i][j] < (int)analysis.space.points.size());
	if(neighbors[i][j] < 0) continue;
	glVertex2v(analysis.space.points[i]);
	glVertex2v(analysis.space.points[neighbors[i][j]]);
      }
    }
    glEnd();

    //draw the points
    glColor3f(1,1,0);
    glPointSize(3.0);
    glBegin(GL_POINTS);
    glEnd();
    glPointSize(3.0);
    glBegin(GL_POINTS);
    glColor3f(0,1,0);
    for(size_t i=0;i<analysis.feasible.points.size();i++) {
      //Real c=analysis.feasibleEigenvector[i]*2.0+0.5;
      Real c=(analysis.feasibleEigenvector[i] > 0? 1:0);
      glColor3f(0,1-c,c);
      glVertex2v(analysis.feasible.points[i]);
    }
    glColor3f(1,0,0);
    for(size_t i=0;i<analysis.infeasible.points.size();i++)
      glVertex2v(analysis.infeasible.points[i]);
    glEnd();

    vector<Config> temp = analysis.feasible.points;
    InNeighborhood eq(&cspace,0.04);
    Clusterize(temp,eq);
    glPointSize(5.0);
    glColor3f(1,0.5,0);
    glBegin(GL_POINTS);
    for(size_t i=0;i<temp.size();i++) {
      glVertex2v(temp[i]);
    }
    glEnd();

    //draw the graph
    glTranslatef(0.1,0.1,0);
    glScalef(0.2,0.2,1);
    glColor3f(1,1,1);
    glBegin(GL_QUADS);
    glVertex2f(-0.1,-0.1);
    glVertex2f(1.1,-0.1);
    glVertex2f(1.1,1.1);
    glVertex2f(-0.1,1.1);
    glEnd();
    glColor3f(0,0,0);
    glBegin(GL_LINES);
    glVertex2f(0,0);
    glVertex2f(1,0);
    glVertex2f(0,0);
    glVertex2f(0,1);
    glEnd();
    glScalef(5.0,0.3,1);
    glColor3f(0,0.5,0);
    glBegin(GL_LINE_STRIP);
    for(size_t i=0;i<graph.size();i++)
      glVertex2dv(graph[i]);
    glEnd();
    glColor3f(0,0,0);
    glBegin(GL_LINES);  //tick mark at 2
    glVertex2f(-0.005,2);
    glVertex2f(0.005,2);
    glEnd();

    glutSwapBuffers();
  }

  virtual void Handle_Click(int button,int state,int x,int y) {
    if(state == GLUT_DOWN) {
      Config q;
      ClickToConfig(x,y,q);
      int numSamples = 200;
      analysis.AnalyzeNeighborhood(&cspace,q,0.1,numSamples);
      graph.clear();
      //20 = minimum number of points needed to get an accurate reading
      for(size_t i=20;i<analysis.feasible.localIntrinsicDims.size();i++) {
	graph.push_back(Vector2(analysis.feasible.localScales[i],analysis.feasible.localIntrinsicDims[i]));
      }
    }
  }
};
