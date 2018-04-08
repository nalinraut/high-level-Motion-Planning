#include "gcp.h"
#include <stdlib.h>
#include <stdio.h>
#include "Timer.h"

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

void MakeRidgedTerrain(PolygonalTerrain& terrain,double x0,double x1,double y0,double slope,int numdivs)
{
  terrain.segments.resize(numdivs);
  for(int i=0;i<numdivs;i++) {
    Real u0 = Real(i)/Real(numdivs);
    Real u1 = Real(i+1)/Real(numdivs);
    terrain.segments[i].a.x = x0 + u0*(x1-x0);
    terrain.segments[i].a.y = y0;
    terrain.segments[i].b.x = x0 + u1*(x1-x0);
    terrain.segments[i].b.y = y0;
  }
  for(int i=0;i<numdivs;i+=2) {
    terrain.segments[i].b.y = slope*(x1-x0)/numdivs;
    terrain.segments[i+1].a.y = slope*(x1-x0)/numdivs;
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

void RunTests(Problem& problem,const char* name)
{
  //solve CP from curvature (should match true linear CP)
  int n=10;
  Timer timer;
  bool res;
  for(int i=0;i<n;i++)
    res = problem.SolveCPFromCurvature(0);
  double t = timer.ElapsedTime()/n;
  printf("%s: 0 curvature test, time %g, result %d, capture point %g %g (normal %g %g)\n",name,t,(int)res,problem.capturePoint.x,problem.capturePoint.y,problem.capturePointNormal.x,problem.capturePointNormal.y);

  //solve curvature from CP (should be 0)
  timer.Reset();
  for(int i=0;i<n;i++)
    res = problem.SolveCurvatureFromCP(problem.capturePoint,Vector2(0,1));
  t = timer.ElapsedTime()/n;
  printf("%s: curvature computation test time %g, result %d, curvature %g\n",name,t,(int)res,problem.pathCurvature);

  //solve curvature,CP pairs
  timer.Reset();
  std::vector<Real> pathCurvatures;
  std::vector<Vector2> capturePoints;
  for(int i=0;i<n;i++)
    problem.SolveAllCurvatureCPs(1e-2,pathCurvatures,capturePoints);
  t = timer.ElapsedTime()/n;
  printf("%s: All results test time %g, %d solutions\n",name,t,(int)pathCurvatures.size());
  /*
  for(size_t i=0;i<pathCurvatures.size();i++) {
    printf("  Curvature %g point %g %g\n",pathCurvatures[i],capturePoints[i].x,capturePoints[i].y);
  }
  */
}

void test()
{
  //set up the initial conditions
  Real h = 1.0;
  //Real vx0 = 0.5;
  Real vx0 = 0.25;
  Problem problem(h,vx0);
  problem.SetFriction(0.5);
  problem.Lmin = 0.33;
  problem.Lmax = 2;

  //set up the terrain
  int ndivs = 1;
  MakeFlatTerrain(problem.terrain,-10,10,0,ndivs);
  //Assert(problem.terrain.IsSorted());
  RunTests(problem,"Flat ground, divs 1, vx0 = 0.5");
  ndivs = 10;
  MakeFlatTerrain(problem.terrain,-10,10,0,ndivs);
  RunTests(problem,"Flat ground, divs 10, vx0 = 0.5");
  ndivs = 100;
  MakeFlatTerrain(problem.terrain,-10,10,0,ndivs);
  RunTests(problem,"Flat ground, divs 100, vx0 = 0.5");
  ndivs = 1000;
  MakeFlatTerrain(problem.terrain,-10,10,0,ndivs);
  RunTests(problem,"Flat ground, divs 1000, vx0 = 0.5");
  ndivs = 10000;
  MakeFlatTerrain(problem.terrain,-10,10,0,ndivs);
  RunTests(problem,"Flat ground, divs 10000, vx0 = 0.5");

  //set up the terrain
  ndivs = 1;
  MakeRidgedTerrain(problem.terrain,-10,10,0,0.5,ndivs);
  //Assert(problem.terrain.IsSorted());
  RunTests(problem,"Ridged ground, divs 1, vx0 = 0.5");
  ndivs = 10;
  MakeRidgedTerrain(problem.terrain,-10,10,0,0.5,ndivs);
  RunTests(problem,"Ridged ground, divs 10, vx0 = 0.5");
  ndivs = 100;
  MakeRidgedTerrain(problem.terrain,-10,10,0,0.5,ndivs);
  RunTests(problem,"Ridged ground, divs 100, vx0 = 0.5");
  ndivs = 1000;
  MakeRidgedTerrain(problem.terrain,-10,10,0,0.5,ndivs);
  RunTests(problem,"Ridged ground, divs 1000, vx0 = 0.5");
  ndivs = 10000;
  MakeRidgedTerrain(problem.terrain,-10,10,0,0.5,ndivs);
  RunTests(problem,"Ridged ground, divs 10000, vx0 = 0.5");


  int nsteps = 7;
  MakeStaircaseTerrain(problem.terrain,0,1,0,0.5,nsteps);
  RunTests(problem,"Stairs, 7 steps, vx0 = 0.5");
  nsteps = 14;
  MakeStaircaseTerrain(problem.terrain,0,2,0,1,nsteps);
  RunTests(problem,"Stairs, 14 steps, vx0 = 0.5");
  nsteps = 70;
  MakeStaircaseTerrain(problem.terrain,0,10,0,5,nsteps);
  RunTests(problem,"Stairs, 70 steps, vx0 = 0.5");

  nsteps = 7;
  MakeStaircaseTerrain(problem.terrain,0,1,0,-0.5,nsteps);
  RunTests(problem,"Stairs down, 7 steps, vx0 = 0.5");
}


int main(int argc,char** argv)
{
  test();
  return 0;
}
