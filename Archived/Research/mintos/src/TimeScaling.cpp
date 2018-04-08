/*****************************************************************************
 *
 * Copyright (c) 2013, the Trustees of Indiana University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Indiana University nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE TRUSTEES OF INDIANA UNIVERSITY ''AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE TRUSTEES OF INDIANA UNIVERSITY BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 * 
 ***************************************************************************/

#include <mintos/TimeScaling.h>
#include <mintos/ContactTimeScaling.h>
#include "../Timer.h"
#include "misc.h"
#include "primitives.h"
#include "Plane2D.h"
#include "Segment2D.h"
#include "LinearProgram.h"
#include "PolytopeProjection.h"
#include "GLPKInterface.h"
#include <mintos/misc/errors.h>
#include <mintos/Config.h>
#include <fstream>
using namespace Mintos;
using namespace Math;
using namespace Math3D;
using namespace Optimization;

//use the new polynomial bounding technique.  Turning this to 0 uses the old
//interval bounding technique, which typically produces looser bounds but
//runs in less time.
#define POLYNOMIAL_DERIV_BOUNDS 0

//if POLYNOMIAL_DERIV_BOUNDS is 0, this should be set to
//1 if the time scaling should use interval analysis to get the derivative
//bounds.  If 0, it will just use collocation
#define INTERVAL_DERIV_BOUNDS 1

//initialize the solver with a better initial state produced via backwards propagation
#define BACKWARDS_PROPAGATION 1

//debug the bounds after solving for the time scaling
#define CHECK_SLP_BOUNDS 0

//debug
#define SAVE_COLLOCATION_POINTS 0

//debug -- testing performance without irrelevant constraint pruning
#define TEST_NO_CONSTRAINT_PRUNING 0

//For a given convex polygon poly given as a set of CCW vertices, finds ind1 and ind2
//such that ind1 and ind1+1 are on <= 0 and > 0 sides of the plane, while ind2 and
//ind2+1 are on >= 0 and < 0 sides of the plane
int PolygonCrossings(const vector<Vector2>& poly,const Plane2D& p,int& ind1,int& ind2)
{
  //brute force method
  Real lastd = p.distance(poly[0]);
  int count=0;
  int last = 0;
  for(size_t i=1;i<poly.size();i++) {
    Real d = p.distance(poly[i]);
    if(lastd <= 0 && d > 0) {
      ind1 = last;
      count++;
    }
    if(lastd >= 0 && d < 0) {
      ind2 = last;
      count++;
    }
    lastd = d;
    last = (int)i;
  }
  Real d = p.distance(poly[0]);
  if(lastd <= 0 && d > 0) {
    ind1 = last;
    count++;
  }
  if(lastd >= 0 && d < 0) {
    ind2 = last;
    count++;
  }
  /*
  if(count > 2) {
    cout<<"Strange number of polygon crossings??? "<<count<<endl;
    cout<<"Plane "<<p.normal<<", "<<p.offset<<endl;
    cout<<"Poly: "<<endl;
    for(size_t i=0;i<poly.size();i++) {
      cout<<"  "<<poly[i]<<endl;
    }
  }
  Assert(count <= 2);
  */
  return count;
}

//for constraints l<=x<=u and a*x <= b, returns the set of active constraints,
//refits the bounds
bool GetActiveBounds(Real& l,Real& u,const Vector& a,const Vector& b,vector<bool>& active)
{
#if TEST_NO_CONSTRAINT_PRUNING
  fill(active.begin(),active.end(),true);
  return true;
#endif //TEST_NO_CONSTRAINT_PRUNING
  int top = -1, bottom = -1;
  for(int i=0;i<a.n;i++) {
    if(a[i] > 0) {
      if(a[i]*u > b[i]) {
	top = i;
	u = b[i]/a[i];
      }
    }
    else if(a[i] < 0) {
      if(a[i]*l > b[i]) {
	bottom = i;
	l = b[i]/a[i];
      }
    }
    else {
      if(b[i] < 0) return false;
    }
    if(l > u) return false;
  }
  fill(active.begin(),active.end(),false);
  if(top >= 0) active[top] = true;
  if(bottom >= 0) active[bottom] = true;
  return true;
}

//for constraints l <= x,y <= u, ax*x + ay*y <= b, returns the set of active
//constraints, refits the bounds
bool GetActiveBounds(Vector2& l,Vector2& u,const Vector& ax,const Vector& ay,const Vector& b,vector<bool>& active)
{
#if TEST_NO_CONSTRAINT_PRUNING
  fill(active.begin(),active.end(),true);
  return true;
#endif //TEST_NO_CONSTRAINT_PRUNING

  if(l.x == u.x) {
    //do 1-D fitting -- this is more robust
    Vector b_xax(ax.n);
    for(int i=0;i<ax.n;i++)
      b_xax[i] = b[i] - l.x*ax[i];
    return GetActiveBounds(l.y,u.y,ay,b_xax,active);
  }
  else if(l.y == u.y) {
    //do 1-D fitting -- this is more robust
    Vector b_yay(ay.n);
    for(int i=0;i<ay.n;i++)
      b_yay[i] = b[i] - l.y*ay[i];
    return GetActiveBounds(l.x,u.x,ax,b_yay,active);
  }
  vector<Vector2> pts(4);
  pts[0] = l;
  pts[1].set(u.x,l.y);
  pts[2] = u;
  pts[3].set(l.x,u.y);
  vector<int> ids(4,-1);
  for(int i=0;i<ax.n;i++) {
    //slice the polygon with ax(i)*x + ay(i)*y <= b
    Plane2D pi;
    pi.normal.set(ax(i),ay(i));
    pi.offset = b(i);
    int ind1,ind2;
    int n=PolygonCrossings(pts,pi,ind1,ind2);
    if(n == 0) {
      if(pi.distance(pts[0]) < 0) { //completely feasible, do nothing
	continue;
      }
      else {
	return false;
      }
    }
    else if(n==1) { //hit a point -- let's assume that 
      continue;
    }
    if(n > 2) {
      /*
      //DEBUG
      cout<<"Strange number of polygon crossings??? "<<n<<endl;
      cout<<"Plane "<<pi.normal<<", "<<pi.offset<<endl;
      cout<<"Poly: "<<endl;
      for(size_t i=0;i<pts.size();i++) {
	cout<<"  "<<pts[i]<<endl;
      }
      */
      continue;
    }
    Assert(n==2);
    //chop the crossings at ind1 and ind2, replace it with the constraints
    int next1=(ind1+1)%pts.size();
    int next2=(ind2+1)%pts.size();
    Segment2D s; 
    s.a=pts[ind1]; s.b=pts[next1];
    Real u;
    pi.intersectsSegment(s,&u);
    Vector2 newpt1 = s.a + u*(s.b-s.a);
    s.a=pts[ind2]; s.b=pts[next2];
    pi.intersectsSegment(s,&u);
    Vector2 newpt2 = s.a + u*(s.b-s.a);
    pts[next1] = newpt1;
    if(next1 == ind2) {
      //corner
      pts.insert(pts.begin()+next1+1,newpt2);
      ids.insert(ids.begin()+next1,i);
    }
    else {
      ids[next1] = i;
      pts[ind2] = newpt2;
      if(next1 < ind2) {
	pts.erase(pts.begin()+next1+1,pts.begin()+ind2);
	ids.erase(ids.begin()+next1+1,ids.begin()+ind2);
      }
      else {
	pts.erase(pts.begin()+next1+1,pts.end());
	pts.erase(pts.begin(),pts.begin()+ind2);
	ids.erase(ids.begin()+next1+1,ids.end());
	ids.erase(ids.begin(),ids.begin()+ind2);
      }
    }
  }
  fill(active.begin(),active.end(),false);
  l = u = pts[0];
  for(size_t i=0;i<pts.size();i++) {
    if(ids[i] >= 0)
      active[ids[i]]=true;
    if(pts[i].x < l.x) l.x = pts[i].x;
    else if(pts[i].x > u.x) u.x = pts[i].x;
    if(pts[i].y < l.y) l.y = pts[i].y;
    else if(pts[i].y > u.y) u.y = pts[i].y;
  }
  return true;
}


/** @brief Defines a sequential linear program (SLP) for minimizing time over
 * squared-rate variables x[i] = ds/dt(si)^2.
 *
 * Note: in the current implementation, AddVel2Bound[s] must be called
 * sequentially for each segment i. 
 *
 * Rather than adding many bounds to a segment i via multiple calls to
 * AddVel2Bound, a single call to AddVel2Bounds is preferred because it
 * internally eliminates redunant constraints, which can have a significant
 * effect on running time.
 */
class TimeScalingSLP
{
public:
  TimeScalingSLP(const vector<Real>& paramdivs);
  //sets ds[i] = vi, i.e., x[i] = vi^2
  void SetFixed(int i,Real vi);
  //sets velocity bound i to ds[i] <= vimax, i.e., x[i] <= vimax^2
  void SetVelBound(int i,Real vimax);
  //adds bounds Ai*x[i] + An*x[i+1] <= bi, returns false if infeasible
  bool AddVel2Bounds(int i,const Vector& Ai,const Vector& An,const Vector& bi);
  bool AddVel2Bound(int i,Real Ai,Real An,Real bi);

  //call this to solve the problem
  bool Solve(int& maxIters,Real xtol=1e-5,Real ftol=1e-7);
  const vector<Real>& GetVelocities() const { return ds; }

  //helpers
  //setup initial status
  void InitializeInitPoint();
  void InitializeGLPK();
  //sets ds and computes T
  void ComputeObjective(const Config& x);
  //after ComputeObjective, computes the gradient and puts it in lp.c
  void ComputeGradient();

  //debugging
  void CheckSolution();

  const vector<Real>& paramdivs;
  LinearProgram_Sparse lp;
  vector<pair<int,int> > segToConstraints;
  //filled out after Solve
  GLPKInterface glpk;
  Vector x;
  vector<Real> ds;
  Real T;
};

///Given a grid and a list of constraint normals and offsets in the ds2-dds
///plane, solves for the time scaling of the given trajectory (traj.path
///is assumed given, and the remaining members are filled in.)
bool SolveSLP(const vector<Real>& paramDivs,
	      const vector<Real>& dsmaxs,
	      const vector<vector<Vector2> >& ds2ddsConstraintNormals,
	      const vector<vector<Real> >& ds2ddsConstraintOffsets,
	      TimeScaledBezierCurve& traj)
{
  Assert(ds2ddsConstraintNormals.size()==paramDivs.size());
  Assert(ds2ddsConstraintOffsets.size()==paramDivs.size());
  TimeScalingSLP slp(paramDivs);
  for(size_t i=0;i<dsmaxs.size();i++) 
    slp.SetVelBound(i,dsmaxs[i]);
  int numTotalConstraints = 0;
  for(size_t i=0;i+1<paramDivs.size();i++) {
    int nc=ds2ddsConstraintNormals[i].size();
    if(i+2 < paramDivs.size())
      nc += ds2ddsConstraintNormals[i+1].size();
    Vector ai(nc),an(nc),b(nc);
    int k=0;
    Real scale = 0.5/(paramDivs[i+1]-paramDivs[i]);
    for(size_t j=0;j<ds2ddsConstraintNormals[i].size();j++,k++) {
      ai(k) = ds2ddsConstraintNormals[i][j].x - ds2ddsConstraintNormals[i][j].y*scale;
      an(k) = ds2ddsConstraintNormals[i][j].y*scale;
      b(k) = ds2ddsConstraintOffsets[i][j];
    }
    if(i+2 < paramDivs.size()) {
      Real scale = 0.5/(paramDivs[i+2]-paramDivs[i+1]);
      for(size_t j=0;j<ds2ddsConstraintNormals[i+1].size();j++,k++) {
	ai(k) = -ds2ddsConstraintNormals[i+1][j].y*scale;
	an(k) = ds2ddsConstraintNormals[i+1][j].x + ds2ddsConstraintNormals[i+1][j].y*scale;
	b(k) = ds2ddsConstraintOffsets[i+1][j];
      }
    }

    numTotalConstraints += nc;
    Assert(k==nc);
    /*
    //TEST: add all bounds, don't do pruning
    for(int j=0;j<nc;j++)
      slp.AddVel2Bound(i,ai[j],an[j],b[j]);
    */
    if(!slp.AddVel2Bounds(i,ai,an,b))
      return false;

    /*
    cout<<i<<" vmax "<<dsmaxs[i]<<endl;
    cout<<ai<<endl;
    cout<<an<<endl;
    cout<<b<<endl;
    cout<<"Total constraints: "<<slp.lp.A.m<<endl;
    getchar();
    */
  }
  printf("Reduced %d constraints to %d\n",numTotalConstraints,slp.lp.A.m);
  //getchar();

  int maxIters = gSLPMaxIters;
  bool res = slp.Solve(maxIters,gSLPXTolerance,gSLPFTolerance);

  if(CHECK_SLP_BOUNDS) {
    slp.CheckSolution();
  }

  if(!res) {
    return false;
  }
  //done, now output the trajectory
  traj.timeScaling.params.resize(paramDivs.size());
  copy(paramDivs.begin(),paramDivs.end(),traj.timeScaling.params.begin());
  traj.timeScaling.times.resize(traj.timeScaling.params.size());
  traj.timeScaling.times[0] = 0;
  traj.timeScaling.ds = slp.GetVelocities();
  for(size_t i=0;i+1<paramDivs.size();i++) {
    Real dt = 2*(paramDivs[i+1]-paramDivs[i])/(traj.timeScaling.ds[i]+traj.timeScaling.ds[i+1]);    
    traj.timeScaling.times[i+1]=traj.timeScaling.times[i]+dt;
  }
  traj.pathSegments.resize(traj.path.durations.size()+1);
  traj.pathSegments[0] = 0;
  for(size_t i=0;i<traj.path.durations.size();i++)
    traj.pathSegments[i+1] = traj.pathSegments[i]+traj.path.durations[i];
  return true;
}

TimeScalingSLP::TimeScalingSLP(const vector<Real>& _paramdivs)
  :paramdivs(_paramdivs)
{
  size_t n=paramdivs.size()-1;
  lp.minimize = true;
  lp.Resize(0,n+1);
  lp.l.setZero();
  segToConstraints.resize(n);
  fill(segToConstraints.begin(),segToConstraints.end(),pair<int,int>(-1,-1));
  ds.resize(n+1,0.0);
  T = Inf;
}

void TimeScalingSLP::CheckSolution()
{
  int n=(int)ds.size()-1;
  vector<bool> anyNonBasic(n+1,false);
  vector<vector<int> > limitingConstraints(n+1);
  for(int i=0;i<=n;i++) {
    if(!glpk.GetVariableBasic(i)) {
      anyNonBasic[i] = true;
      limitingConstraints[i].push_back(-1); 
    }
  }
  for(int i=1;i<=n;i++) {
    int cfirst=segToConstraints[i-1].first;
    int cend=segToConstraints[i-1].second;
    for(int c=cfirst;c<cend;c++)
      if(glpk.GetRowBasic(c) == false) {
	anyNonBasic[i-1] = true;
	anyNonBasic[i] = true;
	limitingConstraints[i-1].push_back(c);
	limitingConstraints[i].push_back(c);
      }
  }
  for(int i=0;i<=n;i++)
    if(!anyNonBasic[i]) printf("Hmm, variable %d not involved in the basis\n",i);
  for(int i=0;i<=n;i++) {
    if(find(limitingConstraints[i].begin(),limitingConstraints[i].end(),-1)!=limitingConstraints[i].end())
      printf("v,");
    else {
      printf("a,");
    }
  }
  printf("\n");
}

void TimeScalingSLP::SetFixed(int i,Real vi)
{
  Assert(i <= (int)segToConstraints.size());
  lp.u(i) = lp.l(i) = Sqr(vi);
}

void TimeScalingSLP::SetVelBound(int i,Real vimax)
{
  Assert(i <= (int)segToConstraints.size());
  lp.u(i) = Sqr(vimax);
}

bool TimeScalingSLP::AddVel2Bounds(int i,const Vector& Ai,const Vector& An,const Vector& bi)
{
  /*
  //TEMP: don't do pruning
  for(int j=0;j<Ai.n;j++)
    AddVel2Bound(i,Ai(j),An(j),bi(j));
  return true;
  */
  Assert(i < (int)segToConstraints.size());
  if(segToConstraints[i] != pair<int,int>(-1,-1)) {
    fprintf(stderr,"TimeScalingSLP: Error, can't add multiple vel bounds yet\n");
    abort();
  }
  Assert(segToConstraints[i].first==-1);
  Assert(segToConstraints[i].second==-1);
  Assert(Ai.n == An.n);
  Assert(Ai.n == bi.n);
  vector<bool> nonredundant(Ai.n,false);
  Vector2 l(0.0,0.0), u(lp.u(i),lp.u(i+1));
  if(IsInf(lp.u(i)) || IsInf(lp.u(i+1))) {
    static bool warned=false;
    if(!warned) {
      fprintf(stderr,"Warning: can't yet eliminate redundant bounds with infinite velocity bound\n");
      warned = true;
    }
    fill(nonredundant.begin(),nonredundant.end(),true);
  }
  else {
    if(!GetActiveBounds(l,u,Ai,An,bi,nonredundant)) {
      fprintf(stderr,"Infeasible set of linear constraints on segment %d\n",i);
      fprintf(stderr,"0 <= xi <= %g\n",u.x);
      fprintf(stderr,"0 <= xn <= %g\n",u.y);
      for(int j=0;j<Ai.n;j++) {
	if(j==Ai.n/2) 
	  fprintf(stderr,"[%g]*xi +\t[%g]*xn\t<= [%g]\n",Ai[j],An[j],bi[j]);
	else
	  fprintf(stderr,"[%g]     \t[%g]   \t   [%g]\n",Ai[j],An[j],bi[j]);
      }
      return false;
    }
  }

  /*
  //TEMP: test no pruning
  for(int num=0;num<Ai.n;num++)
    nonredundant[num]=true;
  */
  
  int nactive = 0;
  for(int num=0;num<Ai.n;num++) {
    if(nonredundant[num])
      nactive++;
  }

  if(lp.A.m+nactive > (int)lp.A.rows.size()) {
    //double size of constraints matrix
    int mnew = lp.A.m * 2;
    if(mnew < lp.A.m + nactive) mnew = lp.A.m + nactive;
    lp.A.rows.resize(mnew);
    Vector oldp = lp.p, oldq = lp.q;
    lp.p.resize(mnew);
    lp.q.resize(mnew);
    lp.p.set(Inf);
    lp.q.set(-Inf);
    if(!oldp.empty()) {
      lp.p.copySubVector(0,oldp);
      lp.q.copySubVector(0,oldq);
    }
  }
  segToConstraints[i].first=lp.A.m;
  segToConstraints[i].second=lp.A.m+nactive;
  lp.A.m = lp.A.m+nactive;
  lp.p.n = lp.A.m;
  lp.q.n = lp.A.m;

  int k=segToConstraints[i].first;
  for(int num=0;num<Ai.n;num++) {
    if(nonredundant[num]) {
      lp.A(k,i) = Ai(num);
      lp.A(k,i+1) = An(num);
      lp.p(k) = bi(num);
      //cout<<"constraint "<<k<<": "<<Ai(num)<<"*x("<<i<<") + "<<An(num)<<"*x("<<i+1<<") <= "<<bi(num)<<endl;;
      k++;
    }
  }
  return true;
}

bool TimeScalingSLP::AddVel2Bound(int i,Real Ai,Real An,Real bi)
{
  Assert(i < (int)segToConstraints.size());
  if(segToConstraints[i] != pair<int,int>(-1,-1) && segToConstraints[i].second != lp.A.m) {
    fprintf(stderr,"TimeScalingSLP: Error, can't add multiple vel bounds yet\n");
    abort();
  }

  if(lp.A.m+1 > (int)lp.A.rows.size()) {
    //double size of constraints matrix
    int mnew = lp.A.m * 2;
    if(mnew == 0) mnew = 1;
    lp.A.rows.resize(mnew);
    Vector oldp = lp.p, oldq = lp.q;
    lp.p.resize(mnew);
    lp.q.resize(mnew);
    lp.p.set(Inf);
    lp.q.set(-Inf);
    if(!oldp.empty()) {
      lp.p.copySubVector(0,oldp);
      lp.q.copySubVector(0,oldq);
    }
  }
  if(segToConstraints[i].first < 0) {
    segToConstraints[i].first=lp.A.m;
    segToConstraints[i].second=lp.A.m+1;
  }
  else {
    segToConstraints[i].second=lp.A.m+1;
  }
  lp.A.m = lp.A.m+1;
  lp.p.n = lp.A.m;
  lp.q.n = lp.A.m;

  int k=lp.A.m-1;
  lp.A(k,i) = Ai;
  lp.A(k,i+1) = An;
  lp.p(k) = bi;
  return true;
}

bool TimeScalingSLP::Solve(int& maxIters,Real xtol,Real ftol)
{
  Timer timer;
  int n = (int)ds.size()-1;

  InitializeInitPoint();
  //possible for some problems to have no constraints? 
  //GLPK aborts if this happens
  if(lp.A.m == 0) {
    ComputeObjective(x);
    Assert(lp.IsFeasible(x));
    return true;
  }

  bool feasible=lp.IsFeasible(x);
  if(!feasible && gDebugTimeScalingInitialPoint) {
    feasible = true;
    if(!lp.SatisfiesBounds(x)) { if(gTimeScalingVerbose>=2) printf("  Bound error\n"); feasible = false; }
    if(!lp.SatisfiesEqualities(x)) { if(gTimeScalingVerbose>=2) printf("  Equality error\n"); feasible = false; }
    if(!lp.SatisfiesInequalities(x)) {
      for(int i=0;i<lp.A.m;i++) {
	Real d = lp.A.dotRow(i,x);
	if(d > lp.p(i)+Epsilon || d < lp.q(i)) {
	  if(gTimeScalingVerbose>=2) printf("Constraint %d volation: %g <= %g <= %g\n",i,lp.q(i),d,lp.p(i));
	  feasible = false;
	}
      }
      if(gTimeScalingVerbose>=2) {
	if(!feasible)      printf("  Inequality error\n");
      }
    }
  }
  ComputeObjective(x);
  //printf("Initial time: %g\n",T);
  ComputeGradient();
  InitializeGLPK();

  Real trustRegionSize = x.maxAbsElement();
  int numIters = 0;
  while(numIters < maxIters) {
    //set up trust region
    if(feasible) {
      //printf("Setting trust region size %g\n",trustRegionSize);
      for(int i=0;i<x.n;i++) {
	Real lo=Max(lp.l(i),x(i)-trustRegionSize);
	Real hi=Min(lp.u(i),x(i)+trustRegionSize);
	//printf("[%g,%g] %g %g, dir %g\n",lp.l(i),lp.u(i),x(i),trustRegionSize,-lp.c(i));
	//Assert(lo <= hi);
	glpk.SetVariableBounds(i,lo,hi);
      }
    }
    else {
      for(int i=0;i<x.n;i++) {
	glpk.SetVariableBounds(i,Min(1e-10,lp.u(i)),lp.u(i));
      }
    }

    Vector xnext;
    LinearProgram::Result res=glpk.Solve(xnext);
    if(res == LinearProgram::Infeasible) {
      if(gTimeScalingVerbose>=1) fprintf(stderr,"SLP: Warning, got an infeasible linear program???\n");
      maxIters = numIters;
      return false;
    }
    else if(res == LinearProgram::Unbounded) {
      if(gTimeScalingVerbose>=1) fprintf(stderr,"SLP: Warning, got an unbounded linear program???\n");
      maxIters = numIters;
      return false;
    }
    else if(res == LinearProgram::Error) {
      fprintf(stderr,"SLP: Warning, linear program solver failed\n");
      maxIters = numIters;
      return false;
    }
    else {
      //numerical error cleanup?
      for(int i=0;i<=n;i++) {
	if(xnext[i] < -1e-5) {
	  if(gTimeScalingVerbose>=2) printf("SLP: Warning, numerical error in LP solve, value %g\n",xnext[i]);
	}
	Assert(xnext[i] >= -1e-5);
	if(xnext[i] < 0) xnext[i] = 0;
      }
      for(int i=0;i<n;i++) {
	Assert(xnext[i] >= 0);
	Assert(xnext[i+1] >= 0);
      }
      numIters++;
      //cout<<"Solved solution: "<<x<<endl;
      Real Told = T;
      ComputeObjective(xnext);

      //found a feasible solution -- test convergence
      Real xdist = x.distance(xnext);
      if(xdist < xtol) {
	if(T < Told)
	  x = xnext;
	else 
	  ComputeObjective(x);
	maxIters = numIters;
	if(gTimeScalingVerbose>=2) printf("SLP step %d converged on x, dist %g with time %g\n",numIters,xdist,T);
	return true;
      }

      bool changed = false;
      if(feasible) {
	//check if the step reduces the objective value; if so, increase the TR
	if(T <= Told) {
	  if(gTimeScalingVerbose>=2) printf("SLP step %d size %g changed time from %g to %g\n",numIters,trustRegionSize,Told,T);
	  //printf("Distance %g\n",xdist);
	  trustRegionSize *= 1.5;
	}
	else {
	  if(gTimeScalingVerbose>=2) printf("SLP step %d size %g increased time from %g to %g, reducing max step\n",numIters,trustRegionSize,Told,T);
	  //reject step and continue
	  ComputeObjective(x);
	  trustRegionSize *= 0.5;
	  continue;
	}
	changed = true;
      }
      else {
	changed = true;
	if(gTimeScalingVerbose>=2) printf("SLP step %d found feasible solution with time %g\n",numIters,T);
      }

      feasible = true;
      if(!changed || xdist<=xtol) {
	x = xnext;
	maxIters = numIters;
	if(gTimeScalingVerbose>=2) {
	  if(!changed)
	    printf("SLP time change %g below tolerance %g, end time %g\n",Told-T,ftol,T);
	  else
	    printf("SLP state change below tolerance %g, val %g, end time %g\n",xtol,xdist,T);
	}
	return true;
      }
      x = xnext;

      //calc new gradient
      Vector oldDt = lp.c;
      ComputeGradient();
      //cout<<"Change in gradient: "<<oldDt.distance(lp.c)<<endl;
      glpk.SetObjective(lp.c,true);
    }
  }
  if(gTimeScalingVerbose>=2) printf("SLP terminated after %d iterations with total time %g\n",numIters,T);
  return true;
}

void TimeScalingSLP::ComputeGradient()
{
  lp.c.setZero();
  for(size_t i=0;i+1<ds.size();i++) {
    if(ds[i] + ds[i+1] <=0) {
      //infinite objective -- push upward?
      lp.c(i) = -1;
    }
    else {
      if(ds[i]!=0)
	lp.c(i) -= (paramdivs[i+1]-paramdivs[i])/(ds[i]*Sqr(ds[i]+ds[i+1]));
      if(ds[i+1]!=0)
	lp.c(i+1) -= (paramdivs[i+1]-paramdivs[i])/(ds[i+1]*Sqr(ds[i]+ds[i+1]));
    }
  }
  lp.c.inplaceMul(1.0/lp.c.maxAbsElement());
}

void TimeScalingSLP::ComputeObjective(const Vector& x)
{
  //use ds as temporary storage
  int n=(int)ds.size();
  Assert(n==x.n);
  for(int i=0;i<n;i++) {
    Assert(x[i] >= 0);
    ds[i] = Sqrt(x[i]);
  }
  T=0;
  for(int i=0;i+1<n;i++) {
    if(x[i] + x[i+1] <=0) {
      /*
      fprintf(stderr,"Warning: two subsequent x variables are forced to be nonpositive? %d and %d\n",i,i+1);
      fprintf(stderr,"upper bounds %g and %g\n",lp.u(i),lp.u(i+1));
      fprintf(stderr,"Values %g and %g\n",x[i],x[i+1]);
      getchar();
      */
      T = Inf;
      return;
    }
    else {
      Assert(x[i]+x[i+1] > 0);
      Assert(ds[i]+ds[i+1] > 0);
      T += 2*(paramdivs[i+1]-paramdivs[i])/(ds[i]+ds[i+1]);
      //if(IsInf(T))
      //printf("%g %g %g %g\n",paramdivs[i+1],paramdivs[i],ds[i],ds[i+1]);
      //Assert(!IsInf(T));
    }
  }
  //printf("Computed T(x): %g\n",T);
}


void TimeScalingSLP::InitializeInitPoint()
{
  int n=(int)ds.size()-1;
  x=lp.u;
  Real maxNonInf = 0;
  for(int i=0;i<x.n;i++) 
    if(!IsInf(x(i))) maxNonInf = Max(maxNonInf,x(i));
  for(int i=0;i<x.n;i++) 
    if(IsInf(x(i))) x(i) = maxNonInf;

  for(int i=1;i+1<n;i++) { 
    int cfirst=segToConstraints[i-1].first;
    int cend=segToConstraints[i-1].second;
    //Real xorig = x[i];
    for(int c=cfirst;c<cend;c++) {
      //Assert(lp.A.rows[c].numEntries()==2);
      //Assert(lp.A.rows[c].find(i-1) != lp.A.rows[c].end());
      //Assert(lp.A.rows[c].find(i) != lp.A.rows[c].end());
      Real axprev = lp.A.rows[c].entries[i-1]*x(i-1);
      Real q=lp.q(c)-axprev,p=lp.p(c)-axprev;
      Real a=lp.A.rows[c].entries[i];
      //solve q <= a*x <= p
      if(a < 0) {
	if(a*x[i] < q) {
	  x[i] = q/a;
	}
      }
      else {
	if(a*x[i] > p) {
	  x[i] = p/a;
	}
      }
    }
    if(x[i] == 0.0 && i != n) {
      if(gTimeScalingVerbose>=2) printf("x[%d] is set to zero, x[%d]=%g\n",i,i-1,x[i-1]);
      /*
      for(int c=cfirst;c<cend;c++) {
	printf("' %g <= %g*%g + %g*%g <= %g\n",lp.q(c),lp.A(c,i-1),x[i-1],lp.A(c,i),xorig,lp.p(c));
      }
      */
    }
    if(x[i] >= 0 && x[i] <= 1e-5 && i!=n) {
      if(gTimeScalingVerbose>=2) {
	printf("Warning: x[%d] is set to small value %g\n",i,x[i]);
	printf("Warning: x[%d]=%g \n",i-1,x[i-1]);
      }
    }
    if(x[i] < 0.0 ) {
      if(x[i-1] == 0.0 ) {
	if(gTimeScalingVerbose>=2) 
	  fprintf(stderr,"Warning: two subsequent x variables are forced to be negative? %d and %d\n",i-1,i);
	x[i] = lp.u(i);
	if(IsInf(lp.u(i)))
	  x[i] = 1.0;
      }
      else
	x[i] = 0.0; 
      // if x[i] becomes 0, enforce it to be zero, and then propagate backwards
      //printf("Variable x[%d] forced to zero, from x[%d]=%g\n",i,i-1,x[i-1]);
      /*
      for(int c=cfirst;c<cend;c++) {
	printf("' %g <= %g*%g + %g*%g <= %g\n",lp.q(c),lp.A(c,i-1),x[i-1],lp.A(c,i),xorig,lp.p(c));
      }
      */
      if(BACKWARDS_PROPAGATION) {
	int j=i;
	while(j > 0) {
	  //maximize x[j] s.t. the constraints are met
	  int cfirst=segToConstraints[j-1].first;
	  int cend=segToConstraints[j-1].second;
	  Real oldxj = x[j-1];
	  for(int c=cfirst;c<cend;c++) {
	    //Assert(lp.A.rows[c].numEntries()==2);
	    //Assert(lp.A.rows[c].find(j-1) != lp.A.rows[c].end());
	    //Assert(lp.A.rows[c].find(j) != lp.A.rows[c].end());
	    Real ax = lp.A.rows[c].entries[j]*x(j);
	    Real q=lp.q(c)-ax,p=lp.p(c)-ax;
	    Real a=lp.A.rows[c].entries[j-1];
	    //solve q <= a*x <= p
	    if(a < 0) {
	      if(a*x[j-1] < q) 
		x[j-1] = q/a;
	    }
	    else {
	      if(a*x[j-1] > p) 
		x[j-1] = p/a;
	    }
	  }
	  if(x[j-1] == oldxj) {
	    //printf("Backwards propagation from %d stopped at %d with variable left unchanged at %g\n",i,j-1,oldxj); 
	    break;
	  }
	  if(x[j-1] >= 0 && x[j-1] <= 1e-5) {
	    if(gTimeScalingVerbose>=2) 
	      printf("Warning: backprop from %d set %d to small value %g\n",i,j-1,x[j-1]);
	  }
	  if(x[j-1] < 0.0) {
	    if(gTimeScalingVerbose>=2) 
	      printf("Backwards propagation from %d stopped at %d with negative\n",i,j-1); 
	    x[j-1]=0;
	    break; 
	  } 
	  j--;
	}
      }
    }
  }
  if(x[n-1] == 0.0 && x[n] == 0.0) 
    x[n-1] = lp.u(n-1);
  if(BACKWARDS_PROPAGATION) {
    int j=n;
    while(j > 0) {
      //maximize x[j] s.t. the constraints are met
      int cfirst=segToConstraints[j-1].first;
      int cend=segToConstraints[j-1].second;
      Real oldxj = x[j-1];
      for(int c=cfirst;c<cend;c++) {
	Assert(lp.A.rows[c].numEntries()==2);
	Assert(lp.A.rows[c].find(j-1) != lp.A.rows[c].end());
	Assert(lp.A.rows[c].find(j) != lp.A.rows[c].end());
	Real ax = lp.A.rows[c].entries[j]*x(j);
	Real q=lp.q(c)-ax,p=lp.p(c)-ax;
	Real a=lp.A.rows[c].entries[j-1];
	//solve q <= a*x <= p
	if(a < 0) {
	  if(a*x[j-1] < q) 
	    x[j-1] = q/a;
	}
	else {
	  if(a*x[j-1] > p) 
	    x[j-1] = p/a;
	}
      }
      if(x[j-1] == oldxj) {
	//printf("Backwards propagation from %d stopped at %d with variable left unchanged at %g\n",n,j-1,oldxj); 
	break;
      }
      if(x[j-1] >= 0 && x[j-1] <= 1e-5) {
	if(gTimeScalingVerbose>=2) 
	  printf("Warning: backprop from %d set %d to small value %g\n",n,j-1,x[j-1]);
      }
      if(x[j-1] < 0.0) {
	if(gTimeScalingVerbose>=2) 
	  printf("Backwards propagation from %d stopped at %d with negative\n",n,j-1); 
	x[j-1]=0;
	break; 
      } 
      j--;
    }
  }
}

void TimeScalingSLP::InitializeGLPK()
{
  glpk.Set(lp);

  //warm up the GLPK basis
  for(int i=0;i<x.n;i++)
    if(x(i) == lp.u(i)) glpk.SetVariableNonBasic(i,true);
  for(int i=1;i<x.n;i++) {
    int cfirst=segToConstraints[i-1].first;
    int cend=segToConstraints[i-1].second;
    for(int c=cfirst;c<cend;c++) {
      Assert(lp.A.rows[c].numEntries()==2);
      Assert(lp.A.rows[c].find(i-1) != lp.A.rows[c].end());
      Assert(lp.A.rows[c].find(i) != lp.A.rows[c].end());
      Real axprev = lp.A.rows[c].entries[i-1]*x(i-1);
      Real q=lp.q(c)-axprev,p=lp.p(c)-axprev;
      Real a=lp.A.rows[c].entries[i];
      if(x(i) == q/a) {
	glpk.SetVariableBasic(i);
	glpk.SetRowNonBasic(c,false);
	break;
      }
      else if (x(i) == p/a) {
	glpk.SetVariableBasic(i);
	glpk.SetRowNonBasic(c,true);
	break;
      }
    }
  }
}


inline Vector2 AddBound(const Vector2& a,const Vector2& b) { return a+b; }

inline Vector2 SubBound(const Vector2& a,const Vector2& b) { return a-Vector2(b.y,b.x); }


Vector2 SqrBound(const Vector2& a)
{
  if(a.x < 0 && a.y > 0) return Vector2(0.0,Max(Sqr(a.x),Sqr(a.y)));
  else if(a.y <= 0) return Vector2(Sqr(a.y),Sqr(a.x));
  else return Vector2(Sqr(a.x),Sqr(a.y));
}

Vector2 MulBound(const Vector2& a,const Vector2& b)
{
  Vector2 res(a.x*b.x,a.x*b.x);
  Real v=a.x*b.y;
  if(v < res.x) res.x = v;
  else if(v > res.y) res.y = v;
  v=a.y*b.y;
  if(v < res.x) res.x = v;
  else if(v > res.y) res.y = v;
  v=a.y*b.x;
  if(v < res.x) res.x = v;
  else if(v > res.y) res.y = v;
  return res;
}

Vector2 DotProductBound(const Vector& amin,const Vector& amax,const Vector& bmin,const Vector& bmax)
{
  if(amin.n==0) return Vector2(0.0,0.0);
  Vector2 v;
  v = MulBound(Vector2(amin(0),amax(0)),Vector2(bmin(0),bmax(0)));
  for(int i=1;i<amin.n;i++)
    v += MulBound(Vector2(amin(i),amax(i)),Vector2(bmin(i),bmax(i)));
  return v;
}

Vector2 BoundNorm2(const Vector& xmin,const Vector& xmax)
{
  Vector2 res(0.0,0.0);
  for(int i=0;i<xmin.n;i++)
    res += SqrBound(Vector2(xmin(i),xmax(i)));
  return res;
}

//returns the max squared-norm of a vector x in the given aabb 
Real MaxNorm2(const Vector& xmin,const Vector& xmax)
{
  Real sumsq = 0.0;
  for(int i=0;i<xmin.n;i++)
    sumsq += Max(Sqr(xmin(i)),Sqr(xmax(i)));
  return sumsq;
}

//returns the min squared-norm of a vector x in the given aabb 
Real MinNorm2(const Vector& xmin,const Vector& xmax)
{
  Real sumsq = 0.0;
  for(int i=0;i<xmin.n;i++) {
    if(xmin(i) <= 0 && xmax(i) >= 0) continue; //contains 0
    else 
      sumsq += Min(Sqr(xmin(i)),Sqr(xmax(i)));
  }
  return sumsq;
}

//if y = x/||x||, returns an aabb bound on y given an aabb bound on x
void NormalizedVectorBound(const Vector& xmin,const Vector& xmax,Vector& ymin,Vector& ymax)
{
  //x / ||x|| => (x1,x2,...,xn)/sqrt(x1^2+x2^2+...+xn^2)
  //consider finding the minimum on axis 1
  //if xmin1 < 0, set x1=xmin1 and minimize ||x|| over all x2,...,xn
  //if xmin1 > 0, set x1=xmin1 and maximize ||x|| over all x2,...,xn
  Vector min2(xmin.n),max2(xmin.n);
  Real summin2=0,summax2=0;
  for(int i=0;i<xmin.n;i++) {
    max2(i) = Max(Sqr(xmin(i)),Sqr(xmax(i)));
    if(xmin(i) <= 0 && xmax(i) >= 0) 
      min2(i) = 0.0;
    else
      min2(i) = Min(Sqr(xmin(i)),Sqr(xmax(i)));
    summax2 += max2(i);
    summin2 += min2(i);
  }
  ymin.resize(xmin.n);
  ymax.resize(xmin.n);
  if(summin2 == 0.0) {
    //contains origin: return whole sphere (or just a portion)
    for(int i=0;i<xmin.n;i++) {
      if(xmin(i) < 0.0)
	ymin(i)=-1.0;
      else
	ymin(i)=1.0;
      if(xmax(i) > 0.0)
	ymax(i)=1.0;
      else
	ymax(i)=0.0;
    }
    return;
  }
  for(int i=0;i<xmin.n;i++) {
    Assert(xmin(i) <= xmax(i));
    if(xmin(i) < 0.0) 
      ymin(i) = -1.0 / Sqrt(1.0 + (summin2-min2(i))/Sqr(xmin(i)));
    else if(xmin(i) > 0.0)
      ymin(i) = 1.0 / Sqrt(1.0 + (summax2-max2(i))/Sqr(xmin(i)));
    else
      ymin(i) = 0;
    if(xmax(i) < 0.0) 
      ymax(i) = -1.0 / Sqrt(1.0 + (summax2-max2(i))/Sqr(xmax(i)));
    else if(xmax(i) > 0.0)
      ymax(i) = 1.0 / Sqrt(1.0 + (summin2-min2(i))/Sqr(xmax(i)));
    else
      ymax(i) = 0.0;
    Assert(ymin(i) <= ymax(i));
  }
}

void ExpandBound(const Vector& x,Vector& bmin,Vector& bmax)
{
  for(int i=0;i<x.n;i++) {
    if(x[i] < bmin[i]) bmin[i]=x[i];
    else if(x[i] > bmax[i]) bmax[i]=x[i];
  }
}


void ExpandBound(const Vector& bmin0,const Vector& bmax0,Vector& bmin,Vector& bmax)
{
  for(int i=0;i<bmin0.n;i++) {
    if(bmin0[i] < bmin[i]) bmin[i]=bmin0[i];
    else if(bmax0[i] > bmax[i]) bmax[i]=bmax0[i];
  }
}

bool ContainsZeroInterior(const Vector& a,const Vector& b,Real tol)
{
  for(int i=0;i<a.n;i++)
    if(a(i) >= -tol || b(i) <= tol) return false;
  return true;
}



int TimeScaling::TimeToSegment(Real t) const
{
  return times.Map(t);
}

Real TimeScaling::TimeToParam(Real t) const
{
  int seg = TimeToSegment(t);
  return TimeToParam(seg,t);
}

Real TimeScaling::TimeToParam(int segment,Real t) const
{
  if(segment < 0) return params[0];
  else if(segment+1 >= (int)params.size()) return params.back();
  Assert(times[segment] <= t && t <= times[segment+1]);
  Assert(times.size()==ds.size());
  Real u=t-times[segment];
  Real a = 0.25*(Sqr(ds[segment+1])-Sqr(ds[segment]))/(params[segment+1]-params[segment]);
  Real b=ds[segment];
  Real s=params[segment]+b*u+a*Sqr(u);
  if(s < params[segment] || s > params[segment+1]) {
    printf("param %g (time %g) not in [%g,%g]\n",s,t,params[segment],params[segment+1]);
    printf("Segment time range [%g,%g]\n",times[segment],times[segment+1]);
    printf("segment ds = %g to %g\n",ds[segment],ds[segment+1]);
    Real dt=(times[segment+1]-times[segment]);
    printf("Segment delta s = %g\n",b*dt+a*Sqr(dt));
    printf("Segment delta t = %g\n",2.0*(params[segment+1]-params[segment])/(ds[segment]+ds[segment+1]));
  }
  Assert(s >= params[segment]-Epsilon && s <= params[segment+1]+Epsilon);
  return Clamp(s,params[segment],params[segment+1]);
}

Real TimeScaling::TimeToParamDeriv(int segment,Real t) const
{
  if(segment < 0) return 0;
  else if(segment+1 >= (int)params.size()) return 0;
  Assert(times[segment] <= t && t <= times[segment+1]);
  Assert(times.size()==ds.size());
  Real u=t-times[segment];
  Real a = 0.25*(Sqr(ds[segment+1])-Sqr(ds[segment]))/(params[segment+1]-params[segment]);
  Real b=ds[segment];
  return b+2.0*a*u;
}

Real TimeScaling::TimeToParamAccel(int segment,Real t) const
{
  if(segment < 0) return 0;
  else if(segment+1 >= (int)params.size()) return 0;
  Assert(times[segment] <= t && t <= times[segment+1]);
  Assert(times.size()==ds.size());
  Real a = 0.25*(Sqr(ds[segment+1])-Sqr(ds[segment]))/(params[segment+1]-params[segment]);
  return 2.0*a;
}

int TimeScaling::ParamToSegment(Real s) const
{
  return params.Map(s);
}

Real TimeScaling::ParamToTime(Real s) const
{
  int seg = params.Map(s);
  return ParamToTime(seg,s);
}

Real TimeScaling::ParamToTime(int segment,Real s) const
{
  if(segment < 0) return times[0];
  else if(segment+1 >= (int)times.size()) return times.back();
  Assert(params[segment] <= s && s <= params[segment+1]);
  Assert(params.size()==ds.size());
  //solve for u in 
  //s=params[segment]+b*u+a*Sqr(u);
  //t = u+times[segment]
  Real a = 0.25*(Sqr(ds[segment+1])-Sqr(ds[segment]))/(params[segment+1]-params[segment]);
  Real b = ds[segment];
  Real c = params[segment]-s;
  Real u1,u2;
  int res=quadratic(a,b,c,u1,u2);
  if(res == 0 || res < 0) {
    fprintf(stderr,"TimeScaling::ParamToTime: Unable to solve for time\n");
    fprintf(stderr,"  s=%g in [%g,%g]\n",s,params[segment],params[segment+1]);
    fprintf(stderr,"  quadratic %g u^2 + %g u + %g = 0\n",a,b,c);
    Real u = (s-params[segment])/(params[segment+1]-params[segment]);
    return times[segment]+u*(times[segment+1]-times[segment]);
  }
  else if(res == 1)
    return u1+times[segment];
  else {
    Assert(res==2);
    if(u1 < 0 || u1 + times[segment] > times[segment+1]) {
      if(u2 < 0 || u2 + times[segment] > times[segment+1]) {
	fprintf(stderr,"TimeScaling::ParamToTime: Solution is invalid?\n");
	fprintf(stderr,"  s=%g in [%g,%g]\n",s,params[segment],params[segment+1]);
	fprintf(stderr,"  quadratic %g u^2 + %g u + %g = 0\n",a,b,c);
	fprintf(stderr,"  solutions %g %g\n", u1,u2);
	Real u = (s-params[segment])/(params[segment+1]-params[segment]);
	return times[segment]+u*(times[segment+1]-times[segment]);
      }
      return times[segment]+u2;
    }
    else {
      return times[segment]+u1;
    }
  }
}

bool TimeScaling::SolveMinTime(const Vector& vmin,const Vector& vmax,
			       const Vector& amin,const Vector& amax,
			       const vector<Real>& paramdivs,
			       const vector<Vector>& dxs,
			       Real ds0,Real dsEnd)
{
  Assert(paramdivs.size()==dxs.size());
  vector<Vector> dxmins(dxs.size()-1),dxmaxs(dxs.size()-1);
  vector<Vector> ddxmins(dxs.size()-1),ddxmaxs(dxs.size()-1);
  for(size_t i=0;i+1<dxs.size();i++) {
    dxmins[i] = dxmaxs[i] = dxs[i];
    Assert(dxs[i].size()==dxs[i+1].size());
    for(int j=0;j<dxs[i].size();j++) {
      dxmins[i][j] = Min(dxmins[i][j],dxs[i+1][j]);
      dxmaxs[i][j] = Max(dxmaxs[i][j],dxs[i+1][j]);
    }
    ddxmins[i] = ddxmaxs[i] = (dxs[i+1]-dxs[i])/(paramdivs[i+1]-paramdivs[i]);
  }
  return SolveMinTime(vmin,vmax,amin,amax,paramdivs,dxmins,dxmaxs,ddxmins,ddxmaxs,ds0,dsEnd);
}

bool TimeScaling::SolveMinTimeArcLength(const Vector& vmin,const Vector& vmax,
					const Vector& amin,const Vector& amax,
					const vector<Real>& paramdivs,
					const vector<Vector>& dxs,
					Real ds0,Real dsEnd)
{
  Assert(paramdivs.size()==dxs.size());
  vector<Vector> dxmins(dxs.size()-1),dxmaxs(dxs.size()-1);
  vector<Vector> ddxmins(dxs.size()-1),ddxmaxs(dxs.size()-1);
  for(size_t i=0;i+1<dxs.size();i++) {
    dxmins[i] = dxmaxs[i] = dxs[i];
    Assert(dxs[i].size()==dxs[i+1].size());
    for(int j=0;j<dxs[i].size();j++) {
      dxmins[i][j] = Min(dxmins[i][j],dxs[i+1][j]);
      dxmaxs[i][j] = Max(dxmaxs[i][j],dxs[i+1][j]);
    }
    ddxmins[i] = ddxmaxs[i] = (dxs[i+1]-dxs[i])/(paramdivs[i+1]-paramdivs[i]);
  }
  return SolveMinTimeArcLength(vmin,vmax,amin,amax,paramdivs,dxmins,dxmaxs,ddxmins,ddxmaxs,ds0,dsEnd);
}



//conditioning step: we can deal with single direction-changing points with zero derivative
//(which need to be stopped at anyway), but inflection points and plateaus need to be skipped
void TimeScaling::ConditionMinTime(vector<Real>& paramdivs,vector<Vector>& dxs,
				   vector<Vector>& dxMins,vector<Vector>& dxMaxs,
				   vector<Vector>& ddxMins,vector<Vector>& ddxMaxs)
{
  Assert(paramdivs.size() == dxs.size());
  Assert(paramdivs.size() == dxMins.size()+1);
  Assert(paramdivs.size() == dxMaxs.size()+1);
  Assert(paramdivs.size() == ddxMins.size()+1);
  Assert(paramdivs.size() == ddxMaxs.size()+1);

  //int d=(int)dxs[0].size();
  int n=(int)dxs.size()-1;
  Real zeroTolerance = 1e-2;
  vector<pair<int,int> > zeroRanges;
  for(int i=0;i<=n;i++) {
    if(dxs[i].isZero(zeroTolerance)) {
      if(zeroRanges.empty() || i-1 != zeroRanges.back().second) 
	zeroRanges.push_back(pair<int,int>(i,i));
      else
	zeroRanges.back().second=i+1;
    }
  }
  //only ends are zero-speed
  if(zeroRanges.empty()) return;
  if(zeroRanges.size()==1 && (zeroRanges[0]==pair<int,int>(0,0) || zeroRanges[0]==pair<int,int>(n,n)))
     return;
  if(zeroRanges.size()==2 && zeroRanges[0]==pair<int,int>(0,0) && zeroRanges[1]==pair<int,int>(n,n))
    return;

  Vector newdxmin, newdxmax;
  Vector newddxmin, newddxmax;
  int numReduced=0;
  for(size_t i=0;i<zeroRanges.size();i++) {
    int kstart=zeroRanges[i].first-numReduced;
    int kend=zeroRanges[i].second-numReduced;
    printf("ConditionMinTime: found zero range %d %d\n",zeroRanges[i].first,zeroRanges[i].second);
    bool merge=true,mergeEnds=true;
    if(kstart==kend) {
      if(kstart == 0 || kend == (int)dxMins.size()) mergeEnds=false;
      merge = false;
    }
    //get range of speeds (very close to zero)
    Vector dxmin=dxs[kstart],dxmax=dxs[kstart];
    for(int k=kstart+1;k<=kend;k++) {
      ExpandBound(dxs[k],dxmin,dxmax);
    }
    if(mergeEnds) {
      //singularity -- test to see if the growth of the interval would improve conditioning
      int k=kstart;
      if(k > 0) k--;
      newdxmin = dxMins[k];
      newdxmax = dxMaxs[k];
      newddxmin = ddxMins[k];
      newddxmax = ddxMaxs[k];
      k+=1;
      while(k <= kend) {
	ExpandBound(dxMins[k],dxMaxs[k],newdxmin,newdxmax);
	ExpandBound(ddxMins[k],ddxMaxs[k],newddxmin,newddxmax);
	k++;
      }
      if(ContainsZeroInterior(newdxmin-dxmin,newdxmax-dxmax,0)) {
	printf("ConditionMinTime: Singularity from %d to %d does not appear to be an inflection\n",zeroRanges[i].first,zeroRanges[i].second);
	cout<<"dx: "<<dxs[kstart]<<", dxmin "<<dxMins[kstart]<<", dxmax "<<dxMaxs[kstart]<<endl;
	if(k > 0)
	  cout<<"prev dxmin "<<dxMins[kstart]<<", dxmax "<<dxMaxs[kstart]<<endl;
	cout<<"Range: "<<newdxmin<<" to "<<newdxmax<<endl;
	mergeEnds = false;
      }
    }
    if(mergeEnds) {
      printf("ConditionMinTime: Merging %d to %d, inclusive\n",kstart,kend);
      int k=kstart;
      if(k>0) k--;
      newdxmin = dxs[k]; newdxmax = dxs[k];
      ExpandBound(dxs[kend+1],dxs[kend+1],newdxmin,newdxmax);

      //eliminate everything from kstart to kend (inclusive)
      paramdivs.erase(paramdivs.begin()+kstart,paramdivs.begin()+kend+1);
      dxs.erase(dxs.begin()+kstart,dxs.begin()+kend+1);
      dxMaxs.erase(dxMaxs.begin()+kstart,dxMaxs.begin()+kend+1);
      dxMaxs[kstart-1]=newdxmax;
      dxMins.erase(dxMins.begin()+kstart,dxMins.begin()+kend+1);
      dxMins[kstart-1]=newdxmin;
      ddxMaxs.erase(ddxMaxs.begin()+kstart,ddxMaxs.begin()+kend+1);
      ddxMaxs[kstart-1]=newddxmax;
      ddxMins.erase(ddxMins.begin()+kstart,ddxMins.begin()+kend+1);
      ddxMins[kstart-1]=newddxmin;      
      numReduced += kend-kstart+1;
    }
    else if(merge) {
      printf("Merging %d to %d into one\n",kstart,kend);
      int k=kstart;
      newdxmin = dxMins[k];
      newdxmax = dxMaxs[k];
      newddxmin = ddxMins[k];
      newddxmax = ddxMaxs[k];
      k+=1;
      while(k <= kend) {
	ExpandBound(dxMins[k],dxMaxs[k],newdxmin,newdxmax);
	ExpandBound(ddxMins[k],ddxMaxs[k],newddxmin,newddxmax);
	k++;
      }
      //just erase the zero velocity intervals, leaving one
      //eliminate everything from kstart to kend, except one
      paramdivs.erase(paramdivs.begin()+kstart,paramdivs.begin()+kend);
      dxs.erase(dxs.begin()+kstart,dxs.begin()+kend);
      dxMaxs.erase(dxMaxs.begin()+kstart,dxMaxs.begin()+kend);
      dxMaxs[kend]=newdxmax;
      dxMins.erase(dxMins.begin()+kstart,dxMins.begin()+kend);
      dxMins[kend]=newdxmin;
      ddxMaxs.erase(ddxMaxs.begin()+kstart,ddxMaxs.begin()+kend);
      ddxMaxs[kend]=newddxmax;
      ddxMins.erase(ddxMins.begin()+kstart,ddxMins.begin()+kend);
      ddxMins[kend]=newddxmin;      
      numReduced += kend-kstart;
    }
  }
  Assert(dxs.size()==paramdivs.size());
  Assert(dxs.size()==dxMins.size()+1);
  Assert(dxs.size()==dxMaxs.size()+1);
  Assert(dxs.size()==ddxMins.size()+1);
  Assert(dxs.size()==ddxMaxs.size()+1);
}

bool TimeScaling::SolveMinTime(const Vector& vmin,const Vector& vmax,
			       const Vector& amin,const Vector& amax,
			       const vector<Real>& paramdivs,
			       const vector<Vector>& dxMins,const vector<Vector>& dxMaxs,
			       const vector<Vector>& ddxMins,const vector<Vector>& ddxMaxs,
			       Real ds0,Real dsEnd)
{
  Assert(paramdivs.size() == dxMins.size()+1);
  Assert(paramdivs.size() == dxMaxs.size()+1);
  Assert(paramdivs.size() == ddxMins.size()+1);
  Assert(paramdivs.size() == ddxMaxs.size()+1);
  Assert(vmin.n == vmax.n);
  Assert(vmin.n == amin.n);
  Assert(amin.n == amax.n);
  int d=vmin.n;    //number of dimensions
  size_t n=dxMins.size();   //number of grid segments
  for(size_t i=0;i<n;i++) {
    Assert(dxMins[i].n == d);
    Assert(dxMaxs[i].n == d);
    Assert(ddxMins[i].n == d);
    Assert(ddxMaxs[i].n == d);
  }
  Assert(vmin.maxElement() <= 0);
  Assert(amin.maxElement() <= 0);
  Assert(vmax.minElement() >= 0);
  Assert(amax.minElement() >= 0);

  TimeScalingSLP slp(paramdivs);
  vector<Real> dsmax(n,Inf);
  for(size_t i=0;i<n;i++) {  
    for(int j=0;j<d;j++) {
      if(dxMaxs[i][j] >= 0 && dxMins[i][j] <= 0) continue;
      dsmax[i] = Min(dsmax[i],Max(vmax[j]/dxMaxs[i][j],vmin[j]/dxMins[i][j]));
    }
    if(!IsFinite(dsmax[i])) dsmax[i] = Inf;
  }
  //setup velocity bounds
  slp.SetVelBound(0,dsmax[0]);
  for(size_t i=0;i+1<n;i++) {
    slp.SetVelBound(i+1,Min(dsmax[i],dsmax[i+1]));
  }
  slp.SetVelBound(n,dsmax[n]);

  //fixed endpoints
  if(ds0 >= 0)
    slp.SetFixed(0,ds0);
  if(dsEnd >= 0)
    slp.SetFixed(n,dsEnd);
  if(n==1 && ds0 == 0 && dsEnd == 0) return false;

  //set up constraints
  //Ai * x[i] + Ai * x[i+1] <= bi
  Vector Ai(d*8), An(d*8);
  Vector bi(d*8);
  for(size_t i=0;i<n;i++) { 
    Assert(paramdivs[i+1]>paramdivs[i]);
    Real scale = 0.5/(paramdivs[i+1]-paramdivs[i]);
    //Real invscale = 2.0*(paramdivs[i+1]-paramdivs[i]);
    //cout<<"seg "<<i<<": "<<dxMins[i]<<", "<<dxMaxs[i]<<endl;
    //cout<<"     "<<ddxMins[i]<<", "<<ddxMaxs[i]<<endl;
    int num=0;
    for(int j=0;j<d;j++) {
      Vector2 dbnd(dxMins[i][j],dxMaxs[i][j]);
      Vector2 ddbnd(ddxMins[i][j],ddxMaxs[i][j]);
      Vector2 abnd(amin[j],amax[j]);
      Assert(dbnd.x <= dbnd.y);
      Assert(ddbnd.x <= ddbnd.y);
      Assert(abnd.x <= abnd.y);
      //more optimistic estimate:
      //2ds * ddbnd * x[i] + dx[i](x[i+1]-x[i]) in abnd*2ds
      //2ds * ddbnd * x[i+1] + dx[i+1](x[i+1]-x[i]) in abnd*2ds
      //conservative estimate
      //2ds * ddbnd * x[i] + dbnd(x[i+1]-x[i]) in abnd*2ds
      //2ds * ddbnd * x[i+1] + dbnd(x[i+1]-x[i]) in abnd*2ds
      //(2ds*ddbnd-dbnd) * x[i] + dbnd * x[i+1] in abnd*2ds
      // -dbnd x[i] + (2ds*ddbnd+dbnd) * x[i+1] in abnd*2ds
      //we know x >= 0, so a sufficient representation is
      //max((2ds*ddbnd-dbnd)x[i]) + max(dbnd)*x[i+1] <= max(abnd)*2ds
      //min((2ds*ddbnd-dbnd)x[i]) + min(dbnd)*x[i+1] >= min(abnd)*2ds
      //max(-dbnd) x[i] + max((2ds*ddbnd+dbnd)x[i+1]) <= max(abnd)*2ds
      //min(-dbnd) x[i] + min((2ds*ddbnd+dbnd)x[i+1]) >= min(abnd)*2ds
      Vector2 a1(ddbnd.x-dbnd.y*scale,ddbnd.y-dbnd.x*scale), b1(dbnd*scale);
      Vector2 a2(-scale*dbnd.y,-scale*dbnd.x), b2(ddbnd+scale*dbnd);
      //Vector2 a1(ddbnd.x*invscale-dbnd.y,ddbnd.y*invscale-dbnd.x), b1(dbnd);
      //Vector2 a2(-dbnd.y,-dbnd.x), b2(ddbnd*invscale+dbnd);
      //abnd *= invscale;
      Assert(a1.x <= a1.y);
      Assert(b1.x <= b1.y);
      Assert(a2.x <= a2.y);
      Assert(b2.x <= b2.y);
      //printf("[%g,%g]*x[%d] + [%g,%g]*x[%d] in [%g,%g]\n",a1.x,a1.y,i,b1.x,b1.y,i+1,abnd.x,abnd.y);
      //printf("[%g,%g]*x[%d] + [%g,%g]*x[%d] in [%g,%g]\n",a2.x,a2.y,i,b2.x,b2.y,i+1,abnd.x,abnd.y);
      //a1.x * x[i] + b1.x *x[i+1] >= abnd.x
      //a1.y * x[i] + b1.x *x[i+1] >= abnd.x
      //a1.x * x[i] + b1.y *x[i+1] <= abnd.y
      //a1.y * x[i] + b1.y *x[i+1] <= abnd.y
      //a2.x * x[i] + b2.x *x[i+1] >= abnd.x
      //a2.x * x[i] + b2.y *x[i+1] >= abnd.x
      //a2.y * x[i] + b2.x *x[i+1] <= abnd.y
      //a2.y * x[i] + b2.y *x[i+1] <= abnd.y
      Ai[num] = -a1.x; An[num] = -b1.x; bi[num] = -abnd.x; num++;
      Ai[num] = -a1.y; An[num] = -b1.x; bi[num] = -abnd.x; num++;
      Ai[num] = a1.x; An[num] = b1.y; bi[num] = abnd.y; num++;
      Ai[num] = a1.y; An[num] = b1.y; bi[num] = abnd.y; num++;
      Ai[num] = -a2.x; An[num] = -b2.x; bi[num] = -abnd.x; num++;
      Ai[num] = -a2.x; An[num] = -b2.y; bi[num] = -abnd.x; num++;
      Ai[num] = a2.y; An[num] = b2.x; bi[num] = abnd.y; num++;
      Ai[num] = a2.y; An[num] = b2.y; bi[num] = abnd.y; num++;
      /*
	if(dbnd.x == 0 || dbnd.y == 0) {
	printf("Singularity: %d: x[i] <= %g, x[i+1] <= %g\n",i,slp.u(i),slp.u(i+1));
	for(int j=k-8;j<k;j++) {
	printf("%g <= %g * x[i] + %g * x[i+1] <= %g:\n",slp.q(j),slp.A(j,i),slp.A(j,i+1),slp.p(j));
	}
	}
      */
    }
    if(!slp.AddVel2Bounds(i,Ai,An,bi))
      return false;
  }
  cout<<n<<" segments, "<<d<<" dimensions, "<<slp.lp.A.m<<" constraints"<<endl;

  printf("Reduced %d constraints to %d\n",n*d*8,slp.lp.A.m);

  int maxIters = gSLPMaxIters;
  bool res = slp.Solve(maxIters,gSLPXTolerance,gSLPFTolerance);

  if(CHECK_SLP_BOUNDS) {
    slp.CheckSolution();
  }

  params.resize(paramdivs.size());
  copy(paramdivs.begin(),paramdivs.end(),params.begin());
  times.resize(params.size());
  times[0] = 0;
  ds = slp.GetVelocities();
  for(size_t i=0;i<n;i++) {
    Assert(ds[i]+ds[i+1] > 0);
    Real dt = 2*(paramdivs[i+1]-paramdivs[i])/(ds[i]+ds[i+1]);    
    times[i+1]=times[i]+dt;
  }

  if(CHECK_SLP_BOUNDS) {
    Real maxGap = 0.0, sumGap = 0.0;
    for(size_t i=0;i<n;i++) {
      //compute min/max y' and y''
      Real minGap = Inf;
      if(ds[i] < 0 || ds[i+1] < 0)
	printf("Negative speed %d?\n",i);
      for(int j=0;j<d;j++) {
	Real dy = ds[i]*dxMins[i][j];
	if(dy < vmin[j]-Epsilon)
	  printf("Speed [%d,%d] = %g < vmin %g\n",i,j,dy,vmin[j]);
	else if(dy > vmax[j]+Epsilon)
	  printf("Speed [%d,%d] = %g > vmax %g\n",i,j,dy,vmax[j]);
	minGap = Min(minGap,dy-vmin[j]);
	minGap = Min(minGap,vmax[j]-dy);
	dy = ds[i]*dxMaxs[i][j];
	if(dy < vmin[j]-Epsilon)
	  printf("Speed [%d,%d] = %g < vmin %g\n",i,j,dy,vmin[j]);
	else if(dy > vmax[j]+Epsilon)
	  printf("Speed [%d,%d] = %g > vmax %g\n",i,j,dy,vmax[j]);
	minGap = Min(minGap,dy-vmin[j]);
	minGap = Min(minGap,vmax[j]-dy);
	dy = ds[i+1]*dxMins[i][j];
	if(dy < vmin[j]-Epsilon)
	  printf("End speed [%d,%d] = %g < vmin %g\n",i+1,j,dy,vmin[j]);
	else if(dy > vmax[j]+Epsilon)
	  printf("End speed [%d,%d] = %g > vmax %g\n",i+1,j,dy,vmax[j]);
	minGap = Min(minGap,dy-vmin[j]);
	minGap = Min(minGap,vmax[j]-dy);
	dy = ds[i+1]*dxMaxs[i][j];
	if(dy < vmin[j]-Epsilon)
	  printf("End speed [%d,%d] = %g < vmin %g\n",i+1,j,dy,vmin[j]);
	else if(dy > vmax[j]+Epsilon)
	  printf("End speed [%d,%d] = %g > vmax %g\n",i+1,j,dy,vmax[j]);
	minGap = Min(minGap,dy-vmin[j]);
	minGap = Min(minGap,vmax[j]-dy);

	Real ddy = Sqr(ds[i])*ddxMins[i][j] + (Sqr(ds[i+1])-Sqr(ds[i]))/(paramdivs[i+1]-paramdivs[i])*0.5*dxMins[i][j];
	if(ddy < amin[j])
	  printf("Accel [%d,%d] = %g < amin %g\n",i+1,j,ddy,amin[j]);
	else if(ddy > amax[j])
	  printf("Accel [%d,%d] = %g > amax %g\n",i+1,j,ddy,amax[j]);
	minGap = Min(minGap,ddy-amin[j]);
	minGap = Min(minGap,amax[j]-ddy);
	ddy = Sqr(ds[i])*ddxMaxs[i][j] + (Sqr(ds[i+1])-Sqr(ds[i]))/(paramdivs[i+1]-paramdivs[i])*0.5*dxMins[i][j];
	if(ddy < amin[j])
	  printf("Accel [%d,%d] = %g < amin %g\n",i+1,j,ddy,amin[j]);
	else if(ddy > amax[j])
	  printf("Accel [%d,%d] = %g > amax %g\n",i+1,j,ddy,amax[j]);
	minGap = Min(minGap,ddy-amin[j]);
	minGap = Min(minGap,amax[j]-ddy);
	ddy = Sqr(ds[i])*ddxMins[i][j] + (Sqr(ds[i+1])-Sqr(ds[i]))/(paramdivs[i+1]-paramdivs[i])*0.5*dxMaxs[i][j];
	if(ddy < amin[j])
	  printf("Accel [%d,%d] = %g < amin %g\n",i+1,j,ddy,amin[j]);
	else if(ddy > amax[j])
	  printf("Accel [%d,%d] = %g > amax %g\n",i+1,j,ddy,amax[j]);
	minGap = Min(minGap,ddy-amin[j]);
	minGap = Min(minGap,amax[j]-ddy);
	ddy = Sqr(ds[i])*ddxMaxs[i][j] + (Sqr(ds[i+1])-Sqr(ds[i]))/(paramdivs[i+1]-paramdivs[i])*0.5*dxMaxs[i][j];
	if(ddy < amin[j])
	  printf("Accel [%d,%d] = %g < amin %g\n",i+1,j,ddy,amin[j]);
	else if(ddy > amax[j])
	  printf("Accel [%d,%d] = %g > amax %g\n",i+1,j,ddy,amax[j]);
	minGap = Min(minGap,ddy-amin[j]);
	minGap = Min(minGap,amax[j]-ddy);

	ddy = Sqr(ds[i+1])*ddxMins[i][j] + (Sqr(ds[i+1])-Sqr(ds[i]))/(paramdivs[i+1]-paramdivs[i])*0.5*dxMins[i][j];
	if(ddy < amin[j])
	  printf("End accel [%d,%d] = %g < amin %g\n",i+1,j,ddy,amin[j]);
	else if(ddy > amax[j])
	  printf("End accel [%d,%d] = %g > amax %g\n",i+1,j,ddy,amax[j]);
	minGap = Min(minGap,ddy-amin[j]);
	minGap = Min(minGap,amax[j]-ddy);
	ddy = Sqr(ds[i+1])*ddxMaxs[i][j] + (Sqr(ds[i+1])-Sqr(ds[i]))/(paramdivs[i+1]-paramdivs[i])*0.5*dxMins[i][j];
	if(ddy < amin[j])
	  printf("End accel [%d,%d] = %g < amin %g\n",i+1,j,ddy,amin[j]);
	else if(ddy > amax[j])
	  printf("End accel [%d,%d] = %g > amax %g\n",i+1,j,ddy,amax[j]);
	minGap = Min(minGap,ddy-amin[j]);
	minGap = Min(minGap,amax[j]-ddy);
	ddy = Sqr(ds[i+1])*ddxMins[i][j] + (Sqr(ds[i+1])-Sqr(ds[i]))/(paramdivs[i+1]-paramdivs[i])*0.5*dxMaxs[i][j];
	if(ddy < amin[j])
	  printf("End accel [%d,%d] = %g < amin %g\n",i+1,j,ddy,amin[j]);
	else if(ddy > amax[j])
	  printf("End accel [%d,%d] = %g > amax %g\n",i+1,j,ddy,amax[j]);
	minGap = Min(minGap,ddy-amin[j]);
	minGap = Min(minGap,amax[j]-ddy);
	ddy = Sqr(ds[i+1])*ddxMaxs[i][j] + (Sqr(ds[i+1])-Sqr(ds[i]))/(paramdivs[i+1]-paramdivs[i])*0.5*dxMaxs[i][j];
	if(ddy < amin[j])
	  printf("End accel [%d,%d] = %g < amin %g\n",i+1,j,ddy,amin[j]);
	else if(ddy > amax[j])
	  printf("End accel [%d,%d] = %g > amax %g\n",i+1,j,ddy,amax[j]);
	minGap = Min(minGap,ddy-amin[j]);
	minGap = Min(minGap,amax[j]-ddy);

      }
      if(minGap > maxGap) maxGap = minGap;
      sumGap += minGap;
    }
    printf("Maximum bound gap %g, average bound gap %g\n",maxGap,sumGap/n);
  /*
  for(int i=0;i<=n;i++) {
    printf("Variable %d\n",i);
    if(i<n) {
      printf("Predicted ddx = ");
      for(int j=0;j<d;j++) {
	Real maxdy=0;
	Real dy=Sqr(ds[i])*ddxMins[i][j] + (Sqr(ds[i+1])-Sqr(ds[i]))/(paramdivs[i+1]-paramdivs[i])*0.5*dxMins[i][j];
	if(Abs(dy) > Abs(maxdy)) maxdy=dy;
	dy=Sqr(ds[i])*ddxMaxs[i][j] + (Sqr(ds[i+1])-Sqr(ds[i]))/(paramdivs[i+1]-paramdivs[i])*0.5*dxMins[i][j];
	if(Abs(dy) > Abs(maxdy)) maxdy=dy;
	dy=Sqr(ds[i])*ddxMins[i][j] + (Sqr(ds[i+1])-Sqr(ds[i]))/(paramdivs[i+1]-paramdivs[i])*0.5*dxMaxs[i][j];
	if(Abs(dy) > Abs(maxdy)) maxdy=dy;
	dy=Sqr(ds[i])*ddxMaxs[i][j] + (Sqr(ds[i+1])-Sqr(ds[i]))/(paramdivs[i+1]-paramdivs[i])*0.5*dxMaxs[i][j];
	if(Abs(dy) > Abs(maxdy)) maxdy=dy;
	printf("%g ",maxdy);
      }
    }
    else {
      printf("Predicted ddx = ");
      for(int j=0;j<d;j++) {
	Real maxdy=0;
	Real dy=Sqr(ds[i])*ddxMins[i-1][j] + (Sqr(ds[i])-Sqr(ds[i-1]))/(paramdivs[i]-paramdivs[i-1])*0.5*dxMins[i-1][j];
	if(Abs(dy) > Abs(maxdy)) maxdy=dy;
	dy=Sqr(ds[i])*ddxMaxs[i-1][j] + (Sqr(ds[i])-Sqr(ds[i-1]))/(paramdivs[i]-paramdivs[i-1])*0.5*dxMins[i-1][j];
	if(Abs(dy) > Abs(maxdy)) maxdy=dy;
	dy=Sqr(ds[i])*ddxMins[i-1][j] + (Sqr(ds[i])-Sqr(ds[i-1]))/(paramdivs[i]-paramdivs[i-1])*0.5*dxMaxs[i-1][j];
	if(Abs(dy) > Abs(maxdy)) maxdy=dy;
	dy=Sqr(ds[i])*ddxMaxs[i-1][j] + (Sqr(ds[i])-Sqr(ds[i-1]))/(paramdivs[i]-paramdivs[i-1])*0.5*dxMaxs[i-1][j];
	if(Abs(dy) > Abs(maxdy)) maxdy=dy;
	printf("%g ",maxdy);
      }
    }
    printf("\n");
    for(size_t j=0;j<limitingConstraints[i].size();j++) {
      int c=limitingConstraints[i][j];
      if(c >= 0) {
	Real ax = lp.A.dotRow(c,x);
	printf("%g <= %g <= %g\n",lp.q(c),ax,lp.p(c));
	for(SparseArray<Real>::iterator k=lp.A.rows[c].begin();k!=lp.A.rows[c].end();k++)
	  printf("%d: %g, ",k->first,k->second);
	printf("\n");
      }
    }
  }
  */
  }
  return res;
}

bool TimeScaling::SolveMinTime(const Vector& vmin,const Vector& vmax,
			       const Vector& amin,const Vector& amax,
			       const GeneralizedCubicBezierSpline& path,
			       Real ds0,Real dsEnd)
{
  Assert(vmin.n == vmax.n);
  Assert(vmin.n == amin.n);
  Assert(amin.n == amax.n);
  int d=vmin.n;
  size_t n=path.segments.size();
  vector<Real> paramdivs(n+1);
  paramdivs[0] = 0;
  for(size_t i=0;i<n;i++)
    paramdivs[i+1]=paramdivs[i]+path.durations[i];

  Assert(vmin.maxElement() <= 0);
  Assert(amin.maxElement() <= 0);
  Assert(vmax.minElement() >= 0);
  Assert(amax.minElement() >= 0);

  TimeScalingSLP slp(paramdivs);
  vector<Real> dsmax(n,Inf);
  Vector vmini,vmaxi,amini,amaxi;
  for(size_t i=0;i<n;i++) {  
    //get derivative bounds
    path.segments[i].GetDerivBounds(vmini,vmaxi,amini,amaxi);
    vmini /= path.durations[i];
    vmaxi /= path.durations[i];
    //don't need to fix amini and amaxi
    for(int j=0;j<d;j++) {
      if(vmaxi[j] >= 0 && vmini[j] <= 0) continue;
      dsmax[i] = Min(dsmax[i],Max(vmax[j]/vmaxi[j],vmin[j]/vmini[j]));
    }
    if(!IsFinite(dsmax[i])) dsmax[i] = Inf;
  }

  //setup velocity bounds
  slp.SetVelBound(0,dsmax[0]);
  for(size_t i=0;i+1<n;i++) {
    slp.SetVelBound(i+1,Min(dsmax[i],dsmax[i+1]));
  }
  slp.SetVelBound(n,dsmax[n]);

  //fixed endpoints
  if(ds0 >= 0)
    slp.SetFixed(0,ds0);
  if(dsEnd >= 0)
    slp.SetFixed(n,dsEnd);
  if(n==1 && ds0 == 0 && dsEnd == 0) return false;

  //spline bound coefficients on a*theta[i], a*theta[i+1], b*(theta[i+1]-theta[i])
  //there's also an offset of b*theta[i]+c/2*(theta[i+1]-theta[i])
  const static Real coeffs[][3] = {
    {0,0,0},         //startpoint
    {-0.5,2.5,1.5},  //endpoint
    {1,0,0.75},
    {0,0,0.25},
    {0,1.0/3.0,7.0/12.0},
    {0.25,0.75,1},
    {1.0/3.0,4.0/3.0,11.0/12.0},
    {0,2.0,5.0/4.0},
  };
  const static int numcoeffs = 8;

  //new velocity constraints
  for(size_t i=0;i<n;i++) { 
    Real invDuration = 1.0 / path.durations[i];
    vector<Real> Ai,An,bi;
    Vector va,vb,vc;
    //Assumes that the path is on a Cartesian space
    path.segments[i].Deriv(0,vc);
    path.segments[i].Accel(0,vb);
    path.segments[i].Accel(1,va);
    //the ending acceleration is 2va+vb, correct it by adding vb and dividing by 2
    va -= vb;
    va *= 0.5;
    for(int j=0;j<d;j++) {
      //coefficients of path derivs
      Real a=va(j),b=vb(j),c=vc(j);
      Real thetai_offset = b - c*0.5;
      Real thetan_offset = c*0.5;
      //printf("a=%g, b=%g, c=%g, deltas=%g, min %g, max %g\n",a,b,c,path.durations[i],amin[j],amax[j]);
      for(int k=0;k<numcoeffs;k++) {
	Real thetai_coeff = (coeffs[k][0]*a - coeffs[k][2]*b + thetai_offset)*Sqr(invDuration);
	Real thetan_coeff = (coeffs[k][1]*a + coeffs[k][2]*b + thetan_offset)*Sqr(invDuration);
	//printf("  coeff[i]=%g, coeff[i+1]=%g\n",thetai_coeff,thetan_coeff);
	Ai.push_back(thetai_coeff);
	An.push_back(thetan_coeff);
	bi.push_back(amax[j]);
	Ai.push_back(-thetai_coeff);
	An.push_back(-thetan_coeff);
	bi.push_back(-amin[j]);
      }
    }
    //printf("Adding %d bounds...\n",Ai.size());
    //int cprev = slp.lp.A.m;
    bool res=slp.AddVel2Bounds(i,Ai,An,bi);
    if(!res) {
      printf("LP was found to be infeasible on segment %d\n",i);
      //printf("Infeasible LP!\n");
      //getchar();
      return false;
    }
    /*
    printf("New rows:\n");
    for(int j=cprev;j<slp.lp.A.m;j++) {
      printf("%g <= ",slp.lp.q(j));
      for(SparseMatrix::RowT::iterator k=slp.lp.A.rows[j].begin();k!=slp.lp.A.rows[j].end();k++)
	printf(" %g * x[%d] + ",k->second,k->first);
      printf("<= %g\n",slp.lp.p(j));
    }
    getchar();
    */
  }
  cout<<n<<" segments, "<<d<<" dimensions, "<<slp.lp.A.m<<" constraints"<<endl;


  printf("Reduced %d constraints to %d\n",n*d*numcoeffs*2,slp.lp.A.m);

  int maxIters = gSLPMaxIters;
  bool res = slp.Solve(maxIters,gSLPXTolerance,gSLPFTolerance);

  if(CHECK_SLP_BOUNDS) {
    slp.CheckSolution();
  }

  params.resize(paramdivs.size());
  copy(paramdivs.begin(),paramdivs.end(),params.begin());
  times.resize(params.size());
  times[0] = 0;
  ds = slp.GetVelocities();
  for(size_t i=0;i<n;i++) {
    Assert(ds[i]+ds[i+1] > 0);
    Real dt = 2*(paramdivs[i+1]-paramdivs[i])/(ds[i]+ds[i+1]);    
    times[i+1]=times[i]+dt;
  }

  return res;
}


bool TimeScaling::SolveMinTimeArcLength(const Vector& vmin,const Vector& vmax,
			       const Vector& amin,const Vector& amax,
			       const vector<Real>& paramdivs,
			       const vector<Vector>& dxMins,const vector<Vector>& dxMaxs,
			       const vector<Vector>& ddxMins,const vector<Vector>& ddxMaxs,
			       Real ds0,Real dsEnd)
{
  Assert(paramdivs.size() == dxMins.size()+1);
  Assert(paramdivs.size() == dxMaxs.size()+1);
  Assert(paramdivs.size() == ddxMins.size()+1);
  Assert(paramdivs.size() == ddxMaxs.size()+1);
  Assert(vmin.n == vmax.n);
  Assert(vmin.n == amin.n);
  Assert(amin.n == amax.n);
  int d=vmin.n;
  int n=(int)dxMins.size();
  for(int i=0;i<n;i++) {
    Assert(dxMins[i].n == d);
    Assert(dxMaxs[i].n == d);
    Assert(ddxMins[i].n == d);
    Assert(ddxMaxs[i].n == d);
  }
  Assert(vmin.maxElement() <= 0);
  Assert(amin.maxElement() <= 0);
  Assert(vmax.minElement() >= 0);
  Assert(amax.minElement() >= 0);

  //solve for ds at each of the divs
  LinearProgram_Sparse lp;
  lp.minimize = true;
  lp.Resize(n*d*8,n+1);
  //set up x=ds^2 variable bounds
  lp.l.setZero();
  vector<Real> dsmax(n,Inf);
  Vector dxnMin,dxnMax,ddxnMin(d),ddxnMax(d);
  for(int i=0;i<n;i++) {  
    NormalizedVectorBound(dxMins[i],dxMaxs[i],dxnMin,dxnMax);
    for(int j=0;j<d;j++) {
      if(dxnMax[j] > 0 && dxnMin[j] < 0) continue;
      dsmax[i] = Min(dsmax[i],Max(vmax[j]/dxnMax[j],vmin[j]/dxnMin[j]));
    }
    if(!IsFinite(dsmax[i])) dsmax[i] = Inf;
  }
  lp.u(0) = Sqr(dsmax[0]);
  for(int i=0;i+1<n;i++) {
    lp.u(i+1) = Sqr(Min(dsmax[i],dsmax[i+1]));
  }
  lp.u(n) = Sqr(dsmax[n-1]);
  //fixed endpoints 
  if(ds0 > 0 || dsEnd > 0) FatalError("TODO: nonzero velocities for arc-length solver");
  if(ds0 >= 0)
    lp.u(0) = lp.l(0) = Sqr(ds0);
  if(dsEnd >= 0)
    lp.u(n) = lp.l(n) = Sqr(dsEnd);
  if(n==1 && ds0 == 0 && dsEnd == 0) return false;

  //set up constraints
  int k=0;
  Vector Aimin(d),Aimax(d);
  vector<pair<int,int> > segToConstraints(n);
  for(int i=0;i<n;i++) { 
    Assert(paramdivs[i+1]>paramdivs[i]);
    Real scale = 0.5/(paramdivs[i+1]-paramdivs[i]);
    //Real invscale = 2.0*(paramdivs[i+1]-paramdivs[i]);
    Real dxlen2min = MinNorm2(dxMins[i],dxMaxs[i]);
    Real dxlen2max = MaxNorm2(dxMins[i],dxMaxs[i]);
    NormalizedVectorBound(dxMins[i],dxMaxs[i],dxnMin,dxnMax); 
    //find (I-dx dx^T /||dx||^2) ddx/||dx^2|| with dx/||dx|| in [dxnMin,dxnMax],
    //ddq in [ddxMins[i],ddxMaxs[i]]
    //get two bounds: (I-[dxnMin,dxnMax][dxnMin,dxnMax]^T)[ddxMins[i],ddxMaxs[i]]
    //vs [ddxMins[i],ddxMaxs[i]]-[dxnMin,dxnMax]([dxnMin,dxnMax]^T[ddxMins[i],ddxMaxs[i]])
    segToConstraints[i].first = k;
    for(int j=0;j<d;j++) {
      Vector2 dxjBound = Vector2(dxnMin[j],dxnMax[j]);
      Vector2 ddxjBound = Vector2(ddxMins[i][j],ddxMaxs[i][j]);
      Vector2 res=SubBound(Vector2(1.0,1.0),SqrBound(dxjBound));
      for(int k=0;k<d;k++) {
	if(k != j)
	  res = SubBound(res,MulBound(dxjBound,Vector2(dxnMin[k],dxnMax[k])));
      }
      Vector2 res2=DotProductBound(dxnMin,dxnMax,ddxMins[i],ddxMaxs[i]);
      res2 = SubBound(ddxjBound,MulBound(dxjBound,res2));
      ddxnMin[j] = Max(res.x,res2.x);
      ddxnMax[j] = Min(res.y,res2.y);
      Assert(ddxnMin[j]<=ddxnMax[j]);
      //divide by ||dx||^2
      if(ddxnMin[j] < 0 && ddxnMax[j] > 0) {
	if(dxlen2min == 0) {
	  //the acceleration bound contains zero -- consider this to be
	  //an inflection point?
	  ddxnMin[j] = ddxnMax[j] = 0;
	}
	else {
	  ddxnMax[j] /= dxlen2min;
	  ddxnMin[j] /= dxlen2min;
	}
      }
      else if(ddxnMin[j] == 0 && ddxnMax[j] == 0) {}
      else { //one-sided
	if(dxlen2min == 0) {
	  //it looks like a stopping point
	  ddxnMin[j] /= dxlen2max;
	  ddxnMax[j] /= dxlen2min;
	}
	else {
	  ddxnMin[j] /= dxlen2max;
	  ddxnMax[j] /= dxlen2min;
	}
      }
    }
    //cout<<"Normalized velocity bounds "<<i<<": "<<dxnMin<<", "<<dxnMax<<endl;
    //cout<<"Normalized acceleration bounds "<<i<<": "<<ddxnMin<<", "<<ddxnMax<<endl;
    for(int j=0;j<d;j++) {
      Vector2 dbnd(dxnMin[j],dxnMin[j]);
      Vector2 ddbnd(ddxnMin[j],ddxnMax[j]);
      Vector2 abnd(amin[j],amax[j]);
      Assert(dbnd.x <= dbnd.y);
      Assert(ddbnd.x <= ddbnd.y);
      Assert(abnd.x <= abnd.y);
      //more optimistic estimate:
      //2ds * ddbnd * x[i] + dx[i](x[i+1]-x[i]) in abnd*2ds
      //2ds * ddbnd * x[i+1] + dx[i+1](x[i+1]-x[i]) in abnd*2ds
      //conservative estimate
      //2ds * ddbnd * x[i] + dbnd(x[i+1]-x[i]) in abnd*2ds
      //2ds * ddbnd * x[i+1] + dbnd(x[i+1]-x[i]) in abnd*2ds
      //(2ds*ddbnd-dbnd) * x[i] + dbnd * x[i+1] in abnd*2ds
      // -dbnd x[i] + (2ds*ddbnd+dbnd) * x[i+1] in abnd*2ds
      //we know x >= 0, so a sufficient representation is
      //max((2ds*ddbnd-dbnd)x[i]) + max(dbnd)*x[i+1] <= max(abnd)*2ds
      //min((2ds*ddbnd-dbnd)x[i]) + min(dbnd)*x[i+1] >= min(abnd)*2ds
      //max(-dbnd) x[i] + max((2ds*ddbnd+dbnd)x[i+1]) <= max(abnd)*2ds
      //min(-dbnd) x[i] + min((2ds*ddbnd+dbnd)x[i+1]) >= min(abnd)*2ds
      Vector2 a1(ddbnd.x-dbnd.y*scale,ddbnd.y-dbnd.x*scale), b1(dbnd*scale);
      Vector2 a2(-scale*dbnd.y,-scale*dbnd.x), b2(ddbnd+scale*dbnd);
      Assert(a1.x <= a1.y);
      Assert(b1.x <= b1.y);
      Assert(a2.x <= a2.y);
      Assert(b2.x <= b2.y);
      //Vector2 a1(ddbnd.x*invscale-dbnd.y,ddbnd.y*invscale-dbnd.x), b1(dbnd);
      //Vector2 a2(-dbnd.y,-dbnd.x), b2(ddbnd*invscale+dbnd);
      //abnd *= invscale;
      //printf("[%g,%g]*x[%d] + [%g,%g]*x[%d] in [%g,%g]\n",a1.x,a1.y,i,b1.x,b1.y,i+1,abnd.x,abnd.y);
      //printf("[%g,%g]*x[%d] + [%g,%g]*x[%d] in [%g,%g]\n",a2.x,a2.y,i,b2.x,b2.y,i+1,abnd.x,abnd.y);
      //a1.x * x[i] + b1.x *x[i+1] >= abnd.x
      //a1.y * x[i] + b1.x *x[i+1] >= abnd.x
      //a1.x * x[i] + b1.y *x[i+1] <= abnd.y
      //a1.y * x[i] + b1.y *x[i+1] <= abnd.y
      //a2.x * x[i] + b2.x *x[i+1] >= abnd.x
      //a2.x * x[i] + b2.y *x[i+1] >= abnd.x
      //a2.y * x[i] + b2.x *x[i+1] <= abnd.y
      //a2.y * x[i] + b2.y *x[i+1] <= abnd.y
      lp.A(k,i) = a1.x; lp.A(k,i+1) = b1.x;  lp.q(k) = abnd.x; k++;
      lp.A(k,i) = a1.y; lp.A(k,i+1) = b1.x;  lp.q(k) = abnd.x; k++;
      lp.A(k,i) = a1.x; lp.A(k,i+1) = b1.y;  lp.p(k) = abnd.y; k++;
      lp.A(k,i) = a1.y; lp.A(k,i+1) = b1.y;  lp.p(k) = abnd.y; k++;
      lp.A(k,i) = a2.x; lp.A(k,i+1) = b2.x;  lp.q(k) = abnd.x; k++;
      lp.A(k,i) = a2.x; lp.A(k,i+1) = b2.y;  lp.q(k) = abnd.x; k++;
      lp.A(k,i) = a2.y; lp.A(k,i+1) = b2.x;  lp.p(k) = abnd.y; k++;
      lp.A(k,i) = a2.y; lp.A(k,i+1) = b2.y;  lp.p(k) = abnd.y; k++;
    }
    segToConstraints[i].second = k;
  }
  Assert(k == lp.A.m);
  //lp.Print(cout);
  //getchar();

  //set up initial solution in a greedy fashion
  Vector x=lp.u;
  for(int i=1;i<n+1;i++) { 
    int cfirst=segToConstraints[i-1].first;
    int cend=segToConstraints[i-1].second;
    Real xorig = x[i];
    for(int c=cfirst;c<cend;c++) {
      Assert(lp.A.rows[c].numEntries()==2);
      Assert(lp.A.rows[c].find(i-1) != lp.A.rows[c].end());
      Assert(lp.A.rows[c].find(i) != lp.A.rows[c].end());
      Real axprev = lp.A.rows[c].entries[i-1]*x(i-1);
      Real q=lp.q(c)-axprev,p=lp.p(c)-axprev;
      Real a=lp.A.rows[c].entries[i];
      //solve q <= a*x <= p
      if(a < 0) {
	if(a*x[i] < q) 
	  x[i] = q/a;
      }
      else {
	if(a*x[i] > p) 
	  x[i] = p/a;
      }
    }
    if(x[i] == 0.0 && i != n) {
      printf("x[%d] is set to zero, x[%d]=%g\n",i,i-1,x[i-1]);
      for(int c=cfirst;c<cend;c++) {
	printf("' %g <= %g*%g + %g*%g <= %g\n",lp.q(c),lp.A(c,i-1),x[i-1],lp.A(c,i),xorig,lp.p(c));
      }
    }
    if(x[i] < 0.0 ) {
      if(x[i-1] == 0.0 ) {
	fprintf(stderr,"Warning: two subsequent x variables are forced to be negative? %d and %d\n",i-1,i);
	x[i] = lp.u(i);
      }
      else
	x[i] = 0.0; 
      // if x[i] becomes 0, enforce it to be zero, and then propagate backwards
      printf("Variable x[%d] forced to zero, from x[%d]=%g\n",i,i-1,x[i-1]);
      for(int c=cfirst;c<cend;c++) {
	printf("' %g <= %g*%g + %g*%g <= %g\n",lp.q(c),lp.A(c,i-1),x[i-1],lp.A(c,i),xorig,lp.p(c));
      }
      if(BACKWARDS_PROPAGATION) {
	int j=i;
	while(j > 0) {
	  //maximize x[j] s.t. the constraints are met
	  int cfirst=segToConstraints[j-1].first;
	  int cend=segToConstraints[j-1].second;
	  Real oldxj = x[j-1];
	  for(int c=cfirst;c<cend;c++) {
	    Assert(lp.A.rows[c].numEntries()==2);
	    Assert(lp.A.rows[c].find(j-1) != lp.A.rows[c].end());
	    Assert(lp.A.rows[c].find(j) != lp.A.rows[c].end());
	    Real ax = lp.A.rows[c].entries[j]*x(j);
	    Real q=lp.q(c)-ax,p=lp.p(c)-ax;
	    Real a=lp.A.rows[c].entries[j-1];
	    //solve q <= a*x <= p
	    if(a < 0) {
	      if(a*x[j-1] < q) 
		x[j-1] = q/a;
	    }
	    else {
	      if(a*x[j-1] > p) 
		x[j-1] = p/a;
	    }
	  }
	  if(x[j-1] == oldxj) break;
	  if(x[j-1] < 0.0) { x[j-1]=0; break; } 
	  j--;
	}
      }
    }
  }
  if(x[n-1] == 0.0 && x[n] == 0.0) 
    x[n-1] = lp.u(n-1);
  if(BACKWARDS_PROPAGATION) {
    int j=n;
    while(j > 0) {
      //maximize x[j] s.t. the constraints are met
      int cfirst=segToConstraints[j-1].first;
      int cend=segToConstraints[j-1].second;
      Real oldxj = x[j-1];
      for(int c=cfirst;c<cend;c++) {
	Assert(lp.A.rows[c].numEntries()==2);
	Assert(lp.A.rows[c].find(j-1) != lp.A.rows[c].end());
	Assert(lp.A.rows[c].find(j) != lp.A.rows[c].end());
	Real ax = lp.A.rows[c].entries[j]*x(j);
	Real q=lp.q(c)-ax,p=lp.p(c)-ax;
	Real a=lp.A.rows[c].entries[j-1];
	//solve q <= a*x <= p
	if(a < 0) {
	  if(a*x[j-1] < q) 
	    x[j-1] = q/a;
	}
	else {
	  if(a*x[j-1] > p) 
	    x[j-1] = p/a;
	}
      }
      if(x[j-1] == oldxj) break;
      if(x[j-1] < 0.0) { x[j-1]=0; break; } 
      j--;
    }
  }
  bool feasible=lp.IsFeasible(x);

  //use ds as temporary storage
  ds.resize(n+1);
  for(int i=0;i<=n;i++) {
    Assert(x[i] >= 0);
    ds[i] = Sqrt(x[i]);
  }
  Real T=0;
  Vector dT(n+1,0.0);
  for(int i=0;i<n;i++) {
    if(x[i] + x[i+1] <=0) {
      fprintf(stderr,"Warning: two subsequent x variables are inititalized to be negative? %d and %d\n",i,i+1);
      fprintf(stderr,"upper bounds %g and %g\n",lp.u(i),lp.u(i+1));
      lp.Print(cout);
      getchar();
      return false;
    }
    Assert(x[i]+x[i+1] > 0);
    T += 2*(paramdivs[i+1]-paramdivs[i])/(ds[i]+ds[i+1]);
    if(x[i]!=0)
      dT(i) -= (paramdivs[i+1]-paramdivs[i])/(ds[i]*Sqr(ds[i]+ds[i+1]));
    if(x[i+1]!=0)
      dT(i+1) -= (paramdivs[i+1]-paramdivs[i])/(ds[i+1]*Sqr(ds[i]+ds[i+1]));
  }
  lp.c = dT;
  //cout<<"Initial solution: "<<x<<endl;
  //cout<<"Gradient "<<dT<<endl;
  //printf("Initial time %g\n",T);

  Real xtol = 1e-5, ftol=1e-8;
  GLPKInterface glpk;
  glpk.Set(lp);

  //warm up the GLPK basis
  for(int i=0;i<n+1;i++)
    if(x(i) == lp.u(i)) glpk.SetVariableNonBasic(i,true);
  for(int i=1;i<=n;i++) {
    int cfirst=segToConstraints[i-1].first;
    int cend=segToConstraints[i-1].second;
    for(int c=cfirst;c<cend;c++) {
      Assert(lp.A.rows[c].numEntries()==2);
      Assert(lp.A.rows[c].find(i-1) != lp.A.rows[c].end());
      Assert(lp.A.rows[c].find(i) != lp.A.rows[c].end());
      Real axprev = lp.A.rows[c].entries[i-1]*x(i-1);
      Real q=lp.q(c)-axprev,p=lp.p(c)-axprev;
      Real a=lp.A.rows[c].entries[i];
      if(x(i) == q/a) {
	glpk.SetVariableBasic(i);
	glpk.SetRowNonBasic(c,false);
	break;
      }
      else if (x(i) == p/a) {
	glpk.SetVariableBasic(i);
	glpk.SetRowNonBasic(c,true);
	break;
      }
    }
  }

  int numIters = 0, maxIters = gSLPMaxIters;
  bool changed=true;
  while(changed && numIters < maxIters) {
    Vector xnext;
    LinearProgram::Result res=glpk.Solve(xnext);
    if(res == LinearProgram::Infeasible) {
      fprintf(stderr,"Warning, got an infeasible linear program???\n");
      break;
    }
    else if(res == LinearProgram::Unbounded) {
      fprintf(stderr,"Warning, got an unbounded linear program???\n");
      break;
    }
    else if(res == LinearProgram::Error) {
      fprintf(stderr,"Warning, linear program solver failed\n");
      break;
    }
    else {
      //numerical error cleanup?
      for(int i=0;i<=n;i++) {
	Assert(xnext[i] >= -1e-10);
	if(xnext[i] < 0) xnext[i] = 0;
      }
      for(int i=0;i<n;i++) {
	Assert(xnext[i] >= 0);
	Assert(xnext[i+1] >= 0);
	if(xnext[i]+xnext[i+1] == 0) {
	  fprintf(stderr,"Warning: two subsequent x variables are forced to be zero? %d and %d\n",i,i+1);
	  fprintf(stderr,"upper bounds %g and %g\n",lp.u(i),lp.u(i+1));	  
	  lp.Print(cout);
	  getchar();
	  return false;
	}
	Assert(xnext[i]+xnext[i+1] > 0);
      }
      numIters++;
      //cout<<"Solved solution: "<<x<<endl;
      Real Told = T;
      for(int i=0;i<=n;i++) 
	ds[i] = Sqrt(xnext[i]);
      T = 0.0;
      for(int i=0;i<n;i++) 
	T += 2*(paramdivs[i+1]-paramdivs[i])/(ds[i]+ds[i+1]);

      //found a feasible solution -- test convergence
      if(x.isEqual(xnext,xtol)) {
	x = xnext;
	changed=false;
	feasible=true;
	break;
      }

      changed = false;
      if(feasible) {
	//do a line search to make sure T is reduced
	if(T >= Told-ftol) {
	  Real len = x.distance(xnext);
	  while(len > xtol) {
	    xnext = (x+xnext)*0.5;
	    len *= 0.5;
	    for(int i=0;i<=n;i++) 
	      ds[i] = Sqrt(xnext[i]);
	    T = 0.0;
	    for(int i=0;i<n;i++) 
	      T += 2*(paramdivs[i+1]-paramdivs[i])/(ds[i]+ds[i+1]);
	    if(T < Told-ftol) {
	      changed = true;
	      break;
	    }
	  }
	  printf("SLP step %d size %g changed time from %g to %g\n",numIters,len,Told,T);
	}
	else 
	  printf("SLP step %d changed time from %g to %g\n",numIters,Told,T);
      }
      else {
	changed = true;
	printf("SLP step %d found feasible solution at time %g\n",numIters,T);
      }

      feasible = true;
      if(!changed || x.isEqual(xnext,xtol)) {
	x = xnext;
	changed=false;
	break;
      }
      x = xnext;

      //calc new gradient
      Vector oldDt = dT;
      dT.setZero();
      for(int i=0;i<n;i++) {
	if(x[i]!=0)
	  dT(i) -= (paramdivs[i+1]-paramdivs[i])/(ds[i]*Sqr(ds[i]+ds[i+1]));
	if(x[i+1]!=0)
	  dT(i+1) -= (paramdivs[i+1]-paramdivs[i])/(ds[i+1]*Sqr(ds[i]+ds[i+1]));
      }
      //cout<<"Change in gradient: "<<oldDt.distance(dT)<<endl;
      glpk.SetObjective(dT,true);
    }
  }
  if(changed)
    printf("SLP terminated after %d iterations with total time %g\n",numIters,T);
  else
    printf("SLP converged after %d iterations with total time %g\n",numIters,T);

  if(CHECK_SLP_BOUNDS) {
    vector<bool> anyNonBasic(n+1,false);
    vector<vector<int> > limitingConstraints(n+1);
    for(int i=0;i<=n;i++) {
      if(!glpk.GetVariableBasic(i)) {
	anyNonBasic[i] = true;
	limitingConstraints[i].push_back(-1); 
      }
    }
    for(int i=1;i<=n;i++) {
      int cfirst=segToConstraints[i-1].first;
      int cend=segToConstraints[i-1].second;
      for(int c=cfirst;c<cend;c++)
	if(glpk.GetRowBasic(c) == false) {
	  anyNonBasic[i-1] = true;
	  anyNonBasic[i] = true;
	  limitingConstraints[i-1].push_back(c);
	  limitingConstraints[i].push_back(c);
	}
    }
    for(int i=0;i<=n;i++)
      if(!anyNonBasic[i]) printf("Hmm, variable %d not involved in the basis\n",i);
    for(int i=0;i<=n;i++) {
      if(find(limitingConstraints[i].begin(),limitingConstraints[i].end(),-1)!=limitingConstraints[i].end())
	printf("v,");
      else {
	printf("a[");
	for(size_t j=0;j<limitingConstraints[i].size();j++) {
	  int seg = limitingConstraints[i][j]/(d*8);
	  int ind = limitingConstraints[i][j]%(d*8);
	  printf("%d %d;",seg,ind);
	}
	printf("],");
      }
    }
    printf("\n");
  }

  params.resize(paramdivs.size());
  copy(paramdivs.begin(),paramdivs.end(),params.begin());
  times.resize(params.size());
  times[0] = 0;
  for(int i=0;i<=n;i++) 
    ds[i] = Sqrt(x[i]);
  for(int i=0;i<n;i++) {
    Real dt = 2*(paramdivs[i+1]-paramdivs[i])/(ds[i]+ds[i+1]);    
    times[i+1]=times[i]+dt;
  }
  return !changed;
}




bool OptimizeTimeScaling(const GeneralizedCubicBezierSpline& path,
			 const Vector& vmin,const Vector& vmax,const Vector& amin,const Vector& amax,
			 TimeScaling& scaling)
{
#if POLYNOMIAL_DERIV_BOUNDS
  //new style
  bool res=scaling.SolveMinTime(vmin,vmax,amin,amax,path,0.0,0.0);
  if(res) Assert(scaling.ds.front()==0.0 && scaling.ds.back()==0.0);
  return res;

#else
  Assert(!path.segments.empty());
  vector<Real> divs(path.segments.size()+1);
  divs[0]=0;
  for(size_t i=0;i<path.segments.size();i++)
    divs[i+1]=divs[i]+path.durations[i];
  size_t n=path.segments.size();
  vector<Vector> vmins(n),vmaxs(n),amins(n),amaxs(n);
  for(size_t i=0;i<n;i++) {
#if INTERVAL_DERIV_BOUNDS
    path.segments[i].GetDerivBounds(vmins[i],vmaxs[i],amins[i],amaxs[i]);
#else
    //just do collocation at the start points
    path.segments[i].Deriv(0,vmins[i]);
    vmaxs[i] = vmins[i];
    path.segments[i].Accel(0,amins[i]);
    amaxs[i] = amins[i];
#endif //INTERVAL_DERIV_BOUNDS

    vmins[i] /= path.durations[i];
    vmaxs[i] /= path.durations[i];
    amins[i] /= Sqr(path.durations[i]);
    amaxs[i] /= Sqr(path.durations[i]);

    for(size_t j=0;j<vmins[i].n;j++)
      if(vmins[i][j] < -vWarningThreshold || vmaxs[i][j] > vWarningThreshold || amins[i][j] < -aWarningThreshold || amaxs[i][j] > aWarningThreshold ) {
	printf("Spline segment %d deriv bounds seem odd on entry %d\n",i,j);
	printf("x0 %g, x1 %g, x2 %g, x3 %g\n",path.segments[i].x0[j],path.segments[i].x1[j],path.segments[i].x2[j],path.segments[i].x3[j]);
	printf("duration %g\n",path.durations[i]);
	printf("Deriv bounds %g %g, accel bounds %g %g\n",vmins[i][j],vmaxs[i][j],amins[i][j],amaxs[i][j]);
	printf("Press enter to continue\n");
	getchar();
      }
  }
  bool res=scaling.SolveMinTime(vmin,vmax,amin,amax,divs,vmins,vmaxs,amins,amaxs,0.0,0.0);
  if(res) Assert(scaling.ds.front()==0.0 && scaling.ds.back()==0.0);
  return res;
#endif  //POLYNOMIAL_BOUNDING
}



bool TimeScaledBezierCurve::OptimizeTimeScaling(const Vector& vmin,const Vector& vmax,const Vector& amin,const Vector& amax)
{
  return ::OptimizeTimeScaling(path,vmin,vmax,amin,amax,timeScaling);
}

void TimeScaledBezierCurve::GetPiecewiseLinear(std::vector<Real>& times,std::vector<Config>& milestones) const
{
  //times = timeScaling.times;
  milestones.resize(path.segments.size()*3+1);
  times.resize(milestones.size());
  times[0] = timeScaling.times[0];
  milestones[0] = path.segments[0].x0;
  for(size_t i=0;i<path.segments.size();i++) {
    Real dt=timeScaling.times[i+1]-timeScaling.times[i];
    times[i*3+1] = timeScaling.times[i]+dt/3.0;
    times[i*3+2] = timeScaling.times[i]+dt*2.0/3.0;
    times[i*3+3] = timeScaling.times[i+1];
    milestones[i*3+1] = path.segments[i].x1;
    milestones[i*3+2] = path.segments[i].x2;
    milestones[i*3+3] = path.segments[i].x3;
  }
}

void TimeScaledBezierCurve::GetDiscretizedPath(Real dt,std::vector<Config>& milestones) const
{
  Real T = EndTime();
  Real t=0;
  //printf("Discretizing to %d milestones\n",(int)Ceil(T/dt));
  milestones.reserve((int)Ceil(T/dt));
  milestones.resize(0);
  Config temp;
  while(t < T) {
    Eval(t,temp);
    milestones.push_back(temp);
    t += dt;
  }
  Eval(T,temp);
  milestones.push_back(temp);
}

Real TimeScaledBezierCurve::EndTime() const
{
  return timeScaling.times.back();
}

void TimeScaledBezierCurve::Eval(Real t,Vector& x) const
{
  int seg=timeScaling.TimeToSegment(t);
  if(seg+1 >= (int)timeScaling.params.size()) {
    x = path.segments.back().x3;
    return;
  }
  Real s=timeScaling.TimeToParam(seg,t);
  if(pathSegments.empty()) {
    //assume the time scaling discretization is the same as that of the path
    Assert(timeScaling.times.size()==path.segments.size()+1);
    Real u=(s-timeScaling.params[seg])/path.durations[seg];
    path.segments[seg].Eval(u,x);
  }
  else {
    //locate the path segment
    Real u;
    Assert(s >= pathSegments.front() && s <= pathSegments.back());
    int seg = pathSegments.Map(s,u);
    if(seg == (int)path.segments.size()) {
      seg--;
      u = 1;
    }
    Assert(seg >= 0 && seg < (int)path.segments.size());
    path.segments[seg].Eval(u,x);
  }
}

void TimeScaledBezierCurve::Deriv(Real t,Vector& dx) const
{
  int seg=timeScaling.TimeToSegment(t);
  if(seg+1 >= (int)timeScaling.params.size()) {
    dx.resize(path.segments.back().x3.n,Zero);
    return;
  }
  Real s=timeScaling.TimeToParam(seg,t);
  Real dsdt=timeScaling.TimeToParamDeriv(seg,t);
  Real u;
  if(pathSegments.empty()) {
    Assert(timeScaling.times.size()==path.segments.size()+1); 
    u=(s-timeScaling.params[seg])/path.durations[seg];
  }
  else {
    //locate the path segment
    Assert(s >= pathSegments.front() && s <= pathSegments.back());
    seg = pathSegments.Map(s,u);
    if(seg == (int)path.segments.size()) {
      seg--;
      u = 1;
    }
    Assert(seg >= 0 && seg < (int)path.segments.size());
  }
  path.segments[seg].Deriv(u,dx);
  dx *= dsdt/path.durations[seg];
}

void TimeScaledBezierCurve::Accel(Real t,Vector& ddx) const
{
  int seg=timeScaling.TimeToSegment(t);
  if(seg+1 >= (int)timeScaling.params.size()) {
    ddx.resize(path.segments.back().x3.n,Zero);
    return;
  }
  Real s=timeScaling.TimeToParam(seg,t);
  Real dsdt=timeScaling.TimeToParamDeriv(seg,t);
  Real ddsdt2=timeScaling.TimeToParamAccel(seg,t);
  Real u;
  if(pathSegments.empty()) {
    Assert(timeScaling.times.size()==path.segments.size()+1);
    u=(s-timeScaling.params[seg])/path.durations[seg];
  }
  else {
    //locate the path segment
    Assert(s >= pathSegments.front() && s <= pathSegments.back());
    seg = pathSegments.Map(s,u);
    if(seg == (int)path.segments.size()) {
      seg--;
      u = 1;
    }
    Assert(seg >= 0 && seg < (int)path.segments.size());
  }
  Vector dx;
  path.segments[seg].Deriv(u,dx);
  path.segments[seg].Accel(u,ddx);
  ddx *= Sqr(dsdt/path.durations[seg]);
  ddx.madd(dx,ddsdt2/path.durations[seg]);
}

void TimeScaledBezierCurve::Plot(const char* fn,const Vector& vmin,const Vector& vmax,const Vector& amin,const Vector& amax,Real res)
{
  ofstream out(fn,ios::out);
  if(!out) {
    fprintf(stderr,"TimeScaledBezierCurve::Plot(): Error opening %s\n",fn);
    return;
  }
  out<<"s,seg,t,ds,dds,dsmax,ddsmin(ds),ddsmax(ds),max dd saturation"<<endl;
  Real s=0;
  Real smax = path.TotalTime();
  Vector dxds,ddxds;
  while(s < smax) {
    Real t=timeScaling.ParamToTime(s);
    int seg=timeScaling.ParamToSegment(s);
    Real ds = timeScaling.TimeToParamDeriv(seg,t);
    Real dds = timeScaling.TimeToParamAccel(seg,t);
    path.Deriv(s,dxds);
    path.Accel(s,ddxds);
    Vector ddxdt;
    Accel(t,ddxdt);
    /*
    Assert(ddxdt.isEqual(ddxds*Sqr(ds) + dds*dxds,1e-2));
    Assert(dxds.n == vmin.n && dxds.n == vmax.n);
    Assert(dxds.n == amin.n && dxds.n == amax.n);
    */
    Real dsmax = Inf;
    Real ddsmin = -Inf, ddsmax = Inf;
    for(int i=0;i<dxds.n;i++) {
      if(dxds(i)*dsmax > vmax(i))
	dsmax = vmax(i)/dxds(i);
      if(dxds(i)*dsmax < vmin(i))
	dsmax = vmin(i)/dxds(i);
      Real a = ddxds(i)*Sqr(ds);
      if(a + ddsmin*dxds(i) < amin(i)) 
	ddsmin = (amin(i) - a) / dxds(i);
      if(a + ddsmax*dxds(i) > amax(i)) 
	ddsmax = (amax(i) - a) / dxds(i);
      if(a + ddsmax*dxds(i) < amin(i)) 
	ddsmax = (amin(i) - a) / dxds(i);
      if(a + ddsmin*dxds(i) > amax(i)) 
	ddsmin = (amax(i) - a) / dxds(i);
    }
    Real maxASat = 0;
    for(int i=0;i<amin.n;i++) {
      Real a = ddxds(i)*Sqr(ds) + dds*dxds(i);
      if(a/amin(i) > maxASat) maxASat = a/amin(i);
      if(a/amax(i) > maxASat) maxASat = a/amax(i);
    }
    out<<s<<","<<seg<<","<<t<<","<<ds<<","<<dds<<","<<dsmax<<","<<ddsmin<<","<<ddsmax<<","<<maxASat<<endl;
    s += res;
  }
}




















CustomTimeScaling::CustomTimeScaling()
{
}

void CustomTimeScaling::SetPath(const GeneralizedCubicBezierSpline& path,const vector<Real>& paramDivs)
{
  Assert(paramDivs.size() >= 2);
  this->paramDivs = paramDivs;
  paramSections.resize(0);
  traj.path = path;

  //create collocation points
  xs.resize(paramDivs.size());
  dxs.resize(paramDivs.size());
  ddxs.resize(paramDivs.size());
  for(size_t i=0;i<paramDivs.size();i++) {
    path.Eval(paramDivs[i],xs[i]);
    path.Deriv(paramDivs[i],dxs[i]);
    path.Accel(paramDivs[i],ddxs[i]);
  }

  ds2ddsConstraintNormals.resize(paramDivs.size());
  ds2ddsConstraintOffsets.resize(paramDivs.size());
  dsmax.resize(paramDivs.size());
  fill(dsmax.begin(),dsmax.end(),Inf);

  if(SAVE_COLLOCATION_POINTS) {
    printf("Saving collocation points to xs.txt, dxs.txt, ddxs.txt\n");
    {
      ofstream out("xs.txt",ios::out);
      for(size_t i=0;i<xs.size();i++)
	out<<xs[i]<<endl;
    }
    {
      ofstream out("dxs.txt",ios::out);
      for(size_t i=0;i<dxs.size();i++)
	out<<dxs[i]<<endl;
    }
    {
      ofstream out("ddxs.txt",ios::out);
      for(size_t i=0;i<ddxs.size();i++)
	out<<ddxs[i]<<endl;
    }
  }
}


void CustomTimeScaling::SetPath(const vector<GeneralizedCubicBezierSpline>& paths,const vector<Real>& paramDivs)
{
  Assert(paramDivs.size() >= 2);
  this->paramDivs = paramDivs;
  TimeSegmentation sectiondivs;
  sectiondivs.push_back(0);
  for(size_t i=0;i<paths.size();i++) {
    if(paths[i].durations.empty())
      sectiondivs.push_back(sectiondivs.back()+1.0/paths.size());
    else
      sectiondivs.push_back(sectiondivs.back()+paths[i].TotalTime());
  }
  paramSections.resize(paramDivs.size());
  for(size_t i=0;i<paramDivs.size();i++) {
    Real u;
    paramSections[i] = sectiondivs.Map(paramDivs[i],u);
    Assert(paramSections[i] >= 0 && paramSections[i] < paths.size());
  }

  //create trajectory storage
  traj.path.segments.resize(0);
  traj.path.durations.resize(0);
  for(size_t i=0;i<paths.size();i++) {
    if(paths[i].durations.empty()) {
      if(!traj.path.durations.empty())
	fprintf(stderr,"Error, cannot mix timed and untimed sections in multipath\n");
      Assert(traj.path.durations.empty());
    }
    traj.path.Concat(paths[i]);
  }

  //create collocation points
  xs.resize(paramDivs.size());
  dxs.resize(paramDivs.size());
  ddxs.resize(paramDivs.size());
  for(size_t i=0;i<paramDivs.size();i++) {
    if(traj.path.durations.empty())
      Assert(paramDivs[i] >= 0.0 && paramDivs[i] <= 1.0);
    else
      Assert(paramDivs[i] >= 0.0 && paramDivs[i] <= traj.path.TotalTime());
    traj.path.Eval(paramDivs[i],xs[i]);
    traj.path.Deriv(paramDivs[i],dxs[i]);
    traj.path.Accel(paramDivs[i],ddxs[i]);
  }

  ds2ddsConstraintNormals.resize(paramDivs.size());
  ds2ddsConstraintOffsets.resize(paramDivs.size());
  dsmax.resize(paramDivs.size());
  fill(dsmax.begin(),dsmax.end(),Inf);

  if(SAVE_COLLOCATION_POINTS) {
    printf("Saving collocation points to xs.txt, dxs.txt, ddxs.txt\n");
    {
      ofstream out("xs.txt",ios::out);
      for(size_t i=0;i<xs.size();i++)
	out<<xs[i]<<endl;
    }
    {
      ofstream out("dxs.txt",ios::out);
      for(size_t i=0;i<dxs.size();i++)
	out<<dxs[i]<<endl;
    }
    {
      ofstream out("ddxs.txt",ios::out);
      for(size_t i=0;i<ddxs.size();i++)
	out<<ddxs[i]<<endl;
    }
  }
}

void CustomTimeScaling::SetStartStop()
{
  dsmax[0] = 0;
  dsmax.back()=0;
}

void CustomTimeScaling::SetVelocityBounds(const Vector& vmin,const Vector& vmax)
{
  int d=xs[0].n;

  //velocity bounds
  for(size_t i=0;i<dsmax.size();i++) {  
    for(int j=0;j<d;j++) {
      dsmax[i] = Min(dsmax[i],Max(vmax[j]/dxs[i][j],vmin[j]/dxs[i][j]));
    }
    if(!IsFinite(dsmax[i])) dsmax[i] = Inf;
  }
}

void CustomTimeScaling::SetAccelerationBounds(const Vector& amin,const Vector& amax)
{
  int d=xs[0].n;

  //acceleration bounds
  //amin <= ddx*ds^2 + dx*dds <= amax
  for(size_t i=0;i<paramDivs.size();i++) { 
    for(int j=0;j<d;j++) {
      Real dx = dxs[i][j];
      Real ddx = ddxs[i][j];
      ds2ddsConstraintNormals[i].push_back(Vector2(ddx,dx));
      ds2ddsConstraintOffsets[i].push_back(amax[j]);
      ds2ddsConstraintNormals[i].push_back(Vector2(-ddx,-dx));
      ds2ddsConstraintOffsets[i].push_back(-amin[j]);
    }
  }
}

bool CustomTimeScaling::Optimize()
{
  return SolveSLP(paramDivs,dsmax,ds2ddsConstraintNormals,ds2ddsConstraintOffsets,traj);
}




TorqueTimeScaling::TorqueTimeScaling(DynamicsEquationBase* _dynamics)
  :dynamics(_dynamics)
{}

void TorqueTimeScaling::Init(const GeneralizedCubicBezierSpline& path,const vector<Real>& colocationParams,const Vector& torquemin,const Vector& torquemax)
{
  CustomTimeScaling::SetPath(path,colocationParams);

  //kinetic energy, coriolis force, gravity
  Vector C,G;
  //coefficients of time scaling
  Vector a,b,c;
  for(size_t i=0;i<paramDivs.size();i++) {
    dynamics->MulInertia(xs[i],dxs[i],a);
    dynamics->MulInertia(xs[i],ddxs[i],b);
    dynamics->EvalCoriolis(xs[i],dxs[i],C);
    dynamics->EvalGravity(xs[i],G);
    b += C;
    c = G;
    //Torque is given by a*dds + b*ds^2 + c = t
    for(int j=0;j<torquemax.n;j++) {
      Real tmax = torquemax[j];
      Real tmin = torquemin[j];
      //b*ds^2 + a*dds <= tmax - c
      //-b*ds^2 - a*dds <= -tmin + c
      ds2ddsConstraintNormals[i].push_back(Vector2(b(j),a(j)));
      ds2ddsConstraintOffsets[i].push_back(tmax-c(j));
      ds2ddsConstraintNormals[i].push_back(Vector2(-b(j),-a(j)));
      ds2ddsConstraintOffsets[i].push_back(-tmin+c(j));
    }
  }
}






ContactTimeScaling::ContactTimeScaling(DynamicsEquationBase* _dynamics)
  :dynamics(_dynamics),forceRobustness(0)
{
}

struct FrictionConePolygon
{
	void set(int k,const Vector3& n,Real kFriction);
	bool contains(const Vector3& f) const;
	bool onBoundary(const Vector3& f) const;

	std::vector<Vector3> edges;
	std::vector<Vector3> planes;
};

size_t NumContactPoints(const ContactFormation& s)
{
  size_t n = 0;
  for(size_t i=0;i<s.contacts.size();i++)
    n += s.contacts[i].size();
  return n;
}

/// Computes vectors y,z s.t. x,y,z is a (minimal) rotation from the 
/// standard basis.
inline void GetCanonicalBasis(const Vector3& x,Vector3&y,Vector3& z) {
  Real scale;
  if(FuzzyEquals(x.x,1.0)) scale = 0;
  else if(FuzzyEquals(x.x,-1.0)) {  //A complete flip of the basis
    y.set(0.0,-1.0,0.0);
    z.set(0.0,0.0,1.0);
    return;
  }
  else scale = (1.0-x.x)/(1.0-Sqr(x.x));
  y.x = -x.y;
  y.y = x.x + scale*Sqr(x.z);
  y.z = -scale*x.y*x.z;
  z.x = -x.z;
  z.y = -scale*x.y*x.z;
  z.z = x.x + scale*Sqr(x.y);
}

void FrictionConePolygon::set(int k,const Vector3& n,Real kFriction)
{
  Assert(k >= 3);
  Assert(kFriction > 0);
  Vector3 xb,yb;
  GetCanonicalBasis(n,xb,yb);
  edges.resize(k);
  planes.resize(k);
  for(int i=0;i<k;i++) {
    Real theta = Real(i)*TwoPi/k;
    edges[i] = (Cos(theta)*xb + Sin(theta)*yb)*kFriction + n;
  }
  for(int i=0;i<k;i++) {
    planes[i].setCross(edges[i],edges[(i+1)%k]);
    planes[i].inplaceNormalize();
    //Assert(dot(planes[i],n) > 0);
  }
}

void GetFrictionConePlanes(const ContactFormation& s,int nFrictionConeEdges,SparseMatrix& A)
{
  int nc=NumContactPoints(s);
  A.resize(nFrictionConeEdges*nc,nc*3);
  A.setZero();

  int m=0;
  int p=0;
  for(size_t i=0;i<s.contacts.size();i++) {
    const vector<ContactPoint>& h=s.contacts[i];
    for(size_t j=0;j<h.size();j++,p++) {
      const ContactPoint& pt=h[j];
      FrictionConePolygon fc;
      fc.set(nFrictionConeEdges,pt.n,pt.kFriction);
      for(int i=0;i<nFrictionConeEdges;i++,m++) {
	A(m,p*3) = -fc.planes[i].x;
	A(m,p*3+1) = -fc.planes[i].y;
	A(m,p*3+2) = -fc.planes[i].z;
      }
    }
  }
}

bool ContactTimeScaling::Init(const vector<GeneralizedCubicBezierSpline>& paths,const vector<Real>& colocationParams,
	    const vector<ContactFormation>& stances,
	    const Vector& tmin,const Vector& tmax,
	    const Vector& amin,const Vector& amax,
	    const Vector& vmin,const Vector& vmax,
	    int numFCEdges)
{
  CustomTimeScaling::SetPath(paths,colocationParams);
  SetVelocityBounds(vmin,vmax);

  int n=xs[0].n;
  int oldSection = -1;
  LinearProgram_Sparse lp;
  Vector C,G;
  //coefficients of time scaling
  Vector a,b,c;
  bool feasible=true;
  Matrix J(3,n) ;
  for(size_t i=0;i<paramDivs.size();i++) {
    Assert(paramSections[i] >= 0 && paramSections[i] < (int)paths.size());
    const ContactFormation& stance = stances[paramSections[i]];
    if(paramSections[i] != oldSection) {
      //reconstruct LP for the contacts in this section

      //now formulate the LP.  Variable 0 is ds^2, variable 1 is dds
      //rows 1-n are torque max
      //rows n+1 - 2n are acceleration max
      //rows 2n+1 + 2n+numFCEdges*nc are the force constraints
      //vel max is encoded in the velocity variable
      int nc = NumContactPoints(stance);
#if TEST_NO_CONTACT
      nc = 0;
#endif // TEST_NO_CONTACT
      lp.Resize(n*2+numFCEdges*nc,2+3*nc);
      lp.A.setZero();
      lp.c.setZero();
      //fill out wrench matrix FC*f <= 0
#if !TEST_NO_CONTACT
      SparseMatrix FC;
      GetFrictionConePlanes(stance,numFCEdges,FC);
      lp.A.copySubMatrix(n*2,2,FC);
      for(int j=0;j<FC.m;j++)
	lp.p(n*2+j) = -forceRobustness;
#endif // !TEST_NO_CONTACT

      lp.l(0) = 0.0;
      lp.l(1) = -Inf;

      oldSection = paramSections[i];
    }
    //configuration specific 
    dynamics->MulInertia(xs[i],dxs[i],a);
    dynamics->MulInertia(xs[i],ddxs[i],b);
    dynamics->EvalCoriolis(xs[i],dxs[i],C);
    dynamics->EvalGravity(xs[i],G);
    b += C;
    c = G;

    //|a dds + b ds^2 + c - Jtf| <= torquemax*scale+shift
    /*
    for(int j=0;j<a.n;j++) {
      if(j==a.n/2)
	printf("[%g] <= [%g]*ds^2 + [%g]*dds + [%g] - Jtf <= [%g]\n",tmin[j],b(j),a(j),c(j),tmax[j]);
      else
	printf("[%g]    [%g]        [%g]       [%g]          [%g]\n",tmin[j],b(j),a(j),c(j),tmax[j]);
    }
    getchar();
    */
    for(int j=0;j<a.n;j++) {
      lp.A(j,0) = b(j);
      lp.A(j,1) = a(j);
      lp.p(j) = tmax[j]-c(j);
      lp.q(j) = tmin[j]-c(j);
    }
#if TEST_NO_CONTACT
    lp.p.set(Inf);
    lp.q.set(-Inf);
#else
    //fill out jacobian transposes
    int ccount=0;
    for(size_t l=0;l<stance.links.size();l++) {
      int link = stance.links[l];
      int target = (stance.targets.empty() ? -1 : stance.targets[l]);
      for(size_t c=0;c<stance.contacts[l].size();c++,ccount++) {
	const Vector3& p=stance.contacts[l][c].x;
	dynamics->EvalJacobian(p,link,target,xs[i],J);
	Assert(J.m == 3 && J.n == n);
	for(int j=0;j<3;j++)
	  for(int k=0;k<n;k++)
	    if(J(j,k) != 0)
	      lp.A(k,2+ccount*3+j) = -J(j,k);
      }
    }
    Assert(ccount == (int)NumContactPoints(stance));
#endif //TEST_NO_CONTACT

    //fill out acceleration constraint |ddx*ds^2 + dx*dds| <= amax
    for(int j=0;j<a.n;j++) {
      lp.q(a.n+j) = amin(j);
      lp.p(a.n+j) = amax(j);
      lp.A(a.n+j,0) = ddxs[i][j];
      lp.A(a.n+j,1) = dxs[i][j];
    }

    //compute upper bounds from vel and acc max
    lp.u(0) = Inf; lp.u(1) = Inf;
    for(int j=0;j<a.n;j++) {
      if(dxs[i][j] < 0)
	lp.u(0) = Min(lp.u(0),Sqr(vmin(j)/dxs[i][j]));
      else
	lp.u(0) = Min(lp.u(0),Sqr(vmax(j)/dxs[i][j]));
    }

    //expand polytope
    Geometry::PolytopeProjection2D proj(lp);
    Geometry::UnboundedPolytope2D poly;
    proj.Solve(poly);
    if(poly.vertices.empty()) {
      //problem is infeasible?
      printf("Problem is infeasible at segment %d\n",i);
      cout<<"x = "<<xs[i]<<endl;
      cout<<"dx = "<<dxs[i]<<endl;
      cout<<"ddx = "<<ddxs[i]<<endl;
      lp.Print(cout);
      getchar();
      feasible=false;
    }

    ds2ddsConstraintNormals[i].resize(poly.planes.size());
    ds2ddsConstraintOffsets[i].resize(poly.planes.size());
    //cout<<"Constraints: "<<endl;
    for(size_t j=0;j<poly.planes.size();j++) {
      ds2ddsConstraintNormals[i][j] = poly.planes[j].normal;
      ds2ddsConstraintOffsets[i][j] = poly.planes[j].offset;
      //cout<<poly.planes[j].normal<<", "<<poly.planes[j].offset<<endl;
    }
    //getchar();
  }
  //done!
  return feasible;
}
