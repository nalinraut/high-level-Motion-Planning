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
#include <mintos/ConstrainedInterpolator.h>
#include <mintos/spline/SplineInterpolate.h>
#include <mintos/math/interpolate.h>
#include <mintos/misc/errors.h>
#include <mintos/Config.h>
#include "primitives.h"
#include <list>
#include <queue>
using namespace Math3D;

namespace Mintos {

#define CONDITION_INITIAL_TANGENTS 0
#define CONDITION_MIDDLE_TANGENTS 0
#define OPTIMIZE_TANGENTS 0

  inline Real Distance(GeodesicManifold* space,const Config& a,const Config& b)
  {
    return (space ? space->Distance(a,b) : a.distance(b));
  }

  inline void Interpolate(GeodesicManifold* space,const Config& a,const Config& b,Real u,Config& x)
  {
    if(space) space->Interpolate(a,b,u,x);
    else interpolate(a,b,u,x);
  }

//scales the tangents ta and tb so that curvature is minimized
//a = -2(qb-qa) + ta + tb
//b = 3(qb-qa) - 2ta - tb
//dC/dta = -b = -3(qb-qa) + 2ta + tb 
//dC/dtb = 3a + b = -3(qb-qa) + ta + 2tb 
//Find scales ca,cb such that ta = ta0*ca, tb = tb0*cb
//dC / dca = ta0^T dC/dta = ta0^T -3(qb-qa) + 2ca ta0^Tta0 + cb ta0^T tb0 = 0
//dC / dcb = tb0^T dC/dtb = tb0^T -3(qb-qa) + ca tb0^Tta0 + 2cb tb0^T tb0 = 0
/*
  3 ta0^T (qb-qa) = 2ca ta0^Tta0 + cb ta0^T tb0  
  3 tb0^T (qb-qa) = ca tb0^Tta0 + 2 cb tb0^T tb0 
 */
void ConditionTangents(GeneralizedCubicBezierCurve& curve)
{
  Vector ta,tb;
  curve.Deriv(0,ta);
  curve.Deriv(1,tb);
  Vector d;
  if(curve.space)
    curve.space->InterpolateDeriv(curve.x0,curve.x3,0,d);
  else
    d.sub(curve.x3,curve.x0);
  Matrix2 A,Ainv;
  A(0,0) = 2.0*dot(ta,ta);
  A(0,1) = A(1,0) = dot(ta,tb);
  A(1,1) = 2.0*dot(tb,tb);
  Vector2 b;
  b.x = 3.0*dot(ta,d);
  b.y = 3.0*dot(tb,d);
  Vector2 scale;
  bool res=Ainv.setInverse(A);
  if(!res) return;
  scale = Ainv*b;
  ta *= scale.x;
  tb *= scale.y;
  /*
  cout<<"A: "<<A<<endl;
  cout<<"b: "<<b<<endl;
  cout<<"Tangent scales: "<<scale<<endl;
  */
  curve.SetNaturalTangents(ta,tb);
}


//dC/dt2 = -b2 + 3a1 + b1 = 
//  -3(p3-p2-p1+p2) + t1 + 4t2 + t3
//scale t2=t20*c
//dC/dc = t20^T [-3(p3-p1) + t1 + 4t2 + 4t3] = 0
//0.25* [t20^T 3(p3-p1) - t20^T(t1+t3)] = c*t20^T t20
//3/4 t20^T (p3-1/3*t3 - (p1+1/3t1)) = c*t20^T t20
void ConditionMiddleTangent(GeneralizedCubicBezierCurve& c1,GeneralizedCubicBezierCurve& c2)
{
  /*
  Vector a;
  c1.Accel(0,a);
  Real oldAccelStart = a.norm();
  c1.Accel(1,a);
  Real oldAccelMid = a.norm();
  c2.Accel(0,a);
  Real oldAccelMid2 = a.norm();
  c2.Accel(1,a);
  Real oldAccelEnd = a.norm();
  */

  Vector t20;
  c1.Deriv(1,t20);
  /*
  //HACK -- measure only for non-free floating robot joints
  for(int i=0;i<6;i++) t20(i) = 0;
  */
  if(t20.normSquared() < 1e-6) return;
  Vector topt1,topt2;
  if(c1.space) {
    c1.space->InterpolateDeriv(c1.x1,c1.x3,1.0,topt1);
    c2.space->InterpolateDeriv(c2.x0,c2.x2,0.0,topt2);
  }
  else {
    topt1 = (c1.x3-c1.x1);
    topt2 = (c2.x2-c2.x0);
  }
  Vector v = (topt2+topt1)*0.25;
  Real c = v.dot(t20)/dot(t20,t20);
  if(c < 0.1 || c > 0.5) {
    cout<<"Middle tangent scale (ideal 1/3): "<<c<<endl;
    getchar();
  }
  if(c1.space)
    c1.space->Integrate(c1.x3,t20*(-c),c1.x2);
  else
    c1.x2 = c1.x3 - c*t20;
  if(c2.space)
    c2.space->Integrate(c2.x0,t20*(-c),c2.x1);
  else
    c2.x1 = c2.x0 + c*t20;

  /*
  c1.Accel(0,a);
  Real newAccelStart = a.norm();
  c1.Accel(1,a);
  Real newAccelMid = a.norm();
  c2.Accel(0,a);
  Real newAccelMid2 = a.norm();
  c2.Accel(1,a);
  Real newAccelEnd = a.norm();
  */
  //printf("Accel change: %g %g %g %g -> %g %g %g %g\n",oldAccelStart,oldAccelMid,oldAccelMid2,oldAccelEnd,newAccelStart,newAccelMid,newAccelMid2,newAccelEnd);
  //getchar();
}

ConstrainedInterpolator::ConstrainedInterpolator(GeodesicManifold* _space,VectorFieldFunction* _constraint)
  :space(_space),constraint(_constraint),inequalities(NULL),maxNewtonIters(10),ftol(1e-4),xtol(1e-3),maxGrowth(0.9),solver(_constraint)
{}


bool ConstrainedInterpolator::Project(Config& x)
{
  if(inequalities) FatalError("TODO: ConstrainedInterpolator with inequalities");
  if(!constraint) return true;
  solver.tolf = ftol;
  solver.tolx = solver.tolmin = ftol*1e-2;
  solver.verbose = 0;
  if(!xmin.empty()) {
    solver.bmin.setRef(xmin);
    solver.bmax.setRef(xmax);
  }
  int iters=maxNewtonIters;
  solver.x = x;
  if(!solver.GlobalSolve(iters)) return false;
  x = solver.x;
  return true;
}

struct Segment
{
  inline bool operator < (const Segment& s) const { return length<s.length; }
  
  list<Config>::iterator prev;
  Real length;
};

void ConstrainedInterpolator::ConstraintValue(const Config& x,Vector& val)
{
  (*constraint)(x,val);
}

bool ConstrainedInterpolator::Make(const Config& qa,const Config& qb,vector<Config>& path,bool checkConstraints)
{
  Vector temp(constraint->NumDimensions());
  ConstraintValue(qa,temp);
  if(temp.maxAbsElement() > ftol) {
    fprintf(stderr,"ConstrainedInterpolator: Warning, initial point a is not on manifold, error %g\n",temp.maxAbsElement());
  }
  ConstraintValue(qb,temp);
  if(temp.maxAbsElement() > ftol) {
    fprintf(stderr,"ConstrainedInterpolator: Warning, initial point b is not on manifold, error %g\n",temp.maxAbsElement());
  }

  list<Config> lpath;
  lpath.push_back(qa);
  lpath.push_back(qb);
  priority_queue<Segment,vector<Segment> > q;
  Segment s;
  s.prev = lpath.begin();
  s.length = Distance(space,qa,qb);
  q.push(s);

  Config x;
  while(!q.empty()) {
    s=q.top(); q.pop();
    if(s.length <= xtol) continue;
    list<Config>::iterator a = s.prev;
    list<Config>::iterator b=a; b++;
    Interpolate(space,*a,*b,0.5,x);
    if(!Project(x)) {
      cout<<"Unable to project "<<x<<endl;
      cout<<"Midpoint "<<*a<<" -> "<<*b<<endl;
      return false;
    }
    if(checkConstraints && space && !space->IsFeasible(x)) {
      cout<<"Infeasible configuration "<<x<<endl;
      return false;
    }

    list<Config>::iterator m=lpath.insert(b,x);

    //insert the split segments back in the queue
    Real l1=Distance(space,*a,x);
    Real l2=Distance(space,x,*b);
    if(l1 > 0.5*(1+maxGrowth)*s.length) {
      cout<<"Excessive growth: "<<l1<<" > "<<0.5*(1+maxGrowth)*s.length<<endl;
      return false;
    }
    if(l2 > 0.5*(1+maxGrowth)*s.length) {
      cout<<"Excessive growth: "<<l2<<" > "<<0.5*(1+maxGrowth)*s.length<<endl;
      return false;
    }
    s.prev = a;
    s.length = l1;
    if(s.length > xtol) q.push(s);

    s.prev = m;
    s.length = l2;
    if(s.length > xtol) q.push(s);
  }

  //read out the path
  path.resize(lpath.size());
  size_t k=0;
  for(list<Config>::iterator i=lpath.begin();i!=lpath.end();i++,k++) {
    path[k] = *i;
  }
  return true;
}









struct Segment2
{
  inline bool operator < (const Segment2& s) const { return length<s.length; }
  
  list<pair<GeneralizedCubicBezierCurve,double> >::iterator prev;
  Real length;
};

SmoothConstrainedInterpolator::SmoothConstrainedInterpolator(GeodesicManifold* _space,VectorFieldFunction* _constraint)
  :space(_space),constraint(_constraint),inequalities(NULL),maxNewtonIters(10),ftol(1e-4),xtol(1e-3),maxGrowth(0.9),solver(_constraint)
{}

bool SmoothConstrainedInterpolator::Make(const Config& a,const Config& b,
	  GeneralizedCubicBezierSpline& path,
	  bool checkConstraints)
{
  //Vector da(a.n,0.0);
  //Vector db(a.n,0.0);
  Vector da,db;
  return Make(a,da,b,db,path,checkConstraints);
}

bool SmoothConstrainedInterpolator::Make(const Config& qa,const Vector& da,const Config& qb,const Vector& db,
					 GeneralizedCubicBezierSpline& path,
					 bool checkConstraints)
{   

  Vector temp(constraint->NumDimensions());
  ConstraintValue(qa,temp);
  if(temp.maxAbsElement() > ftol) {
    fprintf(stderr,"ConstrainedInterpolator: Warning, initial point a is not on manifold, error %g\n",temp.maxAbsElement());
  }
  ConstraintValue(qb,temp);
  if(temp.maxAbsElement() > ftol) {
    fprintf(stderr,"ConstrainedInterpolator: Warning, initial point b is not on manifold, error %g\n",temp.maxAbsElement());
  }

  Vector da2=da,db2=db;
  if(!da.empty()) {
    if(!ProjectVelocity(qa,da2)) {
      fprintf(stderr,"ConstrainedInterpolator: Warning, initial velocity a could not be projected\n");
      da2.setZero();
    }
  }
  if(!db.empty()) {
    if(!ProjectVelocity(qb,db2)) {
      fprintf(stderr,"ConstrainedInterpolator: Warning, initial velocity b could not be projected\n");
      db2.setZero();
    }
  }

  const static double third = 1.0/3.0;
  const static double sixth = 1.0/6.0;
  list<pair<GeneralizedCubicBezierCurve,double> > lpath;
  GeneralizedCubicBezierCurve curve(space);
  curve.x0 = qa;
  curve.x3 = qb;
  curve.SetNaturalTangents(da2,db2);
  bool redo = (da2.empty() || db2.empty());
  if(da2.empty()) {
    curve.Deriv(0,da2);
    if(!ProjectVelocity(qa,da2)) {
      fprintf(stderr,"ConstrainedInterpolator: Warning, start velocity a could not be projected\n");
      da2.setZero();
    }
  }
  if(db2.empty()) {
    curve.Deriv(1,db2);
    if(!ProjectVelocity(qb,db2)) {
      fprintf(stderr,"ConstrainedInterpolator: Warning, end velocity b could not be projected\n");
      db2.setZero();
    }
  }
  if(redo) 
    curve.SetNaturalTangents(da2,db2);

#if CONDITION_INITIAL_TANGENTS
  ConditionTangents(curve);
#endif
  lpath.push_back(pair<GeneralizedCubicBezierCurve,double>(curve,1.0));
  priority_queue<Segment2,vector<Segment2> > q;
  Segment2 s;
  s.prev = lpath.begin();
  s.length = curve.OuterLength();
  q.push(s);

  GeneralizedCubicBezierCurve c1(space),c2(space);
  Config x,v;
  while(!q.empty()) {
    s=q.top(); q.pop();
    if(s.length <= xtol) continue;
    list<pair<GeneralizedCubicBezierCurve,double> >::iterator c = s.prev;
    list<pair<GeneralizedCubicBezierCurve,double> >::iterator n = c; ++n;

    /*
    //optimize end tangents to release tension
    Config* prev=NULL, *next=NULL;
    Real prevdur = 1.0, nextdur = 1.0;
    if(c != lpath.begin()) {
      list<pair<GeneralizedCubicBezierCurve,double> >::iterator p = c; --p;
      prev = &p->first.x0;
      prevdur = p->second;
    }
    if(n != lpath.end()) {
      next = &n->first.x3;
      nextdur = n->second;
    }
    c->first.SetSmoothTangents(prev,next,prevdur/c->second,nextdur/c->second);
    Vector v1,v2;
    c->first.Deriv(0,v1);
    c->first.Deriv(1,v2);
    ProjectVelocity(c->first.x0,v1);
    ProjectVelocity(c->first.x3,v2);
    c->first.SetNaturalTangents(v1,v2);
    */

    //c->first.Eval(0.5,x);
    c->first.Midpoint(x);
    //cout<<"Depth: "<<c->second<<endl;
    //cout<<"Bspline midpoint: "<<x<<", "<<v<<endl;
    //cout<<"Original point :"<<x<<endl;
    if(!Project(x)) {
      ConstraintValue(x,temp);
      if(gConstrainedInterpolateVerbose) cout<<"Projection of point "<<x<<" failed, "<<" error "<<temp.maxAbsElement()<<endl;
      return false;
    }
    //cout<<"Projected midpoint: "<<x<<", "<<v<<endl;
    //getchar(); 

#if OPTIMIZE_TANGENTS
    if(space) FatalError("Can't optimize tangents with a manifold");
    //scale the tangents of the curve so that the midpoint gets closer to x
    //xmid = (x0/8+3/8 x1 + 3/8 x2 + x3/8) + 3/8 (alpha (x1-x0) - beta (x3-x2))
    //Solve least squares
    //cout<<"Projected point :"<<x<<endl;
    Vector t1=c->first.x1-c->first.x0,t2=c->first.x3-c->first.x2;
    Vector xmid = (c->first.x0+c->first.x3)*0.5 + 3.0/8.0*(t1-t2);
    Vector rhs = (x - xmid)*8.0*third;
    Real origDist = xmid.distance(x);
    //cout<<"Xmid "<<xmid<<endl;
    Vector2 Atb(dot(rhs,t1),-dot(rhs,t2));
    Matrix2 AtA;
    AtA(0,0) = dot(t1,t1) + 1e-1*s.length;
    AtA(0,1) = AtA(1,0) = -dot(t1,t2);
    AtA(1,1) = dot(t2,t2) + 1e-1*s.length;
    //cout<<"AtA: "<<AtA<<endl;
    bool res = AtA.inplaceInverse();
    if(res) {
      Vector2 alphabeta = AtA*Atb;
      //cout<<"Scaling: "<<alphabeta<<endl;
      if(space) {
	space->Integrate(c->first.x0,(1.0+alphabeta.x)*t1,c->first.x1);
	space->Integrate(c->first.x3,-(1.0+alphabeta.y)*t2,c->first.x2);
      }
      else {
	c->first.x1 = c->first.x0 + (1.0+alphabeta.x)*t1;
	c->first.x2 = c->first.x3 - (1.0+alphabeta.y)*t2;
      }
      duration1 *= (1.0+alphabeta.x);
      duration2 *= (1.0+alphabeta.y);
      c->first.Eval(0.5,rhs);
      Real newDist = rhs.distance(x);
      //Assert(newDist <= origDist+Epsilon);
      //cout<<"Result: "<<rhs<<endl;
      //getchar();
    }
#endif // OPTIMIZE_TANGENTS

    if(checkConstraints && space && !space->IsFeasible(x)) return false;

    //c->first.Deriv(0.5,v);
    c->first.MidpointDeriv(v);
    ProjectVelocity(x,v);

    //subdivide, insert the split segments into the queue
    c1.x0 = c->first.x0;
    if(space) {
      space->Interpolate(c->first.x0,c->first.x1,0.5,c1.x1); 
      space->Integrate(x,v*(-sixth),c1.x2);
    }
    else {
      interpolate(c->first.x0,c->first.x1,0.5,c1.x1); 
      c1.x2 = x-v*sixth;
    }
    c1.x3 = x;
    c2.x0 = x;
    if(space) {
      space->Integrate(x,v*sixth,c2.x1);
    }
    else {
      c2.x1 = x+v*sixth; 
    }
    Interpolate(space,c->first.x2,c->first.x3,0.5,c2.x2);
    c2.x3 = c->first.x3;
    //sanity check
    /*
    Vector temp;
    c1.Deriv(0,temp);
    if(!temp.isEqual((c->first.x1-c->first.x0)*3.0*0.5,1e-4)) {
      cout<<"Invalid starting derivative of subdivision 1"<<endl;
      cout<<temp<<" vs "<<(c->first.x1-c->first.x0)*3.0*0.5<<endl;
      getchar();
    }
    c1.Deriv(1,temp);
    if(!temp.isEqual(v*0.5,1e-4)) {
      cout<<"Invalid ending derivative of subdivision 1"<<endl;
      cout<<temp<<" vs "<<v*0.5<<endl;
      getchar();
    }
    c2.Deriv(0,temp);
    if(!temp.isEqual(v*0.5,1e-4)) {
      cout<<"Invalid starting derivative of subdivision 2"<<endl;
      cout<<temp<<" vs "<<v*0.5<<endl;
      getchar();
    }
    c2.Deriv(1,temp);
    if(!temp.isEqual((c->first.x3-c->first.x2)*3.0*0.5,1e-4)) {
      cout<<"Invalid ending derivative of subdivision 2"<<endl;
      cout<<temp<<" vs "<<(c->first.x3-c->first.x2)*3.0*0.5<<endl;
      getchar();
    }
    */


#if CONDITION_MIDDLE_TANGENTS
    ConditionMiddleTangent(c1,c2);
#endif // CONDITION_MIDDLE_TANGENTS

    Real l1=c1.OuterLength();
    Real l2=c2.OuterLength();
    if(l1 > 0.5*(1.0+maxGrowth)*s.length) {
      if(gConstrainedInterpolateVerbose) {
	cout<<"Projection exceeded growth factor: ";
	cout<<l1<<" > "<<0.5*(1.0+maxGrowth)*s.length<<endl;
      }
      /*
      cout<<c->first.x0<<", "<<c->first.x1<<", "<<c->first.x2<<", "<<c->first.x3<<endl;
      c->first.Midpoint(x);
      c->first.MidpointDeriv(v);
      cout<<"Midpoint: "<<x<<", deriv "<<v<<endl;
      Project(x);
      ProjectVelocity(x,v);
      cout<<"Projected midpoint: "<<x<<", deriv "<<v<<endl;
      */
      //getchar();
      return false;
    }
    if(l2 > 0.5*(1.0+maxGrowth)*s.length) {
      if(gConstrainedInterpolateVerbose) {
	cout<<"Projection exceeded growth factor: ";
	cout<<l2<<" > "<<0.5*(1.0+maxGrowth)*s.length<<endl;
      }
      /*
      cout<<c->first.x0<<", "<<c->first.x1<<", "<<c->first.x2<<", "<<c->first.x3<<endl;      
      c->first.Midpoint(x);
      c->first.MidpointDeriv(v);
      cout<<"Midpoint: "<<x<<", deriv "<<v<<endl;
      Project(x);
      ProjectVelocity(x,v);
      cout<<"Projected midpoint: "<<x<<", deriv "<<v<<endl;
      */
      return false;
    }

    //need to scale previous and next durations by 0.5
    Real origDuration = c->second;
    c->first = c1;
    c->second = 0.5*origDuration;
    list<pair<GeneralizedCubicBezierCurve,double> >::iterator m=lpath.insert(n,pair<GeneralizedCubicBezierCurve,double>(c2,0.5*origDuration));

    s.prev = c;
    s.length = l1;
    if(s.length > xtol) q.push(s);

    s.prev = m;
    s.length = l2;
    if(s.length > xtol) q.push(s);
  }

  //read out the path
  path.segments.resize(lpath.size());
  path.durations.resize(lpath.size());
  size_t k=0;
  for(list<pair<GeneralizedCubicBezierCurve,double> >::iterator i=lpath.begin();i!=lpath.end();i++,k++) {
    path.segments[k] = i->first;
    path.durations[k] = i->second;
  }
  return true;
}

void SmoothConstrainedInterpolator::ConstraintValue(const Config& x,Vector& val)
{
  (*constraint)(x,val);
}

bool SmoothConstrainedInterpolator::ProjectVelocity(const Config& x,Config& v)
{
  constraint->PreEval(x);
  Matrix J;
  constraint->Jacobian(x,J);
  if(!xmin.empty()) {
    //look through active contraints, set that column to 0
    for(int i=0;i<x.n;i++) {
      if(x(i)==xmin(i) || x(i) == xmax(i)) { 
	v(i) = 0;
	for(int j=0;j<J.m;j++)
	  J(j,i) = 0;
      }
    }
  }
  RobustSVD<Real> svd;
  bool res=svd.set(J);
  if(!res) {
    fprintf(stderr,"SmoothConstrainedInterpolator: Numerical error projecting velocity?\n");
    return false;
  }
  Vector temp;
  svd.nullspaceComponent(v,temp);
  v -= temp;
  return true;
}


bool SmoothConstrainedInterpolator::Project(Config& x)
{
  if(inequalities) FatalError("TODO: SmoothConstrainedInterpolator with inequalities");
  if(!constraint) return true;
  solver.tolf = ftol;
  solver.tolx = solver.tolmin = ftol*1e-2;
  solver.verbose = 0;
  if(!xmin.empty()) {
    solver.bmin.setRef(xmin);
    solver.bmax.setRef(xmax);
  }
  int iters=maxNewtonIters;
  solver.x = x;
  if(!solver.GlobalSolve(iters)) return false;
  x = solver.x;
  return true;
}





bool MultiSmoothInterpolate(SmoothConstrainedInterpolator& interp,const vector<Vector>& pts,GeneralizedCubicBezierSpline& path)
{
  path.segments.resize(0);
  path.durations.resize(0);
  Vector temp;
  GeneralizedCubicBezierSpline cpath;
  vector<GeneralizedCubicBezierCurve> pathSegs;
  MonotonicInterpolate(pts,pathSegs,interp.space);
  //SplineInterpolate(pts,pathSegs,interp.space);
  Assert(pathSegs.size()+1==pts.size());
  vector<Vector> derivs(pts.size());
  pathSegs[0].Deriv(0,derivs[0]);
  for(size_t i=0;i+1<pts.size();i++) 
    pathSegs[i].Deriv(1,derivs[i+1]);
  //project tangent points
  for(size_t i=0;i<pts.size();i++) 
    interp.ProjectVelocity(pts[i],derivs[i]);

  /*
  //condition tangent points?
  for(size_t i=0;i+1<pts.size();i++) 
    pathSegs[i].SetNaturalTangents(derivs[i],derivs[i+1]);
  //TODO: global solve?
  for(int iters=0;iters<10;iters++) {
    for(size_t i=0;i+1<pts.size();i++) {
      ConditionMiddleTangent(pathSegs[i],pathSegs[i+1]);
      pathSegs[i].Deriv(1,derivs[i+1]);
    }
  }
  */

  for(size_t i=0;i+1<pts.size();i++) {
    cpath.segments.resize(0);
    cpath.durations.resize(0);

    Assert(i < pathSegs.size());
    Assert(pathSegs.size()+1==pts.size());
    //bool res=interp.Make(pts[i],di,pts[i+1],dn,cpath);
    bool res=interp.Make(pathSegs[i].x0,derivs[i],pathSegs[i].x3,derivs[i+1],cpath);
    if(!res) {
      printf("Could not make path between point %d and %d\n",i,i+1);
      return false;
    }
    //cout<<"Actual initial velocity: "<<3.0*(cpath.front().x1-cpath.front().x0)/cdurations.front()<<", terminal velocity: "<<3.0*(cpath.back().x3-cpath.back().x2)/cdurations.back()<<endl;
    path.Concat(cpath);
  }
  return true;
}

bool MultiSmoothInterpolate(SmoothConstrainedInterpolator& interp,const vector<Vector>& pts,const Vector& dq0,const Vector& dq1,GeneralizedCubicBezierSpline& path)
{
  path.segments.resize(0);
  path.durations.resize(0);
  Vector temp;
  GeneralizedCubicBezierSpline cpath;
  vector<GeneralizedCubicBezierCurve> pathSegs;
  MonotonicInterpolate(pts,pathSegs,interp.space);
  //SplineInterpolate(pts,pathSegs,interp.space);
  vector<Vector> derivs(pts.size());
  derivs[0] = dq0;
  for(size_t i=0;i+1<pts.size();i++) 
    pathSegs[i].Deriv(1,derivs[i+1]);
  derivs.back() = dq1;
  //project tangent points
  for(size_t i=1;i+1<pts.size();i++) 
    interp.ProjectVelocity(pts[i],derivs[i]);

  /*
  //condition tangents?
  for(size_t i=0;i+1<pts.size();i++) 
    pathSegs[i].SetNaturalTangents(derivs[i],derivs[i+1]);
  //TODO: global solve?
  for(int iters=0;iters<10;iters++) {
    for(size_t i=0;i+2<pts.size();i++) {
      ConditionMiddleTangent(pathSegs[i],pathSegs[i+1]);
      pathSegs[i].Deriv(1,derivs[i+1]);
    }
  }
  */

  for(size_t i=0;i+1<pts.size();i++) {
    cpath.segments.resize(0);
    cpath.durations.resize(0);
    bool res=interp.Make(pathSegs[i].x0,derivs[i],pathSegs[i].x3,derivs[i+1],cpath);
    if(!res) {
      printf("Could not make path between point %d and %d\n",i,i+1);
      return false;
    }
    //cout<<"Actual initial velocity: "<<3.0*(cpath.front().x1-cpath.front().x0)/cdurations.front()<<", terminal velocity: "<<3.0*(cpath.back().x3-cpath.back().x2)/cdurations.back()<<endl;
    path.Concat(cpath);
  }
  return true;
}


bool AppendInterpolate(SmoothConstrainedInterpolator& interp,const Vector& pt,Real suffixDuration,GeneralizedCubicBezierSpline& path)
{
  Assert(!path.segments.empty());
  GeneralizedCubicBezierSpline cpath;
  Vector di,dn;
  if(interp.space) {
    path.segments.back().Deriv(1.0,di);
    di *= suffixDuration/path.durations.back();
  }
  else {
    di = suffixDuration*3.0*(path.segments.back().x3-path.segments.back().x2)/path.durations.back();
  }

  bool res=interp.Make(path.segments.back().x3,di,pt,dn,cpath);
  if(!res) {
    return false;
  }
  cpath.TimeScale(suffixDuration);
  //cout<<"Actual initial velocity: "<<3.0*(cpath.front().x1-cpath.front().x0)/cdurations.front()<<", terminal velocity: "<<3.0*(cpath.back().x3-cpath.back().x2)/cdurations.back()<<endl;
  path.Concat(cpath);
  return true;
}




} //namespace Mintos




