#include "gcp.h"
#include <assert.h>
#include <stdio.h>
#include <math.h>
using namespace GCP;
using namespace std;

#define VERBOSITY_NONE 0
#define VERBOSITY_ERROR 1
#define VERBOSITY_WARNING 2
#define VERBOSITY_INFO 3
//TODO: tweak this to get debug information
#define VERBOSE VERBOSITY_NONE
#define Assert(x,str) if(!(x)) { fprintf(stderr,"Assertion failed: \"%s\"\n",str); assert(x); }

const static Real Epsilon=1e-8;
const static Real Inf= 1e300;

inline Real Sign(Real v) {
  if(v < 0) return -1.0;
  else if(v > 0) return 1.0;
  else return 0.0;
}
inline Real Sqr(Real x) { return x*x; }
inline Real Sqrt(Real x) { return sqrt(x); }
inline Real Abs(Real v) { return fabs(v); }
inline bool InInterval(Real x,Real a,Real b) { return a <= x && x <= b; }
inline Real Ceil(Real x) { return ceil(x); }
inline Real Floor(Real x) { return floor(x); }
inline Real Log(Real x) { return log(x); }
inline Real Log2(Real x) { return log2(x); }
inline Real Min(Real a,Real b) { if(a < b) return a; else return b; }
inline Real Max(Real a,Real b) { if(a > b) return a; else return b; }

Real Distance(const Vector2& a,const Vector2& b) { return Sqrt(Sqr(a.x-b.x)+Sqr(a.y-b.y)); }

///line-line intersection
///Note: does not handle colinear lines
bool LineIntersect(const Vector2& s1,const Vector2& d1,const Vector2& s2,const Vector2& d2,Real& t1,Real& t2)
{
  //s1 + t1*d1 = s2 + t2*d2
  //[d1|-d2] [t1,t2]^T = s2 - s1 
  Real a = d1.x;
  Real b = -d2.x;
  Real c = d1.y;
  Real d = -d2.y;
  Real den = a*d - b*c;
  if(Abs(den) < Epsilon) return false;
  Real dsx = s2.x - s1.x;
  Real dsy = s2.y - s1.y;
  t1 = (d*dsx - b*dsy)/den;
  t2 = (-c*dsx + a*dsy)/den;
  return true;
}

bool SegmentIntersect(const Vector2& a1,const Vector2& b1,const Vector2& a2,const Vector2& b2,Real& t1,Real& t2)
{
  Vector2 d1,d2;
  d1.x = b1.x-a1.x;
  d1.y = b1.y-a1.y;
  d2.x = b2.x-a2.x;
  d2.y = b2.y-a2.y;
  if(LineIntersect(a1,d1,a2,d2,t1,t2)) {
    if(t1 >= 0 && t1 <= 1 && t2 >= 0 && t2 <= 1) return true;
  }
  return false;
}

Vector2::Vector2()
  :x(0),y(0)
{}

Vector2::Vector2(Real _x,Real _y)
  :x(_x),y(_y)
{}

void Segment::Sort(bool increasing)
{
  if(increasing) {
    if(a.x > b.x) {
      std::swap(a,b);
    }
    else if(a.x == b.x && a.y > b.y) {
      std::swap(a,b);
    }
  }
  else {
    if(a.x < b.x) {
      std::swap(a,b);
    }
    else if(a.x == b.x && a.y < b.y) {
      std::swap(a,b);
    }
  }
}

Real Segment::HeightOf(Real x) const
{
  if(a.x == b.x) return Max(a.y,b.y);
  Real u = (x-a.x)/(b.x-a.x);
  return Eval(u).y;
}

Vector2 Segment::Eval(Real u) const
{
  return Vector2(a.x + u*(b.x-a.x),a.y + u*(b.y-a.y));
}

Vector2 Segment::Tangent() const
{
  Real d = ::Distance(a,b);
  return Vector2((b.x-a.x)/d,(b.y-a.y)/d);
}

Vector2 Segment::Normal() const
{
  Vector2 t = Tangent();
  return Vector2(-t.y,t.x);
}

Real Segment::Distance(const Vector2& x) const
{
  return ::Distance(x,ClosestPoint(x));
}

Vector2 Segment::ClosestPoint(const Vector2& x) const
{
  Real dp = (x.x-a.x)*(b.x-a.x)+(x.y-a.y)*(b.y-a.y);
  Real denom = (Sqr(b.x-a.x)+Sqr(b.y-a.y));
  Real u = Max(0.0,Min(dp / denom,1.0));
  return Vector2(a.x + u*(b.x-a.x),a.y + u*(b.y-a.y));
}

void PolygonalTerrain::Range(const Vector2& bmin,const Vector2& bmax,PolygonalTerrain& result) const
{
  result.segments.resize(0);
  for(size_t i=0;i<segments.size();i++) {
    //Assert(segments[i].IsSorted(),"Can't do Range() without unsorted segments yet");
    if(segments[i].b.x < bmin.x || Max(segments[i].a.y,segments[i].b.y) < bmin.y) continue;
    if(segments[i].a.x > bmax.x || Min(segments[i].a.y,segments[i].b.y) > bmax.x) continue;
    //look for intersection
    Real umin = 0, umax = 1;
    if(segments[i].a.x < bmin.x) //crop at bmin
      umin = Max(umin,(bmin.x-segments[i].a.x)/(segments[i].b.x-segments[i].a.x));
    if(segments[i].b.x > bmax.x) //crop at bmax
      umax = Min(umax,(bmax.x-segments[i].a.x)/(segments[i].b.x-segments[i].a.x));
    if(umin >= umax) continue;
    Segment s;
    s.a = segments[i].Eval(umin);
    s.b = segments[i].Eval(umax);
    result.segments.push_back(s);
  }
}

Problem::Problem()
  :pathCurvature(0),frictionCoefficient(Inf),g(9.8),minGs(0),maxGs(3),timeStep(1e-2),hmin(0),hmax(Inf),Lmin(0),Lmax(Inf),hasStancePoint(false),cpXTolerance(5e-3),cpVTolerance(1e-2)
{
}

Problem::Problem(Real h,Real vx0)
  :pathCurvature(0),frictionCoefficient(Inf),g(9.8),minGs(0),maxGs(3),timeStep(1e-2),hmin(0),hmax(Inf),Lmin(0),Lmax(Inf),hasStancePoint(false),cpXTolerance(5e-3),cpVTolerance(1e-2)
{
  x0.y = h;
  v0.x = vx0;
  Assert(vx0 >= 0,"Start x velocity must be positive");
}

void Problem::SetInitialConditions(const Vector2& _x0,const Vector2& _v0)
{
  x0 = _x0;
  v0 = _v0;
  Assert(v0.x >= 0,"Start x velocity must be positive");
}

void Problem::SetGravity(Real _g)
{
  g = _g;
}


void Problem::SetFriction(Real _mu)
{
  frictionCoefficient = _mu;
}

void Problem::SetStancePoint(const Vector2& p)
{
  hasStancePoint = true;
  stancePoint = p;
}

bool Problem::ReachableTerminalCM(const Vector2& cm,const Vector2& cp) const
{
  if(hasStancePoint)
    if(!InInterval(Distance(cm,stancePoint),Lmin,Lmax)) return false;
  if(!InInterval(Distance(cm,cp),Lmin,Lmax)) return false;
  if(!InInterval(cm.y,cp.y+hmin,cp.y+hmax)) return false;
  //check collisions
  for(size_t i=0;i<terrain.segments.size();i++) {
    Real t1,t2;
    if(SegmentIntersect(terrain.segments[i].a,terrain.segments[i].b,cm,cp,t1,t2) && t2 < 0.98) {
      return false;
    }
  }
  return true;
}

bool Problem::SolveCPFromCurvature(Real pathCurvature,Real tolerance)
{
  Assert(v0.x >= 0,"Start x velocity must be positive");
  if(terrain.segments.empty()) {
    if(VERBOSE >= VERBOSITY_WARNING)
      fprintf(stderr,"SolveCPFromCurvature: Terrain must not be empty\n");
    return false;
  }
   
  //TODO: fast pruning of clearly unreachable segments
  vector<Vector2> candidateCPs;
  for(size_t i=0;i<terrain.segments.size();i++)
    if(SolveCPFromCurvature(pathCurvature,terrain.segments[i],tolerance))
      return true;
    else
      candidateCPs.push_back(capturePoint);
  //no solution, pick the best
  Vector2 xfinal,vfinal;
  int numSimIters;
  int bestSimIters = 0;
  Vector2 bestCapturePoint(0,0);
  for(size_t i=0;i<candidateCPs.size();i++) {
    capturePoint = candidateCPs[i];
    Simulate(timeStep,1000,xfinal,vfinal,&numSimIters);
    if(numSimIters > bestSimIters) {
      bestSimIters = numSimIters;
      bestCapturePoint = capturePoint;
    }
  }
  capturePoint = bestCapturePoint;
  return false;
}

bool Problem::SolveCPFromCurvature(Real _pathCurvature,const Segment& segment,Real tolerance)
{
  Assert(tolerance > 0,"Tolerance must be positive");
  Assert(v0.x >= 0,"Start x velocity must be positive");
  Assert(segment.a.x <= segment.b.x, "Line segment not ordered in increasing x coordinate");
  pathCurvature = _pathCurvature;
  Real p0 = segment.a.x;
  Real p1 = segment.b.x;
  //can't work with a segment behind the CM
  if(segment.b.x < x0.x) {
    if(VERBOSE >= VERBOSITY_WARNING)
      printf("Segment behind CM, %g < %g\n",segment.b.x,x0.x);
    return false;
  }
  //can't work with a segment ahead by some maximum 
  Real xmax = x0.x + v0.x * (x0.y - Min(segment.a.y,segment.b.y));
  if(segment.a.x > xmax) {
    if(VERBOSE >= VERBOSITY_WARNING)
      printf("Segment too far forward, %g > %g\n",segment.a.x,xmax);
    return false;
  }
  if(p0 < x0.x) p0 = x0.x;
  if(p1 > xmax) p1 = xmax;

  //intersect segment with leg min/max range to determine valid range
  if(Distance(x0,segment.b) > Lmax) {
    //TODO
  }
  //TODO: limit range by height of parabola
  //TODO: limit range by stance point, if available

  //1. Test the start of the range to make sure there's an overshoot
  //IMPORTANT TODO: does this actually work?  Might there be an alternate point?
  capturePoint = segment.a;
  capturePoint.x = p0;
  capturePoint.y = segment.HeightOf(capturePoint.x);
  capturePointNormal = segment.Normal();
  Vector2 xfinal,vfinal;
  Vector2 bestCapturePoint = capturePoint;
  int bestCaptureIters = 0;
  int numSimIters;
  SimulationResult res = Simulate(timeStep,1000,xfinal,vfinal,&numSimIters);
  bestCaptureIters = numSimIters;
  if(res == Capture) {
    if(ReachableTerminalCM(xfinal,capturePoint)) 
      return true; //lucky
    else { //hmm... unreachable
      if(VERBOSE >= VERBOSITY_INFO)
	printf("Converged on initial point, but it's unreachable.\n");
      return false;
    }
  }
  if(res == Undershoot || res == MaxItersReached) {
    if(VERBOSE >= VERBOSITY_INFO)
      printf("Undershoot on initial point %g, %g, too far forward.\n",segment.a.x,segment.a.y);
    return false;   //too far forward anyway
  }

  //2. Do bisection
  capturePoint.x = p1;
  if(p0 > p1) {
    if(VERBOSE >= VERBOSITY_INFO)
      printf("Limits made the segment null? %g, %g.\n",p0,p1);
    return false;
  }
  int maxBisection = (int)Ceil(Log2((p1-p0)/tolerance));
  if(VERBOSE >= VERBOSITY_INFO)
    printf("%d bisection iterations, range %g %g...\n",maxBisection,p0,p1);
  for(int iters=0;iters<maxBisection;iters++) {
    capturePoint.y = segment.HeightOf(capturePoint.x);
    SimulationResult res = Simulate(timeStep,1000,xfinal,vfinal,&numSimIters);
    if(numSimIters > bestCaptureIters) {
      if(ReachableTerminalCM(xfinal,capturePoint)) {
	bestCaptureIters = numSimIters;
	bestCapturePoint = capturePoint;
      }
    }
    if(res == Capture) {
      if(VERBOSE >= VERBOSITY_INFO)
	printf("Iteration %d: cp %g %g, capture!\n",iters,capturePoint.x,capturePoint.y);
      if(ReachableTerminalCM(xfinal,capturePoint))
	return true;
      else // capture point not reachable, can we just return false or could there be another?
	return false; 
    }
    else if(res == Overshoot) {
      if(VERBOSE >= VERBOSITY_INFO)
	printf("Iteration %d: cp %g %g, overshoot result %g %g, %g %g\n",iters,capturePoint.x,capturePoint.y,xfinal.x,xfinal.y,vfinal.x,vfinal.y);
      p0 = capturePoint.x;
      //IMPORTANT TODO: does this actually work?  Might there be an alternate point?
      if(p0 == p1) {
	capturePoint = bestCapturePoint;
	return false;
      }
    }
    else  { // res == Undershoot || res == MaxItersReached || res == Slip
      if(VERBOSE >= VERBOSITY_INFO) {
	if(res == MaxItersReached) {
	  printf("Iteration %d: cp %g %g, maxiters result %g %g, %g %g\n",iters,capturePoint.x,capturePoint.y,xfinal.x,xfinal.y,vfinal.x,vfinal.y);
	}
	else if(res == Slip) {
	  printf("Iteration %d: cp %g %g, slip result %g %g, %g %g\n",iters,capturePoint.x,capturePoint.y,xfinal.x,xfinal.y,vfinal.x,vfinal.y);
	}
	else {
	  printf("Iteration %d: cp %g %g, undershoot result %g %g, %g %g\n",iters,capturePoint.x,capturePoint.y,xfinal.x,xfinal.y,vfinal.x,vfinal.y);
	}
      }

      p1 = capturePoint.x;
    }
    capturePoint.x = (p0+p1)*0.5;
  }
  if(VERBOSE >= VERBOSITY_INFO) 
    printf("Out of iterations...\n");
  capturePoint = bestCapturePoint;
  return false;
}

bool Problem::SolveCurvatureFromCP(const Vector2& _capturePoint,const Vector2& _capturePointNormal,Real tolerance)
{
  Assert(tolerance > 0,"Tolerance must be positive");
  Assert(v0.x >= 0,"Start x velocity must be positive");
  capturePoint = _capturePoint;
  capturePointNormal = _capturePointNormal;
  Real dx = capturePoint.x-x0.x;
  if(Abs(dx) < cpXTolerance) {
    if(Abs(v0.x) < cpVTolerance) {
      pathCurvature = 0;
      return true;
    }
  }
  Real minh = capturePoint.y + hmin;
  Real maxh = capturePoint.y + hmax;
  if(capturePoint.y + Lmin > minh) minh = capturePoint.y + Lmin;
  if(capturePoint.y + Lmax < maxh) maxh = capturePoint.y + Lmax;
  //limit the possible path curvatures
  //h(final) = 0.5*a*(capturePoint.x - x0)^2 + (capturePoint.x - x0)*slope0 + y0
  Real slope0 = (v0.x > 0 ? v0.y / v0.x : 0);
  Real amin = 2.0*(minh - x0.y - slope0*dx) / Sqr(dx);
  Real amax = 2.0*(maxh - x0.y - slope0*dx) / Sqr(dx);
  if(amin*Sqr(v0.x) < (minGs - 1.0)*g)
    amin = (minGs - 1.0)*g/Sqr(v0.x);
  if(amax*Sqr(v0.x) > (maxGs - 1.0)*g)
    amax = (maxGs - 1.0)*g/Sqr(v0.x);
  //revise minimum/maximum heights over capture point
  minh = slope0*dx+0.5*Sqr(dx)*amin;
  maxh = slope0*dx+0.5*Sqr(dx)*amax;

  if(amin > amax) {
    if(VERBOSE >= VERBOSITY_WARNING) 
      fprintf(stderr,"Empty curvature range\n");
    return false;
  }
  //Real acr = ((capturePoint.y - x0.y) - slope0*(dx))/Sqr(dx);
  //Real bcr = (capturePoint.y-x0.y)/dx;

  //1. Test the start of the range to make sure there's an overshoot
  //IMPORTANT TODO: does this actually work?  Might there be an alternate pont?
  pathCurvature = amin;
  Vector2 xfinal,vfinal;
  Real bestPathCurvature = pathCurvature;
  int bestPathIters = 0;
  int numSimIters;
  SimulationResult res = Simulate(timeStep,1000,xfinal,vfinal,&numSimIters);
  bestPathIters = numSimIters;
  if(res == Capture) {
    if(ReachableTerminalCM(xfinal,capturePoint))
      return true; //lucky
    else
      return false;
  }
  if(res == Undershoot || res == MaxItersReached) return false;   //too far forward anyway

  pathCurvature = amax;

  int maxBisection = (int)Max(0.0,Floor(Log2((maxh-minh)/tolerance)))+1;
  if(VERBOSE >= VERBOSITY_INFO)
    printf("Capture point %g %g given, %d bisection iterations range %g %g...\n",capturePoint.x,capturePoint.y,maxBisection,amin,amax);
  for(int iters=0;iters<maxBisection;iters++) {
    SimulationResult res = Simulate(timeStep,1000,xfinal,vfinal,&numSimIters);
    if(numSimIters > bestPathIters) {
      if(ReachableTerminalCM(xfinal,capturePoint)) {
	bestPathIters = numSimIters;
	bestPathCurvature = pathCurvature;
      }
    }
    if(res == Capture) {
      if(VERBOSE >= VERBOSITY_INFO) 
	printf("Iteration %d: curvature %g, capture result %g %g, %g %g\n",iters,pathCurvature,xfinal.x,xfinal.y,vfinal.x,vfinal.y);
      if(ReachableTerminalCM(xfinal,capturePoint))
	return true;
      else // capture point not reachable, can we just return false or could there be another?
	return false; 
    }
    else if(res == Overshoot) {
      if(VERBOSE >= VERBOSITY_INFO) 
	printf("Iteration %d: curvature %g, overshoot result %g %g, %g %g\n",iters,pathCurvature,xfinal.x,xfinal.y,vfinal.x,vfinal.y);
      amin = pathCurvature;
      //IMPORTANT TODO: does this actually work?  Might there be an alternate pont?
      if(amin >= amax) {
	pathCurvature = bestPathCurvature;
	return false;
      }
    }
    else  { // res == Undershoot || res == MaxItersReached
      if(VERBOSE >= VERBOSITY_INFO) {
	if(res == Undershoot) {
	  printf("Iteration %d: curvature %g, undershoot result %g %g, %g %g\n",iters,pathCurvature,xfinal.x,xfinal.y,vfinal.x,vfinal.y);
	}
	else if(res == Slip) {
	  printf("Iteration %d: curvature %g, slip result %g %g, %g %g\n",iters,pathCurvature,xfinal.x,xfinal.y,vfinal.x,vfinal.y);
	}
	else {
	  printf("Iteration %d: curvature %g, maxiters result %g %g, %g, %g\n",iters,pathCurvature,xfinal.x,xfinal.y,vfinal.x,vfinal.y);
	}
      }
      amax = pathCurvature;
    }
    pathCurvature = (amin+amax)*0.5;
  }
  if(VERBOSE >= VERBOSITY_INFO) {
    minh = slope0*dx+0.5*Sqr(dx)*amin;
    maxh = slope0*dx+0.5*Sqr(dx)*amax;
    printf("Out of iterations, bracket %g %g (height bracket %g %g)\n",amin,amax,minh,maxh);
  }
  pathCurvature = bestPathCurvature;
  return false;
}

void Problem::SolveAllCurvatureCPs(Real resolution,std::vector<Real>& pathCurvatureList,std::vector<Vector2>& cpList)
{
  Assert(v0.x >= 0,"Start x velocity must be positive");
  Assert(resolution > 0,"Resolution must be strictly positive");
  Real oldPathCurvature=pathCurvature;
  Vector2 oldCapturePoint=capturePoint;
  pathCurvatureList.resize(0);
  cpList.resize(0);
  Real minh = hmin;
  Real maxh = hmax;
  if(Lmin > minh) minh = Lmin;
  if(Lmax < maxh) maxh = Lmax;

  PolygonalTerrain terrain2;
  Vector2 bmin,bmax;
  bmin.x = x0.x;
  bmax.x = x0.x + maxh*v0.x;
  bmin.y = x0.y - maxh;
  bmax.y = x0.y - minh;
  terrain.Range(bmin,bmax,terrain2);
  for(size_t i=0;i<terrain2.segments.size();i++) {
    Vector2 cpn = terrain2.segments[i].Normal();
    //printf("Segment %g %g -> %g %g\n",terrain2.segments[i].a.x,terrain2.segments[i].a.y,terrain2.segments[i].b.x,terrain2.segments[i].b.y);
    if(frictionCoefficient != Inf) {
      //find the range of the segment that won't slip by intersecting a cone
      //centered at x0
      Vector2 cpt = terrain2.segments[i].Tangent();
      Vector2 d1(cpt.x * frictionCoefficient - cpn.x,cpt.y * frictionCoefficient - cpn.y);
      Vector2 d2(-cpt.x * frictionCoefficient - cpn.x,-cpt.y * frictionCoefficient - cpn.y);
      //x0 + t*dt = a + u (b-a)
      Vector2 d(terrain2.segments[i].b.x-terrain2.segments[i].a.x,terrain2.segments[i].b.y-terrain2.segments[i].a.y);
      Real t,umin,umax;
      if(LineIntersect(x0,d1,terrain2.segments[i].a,d,t,umax)) {
	if(t < 0) continue; //behind line?
	if(umax > 1) umax = 1;
	if(umax < 0) continue;    //eliminate segment from consideration
      }
      else umax = 1;
      if(LineIntersect(x0,d2,terrain2.segments[i].a,d,t,umin)) {
	if(t < 0) continue; //behind line?
	if(umin < 0) umin = 0;
	if(umin > 1) continue;    //eliminate segment from consideration
      }
      else umin = 0;
      //printf("Segment %d: friction range %g %g\n",i,umin,umax);
      if(umin > 1) continue;
      if(umax < 0) continue;  //eliminate segment from consideration
      if(umin != 0 || umax != 1) {
	Vector2 newa = terrain2.segments[i].Eval(umin);
	Vector2 newb = terrain2.segments[i].Eval(umax);
	terrain2.segments[i].a = newa;
	terrain2.segments[i].b = newb;
      }
    }
    ///discretize at te given resolution
    Real slen = Distance(terrain2.segments[i].a,terrain2.segments[i].b);
    int ndivs = (int)(Ceil(slen/resolution));
    for(int k=0;k<ndivs;k++) {
      Real u=(Real(k)+0.5)/Real(ndivs);
      Vector2 cp = terrain2.segments[i].Eval(u);
      if(VERBOSE >= VERBOSITY_INFO)
	printf("Trying %g %g\n",cp.x,cp.y);
      if(SolveCurvatureFromCP(cp,cpn)) {
	if(VERBOSE >= VERBOSITY_INFO)
	  printf("  result %g\n",pathCurvature);
	pathCurvatureList.push_back(pathCurvature);
	cpList.push_back(cp);
      }
      else {
	if(VERBOSE >= VERBOSITY_INFO)
	  printf("  failed, best guess %g\n",pathCurvature);
      }
    }
  }
  pathCurvature=oldPathCurvature;
  capturePoint=oldCapturePoint;
}

Real Problem::PathHeight(Real x) const
{
  if(Abs(v0.x) < Epsilon) return x0.y;
  Real slope0 = v0.y/v0.x;
  return x0.y + slope0*(x-x0.x) + 0.5*pathCurvature*Sqr(x-x0.x);
}

Problem::SimulationResult Problem::SimulationTrace(Real dt,int maxIters,std::vector<Vector2>& xtrace,std::vector<Vector2>& vtrace) const
{
  Vector2 x=x0,v=v0;
  Real sign = Sign(v.x);
  Assert(sign>=0,"Velocity x component must be positive");
  if(Abs(x.x - capturePoint.x) < cpXTolerance && Abs(v.x) < cpVTolerance) return Capture;
  if(sign == 0) { //will just fall backwards... can just return now
    return Undershoot;
  }
  xtrace.resize(1);
  vtrace.resize(1);
  xtrace[0] = x;
  vtrace[0] = v;
  Real slope0 = v0.y/v0.x;
  Vector2 a; //acceleration
  for(int iters=0;iters<maxIters;iters++) {
    if(Sign(x.x - capturePoint.x) == sign) return Overshoot;
    //current slope dz/dx and acceleration d^2z/dx^2 of path. Can implement
    //other paths by changing these two lines
    Real slope = slope0 + (x.x-x0.x)*pathCurvature;
    Real curvature = pathCurvature;
    //denominator
    Real denom = (x.y - capturePoint.y) - (x.x-capturePoint.x)*slope;
    if(Abs(denom) < Epsilon) { //degeneracy of equations
      if(VERBOSE >= VERBOSITY_WARNING) 
	printf("SimulationTrace: Equations becoming degenerate?\n");
      return Undershoot;
    }
    Real dnormal = capturePointNormal.x*(x.x-capturePoint.x) + capturePointNormal.y*(x.y-capturePoint.y);
    Real dtang = capturePointNormal.y*(x.x-capturePoint.x) - capturePointNormal.x*(x.y-capturePoint.y);
    if(Abs(dtang) > frictionCoefficient*dnormal) {
      //at friction limit, move with maximum deceleration
      //fc = [-mu*(z-zb)*f,(z-zb)*f]^T
      //(fc + fg)/m = r'' = [x'',dz/dx x'' + dz^2/dx^2 x'^2]^T
      //It's a linear equation in x'' and f. Expanding we get
      //-mu*(z-zb)*f/m = x''
      //(z-zb)*f/m - g = dz/dx x'' + dz^2/dx^2 x'^2 = - dz/dx mu*(z-zb)*f/m + dz^2/dx^2 x'^2
      //=> f/m[(z-zb) + dz/dx mu*(z-zb)] = g + dz^2/dx^2 x'^2
      //=> f/m = [g + dz^2/dx^2 x'^2]/[(z-zb) + dz/dx mu*(z-zb)]
      //=> x'' = -mu*(z-zb)[g + dz^2/dx^2 x'^2]/[(z-zb) + dz/dx mu*(z-zb)]
      //this would be useful when taking into account angular inertia, but
      //we're not, so we're violating the LIPM model
      return Slip;
    }
    else {
      a.x = (x.x-capturePoint.x)*(g+Sqr(v.x)*curvature)/denom;
      a.y = ((x.y-capturePoint.y)*Sqr(v.x)*curvature + g*(x.x-capturePoint.x)*slope)/denom;
    }
    x.x += v.x*timeStep;
    x.y = x0.y + (x.x-x0.x)*slope0 + 0.5*Sqr(x.x-x0.x)*curvature;
    v.x += a.x*timeStep;
    v.y = slope*v.x;
    xtrace.push_back(x);
    vtrace.push_back(v);
    if(Abs(x.x - capturePoint.x) < cpXTolerance && Abs(v.x) < cpVTolerance) return Capture;
    if(Sign(v.x) != sign) return Undershoot;
  }
  return MaxItersReached;
}

Problem::SimulationResult Problem::Simulate(Real dt,int maxIters,Vector2& x,Vector2& v,int* numSteps) const
{
  x=x0;
  v=v0;
  Real sign = Sign(v.x);
  Assert(sign>=0,"Velocity x component must be positive");
  if(Abs(x.x - capturePoint.x) < cpXTolerance && Abs(v.x) < cpVTolerance) return Capture;
  if(sign == 0) { //will just fall backwards... can just return now
    printf("Initial x velocity is zero\n");
    if(numSteps) *numSteps=0;
    return Undershoot;
  }
  Real slope0 = v0.y/v0.x;
  Vector2 a; //acceleration
  for(int iters=0;iters<maxIters;iters++) {
    if(Sign(x.x - capturePoint.x) == sign) {
      if(numSteps) *numSteps=iters;
      return Overshoot;
    }
    //current slope dz/dx and acceleration d^2z/dx^2 of path. Can implement
    //other paths by changing these two lines
    Real slope = slope0 + (x.x-x0.x)*pathCurvature;
    Real curvature = pathCurvature;
    //denominator
    Real denom = (x.y - capturePoint.y) - (x.x-capturePoint.x)*slope;
    if(Abs(denom) < Epsilon) { //degeneracy of equations
      if(VERBOSE >= VERBOSITY_WARNING) 
	printf("Simulate: Equations becoming degenerate?\n");
      if(numSteps) *numSteps=iters;
      return Undershoot;
    }
    Real dnormal = capturePointNormal.x*(x.x-capturePoint.x) + capturePointNormal.y*(x.y-capturePoint.y);
    Real dtang = capturePointNormal.y*(x.x-capturePoint.x) - capturePointNormal.x*(x.y-capturePoint.y);
    if(Abs(dtang) > frictionCoefficient*dnormal) {
      //at friction limit, move with maximum deceleration
      //fc = [-mu*(z-zb)*f,(z-zb)*f]^T
      //(fc + fg)/m = r'' = [x'',dz/dx x'' + dz^2/dx^2 x'^2]^T
      //It's a linear equation in x'' and f. Expanding we get
      //-mu*(z-zb)*f/m = x''
      //(z-zb)*f/m - g = dz/dx x'' + dz^2/dx^2 x'^2 = - dz/dx mu*(z-zb)*f/m + dz^2/dx^2 x'^2
      //=> f/m[(z-zb) + dz/dx mu*(z-zb)] = g + dz^2/dx^2 x'^2
      //=> f/m = [g + dz^2/dx^2 x'^2]/[(z-zb) + dz/dx mu*(z-zb)]
      //=> x'' = -mu*(z-zb)[g + dz^2/dx^2 x'^2]/[(z-zb) + dz/dx mu*(z-zb)]
      //this would be useful when taking into account angular inertia, but
      //we're not, so we're violating the LIPM model
      if(numSteps) *numSteps=iters;
      return Slip;
    }
    else {
      a.x = (x.x-capturePoint.x)*(g+Sqr(v.x)*curvature)/denom;
      a.y = ((x.y-capturePoint.y)*Sqr(v.x)*curvature + g*(x.x-capturePoint.x)*slope)/denom;
    }
    x.x += v.x*timeStep;
    x.y = x0.y + (x.x-x0.x)*slope0 + 0.5*Sqr(x.x-x0.x)*curvature;
    v.x += a.x*timeStep;
    v.y = slope*v.x;
    if(Abs(x.x - capturePoint.x) < cpXTolerance && Abs(v.x) < cpVTolerance) {
      if(numSteps) *numSteps=iters;
      return Capture;
    }
    if(Sign(v.x) != sign) {
      if(numSteps) *numSteps=iters;
      return Undershoot;
    }
  }
  if(numSteps) *numSteps=maxIters;
  return MaxItersReached;
}

