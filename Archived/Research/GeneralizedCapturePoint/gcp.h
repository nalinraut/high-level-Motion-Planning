#ifndef GENERALIZED_CAPTURE_POINT_H
#define GENERALIZED_CAPTURE_POINT_H

#include <stdlib.h>
#include <vector>

namespace GCP {

typedef double Real;

class Vector2
{
 public:
  Vector2();
  Vector2(Real x,Real y);

  Real x,y;
};

class Segment
{
 public:
  ///Sorts the endpoints in increasing / decreasing x component
  void Sort(bool increasing=true);
  ///Returns the height of the segment at x coordinate x
  Real HeightOf(Real x) const;
  ///Returns the point at interpolation coordinate u
  Vector2 Eval(Real u) const;
  ///Returns the unit tangent vector of the segment
  Vector2 Tangent() const;
  ///Returns the unit normal vector of the segment (points to the "left")
  Vector2 Normal() const;
  ///Returns the minimum distance to x
  Real Distance(const Vector2& x) const;
  ///Returns the point on the segment with minimum distance to x
  Vector2 ClosestPoint(const Vector2& x) const;

  Vector2 a,b;
};

class PolygonalTerrain
{
 public:
  ///Negates all of the terrain's x values and segment a/b orders.
  ///Useful for setting up problems with negative initial x velocities.
  void FlipX();
  ///Sorts the segments in increasing x component of their start
  //points
  void Sort();
  ///Returns true if the terrain is sorted in increasing x order
  bool IsSorted() const;
  ///Returns true if the terrain has no segments overlapping one another
  ///in the vertical direction
  bool IsHeightfield() const;
  ///Returns true if the terrain forms a simple connected chain with
  ///increasing x component
  bool IsSimple() const;
  ///Evaluates a point on the terrain given u in [0,1]
  Vector2 Evaluate(Real u) const;
  ///Returns the index of the segment below the given point.  Assumes
  ///segments are sorted by increasing x component
  int SegmentBelow(const Vector2& pt) const;
  ///Returns the point on the terrain below the given point.  Assumes
  ///segments are sorted by increasing x component
  Vector2 PointBelow(const Vector2& pt) const;
  ///Retrieves a window of a terrain inside a bounding box
  void Range(const Vector2& bmin,const Vector2& bmax,PolygonalTerrain& result) const;

  std::vector<Segment> segments;
};

class Problem
{
 public:
  enum SimulationResult { Capture, Overshoot, Undershoot, Slip, MaxItersReached };

  ///initialize empty problem.  Gravity assumed to be 9.8m/s^2
  Problem();
  ///initialize standard CP problem along horizontal COM trajectory
  ///with height h, x velocity vx0.  x position, z velocity are assumed
  ///to be 0.  Gravity assumed to be 9.8m/s^2
  Problem(Real h,Real vx0); 
  void SetInitialConditions(const Vector2& x0,const Vector2& v0);
  void SetGravity(Real g);
  void SetFriction(Real mu);
  void SetStancePoint(const Vector2& p);

  ///Solves for a CP that keeps the robot stable along the quadratic path
  ///starting at x0,v0 and has second derivative dz^2/d^2x = pathCurvature.
  ///Sets the pathCurvature and capturePoint attributes. Returns true if
  ///successful.  Returns false if not, and capturePoint is set to the best
  ///found capture point in terms of maximizing time before overshoot / 
  ///undershoot (Note: if false is returned, it may not be reachable...).
  bool SolveCPFromCurvature(Real pathCurvature,Real tolerance=1e-5);
  ///Solves for a path curvature with the given CP, that keeps the robot
  ///stable along the quadratic path starting at x0,v0 and has second
  ///derivative dz^2/d^2x = pathCurvature.  Sets the pathCurvature and
  ///capturePoint attributes.  Returns true if successful.  Returns false
  ///if not, and pathCurvature is set to the best found path curvature
  ///in terms of maximizing time before overshoot / undershoot. (Note: if
  ///false is returned, it may not be reachable...)
    bool SolveCurvatureFromCP(const Vector2& capturePoint,const Vector2& capturePointNormal,Real tolerance=1e-5);
  ///Solves for all (path curvature,CP) pairs for which CP forms a capture
  ///point for the quadratic path starting at x0,v0 and has second derivative
  ///dz^2/d^2x = pathCurvature.  The terrain is discretized to the given
  ///resolution.  (This is faster than just looping through terrain points and
  ///calling SolveCurvatureFromCP)
  void SolveAllCurvatureCPs(Real resolution,std::vector<Real>& pathCurvatureList,std::vector<Vector2>& cpList);

  ///Helper: returns the z for a given x value
  Real PathHeight(Real x) const;
  ///Helper: solves for a CP on the given segment
  bool SolveCPFromCurvature(Real pathCurvature,const Segment& segment,Real tolerance=1e-3);
  ///Given the initial conditions (x0,v0) and the current values of
  ///pathCurvature and capturePoint, performs a simulation trace.
  SimulationResult SimulationTrace(Real dt,int maxSteps,std::vector<Vector2>& xtrace,std::vector<Vector2>& vtrace) const;
  ///Given the initial conditions (x0,v0) and the current values of
  ///pathCurvature and capturePoint, calculate the result of simulation
  SimulationResult Simulate(Real dt,int maxSteps,Vector2& xfinal,Vector2& vfinal,int* numSteps=NULL) const;
  ///Returns true if the given cm satisfies the reachability requirement of
  ///the cp
  bool ReachableTerminalCM(const Vector2& cm,const Vector2& cp) const;

  ///initial position, initial velocity (x-z plane)
  Vector2 x0,v0;
  ///z-direction curvature of COM path
  Real pathCurvature;
  ///value of the capture point
  Vector2 capturePoint;
  ///upward-facing normal of capture point
  Vector2 capturePointNormal;

  ///terrain available for capture points
  PolygonalTerrain terrain;
  ///friction coefficient (default inf)
  Real frictionCoefficient;

  ///gravity strength (default 9.8)
  Real g;
  ///min/maximum G's that the robot can apply on the COM with its legs
  ///(default 0 and 3)
  Real minGs,maxGs;
  ///the integration time step (default 1e-2)
  Real timeStep;
  ///minimum and maximum limits of CM height above terrain (default 0 and inf)
  Real hmin,hmax;
  ///Leg lengths defining minimum and maximum limits of CM distance to
  ///capture point and stance point (default 0 and inf)
  Real Lmin,Lmax;


  ///if true, this can help limit the search 
  bool hasStancePoint; 
  ///value of the current stance point
  Vector2 stancePoint;
		       
  ///Tolerances for CM terminal condition for considering a point
  ///to be a capture point. Specifically, the 
  ///resulting position of CM must be within cpXTolerance of the CP in the 
  ///x direction, and within cpVTolerance of zero velocity.  Larger values
  ///might make the solver converge faster.  (default 1e-3)
  Real cpXTolerance,cpVTolerance;
};



} //namespace GCP

#endif
