#ifndef CAR_CSPACE_H
#define CAR_CSPACE_H

#include <KrisLibrary/planning/RigidBodyCSpace.h>
#include <KrisLibrary/planning/EdgePlannerHelpers.h>
#include <KrisLibrary/planning/KinodynamicSpace.h>
#include <KrisLibrary/planning/Objective.h>
#include <KrisLibrary/planning/InterpolatorHelpers.h>
#include <KrisLibrary/math/angle.h>


/** @brief A kinodynamic space defining a car-like robot in 2D (dubins car)
 *
 * The configuration is (x,y,theta), with (x,y) from bmin to bmax, and
 * theta from 0 to 2pi.
 *
 * The control input is (phi,d) where phi determines the curvature
 * of the path, and d is the distance traveled.  Phi varies between
 * turnmin and turnmax, and d varies between dmin and dmax.
 *
 * The state space is a CarCSpace and the control space is a CarControlSpace.
 */
class CarKinodynamicSpace : public KinodynamicSpace
{
 public:
  CarKinodynamicSpace();
  virtual EdgePlanner* TrajectoryChecker(const ControlInput& u,const SmartPointer<Interpolator>& path);
  void SetObstacles(CSpace* obstacleSpace);
  void SetVisibilityTolerance(Real tol);
  void SetAngleWeight(Real w);
  void SetSteeringLimits(Real turnmin,Real turnmax,Real dmin,Real dmax);
  void CarTransform(const State& x,RigidTransform2D& T) const;

  Real visibilityTolerance;
};

/** @brief The standard car objective function measuring
 * path length of the 2D trace of the motion.
 */
class CarLengthObjectiveFunction : public ObjectiveFunctionalBase
{
public:
  virtual double IncrementalCost(const Interpolator* path) { FatalError("No notion of incremental cost without a control\n"); return 0; }
  virtual double IncrementalCost(const ControlInput& u,const Interpolator* path) { return Abs(u(1)); }
};


/** @brief Basically SE2CSpace but with extra obstacles. */
class CarCSpace : public SE2CSpace
{
public:
  CarCSpace(Real xmin,Real xmax);
  void SetObstacles(CSpace* obstacles);
  void SetVisibilityTolerance(Real tol);
  virtual bool IsFeasible(const Config& x);
  virtual Real ObstacleDistance(const Config& x);
  virtual EdgePlanner* PathChecker(const Config& a,const Config& b) {
    return new EpsilonEdgeChecker(this,a,b,visibilityTolerance);
  }

  bool obstacle2d;
  CSpace* obstacleSpace;
  Real visibilityTolerance;
};

/** @brief Control is (phi,d) sampled from the range
 * [phimin,phimax] x [dmin,dmax]
 */
class CarSteeringControlSet : public BoxSet
{
public:
  CarSteeringControlSet();
  void SetSteeringLimits(Real turnmin,Real turnmax,Real dmin,Real dmax);
  virtual void Sample(const State& x,ControlInput& u);

  Real turnmin,turnmax;
  Real dmin,dmax;
};

class CarControlSpace : public ControlSpace
{
public:
  CarControlSpace(CarCSpace* _cspace);
  void SetVisibilityTolerance(Real tol);
  void SetSteeringLimits(Real turnmin,Real turnmax,Real dmin,Real dmax);
  virtual Interpolator* Simulate(const State& x0, const ControlInput& u);
  virtual void Successor(const State& x0, const ControlInput& u,State& x1);

  CarCSpace* cspace;
};




CarCSpace::CarCSpace(Real xmin,Real xmax)
:SE2CSpace(xmin,xmax),obstacleSpace(NULL),visibilityTolerance(0.01)
{}

void CarCSpace::SetObstacles(CSpace* obstacles) 
{
  if(obstacles->NumDimensions() == 2) obstacle2d = true;
  else obstacle2d = false;
  obstacleSpace = obstacles;
}

void CarCSpace::SetVisibilityTolerance(Real tol)
{
  visibilityTolerance=tol; 
}

bool CarCSpace::IsFeasible(const Config& x)
{
  if(!SE2CSpace::IsFeasible(x)) return false;
  if(obstacleSpace) {
    if(obstacle2d) {
Config x2(2);
x2(0)=x(0);
x2(1)=x(1);
return obstacleSpace->IsFeasible(x2);
    }
    else {
return obstacleSpace->IsFeasible(x);
    }
  }
  return true;
}

Real CarCSpace::ObstacleDistance(const Config& x)
{
  Real dmin = SE2CSpace::ObstacleDistance(x);
  if(obstacleSpace) {
    if(obstacle2d) {
Config x2(2);
x2(0)=x(0);
x2(1)=x(1);
return Min(dmin,obstacleSpace->ObstacleDistance(x2));
    }
    else {
return Min(dmin,obstacleSpace->ObstacleDistance(x));
    }
  }
  return dmin;
}

CarSteeringControlSet::CarSteeringControlSet()
:BoxSet(-1,1,2)
{
  SetSteeringLimits(-1,1,0,1);
}

void CarSteeringControlSet::SetSteeringLimits(Real turnmin,Real turnmax,Real dmin,Real dmax)
{
  BoxSet::bmin[0] = turnmin;
  BoxSet::bmin[1] = dmin;
  BoxSet::bmax[0] = turnmax;
  BoxSet::bmax[1] = dmax;
  this->turnmin = turnmin;
  this->turnmax = turnmax;
  this->dmin = dmin;
  this->dmax = dmax;
}

void CarSteeringControlSet::Sample(const State& x,ControlInput& u)
{
  u.resize(2);
  //if(RandBool(0.5)) { //pick bang control
  if(RandBool(0)) { //pick bang control
    u(0) = Rand(turnmin,turnmax);
    if(Rand()*(Abs(dmin)+Abs(dmax)) < Abs(dmin)) { //go backwards
u(1) = dmin;
    }
    else {
u(1) = dmax;
    }
  }
  else {  //pick random control
    //pick d in probability proportional to f(d)=abs(d)
    //let g(D)=int{d=dmin to D} f(d) / int{d=dmin to dmax} f(d)
    //int{d=dmin to dmax} f(d) = int{d=dmin to 0} -d + int{d=0 to dmax} d
    //=0.5*(dmin^2+dmax^2)
    //if D < 0, int{d=dmin to D} f(d) =
    //          int{d=dmin to D} -d = 0.5*(-D^2+dmin^2)
    //if D > 0, int{d=dmin to D} f(d) =
    //          0.5*dmin^2 + int{d=0 to dmin} d = 0.5*(dmin^2+D^2)
    //so g(D) = D < 0 ? (dmin^2-D^2)/(dmin^2+dmax^2)
    //          D > 0 ? (dmin^2+D^2)/(dmin^2+dmax^2)
    //i.e. g(D) = (dmin^2 + sign(D)D^2)/(dmin^2+dmax^2)
    //
    //thus if s=g(D), sign(D)D^2 = s*(dmin^2+dmax^2)-dmin^2
    Real s = Rand();
    Real d = ((s-1.0)*Sqr(dmin)+s*Sqr(dmax));
    u(0) = Rand(turnmin,turnmax);
    u(1) = (d<0 ? -Sqrt(-d) : Sqrt(d));
  }
}

CarControlSpace::CarControlSpace(CarCSpace* _cspace)
{
  cspace = _cspace;
  SetSteeringLimits(-1,1,0,1);
}

void CarControlSpace::SetSteeringLimits(Real turnmin,Real turnmax,Real dmin,Real dmax)
{
  CarSteeringControlSet* cset = new CarSteeringControlSet();
  cset->SetSteeringLimits(turnmin,turnmax,dmin,dmax);
  myControlSet = cset;
}

class CarInterpolator : public Interpolator
{
public:
  Vector2 pos, fwd, right, cor;
  Real heading;
  Real phi, d;
  State a,b;
  CarInterpolator(const State& x0,const ControlInput& u)
  {
    Assert(x0.n==3);
    Assert(u.n==2);
    pos.set(x0(0),x0(1));
    heading = x0(2);
    fwd.set(Cos(heading),Sin(heading));
    right.set(-fwd.y,fwd.x);
    phi = u(0);
    d = u(1);
    if(!FuzzyZero(phi))
      cor = pos + right/phi;
    a = x0;
    b = x0;
    Eval(1,b);
  }
  virtual ~CarInterpolator() {}
  virtual void Eval(Real u,Config& x) const {
    Vector2 newPos;
    if(FuzzyZero(phi)) {
      newPos = pos + u*d*fwd;
      x.resize(3);
      x(0) = newPos.x;
      x(1) = newPos.y;
      x(2) = heading;
    }
    else {
      //rotate about a center of rotation, with radius 1/phi
      Real theta=u*d*phi;
      Matrix2 R;
      //rotate pos about cor with angle theta
      R.setRotate(theta);
      newPos = R*(pos-cor) + cor;
      x.resize(3);
      x(0)=newPos.x;
      x(1)=newPos.y;
      x(2)=AngleNormalize(heading + theta);
    }
  }

  virtual Real Length() const { return Abs(d); }
  virtual const Config& Start() const { return a; }
  virtual const Config& End() const { return b; } 
  virtual Real ParamStart() const { return 0; }
  virtual Real ParamEnd() const { return 1; }
};

Interpolator* CarControlSpace::Simulate(const State& x0, const ControlInput& u)
{
  return new CarInterpolator(x0,u);
}

void CarControlSpace::Successor(const State& x0, const ControlInput& u,State& x1)
{
  Assert(x0.n==3);
  Assert(u.n==2);
  Vector2 pos(x0(0),x0(1));
  Real heading = x0(2);
  Vector2 fwd(Cos(heading),Sin(heading));
  Vector2 right(-fwd.y,fwd.x);
  Real phi = u(0);
  Real d = u(1);
  Vector2 newPos;
  if(FuzzyZero(phi)) {
    newPos = pos + d*fwd;
    x1.resize(3);
    x1(0) = newPos.x;
    x1(1) = newPos.y;
    x1(2) = heading;
  }
  else {
    //rotate about a center of rotation, with radius 1/phi
    Vector2 cor = pos + right/phi;
    Real theta=0,thetaMax=d*phi;
    Matrix2 R;
    R.setRotate(thetaMax);
    newPos = R*(pos-cor) + cor;
    x1.resize(3);
    x1(0)=newPos.x;
    x1(1)=newPos.y;
    x1(2)=AngleNormalize(heading + thetaMax);
  }
}


CarKinodynamicSpace::CarKinodynamicSpace()
:KinodynamicSpace(NULL,NULL)
{
  CarCSpace* cspace = new CarCSpace(0,1);
  stateSpace = cspace;

  visibilityTolerance = 1e-2;
  controlSpace = new CarControlSpace(cspace);
}

EdgePlanner* CarKinodynamicSpace::TrajectoryChecker(const ControlInput& u,const SmartPointer<Interpolator>& path)
{
  return new EpsilonEdgeChecker(stateSpace,path,visibilityTolerance);
}
void CarKinodynamicSpace::SetObstacles(CSpace* obstacleSpace) {
  CarCSpace* carSpace = dynamic_cast<CarCSpace*>(&*stateSpace);
  carSpace->SetObstacles(obstacleSpace);    
}
void CarKinodynamicSpace::SetVisibilityTolerance(Real tol) {
  CarCSpace* carSpace = dynamic_cast<CarCSpace*>(&*stateSpace);
  carSpace->SetVisibilityTolerance(tol);
  visibilityTolerance=tol;
}
void CarKinodynamicSpace::SetAngleWeight(Real w) {
  CarCSpace* carSpace = dynamic_cast<CarCSpace*>(&*stateSpace);
  carSpace->SetAngleWeight(w);
}

void CarKinodynamicSpace::SetSteeringLimits(Real turnmin,Real turnmax,Real dmin,Real dmax)
{
  CarControlSpace* carControlSpace = dynamic_cast<CarControlSpace*>(&*controlSpace);
  carControlSpace->SetSteeringLimits(turnmin,turnmax,dmin,dmax);
}


void CarKinodynamicSpace::CarTransform(const State& x,RigidTransform2D& T) const
{
  return SE2CSpace::GetTransform(x,T);
}



#endif
