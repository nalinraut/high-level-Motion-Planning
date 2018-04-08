#ifndef PENDULUM_CSPACE_H
#define PENDULUM_CSPACE_H

#include <KrisLibrary/planning/RigidBodyCSpace.h>
#include <KrisLibrary/planning/EdgePlannerHelpers.h>
#include <KrisLibrary/planning/KinodynamicSpace.h>
#include <KrisLibrary/planning/Objective.h>
#include <KrisLibrary/planning/InterpolatorHelpers.h>
#include <KrisLibrary/math/angle.h>


/** @brief A kinodynamic space defining a pendulum swingup task
 *
 * The configuration is (x,v), with x from 0 to 2pi and v from
 * -2pi to 2pi.
 *
 * The control input is (dt,u) where dt is the time and u is the acceleration
 * in the range -0.5 to 0.5.
 *
 * Gravity also pulls down the pendulum with acceleration -9.8m/s^2
 *
 * The state space is a MultiCSpace(SO2CSpace,BoxCSpace)) and the control space is an
 * PendulumControlSpace.
 */
class PendulumKinodynamicSpace : public KinodynamicSpace
{
 public:
  PendulumKinodynamicSpace();
  virtual EdgePlanner* TrajectoryChecker(const ControlInput& u,const SmartPointer<Interpolator>& path);
  void SetVisibilityTolerance(Real tol);
  void SetVelocityLimits(Real vmin,Real vmax);
  void SetAccelerationLimits(Real amin,Real amax);

  Real visibilityTolerance;
};

/** @brief The standard pendulum objective function measuring
 * time to goal.
 */
class ControlTimeObjectiveFunction : public ObjectiveFunctionalBase
{
public:
  virtual double IncrementalCost(const Interpolator* path) { FatalError("No notion of incremental cost without a control\n"); return 0; }
  virtual double IncrementalCost(const ControlInput& u,const Interpolator* path) { return Abs(u(0)); }
};

class PendulumControlSpace : public IntegratedControlSpace
{
public:
  PendulumControlSpace();
  virtual std::string VariableName(int i);
  virtual void Derivative(const State& x, const ControlInput& u,State& dx);
  void SetAccelerationLimits(Real amin,Real amax);
  void SetMassLength(Real L);
  void SetGravity(Real g);

  Real L;
  Real gravityAcceleration;
};




PendulumControlSpace::PendulumControlSpace()
:IntegratedControlSpace(new FiniteSet(Vector(1,-0.5),Vector(1,0.0),Vector(1,0.5)),0.01,0.25),
L(1),gravityAcceleration(-9.8)
{}

std::string PendulumControlSpace::VariableName(int i) { return "ddx"; }

void PendulumControlSpace::Derivative(const State& x, const ControlInput& u,State& dx)
{
  assert(x.n == 2);
  dx.resize(2);
  dx[0] = x[1];
  Real momentArm = L*Cos(x[0]);
  dx[1] = gravityAcceleration*momentArm + u[0];
}

void PendulumControlSpace::SetAccelerationLimits(Real amin,Real amax)
{
  SetBaseControlSet(new FiniteSet(Vector(1,amin),Vector(1,0.0),Vector(1,amax)));
}

void PendulumControlSpace::SetMassLength(Real _L)
{
  L = _L;
}

void PendulumControlSpace::SetGravity(Real g)
{
  gravityAcceleration = g;
}



PendulumKinodynamicSpace::PendulumKinodynamicSpace()
:KinodynamicSpace(NULL,NULL),visibilityTolerance(1e-2)
{
  MultiCSpace* mspace = new MultiCSpace;
  mspace->Add("x",new SO2CSpace());
  mspace->Add("v",new BoxCSpace(-Pi*2,Pi*2));
  stateSpace = mspace;
  controlSpace = new PendulumControlSpace;
}

EdgePlanner* PendulumKinodynamicSpace::TrajectoryChecker(const ControlInput& u,const SmartPointer<Interpolator>& path)
{
  return new EpsilonEdgeChecker(stateSpace,path,visibilityTolerance);
}

void PendulumKinodynamicSpace::SetVisibilityTolerance(Real tol)
{
  visibilityTolerance = tol;
}

void PendulumKinodynamicSpace::SetVelocityLimits(Real vmin,Real vmax)
{
  MultiCSpace* mspace = dynamic_cast<MultiCSpace*>(&*stateSpace);
  BoxCSpace* vspace = dynamic_cast<BoxCSpace*>(&*mspace->components[1]);
  Assert(vspace != NULL);
  vspace->SetDomain(Vector(1,vmin),Vector(1,vmax));
}

void PendulumKinodynamicSpace::SetAccelerationLimits(Real amin,Real amax)
{
  PendulumControlSpace* cspace = dynamic_cast<PendulumControlSpace*>(&*controlSpace);
  cspace->SetAccelerationLimits(amin,amax);
}


#endif
