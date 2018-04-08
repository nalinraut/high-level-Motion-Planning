#ifndef OMPL_INTERFACE_H
#define OMPL_INTERFACE_H

#if HAVE_OMPL

#include <planning/CSpace.h>
#include <planning/AnyMotionPlanner.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/MotionValidator.h>
#include <ompl/base/Planner.h>
namespace ob = ompl::base;

class OMPLCSpace;
class CSpaceOMPLSpaceInformation;
class CSpaceOMPLStateSpace;
class CSpaceOMPLStateSampler;
class CSpaceOMPLValidityChecker;
class KrisLibraryOMPLPlanner;

///helper: convert a Config to an OMPL state
ob::State * ToOMPL(const ob::SpaceInformationPtr& si,const Config& q);

///helper: convert an OMPL state to a Config
Config FromOMPL(const ob::SpaceInformationPtr& si,const ob::State* s);


/** @brief An adaptor that maps OMPL spaces to Klamp't spaces.
 * Must call SetSpaceBounds before using.
 */
class OMPLCSpace : public CSpace
{
public:
  OMPLCSpace(const ob::SpaceInformationPtr& si);
  virtual ~OMPLCSpace() {};
  void SetSpaceBounds(const Config& qmin, const Config& qmax);
  ///helper: sets uniform bounds [qmin,qmax] on each dimension
  void SetSpaceBounds(const double& qmin, const double& qmax);
  ///helper: convert a Config to an OMPL state
  ob::State * ToOMPL(const Config& q) const;
  ///helper: convert an OMPL state to a Config
  Config FromOMPL(const ob::State *) const;
  virtual void Sample(Config& x);
  virtual bool IsFeasible(const Config& x);
  virtual EdgePlanner* LocalPlanner(const Config& x,const Config& y);
  
  ob::SpaceInformationPtr si_;
  Config qMin, qMax;
  int nD;
  double resolution;
};

/** @brief Adapts a CSpace into a SpaceInformation that can be used with
 * OMPL.
 *
 * The StateSpace points to an instance of CSpaceOMPLStateSpace.
 */
class CSpaceOMPLSpaceInformation : public ob::SpaceInformation
{
 public:
  CSpaceOMPLSpaceInformation(CSpace* space);
  ob::State * ToOMPL(const Config& q) const;
  Config FromOMPL(const ob::State *) const;

  CSpace* cspace;
};

/* @brief Creates an OMPL planner from a KrisLibrary planner. 
 *
 * Constructor assumes the factory type is completely specified when setting
 * the OMPL planner name.
 */
class KrisLibraryOMPLPlanner : public ob::Planner
{
 public:
  KrisLibraryOMPLPlanner(const ob::SpaceInformationPtr &si,const MotionPlannerFactory& factory);
  virtual void clear();
  virtual void setup();
  virtual ob::PlannerStatus solve (const ob::PlannerTerminationCondition &ptc);
  virtual void getPlannerData (ob::PlannerData &data) const;

  MotionPlannerFactory factory;
  SmartPointer<OMPLCSpace> cspace;
  SmartPointer<MotionPlannerInterface> planner;
};

/** @brief Adapts a CSpace into a StateSpace that can be used with
 * OMPL.
 */
class CSpaceOMPLStateSpace : public ob::RealVectorStateSpace
{
 public:
  CSpaceOMPLStateSpace(CSpace* space);
  virtual unsigned int getDimension (void) const;
  virtual double getMaximumExtent (void) const;
  virtual void enforceBounds (ob::State *state) const;
  virtual bool satisfiesBounds (const ob::State *state) const;
  virtual double distance (const ob::State *state1, const ob::State *state2) const;
  virtual void interpolate (const ob::State *from, const ob::State *to, const double t, ob::State *state) const;
  virtual ob::StateSamplerPtr allocDefaultStateSampler (void) const;

  CSpace* space;
};

/** @brief Adapts a CSpace into a state sampler that can be used with
 * OMPL.
 */
class CSpaceOMPLStateSampler : public ob::StateSampler
{
 public:
  CSpaceOMPLStateSampler(const CSpaceOMPLStateSpace* space);
  virtual void sampleUniform (ob::State *state);
  virtual void sampleUniformNear (ob::State *state, const ob::State *near, const double distance); 
  virtual void sampleGaussian (ob::State *state, const ob::State *mean, const double stdDev);

  CSpace* space;
};

/** @brief Adapts a CSpace into a validity checker that can be used with
 * OMPL.
 */
class CSpaceOMPLValidityChecker : public ob::StateValidityChecker
{
 public:
  CSpaceOMPLValidityChecker(CSpaceOMPLSpaceInformation* space);
  virtual bool isValid(const ob::State* state) const;

  CSpace* space;
};

/** @brief Adapts a CSpace into a motion validator that can be used with
 * OMPL.
 */
class CSpaceOMPLMotionValidator : public ob::MotionValidator
{
 public:
  CSpaceOMPLMotionValidator(CSpaceOMPLSpaceInformation* space);
  virtual bool checkMotion (const ob::State *s1, const ob::State *s2) const;
  virtual bool checkMotion (const ob::State *s1, const ob::State *s2, std::pair< ob::State *, double > &lastValid) const;

  CSpace* space;
};


#endif //HAVE_OMPL

#endif
