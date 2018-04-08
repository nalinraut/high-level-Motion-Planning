#include "OMPLInterface.h"
#include <planning/EdgePlanner.h>
#include <math/random.h>

#if HAVE_OMPL

#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/goals/GoalState.h>
namespace og = ompl::geometric;

///helper: convert a Config to an OMPL state
ob::State * ToOMPL(const ob::StateSpace* si_,const Config& q)
{
  ob::State* omplstate = si_->allocState();
  vector<Real> vq = q;
  si_->copyFromReals(omplstate,vq);
  return omplstate;
}

///helper: convert an OMPL state to a Config
Config FromOMPL(const ob::StateSpace* si_,const ob::State* s)
{
  vector<Real> vq;
  si_->copyToReals(vq,s);
  return Config(vq);
}


///helper: convert a Config to an OMPL state
ob::State * ToOMPL(const ob::SpaceInformation* si_,const Config& q)
{
  ob::State* omplstate = si_->allocState();
  vector<Real> vq = q;
  si_->getStateSpace()->copyFromReals(omplstate,vq);
  return omplstate;
}

///helper: convert an OMPL state to a Config
Config FromOMPL(const ob::SpaceInformation* si_,const ob::State* s)
{
  vector<Real> vq;
  si_->getStateSpace()->copyToReals(vq,s);
  return Config(vq);
}

og::PathGeometric* ToOMPL(const ob::SpaceInformationPtr& si,const MilestonePath& path)
{
  og::PathGeometric* res = new og::PathGeometric(si);
  for(size_t i=0;i<path.NumMilestones();i++) {
    ob::State* s=ToOMPL(si,path.GetMilestone(i));
    res->append(s);
    si->freeState(s);
  }
}



OMPLCSpace::OMPLCSpace( const ob::SpaceInformationPtr& si)
{
  nD = si->getStateDimension();
  qMin.resize(nD);
  qMax.resize(nD);
  qMin.set(0.0);
  qMax.set(1.0);
  resolution = 0.001;
  si_ = si;
}

void OMPLCSpace::SetSpaceBounds(const Config& qmin, const Config& qmax) {
	qMin = qmin;
	qMax = qmax;
}

void OMPLCSpace::SetSpaceBounds(const double& qmin, const double& qmax) {
	qMin.set(qmin);
	qMax.set(qmax);
}

void OMPLCSpace::Sample(Config& x) {
	x.resize(nD);
	for(int i = 0; i < nD; i++)
		x[i] = Rand(qMin[i], qMax[i]);
}

ob::State * OMPLCSpace::ToOMPL(const Config& q) const
{
  return ::ToOMPL(si_,q);
}

Config OMPLCSpace::FromOMPL(const ob::State * s) const
{
  return ::FromOMPL(si_,s);
}

bool OMPLCSpace::IsFeasible(const Config& x) {
  //ob::RealVectorStateSpace* rspace = (si_->getStateSpace())->as<ob::RealVectorStateSpace>();
  //assert(rspace != NULL);
  ob::State *omplstate = ToOMPL(x);
  bool res = si_->getStateValidityChecker()->isValid(omplstate);
  si_->freeState(omplstate);
  return res;
}

EdgePlanner* OMPLCSpace::LocalPlanner(const Config& x, const Config& y)
{
  return new StraightLineEpsilonPlanner(this, x, y, resolution);
}

CSpaceOMPLSpaceInformation::CSpaceOMPLSpaceInformation(CSpace* _space)
  :ob::SpaceInformation(ob::StateSpacePtr(new CSpaceOMPLStateSpace(_space))),cspace(_space)
{
  setStateValidityChecker(ob::StateValidityCheckerPtr(new CSpaceOMPLValidityChecker(this)));
  setMotionValidator(ob::MotionValidatorPtr(new CSpaceOMPLMotionValidator(this)));
}

ob::State * CSpaceOMPLSpaceInformation::ToOMPL(const Config& q) const
{
  return ::ToOMPL(this,q);
}

Config CSpaceOMPLSpaceInformation::FromOMPL(const ob::State * s) const
{
  return ::FromOMPL(this,s);
}

KrisLibraryOMPLPlanner::KrisLibraryOMPLPlanner(const ob::SpaceInformationPtr &si,const MotionPlannerFactory& _factory)
  :ob::Planner(si,_factory.type.c_str()),factory(_factory)
{}

void KrisLibraryOMPLPlanner::clear()
{
  planner = NULL;
  ob::Planner::clear();
}

void KrisLibraryOMPLPlanner::setup()
{
  ob::ProblemDefinitionPtr pdef = this->getProblemDefinition();
  cspace = new OMPLCSpace(pdef->getSpaceInformation());
  MotionPlanningProblem problem;
  problem.space = cspace;
  Assert(pdef->getStartStateCount() == 1);
  problem.qstart = cspace->FromOMPL(pdef->getStartState(0));
  const ob::GoalState* goalstate = pdef->getGoal()->as<ob::GoalState>();
  if(goalstate)
    problem.qgoal = cspace->FromOMPL(goalstate->getState());
  else {
    FatalError("TODO: create a goalSet CSpace from an OMPL goal set");
  }
  //TODO: objective functions?
  planner = factory.Create(problem);
  ob::Planner::setup();
}

void KrisLibraryOMPLPlanner::getPlannerData (ob::PlannerData &data) const
{
  if(!planner) return;
  RoadmapPlanner roadmap(const_cast<CSpace*>((const CSpace*)cspace));
  MotionPlannerInterface* iplanner = const_cast<MotionPlannerInterface*>((const MotionPlannerInterface*)planner);
  iplanner->GetRoadmap(roadmap);
  for(size_t i=0;i<roadmap.roadmap.nodes.size();i++) {
    unsigned int id = data.addVertex(ob::PlannerDataVertex(cspace->ToOMPL(roadmap.roadmap.nodes[i]),(int)i));
    Assert(id == i);
  }
  for(size_t i=0;i<roadmap.roadmap.nodes.size();i++) {
    RoadmapPlanner::Roadmap::Iterator e;
    for(roadmap.roadmap.Begin(i,e);!e.end();e++)
      data.addEdge(e.source(),e.target());
  }
}

ob::PlannerStatus KrisLibraryOMPLPlanner::solve (const ob::PlannerTerminationCondition &ptc)
{
  while(!ptc()) {
    if(planner->IsSolved()) {
      //convert solution to OMPL solution
      MilestonePath path;
      planner->GetSolution(path);
      ob::PathPtr pptr(ToOMPL(si_,path));
      this->getProblemDefinition()->addSolutionPath(pptr);
      return ob::PlannerStatus(true,false);
    }
    planner->PlanMore();  
  }
  if(planner->IsSolved()) {
    //convert solution to OMPL solution
    MilestonePath path;
    planner->GetSolution(path);
    ob::PathPtr pptr(ToOMPL(si_,path));
    this->getProblemDefinition()->addSolutionPath(pptr);
    return ob::PlannerStatus(true,false);
  }
  return ob::PlannerStatus(false,false);
}


int NumDimensions(CSpace* s)
{
  Vector x;
  s->Sample(x);
  return x.n;
}

CSpaceOMPLStateSpace::CSpaceOMPLStateSpace(CSpace* _space)
  :ob::RealVectorStateSpace(NumDimensions(_space)),space(_space)
{}

unsigned int CSpaceOMPLStateSpace::getDimension (void) const
{
  return NumDimensions(space);
}

double CSpaceOMPLStateSpace::getMaximumExtent (void) const
{
  return Inf;
}

void CSpaceOMPLStateSpace::enforceBounds (ob::State *state) const
{
}

bool CSpaceOMPLStateSpace::satisfiesBounds (const ob::State *state) const
{
  return true;
}

double CSpaceOMPLStateSpace::distance (const ob::State *state1, const ob::State *state2) const
{
  return space->Distance(FromOMPL(this,state1),FromOMPL(this,state2));
}

void CSpaceOMPLStateSpace::interpolate (const ob::State *from, const ob::State *to, const double t, ob::State *state) const
{
  Vector x;
  space->Interpolate(FromOMPL(this,from),FromOMPL(this,to),t,x);
  this->copyFromReals(state,x);
}

ob::StateSamplerPtr CSpaceOMPLStateSpace::allocDefaultStateSampler (void) const
{
  return ob::StateSamplerPtr(new CSpaceOMPLStateSampler(this));
}


CSpaceOMPLStateSampler::CSpaceOMPLStateSampler(const CSpaceOMPLStateSpace* _space)
  :ob::StateSampler(_space),space(_space->space)
{}

void CSpaceOMPLStateSampler::sampleUniform (ob::State *state)
{
  Vector x;
  space->Sample(x);
  space_->copyFromReals(state,x);
}

void CSpaceOMPLStateSampler::sampleUniformNear (ob::State *state, const ob::State *near, const double distance)
{
  Config c = FromOMPL(space_,near);
  Vector x;
  space->SampleNeighborhood(c,distance,x);
  space_->copyFromReals(state,x);
}

void CSpaceOMPLStateSampler::sampleGaussian (ob::State *state, const ob::State *mean, const double stdDev)
{
  sampleUniformNear(state,mean,stdDev*2);
}

CSpaceOMPLValidityChecker::CSpaceOMPLValidityChecker(CSpaceOMPLSpaceInformation* _space)
  :ob::StateValidityChecker(_space),space(_space->cspace)
{
  specs_.clearanceComputationType = ob::StateValidityCheckerSpecs::NONE;
  specs_.hasValidDirectionComputation = false;
}

bool CSpaceOMPLValidityChecker::isValid(const ob::State* state) const
{
  return space->IsFeasible(FromOMPL(si_,state));
}

CSpaceOMPLMotionValidator::CSpaceOMPLMotionValidator(CSpaceOMPLSpaceInformation* _space)
  :ob::MotionValidator(_space),space(_space->cspace)
{
}

bool CSpaceOMPLMotionValidator::checkMotion (const ob::State *s1, const ob::State *s2) const
{
  EdgePlanner* e=space->LocalPlanner(FromOMPL(si_,s1),FromOMPL(si_,s2));
  if(e->IsVisible()) {
    delete e;
    return true;
  }
  return false;
}

bool CSpaceOMPLMotionValidator::checkMotion (const ob::State *s1, const ob::State *s2, std::pair< ob::State *, double > &lastValid) const
{
  EdgePlanner* e=space->LocalPlanner(FromOMPL(si_,s1),FromOMPL(si_,s2));
  if(e) {
    delete e;
    return true;
  }
  lastValid.first = si_->cloneState(s1);
  lastValid.second = 0;
  return false;
}


#endif // HAVE_OMPL
