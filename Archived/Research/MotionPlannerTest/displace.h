#include <KrisLibrary/planning/DisplacementPlanner.h>
#include <KrisLibrary/math/sample.h>
#include <KrisLibrary/math/angle.h>
#include <KrisLibrary/Timer.h>
#include <KrisLibrary/utils/StatCollector.h>


class RotationDisplacementSpace : public CSpace
{
public:
  Real thetamin,thetamax;
  Real costWeight;

  RotationDisplacementSpace(Real _thetamin=0,Real _thetamax=TwoPi)
    :thetamin(_thetamin),thetamax(_thetamax),costWeight(1)
    {}
  virtual void Sample(Config& x) { 
    x.resize(1);
    x(0) = Rand(thetamin,thetamax);
  }
  virtual void SampleNeighborhood(const Config& c,Real r,Config& x) {
    Assert(c.n == 1);
    x.resize(1);
    if(thetamin <= 0 && thetamax >= TwoPi)
      x(0) = Rand(c(0)-r/costWeight,c(0)+r/costWeight);
    else
      x(0) = Rand(Max(c(0)-r/costWeight,thetamin),Min(c(0)+r/costWeight,thetamax));
    x(0) = AngleNormalize(x(0));
  }
  virtual bool IsFeasible(const Config&) { return true; }
  virtual EdgePlanner* LocalPlanner(const Config& a,const Config& b) { return new TrueEdgePlanner(this,a,b); }

  virtual Real Distance(const Config& x, const Config& y) { return Abs(AngleDiff(x(0),y(0)))*costWeight; }
  virtual void Interpolate(const Config& x, const Config& y,Real u,Config& out) { 
    out.resize(1);
    out(0) = AngleInterp(AngleNormalize(x(0)),AngleNormalize(y(0)),u); 
  }
  virtual void Midpoint(const Config& x, const Config& y,Config& out) { 
    Interpolate(x,y,0.5,out);
  }
};

class BoxDisplacementSpace : public CSpace
{
public:
  Vector bmin,bmax;
  Real costWeight;

  BoxDisplacementSpace(const Vector& _bmin,const Vector& _bmax)
    :bmin(_bmin),bmax(_bmax),costWeight(1)
    {}
  virtual void Sample(Config& x) { 
    x.resize(bmin.n);
    for(int i=0;i<x.n;i++)
      x(i) = Rand(bmin(i),bmax(i));
  }
  virtual void SampleNeighborhood(const Config& c,Real r,Config& x) {
    Assert(c.n == bmin.n);
    x.resize(c.n);
    for(int i=0;i<c.n;i++)
      x(i) = Rand(Max(c(i)-r/costWeight,bmin(i)),Min(c(i)+r/costWeight,bmax(i)));
  }
  virtual bool IsFeasible(const Config&) { return true; }
  virtual EdgePlanner* LocalPlanner(const Config& a,const Config& b) { return new TrueEdgePlanner(this,a,b); }

  virtual Real Distance(const Config& x, const Config& y) { return Distance_LInf(x,y)*costWeight; }
};

class SphereDisplacementSpace : public CSpace
{
public:
  int ndims;
  Real radius;
  Real costWeight;

  SphereDisplacementSpace(int _ndims,Real _radius)
    :ndims(_ndims),radius(_radius),costWeight(1)
  {}
  virtual void Sample(Config& x) { 
    vector<Real> vx(ndims);
    SampleHyperBall(radius/costWeight,vx);
    x = vx;
  }
  virtual void SampleNeighborhood(const Config& c,Real r,Config& x) {
    if(r/costWeight > radius) r=radius;
    else r = r/costWeight;
    vector<Real> disp(ndims);
    SampleHyperBall(r,disp);
    x = c+Vector(disp);
  }
  virtual bool IsFeasible(const Config&) { return true; }
  virtual EdgePlanner* LocalPlanner(const Config& a,const Config& b) { return new TrueEdgePlanner(this,a,b); }

  virtual Real Distance(const Config& x, const Config& y) { return Distance_L2(x,y)*costWeight; }
};

class GeometricDisplacementCSpace : public ObstacleDisplacementCSpace
{
public:
  enum MotionType { Fixed, Translation, TranslationX, TranslationY, Rotation };
  Geometric2DCSpace* space;
  vector<MotionType> motionType;

  GeometricDisplacementCSpace(Geometric2DCSpace* _space)
    :space(_space),motionType(_space->NumObstacles(),Translation)
  {
    //boundaries
    for(int i=0;i<4;i++) motionType[i]=Fixed;
  }
  virtual void Sample(Config& q) { space->Sample(q); }
  virtual int NumObstacles() { return space->NumObstacles();}
  virtual string ObstacleName(int i) { return space->ObstacleName(i); }
  virtual CSpace* DisplacementSpace(int obstacle) const { 
    switch(motionType[obstacle]) {
    case Fixed: return NULL;
    case Translation:  return new SphereDisplacementSpace(2,1.0);
    case TranslationX: 
      {
	Vector bmin(2,Vector2(-1.0,0)),bmax(2,Vector2(1,0));
	return new BoxDisplacementSpace(bmin,bmax);
      }
    case TranslationY:
      {
	Vector bmin(2,Vector2(0,-1.0)),bmax(2,Vector2(0,1));
	return new BoxDisplacementSpace(bmin,bmax);
      }
    case Rotation:
      return new RotationDisplacementSpace();
    default:
      return NULL;
    }
  }
  virtual bool IsFeasible(const Config& q,int obstacle,const Vector& d) {
    if(motionType[obstacle]==Fixed) return space->IsFeasible(q,obstacle); 
    else if(motionType[obstacle]==Rotation) {
      GeometricPrimitive2D geom = space->Obstacle(obstacle-4);
      AABB2D bb;
      geom.ToBound(bb);
      Vector2 c=(bb.bmin+bb.bmax)*0.5;
      RigidTransform2D T;
      T.setIdentity();
      T.R.setRotate(d(0));
      T.t = c-T.R*c;
      geom.Transform(T);
      return !geom.Collides(Vector2(q(0),q(1)));
    }
    else {
      Config temp=q-d;
      //cout<<"Domain: "<<space->domain.bmin<<", "<<space->domain.bmax<<endl;
      //cout<<"test: "<<q<<", "<<obstacle<<", "<<d<<", res "<<space->IsFeasible(temp,obstacle)<<endl;
      return space->IsFeasible(temp,obstacle);
    }
  }
  virtual bool IsFeasibleAll(const Config& q,int obstacle) { return false; }
  virtual bool IsVisibleAll(const Config& a,const Config& b,int obstacle) { return false; }
  virtual EdgePlanner* LocalPlanner(const Config& a,const Config& b) {
    return new ExplicitEdgePlanner(this,a,b);
  }
  virtual EdgePlanner* LocalPlanner(const Config& a,const Config& b,int obstacle) {
    if(motionType[obstacle]==Fixed) return space->LocalPlanner(a,b,obstacle);
    if(motionType[obstacle]==Rotation) {
      Segment2D s;
      s.a.set(a);
      s.b.set(b);
      GeometricPrimitive2D geom = space->Obstacle(obstacle-4);
      AABB2D bb;
      geom.ToBound(bb);
      Vector2 c=(bb.bmin+bb.bmax)*0.5;
      RigidTransform2D T;
      T.setIdentity();
      T.R.setRotate(obstacleDisplacements[obstacle](0));
      T.t = c-T.R*c;
      geom.Transform(T);
      if(geom.Collides(s)) return new FalseEdgePlanner(this,a,b);
      else return new TrueEdgePlanner(this,a,b);
    }
    else {
      Config tempa,tempb;
      tempa = a-obstacleDisplacements[obstacle];
      tempb = b-obstacleDisplacements[obstacle];
      return space->LocalPlanner(tempa,tempb,obstacle);
    }
  }
};

class TranslatingDisplacementCSpace : public ObstacleDisplacementCSpace
{
public:
  TranslatingRobot2DCSpace* space;
  enum MotionType { Fixed, Translation, TranslationX, TranslationY, Rotation };
  vector<MotionType> motionType;

  TranslatingDisplacementCSpace(TranslatingRobot2DCSpace* _space)
    :space(_space),motionType(_space->NumObstacles(),Translation)
  {
    //boundaries
    for(int i=0;i<1;i++) motionType[i]=Fixed;
  }
  virtual void Sample(Config& q) { space->Sample(q); }
  virtual int NumObstacles() { return space->NumObstacles();}
  virtual string ObstacleName(int i) { return space->ObstacleName(i); }
  virtual CSpace* DisplacementSpace(int obstacle) const { 
    switch(motionType[obstacle]) {
    case Fixed: return NULL;
    case Translation:  return new SphereDisplacementSpace(2,1.0);
    case TranslationX: 
      {
	Vector bmin(2,Vector2(-1.0,0)),bmax(2,Vector2(1,0));
	return new BoxDisplacementSpace(bmin,bmax);
      }
    case TranslationY:
      {
	Vector bmin(2,Vector2(0,-1.0)),bmax(2,Vector2(0,1));
	return new BoxDisplacementSpace(bmin,bmax);
      }
    case Rotation:
      return new RotationDisplacementSpace();
    default:
      return NULL;
    }
  }
  virtual bool IsFeasible(const Config& q,int obstacle,const Vector& d) {
    if(motionType[obstacle]==Fixed) return space->IsFeasible(q,obstacle); 
    if(motionType[obstacle]==Rotation) {
      GeometricPrimitive2D geom = space->obstacles.Obstacle(obstacle-1);
      AABB2D bb;
      geom.ToBound(bb);
      Vector2 c=(bb.bmin+bb.bmax)*0.5;
      RigidTransform2D T;
      T.setIdentity();
      T.R.setRotate(d(0));
      T.t = c-T.R*c;
      geom.Transform(T);

      RigidTransform2D TrobotInv;
      TrobotInv.t.set(-q(0),-q(1));
      TrobotInv.R.setIdentity();
      geom.Transform(TrobotInv);
      return !space->robot.Collides(geom);
    }
    else {
      Config temp=q-d;
      //cout<<"Domain: "<<space->domain.bmin<<", "<<space->domain.bmax<<endl;
      //cout<<"test: "<<q<<", "<<obstacle<<", "<<d<<", res "<<space->IsFeasible(temp,obstacle)<<endl;
      return space->IsFeasible(temp,obstacle);
    }
  }
  virtual bool IsFeasibleAll(const Config& q,int obstacle) { return false; }
  virtual bool IsVisibleAll(const Config& a,const Config& b,int obstacle) { return false; }
  virtual EdgePlanner* LocalPlanner(const Config& a,const Config& b) {
    return new ExplicitEdgePlanner(this,a,b);
  }
  virtual EdgePlanner* LocalPlanner(const Config& a,const Config& b,int obstacle) {
    if(motionType[obstacle]==Fixed) return space->LocalPlanner(a,b,obstacle);
    if(motionType[obstacle]==Rotation) {
      SingleObstacleCSpace* ospace = new SingleObstacleCSpace(this,obstacle);
      return new EdgePlannerWithCSpaceContainer(ospace,new BisectionEpsilonEdgePlanner(ospace,a,b,space->visibilityEpsilon));
    }
    else {
      Config tempa,tempb;
      tempa = a-obstacleDisplacements[obstacle];
      tempb = b-obstacleDisplacements[obstacle];
      return space->LocalPlanner(tempa,tempb,obstacle);
    }
  }
};

class RigidDisplacementCSpace : public ObstacleDisplacementCSpace
{
public:
  RigidRobot2DCSpace* space;
  enum MotionType { Fixed, Translation, TranslationX, TranslationY, Rotation };
  vector<MotionType> motionType;

  RigidDisplacementCSpace(RigidRobot2DCSpace* _space)
    :space(_space),motionType(_space->NumObstacles(),Translation)
  {
    //boundaries
    for(int i=0;i<1;i++) motionType[i]=Fixed;
  }
  virtual void Sample(Config& q) { space->Sample(q); }
  virtual int NumObstacles() { return space->NumObstacles();}
  virtual string ObstacleName(int i) { return space->ObstacleName(i); }
  virtual CSpace* DisplacementSpace(int obstacle) const { 
    if(motionType[obstacle]==Fixed) return NULL;
    return new SphereDisplacementSpace(2,1.0);
  }
  virtual bool IsFeasible(const Config& q,int obstacle,const Vector& d) {
    if(motionType[obstacle]==Fixed) return space->IsFeasible(q,obstacle); 
    else if(motionType[obstacle]==Rotation) {
      GeometricPrimitive2D geom = space->obstacles.Obstacle(obstacle-1);
      //rotate about geom center
      AABB2D bb;
      geom.ToBound(bb);
      Vector2 c=(bb.bmin+bb.bmax)*0.5;
      RigidTransform2D T;
      T.setIdentity();
      T.R.setRotate(d(0));
      T.t = c-T.R*c;
      geom.Transform(T);

      RigidTransform2D Trobot,TrobotInv;
      Trobot.t.set(q(0),q(1));
      Trobot.R.setRotate(q(2));
      TrobotInv.setInverse(Trobot);
      geom.Transform(TrobotInv);

      return !space->robot.Collides(geom);
    }
    else {
      //perturb obstacle
      Vector2 d2(d(0),d(1));
      GeometricPrimitive2D geom=space->obstacles.Obstacle(obstacle-1);
      RigidTransform2D T;
      T.setIdentity();
      T.t = d2;
      geom.Transform(T);
      
      RigidTransform2D Trobot,TrobotInv;
      Trobot.t.set(q(0),q(1));
      Trobot.R.setRotate(q(2));
      TrobotInv.setInverse(Trobot);
      geom.Transform(TrobotInv);
      
      return !space->robot.Collides(geom);
    }
  }
  virtual bool IsFeasibleAll(const Config& q,int obstacle) { return false; }
  virtual bool IsVisibleAll(const Config& a,const Config& b,int obstacle) { return false; }
  virtual EdgePlanner* LocalPlanner(const Config& a,const Config& b) {
    return new ExplicitEdgePlanner(this,a,b);
  }
  virtual EdgePlanner* LocalPlanner(const Config& a,const Config& b,int obstacle) {
    if(motionType[obstacle]==Fixed) return space->LocalPlanner(a,b,obstacle);
    SingleObstacleCSpace* ospace = new SingleObstacleCSpace(this,obstacle);
    return new EdgePlannerWithCSpaceContainer(ospace,new BisectionEpsilonEdgePlanner(ospace,a,b,space->visibilityEpsilon));
  }
  virtual void Interpolate(const Config& a,const Config& b,Real u,Config& out) {
    space->Interpolate(a,b,u,out);
  }
  virtual void Midpoint(const Config& a,const Config& b,Config& out) {
    space->Midpoint(a,b,out);
  }
  virtual Real Distance(const Config& a,const Config& b) {
    return space->Distance(a,b);
  }
};



//assumes the planner has been initialized
//will sample displacements displacements numDisplacementSamples times
//uniformly throughout the maxIters roadmap samples.
bool PlanDisplacement(DisplacementPlanner& planner,int maxIters,int numDisplacementSamples)
{
  return false;
  /*
  Timer timer;
  vector<Real> limits(numDisplacementSamples+1);
  vector<int> iters(numDisplacementSamples+1);
  for(int i=0;i<=numDisplacementSamples;i++) {
    limits[i] = Real(i)/Real(numDisplacementSamples);
    iters[i] = ((maxIters*(i+1))/numDisplacementSamples) - ((maxIters*i)/numDisplacementSamples);
  }
  vector<int> path;
  vector<int> assignment;
  bool res=planner.Plan(limits,iters,path,assignment);
  if(res) {
    Real cost = planner.Cost(assignment);
    printf("Planning succeeded in time %g, path with %d nodes length %g, cost %d\n",timer.ElapsedTime(),path.size(),planner.OptimalPathTo(1)->pathLength,cost);
  }
  else {
    printf("Planning failed in time %g\n",timer.ElapsedTime());
  }
  printf("  %d nodes, %d edges\n",planner.roadmap.NumNodes(),planner.roadmap.NumEdges());
  printf("Time NN %g, refine %g, explore %g, update in %g, update out %g, overhead %g\n",planner.timeNearestNeighbors,planner.timeRefine,planner.timeExplore,planner.timeUpdateCoversIn,planner.timeUpdateCoversOut,planner.timeOverhead);
  printf("Update paths calls: %d, iterations %d\n",planner.numUpdateCovers,planner.numUpdateCoversIterations);
  return res;
  */
}
