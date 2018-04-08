

struct EmpiricalHistogram
{
  void InitUniform(Real a,Real b,int numBuckets)
  {
    values.resize(numBuckets);
    buckets.resize(numBuckets);
    for(int i=0;i<numBuckets;i++)
      buckets[i] = a + (b-a)*Real(i+1)/numBuckets;
    buckets.back() = Inf;
  }
  void Collect(Real x,Real y) 
  {
    Value(x).collect(y);
  }
  StatCollector& Value(Real x)
  {
    vector<Real>::iterator i=std::lower_bound(buckets.begin(),buckets.end(),x);
    Assert(i != distanceBuckets.end());
    int index=i-distanceBuckets.begin();
    return values[index];
  }

  vector<Real> buckets;
  vector<StatCollector> values;
};

class CSpaceSegmentFeasibility
{
 public:
  EmpiricalHistogram feasibilityByDistance;

  CSpaceSegmentFeasibility() {}
  void InitBuckets(Real maxDist,int numBuckets) { feasibilityByDistance.InitUniform(0,maxDist,numBuckets); }
  void InitPrior(Real prior,Real priorStrength) 
  {
    for(size_t i=0;i<feasibilityByDistance.values.size();i++) {
      feasibilityByDistance.values[i].clear();
      feasibilityByDistance.values[i].weightedCollect(prior,priorStrength);
    }
  }

  void Learn(CSpace& cspace,int maxIters)
  {
    Config q1,q2;
    for(int iters=0;iters<maxIters;iters++) {
      cspace.Sample(q1);
      if(!cspace.IsFeasible(q1)) continue;
      cspace.Sample(q2);
      if(!cspace.IsFeasible(q2)) continue;
      Real d=cspace.Distance(q1,q2);
      EdgePlanner* e=IsVisible(&cspace,q1,q2);
      if(e) {
	feasibilityByDistance.Collect(d,1.0);
	delete e;
      }
      else feasibilityByDistance.Collect(d,0.0);
    }
  }

  Real Predict(Real distance) {
    return Value(distance).average();
  }
};
