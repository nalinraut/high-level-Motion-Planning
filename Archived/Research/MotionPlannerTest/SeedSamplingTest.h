#include <KrisLibrary/math/matrix.h>
#include <KrisLibrary/math/SVDecomposition.h>
#include <KrisLibrary/planning/MotionPlanner.h>
#include "MotionPlannerProgram.h"
#include <KrisLibrary/utils/StatCollector.h>
#include <fstream>
#include <sstream>
using namespace std;

/*
struct BetaOfBetasLikelihoodFunction : public ScalarFieldFunction
{
  BetaOfBetasLikelihoodFunction(const vector<Real>& _posCounts,const vector<Real>& _negCounts,const vector<Real>& _totalCounts)
    :posCounts(_posCounts),negCounts(_negCounts),totalCounts(_totalCounts)
  {}
  virtual Real Eval(const Vector& x) {
    Real sum=0;
    Real cnt=1;
    for(size_t i=1;i<posCounts.size();i++,cnt+=1.0) 
      sum += posCounts[i]*Log(x(0)+cnt);
    cnt=1;
    for(size_t i=1;i<negCounts.size();i++,cnt+=1.0) 
      sum += negCounts[i]*Log(x(1)+cnt);    
    cnt=1;
    for(size_t i=1;i<totalCounts.size();i++,cnt+=1.0) 
      sum -= totalCounts[i]*Log(x(0)+x(1)+cnt);    
    return sum;
  }
  virtual void Gradient(const Vector& x,Vector& grad)
  {
    grad.resize(2);
    grad(0)=grad(1)=0;
    Real cnt=1;
    for(size_t i=1;i<posCounts.size();i++,cnt+=1.0) 
      grad(0) += posCounts[i]/(x(0)+cnt);
    cnt=1;
    for(size_t i=1;i<negCounts.size();i++,cnt+=1.0) 
      grad(1) += negCounts[i]/(x(1)+cnt);
    cnt=1;
    for(size_t i=1;i<totalCounts.size();i++,cnt+=1.0) {
      grad(0) -= totalCounts[i]/(x(0)+x(1)+cnt);    
      grad(1) -= totalCounts[i]/(x(0)+x(1)+cnt);
    }
  }

  const vector<Real>& posCounts;
  const vector<Real>& negCounts;
  const vector<Real>& totalCounts;
};

pair<Real,Real> MaxBetaOfBetas(const vector<pair<int,int> >& itemCounts)
{
  vector<Real> posCounts,negCounts,totalCounts;
  
  for(size_t i=0;i<itemCounts.size();i++) {
    if(itemCounts[i].first >= (int)posCounts.size())
      posCounts.resize(itemCounts[i].first+1,0);
    if(itemCounts[i].second >= (int)negCounts.size())
      negCounts.resize(itemCounts[i].second+1,0);
    if(itemCounts[i].first+itemCounts[i].second >= (int)totalCounts.size())
      totalCounts.resize(itemCounts[i].first+itemCounts[i].second+1,0);
    posCounts[itemCounts[i].first]+=1.0;
    negCounts[itemCounts[i].second]+=1.0;
    totalCounts[itemCounts[i].first+itemCounts[i].second]+=1.0;
  }
  //accumulate the total under a certain count
  for(int i=(int)posCounts.size()-1;i>0;i--)
    posCounts[i-1] += posCounts[i];
  for(int i=(int)negCounts.size()-1;i>0;i--)
    negCounts[i-1] += negCounts[i];
  for(int i=(int)totalCounts.size()-1;i>0;i--)
    totalCounts[i-1] += totalCounts[i];
  
  Vector x(2,1.0);
  Vector grad;
  //max sum[k=posCounts[i] for all i] of k log (x1+i)
  //  + sum[k=negCounts[i] for all i] of k log (x2+i)
  //  - sum[k=totalCounts[i] for all i] of k log (x1+x2+i)
  BetaOfBetasLikelihoodFunction f(posCounts,negCounts,totalCounts);
  f.Gradient(x,grad);
  if(grad(0) <= 0.0 && grad(1) <= 0.0) return pair<Real,Real>(1.0,1.0);
  MinimizationProblem solver(f);
  //TODO
}
*/

struct SeedSamplingProgram : public MotionPlannerProgram
{
  vector<Real> distanceBuckets;
  vector<StatCollector> distanceCounts;
  vector<vector<StatCollector> > distanceCountsByClass;

  vector<Config> points;
  vector<int> pointClasses;

  struct SampleResult
  {
    SampleResult(bool _res,int _sampleIters)
      :res(_res),sampleIters(_sampleIters),testIters(_sampleIters)
    {}
    SampleResult(bool _res,int _sampleIters,int _testIters)
      :res(_res),sampleIters(_sampleIters),testIters(_testIters)
    {}
    operator bool () const { return res; }
    bool operator !() const { return !res; }

    bool res;
    int sampleIters,testIters;
  };

  SeedSamplingProgram()
  {
  }

  virtual void Handle_Display() {
    SetupDisplay();
    DrawCSpace();

    const static float colors[10][3]={
      {0.25,0.5,1},
      {0.25,0.5,0},
      {0.25,0,1},
      {0.25,0,0},
      {0,0.5,1},
      {0,0.5,0},
      {0,0.5,1},
      {0,0,0},
      {0,0,0.5},
      {0.5,0,1},
    };
    glPointSize(3.0);
    glBegin(GL_POINTS);
    for(size_t i=0;i<points.size();i++) {
      int c = pointClasses[i]%10;
      glColor3fv(colors[c]);
      glVertex2d(points[i](0),points[i](1));
    }
    glEnd();
    glutSwapBuffers();
  }

  SampleResult SampleConfigInfeasible(Config& q,int maxIters=100)
  {
    for(int i=0;i<maxIters;i++) {
      cspace.Sample(q);
      if(!cspace.IsFeasible(q)) return SampleResult(true,i+1);
    }
    return SampleResult(false,maxIters);
  }

  SampleResult SampleUniform(int maxIters,Config& q)
  {
    for(int i=0;i<maxIters;i++) {
      cspace.Sample(q);
      if(cspace.IsFeasible(q)) return SampleResult(true,i+1);
    }
    return SampleResult(false,maxIters);
  }

  SampleResult SeedSampleConstant(const Config& seed,Real r,int maxIters,Config& q)
  {
    for(int i=0;i<maxIters;i++) {
      cspace.SampleNeighborhood(seed,r,q);
      if(cspace.IsFeasible(q)) return SampleResult(true,i+1);
    }
    return SampleResult(false,maxIters);
  }

  SampleResult SeedSampleLinear(const Config& seed,Real maxR,int maxIters,Config& q)
  {
    for(int i=0;i<maxIters;i++) {
      Real r = Real(i+1)/Real(maxIters)*maxR;
      cspace.SampleNeighborhood(seed,r,q);
      if(cspace.IsFeasible(q)) return SampleResult(true,i+1);
    }
    //return SampleResult(false,maxIters);
    SampleResult res=SeedSampleConstant(seed,maxR,100-maxIters,q);
    res.sampleIters += maxIters;
    res.testIters += maxIters;
    return res;
  }

  SampleResult SeedSampleHarmonic(const Config& seed,Real maxR,int maxIters,Config& q)
  {
    for(int i=0;i<maxIters;i++) {
      Real r = maxR/(maxIters-i-1);
      cspace.SampleNeighborhood(seed,r,q);
      if(cspace.IsFeasible(q)) return SampleResult(true,i+1);
    }
    SampleResult res=SeedSampleConstant(seed,maxR,100-maxIters,q);
    res.sampleIters += maxIters;
    res.testIters += maxIters;
    return res;
  }

  SampleResult SeedSampleGeometric(const Config& seed,Real maxR,int maxIters,int numSamplesPerRound,Config& q)
  {
    Assert(maxIters < 32);
    int divSize = 1 << maxIters;
    for(int i=0;i<maxIters;i++) {
      Real r = maxR / Real(divSize);
      for(int j=0;j<numSamplesPerRound;j++) {
	cspace.SampleNeighborhood(seed,r,q);
	if(cspace.IsFeasible(q)) SampleResult(true,numSamplesPerRound*i+j+1);
      }
      divSize >>= 1;
    }
    SampleResult res=SeedSampleConstant(seed,maxR,100-maxIters,q);
    res.sampleIters += maxIters;
    res.testIters += maxIters;
    return res;
  }

  Real PayoutFunction(Real d) const { return 0.5-d; }

  pair<Real,Real> PickOptimalSampleParams(const vector<StatCollector>& distanceCountsLocal,Real cSample,Real cTest,Real establishedPayout) const
  {
    vector<Real> candidateCutoffs(distanceBuckets.size()-1);
    for(size_t m=0;m+1<distanceBuckets.size();m++) {
      Real zi=PayoutFunction(0.5*(distanceBuckets[m]+distanceBuckets[m+1]));
      Real thetai = distanceCountsLocal[m].average();
      candidateCutoffs[m] = zi - cTest/thetai - establishedPayout;
    }
    static vector<Real> bucketVolume;
    if(bucketVolume.empty()) {
      bucketVolume.resize(distanceBuckets.size());
      //TODO: real bucket volume
      //for(size_t i=0;i<bucketVolume.size();i++) bucketVolume[i] = 1.0;
      for(size_t i=0;i<bucketVolume.size();i++) bucketVolume[i] = Sqr(distanceBuckets[i]);
      for(size_t i=1;i<bucketVolume.size();i++) bucketVolume[i] += bucketVolume[i-1];
    }
    size_t maxM = 0;
    Real maxCutoff = 0;
    Real maxu = 0;
    for(size_t j=0;j<candidateCutoffs.size();j++) {
      if(candidateCutoffs[j] < 0) continue;
      for(size_t m=0;m<distanceBuckets.size();m++) {
	Real sumIndicator=0,sumFeasible=0,sumZ=0;
	for(int i=0;i+1<m;i++) {
	  Real thetai=distanceCountsLocal[i].average();
	  Real zi=PayoutFunction(0.5*(distanceBuckets[m]+distanceBuckets[m+1]))-establishedPayout;
	  Real pi = bucketVolume[i+1]-bucketVolume[i]; //probability of sampling between r[i] and r[i+1].
	  if(zi*thetai-cTest > candidateCutoffs[j]*thetai) {
	    sumIndicator += pi;
	    sumFeasible += thetai*pi;
	    sumZ += thetai*zi*pi;
	  }
	}
	/*
	sumIndicator /= bucketVolume[m+1];
	sumFeasible /= bucketVolume[m+1];
	sumZ /= bucketVolume[m+1];;
	*/
	Real u = (sumZ -cSample*bucketVolume[m+1] - sumIndicator*cTest);///sumFeasible/bucketVolume[m+1];
	if(u > maxu*sumFeasible) {
	  maxu=u/sumFeasible;
	  maxM = m+1;
	  maxCutoff = candidateCutoffs[j];
	}
      }
    }
    if(maxCutoff == 0) return pair<Real,Real>(-1,-1);
    return pair<Real,Real>(distanceBuckets[maxM],maxCutoff+establishedPayout);
  }

  SampleResult SeedSampleOptimal(const Config& seed,Real cSample,Real cTest,Config& q)
  {
    LoadDistances();
    vector<StatCollector> distanceCountsLocal = distanceCounts;
    //vector<StatCollector> distanceCountsLocal(distanceCounts.size());
    //for(size_t i=0;i<distanceCounts.size();i++)
    //distanceCountsLocal[i].weightedCollect(distanceCounts[i].average(),distanceBuckets[i+1]*10.0);
    //ofstream out("optimal-samples.csv",ios::app);
    int numSamples=0,numTests=0;
    Real establishedPayout = 0;
    Real payoutRadius = 0;
    Config qtemp;
    for(int iters=0;iters<1000;iters++) {
      pair<Real,Real> choice = PickOptimalSampleParams(distanceCountsLocal,cSample,cTest,establishedPayout);
      Real r = choice.first;
      Real cutoff = choice.second;
      //printf("Iters %d optimal sampling params %g %g\n",iters,r,cutoff);
      if(r < 0) break;
      //out<<r<<",";
      cspace.SampleNeighborhood(seed,r,qtemp);
      numSamples++;

      Real d=cspace.Distance(seed,qtemp);
      Real z=PayoutFunction(d);
      vector<Real>::iterator i=std::lower_bound(distanceBuckets.begin(),distanceBuckets.end(),d);
      Assert(i != distanceBuckets.end());
      int index=i-distanceBuckets.begin();
      Real p=distanceCountsLocal[index].average();
      if(z - cTest/p > cutoff) {
	numTests++;
	if(cspace.IsFeasible(qtemp)) { //TODO: do we keep going?
	  distanceCountsLocal[index].collect(1.0);
	  Assert(z > establishedPayout);
	  establishedPayout = z;
	  payoutRadius = r;
	  q = qtemp;
	}
	else
	  distanceCountsLocal[index].collect(0.0);
      }
    }
    //out<<endl;
    //out.close();
    //ofstream out2("optimal-payouts.csv",ios::app);
    //out2<<payoutRadius<<","<<establishedPayout<<endl;
    //out2.close();
    return SampleResult((payoutRadius!=0),numSamples,numTests);
  }

  SampleResult SeedSampleByClass(const Config& seed,Real cSample,Real cTest,Config& q)
  {
    LoadDistances();
    //p(class i | data) = p(data | class i) p(class i)/p(data)
    vector<Real> pclass(distanceCountsByClass.size(),1.0/distanceCountsByClass.size());
    vector<StatCollector> distanceCountsLocal(distanceCounts.size());
    for(size_t i=0;i<distanceCounts.size();i++) {
      for(size_t c=0;c<pclass.size();c++)
	distanceCountsLocal[i].weightedCollect(distanceCountsByClass[c][i].average(),pclass[c]);
    }
    Real newDataWeight = 1.0;
    vector<int> numFeasible(distanceCounts.size(),0);
    vector<int> numInfeasible(distanceCounts.size(),0);
    //ofstream out("class-samples.csv",ios::app);
    int numSamples=0,numTests=0;
    Real establishedPayout = 0;
    Real payoutRadius = 0;
    Config qtemp;
    for(int iters=0;iters<1000;iters++) {
      pair<Real,Real> choice = PickOptimalSampleParams(distanceCountsLocal,cSample,cTest,establishedPayout);
      Real r = choice.first;
      Real cutoff = choice.second;
      //printf("Iters %d optimal sampling params %g %g\n",iters,r,cutoff);
      if(r < 0) break;
      //out<<r<<",";
      cspace.SampleNeighborhood(seed,r,qtemp);
      numSamples++;

      Real d=cspace.Distance(seed,qtemp);
      Real z=PayoutFunction(d);
      vector<Real>::iterator i=std::lower_bound(distanceBuckets.begin(),distanceBuckets.end(),d);
      Assert(i != distanceBuckets.end());
      int index=i-distanceBuckets.begin();
      Real p=distanceCountsLocal[index].average();
      if(z - cTest/p > cutoff) {
	numTests++;
	if(cspace.IsFeasible(qtemp)) { //TODO: do we keep going?
	  Assert(z > establishedPayout);
	  establishedPayout = z;
	  payoutRadius = r;
	  q = qtemp;
	  numFeasible[index]++;
	  for(size_t c=0;c<pclass.size();c++)
	    pclass[c] = distanceCountsByClass[c][index].average()*pclass[c];
	}
	else {
	  numInfeasible[index]++;
	  for(size_t c=0;c<pclass.size();c++)
	    pclass[c] = (1.0-distanceCountsByClass[c][index].average())*pclass[c];
	}
	//revise estimates from class weights
	Real sumWeights = 0;
	for(size_t c=0;c<pclass.size();c++)
	  sumWeights += pclass[c];
	for(size_t c=0;c<pclass.size();c++)
	  pclass[c] /= sumWeights;
	for(size_t i=0;i<distanceCounts.size();i++) {
	  distanceCountsLocal[i].clear();
	  for(size_t c=0;c<pclass.size();c++)
	    distanceCountsLocal[i].weightedCollect(distanceCountsByClass[c][i].average(),pclass[c]);
	  distanceCountsLocal[i].weightedCollect(1.0,numFeasible[i]*newDataWeight);
	  distanceCountsLocal[i].weightedCollect(0.0,numInfeasible[i]*newDataWeight);
	}
      }
    }
    //out<<endl;
    //out.close();
    //ofstream out2("class-payouts.csv",ios::app);
    //out2<<payoutRadius<<","<<establishedPayout<<endl;
    //out2.close();
    return SampleResult((payoutRadius!=0),numSamples,numTests);
  }

  void TestDistances(int maxIters,int innerIters=1000)
  {
    const static int numBuckets = 50;
    if((int)distanceCounts.size() != numBuckets) {
      distanceCounts.resize(numBuckets);
      distanceBuckets.resize(numBuckets);
      for(int i=0;i<20;i++)
	distanceBuckets[i] = Real(i+1)/20*0.2;
      for(int i=20;i<numBuckets;i++)
	distanceBuckets[i] = 0.2 + Real(i-20+1)/(numBuckets-20)*0.8;
      distanceBuckets.back() = Inf;
    }
    points.resize(0);
    pointClasses.resize(0);
    vector<Vector> items;
    vector<Vector> itemCounts;
    vector<StatCollector> localCounts(distanceCounts.size());
    for(int iters=0;iters<maxIters;iters++) {
      Config q1,q2;
      if(!SampleConfigInfeasible(q1,1000)) continue;
      for(int i=0;i<numBuckets;i++)
	localCounts[i].clear();
      points.push_back(q1);
      for(int i2=0;i2<innerIters;i2++) {
	cspace.Sample(q2);
	//cspace.SampleNeighborhood(q1,0.3,q2);
	Real d=cspace.Distance(q1,q2);
	vector<Real>::iterator i=std::lower_bound(distanceBuckets.begin(),distanceBuckets.end(),d);
	Assert(i != distanceBuckets.end());
	int index=i-distanceBuckets.begin();      
	if(cspace.IsFeasible(q2)) {
	  localCounts[index].collect(1.0);
	  distanceCounts[index].collect(1.0);
	}
	else {
	  localCounts[index].collect(0.0);
	  distanceCounts[index].collect(0.0);
	}
      }
      //collect localCounts in a vector
      Vector item(localCounts.size());
      Vector itemCount(localCounts.size());
      for(size_t i=0;i<localCounts.size();i++) {
	if(localCounts[i].number() != 0.0)
	  item(i) = localCounts[i].average();
	else
	  item(i) = 0.5;
	itemCount(i) = localCounts[i].number();
      }
      items.push_back(item);
      itemCounts.push_back(itemCount);
    }

    cout<<"Running EM on point classes"<<endl;
    //analyze, do a EM algorithm to extract classes
    const static int numClasses = 8;
    vector<int> itemClass(items.size());
    vector<vector<StatCollector> > tempCounts(numClasses);
    for(int c=0;c<numClasses;c++)
      tempCounts[c].resize(numBuckets);

    //do PCA first
    Matrix A(items.size(),numBuckets);
    Vector mean(numBuckets,Zero);
    for(size_t i=0;i<items.size();i++)
      mean += items[i];
    mean.inplaceDiv(items.size());
    for(size_t i=0;i<items.size();i++) {
      Vector temp; temp.sub(items[i],mean);
      A.copyRow(i,temp);
    }
    SVDecomposition<Real> svd;
    if(!svd.set(A)) Abort();
    svd.sortSVs();
    Assert(svd.W(0) >= svd.W(1));
    //classify according to singular values
    for(size_t i=0;i<items.size();i++) {
      int c = 0;
      if(svd.U(i,0) > 0.0) c |= 0x1;
      if(svd.U(i,1) > 0.0) c |= 0x2;
      if(svd.U(i,2) > 0.0) c |= 0x4;
      itemClass[i] = c;
    }
    Vector temp;
    cout<<"Mean: "<<mean<<endl;
    svd.V.getColRef(0,temp);
    cout<<"Singular value "<<svd.W(0)<<": "<<temp<<endl;
    svd.V.getColRef(1,temp);
    cout<<"Singular value "<<svd.W(1)<<": "<<temp<<endl;
    svd.V.getColRef(2,temp);
    cout<<"Singular value "<<svd.W(2)<<": "<<temp<<endl;
    Real maxLikelihood=-Inf;
    int numOuterIters=1;
    int numInnerIters=50;
    vector<vector<Real> > logpr0(numClasses),logpr1(numClasses);
    for(int c=0;c<numClasses;c++) {
      logpr0[c].resize(numBuckets);
      logpr1[c].resize(numBuckets);
    }
    for(int outerIter=0;outerIter<numOuterIters;outerIter++) {
      cout<<"Outer iteration "<<outerIter<<endl;
      Real likelihood = -Inf;
      //initial classes
      if(outerIter != 0) {
	for(size_t i=0;i<items.size();i++)
	  itemClass[i] = RandInt(numClasses);
      }
      int slice = 0;
      for(int innerIter=0;innerIter<numInnerIters;innerIter++) {
	cout<<"   Inner iteration "<<innerIter;
	//expectation step
	for(int c=0;c<numClasses;c++) 
	  for(int k=0;k<numBuckets;k++) {
	    tempCounts[c][k].clear();
	    tempCounts[c][k].weightedCollect(0.5,2.0);  //prior
	  }
	for(size_t i=0;i<items.size();i++)
	  for(int k=0;k<numBuckets;k++) 
	    tempCounts[itemClass[i]][k].weightedCollect(items[i](k),itemCounts[i](k));
	//precompute log probabilities
	for(int c=0;c<numClasses;c++)
	  for(int k=0;k<numBuckets;k++) {
	    Real theta=tempCounts[c][k].average();
	    Assert(theta > 0 && theta < 1);
	    logpr1[c][k]=Log(theta);
	    logpr0[c][k]=Log(1.0-theta);
	  }
	//maximization step
	likelihood = 0;
	bool changed=false;
	for(size_t i=0;i<items.size();i++) {
	//for(size_t s=0;s<20;s++) {
	  //int i = (slice++)%items.size();
	  int bestClass = itemClass[i];
	  Real maxLikelihoodi = -Inf;
	  for(int c=0;c<numClasses;c++) {
	    Real likelihoodi=0;
	    for(int k=0;k<numBuckets;k++) {
	      Real logtheta=logpr1[c][k];
	      Real log_one_theta=logpr0[c][k];
	      Real logp=items[i](k)*itemCounts[i](k)*logtheta+(1.0-items[i](k))*itemCounts[i](k)*log_one_theta;
	      likelihoodi += logp;
	    }
	    if(likelihoodi > maxLikelihoodi) {
	      maxLikelihoodi = likelihoodi;
	      bestClass = c;
	    }
	  }
	  if(bestClass!=itemClass[i]) changed=true;
	  itemClass[i]=bestClass;
	  likelihood += maxLikelihoodi;
	}
	cout<<", likelihood "<<likelihood<<endl;
	if(!changed) {
	  cout<<"Convergence on inner iteration "<<innerIter<<endl;
	  break;
	}
      }
      //Real temp=likelihood;
      //likelihood eval
      likelihood = 0;
      for(size_t i=0;i<items.size();i++)
	for(int k=0;k<numBuckets;k++) {
	  Real logtheta=logpr1[itemClass[i]][k];
	  Real log_one_theta=logpr0[itemClass[i]][k];
	  Real logp=items[i](k)*itemCounts[i](k)*logtheta+(1.0-items[i](k))*itemCounts[i](k)*log_one_theta;
	  likelihood += logp;
	}
      //printf("Saved likelihood: %g, new likelihood %g\n",temp,likelihood);
      //Assert(likelihood == temp);
      if(likelihood > maxLikelihood) {
	maxLikelihood = likelihood;
	distanceCountsByClass = tempCounts;
      }
    }
    cout<<"Likelihood: "<<maxLikelihood<<endl;
    vector<int> classCounts(numClasses,0);
    for(size_t i=0;i<items.size();i++)
      classCounts[itemClass[i]]++;
    for(size_t i=0;i<classCounts.size();i++)
      cout<<"Class "<<i<<": "<<classCounts[i]<<endl;
    pointClasses = itemClass;
  }

  void TestPerturbSampling(int maxIters)
  {
    StatCollector uniformDist,uniformCost;
    StatCollector constantDist10,constantCost10;
    StatCollector constantDist100,constantCost100;
    StatCollector linearDist10,linearCost10;
    StatCollector linearDist100,linearCost100;
    StatCollector harmonicDist10,harmonicCost10;
    StatCollector harmonicDist100,harmonicCost100;
    StatCollector geometricDist10,geometricCost10;
    StatCollector geometricDist100,geometricCost100;
    StatCollector optimalDist,optimalSamples,optimalTests;
    StatCollector classDist,classSamples,classTests;
    Real maxR = 0.5;
    Real cs = 0.005, ct = 0.01;
    for(int iters=0;iters<maxIters;iters++) {
      cout<<"Iteration "<<iters<<endl;
      Config q1,q2;
      if(!SampleConfigInfeasible(q1)) continue;

      SampleResult res=SampleUniform(100,q2);
      if(!res) uniformCost.collect(100);
      else {
	Assert(cspace.IsFeasible(q2));
	uniformCost.collect(res.sampleIters);
	uniformDist.collect(cspace.Distance(q1,q2));
      }
      res=SeedSampleConstant(q1,maxR,10,q2);
      if(!res) constantCost10.collect(res.sampleIters);
      else {
	Assert(cspace.IsFeasible(q2));
	constantCost10.collect(res.sampleIters);
	constantDist10.collect(cspace.Distance(q1,q2));
      }
      res=SeedSampleConstant(q1,maxR,100,q2);
      if(!res) constantCost100.collect(res.sampleIters);
      else {
	Assert(cspace.IsFeasible(q2));
	constantCost100.collect(res.sampleIters);
	constantDist100.collect(cspace.Distance(q1,q2));
      }
      res=SeedSampleLinear(q1,maxR,10,q2);
      if(!res) linearCost10.collect(res.sampleIters);
      else {
	Assert(cspace.IsFeasible(q2));
	linearCost10.collect(res.sampleIters);
	linearDist10.collect(cspace.Distance(q1,q2));
      }
      res=SeedSampleLinear(q1,maxR,100,q2);
      if(!res) linearCost100.collect(res.sampleIters);
      else {
	Assert(cspace.IsFeasible(q2));
	linearCost100.collect(res.sampleIters);
	linearDist100.collect(cspace.Distance(q1,q2));
      }
      res=SeedSampleHarmonic(q1,maxR,10,q2);
      if(!res) harmonicCost10.collect(res.sampleIters);
      else {
	Assert(cspace.IsFeasible(q2));
	harmonicCost10.collect(res.sampleIters);
	harmonicDist10.collect(cspace.Distance(q1,q2));
      }
      res=SeedSampleHarmonic(q1,maxR,100,q2);
      if(!res) harmonicCost100.collect(res.sampleIters);
      else {
	Assert(cspace.IsFeasible(q2));
	harmonicCost100.collect(res.sampleIters);
	harmonicDist100.collect(cspace.Distance(q1,q2));
      }
      res=SeedSampleGeometric(q1,maxR,10,1,q2);
      if(!res) geometricCost10.collect(res.sampleIters);
      else {
	Assert(cspace.IsFeasible(q2));
	geometricCost10.collect(res.sampleIters);
	geometricDist10.collect(cspace.Distance(q1,q2));
      }
      res=SeedSampleGeometric(q1,maxR,10,10,q2);
      if(!res) geometricCost100.collect(res.sampleIters);
      else {
	Assert(cspace.IsFeasible(q2));
	geometricCost100.collect(res.sampleIters);
	geometricDist100.collect(cspace.Distance(q1,q2));
      }
      res=SeedSampleOptimal(q1,cs,ct,q2);
      if(res) {
	Assert(cspace.IsFeasible(q2));
	optimalSamples.collect(res.sampleIters);
	optimalTests.collect(res.testIters);
	optimalDist.collect(cspace.Distance(q1,q2));
      }
      else optimalSamples.collect(res.sampleIters);
      res = SeedSampleByClass(q1,cs,ct,q2);
      if(res) {
	Assert(cspace.IsFeasible(q2));
	classSamples.collect(res.sampleIters);
	classTests.collect(res.testIters);
	classDist.collect(cspace.Distance(q1,q2));
      }
      else classSamples.collect(res.sampleIters);
    }
    cout<<"Seed sampling stats: "<<endl;
    cout<<"Uniform cost: "; uniformCost.Print(cout); cout<<endl;
    cout<<"Uniform dist: "; uniformDist.Print(cout); cout<<endl;
    cout<<"Constant(10) cost: "; constantCost10.Print(cout); cout<<endl;
    cout<<"Constant(10) dist: "; constantDist10.Print(cout); cout<<endl;
    cout<<"Constant(100) cost: "; constantCost100.Print(cout); cout<<endl;
    cout<<"Constant(100) dist: "; constantDist100.Print(cout); cout<<endl;
    cout<<"Linear(10) cost: "; linearCost10.Print(cout); cout<<endl;
    cout<<"Linear(10) dist: "; linearDist10.Print(cout); cout<<endl;
    cout<<"Linear(100) cost: "; linearCost100.Print(cout); cout<<endl;
    cout<<"Linear(100) dist: "; linearDist100.Print(cout); cout<<endl;
    cout<<"Harmonic(10) cost: "; harmonicCost10.Print(cout); cout<<endl;
    cout<<"Harmonic(10) dist: "; harmonicDist10.Print(cout); cout<<endl;
    cout<<"Harmonic(100) cost: "; harmonicCost100.Print(cout); cout<<endl;
    cout<<"Harmonic(100) dist: "; harmonicDist100.Print(cout); cout<<endl;
    cout<<"Geometric(10) cost: "; geometricCost10.Print(cout); cout<<endl;
    cout<<"Geometric(10) dist: "; geometricDist10.Print(cout); cout<<endl;
    cout<<"Geometric(100) cost: "; geometricCost100.Print(cout); cout<<endl;
    cout<<"Geometric(100) dist: "; geometricDist100.Print(cout); cout<<endl;
    cout<<"Optimal samples: "; optimalSamples.Print(cout); cout<<endl;
    cout<<"Optimal tests: "; optimalTests.Print(cout); cout<<endl;
    cout<<"Optimal dist: "; optimalDist.Print(cout); cout<<endl;
    cout<<"Class samples: "; classSamples.Print(cout); cout<<endl;
    cout<<"Class tests: "; classTests.Print(cout); cout<<endl;
    cout<<"Class dist: "; classDist.Print(cout); cout<<endl;
    cout<<endl;
    cout<<"Seed sampling utilities: "<<endl;
    cout<<"Uniform: "<<PayoutFunction(uniformDist.average())-(cs+ct)*uniformCost.average()<<", payout "<<PayoutFunction(uniformDist.average())<<", cost "<<(cs+ct)*uniformCost.average()<<endl;
    cout<<"Constant(10): "<<PayoutFunction(constantDist10.average())-(cs+ct)*constantCost10.average()<<", payout "<<PayoutFunction(constantDist10.average())<<", cost "<<(cs+ct)*constantCost10.average()<<endl;
    cout<<"Constant(100): "<<PayoutFunction(constantDist10.average())-(cs+ct)*constantCost100.average()<<", payout "<<PayoutFunction(constantDist100.average())<<", cost "<<(cs+ct)*constantCost100.average()<<endl;
    cout<<"Linear(10): "<<PayoutFunction(linearDist10.average())-(cs+ct)*linearCost10.average()<<", payout "<<PayoutFunction(linearDist10.average())<<", cost "<<(cs+ct)*linearCost10.average()<<endl;
    cout<<"Linear(100): "<<PayoutFunction(linearDist10.average())-(cs+ct)*linearCost100.average()<<", payout "<<PayoutFunction(linearDist100.average())<<", cost "<<(cs+ct)*linearCost100.average()<<endl;
    cout<<"Harmonic(10): "<<PayoutFunction(harmonicDist10.average())-(cs+ct)*harmonicCost10.average()<<", payout "<<PayoutFunction(harmonicDist10.average())<<", cost "<<(cs+ct)*harmonicCost10.average()<<endl;
    cout<<"Harmonic(100): "<<PayoutFunction(harmonicDist10.average())-(cs+ct)*harmonicCost100.average()<<", payout "<<PayoutFunction(harmonicDist100.average())<<", cost "<<(cs+ct)*harmonicCost100.average()<<endl;
    cout<<"Geometric(10): "<<PayoutFunction(geometricDist10.average())-(cs+ct)*geometricCost10.average()<<", payout "<<PayoutFunction(geometricDist10.average())<<", cost "<<(cs+ct)*geometricCost10.average()<<endl;
    cout<<"Geometric(100): "<<PayoutFunction(geometricDist10.average())-(cs+ct)*geometricCost100.average()<<", payout "<<PayoutFunction(geometricDist100.average())<<", cost "<<(cs+ct)*geometricCost100.average()<<endl;
    cout<<"Optimal: "<<PayoutFunction(optimalDist.average())-cs*optimalSamples.average()-ct*optimalTests.average()<<", payout "<<PayoutFunction(optimalDist.average())<<", cost "<<cs*optimalSamples.average()+ct*optimalTests.average()<<endl;
    cout<<"Class: "<<PayoutFunction(classDist.average())-cs*classSamples.average()-ct*classTests.average()<<", payout "<<PayoutFunction(classDist.average())<<", cost "<<cs*classSamples.average()+ct*classTests.average()<<endl;
  }

  virtual void Handle_Keypress(unsigned char key,int x,int y)
  {
    if(key == 't') {
      TestPerturbSampling(1000);
      Refresh();
    }
    else if(key == ' ') {
      printf("Testing distances...\n");
      TestDistances(10000);

      ofstream out("infeasible_distances.txt");
      for(int i=0;i<distanceCounts.size();i++) {
	out<<distanceBuckets[i]<<" "<<distanceCounts[i].average()<<endl;
      }
      out.close();
      {
	ofstream out("infeasible_distances_by_class.txt");
	for(int i=0;i<distanceCounts.size();i++) {
	  out<<distanceBuckets[i]<<" ";
	  for(size_t j=0;j<distanceCountsByClass.size();j++) {
	    if(distanceCountsByClass[j][i].number()==0)
	      out<<"0.5 ";
	    else
	      out<<distanceCountsByClass[j][i].average()<<" ";
	  }
	  out<<endl;
	}
	out.close();
      }
      printf("Done.\n");
      Refresh();
    }
  }

  virtual void Handle_Click(int button,int state,int x,int y) {
    if(state == GLUT_DOWN) {
      Refresh();
    }
  }

  bool LoadDistances()
  {
    if(!distanceBuckets.empty()) return true;
    ifstream in("infeasible_distances.txt");
    if(in) printf("Reading in distances from infeasible_distances.txt...\n");
    else return false;
    while(in) {
      char buf[256];
      in.getline(buf,256);
      if(!in) break;
      if(buf[0] == '#') //comment char
	continue;
      stringstream ss;
      ss.str(buf);
      Real bucket,val;
      ss>>bucket;
      if(!ss) {
	//probably the last one (inf)
	distanceBuckets.push_back(Inf);
	ss.clear(ios::goodbit);
	ss.ignore(100,' ');
      }
      else {
	distanceBuckets.push_back(bucket);
      }
      ss>>val;
      distanceCounts.resize(distanceCounts.size()+1);
      distanceCounts.back().collect(val);
    }
    {
      ifstream in("infeasible_distances_by_class.txt");
      if(in) printf("Reading in distances from infeasible_distances_by_class.txt...\n");
      else return false;
      int line=0;
      while(in) {
	char buf[1024];
	in.getline(buf,1024);
	if(!in) break;
	if(buf[0] == '#') //comment char
	  continue;
	stringstream ss;
	ss.str(buf);
	Real bucket;
	ss>>bucket;
	if(!ss) {
	  //probably the last one (inf)
	  Assert(IsInf(distanceBuckets[line]));
	  ss.clear(ios::goodbit);
	  ss.ignore(5,' ');
	}
	else {
	  Assert(distanceBuckets[line] == bucket);
	}
	vector<Real> vals;
	while(ss) {
	  Real val;
	  ss>>val;
	  if(ss) vals.push_back(val);
	}
	if(distanceCountsByClass.empty()) {
	  distanceCountsByClass.resize(vals.size());
	}
	else if(vals.size() != distanceCountsByClass.size()) {
	  cerr<<"Unable to read all values for line "<<line<<", only read "<<vals.size()<<" not "<<distanceCountsByClass.size()<<endl;
	  return false;
	}
	for(size_t i=0;i<vals.size();i++) {
	  distanceCountsByClass[i].resize(distanceCountsByClass[i].size()+1);
	  distanceCountsByClass[i].back().collect(vals[i]);
	}
	line++;
      }
    }
    return true;
  }

  inline Real DistanceProbability(Real d)
  {
    if(distanceBuckets.empty())
      return Exp(-Pow(d,2.0)*3.0);

    vector<Real>::iterator i=std::lower_bound(distanceBuckets.begin(),distanceBuckets.end(),d);
    Assert(i != distanceBuckets.end());
    int index=i-distanceBuckets.begin();
    return distanceCounts[index].average();
  }
};
