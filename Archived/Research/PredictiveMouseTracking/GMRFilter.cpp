#include "GMRFilter.h"
#include <math/LDL.h>
#include <math/indexing.h>
#include <math/sample.h>
#include <string.h>


bool IsFinite(const Matrix& m)
{
  for(int i=0;i<m.m;i++)
    for(int j=0;j<m.n;j++)
      if(!IsFinite(m(i,j))) return false;
  return true;
}


Real Normalize(vector<Real>& w)
{
  Real sumw = 0.0;
  for(size_t i=0;i<w.size();i++) sumw += w[i];
  if(sumw == 0.0) {
    fill(w.begin(),w.end(),1.0/w.size());
    return sumw;
  }
  else if(IsInf(sumw)) {
    assert(IsInf(sumw)==1);
    int numinf = 0;
    for(size_t i=0;i<w.size();i++)
      if(IsInf(sumw)==1) numinf ++;
    for(size_t i=0;i<w.size();i++)
      if(IsInf(sumw)==1) w[i] = 1.0/Real(numinf);
      else w[i] = 0.0;
    return sumw;
  }
  else {
    for(size_t i=0;i<w.size();i++) w[i]/=sumw;
    return sumw;
  }
}

void GetLinearFit(const GaussianMixtureRegression& gmr,Matrix& A,Vector& b,Matrix& err)
{
  /*
  //totally hacky way to fit things...
  Matrix Ai;
  Vector bi;
  gmr.regressions[0].GetLinearEquation(Ai,bi);
  A.mul(Ai,gmr.joint.phi[0]);
  err.mul(gmr.regressions[0].ycov,gmr.joint.phi[0]);
  b.mul(bi,gmr.joint.phi[0]);
  for(size_t i=1;i<gmr.joint.phi.size();i++) {
    gmr.regressions[i].GetLinearEquation(Ai,bi);
    A.madd(Ai,gmr.joint.phi[i]);
    b.madd(bi,gmr.joint.phi[i]);
    err.madd(gmr.regressions[i].ycov,gmr.joint.phi[i]);
  }
  //TODO: error should also include mean estimation errors, distribution of x
  */
  Vector mean;
  Matrix cov;
  gmr.joint.GetMean(mean);
  gmr.joint.GetCovariance(cov);
  GaussianRegression reg;
  reg.Set(mean,cov,gmr.xindices,gmr.yindices);
  reg.GetLinearEquation(A,b);
  reg.GetNoiseCovariance(err);
}


namespace Statistics {
//defined in statistics/GaussianMixtureModel.cpp
Real LogDotProduct(const Vector& mu1,const Matrix& K1,const Vector& mu2,const Matrix& K2);
} //namespace Statistics

void RunEM(GaussianMixtureModelRaw& gmm,int numComponents,int maxIters)
{
  GaussianMixtureModelRaw orig = gmm;
  //initialize EM centers
  gmm.Resample(numComponents);
  numComponents = (int)gmm.phi.size();
  //probability that i'th original gaussian belongs to j'th new gaussian
  vector<vector<Real> > p(orig.means.size());
  for(size_t i=0;i<p.size();i++)
    p[i].resize(numComponents);
  
  Vector mean(gmm.means[0].n),diff;
  Matrix cov(gmm.means[0].n,gmm.means[0].n);
  
  for(int iters=0;iters<maxIters;iters++) {
    //printf("EM step %d\n",iters);
    for(size_t i=0;i<p.size();i++) {
      for(int j=0;j<numComponents;j++) 
	p[i][j] = LogDotProduct(orig.means[i],orig.covariances[i],gmm.means[j],gmm.covariances[j])+Log(gmm.phi[j]);
      Real maxp = -Inf;
      for(int j=0;j<numComponents;j++) 
	maxp = Max(maxp,p[i][j]);
      if(!IsFinite(maxp)) {
	fill(p[i].begin(),p[i].end(),0.0);
	continue;
      }
      Assert(IsFinite(maxp));
      for(int j=0;j<numComponents;j++) 
	p[i][j] = Exp(p[i][j]-maxp);
      Normalize(p[i]);
    }
    
    //M step: assign gaussians from mean/stddev 
    for(int j=0;j<numComponents;j++) {
      if(gmm.phi[j] == 0.0) continue;
      //compute weight and mean
      Real sumw = 0.0;
      mean.setZero();
      for(size_t i=0;i<p.size();i++) {
	if(p[i][j]==0) continue;
	sumw += p[i][j];
	mean.madd(orig.means[i],p[i][j]);
      }
      gmm.phi[j] = sumw;
      if(sumw == 0.0) {
	printf("Gaussian %d dropped out\n",j);
	continue;
      }
      mean /= sumw;
      gmm.means[j] = mean;
      //compute covariance
      cov.setZero();
      for(size_t i=0;i<p.size();i++) { 
	if(p[i][j] == 0) continue;
	cov.madd(orig.covariances[i],p[i][j]);
	diff.sub(orig.means[i],mean);
	for(int m=0;m<cov.m;m++)
	  for(int n=0;n<cov.n;n++)
	    cov(m,n) += diff[m]*diff[n]*p[i][j];
      }
      cov *= 1.0/sumw;
      gmm.covariances[j] = cov;
      if(!IsFinite(cov)) {
	printf("After refitting, covariance %d became non finite\n",j);
	getchar();
      }
    }
    //adjust gmm phi
    Normalize(gmm.phi);

    //compute log likelihood to determine convergence?
  }  
}

//strategy can be "em", "cluster_kl", or "cluster_emd"
bool Refit(GaussianMixtureModelRaw& gmm,int numComponents,const char* strategy="cluster_kl")
{
  if((int)gmm.phi.size() <= numComponents) return true;

  if(0==strcmp(strategy,"em")) {
    RunEM(gmm,numComponents,10);
  }
  else {
    GaussianMixtureModel copy;
    bool res=gmm.Get(copy);
    if(!res) {
      printf("Warning: failed to set GMM decompositions properly\n");
      gmm.Resample(numComponents);
      return false;
    }
    gmm.Cluster(copy,numComponents);
  }
  return true;
}


//For the observation model obs = Ax+b+e, with e~N(0,ocov)
//with prior distribution x ~ N(xmean,xcov)
//updates xmean',xcov' to the posterior distribution over x, given obs.
//Returns the log probability of making the given observation P(obs) given 
//the prior distribution of x
Real KalmanUpdate(Vector& xmean,Matrix& xcov,const Vector& obs,const Matrix& A,const Vector& b,const Matrix& ocov)
{
  Matrix B,C,Cinv,mtemp,mtemp2;
  B.mul(A,xcov);
  C.mulTransposeB(B,A);
  C += ocov;
  LDLDecomposition<Real> ldl;
  ldl.verbose = 0;
  ldl.set(C);

  
  //obs ~ N(A xmean + b, A xcov A^T + ocov) = N(expectedObs,C)
  //P(obs+expectedObs) = (2pi)^(-d/2) det(C)^(-1/2) exp(-1/2 obs^T C^-1 obs)
  Vector D;
  ldl.getD(D);
  bool printres = false;
  Real logdet = 0.0;
  int d = 0;
  for(int i=0;i<D.n;i++) {
    if(!IsFinite(D(i))) {
      printf("Kalman update: observation matrix became non-finite\n");
      //cout<<ocov<<endl;
      //cout<<xcov<<endl;
      //cout<<C<<endl;
      return -Inf;
    }
    else if(D(i) < -Epsilon) {
      printf("Kalman update: observation matrix not positive definite %d=%g\n",i,D(i));
      LDLDecomposition<Real> ldl2; 
      ldl2.set(xcov);
      cout<<xcov<<endl;
      cout<<ldl2.LDL<<endl;
      getchar();
      //cout<<ocov<<endl;
      //cout<<xcov<<endl;
      return -Inf;
    }
    else if(D(i) < 0) {
      printres = true;
      ldl.LDL(i,i) = 0.0;
      D(i) = 0;
    }
    else if(D(i) < 1e-8) {
      //printf("Kalman update: warning, LDL becoming badly conditioned %d=%g\n",i,D(i));
      logdet += Log(D(i));
      d++;
    }
    else {
      logdet += Log(D(i));
      d++;
    }
  }
  Real loginvc = 0.5*logdet + 0.5*Real(d)*Log(2.0*Pi);

  Vector expectedObs,innovation,vtemp1,vtemp2;
  A.mul(xmean,expectedObs);
  expectedObs += b;
  innovation.sub(obs,expectedObs);

  //cout<<"Predict obs mean"<<expectedObs<<endl;
  //cout<<"Actual obs: "<<obs<<endl;
  //cout<<"Predicted obs cov"<<endl<<C<<endl;

  Matrix K;
  ldl.getPseudoInverse(Cinv);
  K.mulTransposeA(B,Cinv);

  K.mul(innovation,vtemp2);
  xmean += vtemp2;

  mtemp.mul(K,B);
  xcov -= mtemp;

  if(!IsFinite(xcov)) {
    printf("KalmanUpdate: Covariance became infinite\n");
    cout<<mtemp2<<endl;
    cout<<C<<endl;
    cout<<Cinv<<endl;
    getchar();
    return -Inf;
  }

  Cinv.mul(innovation,vtemp1);
  if(printres) {
    printf("KalmanUpdate: Result is %g\n",-0.5*vtemp1.dot(innovation)-loginvc);
    //printf(" first term %g, second term %g, logdet %g, log(2pi)^d %g\n",-0.5*vtemp1.dot(innovation),-loginvc,logdet,0.5*Real(d)*Log(2.0*Pi));
    if(IsNaN(-0.5*vtemp1.dot(innovation)-loginvc)) return -Inf;
  }
  return -0.5*vtemp1.dot(innovation)-loginvc;
}

void AugmentY(GaussianRegression& g,int n)
{
  Vector ymean2(g.ymean.n+n,Zero);
  ymean2.copySubVector(0,g.ymean);
  Matrix ycov2(g.ymean.n+n,g.ymean.n+n,Zero);
  ycov2.copySubMatrix(0,0,g.ycov);
  Matrix yxcov2(g.ymean.n+n,g.yxcov.n,Zero);
  yxcov2.copySubMatrix(0,0,g.yxcov);
  swap(g.ymean,ymean2);
  swap(g.ycov,ycov2);
  swap(g.yxcov,yxcov2);
  Matrix A2(g.A.m+n,g.A.n,Zero);
  A2.copySubMatrix(0,0,g.A);
  swap(g.A,A2);
  Vector b2(g.b.n+n,Zero);
  b2.copySubVector(0,g.b);
  swap(g.b,b2);
}


GMRFilter::GMRFilter()
{
  samplingStrategy = CollapseNext;
  maxGaussians = 0;
}

void GMRFilter::Scale(const Vector& obsScale,const Vector& hiddenScale)
{
  Assert(Afull.isEmpty());
  Assert(Apred.empty());
  Matrix Aobs(obsgmm.gaussians[0].mu.n,obsgmm.gaussians[0].mu.n,0.0);
  for(size_t i=0;i<hiddenIndices.size();i++)
    Aobs(hiddenIndices[i],hiddenIndices[i]) = hiddenScale[i];
  for(size_t i=0;i<obsIndices.size();i++)
    Aobs(obsIndices[i],obsIndices[i]) = obsScale[i];
  for(size_t i=0;i<obsHistoryIndices.size();i++)
    for(size_t j=0;j<obsHistoryIndices.size();j++)
      Aobs(obsHistoryIndices[i][j],obsHistoryIndices[i][j]) = obsScale[j];
  Vector zero(Aobs.m,0.0);
  GaussianMixtureModel temp=obsgmm;
  obsgmm.SetLinearTransform(temp,Aobs,zero);


  Matrix Atrans(transgmm.gaussians[0].mu.n,transgmm.gaussians[0].mu.n,0.0);
  for(size_t i=0;i<currentIndices.size();i++)
    Atrans(currentIndices[i],currentIndices[i]) = hiddenScale[i];
  for(size_t i=0;i<nextIndices.size();i++)
    Atrans(nextIndices[i],nextIndices[i]) = hiddenScale[i];
  for(size_t i=0;i<transHistoryIndices.size();i++)
    for(size_t j=0;j<transHistoryIndices.size();j++)
      Atrans(transHistoryIndices[i][j],transHistoryIndices[i][j]) = obsScale[j];

  zero.resize(Atrans.m,0.0);
  temp=transgmm;
  transgmm.SetLinearTransform(temp,Atrans,zero);
  Init();
}

void GMRFilter::Init()
{
  //sanity check
  Assert(!obsgmm.gaussians.empty());
  vector<bool> used(obsgmm.gaussians[0].mu.n,false);
  for(size_t i=0;i<hiddenIndices.size();i++) {
    Assert(hiddenIndices[i] >= 0 && hiddenIndices[i] < (int)used.size());
    Assert(!used[hiddenIndices[i]]);
    used[hiddenIndices[i]]=true;
  }
  int k=-1;
  for(size_t i=0;i<obsIndices.size();i++) {
    Assert(obsIndices[i] >= 0 && obsIndices[i] < (int)used.size());
    Assert(!used[obsIndices[i]]);
    used[obsIndices[i]]=true;
    //must be in order
    if(k >= 0) Assert(obsIndices[i] == k+1);
    k = obsIndices[i];
  }
  k=-1;
  for(size_t i=0;i<obsHistoryIndices.size();i++) {
    Assert(obsHistoryIndices[i].size()==obsIndices.size());
    for(size_t j=0;j<obsHistoryIndices[i].size();j++) {
      Assert(obsHistoryIndices[i][j] >= 0 && obsHistoryIndices[i][j] < (int)used.size());
      Assert(!used[obsHistoryIndices[i][j]]);
      used[obsHistoryIndices[i][j]]=true;
      //must be in order
      if(k >= 0) Assert(obsHistoryIndices[i][j] == k+1);
      k = obsHistoryIndices[i][j];
    }
  }
  for(size_t i=0;i<used.size();i++)
    Assert(used[i]); 

  //build indexing lists
  vector<int> historyIndicesFlat;
  for(size_t i=0;i<obsHistoryIndices.size();i++) 
    for(size_t j=0;j<obsHistoryIndices[i].size();j++) 
      historyIndicesFlat.push_back(obsHistoryIndices[i][j]);
  vector<int> obsHistoryFlat=obsIndices;
  obsHistoryFlat.insert(obsHistoryFlat.end(),historyIndicesFlat.begin(),historyIndicesFlat.end());

  //printf("%d observation indices, %d history indices, %d hidden indices\n",obsIndices.size(),historyIndicesFlat.size(),hiddenIndices.size());

  //build observation GMR conditional on hidden variables, history
  gmr_obs.joint = obsgmm;
  vector<int> hiddenHistoryFlat=hiddenIndices;
  hiddenHistoryFlat.insert(hiddenHistoryFlat.end(),historyIndicesFlat.begin(),historyIndicesFlat.end());
  printf("Building observation model\n");
  gmr_obs.SetXIndices(hiddenHistoryFlat);
  printf("Done\n");

  //do we want to build a transition model?
  if(nextIndices.size() != 0) {
    Assert(!transgmm.gaussians.empty());
    Assert(currentIndices.size() == hiddenIndices.size());
    Assert(transHistoryIndices.size() == obsHistoryIndices.size());
    historyIndicesFlat.resize(0);
    for(size_t i=0;i<transHistoryIndices.size();i++) 
      for(size_t j=0;j<transHistoryIndices[i].size();j++) 
	historyIndicesFlat.push_back(transHistoryIndices[i][j]);
    Assert(int(currentIndices.size()+historyIndicesFlat.size()+nextIndices.size())==transgmm.gaussians[0].mu.n);
    hiddenHistoryFlat = currentIndices;
    hiddenHistoryFlat.insert(hiddenHistoryFlat.end(),historyIndicesFlat.begin(),historyIndicesFlat.end());

    gmr_trans.joint = transgmm;
    printf("Building transition model\n");
    gmr_trans.SetXIndices(hiddenHistoryFlat);
    Assert(gmr_trans.yindices == nextIndices);
  }

  Reset();
}

void GMRFilter::Reset()
{
  //marginalize out the observation variables to get initial belief
  int nhidden=(int)hiddenIndices.size();
  int nobs = (int)obsIndices.size();
  int nhist=(int)obsHistoryIndices.size()*nobs;
  vector<int> hiddenHistIndices = hiddenIndices;
  for(size_t i=0;i<obsHistoryIndices.size();i++)
    hiddenHistIndices.insert(hiddenHistIndices.end(),obsHistoryIndices[i].begin(),obsHistoryIndices[i].end());

  Vector temp;
  if(initgmm.phi.empty()) {
    hidden.phi = obsgmm.phi;
    hidden.means.resize(obsgmm.gaussians.size());
    hidden.covariances.resize(obsgmm.gaussians.size());
    hidden_hist.phi = hidden.phi;
    hidden_hist.means.resize(hidden.means.size());
    hidden_hist.covariances.resize(hidden.means.size());
    for(size_t i=0;i<obsgmm.gaussians.size();i++) {
      hidden.means[i].resize(nhidden);
      hidden.covariances[i].resize(nhidden,nhidden);
      hidden_hist.means[i].resize(nhidden+nhist);
      hidden_hist.covariances[i].resize(nhidden+nhist,nhidden+nhist,Zero);
      //get mean from obsgmm
      GetElements(obsgmm.gaussians[i].mu,hiddenIndices,hidden.means[i]);
      GetElements(obsgmm.gaussians[i].mu,hiddenHistIndices,hidden_hist.means[i]);
      //cout<<"hidden/history mean: "<<hidden_hist.means[i]<<endl;
      //TEMP: try zeroing out history
      hidden_hist.means[i].setZero();
      hidden_hist.means[i].copySubVector(0,hidden.means[i]);

      //get covariance from obsgmm
      Matrix cov;
      obsgmm.gaussians[i].getCovariance(cov);
      GetElements(cov,hiddenIndices,hiddenIndices,hidden.covariances[i]);
      //zero covariance on history
      hidden_hist.covariances[i].copySubMatrix(0,0,hidden.covariances[i]); 
    }
  }
  else {
    Assert(initgmm.gaussians[0].mu.n == nhidden);
    hidden.Set(initgmm);
    hidden_hist.phi = hidden.phi;
    hidden_hist.means.resize(hidden.means.size());
    hidden_hist.covariances.resize(hidden.means.size());
    for(size_t i=0;i<initgmm.gaussians.size();i++) {
      //zero covariance on mean and history
      hidden_hist.means[i].resize(nhidden+nhist,Zero);
      hidden_hist.covariances[i].resize(nhidden+nhist,nhidden+nhist,Zero);
      hidden_hist.means[i].copySubVector(0,hidden.means[i]);
      hidden_hist.covariances[i].copySubMatrix(0,0,hidden.covariances[i]); 
    }
  }
  if(samplingStrategy == Sample || samplingStrategy == Fit) 
    if((int)hidden.means.size() > maxGaussians) {
      if(samplingStrategy == Sample)
	hidden.Resample(maxGaussians);
      else {
	if(!Refit(hidden,maxGaussians)) {
	  printf("Warning: failed to refit initial distribution\n");
	}
      }
      HiddenToHiddenHist();
    }

  //initialize the history to the mean
  history.resize(obsHistoryIndices.size());
  Vector hhmean;
  hidden_hist.GetMean(hhmean);
  for(size_t h=0;h<history.size();h++) {
    history[h].resize(nobs);
    hhmean.getSubVectorCopy(nhidden + h*nobs,history[h]);
    for(size_t i=0;i<hidden_hist.means.size();i++) {
      hidden_hist.means[i].copySubVector(nhidden + h*nobs,history[h]);
    }
  }
  logObservationProbability = 0.0;

  //Assert(IsConsistent());
}

void GMRFilter::Propagate(const GaussianMixtureRegression& gmr,GaussianMixtureModelRaw& x,GaussianMixtureModelRaw& y) const
{
  if(samplingStrategy == CollapseNext) {
    GaussianMixtureModelRaw ytemp;
    y.phi = x.phi;
    for(size_t i=0;i<x.means.size();i++) {
      gmr.GetY(x.means[i],x.covariances[i],ytemp);
      //get the mean/cov of this GMM
      ytemp.GetMean(y.means[i]);
      ytemp.GetCovariance(y.covariances[i]);
    }
  }
  else if(samplingStrategy == CollapsePrev) {
    Vector mean;
    Matrix cov;
    x.GetMean(mean);
    x.GetCovariance(cov);
    gmr.GetY(mean,cov,y);
  }
  else if(samplingStrategy == Sample || samplingStrategy == Fit) {
    if(&y == &x) { //errors if y is the same pointer as x
      GaussianMixtureModelRaw xtemp=x;
      Propagate(gmr,xtemp,y);
      return;
    }
    y.phi.resize(x.means.size()*gmr.joint.gaussians.size());
    y.means.resize(y.phi.size());
    y.covariances.resize(y.phi.size());
    GaussianMixtureModelRaw ytemp;
    for(size_t i=0;i<x.means.size();i++) {
      if(!IsFinite(x.covariances[i])) {
	printf("Warning: input covariance matrix %d is not finite\n",i);
	for(size_t j=0;j<gmr.regressions.size();j++) {
	  size_t k=i*ytemp.means.size()+j;	  
	  y.phi[k] = 0.0;
	  y.means[k].resize(gmr.yindices.size(),0.0);
	  y.covariances[k].resize(gmr.yindices.size(),gmr.yindices.size(),0.0);
	}
	continue;
      }
      gmr.GetY(x.means[i],x.covariances[i],ytemp);
      for(size_t j=0;j<ytemp.means.size();j++) {
	size_t k=i*ytemp.means.size()+j;
	y.phi[k] = x.phi[i]*ytemp.phi[j];
	y.means[k] = ytemp.means[j];
	y.covariances[k] = ytemp.covariances[j];
	Gaussian<Real> temp;
	temp.setMean(y.means[k]);
	if(!temp.setCovariance(y.covariances[k],1)) {
	  printf("propagating component %d through model component %d failed\n",i,j);
	  cout<<x.covariances[i]<<endl;
	  cout<<y.covariances[k]<<endl;
	  getchar();
	}
	if(!IsFinite(y.covariances[k])) {
	  printf("Covariance %d %d became infinite\n",i,j);
	  cout<<y.covariances[k]<<endl;
	  getchar();
	  continue;
	}
      }
    }
    if(samplingStrategy == Sample) {
      //do a resampling
      if((int)y.phi.size() > maxGaussians)
	y.Resample(maxGaussians);
    }
    else {
      if(!Refit(y,maxGaussians)) {
	printf("Warning: failed to refit on propagate\n");
      }
    }
    for(size_t k=0;k<y.covariances.size();k++)
      if(!IsFinite(y.covariances[k])) {
	printf("After resample, covariance %d became infinite\n",k);
	cout<<y.covariances[k]<<endl;
	getchar();
	continue;
      }
  }
}

void GMRFilter::Propagate(const vector<Real>& weights,
	       const vector<Matrix>& As,const vector<Vector>& bs,
	       const vector<Matrix>& errs,
		 GaussianMixtureModelRaw& x,GaussianMixtureModelRaw& y) const
{
  if(samplingStrategy == CollapseNext) {
    GaussianMixtureModelRaw ytemp;
    ytemp.phi = weights;
    ytemp.means.resize(weights.size());
    ytemp.covariances.resize(weights.size());
    Matrix mtemp;
    y.phi = x.phi;
    for(size_t i=0;i<x.means.size();i++) {
      for(size_t j=0;j<weights.size();j++) {
	As[j].mul(x.means[i],ytemp.means[j]);
	ytemp.means[j] += bs[j];
	mtemp.mul(As[j],x.covariances[i]);
	ytemp.covariances[j].mulTransposeB(mtemp,As[j]);
	ytemp.covariances[j] += errs[j];
      }
      //get the mean/cov of this GMM
      ytemp.GetMean(y.means[i]);
      ytemp.GetCovariance(y.covariances[i]);
    }
  }
  else if(samplingStrategy == CollapsePrev) {
    Vector mean;
    Matrix cov;
    x.GetMean(mean);
    x.GetCovariance(cov);
    y.phi = weights;
    y.means.resize(weights.size());
    y.covariances.resize(weights.size());
    Matrix mtemp;
    for(size_t j=0;j<weights.size();j++) {
      As[j].mul(mean,y.means[j]);
      y.means[j] += bs[j];
      mtemp.mul(As[j],cov);
      y.covariances[j].mulTransposeB(mtemp,As[j]);
      y.covariances[j] += errs[j];
    }
  }
  else if(samplingStrategy == Sample || samplingStrategy == Fit) {
    if(&y == &x) { //errors if y is the same pointer as x
      GaussianMixtureModelRaw xtemp=x;
      Propagate(weights,As,bs,errs,xtemp,y);
      return;
    }
    y.phi.resize(x.means.size()*weights.size());
    y.means.resize(y.phi.size());
    y.covariances.resize(y.phi.size());
    Matrix mtemp;
    for(size_t i=0;i<x.means.size();i++) {
      for(size_t j=0;j<weights.size();j++) {
	size_t k=i*weights.size()+j;
	y.phi[k] = x.phi[i]*weights[j];
	As[j].mul(x.means[i],y.means[k]);
	y.means[k]+=bs[j];
	mtemp.mul(As[j],x.covariances[i]);
	y.covariances[k].mulTransposeB(mtemp,As[j]);
	y.covariances[k] += errs[j];
      }
    }
    if(samplingStrategy == Sample) {
      //do a resampling
      if((int)y.phi.size() > maxGaussians)
	y.Resample(maxGaussians);
    }
    else {
      if(!Refit(y,maxGaussians)) {
	printf("Warning: failed to refit on propagate\n");
      }
    }
  }
}

Real GMRFilter::ObservationUpdate(const GaussianMixtureRegression& gmr_obs,const Vector& obs,GaussianMixtureModelRaw& x) const
{
  //compute hidden,history|obs ~ N([mu1,mu2],[K11,K12;K21,K22])
  //given prior hidden,history ~ N([muh,hist],[Kh,0;0,0])
  //Let x = hidden,history
  //Model obs = H x + i + Ko
  //joint x,obs ~ N([mux,a],[Kx,B;B^T,C])
  //Kalman update says:
  //  a = H mux + i (mean of predicted observation)
  //  C = H Kx H^T + Ko (covariance of predicted observation)
  //  mux' = mux-Kx H^T C^-1 (obs-(H mux+i))
  //  Kx'  = Kx-Kx H^T C^-1 H Kx
  //Note that Kx = [Kh,0;0,0]
  //  muh' = muh+[Kh,0] H^T C^-1 (obs-(H mux+i))
  //  Kh'  = Kh- [Kh,0] H^T C^-1 H [Kh,0]^T
  Matrix A,err;
  Vector b;
  //probability of obs given xi
  vector<Real> logobsprobs;
  if(samplingStrategy == CollapseNext) {
    GetLinearFit(gmr_obs,A,b,err);
    logobsprobs.resize(x.means.size());
    for(size_t i=0;i<x.means.size();i++)
      logobsprobs[i] = KalmanUpdate(x.means[i],x.covariances[i],obs,A,b,err);
  }
  else if(samplingStrategy == CollapsePrev) {
    Vector mean;
    Matrix cov;
    x.GetMean(mean);
    x.GetCovariance(cov);
    logobsprobs.resize(gmr_obs.regressions.size());
    for(size_t i=0;i<gmr_obs.regressions.size();i++) {
      gmr_obs.regressions[i].GetLinearEquation(A,b);
      gmr_obs.regressions[i].GetNoiseCovariance(err);
      x.means[i] = mean;
      x.covariances[i] = cov;
      x.phi[i] = gmr_obs.joint.phi[i];
      logobsprobs[i] = KalmanUpdate(x.means[i],x.covariances[i],obs,A,b,err);
    }
  }
  else {
    GaussianMixtureModelRaw xtemp;
    xtemp.phi.resize(x.means.size()*gmr_obs.regressions.size());
    xtemp.means.resize(xtemp.phi.size());
    xtemp.covariances.resize(xtemp.phi.size());
    logobsprobs.resize(xtemp.phi.size());
    for(size_t j=0;j<gmr_obs.regressions.size();j++) {
      gmr_obs.regressions[j].GetLinearEquation(A,b);
      gmr_obs.regressions[j].GetNoiseCovariance(err);
      if(!IsFinite(err)) {
	printf("Warning, observation noise term %d is non finite\n",j);
	cout<<gmr_obs.regressions[j].ycov<<endl;
	cout<<gmr_obs.regressions[j].yxcov<<endl;
	cout<<gmr_obs.regressions[j].xcovinv<<endl;
      }
      for(size_t i=0;i<x.means.size();i++) {
	int k=i+j*x.means.size();
	xtemp.phi[k] = x.phi[i]*gmr_obs.joint.phi[j];
	xtemp.means[k] = x.means[i];
	xtemp.covariances[k] = x.covariances[i];
	logobsprobs[k] = KalmanUpdate(xtemp.means[k],xtemp.covariances[k],obs,A,b,err);
	//printf("Kalman update %d %d = %g\n",i,j,logobsprobs[k]);
      }
    }
    x = xtemp;
  }

  Assert(x.phi.size()==logobsprobs.size());
  Real maxlogp=-Inf;
  for(size_t i=0;i<logobsprobs.size();i++) 
    maxlogp=Max(maxlogp,logobsprobs[i]);
  //divide xi by Exp(maxlogp)
  Real sumw;
  if(IsFinite(maxlogp)) {
    for(size_t i=0;i<x.phi.size();i++) 
      x.phi[i]=x.phi[i]*Exp(logobsprobs[i]-maxlogp); 
    for(size_t i=0;i<x.phi.size();i++) {
      if(!IsFinite(x.phi[i])) {
	x.phi[i]=0.0;
	x.means[i].setZero();
	x.covariances[i].setIdentity();
      }
    }
    sumw = Normalize(x.phi);
  }
  else {
    printf("log probability is infinite: %g!\n",maxlogp);
    fill(x.phi.begin(),x.phi.end(),1.0/x.phi.size());
    sumw = 0.0;
    return -Inf;
  }

  //do resampling
  if(samplingStrategy==Sample) {
    if((int)x.phi.size() > maxGaussians)
      x.Resample(maxGaussians);
  }
  else if(samplingStrategy==Fit) {
    if(!Refit(x,maxGaussians)) {
      printf("Warning: failed to refit on observation update\n");
    }
  }

  for(size_t i=0;i<x.phi.size();i++) {
    Assert(IsFinite(x.phi[i]));
  }

  //constrain history to have zero variance
  int nhidden=(int)hiddenIndices.size();
  int nobs = (int)obsIndices.size();
  int nhist=(int)obsHistoryIndices.size()*nobs;
  for(size_t k=0;k<x.covariances.size();k++) {
    for(int i=0;i<nhist;i++)
      for(int j=0;j<x.covariances[k].n;j++)
	x.covariances[k](nhidden+i,j)=x.covariances[k](j,nhidden+i)=0.0;

  }

  //sumw = sum(P(o|x)P(x))/Exp(maxlogp)
  //log P(o) = log(sum P(o|x)P(x)) = log(sumw)+maxlogp
  return Log(sumw)+maxlogp;
}

void GMRFilter::HiddenToHiddenHist()
{
  int nhidden = (int)hiddenIndices.size();
  int nobs = (int)obsIndices.size();
  int nhist = (int)obsHistoryIndices.size()*nobs;
  
  //copy hidden back into hidden_hist
  hidden_hist.means.resize(hidden.means.size());
  hidden_hist.covariances.resize(hidden.covariances.size());
  hidden_hist.phi = hidden.phi;
  for(size_t i=0;i<hidden_hist.means.size();i++) {
    if(hidden_hist.means[i].n !=nhidden+nhist) {
      hidden_hist.means[i].resize(nhidden+nhist);
      for(size_t j=0;j<history.size();j++)
	hidden_hist.means[i].copySubVector(nhidden+j*nobs,history[j]);
    }
    hidden_hist.means[i].copySubVector(0,hidden.means[i]);
    if(hidden_hist.covariances[i].m != nhidden+nhist) {
      hidden_hist.covariances[i].resize(nhidden+nhist,nhidden+nhist,Zero);
    }
    hidden_hist.covariances[i].copySubMatrix(0,0,hidden.covariances[i]);
      /*
      if(!IsFinite(hidden.covariances[i])) {
	cout<<"Hidden covariance became non-finite"<<endl;
	cout<<hidden.covariances[i]<<endl;
	getchar();
	break;
      }
      */
  }
}
void GMRFilter::HiddenHistToHidden()
{
  int nhidden = (int)hiddenIndices.size();
  int nobs = (int)obsIndices.size();
  int nhist = (int)obsHistoryIndices.size()*nobs;

  //update hidden
  hidden.phi = hidden_hist.phi;
  hidden.means.resize(hidden_hist.phi.size());
  hidden.covariances.resize(hidden_hist.phi.size());
  for(size_t i=0;i<hidden.means.size();i++) {
    hidden.means[i].resize(nhidden);
    hidden.covariances[i].resize(nhidden,nhidden);
    hidden_hist.means[i].getSubVectorCopy(0,hidden.means[i]);
    hidden_hist.covariances[i].getSubMatrixCopy(0,0,hidden.covariances[i]);
  }
}

void GMRFilter::Predict()
{
  //Assert(IsConsistent());
  if(!gmr_trans.xindices.empty()) {
    Propagate(gmr_trans,hidden_hist,hidden);

    HiddenToHiddenHist();
  }
  //Assert(IsConsistent());
}

void GMRFilter::Update(const Vector& obs)
{
  //update hidden distribution
  logObservationProbability += ObservationUpdate(gmr_obs,obs,hidden_hist);

  HiddenHistToHidden();

  int nhidden = (int)hiddenIndices.size();
  int nobs = (int)obsIndices.size();
  //update history
  for(int i=(int)history.size()-1;i>0;i--)
    history[i]=history[i-1];
  history[0] = obs;
  //update hidden_hist distribution
  for(size_t i=0;i<hidden_hist.means.size();i++) {
    Vector vtemp;
    vtemp.setRef(hidden_hist.means[i],nhidden);
    for(size_t h=0;h<history.size();h++)
      vtemp.copySubVector(h*nobs,history[h]);
  }

  //Assert(IsConsistent());
} 

void GMRFilter::GetHidden(GaussianMixtureModelRaw& gmm_hidden) 
{
  gmm_hidden = hidden;
}


void ShiftDown(Matrix& A,int cnt)
{
  //first shift the history backwards by nobs
  for(int j=A.m-1;j>=cnt;j--) {
    for(int l=0;l<A.n;l++)
      A(j,l) = A(j-cnt,l);
  }
}

void ShiftDownRight(Matrix& A,int cntdown,int cntright)
{
  for(int j=A.m-1;j>=cntdown;j--) 
    for(int k=A.n-1;k>=cntright;k--) 
      A(j,k) = A(j-cntdown,k-cntright);
}

void GMRFilter::FillFullLinearFit()
{
  int nhidden=(int)hiddenIndices.size(),nhist=(int)(obsHistoryIndices.size()*obsIndices.size()),nobs=(int)obsIndices.size();
  int nsum = nobs;

  Matrix mtemp1,mtemp2;
  Vector vtemp1,vtemp2;
  Matrix Atrans,Aobs,etrans,eobs;
  Matrix At1,At2,Ao1,Ao2;
  Vector btrans,bobs;
  GetLinearFit(gmr_trans,Atrans,btrans,etrans);
  GetLinearFit(gmr_obs,Aobs,bobs,eobs);
  Afull.resize(nhidden+nhist+nsum,nhidden+nhist+nsum,Zero);
  efull.resize(Afull.m,Afull.n,Zero);
  bfull.resize(nhidden+nhist+nsum,Zero);
  //set up big transition matrix
  Afull.copySubMatrix(0,0,Atrans);
  bfull.copySubVector(0,btrans);
  efull.copySubMatrix(0,0,etrans);
  //set up history shift
  for(int i=0;i<nobs;i++)
    //copy observation into first history place
    Afull(nhidden+nobs+i,nhidden+i)=1.0;
  //copy each history over one
  for(size_t i=1;i<history.size();i++)
    for(int j=0;j<nobs;j++)
      Afull(nhidden+i*nobs+j,nhidden+(i-1)*nobs+j)=1.0;
  //set up observation row
  At1.setRef(Atrans,0,0,1,1,nhidden,nhidden);
  At2.setRef(Atrans,0,nhidden,1,1,nhidden,nhist);
  Ao1.setRef(Aobs,0,0,1,1,nobs,nhidden);
  Ao2.setRef(Aobs,0,nhidden,1,1,nobs,nhist);
  mtemp1.mul(Ao1,Atrans);
  mtemp2.setRef(mtemp1,0,nhidden,1,1,nobs,nhist);
  mtemp2 += Ao2;
  Afull.copySubMatrix(nhidden,0,mtemp1);
  Ao1.mul(btrans,vtemp1);
  vtemp1 += bobs;
  bfull.copySubVector(nhidden,vtemp1);
  //error term
  mtemp1.clear();
  mtemp2.clear();
  mtemp1.mul(Ao1,etrans);
  mtemp2.mulTransposeB(mtemp1,Ao1);
  mtemp2 += eobs;
  efull.copySubMatrix(nhidden,nhidden,mtemp2);
  //set up sum matrix
  if(nsum != 0) {
    mtemp1.clear();
    mtemp1.setRef(Afull,nhidden,0,1,1,nobs,nhidden+nhist);
    Afull.copySubMatrix(nhidden+nhist,0,mtemp1);
    for(int i=0;i<nobs;i++) {
      Afull(nhidden+nhist+i,nhidden+nhist+i)=1.0;
    }
    vtemp1.clear();
    vtemp1.setRef(bfull,nhidden,1,nobs);
    bfull.copySubVector(nhidden+nhist,vtemp1);
    efull.copySubMatrix(nhidden+nhist,nhidden+nhist,eobs);
  }
  /*
  cout<<"Full matrix:"<<endl;
  cout<<Afull<<endl;
  cout<<bfull<<endl;
  getchar();
  */
}

void GMRFilter::GetPrediction(int horizon,GaussianMixtureModelRaw& gmm_hidden_obs)
{
  int nhidden=(int)hiddenIndices.size(),nhist=(int)(obsHistoryIndices.size()*obsIndices.size()),nobs=(int)obsIndices.size();

  if(horizon == 0) {
    gmm_hidden_obs.Resize(hidden.means.size(),nhidden+nobs);
    gmm_hidden_obs.phi = hidden.phi;
    for(size_t i=0;i<hidden.means.size();i++) {
      gmm_hidden_obs.means[i].copySubVector(0,hidden.means[i]);
      gmm_hidden_obs.means[i].copySubVector(nhidden,history[0]);
      gmm_hidden_obs.covariances[i].setZero();
      gmm_hidden_obs.covariances[i].copySubMatrix(0,0,hidden.covariances[i]);
    }
    return;
  }

  //do we compute a sum of observations, or not?
  int nsum = nobs;
  //int nsum = 0;

  //layout of full matrix is (hidden,obs,history,sum)
  //hidden' = At1*hidden + At2*history + bt + et
  //obs' = Ao1*hidden' + Ao2*history + bo + eo
  //     = Ao1*At1*hidden + (Ao1*At2+Ao2)*history + (Ao1*bt+bo) + Ao1*et + eo
  //sum' = sum + obs'
  //[hidden' ] = [At1     |At2        |   ][hidden ] + [bt       ]
  //[obs'    ]   [Ao1*At1 |Ao1*At2+Ao2|   ][obs    ]   [Ao1*bt+bo]
  //[history']   [        |Eo  | S    |   ][history]   [0        ]
  //[sum'    ]   [Ao1*At1 |Ao1*At2+Ao2| I ][sum    ]   [Ao1*bt+bo]
  //error term is
  //[et |             |   |             ]
  //[   |eo+Ao1etAo1^T|   |             ]
  //[                                   ]
  //[                     |eo+Ao1etAo1^T]
  if(Afull.isEmpty()) FillFullLinearFit();
  GaussianMixtureModelRaw all_dist;
  all_dist.Resize(hidden.means.size(),nhidden+nhist+nsum);
  all_dist.phi = hidden.phi;
  //setup current distribution in all_dist
  for(size_t i=0;i<hidden.means.size();i++) {
    all_dist.means[i].setZero();
    all_dist.covariances[i].setZero();
    all_dist.means[i].copySubVector(0,hidden.means[i]);
    all_dist.covariances[i].copySubMatrix(0,0,hidden.covariances[i]);
    for(size_t j=0;j<history.size();j++) 
      all_dist.means[i].copySubVector(nhidden+j*nobs,history[j]);
  }

  //propagate
  //TODO: How to combine k*k components into k components?
  Vector vtemp1;
  Matrix mtemp1,mtemp2;
  for(int h=0;h<horizon;h++) {
    for(size_t i=0;i<all_dist.means.size();i++) {
      vtemp1.clear();
      Afull.mul(all_dist.means[i],vtemp1);
      all_dist.means[i].add(vtemp1,bfull);


      mtemp1.clear();
      mtemp2.clear();
      mtemp1.mul(Afull,all_dist.covariances[i]);
      mtemp2.mulTransposeB(mtemp1,Afull);
      all_dist.covariances[i].add(mtemp2,efull);
    }
  }

  /*
  cout<<"Covariance:"<<endl;
  cout<<all_dist.covariances[0]<<endl;
  getchar();
  */

  //copy hidden,obs into gmm_hidden_obs
  //unless nsum is nonzero, then copy the sum into observation slot
  gmm_hidden_obs.Resize(all_dist.means.size(),nhidden+nobs);
  gmm_hidden_obs.phi = all_dist.phi;
  vtemp1.clear();
  mtemp1.clear();
  mtemp2.clear();
  for(size_t i=0;i<all_dist.means.size();i++) {
    if(nsum) {
      //top of mean = hidden
      vtemp1.setRef(all_dist.means[i],0,1,nhidden);
      gmm_hidden_obs.means[i].copySubVector(0,vtemp1);
      vtemp1.setRef(all_dist.means[i],nhidden+nhist);
      gmm_hidden_obs.means[i].copySubVector(nhidden,vtemp1);

      //covariance matrix upper left = hidden covariance
      mtemp1.setRef(all_dist.covariances[i],0,0,1,1,nhidden,nhidden);
      gmm_hidden_obs.covariances[i].copySubMatrix(0,0,mtemp1);      
      //lower right = sum covariance
      mtemp1.setRef(all_dist.covariances[i],nhidden+nhist,nhidden+nhist,1,1,nobs,nobs);
      gmm_hidden_obs.covariances[i].copySubMatrix(nhidden,nhidden,mtemp1);
      //upper right hidden/sum covariance
      mtemp1.setRef(all_dist.covariances[i],0,nhidden+nhist,1,1,nhidden,nobs);
      gmm_hidden_obs.covariances[i].copySubMatrix(0,nhidden,mtemp1);
      //lower left sum/hidden covariance
      mtemp1.setRef(all_dist.covariances[i],nhidden+nhist,0,1,1,nobs,nhidden);
      gmm_hidden_obs.covariances[i].copySubMatrix(nhidden,0,mtemp1);
    }
    else {
      //hidden/obs are in order, just copy them over
      vtemp1.setRef(all_dist.means[i],0,1,nhidden+nobs);
      gmm_hidden_obs.means[i] = vtemp1;
      mtemp1.setRef(all_dist.covariances[i],0,0,1,1,nhidden+nobs,nhidden+nobs);
      gmm_hidden_obs.covariances[i] = mtemp1;
    }
  }
}

void GMRFilter::GetPredictionTrace(int horizon,vector<Vector>& hidden_obs_means) 
{
  int nhidden=(int)hiddenIndices.size(),nhist=(int)(obsHistoryIndices.size()*obsIndices.size()),nobs=(int)obsIndices.size();

  if(horizon == 0) {
    Vector hiddenMean;
    hidden.GetMean(hiddenMean);
    hidden_obs_means.resize(1);
    hidden_obs_means[0].resize(nhidden+nobs);
    hidden_obs_means[0].copySubVector(0,hiddenMean);
    hidden_obs_means[0].copySubVector(nhidden,history[0]);
    return;
  }

  //do we compute a sum of observations, or not?
  int nsum = nobs;
  //int nsum = 0;

  //layout of full matrix is (hidden,obs,history,sum)
  //hidden' = At1*hidden + At2*history + bt + et
  //obs' = Ao1*hidden' + Ao2*history + bo + eo
  //     = Ao1*At1*hidden + (Ao1*At2+Ao2)*history + (Ao1*bt+bo) + Ao1*et + eo
  //sum' = sum + obs'
  //[hidden' ] = [At1     |At2        |   ][hidden ] + [bt       ]
  //[obs'    ]   [Ao1*At1 |Ao1*At2+Ao2|   ][obs    ]   [Ao1*bt+bo]
  //[history']   [        |Eo  | S    |   ][history]   [0        ]
  //[sum'    ]   [Ao1*At1 |Ao1*At2+Ao2| I ][sum    ]   [Ao1*bt+bo]
  //error term is
  //[et |             |   |             ]
  //[   |eo+Ao1etAo1^T|   |             ]
  //[                                   ]
  //[                     |eo+Ao1etAo1^T]
  if(Afull.isEmpty()) FillFullLinearFit();

  Vector allMean;
  allMean.resize(nhidden+nhist+nsum,Zero);
  Vector hiddenMean;
  hiddenMean.setRef(allMean,0,1,nhidden);
  hidden.GetMean(hiddenMean);
  for(size_t j=0;j<history.size();j++) 
    allMean.copySubVector(nhidden+j*nobs,history[j]);
  //propagate memoization
  //xt+1 = A xt + b
  //xt+2 = A^2 xt + (A + I) b
  //xt+k = A^k xt + (sum_i=0^k-1 A^i) b
  //precompute A^k and (sum_i=0^k-1 A^i) b
  size_t kmemo = Apred.size();
  if(horizon > (int)kmemo) {
    Apred.resize(horizon);
    bpred.resize(horizon);
    if(kmemo == 0) {
      Apred[0] = Afull;
      bpred[0] = bfull;
      kmemo++;
    }
    for(int h=(int)kmemo;h<horizon;h++) {
      Apred[h].mul(Afull,Apred[h-1]);
      Afull.mul(bpred[h-1],bpred[h]);
      bpred[h]+=bfull;
    }
  }
  hidden_obs_means.resize(horizon);
  Vector vtemp1,vtemp2;
  for(int h=0;h<horizon;h++) {
    vtemp1.clear();
    Apred[h].mul(allMean,vtemp1);
    vtemp2.add(vtemp1,bpred[horizon-1]);
    
    //copy into hidden_obs_means
    hidden_obs_means[h].resize(nhidden+nobs);
    vtemp1.clear();
    if(nsum) {
      //top of mean = hidden
      vtemp1.setRef(vtemp2,0,1,nhidden);
      hidden_obs_means[h].copySubVector(0,vtemp1);
      vtemp1.setRef(vtemp2,nhidden+nhist);
      hidden_obs_means[h].copySubVector(nhidden,vtemp1);
    }
    else {
      //hidden/obs are in order, just copy them over
      vtemp1.setRef(vtemp2,0,1,nhidden+nobs);
      hidden_obs_means[h] = vtemp1;
    }
  }
}

bool GMRFilter::IsConsistent()
{
  if(hidden.phi != hidden_hist.phi) {
    printf("Hidden weights not consistent\n");
    return false;
  }
  if(hidden.means.size() != hidden_hist.phi.size()) {
    printf("Invalid size of hidden means\n");
    return false;
  }
  if(hidden.covariances.size() != hidden_hist.phi.size()) {
    printf("Invalid size of hidden covariances\n");
    return false;
  }
  int nhidden=(int)hiddenIndices.size(),nhist=(int)(obsHistoryIndices.size()*obsIndices.size()),nobs=(int)obsIndices.size(); 
  hidden.means.resize(hidden_hist.phi.size());
  hidden.covariances.resize(hidden_hist.phi.size());
  for(size_t i=0;i<hidden.means.size();i++) {
    Vector vtemp;
    vtemp.setRef(hidden_hist.means[i],0,1,nhidden);
    if(vtemp != hidden.means[i]) {
      printf("Mean %d hidden components aren't equal\n",i);
      cout<<vtemp<<endl;
      cout<<hidden.means[i]<<endl;
      return false;
    }
    Matrix mtemp;
    mtemp.setRef(hidden_hist.covariances[i],0,0,1,1,nhidden,nhidden);
    if(mtemp != hidden.covariances[i]) {
      printf("Covariance %d hidden components aren't equal\n",i);
      cout<<mtemp<<endl;
      cout<<hidden.covariances[i]<<endl;
      return false;
    }
    for(size_t h=0;h<history.size();h++) {
      vtemp.setRef(hidden_hist.means[i],nhidden+h*nobs,1,nobs);
      if(vtemp != history[h]) {
	printf("Mean %d history component %d doesn't match\n",i,h);
	cout<<vtemp<<endl;
	cout<<history[h]<<endl;
	return false;
      }
    }
  }
  return true;
}
