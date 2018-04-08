#include "LPHMMFilter.h"

LPHMMFilter::LPHMMFilter() 
  : historyLength(1),maxComponents(10) 
{ }

void LPHMMFilter::Init()
{
  //if(historyLength > 1) {
  if(false) {
    //fix up history prediction
    const vector<int>& xindices=lphmmr.xindices;
    int no = xindices.size()/historyLength;
    for(size_t i=0;i<lphmmr.joint.emissionModels.size();i++) {
      Matrix Ahist,errhist;
      Vector bhist;
      Ahist.setRef(lphmmr.joint.emissionModels[i].A,xindices[no],xindices[0],1,1,(historyLength-1)*no,historyLength*no);
      Ahist.setZero();
      for(int j=0;j<(historyLength-1)*no;j++)
	Ahist(j,j) = 1.0;

      bhist.setRef(lphmmr.joint.emissionModels[i].error.mu,xindices[no],1,(historyLength-1)*no);
      bhist.setZero();

      //clear rows of error corresponding to history prediction
      Matrix cov;
      lphmmr.joint.emissionModels[i].error.getCovariance(cov);
      errhist.setRef(cov,xindices[no],0,1,1,(historyLength-1)*no,cov.n);
      errhist.setZero();
      errhist.setRef(cov,0,xindices[no],1,1,cov.n,(historyLength-1)*no);
      errhist.setZero();
      for(int j=0;j<(historyLength-1)*no;j++)
	cov(xindices[no]+j,xindices[no]+j) = 1;
      lphmmr.joint.emissionModels[i].error.setCovariance(cov);
    }
    lphmmr.SetXIndices(xindices);

    cout<<"New models: "<<endl;
    for(size_t i=0;i<lphmmr.xRegressions.size();i++) {
      cout<<lphmmr.xRegressions[i].A<<endl;
      cout<<lphmmr.xRegressions[i].error.mu<<endl;
      cout<<lphmmr.xRegressions[i].error.L<<endl;
    }
  }
  Reset();
}

void LPHMMFilter::Reset()
{
  lphmmr.GetInitial(state);
  history.clear();
  logObservationProbability = 0;
}
void LPHMMFilter::Predict() 
{ 
  LinearProcessHMMState next; 
  lphmmr.Predict(state,next);
  state=next;
}

void LPHMMFilter::Update(const Vector& obs)
{
  LinearProcessHMMState next;
  if(history.empty()) {
    assert(historyLength >= 1);
    history.resize(historyLength);
    fill(history.begin(),history.end(),obs);
  }
  else {
    for(int i=(int)history.size()-1;i>0;i--)
      history[i]=history[i-1];
    history[0] = obs;
  }
  //assumes the x variable contains coherent stack of history in increasing 
  //predecessor order
  Vector histvec(history.size()*obs.n);
  for(size_t i=0;i<history.size();i++)
    histvec.copySubVector(i*obs.n,history[i]);

  lphmmr.Update(state,histvec,next);
  next.CollapseWeighted(maxComponents);

  //compute probability of observation (not the history)
  vector<int> obsindices(obs.n);
  for(int i=0;i<obs.n;i++)
    obsindices[i] = i;
  Real px = 0.0;
  if(state.xPrev.empty() && state.x.empty()) {
    for(int i=0;i<state.p.n;i++) {
      if(state.p(i) != 0.0)
	px += state.p(i) * lphmmr.xPriors[i].partialProbability(obs,obsindices);
    }
  }
  else {
    for(int i=0;i<state.p.n;i++) {
      if(state.p(i) != 0.0) {
	Vector vtemp,vtemp2;
	if(!state.x.empty()) 
	  lphmmr.xRegressions[i].A.mul(state.x,vtemp);
	else
	  lphmmr.xRegressions[i].A.mul(state.xPrev,vtemp);
	vtemp2.setRef(vtemp,0,1,obs.n);
	vtemp2 -= obs;
	vtemp2.inplaceNegative();
	px += state.p(i) * lphmmr.xRegressions[i].error.partialProbability(vtemp2,obsindices);
      }
    }
  }
  logObservationProbability += Log(px);
  state=next;
}
Vector LPHMMFilter::LastObservation()
{
  if(history.empty()) return Vector();
  return history[0];
}
void LPHMMFilter::GetHidden(GaussianMixtureModelRaw& gmm_hidden)
{
  state.GetY(gmm_hidden);
}

void LPHMMFilter::GetPrediction(int horizon,GaussianMixtureModelRaw& gmm_hidden_obs)
{
  LinearProcessHMMState pred;
  lphmmr.Predict(state,horizon,pred);
  pred.GetXY(gmm_hidden_obs);
}
