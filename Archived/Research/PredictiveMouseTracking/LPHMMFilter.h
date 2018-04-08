#ifndef LPHMM_FILTER_H
#define LPHMM_FILTER_H

#include "Filter.h"
#include <statistics/LinearProcessHMM.h>

struct LPHMMFilter : public FilterBase
{
  LPHMMFilter();
  virtual ~LPHMMFilter() {}
  virtual void Init();
  virtual void Reset();
  virtual void Predict() ;
  virtual void Update(const Vector& obs);
  virtual Real LogObservationLikelihood() { return logObservationProbability; }
  virtual void ClearObservationLikelihood() { logObservationProbability=0;}
  virtual Vector LastObservation();
  virtual void GetHidden(GaussianMixtureModelRaw& gmm_hidden);
  virtual void GetPrediction(int horizon,GaussianMixtureModelRaw& gmm_hidden_obs);

  LinearProcessHMMRegression lphmmr;
  LinearProcessHMMState state;
  vector<Vector> history;
  int historyLength;
  int maxComponents;
  Real logObservationProbability;
};

#endif

