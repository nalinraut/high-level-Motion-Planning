#ifndef FILTER_BASE_H
#define FILTER_BASE_H

#include <statistics/GaussianMixtureModel.h>
#include <math/matrix.h>
using namespace Statistics;
using namespace std;

/** @brief A base class for a probabilistic filter on a continuous space.
 */
class FilterBase
{
 public:
  FilterBase() {}
  virtual ~FilterBase() {}
  virtual void Init() {}
  virtual void Reset() {}
  virtual void Predict() { FatalError("Not implemented by subclass"); }
  virtual void Update(const Vector& obs) { FatalError("Not implemented by subclass"); }
  virtual Real LogObservationLikelihood() { return 0.0; }
  virtual void ClearObservationLikelihood() {}
  virtual Vector LastObservation() { FatalError("Not implemented by subclass"); return Vector(); }
  virtual void GetHidden(GaussianMixtureModelRaw& gmm_hidden) { FatalError("Not implemented by subclass"); }
  virtual void GetPrediction(int horizon,GaussianMixtureModelRaw& gmm_hidden_obs) { FatalError("Not implemented by subclass"); }
  virtual void GetPredictionTrace(int horizon,vector<Vector>& hidden_obs_means) { FatalError("Not implemented by subclass");}

  //convenience functions -- can be overridden if there are more efficient methods
  //for accessing just means, or means/covariances
  virtual void GetHidden(Vector& hidden)
  {
    Matrix cov;
    GetHidden(hidden,cov);
  }
  virtual void GetHidden(Vector& hiddenmean,Matrix& hiddencov)
  {
    GaussianMixtureModelRaw gmm;
    GetHidden(gmm);
    gmm.GetMean(hiddenmean);
    gmm.GetCovariance(hiddencov);
  }
  virtual void GetPrediction(int horizon,Vector& hidden_obs) {}
  virtual void GetPrediction(int horizon,Vector& hidden_obs_mean,Matrix& hidden_obs_cov) {}
};

#endif
