#ifndef GMR_FILTER_H
#define GMR_FILTER_H

#include "Filter.h"

//this should probably be in some header file somewhere else...
Real Normalize(vector<Real>& w);

/** @brief Performs a Gaussian mixture autoregression.
 *
 * The GMR is defined over hidden variables z, observation variables o,
 * and an observation history h.  It maintains a distribution over
 * (zt,ot,ht) with ht=(ot-1,...,ot-m).  Since ot and ht are observed, a
 * distribution doesn't need to be stored.
 *
 * The estimated distribution over hidden variables and predicted 
 * observation variables can be retreived via GetHidden and GetPrediction.
 *
 * The user must set up obsgmm, transgmm, and all of the indices defining
 * z, o, and h.  Also, the user may set up a hidden state drift covariance
 * (if applicable).
 * To use, begin by calling Init() after these settings are set up.  Then,
 * for each time step call Predict() and Update(obs).
 */
class GMRFilter : public FilterBase
{
public:
  GMRFilter();
  //setup obsgmm, transgmm, initgmm and the indices before calling this
  virtual void Init();
  virtual void Reset();
  virtual void Predict();
  virtual void Update(const Vector& obs);
  virtual void GetHidden(GaussianMixtureModelRaw& gmm_hidden);
  virtual void GetPrediction(int horizon,GaussianMixtureModelRaw& gmm_hidden_obs);
  virtual void GetPredictionTrace(int horizon,vector<Vector>& hidden_obs_means);
  virtual Real LogObservationLikelihood() { return logObservationProbability; }
  virtual void ClearObservationLikelihood() { logObservationProbability = 0.0; }
  virtual Vector LastObservation() { return history[0]; }

  //propagate a gmm x through a gmr, with the current sampling strategy
  void Propagate(const GaussianMixtureRegression& gmr,GaussianMixtureModelRaw& x,GaussianMixtureModelRaw& y) const;
  //propagate a gmm x to a weighted set of linear models, with the current
  //sampling strategy
  void Propagate(const vector<Real>& weights,
		 const vector<Matrix>& As,const vector<Vector>& bs,
		 const vector<Matrix>& errs,
		 GaussianMixtureModelRaw& x,GaussianMixtureModelRaw& y) const;
  //update P(x|obs) given a prior on x, P(obs|x), and a value of obs.
  //returns log P(obs|x)
  Real ObservationUpdate(const GaussianMixtureRegression& gmr_obs,const Vector& obs,GaussianMixtureModelRaw& x) const;

  //Set this to be a model on obs' = diag(obsScale)*obs,
  //hidden' = diag(hiddenScale)*hidden
  void Scale(const Vector& obsScale,const Vector& hiddenScale);

  void FillFullLinearFit();
  void HiddenToHiddenHist();
  void HiddenHistToHidden();
  //tests for validity of internal representations
  bool IsConsistent();

  enum SamplingStrategy { CollapsePrev, CollapseNext, Sample, Fit };
  //how to deal with combinatorial multiplicity of gaussian
  SamplingStrategy samplingStrategy;
  //for the Sample/Fit strategies, how many components needed before resampling
  int maxGaussians;

  //settings: observation model
  GaussianMixtureModel obsgmm;
  vector<int> hiddenIndices,obsIndices;
  vector<vector<int> > obsHistoryIndices;

  //settings: hidden transition model -- assume indices are in same order
  //as in observation model
  GaussianMixtureModel transgmm;
  vector<int> currentIndices,nextIndices;
  vector<vector<int> > transHistoryIndices;

  //settings: initial distribution of hidden variables
  GaussianMixtureModel initgmm;

  //temporary variable: predict transition of hidden state given hidden/history
  GaussianMixtureRegression gmr_trans;
  //temporary variable: predict obs given hidden/history
  GaussianMixtureRegression gmr_obs;

  //full matrix (hist,obs,hist) fitted to transition model
  Matrix Afull,efull;
  Vector bfull;
  //prediction matrices, memoized
  vector<Matrix> Apred;
  vector<Vector> bpred;

  //state
  GaussianMixtureModelRaw hidden,hidden_hist;
  vector<Vector> history;
  Real logObservationProbability;
};

#endif
