#ifndef FREEFORM_TASK_PREDICTOR_H
#define FREEFORM_TASK_PREDICTOR_H

#include <math3d/primitives.h>
#include "GMRFilter.h"
#include "LPHMMFilter.h"
#include <utils/SmartPointer.h>
using namespace std;
using namespace Math;
using namespace Math3D;

struct MousePredictionSetup
{
  bool Read(const char* fn);
  const char* Type();
  bool SetupFilter(GMRFilter& filter);
  bool SetupFilter(LPHMMFilter& filter);
  //returns the log-probability of the obsrvation
  Real Update(FilterBase* filter,int x,int y,int dx,int dy,Real dt) const;
  void GetPredictions(FilterBase* filter,int time,int lastx,int lasty,GaussianMixtureModelRaw& mouse,GaussianMixtureModelRaw& goal) const;
  void GetPredictions(FilterBase* filter,int horizon,int lastx,int lasty,vector<Vector2>& mouse,vector<Vector2>& goal) const;

  string name;
  double timestep;
  string transgmmfn,obsgmmfn,initgmmfn;
  string lphmmfn;
  vector<string> hiddenNames;
  vector<string> observationNames;
  bool obs_absolute,goal_absolute;

  //shift/scaling
  int numVariables;
  vector<Real> offset,scale;

  //for the hidden GMM
  vector<int> currentVars,nextVars;
  vector<vector<int> > transHistoryVars;

  //for the observation GMM
  vector<int> hiddenVars,observationVars;
  vector<vector<int> > obsHistoryVars;

  //outputting the target -- original indices and subset of hidden indices
  vector<int> goalVars;
  vector<int> goalSubset;
};


class FreeformTaskPredictor
{
public:
  bool Init(const char* setupfile);
  void Reset(int x,int y);
  //returns true if any predictions were updated
  bool MouseInput(Real time,int x,int y);
  bool IdleInput(Real time);
  //updates pfilter, predMouse, and predGoal
  //if getTrace is true, updates predMouseTrace and predGoalTrace
  void UpdatePredictions(int horizon = 0, bool getTrace=true);

  void PrintLinearModels();
  bool RunReachTrials(const char* infn,const char* outfn);
  bool RunTrajTrials(const char* infn,const char* outfn);
  //bool TrainRecognizer(const vector<string>& testfns,const char* outfn);
  bool UpdateFilters(Real t,int x,int y);

  vector<MousePredictionSetup> filterInfo;
  vector<SmartPointer<FilterBase> > filters;
  vector<double> lasttimes;
  vector<int> lastmousex,lastmousey;
  int lastx,lasty;
  Real lastt;
  vector<vector<Real> > logpobs;

  vector<Real> pfilter;
  vector<GaussianMixtureModelRaw> predMouse,predGoal;
  vector<vector<Vector2> > predMouseTrace,predGoalTrace;
};

#endif
