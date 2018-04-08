#include "FreeformTaskPredictor.h"
#include <utils/stringutils.h>
#include <math/backsubstitute.h>
#include <tinyxml/tinyxml.h>
#include <fstream>
#include <sstream>


template <class T>
bool ReadStr(const string& str,T& val)
{
  stringstream ss(str);
  ss>>val;
  return ss;
}

bool ReadCSV(const char* fn,vector<vector<string> >& array,char delim=',')
{
  array.resize(0);
  string line;
  int pos;
 
  ifstream in(fn,ios::in);
  if(!in) { return false;   }
  while( getline(in,line) ) {
    vector<string> ln;
    while( (pos = line.find(delim)) != string::npos)
      {
	string field = line.substr(0,pos);
	line = line.substr(pos+1);
	ln.push_back(field);
      }
    ln.push_back(line.substr(0,line.length()-1));
    array.push_back(ln);
  }
  return true;
}

bool ReadTestTrials(const char* fn,vector<vector<Vector> >& trialData)
{
  //read a CSV file
  vector<vector<string> > entries;
  if(!ReadCSV(fn,entries)) {
    printf("Unable to read test file %s\n",fn);
    return false;
  }
  for(size_t i=1;i<entries.size();i++) {
    assert(entries.size()>=6);
    int trial;
    Real time,dx,dy,gx,gy,gr=0;
    ReadStr(entries[i][0],trial);
    ReadStr(entries[i][1],time);
    ReadStr(entries[i][2],dx);
    ReadStr(entries[i][3],dy);
    ReadStr(entries[i][4],gx);
    ReadStr(entries[i][5],gy);
    if(entries[i].size() > 6) {
      ReadStr(entries[i][6],gr);
    }
    Vector v(6);
    v[0] = time;
    v[1] = dx;
    v[2] = dy;
    v[3] = gx;
    v[4] = gy;
    v[5] = gr;
    assert(trial >= 0);
    if(trial >= trialData.size())
      trialData.resize(trial+1);
    trialData[trial].resize(trialData[trial].size()+1);
    trialData[trial].back()=v;
  }
  return true;
}

Real PHitSphere(const Gaussian<Real>& g,const Vector& c,Real r)
{
  //determine whether r is small or large w.r.t. the principal axes of the
  //covariance
  assert(g.mu.n == 2);
  //look for very far distance
  Vector temp,y;
  temp.sub(c,g.mu);
  LBackSubstitute(g.L,temp,y);
  bool approximationValid=true;
  for(int i=0;i<g.L.n;i++) {
    //use approximation if y(i) is not too close to center
    Real dist = Abs(y(i)) - Abs(r/g.L(i,i));
    if(dist < 1.0) //must be farther than one std dev on all axes
      approximationValid = false;
  }
  if(approximationValid)
    //TODO: generalize to hyperspheres
    return g.probability(c)*TwoPi*Sqr(r);

  //pointwise approximation is not valid, now compute integral instead
  int samples = 1000;
  int hits = 0;
  for(int i=0;i<samples;i++) {
    g.generate(temp);
    if(c.distanceSquared(temp) < Sqr(r))
      hits++;
  }
  return Real(hits)/Real(samples);
}


//approximation of the probability that x~N(mu,cov) lies within a hypersphere
//of radius r and center c
Real PHitSphere(const Vector& mu,const Matrix& cov,const Vector& c,Real r)
{
  Gaussian<Real> g;
  g.mu = mu;
  bool res=g.setCovariance(cov,1);
  if(!res) return 0.0;
  return PHitSphere(g,c,r);
}

Real PHitSphere(const GaussianMixtureModelRaw& gmm,const Vector& c,Real r)
{
  Real p = 0.0;
  for(size_t i=0;i<gmm.phi.size();i++) 
    p += gmm.phi[i]*PHitSphere(gmm.means[i],gmm.covariances[i],c,r);
  return p;
}



template <class T>
void ReadList(const string& s,vector<T>& items)
{
  stringstream ss(s);
  T val;
  while(ss) {
    ss>>val;
    if(ss) { items.push_back(val); }
  }
}

const char* MousePredictionSetup::Type()
{
  if(lphmmfn.empty()) { return "gmr"; }
  else return "lphmm";
}

bool MousePredictionSetup::Read(const char* fn)
{
  TiXmlDocument doc;
  if(!doc.LoadFile(fn)) return false;
  TiXmlElement* e=doc.RootElement();
  assert(e != NULL);
  TiXmlElement* setup = e->FirstChildElement("setup");
  assert(setup != NULL);
  if(setup->QueryStringAttribute("name",&name)!=TIXML_SUCCESS) {
    return false;
  }
  if(setup->QueryValueAttribute("numVariables",&numVariables)!=TIXML_SUCCESS) {
    return false;
  }
  if(setup->QueryValueAttribute("timestep",&timestep)!=TIXML_SUCCESS) {
    return false;
  }
  string hiddenstr,obsstr,histstr,goalstr;
  if(setup->QueryStringAttribute("observation",&obsstr)==TIXML_SUCCESS) {
    //has labels
    observationNames=Split(obsstr,";");
    cout<<obsstr<<endl;
  }
  if(setup->QueryStringAttribute("hidden",&hiddenstr)==TIXML_SUCCESS) {
    //has labels
    hiddenNames=Split(hiddenstr,";");
    cout<<hiddenstr<<endl;
  }

  TiXmlElement* prep = e->FirstChildElement("preprocessor");
  offset.resize(numVariables);
  scale.resize(numVariables);
  if(prep) {
    string offsetstr,scalestr;
    if(prep->QueryStringAttribute("offset",&offsetstr)!=TIXML_SUCCESS) {
      return false;
    }
    if(prep->QueryStringAttribute("scale",&scalestr)!=TIXML_SUCCESS) {
      return false;
    }
    {
      stringstream ss(offsetstr);
      for(int i=0;i<numVariables;i++)
	ss >> offset[i];
    }
    {
      stringstream ss(scalestr);
      for(int i=0;i<numVariables;i++)
	ss >> scale[i];
    }
  }
  else {
    fill(offset.begin(),offset.end(),0);
    fill(scale.begin(),scale.end(),1);
  }

  TiXmlElement* gmm = e->FirstChildElement("observation_model");
  assert(gmm != NULL);
  if(gmm->QueryStringAttribute("gmm",&obsgmmfn)!=TIXML_SUCCESS) {
    //return false;
  }
  if(gmm->QueryStringAttribute("lphmm",&lphmmfn)!=TIXML_SUCCESS) {
    //return false;
  }
  if(obsgmmfn.empty() &&  lphmmfn.empty()) {
    printf("Failed to read gmm or lphmm attribute\n");
    return false;
  }
  if(gmm->QueryStringAttribute("hidden",&hiddenstr)!=TIXML_SUCCESS) {
    return false;
  }
  if(gmm->QueryStringAttribute("observation",&obsstr)!=TIXML_SUCCESS) {
    printf("Failed to read observation attribute\n");
    return false;
  }
  if(gmm->QueryStringAttribute("history",&histstr)!=TIXML_SUCCESS) {
    printf("Not reading any history\n");
    //return false;
  }
  if(gmm->QueryStringAttribute("goal",&goalstr)!=TIXML_SUCCESS) {
    printf("Failed to read goal attribute\n");
    return false;
  }
  ReadList(hiddenstr,hiddenVars);
  ReadList(obsstr,observationVars);
  vector<string> hist = Split(histstr,";");
  obsHistoryVars.resize(hist.size());
  for(size_t i=0;i<hist.size();i++) {
    ReadList(hist[i],obsHistoryVars[i]);
  }
  ReadList(goalstr,goalVars);
  //find hidden indices of goals
  goalSubset.resize(goalVars.size());
  for(size_t i=0;i<goalVars.size();i++) {
    int index=-1;
    for(size_t j=0;j<hiddenVars.size();j++) 
      if(goalVars[i] == hiddenVars[j])
	index = (int)j;
    assert(index >= 0);
    goalSubset[i] = index;
  }

  gmm = e->FirstChildElement("transition_model");
  if(gmm) {
    if(gmm->QueryStringAttribute("gmm",&transgmmfn)!=TIXML_SUCCESS) {
      return false;
    }
    if(gmm->QueryStringAttribute("current",&hiddenstr)!=TIXML_SUCCESS) {
      return false;
    }
    if(gmm->QueryStringAttribute("next",&obsstr)!=TIXML_SUCCESS) {
      return false;
    }
    if(gmm->QueryStringAttribute("history",&histstr)!=TIXML_SUCCESS) {
      return false;
    }
    ReadList(hiddenstr,currentVars);
    ReadList(obsstr,nextVars);
    vector<string> hist = Split(histstr,";");
    transHistoryVars.resize(hist.size());
    for(size_t i=0;i<hist.size();i++) {
      ReadList(hist[i],transHistoryVars[i]);
    }
    if(obsHistoryVars.size() != transHistoryVars.size()) {
      printf("History size must match in observation and transition models\n");
      return false;
    }

    if(gmm->QueryStringAttribute("init",&initgmmfn)==TIXML_SUCCESS) {
      //it's ok
    }
    else
      initgmmfn = "";
  }

  //put model filenames into path-relative form
  char* buf = new char[strlen(fn)+1];
  GetFilePath(fn,buf);
  if(!transgmmfn.empty())
    transgmmfn = string(buf) + transgmmfn;
  if(!obsgmmfn.empty())
    obsgmmfn = string(buf) + obsgmmfn;
  if(!initgmmfn.empty())
    initgmmfn = string(buf) + initgmmfn;
  if(!lphmmfn.empty())
    lphmmfn = string(buf) + lphmmfn;
  delete [] buf;

  obs_absolute = goal_absolute = false;
  return true;
}

bool MousePredictionSetup::SetupFilter(GMRFilter& filter)
{
  assert(lphmmfn.empty());

  ifstream in(obsgmmfn.c_str(),ios::in);
  if(!in) {
    printf("Unable to open observation model file %s\n",obsgmmfn.c_str());
    return false;
  }
  in >> filter.obsgmm;
  if(!in) {
    printf("Unable to read observation model from %s\n",obsgmmfn.c_str());
    return false;
  }
  in.close();

  if(!transgmmfn.empty())
  {
    ifstream in(transgmmfn.c_str(),ios::in);
    if(!in) {
      printf("Unable to open transition model file %s\n",transgmmfn.c_str());
      return false;
    }
    in >> filter.transgmm;
    if(!in) {
      printf("Unable to read transition model from %s\n",transgmmfn.c_str());
      return false;
    }
    in.close();
  }

  if(!initgmmfn.empty()) {
    ifstream in(initgmmfn.c_str(),ios::in);
    if(!in) {
      printf("Unable to open initial model file %s\n",initgmmfn.c_str());
      return false;
    }
    in >> filter.initgmm;
    if(!in) {
      printf("Unable to read initial  model from %s\n",initgmmfn.c_str());
      return false;
    }
    in.close();
  }

  //observation model
  filter.hiddenIndices=hiddenVars;
  filter.obsIndices=observationVars;
  filter.obsHistoryIndices=obsHistoryVars;

  //transition model
  filter.currentIndices = currentVars;
  filter.nextIndices = nextVars;
  filter.transHistoryIndices = transHistoryVars;

  filter.Init();

  /*
  //TEMP: do rescaling
  for(size_t i=0;i<obsHistoryVars.size();i++)
    for(size_t j=0;j<obsHistoryVars[i].size();j++)
      Assert(scale[obsHistoryVars[i][j]] == scale[observationVars[j]]);
  Vector obsScale(observationVars.size()), hiddenScale(hiddenVars.size());
  for(size_t i=0;i<observationVars.size();i++)
    obsScale[i] = scale[observationVars[i]];
  for(size_t i=0;i<hiddenVars.size();i++)
    hiddenScale[i] = scale[hiddenVars[i]];
  filter.Scale(obsScale,hiddenScale);
  fill(scale.begin(),scale.end(),1.0);
  */

  //account for finite number of pixels
  //distribute prediction over a hypothetical 20x20 grid
  //use standard deviation of half a pixel
  Real npixels = 0.5;
  Real stdx = npixels/scale[observationVars[0]];
  Real stdy = npixels/scale[observationVars[1]];
  printf("Adding noise of std dev %g %g\n",stdx,stdy);
  for(size_t i=0;i<filter.gmr_obs.regressions.size();i++) {
    filter.gmr_obs.regressions[i].ycov(0,0) += Sqr(stdx);
    filter.gmr_obs.regressions[i].ycov(1,1) += Sqr(stdy);
    cout<<filter.gmr_obs.regressions[i].ycov<<endl;
  }
  //getchar();
  return true;
}

bool MousePredictionSetup::SetupFilter(LPHMMFilter& filter)
{
  ifstream in(lphmmfn.c_str(),ios::in);
  if(!in) {
    printf("Unable to open LPHMM model file %s\n",lphmmfn.c_str());
    return false;
  }
  in >> filter.lphmmr.joint;
  if(!in) {
    printf("Unable to read LPHMM model from %s\n",lphmmfn.c_str());
    return false;
  }
  in.close();

  assert(obsgmmfn.empty());
  assert(transgmmfn.empty());
  assert(initgmmfn.empty());
  vector<int> obs_hist_indices=observationVars;
  for(size_t i=0;i<obsHistoryVars.size();i++)
    obs_hist_indices.insert(obs_hist_indices.end(),obsHistoryVars[i].begin(),obsHistoryVars[i].end());
  //make sure they're in increasing coherent order
  for(size_t i=0;i+1<obs_hist_indices.size();i++) {
    if(obs_hist_indices[i+1] <= obs_hist_indices[i]) {
      printf("Error, incoherent LPHMM history sequence -- must be in order\n");
      return false;
    }
  }
  filter.historyLength = obsHistoryVars.size()+1;

  //observation model
  filter.lphmmr.SetXIndices(obs_hist_indices);
  filter.Init();

  //HACK: should put this in setup file
  //obs_absolute = true;
  //goal_absolute = true;
  obs_absolute = true;
  goal_absolute = true;
  return true;
}

Real MousePredictionSetup::Update(FilterBase* filter,int x,int y,int dx,int dy,Real dt) const
{
  if(timestep == 0 || FuzzyEquals(dt,timestep)) {
    Vector m(2);
    if(obs_absolute) {
      m(0) = (Real(x)-offset[observationVars[0]])/scale[observationVars[0]];
      m(1) = (Real(y)-offset[observationVars[1]])/scale[observationVars[1]];
    }
    else {
      m(0) = (Real(dx)-offset[observationVars[0]])/scale[observationVars[0]];
      m(1) = (Real(dy)-offset[observationVars[1]])/scale[observationVars[1]];
    }
    filter->ClearObservationLikelihood();
    filter->Predict();
    filter->Update(m);
    Real obsScale = scale[observationVars[0]]*scale[observationVars[1]];
    return filter->LogObservationLikelihood() - Log(obsScale);
  }
  else {
    int n = (int)Ceil(dt/timestep);
    Real u=1.0/Real(n);
    Vector m(2);
    if(obs_absolute) {
      //TODO: interpolate mouse position between currnt and destination
      m(0) = (Real(x)-offset[observationVars[0]])/scale[observationVars[0]];
      m(1) = (Real(y)-offset[observationVars[1]])/scale[observationVars[1]];
    }
    else {
      m(0) = (Real(dx)*u-offset[observationVars[0]])/scale[observationVars[0]];
      m(1) = (Real(dy)*u-offset[observationVars[1]])/scale[observationVars[1]];
    }
    filter->ClearObservationLikelihood();
    Real obsScale = scale[observationVars[0]]*scale[observationVars[1]];
    //printf("Scale %g %g, adding %g to logobs\n",scale[observationVars[0]],scale[observationVars[1]],-n*Log(obsScale));
    for(int j=1;j<=n;j++) {
      filter->Predict();
      filter->Update(m);
    }
    //printf("Raw obs likelihood %s: %g, result %g\n",name.c_str(),filter->LogObservationLikelihood(),filter->LogObservationLikelihood()-n*Log(obsScale));
    return filter->LogObservationLikelihood()-n*Log(obsScale);
  }
}

void MousePredictionSetup::GetPredictions(FilterBase* filter,int time,int lastx,int lasty,GaussianMixtureModelRaw& mouse,GaussianMixtureModelRaw& goal) const
{
  if(time == 0) {
    GaussianMixtureModelRaw hidden;
    filter->GetHidden(hidden);
    goal.SetSubset(hidden,goalSubset);
    mouse.Resize(1,2);
    mouse.phi[0] = 1.0;
    mouse.means[0]=filter->LastObservation();
    mouse.covariances[0].setZero();

    Matrix Agoal(2,2,Zero),Amouse(2,2,Zero);
    Vector bgoal(2),bmouse(2);
    Agoal(0,0) = scale[goalVars[0]];
    Agoal(1,1) = scale[goalVars[1]];
    bgoal(0) = offset[goalVars[0]];
    bgoal(1) = offset[goalVars[1]];
    Amouse(0,0) = scale[observationVars[0]];
    Amouse(1,1) = scale[observationVars[1]];
    bmouse(0) = offset[observationVars[0]];
    bmouse(1) = offset[observationVars[1]];
    if(!obs_absolute) {
      bmouse(0) += lastx;
      bmouse(1) += lasty;
    }
    mouse.SetLinearTransform(mouse,Amouse,bmouse);
    if(!goal_absolute) {
      bgoal(0) += lastx;
      bgoal(1) += lasty;
    }
    goal.SetLinearTransform(goal,Agoal,bgoal);
  }
  else {
    GaussianMixtureModelRaw gmm_hidden_obs,goalmouse;
    filter->GetPrediction(time,gmm_hidden_obs);
    int nhidden = (int)hiddenVars.size();
    int nobs = (int)observationVars.size();
    vector<int> goalmousesubset(4);
    goalmousesubset[0] = goalSubset[0];
    goalmousesubset[1] = goalSubset[1];
    goalmousesubset[2] = nhidden;
    goalmousesubset[3] = nhidden+1;
    goalmouse.SetSubset(gmm_hidden_obs,goalmousesubset);
    Matrix Ascale(4,4,Zero);
    Vector boffset(4);
    for(int j=0;j<2;j++) {
      Ascale(j,j) = scale[goalVars[j]];
      boffset(j) = offset[goalVars[j]];
      Ascale(j+2,j+2) = scale[observationVars[j]];
      boffset(j+2) = offset[observationVars[j]];
    }
    if(!obs_absolute) {
      //just offset the mouse position, it will be added later
      boffset(2) += lastx;
      boffset(3) += lasty;
    }
    goalmouse.SetLinearTransform(goalmouse,Ascale,boffset);
    vector<int> mouseIndices(2);
    mouseIndices[0] = 2;
    mouseIndices[1] = 3;
    mouse.SetSubset(goalmouse,mouseIndices);

    Matrix adder(2,4,Zero);
    Vector badder(2,Zero);
    adder(0,0) = adder(1,1) = 1.0;
    if(!goal_absolute) {
      adder(0,2) = adder(1,3) = 1.0;
    }
    goal.SetLinearTransform(goalmouse,adder,badder);
  }
}

void MousePredictionSetup::GetPredictions(FilterBase* filter,int horizon,int lastx,int lasty,vector<Vector2>& mouse,vector<Vector2>& goal) const
{
  if(horizon == 0) {
    goal.resize(1);
    mouse.resize(1);
    GaussianMixtureModelRaw hidden,goal_dist;
    filter->GetHidden(hidden);
    goal_dist.SetSubset(hidden,goalSubset);
    Vector goal_mean;
    goal_dist.GetMean(goal_mean);
    goal[0].set(goal_mean[0],goal_mean[1]);
    Vector lastobs = filter->LastObservation();
    mouse[0].set(lastobs[0],lastobs[1]);

    goal[0].x = scale[goalVars[0]]*goal[0].x + offset[goalVars[0]];
    goal[0].y = scale[goalVars[1]]*goal[0].y + offset[goalVars[1]];
    mouse[0].x = scale[observationVars[0]]*mouse[0].x + offset[observationVars[0]];
    mouse[0].y = scale[observationVars[1]]*mouse[0].y + offset[observationVars[1]];
    if(!obs_absolute) {
      mouse[0].x += lastx;
      mouse[0].y += lasty;
    }
    if(!obs_absolute) {
      goal[0].x += mouse[0].x;
      goal[0].y += mouse[0].y;
    }
  }
  else {
    vector<Vector> hidden_obs_means;
    filter->GetPredictionTrace(horizon,hidden_obs_means);
    int nhidden = (int)hiddenVars.size();
    vector<int> mousesubset(2);
    mousesubset[0] = nhidden;
    mousesubset[1] = nhidden+1;
    goal.resize(hidden_obs_means.size());
    mouse.resize(hidden_obs_means.size());
    for(size_t h=0;h<goal.size();h++) {
      goal[h].set(hidden_obs_means[h][goalVars[0]],hidden_obs_means[h][goalVars[1]]);
      mouse[h].set(hidden_obs_means[h][mousesubset[0]],hidden_obs_means[h][mousesubset[1]]);
      goal[h].x = scale[goalVars[0]]*goal[h].x+offset[goalVars[0]];
      goal[h].y = scale[goalVars[1]]*goal[h].y+offset[goalVars[1]];
      mouse[h].x = scale[observationVars[0]]*mouse[h].x+offset[observationVars[0]];
      mouse[h].y = scale[observationVars[1]]*mouse[h].y+offset[observationVars[1]];
      if(!obs_absolute) {
	mouse[h].x += lastx;
	mouse[h].y += lasty;
      }
      if(!goal_absolute) 
	goal[h] += mouse[h];
    }
  }
}


bool FreeformTaskPredictor::Init(const char* setupfile)
{
  lastx = -1;
  lasty = -1;
  lastt = 0;
  size_t oldSize = filterInfo.size();

  TiXmlDocument doc;
  if(!doc.LoadFile(setupfile)) {
    printf("Xml task setup file %s could not be opened/loaded\n",setupfile);
    return false;
  }
  TiXmlElement* e=doc.RootElement();
  assert(e != NULL);
  TiXmlElement* task = e->FirstChildElement("task");
  while(task != NULL) {
    filterInfo.resize(filterInfo.size()+1);
    filters.resize(filterInfo.size());
    pfilter.resize(filterInfo.size(),-1);
    logpobs.resize(filterInfo.size());
    MousePredictionSetup& setup=filterInfo.back();

    //parse attributes
    string name;
    if(task->QueryValueAttribute("name",&name)!=TIXML_SUCCESS) {
      name="";
    }
    if(task->QueryValueAttribute("prior",&pfilter.back())!=TIXML_SUCCESS) {
      pfilter.back()=-1;
    }

    //parse models
    TiXmlElement* model=task->FirstChildElement();
    if(!model) {
      printf("Task %d doesn't have a model element\n",filters.size()-1);
      return false;
    }
    if(0==strcmp(model->Value(),"gmr")) {
      string modelfile;
      if(model->QueryValueAttribute("file",&modelfile)!=TIXML_SUCCESS) {
	printf("GMR element doesn't have a file attribute %s\n",name.c_str());
	return false;
      }
      if(!setup.Read(modelfile.c_str())) {
	printf("Unable to read gmr setup file %s\n",modelfile.c_str());
	return false;
      }
      if(!name.empty()) setup.name = name;
      GMRFilter* filter = new GMRFilter;
      if(!setup.SetupFilter(*filter)) {
	printf("Warning, unable to set up filter %s\n",setup.name.c_str());
	exit(-1);
      }

      //read attributes
      string strategy;
      if(model->QueryValueAttribute("samplingStrategy",&strategy)==TIXML_SUCCESS) {
	if(strategy == "fit")
	  filter->samplingStrategy = GMRFilter::Fit;	
	else if(strategy == "sample")
	  filter->samplingStrategy = GMRFilter::Sample;	
	else if(strategy == "collapsePrev")
	  filter->samplingStrategy = GMRFilter::CollapsePrev;	
	else if(strategy == "collapseNext")
	  filter->samplingStrategy = GMRFilter::CollapseNext;
	else {
	  printf("Invalid sampling strategy %s\n",strategy.c_str());
	  printf("Valid values are fit, sample, collapsePrev, and collapseNext\n");
	  exit(-1);
	}
      }
      if(model->QueryValueAttribute("maxComponents",&filter->maxGaussians)!=TIXML_SUCCESS) {
	filter->maxGaussians = 10;
      }
      filters.back() = filter;
    }
    else if(0==strcmp(model->Value(),"lphmm")) {
      string modelfile;
      if(model->QueryValueAttribute("file",&modelfile)!=TIXML_SUCCESS) {
	printf("LPHMM element doesn't have a file attribute %s\n",name.c_str());
	return false;
      }
      if(!setup.Read(modelfile.c_str())) {
	printf("Unable to read lphmm setup file %s\n",modelfile.c_str());
	return false;
      }
      if(!name.empty()) setup.name = name;
      LPHMMFilter* filter = new LPHMMFilter;
      if(!name.empty()) setup.name = name;
      if(!setup.SetupFilter(*filter)) {
	printf("Warning, unable to set up filter %s\n",setup.name.c_str());
	exit(-1);
      }

      //read attributes
      if(model->QueryValueAttribute("maxComponents",&filter->maxComponents)!=TIXML_SUCCESS) {
	filter->maxComponents = 10;
      }
      filters.back() = filter;
    }
    else {
      FatalError("Unknown filter type %s",model->Value());
    }

    task = task->NextSiblingElement("task");
  }
  if(filters.empty()) {
    printf("No filters specified in setup file %s\n",setupfile);
    return false;
  }

  //clean up priors
  Real ppriorSum = 0.0;
  int numEmpty = 0;
  for(size_t i=oldSize;i<pfilter.size();i++) {
    if(pfilter[i] >= 0) ppriorSum += pfilter[i];
    else numEmpty++;
  }
  if(ppriorSum > 1) {
    printf("Prior values add up to more than 1!\n");
    return false;
  }
  for(size_t i=oldSize;i<pfilter.size();i++) 
    if(pfilter[i] < 0) pfilter[i] = (1.0-ppriorSum)/numEmpty;
  Normalize(pfilter);
  return true;
}

void FreeformTaskPredictor::Reset(int x,int y)
{
  lasttimes.resize(filters.size());
  lastmousex.resize(filters.size());
  lastmousey.resize(filters.size());
  for(size_t i=0;i<filters.size();i++) {
    filters[i]->Reset();
    filterInfo[i].Update(filters[i],x,y,0,0,filterInfo[i].timestep);
    lasttimes[i] = 0.0;
    lastmousex[i] = x;
    lastmousey[i] = y;
    pfilter[i] = 1.0/Real(filters.size());
    logpobs[i].resize(0);
  }
  lastx = x;
  lasty = y;
  lastt = 0;
}

bool FreeformTaskPredictor::UpdateFilters(Real t,int x,int y)
{
  int numUpdated=0;
  vector<Real> avglogpobs(filters.size(),0.0);
  vector<bool> updated(filters.size(),false);
  for(size_t i=0;i<filters.size();i++) {
    if(t < lasttimes[i] + filterInfo[i].timestep) continue;
    if(filterInfo[i].timestep == 0 && x==lastmousex[i] && y==lastmousey[i]) continue;

    updated[i] = true;
    numUpdated++;
    printf("Updating filter %s...\n",filterInfo[i].name.c_str());
    Real logpobs_i = filterInfo[i].Update(filters[i],x,y,x-lastmousex[i],y-lastmousey[i],t-lasttimes[i]);

    logpobs[i].push_back(logpobs_i);
    //take average of last k observations
    //int k=10;
    int k=1;
    int nresize=100;
    if(logpobs[i].size() > nresize) {
      copy(logpobs[i].begin()+nresize-k,logpobs[i].end(),logpobs[i].begin());
      logpobs[i].resize(k);
    }
    Real avglogobs = 0.0;
    int n =Min(k,(int)logpobs[i].size());
    for(size_t j=logpobs[i].size()-n;j<logpobs[i].size();j++)
      avglogobs += logpobs[i][j];
    avglogobs /= n;
    
    printf("Average log obs %s: %g\n",filterInfo[i].name.c_str(),avglogobs);
    avglogpobs[i] = avglogobs;
  }

  vector<Real> p=pfilter;
  for(size_t i=0;i<filters.size();i++) {
    if(!updated[i]) continue;

    //number of effective internal time steps
    Real nsteps = (filterInfo[i].timestep == 0? 1.0 : (t-lasttimes[i])/filterInfo[i].timestep);
    Real pobs = Exp(avglogpobs[i]);
   
    //adjust based on observation probability
    Real panomaly = 0;
    pobs = (1.0-panomaly)*pobs + panomaly;
    p[i]*=pobs;
  }
  Normalize(p);

  //compute transition distribution away from each distribution -- exponential distribution with rate 0.2
  Real switchProbability = (1.0-Exp(-(t-lastt)*0.2));
  //printf("Switch probability %g\n",switchProbability);
  for(size_t i=0;i<filters.size();i++) {
    pfilter[i] = (1.0-switchProbability)*p[i];
    for(size_t j=0;j<filters.size();j++) 
      pfilter[i] += switchProbability*p[j]/filters.size();
  }

  Normalize(pfilter);

  /*
  for(size_t j=0;j<p.size();j++)
    printf("%g ",p[j]);
  printf("\n");
  //getchar();
  */

  for(size_t i=0;i<filters.size();i++) {
    if(!updated[i]) continue;
    lasttimes[i] = t;
    lastmousex[i] = x;
    lastmousey[i] = y;
  }
  return numUpdated != 0;
}

bool FreeformTaskPredictor::MouseInput(Real t,int x,int y)
{
  bool res=UpdateFilters(t,x,y);
  lastx=x;
  lasty=y;
  lastt=t;
  return res;
}

bool FreeformTaskPredictor::IdleInput(Real t)
{
  bool res=false;
  res = UpdateFilters(t,lastx,lasty);
  lastt = t;
  return res;
}

void FreeformTaskPredictor::UpdatePredictions(int horizon,bool getTrace)
{
  predGoal.resize(filters.size());
  predMouse.resize(filters.size());
  predMouseTrace.resize(filters.size());
  predGoalTrace.resize(filters.size());
  for(size_t i=0;i<filters.size();i++) {
    filterInfo[i].GetPredictions(filters[i],horizon,lastmousex[i],lastmousey[i],predMouse[i],predGoal[i]);
    if(getTrace) {
      predMouseTrace[i].resize(horizon);
      predGoalTrace[i].resize(horizon);
      if(horizon > 0)
	filterInfo[i].GetPredictions(filters[i],horizon,lastmousex[i],lastmousey[i],predMouseTrace[i],predGoalTrace[i]);
    }
  }

  Normalize(pfilter);
}

void FreeformTaskPredictor::PrintLinearModels()
{
  FatalError("Obsolete");
  /*
  //Printing the linear models
  for(size_t i=0;i<filters.size();i++) {
    cout<<filterInfo[i].name<<endl;
    cout<<"Observation:"<<endl;
    for(size_t j=0;j<filters[i].gmr_obs.regressions.size();j++) {
      const GaussianRegression& gr=filters[i].gmr_obs.regressions[j];
      cout<<"Component "<<j<<", weight "<<filters[i].gmr_obs.joint.phi[j]<<endl;
      Matrix temp;
      Vector vtemp;
      gr.GetLinearEquation(temp,vtemp);
      cout<<"A"<<endl;
      cout<<temp<<endl;
      cout<<"b"<<endl;
      cout<<vtemp<<endl;
      cout<<"err"<<endl;
      cout<<gr.ycov<<endl;
    }
    cout<<"Transition: "<<endl;
    for(size_t j=0;j<filters[i].gmr_trans.regressions.size();j++) {
      const GaussianRegression& gr=filters[i].gmr_trans.regressions[j];
      cout<<"Component "<<j<<", weight "<<filters[i].gmr_trans.joint.phi[j]<<endl;
      Matrix temp;
      Vector vtemp;
      gr.GetLinearEquation(temp,vtemp);
      cout<<"A"<<endl;
      cout<<temp<<endl;
      cout<<"b"<<endl;
      cout<<vtemp<<endl;
      cout<<"err"<<endl;
      cout<<gr.ycov<<endl;
    }
  }
  */
}

bool FreeformTaskPredictor::RunReachTrials(const char* infn,const char* outfn)
{
  vector<vector<Vector> > trialData;
  if(!ReadTestTrials(infn,trialData))
    return false;
  ofstream out(outfn,ios::out);
  if(!out) return false;
  out<<"trial,goal x,goal y,mouse x,mouse y,mouse dist,pred x,pred y,pred dist";
  for(size_t i=0;i<filterInfo.size();i++)
    out<<",P("<<filterInfo[i].name<<")";
  out<<endl;
  Real avgSpeedup=0.0;
  Real avgSpeedupFirst=0.0;
  int numMissed = 0;
  Real missDist = 0.0,missTime = 0.0;
  Real sumMouseError2=0.0,sumPredError2=0.0;
  for(size_t i=0;i<trialData.size();i++) {
    if(trialData[i].empty()) continue;
    Real mouseError2=0.0,predError2=0.0;
    Real firstHitTimeMouse=Inf,firstHitTimePred=Inf;
    Real lastMissTimeMouse=0.0,lastMissTimePred=0.0;
    Real minPredDistance = Inf;
    Real sumt = 0.0;
    Real oldt = 0.0;
    Real sumx=400,sumy=300;
    Reset(400,300);
    Real pmiss = 1.0;
    for(size_t j=0;j<trialData[i].size();j++) {
      assert(trialData[i][j].n == 6);
      Real time=trialData[i][j][0];
      Real dx=trialData[i][j][1];
      Real dy=trialData[i][j][2];
      Real gx=trialData[i][j][3];
      Real gy=trialData[i][j][4];
      Real gr=trialData[i][j][5];

      sumx+=dx;
      sumy+=dy;
      Real dt = time-oldt;
      Real r=Sqrt(Sqr(gx)+Sqr(gy));
      Real dist = Max(r-gr,0.0);
      if(r > gr)
	lastMissTimeMouse=time;
      else
	firstHitTimeMouse = Min(firstHitTimeMouse,time);
      out<<i<<","<<sumx+gx<<","<<sumy+gy<<","<<sumx<<","<<sumy<<","<<dist<<",";
      mouseError2 += dt*Sqr(dist);
      UpdateFilters(time,sumx,sumy);
      lastx = sumx;
      lasty = sumy;
      lastt = time;
      UpdatePredictions(0,false);
      Vector gmean;
      for(size_t f=0;f<filters.size();f++) {
	Vector fmean;
	predGoal[f].GetMean(fmean);
	if(f==0) gmean.mul(fmean,pfilter[0]);
	else gmean.madd(fmean,pfilter[f]);
      }
      Vector gtrue(2);
      r = Sqrt(Sqr(gmean[0]-gx-sumx)+Sqr(gmean[1]-gy-sumy));
      dist = Max(r-gr,0.0);

      out<<gmean[0]<<","<<gmean[1]<<","<<dist;
      for(size_t f=0;f<filters.size();f++) 
	out<<","<<pfilter[f];
      out<<endl;

      //compute hit probability
      /*
	gtrue(0) = gx;
	gtrue(1) = gy;
	Real pmiss_t = 1.0 - PHitSphere(goal,gtrue,gr);
	lastMissTimePred += pmiss*pmiss_t*dt;
	pmiss = pmiss*pmiss_t;
      */
      if(r > gr)
	lastMissTimePred = time;
      else
	firstHitTimePred = Min(firstHitTimePred,time);
      minPredDistance = Min(minPredDistance,r-gr);
      predError2 += dt*Sqr(dist);
      oldt = time;
      sumt = time;
    }
    if(sumt == 0.0) {
      printf("Trial %d skipped with time 0\n",i);
      continue;
    }
    mouseError2 /= sumt;
    predError2 /= sumt;
    sumMouseError2 += mouseError2;
    sumPredError2 += predError2;

    if(lastMissTimeMouse != sumt) {
      avgSpeedup += lastMissTimeMouse-lastMissTimePred;
      if(!IsInf(firstHitTimePred))
	avgSpeedupFirst += firstHitTimeMouse - firstHitTimePred;
    }
    else {
      numMissed ++;
      missDist += minPredDistance;
	  
      //keep on simulating until convergence?
      Real gx=trialData[i].back()[3];
      Real gy=trialData[i].back()[4];
      Real gr=trialData[i].back()[5];
      Real t = 0.0;
      while(t < 1.0) {
	t += filterInfo[0].timestep;
	UpdateFilters(t,sumx,sumy);
	lastx = sumx;
	lasty = sumy;
	lastt = t;
	UpdatePredictions(0,false);
	Vector gmean;
	for(size_t f=0;f<filters.size();f++) {
	  Vector fmean;
	  predGoal[f].GetMean(fmean);
	  if(f==0) gmean.mul(fmean,pfilter[0]);
	  else gmean.madd(fmean,pfilter[f]);
	}
	Real r = Sqrt(Sqr(gmean[0]-gx-sumx)+Sqr(gmean[1]-gy-sumy));
	if(r < gr) 
	  break;
      }
      missTime += t;
      printf("Trial %d missed by distance %g, converged in %g seconds\n",i,minPredDistance,t);
    }
  }
  out.close();
  printf("RMSE, mouse position %g\n",Sqrt(sumMouseError2/trialData.size()));
  printf("RMSE, predicted position %g\n",Sqrt(sumPredError2/trialData.size()));
  printf("Average miss time improvement %g\n",avgSpeedup/(trialData.size()-numMissed));
  printf("Average hit time improvement %g\n",avgSpeedupFirst/(trialData.size()-numMissed));
  printf("%d missed, miss distance %g, settle time %g\n",numMissed,missDist/numMissed,missTime/numMissed);
  return true;
}

bool FreeformTaskPredictor::RunTrajTrials(const char* infn,const char* outfn)
{
  vector<vector<Vector> > trialData;
  if(!ReadTestTrials(infn,trialData)) {
    printf("Unable to read test data file %s\n",infn);
    return false;
  }
  const static int pred_step [] = {10,20,30,40};
  ofstream out(outfn,ios::out);
  if(!out) return false;
  out<<"trial,mouse x,mouse y,goal x,goal y,pred x,pred y,pred x 10,pred y 10,pred x 20,pred y 20,pred x 30,pred y 30,pred x 40,pred y 40";
  for(size_t i=0;i<filterInfo.size();i++)
    out<<",P("<<filterInfo[i].name<<")";
  out<<endl;
  Real sumMouseError2=0.0,sumPredError2=0.0;
  Real sumTimes = 0.0;
  for(size_t i=0;i<trialData.size();i++) {
    if(trialData[i].empty()) continue;
    Real mouseError2=0.0,predError2=0.0;
    Real sumt = 0.0;
    Real oldt = 0.0;
    Real sumx=400.0,sumy=300.0;
    Reset(400,300);
    for(size_t j=0;j<trialData[i].size();j++) {
      assert(trialData[i][j].n == 6);
      Real time=trialData[i][j][0];
      Real dx=trialData[i][j][1];
      Real dy=trialData[i][j][2];
      Real gx=trialData[i][j][3];
      Real gy=trialData[i][j][4];

      sumx += dx;
      sumy += dy;
      Real dt = time-oldt;
      Real r=Sqrt(Sqr(gx)+Sqr(gy));
      mouseError2 += dt*Sqr(r);
      out<<i<<","<<sumx<<","<<sumy<<","<<sumx+gx<<","<<sumy+gy;
      UpdateFilters(time,sumx,sumy);
      lastx = sumx;
      lasty = sumy;
      lastt = time;
      UpdatePredictions(0,false);
      Vector gmean;
      for(size_t f=0;f<filters.size();f++) {
	Vector fmean;
	predGoal[f].GetMean(fmean);
	if(f==0) gmean.mul(fmean,pfilter[0]);
	else gmean.madd(fmean,pfilter[f]);
      }
      r = Sqrt(Sqr(gmean[0]-gx-sumx)+Sqr(gmean[1]-gy-sumy));
      out<<","<<gmean[0]<<","<<gmean[1];
      predError2 += dt*Sqr(r);

      for(int k=0;k<4;k++) {
	UpdatePredictions(pred_step[k],false);
	for(size_t f=0;f<filters.size();f++) {
	  Vector fmean;
	  predGoal[f].GetMean(fmean);
	  if(f==0) gmean.mul(fmean,pfilter[0]);
	  else gmean.madd(fmean,pfilter[f]);
	}
	out<<","<<gmean[0]<<","<<gmean[1];
      }
      for(size_t f=0;f<filters.size();f++) 
	out<<","<<pfilter[f];
      out<<endl;

      oldt = time;
      sumt += dt;
    }
    if(sumt == 0.0) {
      printf("Trial %d skipped with time 0\n",i);
      continue;
    }
    //mouseError2 /= sumt;
    //predError2 /= sumt;
    sumMouseError2 += mouseError2;
    sumPredError2 += predError2;
    sumTimes += sumt;
    printf("Trial %d mouse RMSE %g pred RMSE %g\n",i,Sqrt(mouseError2/sumt),Sqrt(predError2/sumt));
  }
  out.close();
  /*
    printf("Mean squared error, mouse position %g\n",Sqrt(sumMouseError2/trialData.size()));
    printf("Mean squared error, predicted position %g\n",Sqrt(sumPredError2/trialData.size()));
  */
  printf("Mean squared error, mouse position %g\n",Sqrt(sumMouseError2/sumTimes));
  printf("Mean squared error, predicted position %g\n",Sqrt(sumPredError2/sumTimes));
  return true;
}

