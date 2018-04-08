#include "GMRFilter.h"
#include <time.h>
#include <fstream>
#include <sstream>
using namespace std;
using namespace Statistics;

bool ReadCSV(const char* fn,vector<vector<string> >& array,char delim=',')
{
  array.resize(0);
  string line;
  int pos;
 
  ifstream in(fn,ios::in);
  if(!in) { return false;   }
  while( getline(in,line) ) {
    vector<string> ln;
    while( (pos = line.find(delim)) >= 0)
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

bool ReadGMM(GaussianMixtureModel& gmm,const char* fn)
{
  //input GMM
  ifstream in(fn,ios::in);
  in>>gmm;
  if(!in) return false;
  in.close();
  return true;
}


//given a gmm model, and a csv trial
int RunGMRFilter(const char* gmmfn,const char* data,const string& trialvar,const vector<vector<string> >& historyVars)
{
  GaussianMixtureModel gmm;
  if(!ReadGMM(gmm,gmmfn)) {
    printf("Error reading gmm file %s\n",gmmfn);
    return 1;
  }
  vector<vector<string> > entries;
  if(!ReadCSV(data,entries)) {
    printf("Error reading CSV file %s\n",data);
    return 1;
  }
  if(entries.empty()) {
    printf("CSV file %s is empty\n",data);
    return 1;
  }
  printf("Done reading %s\n",data);

  vector<string> labels = entries[0];
  if((int)labels.size()-1 != gmm.gaussians[0].mu.n) {
    printf("Data does not match the size of the GMM\n");
    return 1;
  }
  //segment out trials
  int trialIndex=-1;
  for(size_t i=0;i<labels.size();i++)
    if(labels[i] == trialvar) {
      trialIndex = (int)i;
      break;
    }
  if(trialIndex < 0) {
    printf("Trial variable %s not found in dataset\n",trialvar.c_str());
    return 1;
  }
  labels.erase(labels.begin()+trialIndex);

  vector<pair<int,int> > trials;
  trials.push_back(pair<int,int>(0,entries.size()-1));
  for(size_t i=1;i<entries.size();i++) {
    if(entries[i][trialIndex] != entries[trials.back().first+1][trialIndex]) {
      trials.back().second = (int)i-1;
      trials.push_back(pair<int,int>(i-1,entries.size()-1));
    }
  }
  printf("Parsed out %d trials\n",trials.size());

  vector<Vector> examples(entries.size()-1);
  for(size_t i=1;i<entries.size();i++) {
    //drop the trial index variable
    examples[i-1].resize(entries[i].size()-1);
    for(size_t j=0;j<entries[i].size();j++) {
      size_t k= ((int)j > trialIndex? j-1:j);
      stringstream ss(entries[i][j]);
      ss >> examples[i-1][k];
      if(ss.bad()) {
	printf("Error: non numeric value on line %d, column %d\n",i,j);
	return 1;
      }
    }
  }

  /*  
  //scale/translate
  Vector emin=examples[0],emax=examples[0];
  Vector mean=examples[0],stddev(emin.n,Zero);
  for(size_t i=1;i<examples.size();i++) {
    for(int j=0;j<emin.n;j++)
      if(examples[i](j) < emin(j)) emin(j)=examples[i](j);
      else if(examples[i](j) > emax(j)) emax(j)=examples[i](j);
    mean += examples[i];
  }
  mean /= examples.size();
  for(size_t i=0;i<examples.size();i++) {
    for(int j=0;j<emin.n;j++)
      stddev(j) += Sqr(examples[i](j) - mean(j));
  }
  stddev /= examples.size();
  for(int j=0;j<emin.n;j++)
    stddev(j) = Sqrt(stddev(j));
  */
  /*
  printf("Ranges:\n");
  for(size_t i=0;i<labels.size();i++)
    cout<<"  "<<labels[i]<<": "<<emax(i)-emin(i)<<endl;
  printf("Midpoints:\n");
  for(size_t i=0;i<labels.size();i++)
    cout<<"  "<<labels[i]<<": "<<(emax(i)+emin(i))*0.5<<endl;
  printf("Stddevs:\n");
  for(size_t i=0;i<labels.size();i++)
    cout<<"  "<<labels[i]<<": "<<stddev(i)<<endl;
  printf("Means:\n");
  for(size_t i=0;i<labels.size();i++)
    cout<<"  "<<labels[i]<<": "<<mean(i)<<endl;
  */
  /*
  //scale to range [-1,1]
  for(size_t i=0;i<examples.size();i++) {
    examples[i] -= mean;
    for(int j=0;j<emin.n;j++)
      examples[i](j) /= stddev(j);
  }
  */

  GMRFilter filter;
  filter.obsgmm = gmm;  
  for(size_t i=0;i<historyVars[0].size();i++) {
    int index=-1;
    for(size_t j=0;j<labels.size();j++)
      if(labels[j] == historyVars[0][i]) {
	index = (int)j;
	break;
      }
    assert(index >= 0);
    filter.obsIndices.push_back(index);
  }
  for(size_t i=1;i<historyVars.size();i++) {
    for(size_t j=0;j<historyVars[i].size();j++) {
      int index=-1;
      for(size_t k=0;k<labels.size();k++)
	if(labels[k] == historyVars[i][j]) {
	  index = (int)k;
	  break;
	}
      if(index < 0) {
	printf("Couldnt find potential history variable \"%s\"\n",historyVars[i][j].c_str());
	break;
      }
      filter.historyIndices.resize(i);
      filter.historyIndices[i-1].push_back(index);
    }
  }
  if(filter.historyIndices.empty()) {
    printf("Couldn't find any history variables in dataset\n");
    for(size_t i=0;i<labels.size();i++)
      printf("  \"%s\"\n",labels[i].c_str());
    return 1;
  }
  //put all the other indices into hidden
  vector<bool> used(labels.size(),false);
  for(size_t i=0;i<filter.obsIndices.size();i++)
    used[filter.obsIndices[i]]=true;
  for(size_t i=0;i<filter.historyIndices.size();i++)
    for(size_t j=0;j<filter.historyIndices[i].size();j++)
      used[filter.historyIndices[i][j]]=true;
  for(size_t i=0;i<labels.size();i++)
    if(!used[i]) filter.hiddenIndices.push_back(i);
  filter.Init();  

  //run the filter
  const char* outfn = "filter_out.csv";
  printf("Writing predictions to %s\n",outfn);
  ofstream out(outfn,ios::out);
  out<<"trial";
  for(size_t i=0;i<filter.hiddenIndices.size();i++) 
    out<<","<<labels[filter.hiddenIndices[i]];
  for(size_t i=0;i<filter.hiddenIndices.size();i++) 
    out<<","<<"Var("<<labels[filter.hiddenIndices[i]]+")";
  out<<endl;
  GaussianMixtureModel gmm_hidden;
  for(size_t i=0;i<trials.size();i++) {
    filter.Reset();
    for(int a=trials[i].first;a<trials[i].second;a++) {
      //use observation from trial a to update filter
      Vector obs(filter.obsIndices.size());
      for(size_t j=0;j<filter.obsIndices.size();j++)
	obs(j) = examples[a][filter.obsIndices[j]];
      filter.Predict();
      filter.Update(obs);

      //measure accuracy of filter
      filter.GetHiddenDistribution(gmm_hidden);
      Vector hidden_pred;
      gmm_hidden.GetMean(hidden_pred);
      Matrix hiddenCov;
      Vector hiddenVar;
      gmm_hidden.GetCovariance(hiddenCov);
      gmm_hidden.GetVariance(hiddenVar);
      for(int j=0;j<hiddenVar.n;j++) {
	if(!FuzzyEquals(hiddenVar(j),hiddenCov(j,j))) {
	  printf("Warning, error in GetVariance? %g != %g\n",hiddenVar(j),hiddenCov(j,j));
	}
      }

      //rescale
      for(int j=0;j<hidden_pred.n;j++) {
	hidden_pred(j) = hidden_pred(j)*stddev(filter.hiddenIndices[j])+mean(filter.hiddenIndices[j]);
	hiddenVar(j) *= Sqr(stddev(filter.hiddenIndices[j]));
      }
      //spit out line
      out<<i+1;
      for(int j=0;j<hiddenVar.n;j++) 
	out<<","<<hidden_pred(j);
      for(int j=0;j<hiddenVar.n;j++) 
	out<<","<<hiddenVar(j);
      out<<endl;
    }
  }
  out.close();
}

int main(int argc,const char** argv)
{
  if(argc < 4) {
    printf("Usage: %s gmm trialvar data.csv\n",argv[0]);
    return 0;
  }
  const char* gmmfn = argv[1];
  const char* trialvar = argv[2];
  const char* datafn = argv[3];
  vector<vector<string> > historyvars;
  historyvars.resize(3);
  historyvars[0].resize(2);
  historyvars[1].resize(2);
  historyvars[2].resize(2);
  historyvars[0][0] = "widget dx [-1]";
  historyvars[0][1] = "widget dy [-1]";
  historyvars[1][0] = "widget dx [-2]";
  historyvars[1][1] = "widget dy [-2]";
  historyvars[2][0] = "widget dx [-3]";
  historyvars[2][1] = "widget dy [-3]";
  return RunGMRFilter(gmmfn,datafn,trialvar,historyvars);
}
