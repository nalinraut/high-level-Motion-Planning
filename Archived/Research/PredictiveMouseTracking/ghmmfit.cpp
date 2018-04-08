#include <statistics/GaussianHMM.h>
#include <statistics/KMeans.h>
#include <math/matrix.h>
#include <math/random.h>
#include <math/indexing.h>
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

bool WriteGHMM(const GaussianHMM& ghmm,const char* fn)
{
  //output GHMM
  ofstream out(fn,ios::out);
  out<<ghmm;
  out.close();
}

int RunGHMMFit(const char* in,const char* trialLabel,int k,int numIters,const char* outfn)
{
  vector<vector<string> > entries;
  if(!ReadCSV(in,entries)) {
    printf("Error reading CSV file %s\n",in);
    return 1;
  }
  if(entries.empty()) {
    printf("CSV file %s is empty\n",in);
    return 1;
  }
  if(k > entries.size()) {
    printf("CSV file %s has fewer entries than k=%d\n",in,k);
    return 1;
  }
  printf("Done reading %s\n",in);
  vector<string> labels = entries[0];
  vector<Vector> examples(entries.size()-1);
  for(size_t i=1;i<entries.size();i++) {
    examples[i-1].resize(entries[i].size());
    for(size_t j=0;j<entries[i].size();j++) {
      stringstream ss(entries[i][j]);
      ss >> examples[i-1][j];
      if(ss.bad()) {
	printf("Error: non numeric value on line %d, column %d\n",i,j);
	return 1;
      }
    }
  }

  //TODO: select the trial index dynamically using the trialLabel name
  int trialIndex = 0;
  vector<vector<Vector> > trials;
  int curTrial=-1;
  for(size_t i=0;i<examples.size();i++) {
    if(examples[i][trialIndex] != curTrial) {
      curTrial = examples[i][trialIndex];
      trials.resize(trials.size()+1);
    }
    trials.back().push_back(examples[i]);
  }

  //now take out the trial variable from examples and trials
  vector<int> trialIndices(1,trialIndex);
  for(size_t i=0;i<examples.size();i++)
    RemoveElements(examples[i],trialIndices);
  for(size_t i=0;i<trials.size();i++)
    for(size_t j=0;j<trials[i].size();j++)
      RemoveElements(trials[i][j],trialIndices);

  //scale/translate
  /*
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
  //scale to range [-1,1]
  for(size_t i=0;i<examples.size();i++) {
    examples[i] -= mean;
    for(int j=0;j<emin.n;j++)
      examples[i](j) /= stddev(j);
  }
  */

  printf("Starting k-means initialization\n");
  int maxKmeansIters=numIters;
  KMeans kmeans(examples,k);
  kmeans.RandomInitialCenters();
  kmeans.Iterate(maxKmeansIters);
  kmeans.CalcLabelsFromCenters();
  printf("Kmeans converged in %d/%d iterations\n",maxKmeansIters,numIters);
  vector<vector<Vector> > clusters(k);
  for(size_t i=0;i<kmeans.labels.size();i++)
    clusters[kmeans.labels[i]].push_back(examples[i]);
  GaussianMixtureModel gmm(k,labels.size());
  printf("Cluster sizes: ");
  for(int i=0;i<k;i++) {
    printf("%d ",clusters[i].size());
    if(clusters[i].size() < labels.size()+1) {
      printf("Cluster %d is too small, size %d\n",i,clusters[i].size());
    }
    gmm.gaussians[i].setMaximumLikelihood(clusters[i]);
    gmm.phi[i] = Real(clusters[i].size())/examples.size();
  }
  printf("\n");

  printf("Training EM...\n");
  GaussianHMM ghmm(gmm);
  Real tol=1e-1;
  ghmm.TrainEM(trials,tol,numIters,1);

  Real lltotal=0.0;
  for(size_t i=0;i<trials.size();i++)
    lltotal += ghmm.LogLikelihood(trials[i]);
  printf("Log likelihood of data: %g\n",lltotal);

  printf("Writing GHMM to %s",outfn);
  WriteGHMM(ghmm,outfn);
}

int main(int argc,const char** argv)
{
  Srand(time(NULL));
  if(argc < 4) {
    printf("Usage: %s in.csv trialLabel k [numIters] [outfile]\n",argv[0]);
    return 0;
  }
  const char* in = argv[1];
  const char* trialLabel = argv[2];
  int k = atoi(argv[3]);
  int numIters = 200;
  const char* out="out.ghmm";
  if(argc >= 5) numIters = atoi(argv[4]);
  if(argc >= 6) out=argv[5];

  return RunGHMMFit(in,trialLabel,k,numIters,out);
}
