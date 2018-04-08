#include <statistics/LinearProcessHMM.h>
#include <statistics/KMeans.h>
#include <math/matrix.h>
#include <math/random.h>
#include <math/indexing.h>
#include <string.h>
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

bool WriteLPHMM(const LinearProcessHMM& lphmm,const char* fn)
{
  //output LPHMM
  ofstream out(fn,ios::out);
  out<<lphmm;
  out.close();
}

int RunLPHMMFit(const char* in,const char* trialLabel,int k,int numIters,const char* outfn, bool whiten=true)
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

  //scale/translate
  Vector translate(examples[0].n,0.0),scale(examples[0].n,1.0);
  if(whiten) {
    Vector emin=examples[0],emax=examples[0];
    Vector mean=examples[0];
    for(size_t i=1;i<examples.size();i++) {
      for(int j=0;j<emin.n;j++)
	if(examples[i](j) < emin(j)) emin(j)=examples[i](j);
	else if(examples[i](j) > emax(j)) emax(j)=examples[i](j);
      mean += examples[i];
    }
    mean /= examples.size();
    Vector stddev(mean.n,Zero);
    for(size_t i=0;i<examples.size();i++) {
      for(int j=0;j<emin.n;j++)
	stddev(j) += Sqr(examples[i](j) - mean(j));
    }
    stddev /= examples.size();
    for(int j=0;j<emin.n;j++)
      stddev(j) = Sqrt(stddev(j));

    //don't rescale trial index
    mean[trialIndex] = 0;
    stddev[trialIndex] = 1.0;

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
    translate.setNegative(mean);
    for(int j=0;j<emin.n;j++)
      scale(j) = 1.0/stddev(j);
    //scale to range [-1,1]
    for(size_t i=0;i<examples.size();i++) {
      examples[i] -= mean;
      for(int j=0;j<emin.n;j++)
	examples[i](j) /= stddev(j);
    }
  }

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
  RemoveElements(translate,trialIndices);
  RemoveElements(scale,trialIndices);

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
  LinearProcessHMM lphmm(gmm);
  Real tol=1e-1;
  lphmm.TrainEM(trials,tol,numIters,1);

  Real lltotal=0.0;
  for(size_t i=0;i<trials.size();i++)
    lltotal += lphmm.LogLikelihood(trials[i]);
  printf("Log likelihood of data: %g\n",lltotal);

  if(whiten) {
    //scale and translate out
    printf("Scaling continuous priors\n");
    for(size_t i=0;i<lphmm.continuousPriors.size();i++) {
      for(int j=0;j<translate.n;j++) {
	printf("Undoing scaling %s by %g, translating by %g\n",labels[j+1].c_str(),scale(j),translate(j));
	printf("Original mu: %g\n",lphmm.continuousPriors[i].mu(j));
	lphmm.continuousPriors[i].mu(j) = lphmm.continuousPriors[i].mu(j)/scale(j)-translate(j);
	printf("New mu: %g\n",lphmm.continuousPriors[i].mu(j));
	Vector vtemp;
	lphmm.continuousPriors[i].L.getColRef(j,vtemp);
	vtemp /= scale(j);
      }
    }
    printf("Scaling emission models\n");
    for(size_t i=0;i<lphmm.emissionModels.size();i++) {
      //x = (y+translate)*scale
      //y = x/scale - translate
      //x' = Ai*x + e, e~N(bi,Si)
      //x' = Ai*(y+translate)*scale + e 
      //y' = x'/scale - translate = [(Ai*(y+translate)*scale+e]/scale - translate
      //   = Ai*y + Ai*translate + e/scale - translate = Ai*y + (Ai-I)*translate + e/scale
      //e' ~= N((Ai-I)*translate+bi/scale,Si/scale^2)
      Vector Atranslate;
      lphmm.emissionModels[i].A.mul(translate,Atranslate);
      for(int j=0;j<translate.n;j++) {
	printf("Undoing scaling %s by %g, translating by %g\n",labels[j+1].c_str(),scale(j),translate(j));
	printf("Original error mean: %g\n",lphmm.emissionModels[i].error.mu(j));
	Real newmean = Atranslate[j]-translate(j)+lphmm.emissionModels[i].error.mu(j)/scale(j);
	lphmm.emissionModels[i].error.mu(j) = newmean;
	printf("New error mean: %g\n",lphmm.emissionModels[i].error.mu(j));
      }
      Matrix cov;
      lphmm.emissionModels[i].error.getCovariance(cov);
      for(int j=0;j<translate.n;j++)
	for(int k=0;k<translate.n;k++)
	  cov(j,k) /= scale(j)*scale(k);
      bool res=lphmm.emissionModels[i].error.setCovariance(cov);
      Assert(res==true);
    }
  }

  printf("Writing LPHMM to %s",outfn);
  WriteLPHMM(lphmm,outfn);
}

const char* OPTIONS_STRING = "Options:\
\t-w: whitens the data before running Kmeans / EM\n\
\t-n iters: number of iterations (default: 200)\n\
\t-o file: set output file (default: out.gmm)\n\
\t-t label: set the trial label (default: none)\n\
";


int main(int argc,const char** argv)
{
  Srand(time(NULL));
  if(argc < 4) {
    printf("Usage: %s in.csv k [options]\n",argv[0]);
    return 0;
  }
  const char* in = argv[1];
  int k = atoi(argv[2]);
  const char* trialLabel = "trial";
  bool whiten = false;
  int numIters = 200;
  const char* out="out.lphmm";

  for(int i=3;i<argc;i++) {
    if(argv[i][0] == '-') {
      if(0==strcmp(argv[i],"-n")) {
	numIters = atoi(argv[i+1]);
	i++;
      }
      else if(0==strcmp(argv[i],"-w")) {
	whiten = true;
      }
      else if(0==strcmp(argv[i],"-o")) {
	out=argv[i+1];
	i++;
      }
      else if(0==strcmp(argv[i],"-t")) {
	trialLabel = argv[i+1];
	i++;
      }
      else {
	printf("Invalid option %s\n",argv[i]);
	printf(OPTIONS_STRING);
	return 1;
      }
    }
    else {
      printf("Invalid extra argument %s\n",argv[i]);
      printf("Usage: %s in.csv k [options]\n",argv[0]);
      return 1;
    }
  }
  return RunLPHMMFit(in,trialLabel,k,numIters,out,whiten);
}

