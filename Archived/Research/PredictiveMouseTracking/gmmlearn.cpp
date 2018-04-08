#include <statistics/GaussianMixtureModel.h>
#include <statistics/KMeans.h>
#include <math/matrix.h>
#include <math/random.h>
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

bool WriteGMM(const GaussianMixtureModel& gmm,const char* fn)
{
  //output GMM
  ofstream out(fn,ios::out);
  out<<gmm;
  out.close();
}

Real LearnGMM(GaussianMixtureModel& gmm,const vector<Vector>& examples,int numIters)
{
  assert(!examples.empty());

  int k=(int)gmm.phi.size();
  int maxKmeansIters=numIters/5;
  printf("Starting k-means initialization, %d examples\n",examples.size());
  KMeans kmeans(examples,k);
  kmeans.RandomInitialCenters();
  kmeans.Iterate(maxKmeansIters);
  kmeans.CalcLabelsFromCenters();
  printf("Kmeans converged in %d/%d iterations\n",maxKmeansIters,numIters/5);
  vector<vector<Vector> > clusters(k);
  for(size_t i=0;i<kmeans.labels.size();i++) {
    clusters[kmeans.labels[i]].push_back(examples[i]);
  }
  for(int i=0;i<k;i++) {
    if(clusters[i].size() < examples[0].size()+1) {
      printf("Cluster %d is too small, size %d\n",i,clusters[i].size());
    }
    gmm.gaussians[i].setMaximumLikelihood(clusters[i]);
    gmm.phi[i] = Real(clusters[i].size())/examples.size();
  }
  
  printf("Training GMM\n");
  Real tol=1e-1;
  int verbose=0;
  gmm.TrainEM(examples,tol,numIters,verbose);
}

Real GMMCrossValidation(const vector<Vector>& examples,int k,int numIters,int cross_validate_folds)
{
  Real sumll = 0.0;
  vector<Vector> training,testing;
  for(int fold=0;fold<cross_validate_folds;fold++) {
    training.resize(0);
    testing.resize(0);
    training.insert(training.end(),examples.begin(),examples.begin()+fold*examples.size()/cross_validate_folds);
    testing.insert(testing.end(),examples.begin()+fold*examples.size()/cross_validate_folds,examples.begin()+(fold+1)*examples.size()/cross_validate_folds);
    training.insert(training.end(),examples.begin()+(fold+1)*examples.size()/cross_validate_folds,examples.end());

    printf("Fold %d, k=%d\n",fold,k);
    printf("Training size %d, testing size %d\n",training.size(),testing.size());
    GaussianMixtureModel gmm(k,examples[0].size());
    LearnGMM(gmm,training,numIters);
    Real ll = gmm.LogLikelihood(testing);
    sumll += ll;    
    printf("Log likelihood %g\n",ll);
  }
  return sumll;
}


int RunGMMLearn(const char* in,int numIters,const char* outfn,int cross_validate_folds=5,bool whiten=false)
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

  //scale/translate
  if(whiten) {
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
  }

  int paramspergaussian = labels.size()+labels.size()*(labels.size()+1)/2;
  paramspergaussian *= 10;
  printf("Exploring mixtures of up to %d components\n",(examples.size()*(cross_validate_folds-1))/(cross_validate_folds*paramspergaussian));

  int k=1;
  //int k=10;
  int kbest=1;
  Real bestML=-Inf;
  ofstream out("learning_curve.txt",ios::out|ios::app);
  out<<in<<endl;
  out.close();
  while(k*paramspergaussian<examples.size()*(cross_validate_folds-1)/cross_validate_folds) {
    Real accuracy = GMMCrossValidation(examples,k,numIters,cross_validate_folds);
    ofstream out("learning_curve.txt",ios::out|ios::app);
    out<<k<<","<<accuracy<<endl;
    out.close();
    if(accuracy < bestML) {
      bestML = accuracy;
      kbest = k;
    }
    k*=3;
    k+=1;
    k/=2;
    //k += 1;
  }
  GaussianMixtureModel gmm(kbest,labels.size());
  LearnGMM(gmm,examples,numIters);
  printf("Writing GMM to %s",outfn);
  WriteGMM(gmm,outfn);
}

int main(int argc,const char** argv)
{
  Srand(time(NULL));
  if(argc < 2) {
    printf("Usage: %s in.csv [numIters] [outfile]\n",argv[0]);
    return 0;
  }
  const char* in = argv[1];
  int numIters = 200;
  const char* out="out.gmm";
  if(argc >= 3) numIters = atoi(argv[2]);
  if(argc >= 4) out=argv[3];

  return RunGMMLearn(in,numIters,out);
}
