#include <statistics/GaussianMixtureModel.h>
#include <math/matrix.h>
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

int RunGMRTest(const char* gmmfn,const char* data,const vector<string>& dependentVariables)
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
  if((int)labels.size() != gmm.gaussians[0].mu.n) {
    printf("Data does not match the size of the GMM\n");
    return 1;
  }

  //parse out dependent/independent variables
  vector<int> depmap(labels.size(),-1);
  for(size_t i=0;i<dependentVariables.size();i++) {
    for(size_t j=0;j<labels.size();j++) {
      if(labels[j] == dependentVariables[i]) {
	//duplicate dependent variables?
	assert(depmap[j] < 0);
	depmap[j] = (int)i;
	break;
      }
    }
  }
  vector<int> depindices,indepindices;
  for(size_t i=0;i<depmap.size();i++){
    if(depmap[i] < 0)
      indepindices.push_back(i);
    else
      depindices.push_back(i);
  }

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

  Vector ysqerr(depindices.size(),Zero),yavgerr(depindices.size(),Zero);
  Real ll=Zero;
  GaussianMixtureRegression gmr(gmm);
  gmr.SetXIndices(indepindices);
  assert(depindices == gmr.yindices);
  assert(indepindices == gmr.xindices);
  GaussianMixtureModel ygmm;
  for(size_t i=0;i<examples.size();i++) {
    Vector xsub(indepindices.size()),ysub(depindices.size());
    GetElements(examples[i],indepindices,xsub);
    GetElements(examples[i],depindices,ysub);

    //predict
    gmr.GetY(xsub,ygmm);
    //evaluate error
    ll += Log(ygmm.Probability(ysub));
    Vector ymean(depindices.size(),Zero);
    Real phisum = Zero;
    for(size_t j=0;j<ygmm.gaussians.size();j++) {
      ymean.madd(ygmm.gaussians[j].mu,ygmm.phi[j]);
      phisum += ygmm.phi[j];
    }
    ymean /= phisum;
    for(int j=0;j<ysqerr.n;j++) {
      ysqerr(j) += Sqr(ymean(j)-ysub(j));
      yavgerr(j) += ysub(j)-ymean(j);
    }
  }
  /*
  for(int j=0;j<ysqerr.n;j++) {
    ysqerr(j) *= Sqr(stddev(j));
    yavgerr(j) *= stddev(j);
  }
  */
  ysqerr /= examples.size();
  yavgerr /= examples.size();

  cout<<"Unconditioned standard errors:"<<endl;
  for(size_t i=0;i<depindices.size();i++) {
    Real val=Zero;
    for(size_t j=0;j<examples.size();j++) {
      Real mean=0;
      Real phisum=0;
      for(size_t k=0;k<gmm.gaussians.size();k++) {
	mean += gmm.gaussians[k].mu(depindices[i]);
	phisum += gmm.phi[k];
      }
      mean /= phisum;
      val += Sqr(examples[j][depindices[i]]-mean);
    }
    //val = Sqrt(val*Sqr(stddev(depindices[i]))/examples.size());
    val = Sqrt(val/examples.size());
    cout<<"  "<<labels[depindices[i]]<<": "<<val<<endl;
  }

  cout<<"Standard errors:"<<endl;
  for(size_t i=0;i<depindices.size();i++)
    cout<<"  "<<labels[depindices[i]]<<": "<<Sqrt(ysqerr(i))<<endl;
  cout<<"Average errors:"<<endl;
  for(size_t i=0;i<depindices.size();i++)
    cout<<"  "<<labels[depindices[i]]<<": "<<yavgerr(i)<<endl;
  cout<<"Log likelihood: "<<ll<<endl;
}

int main(int argc,const char** argv)
{
  if(argc < 4) {
    printf("Usage: %s gmm data.csv dep1 dep2 ...\n",argv[0]);
    return 0;
  }
  const char* gmmfn = argv[1];
  const char* datafn = argv[2];
  vector<string> deps;
  for(int i=3;i<argc;i++)
    deps.push_back(argv[i]);

  return RunGMRTest(gmmfn,datafn,deps);
}
