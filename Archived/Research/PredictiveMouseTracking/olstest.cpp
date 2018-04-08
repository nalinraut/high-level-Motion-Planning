#include <math/matrix.h>
#include <math/indexing.h>
#include <statistics/LinearModel.h>
#include <time.h>
#include <fstream>
#include <sstream>
#include <vector>
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

vector<Vector> TrialsToHistories(const vector<Vector>& examples,int trialIndex,const vector<int>& indepIndices,int historyLen)
{
  vector<Vector> histExamples(examples.size());
  Vector hist(indepIndices.size()*historyLen);
  int curtrial = -1;
  for(size_t i=0;i<examples.size();i++) {
    if(curtrial != examples[i][trialIndex]) {
      //new trial, copy example into there (or set to zero?)
      for(size_t k=0;k<indepIndices.size();k++)
	hist[k] = examples[i][indepIndices[k]];
      Vector sub;
      sub.setRef(hist,0,1,indepIndices.size());
      for(int d=1;d<historyLen;d++)
	hist.copySubVector(d*indepIndices.size(),sub);
      //TEMP?
      //hist.setZero();
      curtrial = examples[i][trialIndex];
    }
    else {
      //shift history
      Vector sub;
      sub.setRef(hist,indepIndices.size());
      hist.copySubVector(0,sub);
    }
    //put indep indices into last slot of hist
    for(size_t k=0;k<indepIndices.size();k++)
      hist[indepIndices.size()*(historyLen-1)+k] = examples[i][indepIndices[k]];
    histExamples[i] = hist;
  }
  return histExamples;
}

int RunOLSTest(const char* train,const char* test,const vector<string>& dependentVariables,int historyLen)
{
  vector<vector<string> > entries;
  if(!ReadCSV(train,entries)) {
    printf("Error reading CSV file %s\n",train);
    return 1;
  }
  if(entries.empty()) {
    printf("CSV file %s is empty\n",train);
    return 1;
  }
  printf("Done reading %s\n",train);
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
  int trialindex=-1;
  vector<int> depindices,indepindices;
  for(size_t i=0;i<depmap.size();i++){
    if(depmap[i] < 0) {
      if(labels[i] == "trial")
	trialindex = i;
      else
	indepindices.push_back(i);
    }
    else
      depindices.push_back(i);
  }
  assert(trialindex >= 0);

  //train linear models
  vector<LinearModel> ols(depindices.size());
  vector<Vector> histories;
  printf("Copying histories\n");
  histories=TrialsToHistories(examples,trialindex,indepindices,historyLen);
  printf("Training...\n");
  for(size_t d=0;d<depindices.size();d++) {
    vector<Real> outcomes(examples.size());
    for(size_t i=0;i<examples.size();i++) 
      outcomes[i] = examples[i][depindices[d]];
    printf("Least squares solve...\n");
    bool res=ols[d].LeastSquares(histories,outcomes);
    if(!res) {
      printf("Error solving for dep variable",labels[depindices[d]].c_str());
      return 1;
    }
    cout<<"Coeffs for "<<labels[depindices[d]]<<":"<<endl;
    cout<<ols[d].coeffs<<endl;
  }

  printf("Testing...\n");
  //load test entries
  vector<vector<string> > test_entries;
  if(!ReadCSV(test,test_entries)) {
    printf("Error reading CSV file %s\n",train);
    return 1;
  }
  vector<Vector> test_examples(test_entries.size()-1);
  for(size_t i=1;i<test_entries.size();i++) {
    test_examples[i-1].resize(test_entries[i].size());
    for(size_t j=0;j<test_entries[i].size();j++) {
      stringstream ss(test_entries[i][j]);
      ss >> test_examples[i-1][j];
      if(ss.bad()) {
	printf("Error: non numeric value on line %d, column %d\n",i,j);
	return 1;
      }
    }
  }
  printf("Saving predictions to predictions.csv\n");
  ofstream out("predictions.csv");
  for(size_t i=0;i<depindices.size();i++)
    out<<labels[depindices[i]]<<",";
  out<<endl;
  vector<Vector> test_histories=TrialsToHistories(test_examples,trialindex,indepindices,historyLen);
  Real sqerr = 0.0;
  for(size_t i=0;i<test_histories.size();i++) {
    for(size_t d=0;d<depindices.size();d++) {
      Real res=ols[d].Evaluate(test_histories[i]);
      out<<res<<",";
      sqerr += Sqr(res-test_examples[i][depindices[d]]);
    }
    out<<endl;
  }
  printf("Mean squared error: %g\n",sqerr/test_histories.size());
  out.close();
}

int main(int argc,const char** argv)
{
  if(argc < 5) {
    printf("Usage: %s train.csv test.csv h dep1 dep2 ...\n",argv[0]);
    return 0;
  }
  const char* trainfn = argv[1];
  const char* testfn = argv[2];
  int hist = atoi(argv[3]);
  vector<string> deps;
  for(int i=4;i<argc;i++)
    deps.push_back(argv[i]);

  return RunOLSTest(trainfn,testfn,deps,hist);
}
