#include <graph/UndirectedGraph.h>
#include <graph/ConnectedComponents.h>
#include <graph/IO.h>
#include <set>
#include <math/math.h>
#include <utils/StatCollector.h>
#include <getopt.h>
#include <string.h>
#include <fstream>
using namespace std;
using namespace Math;

inline Real AlphaBetaObj(const pair<Real,Real>& ab) { return Log(ab.first)+Log(ab.second); }

template<class Set1Iter, class Set2Iter> 
bool set_disjoint(Set1Iter it1, Set1Iter it1End, Set2Iter it2, Set2Iter it2End)
{
  if(it1 == it1End) return true;
  if(it2 == it2End) return true;

  Set1Iter it1Last = it1End;  --it1Last;
  Set2Iter it2Last = it2End;  --it2Last;
  if(*it1 > *it2Last || *it2 > *it2Last) return true;

  while(it1 != it1End && it2 != it2End) {
    if(*it1 == *it2) return false;
    if(*it1 < *it2) { it1++; }
    else { it2++; }
  }
  return true;
}


template <class N,class E>
void ComputeSkeleton(const Graph::UndirectedGraph<N,E>& roadmap)
{
  Graph::UndirectedGraph<set<int>,set<int> > skeleton;
  set<int> free;
  for(size_t i=0;i<roadmap.nodes.size();i++)
    free.insert(i);
  vector<int> degree(roadmap.nodes.size());
  for(size_t i=0;i<roadmap.nodes.size();i++)
    degree[i] = roadmap.Degree(i);
  while(!free.empty()) {
    printf("%d remaining\n",free.size());
    int best=-1;
    int dbest=-1;
    for(set<int>::const_iterator i=free.begin();i!=free.end();i++) {
      if(degree[*i] > dbest) {
	best = *i;
	dbest = degree[*i];
      }
    }
    set<int> visSet;
    Graph::Get1Ring(roadmap,best,visSet);
    visSet.insert(best);
    int j=skeleton.AddNode(visSet);
    for(size_t i=0;i<skeleton.nodes.size();i++) {
      if((int)i == j) continue;
      if(!set_disjoint(skeleton.nodes[i].begin(),skeleton.nodes[i].end(),
		       visSet.begin(),visSet.end())) {
	set<int> intersection;
	set_intersection(skeleton.nodes[i].begin(), skeleton.nodes[i].end(),
			 visSet.begin(),visSet.end(),inserter(intersection,intersection.begin()));
	skeleton.AddEdge(i,j,intersection);
      }
    }
    for(set<int>::const_iterator i=visSet.begin();i!=visSet.end();i++) {
      if(free.count(*i)) {
	free.erase(free.find(*i));
      }
    }
    //adjust degrees
    for(set<int>::const_iterator i=free.begin();i!=free.end();i++) {
      int d=0;
      Graph::UndirectedEdgeIterator<E> e;
      for(roadmap.Begin(*i,e);!e.end();e++) {
	if(free.count(e.target())) d++;
      }
      degree[*i]=d;
    }
  }
  Graph::UndirectedGraph<string,string> skelSave;
  Graph::CopyStructure(skeleton,skelSave);
  for(size_t i=0;i<skeleton.nodes.size();i++) {
    stringstream ss;
    ss<<skeleton.nodes[i].size();
    skelSave.nodes[i] = ss.str();
  }
  for(size_t i=0;i<skeleton.nodes.size();i++) {
    Graph::EdgeIterator<set<int> > e;
    for(skeleton.Begin(i,e);!e.end();++e) {
      stringstream ss;
      ss<<e->size();
      *skelSave.FindEdge(e.source(),e.target()) = ss.str();
    }
  }
  printf("Saving to skeleton.tgf\n");
  ofstream out("skeleton.tgf");
  Graph::Save_TGF(out,skelSave);
  out.close();
}

template <class N,class E>
void GetDegreeHistogram(const Graph::UndirectedGraph<N,E>& roadmap,vector<int>& degreeHistogram)
{
  degreeHistogram.resize(roadmap.nodes.size());
  fill(degreeHistogram.begin(),degreeHistogram.end(),0);
  for(size_t i=0;i<roadmap.nodes.size();i++)
    degreeHistogram[roadmap.Degree(i)]++;
}

/// Computes the epsilon constant for the roadmap
template <class N,class E>
Real CalcEpsilon(const Graph::UndirectedGraph<N,E>& roadmap)
{
  Real mineps = 1;
  for(size_t i=0;i<roadmap.nodes.size();i++) {
    size_t nconnected=roadmap.Degree(i);
    mineps = Min(mineps,Real(nconnected)/Real(roadmap.nodes.size()-1));
  }
  return mineps;
}

//For a roadmap V,E with Rsize=|V|
//Given a candidate set S, and the number of seen nodes outside of S
//computes the alpha/beta constants that maximize f(alpha,beta)
pair<Real,Real> ComputeAlphaBeta(const set<int>& S,const vector<int>& numVisible,size_t Rsize,Real branch=Inf,bool print=false)
{
  Real betaScale = 1.0/(Rsize-S.size());
  Real alphaScale = 1.0/S.size();
  vector<int> visSorted(S.size(),0);  //sorted in decreasing order
  int k=0;
  for(set<int>::const_iterator i=S.begin();i!=S.end();i++,k++)
    visSorted[k] = -numVisible[*i];  //note the negative
  sort(visSorted.begin(),visSorted.end());
  //max min(i/|S|,v[i]/|R-S|)
  pair<Real,Real> ab(0,1); //0% of S can see 100% of R/S
  Real objective=AlphaBetaObj(ab);
  if(print) printf("Visibility histogram: ");
  for(size_t i=0;i<visSorted.size();i++) {
    if(print) printf("%d ",-visSorted[i]);
    pair<Real,Real> abi(Real(i+1)*alphaScale,Real(-visSorted[i])*betaScale);
    if(AlphaBetaObj(abi) > objective) {
      ab=abi;
      objective = AlphaBetaObj(abi);
    }
    if(!print) {
      if(abi.first < objective) break; //early termination -- found minimum
      if(objective > branch) break; //early termination -- exceeded branch
    }
  }
  if(print) printf("\n");
  return ab;
}

/** @brief Computes the maximum (alpha,beta) constants for subset S of the
 * the roadmap.
 */
template <class N,class E>
pair<Real,Real> ComputeAlphaBeta(const Graph::UndirectedGraph<N,E>& roadmap,const set<int>& S,bool print=false)
{
  vector<int> numVisible(roadmap.nodes.size(),0);
  for(set<int>::const_iterator i=S.begin();i!=S.end();i++) {
    Graph::UndirectedEdgeIterator<N> e;
    for(roadmap.Begin(*i,e);!e.end();e++) 
      if(S.count(e.target())==0)
	numVisible[*i]++;
  }
  return ComputeAlphaBeta(S,numVisible,roadmap.nodes.size(),Inf,print);
}

/** @brief Tries a greedy approach to find the minimal (alpha,beta) constants
 * for the roadmap.
 *
 * Approach greedily modifies a set initialized to each vertex to improve the
 * (alpha,beta) constants according to the f(alpha,beta) objective.
 */
template <class N,class E>
pair<Real,Real> CalcAlphaBetaGreedy(const Graph::UndirectedGraph<N,E>& roadmap,set<int>* minSet=NULL)
{
  pair<Real,Real> minab(1,1);
  vector<bool> visited(roadmap.nodes.size(),false);
  for(size_t i=0;i<roadmap.nodes.size();i++) {
    //hypothesis: if a node is clustered in a set, then we don't have to
    //check it again because the greedy growth of the set from i will converge
    //to the same set
    if(visited[i]) continue;   

    printf("%d\n",i);
    //greedy growth of component
    pair<Real,Real> abi(1.0,0.0);
    set<int> component;
    set<int> adjacent;
    
    //initialize at i
    vector<int> numVisible(roadmap.nodes.size(),0);
    component.insert(i);
    Graph::UndirectedEdgeIterator<N> e;
    for(roadmap.Begin(i,e);!e.end();e++) {
      Assert(e.source()==i);
      adjacent.insert(e.target());
      numVisible[i]++;
    }
    abi.second = Real(adjacent.size())/Real(roadmap.nodes.size()-component.size());
    //printf("Starting abi %g %g\n",abi.first,abi.second);
    bool canExpand=true;
    while(canExpand) {
      pair<Real,Real> minobj = abi;
      int bestExpand=-1;
      int bestReject=-1;
      for(set<int>::const_iterator j=adjacent.begin();j!=adjacent.end();j++) {
	int n=*j;
	//consider adding n to the component, see if this reduces min(alpha,beta)
	//add visiblity count
	for(roadmap.Begin(n,e);!e.end();e++) {
	  Assert(e.source()==n);
	  if(component.count(e.target())==0)
	    numVisible[n]++;
	  else {
	    numVisible[e.target()]--;
	    Assert(numVisible[e.target()] >= 0);
	  }
	}
	component.insert(n);
	
	//perform visibility count
	pair<Real,Real> candidateAB = ComputeAlphaBeta(component,numVisible,roadmap.nodes.size(),AlphaBetaObj(minobj));
	if(AlphaBetaObj(candidateAB) < AlphaBetaObj(minobj)) {
	  minobj = candidateAB;
	  bestExpand = n;
	} 
	
	component.erase(component.find(n));
	//remove visiblity count
	for(roadmap.Begin(n,e);!e.end();e++) {
	  Assert(e.source()==n);
	  if(component.count(e.target())==0) {
	    numVisible[n]--;
	    Assert(numVisible[n] >= 0);
	  }
	  else 
	    numVisible[e.target()]++;
	}
      }
      
      if(component.size()>1) {
	set<int> tempComponent=component;
	for(set<int>::const_iterator j=component.begin();j!=component.end();j++) {
	  int n=*j;
	  if((int)n==i) continue;
	  //consider deleting n to the component, see if this reduces min(alpha,beta)
	  //add visiblity count
	  for(roadmap.Begin(n,e);!e.end();e++) {
	    Assert(e.source()==n);
	    if(component.count(e.target())!=0) {
	      numVisible[e.target()]++;
	      Assert(numVisible[e.target()] >= 0);
	    }
	  }
	  tempComponent.erase(tempComponent.find(n));
	  
	  //perform visibility count
	  pair<Real,Real> candidateAB = ComputeAlphaBeta(tempComponent,numVisible,roadmap.nodes.size(),AlphaBetaObj(minobj));
	  if(AlphaBetaObj(candidateAB) < AlphaBetaObj(minobj)) {
	    minobj = candidateAB;
	    bestExpand = -1;
	    bestReject = n;
	  } 

	  tempComponent.insert(n);
	  //remove visiblity count
	  for(roadmap.Begin(n,e);!e.end();e++) {
	    Assert(e.source()==n);
	    if(component.count(e.target())!=0) 
	      numVisible[e.target()]--;
	  }
	}
      }
      
      if(bestExpand >= 0) {
	//printf("Add %d, %g %g\n",bestExpand,minobj.first,minobj.second);
	//accept the addition of n to the component
	canExpand=true;
	abi = minobj;
	int n=bestExpand;
	component.insert(n);
	adjacent.erase(adjacent.find(n));
	for(roadmap.Begin(n,e);!e.end();e++) {
	  Assert(e.source()==n);
	  if(component.count(e.target())==0) {
	    adjacent.insert(e.target());
	    numVisible[n]++;
	  }
	  else {
	    numVisible[e.target()]--;
	  }
	}
      }
      else if(bestReject >= 0) {
	//printf("Delete %d, %g %g\n",bestReject,minobj.first,minobj.second);
	//accept the addition of n to the component
	canExpand=true;
	abi = minobj;
	int n=bestReject;
	component.erase(component.find(n));
	adjacent.insert(n);
	for(roadmap.Begin(n,e);!e.end();e++) {
	  Assert(e.source()==n);
	  if(component.count(e.target())==0) {
	    numVisible[n]--;
	    //TODO: this is messed up, the node may still be adjacent to another
	    adjacent.erase(adjacent.find(e.target()));
	    Assert(numVisible[n]>=0);
	  }
	  else {
	    numVisible[e.target()]++;
	  }
	}
	//HACK: recompute adjacent
	adjacent.clear();
	for(set<int>::const_iterator j=component.begin();j!=component.end();j++) {
	  for(roadmap.Begin(*j,e);!e.end();e++) {
	    Assert(e.source()==*j);
	    if(component.count(e.target())==0) 
	      adjacent.insert(e.target());
	  }
	}
      }
      else canExpand=false;
      
      if(canExpand) {
	pair<Real,Real> ab=ComputeAlphaBeta(component,numVisible,roadmap.nodes.size(),Inf);
	if(ab != minobj) {
	  printf("Error\n");
	  printf("updating alpha=%g, beta=%g\n",ab.first,ab.second);
	  printf("minobj alpha=%g, beta=%g\n",minobj.first,minobj.second);
	  getchar();
	}
      }
    }
    int numVisited=0;
    for(set<int>::const_iterator j=component.begin();j!=component.end();j++) {
      if(visited[*j]) numVisited++;
      visited[*j]=true;
    }
    printf("Num already visited %d\n",numVisited);
    numVisited=0;
    for(size_t j=0;j<roadmap.nodes.size();j++) {
      if(visited[j]) numVisited++;
      printf("%d ",(int)visited[j]);
    }
    printf("\n");
    printf("Num total visited %d\n",numVisited);
    
    /*
      if(FuzzyEquals(AlphaBetaObj(abi),AlphaBetaObj(minab),1e-10)) {
        printf("equal alpha beta\n");
	if(minSet) printf("In set: %d\n",minSet->count(i));
      }
      else
	if(minSet) printf("In set: %d\n",minSet->count(i));
      */
    if(AlphaBetaObj(abi)+1e-10 < AlphaBetaObj(minab)) {
      minab = abi;
      if(minSet) {
	*minSet = component;
	pair<Real,Real> ab=ComputeAlphaBeta(component,numVisible,roadmap.nodes.size(),Inf,true);
	printf("Setting alpha=%g, beta=%g\n",ab.first,ab.second);
	printf("Abi alpha=%g, beta=%g\n",abi.first,abi.second);
      }
    }
  }
  return minab;
}

const option options [] = {
  {"stats",0,0,0},
  {"epsilon",0,0,0},
  {"abgreedy",0,0,0},
  {"degreehistogram",0,0,0},
  {"components",0,0,0},
  {"skeleton",0,0,0},
  {0,0,0,0},
};

const char* option_desc [] = {
  "Print basic stats (on by default)",
  "Compute epsilon-goodness lower bound",
  "Compute the alpha-beta constants using a greedy approach",
  "Print the histogram of vertex degrees",
  "Compute connected components",
  "Compute roadmap skeleton",
};

void printopts(const option* opts,const char** desc=NULL)
{
  while(opts->name) {
    printf("\t-%s",opts->name);
    if(opts->has_arg==1)
      printf(" args");
    else if(opts->has_arg==2)
      printf(" [args]");
    
    if(desc) {
      printf(" : %s",*desc);
      desc++;
    }

    printf("\n");
    opts++;
  }
}

int main(int argc,char** argv)
{
  bool stats=true,epsilon=false,abgreedy=false,degreehistogram=false,components=true,skeleton=false;
  while(1) {
    int option_index=0;
    int c=getopt_long_only(argc,argv,"",options,&option_index);
    if(c == -1) break;
    else if(c=='?') {
      printf("Unknown argument %c\n",optopt);
    }
    else if(c==0) {
      if(0==strcmp(options[option_index].name,"epsilon"))
	epsilon=true;
      else if(0==strcmp(options[option_index].name,"abgreedy"))
	abgreedy=true;
      else if(0==strcmp(options[option_index].name,"stats"))
	stats=true;
      else if(0==strcmp(options[option_index].name,"degreehistogram"))
	degreehistogram=true;
      else if(0==strcmp(options[option_index].name,"components"))
	components=true;
      else if(0==strcmp(options[option_index].name,"skeleton"))
	skeleton=true;
      else {
	printf("Warning: option %d=%s is not yet supported\n",option_index,options[option_index].name);
      }
    }
  }
  int index=optind;  //start reading from optind
  if(index >= argc) {
    printf("USAGE: RoadmapStats [options] graph_file\n");
    printopts(options,option_desc);
    return 1;
  }

  ifstream in(argv[optind]);
  Graph::UndirectedGraph<string,string> Gstr;
  if(!Graph::Load_TGF(in,Gstr)) {
    printf("Failed loading TGF file %s\n",argv[optind]);
    return 1;
  }
  printf("Loaded TGF file %s\n",argv[optind]);
  printf("***** Stats *****\n");
  printf("%d nodes, %d edges\n",Gstr.NumNodes(),Gstr.NumEdges());
  StatCollector degree;  
  for(size_t i=0;i<Gstr.nodes.size();i++)
    degree.collect(Gstr.Degree(i));
  printf("Degree: "); degree.Print(cout);
  printf("\n");

  if(degreehistogram) {
    printf("***** Degree histogram *****\n");
    vector<int> hist;
    GetDegreeHistogram(Gstr,hist);
    for(int i=0;i<=(int)degree.maximum();i++)
      printf("%d\t",i);
    printf("\n");
    for(int i=0;i<=(int)degree.maximum();i++)
      printf("%d\t",hist[i]);
    printf("\n");
  }

  if(epsilon) {
    printf("Epsilon = %g\n",CalcEpsilon(Gstr));
  }

  if(components) {
    Graph::ConnectedComponents ccs;
    ccs.Compute(Gstr);
    printf("***** Components *****\n");
    printf("%d connected components\n",ccs.NumComponents());
    printf("Sizes: ");
    vector<int> reps,cc;
    ccs.GetRepresentatives(reps); 
    for(size_t i=0;i<reps.size();i++) {
      ccs.EnumerateComponent(reps[i],cc);
      printf("%d ",cc.size());
    }
    printf("\n");
  }

  if(skeleton) {
    ComputeSkeleton(Gstr);
  }
}
