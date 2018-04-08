#include "XmlReader.h"
#include "DrawableSpace.h"
#include <graph/IO.h>
#include <math/random.h>
#include <Timer.h>
#include <getopt.h>
#include <fstream>
using namespace std;


const option options [] = {
  {"planner",required_argument,0,0},
  {"query",required_argument,0,0},
  {"out",required_argument,0,0},
  {"iters",required_argument,0,0},
  {"time",required_argument,0,0},
  {"milestones",required_argument,0,0},
  {"seed",required_argument,0,0},
  {"quiet",0,0,0},
  {"verbose",0,0,0},
  {0,0,0,0},
};

const char* option_desc [] = {
  "Planner xml file",
  "Query index or name",
  "File to output roadmap",
  "Terminate when the planner reaches an iteration limit",
  "Terminate when the planner reaches a time limit, in seconds",
  "Terminate when the roadmap reaches a milestone limit",
  "Seeds the random number generator",
  "Print nothing",
  "Print detailed timing information",
};

void printopts(const option* opts,const char** desc=NULL)
{
  while(opts->name) {
    printf("\t-%s",opts->name);
    if(opts->has_arg==required_argument)
      printf(" args");
    else if(opts->has_arg==optional_argument)
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
  Srand(time(NULL));
  //settings
  int maxIters=1000;
  int milestoneLimit=INT_MAX;
  double timeLimit = Inf;
  int verbose=1;
  bool compactSave=false;
  string queryName;
  string outputFile;
  DrawableCSpace cspace;
  MotionPlannerFactory factory;
  factory.knn = 0;
  factory.ignoreConnectedComponents=true;
  factory.storeEdges=false;

  while(1) {
    int option_index=0;
    int c=getopt_long_only(argc,argv,"",options,&option_index);
    if(c == -1) break;
    else if(c=='?') {
      printf("Unknown argument %c\n",optopt);
    }
    else if(c==0) {
      if(0==strcmp(options[option_index].name,"planner")) {
	XmlDocument doc;
	if(!doc.Load(optarg)) {
	  printf("Error loading XML document %s\n",optarg);
	  return 1;
	}
	if(0==strcmp(doc.RootElement()->Value(),"planner")) {
	  if(!XmlParse(doc.RootElement(),factory)) {
	    printf("Error reading planner from XML document %s\n",optarg);
	    return 1;
	  }
	}
	else {
	  printf("Error reading planner from XML document %s\n",optarg);
	  return 1;
	}
      }
      else if(0==strcmp(options[option_index].name,"query")) {
	queryName=optarg;
      }
      else if(0==strcmp(options[option_index].name,"out")) {
	outputFile=optarg;
      }
      else if(0==strcmp(options[option_index].name,"time")) {
	timeLimit = atof(optarg);
      }
      else if(0==strcmp(options[option_index].name,"milestones")) {
	milestoneLimit = atoi(optarg);
      }
      else if(0==strcmp(options[option_index].name,"iters")) {
	maxIters = atoi(optarg);
      }
      else if(0==strcmp(options[option_index].name,"seed")) {
	int seed = atoi(optarg);
	Srand(seed);
      }
      else if(0==strcmp(options[option_index].name,"quiet")) {
	verbose=0;
      }
      else if(0==strcmp(options[option_index].name,"verbose")) {
	verbose=2;
      }
      else {
	printf("Warning: option %d=%s is not yet supported\n",option_index,options[option_index].name);
      }
    }
  }

  int index = optind;
  if(index >= argc) {
    printf("USAGE: PRMBuild [options] space\n");
    printopts(options,option_desc);
    return 1;
  }

  XmlDocument doc;
  if(!doc.Load(argv[index])) {
    printf("Error loading XML document %s\n",argv[index]);
    return 1;
  }
  if(!cspace.ReadFromXml(doc.RootElement())) {
    printf("Error reading cspace from XML document %s\n",argv[index]);
    return 1;
  }
  if(verbose) printf("Cspace file %s read\n",argv[index]);

  //setup queries
  int queryIndex = -1;
  if(queryName.length()!=0) {
    //setup a query
    for(size_t i=0;i<cspace.queries.size();i++)
      if(cspace.queries[i].name == queryName) {
	queryIndex = (int)i;
	break;
      }
    if(queryIndex < 0) {
      queryIndex = atoi(queryName.c_str());
    }
    if(queryIndex < 0 || queryIndex >= (int)cspace.queries.size()) {
      printf("Invalid query specified\n");
      return 1;
    }
  }

  //Now initialize the planner
  MotionPlannerInterface* planner=factory.Create(cspace);
  int startNode,goalNode;
  Timer timer;
  int iters;
  //query is specified, add them to the planner
  if(queryIndex >= 0) {
    Config start,goal;
    cspace.queries[queryIndex].GetEndpoints(start,goal);
    startNode = planner->AddMilestone(start);
    goalNode = planner->AddMilestone(goal);
  }
  //start planning
  for(iters=0;iters<maxIters;iters++) {
    if(verbose == 2) {
      printf("Iteration %d\n",iters);
    }
    planner->PlanMore();

    if(queryIndex >= 0) { //check if query is solved
      if(planner->IsConnected(startNode,goalNode)) {
	if(verbose)
	  printf("Solved query successfully\n");
	break;
      }
    }

    if(!IsInf(timeLimit)) {
      if(timer.ElapsedTime() >= timeLimit) {
	if(verbose)
	  printf("Terminated by time limit: %ss elapsed\n",timer.ElapsedTime());
	break;
      }
    }
    if(milestoneLimit < maxIters) {
      if(planner->NumMilestones() >= milestoneLimit) {
	if(verbose)
	  printf("Terminated by milestone limit: %d milestones\n",planner->NumMilestones());
	break;
      }
    }
  }
  if(verbose)
    printf("Planning done, %d iterations, %d milestones, %gs elapsed\n",iters,planner->NumMilestones(),timer.ElapsedTime());

  if(!outputFile.empty()) {
    if(verbose) 
      printf("Saving roadmap to %s\n",outputFile.c_str());

    RoadmapPlanner prm(cspace);
    planner->GetRoadmap(prm);
    
    Graph::Graph<string,string> Gstr;
    if(compactSave)
      Graph::CopyStructure(prm.roadmap,Gstr);
    else
      Graph::NodesToStrings(prm.roadmap,Gstr);

    ofstream out(outputFile.c_str());
    Graph::Save_TGF(out,Gstr);
    out.close();
  }
  delete planner;

  return 0;
}
