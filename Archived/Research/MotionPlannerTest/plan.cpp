#include "MotionPlannerProgram.h"
#include <KrisLibrary/Timer.h>
#include <KrisLibrary/utils/PropertyMap.h>
#include <KrisLibrary/planning/AnyMotionPlanner.h>
#include <KrisLibrary/utils/ioutils.h>
#include <KrisLibrary/math/random.h>
#include <KrisLibrary/utils/stringutils.h>
#include <string.h>
#include <time.h>
#include <fstream>

struct AnyPlannerProgram : public MotionPlannerProgram
{
  bool hasStart,hasGoal,hasPath;
  Config start,goal;
  MilestonePath path;
  MotionPlannerFactory factory;
  SmartPointer<MotionPlannerInterface> planner;
  Real cumulativeTime;
  bool drawRoadmap;

  AnyPlannerProgram()
  {
    hasStart=hasGoal=hasPath=false;
    drawRoadmap = true;
  }

  bool LoadPlannerSettings(const string& settingsString) 
  {
    if(!settingsString.empty()) {
      bool res = factory.LoadJSON(settingsString);
      if(!res) 
	printf("Warning, incorrectly formatted planner settings file\n");
      return res;
    }
    return false;
  }

  virtual bool Initialize() {
    if(!MotionPlannerProgram::Initialize()) return false;
    DefaultPlannerSetup();
    return true;
  }

  void DefaultPlannerSetup()
  {
    if(cspace.planningQueries.empty()) {
      start.resize(2);
      start[0] = 0.3;
      start[1] = 0.5;
      goal.resize(2);
      goal[0] = 0.7;
      goal[1] = 0.5;
    }
    else {
      start = cspace.planningQueries[0].first;
      goal = cspace.planningQueries[0].second;
    }
    hasStart = hasGoal = true;
    InitPlanner();
  }

  void InitPlanner()
  {
    planner = factory.Create(cspace,start,goal);
    cumulativeTime = 0;
    hasPath = false;
    path.edges.clear();
  }

  string PlanAll(MilestonePath& path,HaltingCondition& cond)
  {
    string res = planner->Plan(path,cond);
    cout<<"Planner result \""<<res<<"\",";
    if(!path.edges.empty()) 
      cout<<" path length "<<path.Length()<<",";
    cout<<" stats: ";
    PropertyMap stats;
    planner->GetStats(stats);
    stats.Print(cout);
    cout<<endl;
    return res;
  }

  void PlanStep()
  {
    planner->PlanMore();
  }

  void PlanMore(int num=1000,FILE* csvout=stdout)
  {
    Assert(hasStart&&hasGoal);
    Timer timer;
    for(int i=0;i<num;i++) 
      PlanStep();
    cumulativeTime += timer.ElapsedTime();

    if(csvout == NULL) return;

    //extract minimum cost path
    hasPath = planner->IsSolved();
    if(hasPath) {
      planner->GetSolution(path);
      /*
      //TODO: fix this problem
      if(planner.spp.epsilon == 0)
	Assert(path.IsFeasible());
      */
    }

    Real cost = Inf;
    if(!path.edges.empty()) cost = path.Length();
    PropertyMap stats;
    planner->GetStats(stats);
    if(csvout == stdout) {
      printf("Plan step %d, time %g, resulting cost %g\n",planner->NumIterations(),cumulativeTime,cost);
      cout<<"Stats: ";
      stats.Print(cout);
      cout<<endl;
    }
    else {
      fprintf(csvout,"%d,%g,%g",planner->NumIterations(),cumulativeTime,cost);
      for(PropertyMap::const_iterator i=stats.begin();i!=stats.end();i++)
	fprintf(csvout,",%s",i->second.c_str());
      fprintf(csvout,"\n");
    }
  }

  void BatchTest(const char* filename,int numTrials=10,int maxIters=100000,Real tmax = 10,Real printIncrement=0.1)
  {
    printf("Batch test, saving to %s:\n",filename);
    FILE* f = NULL;
    f = fopen(filename,"w");
    if(!f) {
      fprintf(stderr,"Unable to open file %s for writing\n",filename);
      return;
    }
    fprintf(f,"trial,plan iters,plan time,best cost");
    PropertyMap stats;
    planner->GetStats(stats);
    for(PropertyMap::const_iterator i=stats.begin();i!=stats.end();i++)
      fprintf(f,",%s",i->first.c_str());
    fprintf(f,"\n");

    for(int trials=0;trials<numTrials;trials++) {
      InitPlanner();
      Assert(cumulativeTime == 0);
      Timer timer;
      Real lastPrintTime = -Inf;
      for(int i=0;i<maxIters;i++) {
	if(cumulativeTime >= lastPrintTime + printIncrement) {
	  lastPrintTime = timer.ElapsedTime();
	  fprintf(f,"%d,",trials);
	  PlanMore(1,f);
	}
	else 
	  PlanMore(1,NULL);

	if(cumulativeTime > tmax) {
	  printf("Time %g, cumulative time %g\n",timer.ElapsedTime(),cumulativeTime);
	  break;
	}
      }
    }
    fclose(f);
  }

  virtual void Handle_Display() {
    SetupDisplay();
    DrawCSpace();

    if(drawRoadmap) {
      RoadmapPlanner roadmap(cspace);
      planner->GetRoadmap(roadmap);
      DrawGraphCallback callback(roadmap.roadmap,&cspace);
      callback.nodeColor.set(1,0,1);
      callback.edgeColor.set(0,0,0,0.75);
      callback.doLazy = true;
      callback.lazyEdgeColor.set(0.5,0.5,0.5,0.25);
      glPointSize(3.0);
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
      callback.Draw();
      glDisable(GL_BLEND);
    }
    if(hasPath) {
      glColor3f(0,0,1);
      glLineWidth(4.0);
      for(size_t i=0;i<path.edges.size();i++)
	cspace.DrawPath(path.edges[i]->Start(),path.edges[i]->Goal());
      glLineWidth(1.0);
    }
    if(hasStart) {
      glPointSize(7.0);
      glColor3f(0,1,0);
      cspace.DrawConfiguration(start,1);
      glPointSize(3.0);
    }
    if(hasGoal) {
      glPointSize(7.0);
      glColor3f(1,0,0);
      cspace.DrawConfiguration(goal,2);
      glPointSize(3.0);
    }
    glutSwapBuffers();
  }

  bool SampleConfig(Config& q,int maxIters=100)
  {
    for(int i=0;i<maxIters;i++) {
      cspace->Sample(q);
      if(cspace->IsFeasible(q)) return true;
    }
    return false;
  }

  bool SampleProblem()
  {
    if(!SampleConfig(start)) return false;
    if(!SampleConfig(goal)) return false;

    hasStart = true;
    hasGoal = true;
    hasPath = false;
    InitPlanner();
    return true;
  }

  virtual void Handle_Keypress(unsigned char key,int x,int y)
  {
    if(key == '1') {
      PlanMore(1);
    }
    else if(key == ' ') {
      PlanMore();
    }
    else if(key == 's') {
      if(!SampleProblem()) return;
    }
    else if(key == 'r') { 
      hasStart=false;
      hasGoal=false;
      hasPath=false;
    }
    else if(key == 't') {
      string outputfile = string("results/")+factory.type+string("csv");
      BatchTest(outputfile.c_str());
    }
    else if(key == 'd') {
      drawRoadmap = !drawRoadmap;
    }
    Refresh();
  }

  virtual void Handle_Click(int button,int state,int x,int y) {
    if(state == GLUT_DOWN) {
      Config q;
      ClickToConfig(x,y,q);
      if(!hasStart) {
	start=q;
	hasStart=true;
      }
      else if(!hasGoal) {
	goal=q;
	hasGoal=true;
	InitPlanner();
      }
      else {
	PlanMore();
      }
      Refresh();
    }
  }
};



inline bool LoadCSpace(DrawableCSpace& cspace,const char* file)
{
  XmlDocument doc;
  if(!doc.Load(file)) {
    fprintf(stderr,"Error loading XML document %s\n",file);
    return false;
  }
  return cspace.ReadFromXml(doc.RootElement());
}


int main(int argc,const char** argv)
{
  if(argc <= 1) {
    printf("USAGE: Plan [options] cspace_file\n");
    printf("OPTIONS:\n");
    printf("-o filename: the output path (default none)\n");
    printf("-p settings: set the planner configuration file\n");
    printf("-v: visualize the planner rather than just plan\n");
    printf("-opt: do optimal planning (do not terminate on the first found solution)\n");
    printf("-n iters: set the default number of iterations (default 1000)\n");
    printf("-t time: set the planning time limit (default infinity)\n");
    printf("-b n statsfile: batch test with n iterations, save planning data\n     to the given file (CSV format)\n");
    return 0;
  }
  Srand(time(NULL));
  const char* outputfile = NULL;
  HaltingCondition termCond;
  string plannerSettings;
  bool visualize = false;
  int batchSize = 0;
  int i;
  //parse command line arguments
  for(i=1;i<argc;i++) {
    if(argv[i][0]=='-') {
      if(0==strcmp(argv[i],"-n")) {
	termCond.maxIters = atoi(argv[i+1]);
	i++;
      }
      else if(0==strcmp(argv[i],"-t")) {
	termCond.timeLimit = atof(argv[i+1]);
	i++;
      }
      else if(0==strcmp(argv[i],"-opt")) {
	termCond.foundSolution = false;
      }
      else if(0==strcmp(argv[i],"-p")) {
	if(!GetFileContents(argv[i+1],plannerSettings)) {
	  printf("Unable to load planner settings file %s\n",argv[i+1]);
	  return 1;
	}
	i++;
      }
      else if(0==strcmp(argv[i],"-o")) {
	outputfile = argv[i+1];
	i++;
      }
      else if(0==strcmp(argv[i],"-v")) {
	visualize = true;
      }
      else if(0==strcmp(argv[i],"-b")) {
	batchSize = atoi(argv[i+1]);
	outputfile = argv[i+2];
	i += 2;
      }
      else {
	printf("Invalid option %s\n",argv[i]);
	return 1;
      }
    }
    else break;
  }
  if(i+1 < argc) {
    printf("Too few arguments provided\n");
    printf("USAGE: Plan [options] cspace_file\n");
    return 1;
  }
  if(i+1 > argc) {
    printf("Warning: extra arguments provided\n");
  }
  const char* cspacefile = argv[i];

  AnyPlannerProgram program;
  program.LoadPlannerSettings(plannerSettings);
  if(!LoadCSpace(program.cspace,cspacefile)) {
    printf("Error reading cspace from %s\n",cspacefile);
    return 1;
  }
  if(visualize) {
    string name = string("Motion Planner (")+program.factory.type+string(")");
    return program.Run(name.c_str());
  }
  else {
    //just plan on command line
    program.DefaultPlannerSetup();
    if(batchSize > 0) {
      program.BatchTest(outputfile,batchSize,termCond.maxIters,termCond.timeLimit);
    }
    else {
      //single plan, output the path
      MilestonePath path;
      program.PlanAll(path,termCond);
      //save to disk
      if(!path.edges.empty() && outputfile != NULL) {
	cout<<"Saving result to "<<outputfile<<endl;
	ofstream f(outputfile,ios::out);
	for(int i=0;i<path.NumMilestones();i++) {
	  f << path.GetMilestone(i)<< endl;
	}
	f.close();
      }
    }
  }
  return 0;
}
