#include "MotionPlannerProgram.h"
#include <KrisLibrary/Timer.h>
#include <utils/PropertyMap.h>
#include <KrisLibrary/planning/AnyMotionPlanner.h>
#include <KrisLibrary/Graph/ShortestPaths.h>
#include <KrisLibrary/Graph/Path.h>
#include <KrisLibrary/utils/ioutils.h>
#include <KrisLibrary/utils/csv.h>
#include <KrisLibrary/math/random.h>
#include <KrisLibrary/utils/stringutils.h>
#include <string.h>
#include <time.h>
#include <fstream>

//Graph search callbacks
class EdgeDistance
{
 public:
  Real operator () (const SmartPointer<EdgePlanner>& e,int s,int t)
  {
    assert(e->Space() != NULL);
    Real res = e->Space()->Distance(e->Start(),e->Goal());
    if(res <= 0) {
      printf("RoadmapPlanner: Warning, edge has nonpositive length %g\n",res);
      return Epsilon;
    }
    return res;
  }
};


struct MotionPrimitive
{
  Config start,goal;
  vector<Config> path;
};

struct LearningPlannerProgram : public MotionPlannerProgram
{
  bool hasStart,hasGoal,hasPath;
  Config start,goal;
  MilestonePath path;
  MotionPlannerFactory factory;
  SmartPointer<MotionPlannerInterface> planner;
  Real cumulativeTime;
  bool drawRoadmap;

  LearningPlannerProgram()
  {
    factory.type = "prm*";
    hasStart=hasGoal=hasPath=false;
    drawRoadmap = true;
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

  bool LoadPrimitives(const char* fn,vector<MotionPrimitive>& primitives)
  {
    CSVTable csv;
    if(!csv.Load(fn,true)) return false;
    printf("Parsing...\n");
    primitives.resize(csv.NumEntries());
    int successfulIndex = csv.ItemIndex("connected");
    int pathIndex = csv.ItemIndex("path");
    for(int entry=0;entry<csv.NumEntries();entry++) {
      MotionPrimitive& p = primitives[entry];
      CSVTable::Entry& ev = csv.entries[entry];
      if(ev.size() != start.n*2+3) {
	printf("Entry %d of incorrect size %d vs %d\n",entry,ev.size(),start.n*2+3);
	break;
      }

      //successful
      if(ev[successfulIndex].as<bool>()==true) {
	p.start.resize(start.n);
	p.goal.resize(start.n);
	int k=0;
	for(int i=0;i<start.n;i++,k++)
	  p.start[i] = ev[k].as<Real>();
	p.start.resize(start.n);
	for(int i=0;i<goal.n;i++,k++)
	  p.goal[i] = ev[k].as<Real>();
	stringstream ss(ev[pathIndex]);
	Config temp;
	while(ss >> temp) 
	  p.path.push_back(temp);
      }
      //save memory -- weird STL trick, clear() doesn't work
      CSVTable::Entry().swap(ev);
    }
    return true;
  }

  bool Adapt(const vector<Config>& path,const Config& newStart,const Config& newGoal,
	     MilestonePath& result,Real& cost,Real& time)
  {
    Timer timer;
    vector<Config> newPath(path.size());
    //linear warp
    Config p,q,d,f,g;
    Real fnorm,gnorm;
    f.sub(path.back(),path.front());
    g.sub(newGoal,newStart);
    gnorm = g.norm();
    fnorm = f.norm();
    Assert(fnorm  > 1e-10);
    Assert(gnorm  > 1e-10);
    newPath[0] = newStart;
    for(size_t i=1;i+1<path.size();i++) {
      /*
      d.sub(path[i],path.front());
      p.mul(f,d.dot(f)/Sqr(fnorm));
      q.mul(g,d.dot(f)/(fnorm*gnorm));
      newPath[i] = (d-p+q)+newStart;
      */
      newPath[i] = path[i];
    }
    newPath.back() = newGoal;
    //test feasibility
    result.edges.resize(newPath.size()-1);
    for(size_t i=0;i+1<newPath.size();i++) {
      result.edges[i] = cspace->LocalPlanner(newPath[i],newPath[i+1]);
      if(!result.edges[i]->IsVisible()) {
	time = timer.ElapsedTime();
	cost = 0;
	return false;
      }
    }
    cost = result.Length();
    time = timer.ElapsedTime();
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
    else if(key == 'p') {
      vector<MotionPrimitive> primitives;
      printf("Loading primitives from roadmap_paths.csv...\n");
      LoadPrimitives("roadmap_paths.csv",primitives);
      printf("Selecting primitive (euclidean)...\n");
      Real dmin = Inf;
      int primitive = -1;
      for(size_t i=0;i<primitives.size();i++) {
	Real d = primitives[i].start.distance(start) + primitives[i].goal.distance(goal);
	if(d < dmin) {
	  dmin = d;
	  primitive = (int)i;
	}
      }

      printf("Adapting primitive %d..., score %g\n",primitive,dmin);   
      Real cost,time;
      hasPath = Adapt(primitives[primitive].path,start,goal,
		      path,cost,time);
      printf("Result: %d\n",(int)hasPath);
    }
    else if(key == 'a') {
      vector<MotionPrimitive> primitives;
      printf("Loading primitives from roadmap_paths.csv...\n");
      LoadPrimitives("roadmap_paths.csv",primitives);
      int numAdaptations = 10000000;
      printf("Saving results to adaptation_results.csv\n");
      printf("Saving weighted cost to adaptation_costs.txt\n");
      ofstream out("adaptation_results.csv",ios::out);
      ofstream out2("adaptation_costs.txt",ios::out);
      out<<"source,target,success,cost,time"<<endl;
      for(int num=0;num<numAdaptations;num++) {
	if(num %10000 == 0) printf("%d / %d\n",num,numAdaptations);
	int i = RandInt(primitives.size());
	int j = RandInt(primitives.size());
	while(i==j)
	  j = RandInt(primitives.size());
	//adapt i->j
	bool success;
	MilestonePath result;
	Real cost;
	Real time;
	success = Adapt(primitives[i].path,primitives[j].start,primitives[j].goal,
			result,cost,time);
	out<<i<<","<<j<<","<<success<<","<<cost<<","<<time<<endl;
	out2<<i<<" "<<j<<" "<<(success? cost : 10)<<endl;
      }
      out.close();
      out2.close();
    }
    else if(key == 's') {
      if(planner) {
	RoadmapPlanner roadmap(cspace);
	planner->GetRoadmap(roadmap);
	printf("Saving 1M paths to roadmap_paths.csv\n");
	ofstream out("roadmap_paths.csv",ios::out);
	for(int i=0;i<start.n;i++)
	  out<<"start["<<i<<"],";
	for(int i=0;i<goal.n;i++)
	  out<<"goal["<<i<<"],";
	out<<"connected,length,path"<<endl;;
	int numPoints = 1000;
	for(int num=0;num<numPoints;num++) {
	  printf("%d / %d\n",num,numPoints);
	  int i=RandInt(roadmap.roadmap.nodes.size());
	  EdgeDistance distanceWeightFunc;
	  Graph::ShortestPathProblem<Config,SmartPointer<EdgePlanner> > spp(roadmap.roadmap);
	  spp.InitializeSource(i);
	  spp.FindAllPaths_Undirected(distanceWeightFunc);
	  for(int num2=0;num2<numPoints;num2++) {
	    int j=RandInt(roadmap.roadmap.nodes.size());
	    while(i==j)
	      j=RandInt(roadmap.roadmap.nodes.size());
	    for(int k=0;k<start.n;k++) 
	      out<<roadmap.roadmap.nodes[i][k]<<",";
	    for(int k=0;k<start.n;k++) 
	      out<<roadmap.roadmap.nodes[j][k]<<",";
	    if(!IsInf(spp.d[j])) {
	      out<<"1"<<",";
	      vector<int> nodes;
	      bool res=Graph::GetAncestorPath(spp.p,j,i,nodes);
	      Assert(res == true);
	      Assert(i == nodes.front());
	      Assert(j == nodes.back());
	      Real len = 0;
	      for(size_t k=0;k+1<nodes.size();k++)
		len += cspace->Distance(roadmap.roadmap.nodes[nodes[k]],roadmap.roadmap.nodes[nodes[k+1]]);
	      out<<len<<",";
	      for(size_t k=0;k<nodes.size();k++)
		out<<roadmap.roadmap.nodes[nodes[k]]<<'\t';
	      out<<endl;
	    }
	    else
	      out<<"0,,"<<endl;
	  }
	}
	out.close();
	printf("Done.\n");
      }
    }
    else if(key == 'r') { 
      hasStart=false;
      hasGoal=false;
      hasPath=false;
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
    printf("USAGE: LearningPlanner cspace_file\n");
    return 0;
  }
  Srand(time(NULL));
  const char* cspacefile = argv[1];

  LearningPlannerProgram program;
  if(!LoadCSpace(program.cspace,cspacefile)) {
    printf("Error reading cspace from %s\n",cspacefile);
    return 1;
  }
  return program.Run("Learning Planner");
}
