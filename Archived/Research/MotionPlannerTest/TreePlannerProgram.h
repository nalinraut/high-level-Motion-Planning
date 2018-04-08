#include <KrisLibrary/planning/MotionPlanner.h>
#include "MotionPlannerProgram.h"
#include <fstream>
#include <sstream>
using namespace std;

const static Real kShortcutCostCoefficient = 0.01;

//cos (theta) = dot(b-a,c-d)/D(b,a)D(c,d)
//D^2(x,y) = dot(x-y,x-y) = dot(x,x)+dot(y,y)-2dot(x,y)
//=> dot(x,y) = 1/2(D^2(x,y)-dot(x,x)-dot(y,y))
//dot(b-a,c-d)
//dot(b-a,c-d)=dot(b,c)+dot(a,d)-dot(b,d)-dot(a,c)
//=1/2[D^2(b,c)+D^2(a,d)-D^2(b,d)-D^2(a,c)]
Real CosAngle(CSpace& space,const Config& a,const Config& b,const Config& c,const Config& d)
{
  Real l1=space.Distance(a,b),l2=space.Distance(c,d);
  Real d1=space.Distance(b,c);
  Real d2=space.Distance(a,d);
  Real d3=space.Distance(b,d);
  Real d4=space.Distance(a,c);
  return Half*(Sqr(d1)+Sqr(d2)-Sqr(d3)-Sqr(d4))/(l1*l2);
}


struct TreePlannerProgram : public MotionPlannerProgram
{
  bool hasStart,hasGoal,hasPath;
  Config start,goal;
  MilestonePath path;
  BidirectionalRRTPlanner rrt;
  Array2D<StatCollector> counts;
  vector<StatCollector> distanceCounts;
  vector<Real> distanceBuckets;
  vector<StatCollector> reductionRandomByIters,costRandomByIters;
  vector<StatCollector> reductionAdaptiveByIters,costAdaptiveByIters;

  TreePlannerProgram()
    :rrt(cspace)
  {
    hasStart=hasGoal=hasPath=false;
  }

  virtual bool Initialize() {
    if(!MotionPlannerProgram::Initialize()) return false;
    return true;
  }

  bool FeasibleShortcut(Real u1,Real u2)
  {
    Config q1,q2;
    path.Eval(u1,q1);
    path.Eval(u2,q2);
    EdgePlanner* e=IsVisible(cspace,q1,q2);
    if(e) {
      delete e;
      return true;
    }
    return false;
  }

  virtual void Handle_Display() {
    SetupDisplay();
    DrawCSpace();

    DrawTreeCallback callback;
    callback.nodeColor.set(1,0,1);
    callback.edgeColor.set(0.5,0,0.5);
    glPointSize(3.0);
    glLineWidth(2.0);
    for(size_t i=0;i<rrt.connectedComponents.size();i++) {
      if(rrt.connectedComponents[i])
	rrt.connectedComponents[i]->DFS(callback);
    }
    if(hasPath) {
      glColor3f(0,0,1);
      glLineWidth(4.0);
      glBegin(GL_LINE_STRIP);
      for(size_t i=0;i<path.NumMilestones();i++)
	glVertex2v(path.GetMilestone(i));
      glEnd();
    }
    if(hasStart) {
      glPointSize(7.0);
      glColor3f(0,1,0);
      glBegin(GL_POINTS);
      glVertex2v(start);
      glEnd();
    }
    if(hasGoal) {
      glPointSize(7.0);
      glColor3f(1,0,0);
      glBegin(GL_POINTS);
      glVertex2v(goal);
      glEnd();
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

    rrt.Init(start,goal);
    for(int i=0;i<1000;i++) {
      if(rrt.Plan()) {
	rrt.CreatePath(path);
	hasPath = true;
	hasStart = true;
	hasGoal = true;
	return true;
      }
    }
    return false;
  }

  void TestDistances(int maxIters)
  {
    const static int numBuckets = 50;
    if((int)distanceCounts.size() != numBuckets) {
      distanceCounts.resize(numBuckets);
      distanceBuckets.resize(numBuckets);
      for(int i=0;i<numBuckets;i++)
	distanceBuckets[i] = Real(i+1)/numBuckets;
      distanceBuckets.back() = Inf;
    }
    for(int iters=0;iters<maxIters;iters++) {
      Config q1,q2;
      if(!SampleConfig(q1)) continue;
      if(!SampleConfig(q2)) continue;
      Real d=cspace->Distance(q1,q2);
      vector<Real>::iterator i=std::lower_bound(distanceBuckets.begin(),distanceBuckets.end(),d);
      Assert(i != distanceBuckets.end());
      int index=i-distanceBuckets.begin();
      EdgePlanner* e=IsVisible(cspace,q1,q2);
      if(e) {
	distanceCounts[index].collect(1.0);
	delete e;
      }
      else distanceCounts[index].collect(0.0);
    }
  }

  void TestShortcuts(int maxIters)
  {
    for(int iters=0;iters<maxIters;iters++) {
      cout<<"Iteration "<<iters<<endl;
      if(!SampleProblem()) continue;

      int m=100,n=100;
      if(counts.m != m+1 || counts.n != n+1)
	counts.resize(m+1,n+1);
      Real xscale = 1.0/m;
      Real yscale = 1.0/n;
      for(int i=0;i<=m;i++)
	for(int j=0;j<=n;j++)
	  if(FeasibleShortcut(Real(i)*xscale,Real(j)*yscale))
	    counts(i,j).collect(1.0);
	  else
	    counts(i,j).collect(0.0);
    }
  }

  bool TryShortcut(Real u1,Real u2)
  {
    if(u1 > u2) std::swap(u1,u2);
    Config q1,q2;
    int e1=path.Eval2(u1,q1);
    int e2=path.Eval2(u2,q2);
    if(e1 == e2) //on same segment
      return false;
    Assert(e1 >= 0 && e1 < (int)path.edges.size());
    Assert(e2 >= 0 && e2 < (int)path.edges.size());
    if(!cspace->IsFeasible(q1) || !cspace->IsFeasible(q2)) {
      //cout<<"Precision error with edge checker... config not feasible"<<endl;
      return false;
    }
    EdgePlanner* e=IsVisible(cspace,q1,q2);
    if(e) {
      EdgePlanner* ee1=cspace->LocalPlanner(path.edges[e1]->Start(),q1);
      EdgePlanner* ee2=cspace->LocalPlanner(q2,path.edges[e2]->End());
      if(!ee1->IsVisible() || !ee2->IsVisible()) {
	//cout<<"Precision error with edge checker..."<<endl;
	delete ee1;
	delete ee2;
	delete e;
      }
      else {
	//replace segment
	path.edges.erase(path.edges.begin()+e1,path.edges.begin()+e2+1);
	path.edges.insert(path.edges.begin()+e1,ee1);
	path.edges.insert(path.edges.begin()+e1+1,e);
	path.edges.insert(path.edges.begin()+e1+2,ee2);
	return true;
      }
    }
    return false;
  }

  Real ShortcutCost(Real u1,Real u2) 
  {
    Config q1,q2;
    int e1=path.Eval2(u1,q1);
    int e2=path.Eval2(u2,q2);
    return cspace->Distance(q1,q2)*kShortcutCostCoefficient;
  }

  void TestRandomShortcuts(int num)
  {
    Assert(hasPath);
    ofstream out("random-shortcuts.txt");
    Real cost=0;
    out<<"# N random shortcuts     path length    cost"<<endl;
    Real initialLength = path.Length();
    Real minimalLength = cspace->Distance(path.Start(),path.End());
    reductionRandomByIters.resize(num+1);
    costRandomByIters.resize(num+1);
    for(int iters=0;iters<num;iters++) {
      out<<iters<<" "<<path.Length()<<" "<<cost<<endl;
      reductionRandomByIters[iters].collect(path.Length()/minimalLength);
      costRandomByIters[iters].collect(cost);

      Real u1 = Rand();
      Real u2 = Rand();
      TryShortcut(u1,u2);
      cost += ShortcutCost(u1,u2);
    }
    out<<num<<" "<<path.Length()<<" "<<cost<<endl;
    reductionRandomByIters[num].collect(path.Length()/minimalLength);
    costRandomByIters[num].collect(cost);
    out.close();
  }

  struct CandidateShortcut
  {
    Real t1,t2; //parameter on path
    Real u1,u2; //parameter on edge
    int e1,e2;
    Config q1,q2;
    Real length,pathLength;
    Real closestFailed;
  };

  bool TryShortcut(const CandidateShortcut& s,Config* failedConfig=NULL)
  {    
    if(s.e1 == s.e2) //on same segment
      return false;
    Assert(s.e1 < s.e2);
    if(!cspace->IsFeasible(s.q1)) {
      if(failedConfig) *failedConfig = s.q1;
      //cout<<"Precision error with edge checker... config not feasible"<<endl;
      return false;
    }
    if(!cspace->IsFeasible(s.q2)) {
      if(failedConfig) *failedConfig = s.q1;
      //cout<<"Precision error with edge checker... config not feasible"<<endl;
      return false;
    }
    EdgePlanner* e=cspace->LocalPlanner(s.q1,s.q2);
    if(!e->IsVisible()) {
      if(failedConfig) *failedConfig = dynamic_cast<BisectionEpsilonEdgePlanner*>(e)->InfeasibleConfig();
      delete e;
    }
    else {
      EdgePlanner* ee1=cspace->LocalPlanner(path.edges[s.e1]->Start(),s.q1);
      EdgePlanner* ee2=cspace->LocalPlanner(s.q2,path.edges[s.e2]->End());
      if(!ee1->IsVisible()) {
	if(failedConfig) *failedConfig = dynamic_cast<BisectionEpsilonEdgePlanner*>(ee1)->InfeasibleConfig();
	delete ee1;
	delete ee2;
	delete e;
      }
      else if(!ee2->IsVisible()) {
	//cout<<"Precision error with edge checker..."<<endl;
	if(failedConfig) *failedConfig = dynamic_cast<BisectionEpsilonEdgePlanner*>(ee2)->InfeasibleConfig();
	delete ee1;
	delete ee2;
	delete e;
      }
      else {
	//replace segment
	path.edges.erase(path.edges.begin()+s.e1,path.edges.begin()+s.e2+1);
	path.edges.insert(path.edges.begin()+s.e1,ee1);
	path.edges.insert(path.edges.begin()+s.e1+1,e);
	path.edges.insert(path.edges.begin()+s.e1+2,ee2);
	return true;
      }
    }
    return false;
  }

  inline Real ShortcutCost(const CandidateShortcut& s) const
  {
    return s.length*kShortcutCostCoefficient;
  }

  inline Real ShortcutProbability(const CandidateShortcut& s)
  {
    if(distanceBuckets.empty()) {
      ifstream in("distances.txt");
      if(in) printf("Reading in distances from distances.txt...\n");
      while(in) {
	char buf[256];
	in.getline(buf,256);
	if(!in) break;
	if(buf[0] == '#') //comment char
	  continue;
	stringstream ss;
	ss.str(buf);
	Real bucket,val;
	ss>>bucket;
	if(!ss) {
	  //probably the last one (inf)
	  distanceBuckets.push_back(Inf);
	  ss.setstate(ios::goodbit);
	  ss.ignore(100,' ');
	}
	else {
	  distanceBuckets.push_back(bucket);
	}
	ss>>val;
	distanceCounts.resize(distanceCounts.size()+1);
	distanceCounts.back().collect(val);
      }
    }
    if(distanceBuckets.empty())
      return Exp(-Pow(s.length,2.0)*3.0);

    vector<Real>::iterator i=std::lower_bound(distanceBuckets.begin(),distanceBuckets.end(),s.length);
    Assert(i != distanceBuckets.end());
    int index=i-distanceBuckets.begin();
    return distanceCounts[index].average();
  }

  void EvaluateShortcut(CandidateShortcut& s,const vector<Real>& cumLength)
  {
    path.edges[s.e1]->Eval(s.u1,s.q1);
    path.edges[s.e2]->Eval(s.u2,s.q2);
    s.length = cspace->Distance(s.q1,s.q2);
    Real plen1=cumLength[s.e1]+s.u1*(cumLength[s.e1+1]-cumLength[s.e1]);
    Real plen2=cumLength[s.e2]+s.u2*(cumLength[s.e2+1]-cumLength[s.e2]);
    s.pathLength = plen2-plen1;
    s.closestFailed = Inf;
    if(s.pathLength < s.length-1e-5) {
      printf("Uh... cumulative path length is less than distance between pts\n");
      printf("Points %g %g, %g %g\n",s.q1[0],s.q1[1],s.q2[0],s.q2[1]);
      printf("len %g, path len %g\n",s.length,s.pathLength);
      printf("edge cum lengths %g->%g, %g->%g\n",cumLength[s.e1],cumLength[s.e1+1],cumLength[s.e2],cumLength[s.e2+1]);
      printf("edge params %g, %g\n",s.u1,s.u2);
    }
    Assert(s.pathLength >= s.length-1e-5);
  }

  void SampleShortcuts(vector<CandidateShortcut>& shortcuts)
  {
    vector<Real> cumLength(path.edges.size()+1);
    cumLength[0] = 0;
    for(size_t i=0;i<path.edges.size();i++)
      cumLength[i+1] = cumLength[i] + cspace->Distance(path.edges[i]->Start(),path.edges[i]->End());
    Assert(hasPath);
    for(int i=0;i<shortcuts.size();i++) {
      CandidateShortcut s;
      s.e1 = RandInt(path.edges.size());
      s.e2 = RandInt(path.edges.size());
      s.u1 = Rand();
      s.u2 = Rand();
      if(s.e1 > s.e2) swap(s.e1,s.e2);
      else if(s.e1 == s.e2 && s.u1 > s.u2) swap(s.u1,s.u2);
      s.t1 = Real(s.u1 + s.e1)/path.edges.size();
      s.t2 = Real(s.u2 + s.e2)/path.edges.size();
      EvaluateShortcut(s,cumLength);
      shortcuts[i] = s;
    }
  }

  Real ShortcutDifference(const CandidateShortcut& a,const CandidateShortcut& b)
  {
    Real overlap1=Max(a.t1,b.t1),
      overlap2=Min(a.t2,b.t2);
    if(overlap1 >= overlap2) return Inf;
    Real fraction = (overlap2 - overlap1)/(a.t2-a.t1);
    //if there was one failure along j (distributed accordingly to the 
    //distance weighted probabilities, what's the probability that it
    //will affect i? 
    
    //Real cosangle = CosAngle(cspace,a.q1,a.q2,b.q1,b.q2);
    Config qi,qj;
    cspace->Interpolate(a.q1,a.q2,(overlap1+overlap2-a.t1)/(a.t2-a.t1),qi);
    cspace->Interpolate(b.q1,b.q2,(overlap1+overlap2-b.t1)/(b.t2-b.t1),qj);
    return cspace->Distance(qi,qj);
  }


  Real ShortcutDifference(const CandidateShortcut& a,const Config& x)
  {
    //distance between point x and seg a,b in euclidean space
    //d2 = |x-a-(b-a)dot(x-a,b-a)/D(b,a)^2|^2
    //   = dot(x-a,x-a)-dot(x-a,b-a)^2/D(b,a)^2
    //   = D(x,a)^2-dot(x-a,b-a)^2/D(b,a)^2
    //dot(x-a,b-a)=1/2[D^2(x,b)+D^2(a,a)-D^2(x,a)-D^2(a,b)]
    //=1/2[D^2(x,b)-D^2(x,a)-D^2(a,b)]
    Real dab2=Sqr(cspace->Distance(a.q1,a.q2));
    Real dxa2=Sqr(cspace->Distance(a.q1,x));
    Real dxb2=Sqr(cspace->Distance(a.q2,x));
    if(dxb2 < dxa2-dab2) return Sqrt(dxa2);
    else if(dxa2 < dxb2-dab2) return Sqrt(dxb2);
    return Sqrt(dxa2 - 0.25*Sqr(dxb2-dxa2-dab2)/dab2);
    //dxa2 - 0.25*Sqr(dxb2-dxa2-dab2)/dab2 
    //dxa2 - 0.25*[(dxb2-dxa2-dab2)(dxb2-dxa2)/dab2-(dxb2-dxa2-dab2)] 
    //+0.5[dxa2+dxb2-dab2]-0.25*[(dxb2-dxa2)(dxb2-dxa2)/dab2]
    //so it is symmetric

    //if dot(x-a,b-a)<0 then closest is a
    //if dot(x-b,a-b)<0 then closest is b
  }

  void UpdateDistanceToFailed(CandidateShortcut& s,const vector<CandidateShortcut>& failed) {
    s.closestFailed = Inf;
    for(size_t j=0;j<failed.size();j++) {
      Real d = ShortcutDifference(s,failed[j]);
      s.closestFailed = Min(d,s.closestFailed);
    }
  }

  void UpdateDistanceToFailed(CandidateShortcut& s,const vector<Config>& failed) {
    s.closestFailed = Inf;
    for(size_t j=0;j<failed.size();j++) {
      Real d = ShortcutDifference(s,failed[j]);
      s.closestFailed = Min(d,s.closestFailed);
    }
  }

  void UpdateWithFailure(vector<CandidateShortcut>& samples,const CandidateShortcut& failed) {
    for(size_t j=0;j<samples.size();j++) {
      Real d=ShortcutDifference(samples[j],failed);
      samples[j].closestFailed = Min(d,samples[j].closestFailed);
    }
  }

  void UpdateWithFailure(vector<CandidateShortcut>& samples,const Config& failed) {
    for(size_t j=0;j<samples.size();j++) {
      Real d=ShortcutDifference(samples[j],failed);
      samples[j].closestFailed = Min(d,samples[j].closestFailed);
    }
  }

  void TestAdaptiveShortcuts(int num)
  {
    Assert(hasPath);
    ofstream out("adaptive-shortcuts.txt");
    vector<CandidateShortcut> sampledShortcuts(num);
    SampleShortcuts(sampledShortcuts);
    Real cost = 0;
    Real initialLength = path.Length();
    Real minimalLength = cspace->Distance(path.Start(),path.End());
    //vector<CandidateShortcut> failedShortcuts;
    //failedShortcuts.reserve(num);
    vector<Config> failedConfigs;
    failedConfigs.reserve(num);
    reductionAdaptiveByIters.resize(num+1);
    costAdaptiveByIters.resize(num+1);
    out<<"# N adaptive shortcuts     path length    cost"<<endl;
    for(int iters=0;iters<num;iters++) {
      printf("iter %d, %d shortcuts, %d failed\n",iters,sampledShortcuts.size(),failedConfigs.size());
      out<<iters<<" "<<path.Length()<<" "<<cost<<endl;
      reductionAdaptiveByIters[iters].collect(path.Length()/minimalLength);
      costAdaptiveByIters[iters].collect(cost);

      Real bestVal = 0;
      int best = -1;
      for(size_t i=0;i<sampledShortcuts.size();i++) {
	Real p=ShortcutProbability(sampledShortcuts[i]); //base probability
	Real minDist = sampledShortcuts[i].closestFailed;
	//printf("min dist %g\n",minDist);
	p *= 1.0 - Exp(-minDist*10);

	Real val = p*(sampledShortcuts[i].pathLength - sampledShortcuts[i].length) - ShortcutCost(sampledShortcuts[i]);
	if(val > bestVal)
	  best = (int)i;
      }
      if(best < 0) {
	iters++;
	while(iters <= num) {
	  reductionAdaptiveByIters[iters].collect(path.Length()/minimalLength);
	  costAdaptiveByIters[iters].collect(cost);
	  iters++;
	}
	return;
      }
      CandidateShortcut s=sampledShortcuts[best];
      Config failedConfig;
      bool res=TryShortcut(s,&failedConfig);
      cost += ShortcutCost(s);
      sampledShortcuts[best] = sampledShortcuts.back();
      sampledShortcuts.resize(sampledShortcuts.size()-1);
      if(res) {
	//succeeded -- now update the sampled shortcuts
	int numReducedEdges = (s.e2-s.e1-2);
	vector<Real> cumLength(path.edges.size()+1);
	cumLength[0] = 0;
	for(size_t i=0;i<path.edges.size();i++)
	  cumLength[i+1] = cumLength[i] + cspace->Distance(path.edges[i]->Start(),path.edges[i]->End());
	for(size_t i=0;i<sampledShortcuts.size();i++) {
	  bool p1_past_s1=false;
	  bool p1_past_s2=false;
	  bool p2_past_s1=false;
	  bool p2_past_s2=false;
	  if(sampledShortcuts[i].t1 > s.t1) p1_past_s1=true;
	  if(sampledShortcuts[i].t1 > s.t2) p1_past_s2=true;
	  if(sampledShortcuts[i].t2 > s.t1) p2_past_s1=true;
	  if(sampledShortcuts[i].t2 > s.t2) p2_past_s2=true;
	  if(sampledShortcuts[i].e1 == s.e1)   //it's on the split segment
	    sampledShortcuts[i].u1 = sampledShortcuts[i].u1/s.u1;
	  if(sampledShortcuts[i].e2 == s.e1)   //it's on the split segment
	    sampledShortcuts[i].u2 = sampledShortcuts[i].u2/s.u1;
	  if(sampledShortcuts[i].e1 == s.e2)  //it's on the split segment
	    sampledShortcuts[i].u1 = 1.0-(1.0-sampledShortcuts[i].u1)/(1.0-s.u2);
	  if(sampledShortcuts[i].e2 == s.e2)  //it's on the split segment
	    sampledShortcuts[i].u2 = 1.0-(1.0-sampledShortcuts[i].u2)/(1.0-s.u2);
	  if(sampledShortcuts[i].u1 > 1.0)
	    Assert(p1_past_s1&&!p1_past_s2);
	  if(sampledShortcuts[i].u2 > 1.0)
	    Assert(p2_past_s1&&!p2_past_s2);
	  if(p1_past_s2) {  //past shortcut
	    sampledShortcuts[i].e1 -= numReducedEdges;
	  }
	  else if(p1_past_s1) {  //endpoint in shortcut
	    sampledShortcuts[i].u1 = (sampledShortcuts[i].t1 - s.t1)/(s.t2-s.t1);
	    sampledShortcuts[i].e1 = s.e1+1;
	  }
	  if(p2_past_s2) {
	    sampledShortcuts[i].e2 -= numReducedEdges;
	  }
	  else if(p2_past_s1) {  //endpoint in shortcut
	    sampledShortcuts[i].u2 = (sampledShortcuts[i].t2 - s.t1)/(s.t2-s.t1);
	    sampledShortcuts[i].e2 = s.e1+1;
	  }
	  sampledShortcuts[i].t1 = (sampledShortcuts[i].u1 + Real(sampledShortcuts[i].e1))/path.edges.size();
	  sampledShortcuts[i].t2 = (sampledShortcuts[i].u2 + Real(sampledShortcuts[i].e2))/path.edges.size();
	  bool overlap = !(!p2_past_s1 || p1_past_s2);
	  if(overlap) {
	    //UpdateDistanceToFailed(sampledShortcuts[i],failedShortcuts);
	    UpdateDistanceToFailed(sampledShortcuts[i],failedConfigs);
	    EvaluateShortcut(sampledShortcuts[i],cumLength);
	  }
	}
      }
      else {
	//record information to avoid repeating similar problems
	//failedShortcuts.push_back(s);
	//UpdateWithFailure(sampledShortcuts,s);
	failedConfigs.push_back(failedConfig);
	UpdateWithFailure(sampledShortcuts,failedConfig);
      }
    }
    reductionAdaptiveByIters[num].collect(path.Length()/minimalLength);
    costAdaptiveByIters[num].collect(cost);
    out<<num<<" "<<path.Length()<<" "<<cost<<endl;
    out.close();
  }

  virtual void Handle_Keypress(unsigned char key,int x,int y)
  {
    if(key == 't') {
      TestShortcuts(100);

      ofstream out("shortcuts.txt");
      for(int i=0;i<counts.m;i++) {
	for(int j=0;j<counts.n;j++)
	  out<<counts(i,j).average()<<" ";
	out<<endl;
      }
      out.close();
      Refresh();
    }
    else if(key == ' ') {
      TestDistances(100000);

      ofstream out("distances.txt");
      for(int i=0;i<distanceCounts.size();i++) {
	out<<distanceBuckets[i]<<" "<<distanceCounts[i].average()<<endl;
      }
      out.close();
      Refresh();
    }
    else if(key == 's') {
      if(!hasPath) {
	if(!SampleProblem()) return;
      }
      MilestonePath savedPath = path;
      TestRandomShortcuts(1000);
      path = savedPath;
      TestAdaptiveShortcuts(10000);
      Refresh();
    }
    else if(key == 'S') {
      for(int iters=0;iters<200;iters++) {
	printf("Problem %d\n",iters);
	if(!SampleProblem()) continue;
	MilestonePath savedPath = path;
	TestRandomShortcuts(1000);
	path = savedPath;
	TestAdaptiveShortcuts(10000);
      }
      {
	ofstream out("random-shortcuts-1000.txt");
	out<<"# iter, red avg, min, max, std, cost avg, min, max, std"<<endl;
	for(size_t i=0;i<reductionRandomByIters.size();i++) {
	  if(reductionRandomByIters[i].number() == 0) break;
	  out<<i<<" ";
	  out<<reductionRandomByIters[i].average()<<" "<<reductionRandomByIters[i].minimum()<<" "<<reductionRandomByIters[i].maximum()<<" "<<reductionRandomByIters[i].stddev()<<"  ";
	  out<<costRandomByIters[i].average()<<" "<<costRandomByIters[i].minimum()<<" "<<costRandomByIters[i].maximum()<<" "<<costRandomByIters[i].stddev()<<endl;
	}
	out.close();
      }
      {
	ofstream out("adaptive-shortcuts-1000.txt");
	out<<"# iter, red avg, min, max, std, cost avg, min, max, std"<<endl;
	for(size_t i=0;i<reductionAdaptiveByIters.size();i++) {
	  if(reductionAdaptiveByIters[i].number() == 0) break;
	  out<<i<<" ";
	  out<<reductionAdaptiveByIters[i].average()<<" "<<reductionAdaptiveByIters[i].minimum()<<" "<<reductionAdaptiveByIters[i].maximum()<<" "<<reductionAdaptiveByIters[i].stddev()<<"   ";
	  out<<costAdaptiveByIters[i].average()<<" "<<costAdaptiveByIters[i].minimum()<<" "<<costAdaptiveByIters[i].maximum()<<" "<<costAdaptiveByIters[i].stddev()<<endl;
	}
	out.close();
      }
      Refresh();
    }
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
	rrt.Init(start,goal);
	hasPath = false;
      }
      else if(!hasPath) {
	for(int iters=0;iters<100;iters++) {
	  if(rrt.Plan()) {
	    rrt.CreatePath(path);
	    hasPath = true;
	    break;
	  }
	}
      }
      else if(hasPath) {
	hasStart=false;
	hasGoal=false;
	hasPath=false;
      }
      Refresh();
    }
  }
};
