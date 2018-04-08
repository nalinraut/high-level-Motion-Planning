#include <KrisLibrary/GLdraw/GLUTNavigationProgram.h>
#include <KrisLibrary/planning/CSpace.h>
#include <KrisLibrary/planning/RobotKinematics3D.h>
#include <KrisLibrary/planning/IKFunctions.h>
#include <KrisLibrary/utils/permutation.h>
#include <KrisLibrary/math/angle.h>
#include <KrisLibrary/math/random.h>
using namespace Math;

const static int nlegs=3;

//find the angle that minimizes the distance between a and b
Real MinimalRotation(const Vector2& a,const Vector2& b)
{
  Real p = a.x*b.x + a.y*b.y;
  Real q = a.x*b.y - a.y*b.x;
  //maximize cos(theta) p - sin(theta) q
  Real c,d;
  TransformCosSin_Cos(p,q,c,d);
  //now maximize c*cos(theta+d)
  return -d;
}

struct ClosedChainCSpace : public CSpace
{
  RobotKinematics3D robot;
  vector<int> eeLink;
  vector<Vector3> eeLocalPos,eeDesPos;
  IKProblem ikProblem;

  ClosedChainCSpace()
  {
    robot.Initialize(2+2*nlegs);
    robot.parents[0]=-1;
    robot.parents[1]=0;
    for(int i=0;i<nlegs;i++) {
      robot.parents[2+i*2] = 1;
      robot.parents[2+i*2+1] = 2+i*2;
    }
    robot.links[0].T0_Parent.setIdentity();
    robot.links[1].T0_Parent.setIdentity();
    robot.links[0].SetTranslationJoint(Vector3(1,0,0));
    robot.links[1].SetTranslationJoint(Vector3(0,1,0));
    for(int i=0;i<nlegs;i++) {
      Real angle = TwoPi*i/Real(nlegs);
      robot.links[2+i*2].T0_Parent.R.setRotateZ(angle);
      robot.links[2+i*2].T0_Parent.t.set(0.5*Cos(angle),0.5*Sin(angle),0);
      robot.links[2+i*2+1].T0_Parent.R.setIdentity();
      robot.links[2+i*2+1].T0_Parent.t.set(1,0,0);
      robot.links[2+i*2].SetRotationJoint(Vector3(0,0,1));
      robot.links[2+i*2+1].SetRotationJoint(Vector3(0,0,1));
    }
    robot.qMin[0] = -Inf;
    robot.qMin[1] = -Inf;
    robot.qMax[0] = Inf;
    robot.qMax[1] = Inf;
    for(int i=0;i<nlegs;i++) {
      robot.qMin[2+i*2]=-Inf;
      robot.qMin[2+i*2+1]=-Inf;
      robot.qMax[2+i*2]=Inf;
      robot.qMax[2+i*2+1]=Inf;
    }
    eeLink.resize(nlegs);
    eeLocalPos.resize(nlegs);
    eeDesPos.resize(nlegs);
    ikProblem.goals.resize(nlegs);
    for(int i=0;i<nlegs;i++) {
      eeLink[i] = 2*i+3;
      eeLocalPos[i].set(1,0,0);
      Real angle = TwoPi*i/Real(nlegs);
      eeDesPos[i].set(1.5*Cos(angle),1.5*Sin(angle),0);
      ikProblem.goals[i].link = eeLink[i];
      ikProblem.goals[i].localPosition = eeLocalPos[i];
      ikProblem.goals[i].SetLinearPosition(eeDesPos[i],Vector3(0,0,1));
    }
  }

  int NumDifferingKneeBends(const Vector& a,const Vector& b) const
  {
    int n=0;
    for(int i=0;i<nlegs;i++) {
      if(Sign(AngleDiff(a(2+i*2+1),0))*Sign(AngleDiff(b(2+i*2+1),0)) == -1) n++;
    }
    return n;
  }

  void GetKneeBends(const Config& a,vector<int>& bends) const
  {
    bends.resize(nlegs);
    for(int i=0;i<nlegs;i++) 
      bends[i] = (int)Sign(AngleDiff(a(2+i*2+1),0));
  }

  bool SolveNumericalIK()
  {
    int iters=100;
    return SolveIK(robot,ikProblem,1e-3,iters,0);
  }

  bool SolveAnalyticIK(int leg,int kneeBend)
  {
    bool solved=true;
    int link = 2+leg*2;
    Vector3 temp;
    robot.q(link) = 0;
    robot.UpdateFrames();
    robot.links[link].T_World.mulPointInverse(eeDesPos[leg],temp);
    //find knee bend
    Real L2=temp.normSquared();
    Real a;
    if(L2 > 4.0) {
      a = 0;
      solved=false;
    }
    else {
      Real cosa = Clamp(L2*0.5-1.0,-1.0,1.0);
      a= Acos(cosa);
      if(kneeBend == 0) a *= (RandBool()?1.0:-1.0);  //random knee bend
      else if(kneeBend == -1) a = -a;
    }
    robot.q[link+1] = AngleNormalize(a);
    //find hip angle
    robot.q[link] = AngleNormalize(MinimalRotation(Vector2(1+Cos(a),Sin(a)),Vector2(temp.x,temp.y)));
    return solved;
  }

  bool SolveAnalyticIK(const vector<int>& kneeBends)
  {
    for(int i=2;i<robot.q.n;i++)
      robot.q(i) = 0;
    robot.UpdateFrames();
    bool solved=true;
    for(int i=0;i<nlegs;i++) {
      if(!SolveAnalyticIK(i,kneeBends[i])) solved=false;
    }
    return solved;
  }

  virtual void Sample(Config& q)
  {
    robot.q(0) = Rand(-2,2);
    robot.q(1) = Rand(-2,2);
    vector<int> kneeBends(nlegs,0);
    SolveAnalyticIK(kneeBends);
    q = robot.q;
  }

  Real ContactDistance(const Config& q) {
    Real maxDist = 0;
    robot.UpdateConfig(q);
    for(int i=0;i<nlegs;i++) {
      Vector3 d=robot.links[eeLink[i]].T_World*eeLocalPos[i]-eeDesPos[i];
      maxDist = Max(maxDist,Abs(d.x));
      maxDist = Max(maxDist,Abs(d.y));
    }
    return maxDist;
  }
  virtual bool IsFeasible(const Config& q)
  {
    robot.UpdateConfig(q);
    for(int i=0;i<nlegs;i++) {
      Vector3 d=robot.links[eeLink[i]].T_World*eeLocalPos[i]-eeDesPos[i];
      if(Abs(d.x) > 1e-3) return false;
      if(Abs(d.y) > 1e-3) return false;
    }
    return true;
  }
  virtual void Midpoint(const Config& a,const Config& b,Config& q)
  {
    q.resize(a.n);
    for(int i=0;i<2;i++)
      q[i] = 0.5*(a(i)+b(i));
    for(int i=2;i<a.n;i++)
      q[i] = AngleInterp(a(i),b(i),0.5);
    robot.UpdateConfig(q);
    if(SolveNumericalIK()) Assert(IsFeasible(robot.q));
    q = robot.q;
  }
  virtual void Interpolate(const Config& a,const Config& b,Real u,Config& q)
  {
    q.resize(a.n);
    for(int i=0;i<2;i++)
      q[i] = a(i)+u*(b(i)-a(i));
    for(int i=2;i<a.n;i++)
      q[i] = AngleInterp(a(i),b(i),u);
    robot.UpdateConfig(q);
    if(SolveNumericalIK()) Assert(IsFeasible(robot.q));
    q = robot.q;
  }

  virtual EdgePlanner* LocalPlanner(const Config& a,const Config& b)
  {
    return new BisectionEpsilonEdgePlanner(a,b,this,5e-2);
  }
  virtual bool AnalyticConnect_SameKnee(const Config& a,const Config& b,const vector<int>& kneeBends)
  {
    Assert(NumDifferingKneeBends(a,b)==0);    
    Real res=1e-2;
    int divs = (int)Ceil(Sqrt(Sqr(b(0)-a(0))+Sqr(b(1)-a(1)))/res);
    for(int i=1;i<divs;i++) {
      Real u=Real(i)/Real(divs);
      robot.q(0) = a(0)+u*(b(0)-a(0));
      robot.q(1) = a(1)+u*(b(1)-a(1));
      if(!SolveAnalyticIK(kneeBends)) return false;
    }
    return true;
  }
  virtual bool AnalyticConnect(const Config& a,const Config& b)
  {
    vector<int> abends,bbends;
    GetKneeBends(a,abends);
    GetKneeBends(b,bbends);
    vector<int> order;
    for(int i=0;i<nlegs;i++)
      if(abends[i]*bbends[i] == -1) order.push_back(i);
    for(int i=0;i<nlegs;i++) {
      if(abends[i] == 0) abends[i] = bbends[i];
      if(bbends[i] == 0) bbends[i] = abends[i];
      //both straight legged
      if(abends[i] == 0) abends[i] = bbends[i] = 1;
    }
    /*
    cout<<"Analytic connect: "<<endl;
    cout<<"a= "<<a<<endl;
    for(size_t i=0;i<nlegs;i++)
      cout<<abends[i]<<" ";
    cout<<endl;
    cout<<"b= "<<b<<endl;
    for(size_t i=0;i<nlegs;i++)
      cout<<bbends[i]<<" ";
    cout<<endl;
    */
    RandomlyPermute(order);
    Config q=a,qnext;
    for(size_t i=0;i<order.size();i++) {
      Real u=Real(i)/(order.size()+1);
      int link = 2+2*order[i];
      //switch knee bend i
      robot.UpdateConfig(a);
      Vector3 dir = robot.links[link+1].T_World.R*Vector3(1,0,0);
      Real anglea = Atan2(dir.y,dir.x);
      robot.UpdateConfig(b);
      dir = robot.links[link+1].T_World.R*Vector3(1,0,0);
      Real angleb = Atan2(dir.y,dir.x);
      Real angle = AngleInterp(AngleNormalize(anglea),AngleNormalize(angleb),u);
      Vector3 hip_pos(-2.0*Cos(angle),-2.0*Sin(angle),0);
      hip_pos += eeDesPos[order[i]];
      Vector3 center_pos = hip_pos - robot.links[link].T0_Parent.t;
      qnext = q;
      qnext[0] = center_pos.x;
      qnext[1] = center_pos.y;
      Real baseAngle = Real(order[i])*TwoPi/nlegs;
      qnext[link] = AngleNormalize(AngleDiff(angle,baseAngle));
      qnext[link+1] = 0;
      robot.q = qnext;
      robot.UpdateFrames();
      for(int leg=0;leg<nlegs;leg++) {
	if(leg != order[i])
	  if(!SolveAnalyticIK(leg,abends[leg])) {
	    return false;
	  }
      }
      qnext = robot.q;
      if(!IsFeasible(qnext)) {
	cout<<"Whoops... something wrong: "<<ContactDistance(qnext)<<endl;
	getchar();
      }
      if(!AnalyticConnect_SameKnee(q,qnext,abends)) {
	return false;
      }
      q = qnext;
      abends[order[i]] = bbends[order[i]];
    }
    if(!AnalyticConnect_SameKnee(q,b,abends)) return false;
    return true;
  }
};

struct ClosedChainPlannerProgram : public GLUTNavigationProgram
{
  ClosedChainCSpace cspace;
  vector<Config> configs;
  int thirdDim;

  ClosedChainPlannerProgram()
  {
    thirdDim = 3;
    const static int maxSamples = 1000;
    for(int iters=0;iters<maxSamples;iters++) {
      Config temp;
      cspace.Sample(temp);
      if(cspace.IsFeasible(temp)) configs.push_back(temp);
    }
    cout<<configs.size()<<" sampled out of "<<maxSamples<<endl;
  }

  virtual bool Initialize()
  {
    glClearColor(1,1,1,1);
    viewport.scale = 5.0;
    return GLUTNavigationProgram::Initialize();
  }

  virtual void RenderWorld() {
    glDisable(GL_LIGHTING);
    glColor3f(1,0,0);
    glPointSize(5.0);
    glBegin(GL_POINTS);
    for(size_t i=0;i<cspace.eeDesPos.size();i++)
      glVertex3v(cspace.eeDesPos[i]);
    glEnd();
    glPointSize(3.0);
    glColor3f(0,0,0);
    glBegin(GL_POINTS);
    for(size_t i=0;i<configs.size();i++) {
      glVertex3d(configs[i][0],configs[i][1],configs[i][thirdDim]);
    }
    glEnd();
  }
  virtual void Handle_Keypress(unsigned char c,int x,int y) {
    if(c == ' ') {
      thirdDim++;
      if(thirdDim >= configs[0].n)
	thirdDim = 0;
      Refresh();
    }
    else if(c == 'k') {
      int nd=0;
      int n=0;
      for(size_t i=0;i<configs.size();i++)
	for(size_t j=0;j<i;j++) {
	  nd+=cspace.NumDifferingKneeBends(configs[i],configs[j]);
	  n++;
	}
      cout<<nd<<" differing knee bends out of "<<n<<", average "<<Real(nd)/n<<endl;
    }
    else if(c == 'e') {
      vector<int> n(nlegs+1,0);
      vector<int> ne(nlegs+1,0);
      vector<int> ne2(nlegs+1,0);
      Real failDist = 0;
      int numFailures=0;
      const static int numSamples = 1000;
      for(int iters=0;iters<numSamples;iters++) {
	if((iters*100)%numSamples==0) cout<<(iters*100)/numSamples<<"%"<<endl;
	int i = RandInt(configs.size());
	int j = RandInt(configs.size());
	int nd=cspace.NumDifferingKneeBends(configs[i],configs[j]);
	n[nd]++;
	BisectionEpsilonEdgePlanner* e = dynamic_cast<BisectionEpsilonEdgePlanner*>(cspace.LocalPlanner(configs[i],configs[j]));
	if(e->IsVisible()) ne[nd]++;
	else {
	  failDist += cspace.ContactDistance(e->InfeasibleConfig());
	  numFailures++;
	}
	delete e;
	
	if(cspace.AnalyticConnect(configs[i],configs[j]))
	  ne2[nd]++;
      }
      for(size_t i=0;i<n.size();i++) {
	cout<<i<<" bends: numerical "<<ne[i]<<" of "<<n[i]<<"."<<endl;
	cout<<" analytic "<<ne2[i]<<" of "<<n[i]<<"."<<endl;
      }
      cout<<"Failure distance: "<<failDist / numFailures<<endl;
    }
    else if(c == 'd') {
      vector<int> n(nlegs+1,0);
      vector<int> ne(nlegs+1,0);
      vector<int> ne2(nlegs+1,0);
      Real failDist = 0;
      int numFailures=0;
      for(size_t i=0;i<configs.size();i++) {
	for(size_t j=0;j<i;j++) {
	  if(cspace.Distance(configs[i],configs[j]) < 2.0) {
	    int nd=cspace.NumDifferingKneeBends(configs[i],configs[j]);
	    n[nd]++;
	    BisectionEpsilonEdgePlanner* e = dynamic_cast<BisectionEpsilonEdgePlanner*>(cspace.LocalPlanner(configs[i],configs[j]));
	    if(e->IsVisible()) ne[nd]++;
	    else {
	      failDist += cspace.ContactDistance(e->InfeasibleConfig());
	      numFailures++;
	    }
	    delete e;
	    
	    if(cspace.AnalyticConnect(configs[i],configs[j]))
	      ne2[nd]++;
	  }
	}
      }
      for(size_t i=0;i<n.size();i++) {
	cout<<i<<" bends: numerical "<<ne[i]<<" of "<<n[i]<<"."<<endl;
	cout<<" analytic "<<ne2[i]<<" of "<<n[i]<<"."<<endl;
      }
      cout<<"Failure distance: "<<failDist / numFailures<<endl;
    }

  }
};

