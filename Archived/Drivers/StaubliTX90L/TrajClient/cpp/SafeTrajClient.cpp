#include "SafeTrajClient.h"
#include <spline/PiecewisePolynomial.h>
#include "ParabolicRamp.h"
#include "Timer.h"
#include <assert.h>
using namespace std;
using namespace ParabolicRamp;
using namespace Spline;

Real kVelSafetyMultiplier = 0.99;
Real kAccSafetyMultiplier = 0.99;

PiecewisePolynomial Cast(const ParabolicRamp1D& ramp)
{
  PiecewisePolynomial res;
  res.segments.resize(3);
  res.timeShift.resize(3);
  res.times.resize(4);
  res.times[0] = 0;
  res.times[1] = ramp.tswitch1;
  res.times[2] = ramp.tswitch2;
  res.times[3] = ramp.ttotal;
  res.segments[0].Resize(3);
  res.segments[0].coef[0] = ramp.x0;
  res.segments[0].coef[1] = ramp.dx0;
  res.segments[0].coef[2] = 0.5*ramp.a1;
  res.timeShift[0] = 0;
  res.segments[1].Resize(2);
  res.segments[1].coef[0] = ramp.Evaluate(ramp.tswitch1);
  res.segments[1].coef[1] = ramp.Derivative(ramp.tswitch1);
  res.timeShift[1] = ramp.tswitch1;
  res.segments[2].Resize(3);
  res.segments[2].coef[0] = ramp.x1;
  res.segments[2].coef[1] = ramp.dx1;
  res.segments[2].coef[2] = 0.5*ramp.a2;
  res.timeShift[2] = ramp.ttotal;

  if(ramp.ttotal == ramp.tswitch2) {
    res.times.erase(--res.times.end());
    res.segments.erase(--res.segments.end());
    res.timeShift.erase(--res.timeShift.end());
  }
  if(ramp.tswitch1 == ramp.tswitch2) {
    res.times.erase(++res.times.begin());
    res.segments.erase(++res.segments.begin());
    res.timeShift.erase(++res.timeShift.begin());
  }
  if(ramp.tswitch1 == 0 && res.segments.size()>1) {
    res.times.erase(res.times.begin());
    res.segments.erase(res.segments.begin());
    res.timeShift.erase(res.timeShift.begin());
  }
  return res;
}

PiecewisePolynomialND Cast(const ParabolicRampND& ramp)
{
  PiecewisePolynomialND res;
  res.elements.resize(ramp.ramps.size());
  for(size_t i=0;i<ramp.ramps.size();i++)
    res.elements[i] = Cast(ramp.ramps[i]);
  return res;
}

//concatenates the ramps
PiecewisePolynomial Cast(const vector<ParabolicRamp1D>& ramps)
{
  assert(!ramps.empty());
  if(ramps.size()==1) return Cast(ramps[0]);
  PiecewisePolynomial p = Cast(ramps[0]);
  for(size_t i=1;i<ramps.size();i++)
    p.Concat(Cast(ramps[i]),true);
  return p;
}

PiecewisePolynomialND Cast(const vector<vector<ParabolicRamp1D> >& ramps)
{
  PiecewisePolynomialND res;
  res.elements.resize(ramps.size());
  for(size_t i=0;i<ramps.size();i++)
    res.elements[i] = Cast(ramps[i]);
  return res;
}


SafeTrajClient::SafeTrajClient(const char* host,int port)
  :milestoneBatch(100),nonblock(false),virtualController(false),sendIndex(0)
{
  if(host) {
    bool res=Connect(host,port);
  }
}

void SafeTrajClient::Disconnect()
{
  virtualController = false;
  t.Disconnect();
  mirror.Disconnect();

  sendIndex = 0;
  sendqs.clear();
  sendts.clear();
}

bool SafeTrajClient::IsConnected() const
{
  return t.IsConnected();
}

bool SafeTrajClient::Connect(const char* host,int port)
{
  assert(!t.IsConnected());
  assert(!virtualController);

  if(!t.Connect(host,port)) return false;
  if(!mirror.Sync(t)) return false;
  latency = 0;
  return true;
}

bool SafeTrajClient::SetVirtual(const char* xmlFile)
{
  assert(!IsConnected());
  if(!mirror.Setup(xmlFile)) return false;
  virtualController = true;
  latency = 0;
  return true;
}

ReadOnlyTrajClient SafeTrajClient::Client()
{
  return ReadOnlyTrajClient(this->t);
}

void SafeTrajClient::CalibrateLatency(int iters)
{
  if(virtualController) return;
  Timer timer;
  for (int i=0;i<iters;i++)
    this->t.Echo("");
  this->latency = timer.ElapsedTime()/iters;
}

void SafeTrajClient::SetNonblocking(bool nonblock)
{
  FlushSendQueue();
  this->nonblock = nonblock;
}

void SafeTrajClient::SetJointLimits(const Vector& qmin,const Vector& qmax)
{
  mirror.SetJointLimits(qmin,qmax);
  if(!virtualController) 
    this->t.SetJointLimits(mirror.qmin,mirror.qmax);
}


void SafeTrajClient::SetVelocityLimits(const Vector& vmax)
{
  mirror.SetVelocityLimits(vmax);
  if(!virtualController) 
    this->t.SetVelocityLimits(mirror.vmax);
}

void SafeTrajClient::SetAccelerationLimits(const Vector& amax)
{
  mirror.SetAccelerationLimits(amax);
  if(!virtualController) 
    this->t.SetAccelerationLimits(mirror.amax);
}

void SafeTrajClient::SetDecelerationLimits(const Vector& dmax)
{
  mirror.SetDecelerationLimits(dmax);
  if(!virtualController) 
    this->t.SetDecelerationLimits(mirror.dmax);
}

void SafeTrajClient::SetSpeedScale(double rate)
{
  assert(rate > 0.0 && rate <= 1.0);
  for(size_t i=0;i<this->mirror.vmax.size();i++) {
    this->mirror.vmax[i] = this->mirror.vmax0[i]*rate;
    this->mirror.amax[i] = this->mirror.amax0[i]*rate;
    this->mirror.dmax[i] = this->mirror.dmax0[i]*rate;
  }
  if(!virtualController) {
    this->t.SetVelocityLimits(this->mirror.vmax);
    this->t.SetAccelerationLimits(this->mirror.amax);
    this->t.SetDecelerationLimits(this->mirror.dmax);
  }
}

bool SafeTrajClient::GetEndConfig(Vector& jend)
{
  if(sendqs.empty()) return mirror.GetEndConfig(jend);
  else {
    jend = sendqs.back();
    return true;
  }
}
bool SafeTrajClient::GetEndVelocity(Vector& vend)
{
  if(sendqs.empty()) return mirror.GetEndVelocity(vend);
  else {
    if(sendqs.size()==1)
      mirror.GetEndConfig(vend);
    else
      vend = sendqs[sendqs.size()-2];
    for(size_t i=0;i<vend.size();i++)
      vend[i] = (sendqs.back()[i]-vend[i])/sendts[i];
    return true;
  }
}

bool SafeTrajClient::Brake(double dt)
{
  if(dt < 0) return false;
  double t = GetCurrentTime()+dt;
  Vector q;
  Vector dq;
  mirror.GetTrajConfig(t,q);
  mirror.GetTrajVelocity(t,dq);
  ParabolicRampND ramp;
  ramp.x0 = q;
  ramp.dx0 = dq;
  ramp.SolveBraking(mirror.dmax);
  if(!this->ResetTrajectoryAbs(t)) return false;
  return this->SendTrajectory(t,Cast(ramp),ramp.endTime);
}

bool SafeTrajClient::BrakeAbs(double t)
{
  if(t < GetCurrentTime()) return false;
  Vector q;
  Vector dq;
  mirror.GetTrajConfig(t,q);
  mirror.GetTrajVelocity(t,dq);
  ParabolicRampND ramp;
  ramp.x0 = q;
  ramp.dx0 = dq;
  ramp.SolveBraking(mirror.dmax);
  return this->SendTrajectory(t,Cast(ramp),ramp.endTime);
}

bool SafeTrajClient::ResetTrajectoryAbs(double t)
{
  if(mirror.GetTrajOffset(t) >= mirror.numSegments && sendIndex < (int)sendts.size()) {
    fprintf(stderr,"TODO: still sending to motion queue, reset is asked for in send queue\n");
    return false;
  }

  //clear send status
  sendts.clear();
  sendqs.clear();
  sendIndex = 0;

  if(!virtualController) {
    bool res=this->t.ResetTrajectoryAbs(t);
    if(!res) return false;
  }

  this->mirror.ResetTrajectoryAbs(t);
  return true;
}

bool SafeTrajClient::ResetTrajectoryRel(double dt)
{
  if(mirror.GetTrajOffset(GetCurrentTime()+dt) >= mirror.numSegments && sendIndex < (int)sendts.size()) {
    fprintf(stderr,"TODO: still sending to motion queue, reset is asked for in send queue\n");
    return false;
  }

  //clear send status
  sendts.clear();
  sendqs.clear();
  sendIndex = 0;

  if(!virtualController) {
    bool res=this->t.ResetTrajectoryRel(dt);
    if(!res) return false;
    double t=this->t.GetTrajEndTime();
    this->mirror.ResetTrajectoryAbs(t);
  }
  else
    this->mirror.ResetTrajectoryRel(dt);

  return true;
}

bool SafeTrajClient::MoveTo(const Vector& q,double rate,double dt)
{
  assert(rate > 0.0 and rate <= 1.0);
  this->CheckState(q);
        
  ParabolicRampND ramp;
  GetEndConfig(ramp.x0);
  GetEndVelocity(ramp.dx0);
  for(size_t i=0;i<ramp.dx0.size();i++)
    if(Abs(ramp.dx0[i]) <= 0.004*mirror.dmax[i])
      ramp.dx0[i] = 0.0;
    else
      printf("MoveTo: Warning, end velocity %d is not zero\n",i);
  ramp.x1 = q;
  ramp.dx1.resize(q.size(),0.0);
  //scale accel/velocity limits
  Vector ramax,rvmax;
  Real accRate = rate*kAccSafetyMultiplier;
  Real velRate = rate*kVelSafetyMultiplier;
  ramax.resize(mirror.amax.size());
  rvmax.resize(mirror.vmax.size());
  for(size_t i=0;i<mirror.dmax.size();i++) {
    ramax[i] = accRate*Min(mirror.amax[i],mirror.dmax[i]);
    rvmax[i] = velRate*mirror.vmax[i];
  }
  //solve
  if (dt == 0.0) {
    bool res=ramp.SolveMinTimeLinear(ramax,rvmax);
    assert(res);
  }
  else {
    bool res=ramp.SolveMinAccelLinear(rvmax,dt);
    assert(res);
    Vector a;
    ramp.Accel(0,a);
    //check for acceleration feasibility
    for(size_t i=0;i<a.size();i++)
      if(fabs(a[i]) > mirror.amax[i]) {
	res=ramp.SolveMinTimeLinear(ramax,rvmax);
	assert(res);
	break;
      }
  }
  PiecewisePolynomialND poly=Cast(ramp);
  /*
  printf("Testing poly...\n");
  printf("%d elements\n",poly.elements.size());
  for(size_t i=0;i<poly.elements.size();i++) {
    printf("element %d\n",i);
    printf("%d segments, %d shift, %d times\n",poly.elements[i].segments.size(),poly.elements[i].timeShift.size(),poly.elements[i].times.size());
  }
  poly.Evaluate(0);
  poly.Evaluate(0.5);
  */
  bool res=CheckTrajectory(0,poly,poly.EndTime());
  assert(res);
  return this->SendTrajectory(0,poly,ramp.endTime);
}

bool SafeTrajClient::MoveToImmediate(double dt,const Vector& q,double rate)
{
  assert(rate > 0.0 and rate <= 1.0);
  this->CheckState(q);

  //scale accel/velocity limits
  Vector ramax,rvmax;
  Real accRate = rate*kAccSafetyMultiplier;
  Real velRate = rate*kVelSafetyMultiplier;
  ramax.resize(mirror.dmax.size());
  rvmax.resize(mirror.vmax.size());
  for(size_t i=0;i<mirror.dmax.size();i++) {
    ramax[i] = accRate*Min(mirror.amax[i],mirror.dmax[i]);
    rvmax[i] = velRate*mirror.vmax[i];
  }
        
  double t=GetCurrentTime();
  ParabolicRampND ramp;
  mirror.GetTrajConfig(t+dt,ramp.x0);
  mirror.GetTrajVelocity(t+dt,ramp.dx0);
  ramp.x1 = q;
  ramp.dx1.resize(q.size(),0.0);
  //solve
  //bool res=ramp.SolveMinTime(ramax,rvmax);
  //assert(res);
  vector<vector<ParabolicRamp1D> > ramps;
  Real endTime = SolveMinTimeBounded(ramp.x0,ramp.dx0,ramp.x1,ramp.dx1,
				     ramax,rvmax,mirror.qmin,mirror.qmax,
				     ramps);
  assert(endTime >= 0);
  return this->SendTrajectory(t+dt,Cast(ramps),endTime);
}

bool SafeTrajClient::MoveTraj(const std::vector<Vector>& q,const std::vector<Vector>& dq,const std::vector<double>& dt,double rate)
{
  //sanity check
  if(!dq.empty())
    assert(dq.size()==q.size());
  if(!dt.empty())
    assert(dt.size()==q.size());
  for(size_t i=0;i<q.size();i++) {
    assert(q[i].size()==mirror.qmin.size());
    if(!dq.empty())
      assert(dq[i].size()==mirror.qmin.size());
    if(!dt.empty()) 
      assert(dt[i] >= 0);
  }
 
  if(q.empty()) return true;       

  //scale accel/velocity limits
  Vector ramax,rvmax,rdmax;
  Real accRate = rate*kAccSafetyMultiplier;
  Real velRate = rate*kVelSafetyMultiplier;
  ramax.resize(mirror.amax.size());
  rdmax.resize(mirror.amax.size());
  rvmax.resize(mirror.vmax.size());
  for(size_t i=0;i<mirror.amax.size();i++) {
    ramax[i] = accRate*mirror.amax[i];
    rdmax[i] = accRate*mirror.dmax[i];
    rvmax[i] = velRate*mirror.vmax[i];
  }

  /*
  vector<ParabolicRampND> ramps(q.size());
  mirror.GetEndConfig(ramps[0].x0);
  mirror.GetEndVelocity(ramps[0].dx0);
  for(size_t i=0;i<q.size();i++) {
    ramps[i].x1=q[i];
    if(dq.empty()) 
      ramps[i].x1=ramps[0].dx0;
    else
      ramps[i].dx1=dq[i];
    
    if(dt.empty()) {
      bool res=ramps[i].SolveMinTime(ramax,rvmax);
      assert(res);
    }
    else {
      bool res=ramps[i].SolveMinAccel(rvmax,dt[i]);
      assert(res);
      //TODO: check accel
    }

    if(i+1 < q.size()) {
      ramps[i+1].x0 =ramps[i].x1;
      ramps[i+1].dx0 =ramps[i].dx1;
    }
  }
  
  if(!dq.empty()) {
    bool nonzero=false;
    for(size_t i=0;i<dq.back().size();i++)
      if(Abs(dq.back()[i]) != 0.0) nonzero = true;
    if(nonzero) {
      //final dq is nonzero, add a braking trajectory
      ramps.resize(ramps.size()+1);
      ramps.back().x0 = q.back();
      ramps.back().dx0 = dq.back();
      ramps.back().SolveBraking(rdmax);
    }
  }
  */
  vector<vector<vector<ParabolicRamp1D> > > ramps(q.size());
  Vector xp,dxp;
  mirror.GetEndConfig(xp);
  mirror.GetEndVelocity(dxp);
  for(size_t i=0;i<q.size();i++) {
    Vector x = q[i];
    Vector dx;
    if(dq.empty()) 
      dx.resize(x.size(),0.0);
    else
      dx=dq[i];
    
    if(dt.empty()) {
      Real endTime = SolveMinTimeBounded(xp,dxp,x,dx,ramax,rvmax,mirror.qmin,mirror.qmax,ramps[i]);
      assert(endTime >= 0);
    }
    else {
      bool res = SolveMinAccelBounded(xp,dxp,x,dx,dt[i],rvmax,mirror.qmin,mirror.qmax,ramps[i]);
      assert(res);
      //TODO: check accel
    }

    xp = x;
    dxp = dx;
  }
  
  if(!dq.empty()) {
    bool nonzero=false;
    for(size_t i=0;i<dq.back().size();i++)
      if(Abs(dq.back()[i]) != 0.0) nonzero = true;
    if(nonzero) {
      //final dq is nonzero, add a braking trajectory
      ramps.resize(ramps.size()+1);
      ParabolicRampND ramp;
      ramp.x0 = q.back();
      ramp.dx0 = dq.back();
      ramp.SolveBraking(rdmax);
      ramps.back().resize(ramp.ramps.size());
      for(size_t i=0;i<ramp.ramps.size();i++) 
	ramps.back()[i].resize(1,ramp.ramps[i]);
    }
  }

  PiecewisePolynomialND poly;
  poly=Cast(ramps[0]);
  for(size_t i=1;i<ramps.size();i++)
    poly.Concat(Cast(ramps[i]),true);
  bool res=CheckTrajectory(0,poly,poly.EndTime());
  assert(res);
  SendTrajectory(0,poly,poly.EndTime());
  return true;
}

bool SafeTrajClient::MoveTraj(const std::vector<Vector>& q,const std::vector<Vector>& dq,double rate)
{
  vector<double> dt;
  return MoveTraj(q,dq,dt,rate);
}

bool SafeTrajClient::MoveTraj(const std::vector<Vector>& q,const std::vector<double>& dt,double rate)
{
  vector<Vector> dq;
  return MoveTraj(q,dq,dt,rate);
}

bool SafeTrajClient::MoveTraj(const std::vector<Vector>& q,double rate)
{
  vector<Vector> dq;
  vector<double> dt;
  return MoveTraj(q,dq,dt,rate);
}


bool SafeTrajClient::CheckState(const Vector& q) const
{
  Vector dq,ddq;
  return CheckState(q,dq,ddq);
}

bool SafeTrajClient::CheckState(const Vector& q,const Vector& dq) const
{
  Vector ddq;
  return CheckState(q,dq,ddq);
}

bool SafeTrajClient::CheckState(const Vector& q,const Vector& dq,const Vector& ddq) const
{
  assert(q.size() == mirror.qmin.size());
  double dt = 1.0/mirror.maxRate;
  for(size_t i=0;i<q.size();i++) {
    if(q[i] < mirror.qmin[i] || q[i] > mirror.qmax[i]) {
      fprintf(stderr,"CheckState: %g > q[%d]=%g > %g\n",mirror.qmin[i],i,q[i],mirror.qmax[i]);
      return false;
    }
    if(!dq.empty()) {
      if (fabs(dq[i]) > mirror.vmax[i]) {
	fprintf(stderr,"CheckState: Vel[%d]=%g > %g\n",i,dq[i],mirror.vmax[i]);
	return false;
      }
    }
    if(!ddq.empty()) {
      if(fabs(dq[i]+ddq[i]*dt) > fabs(dq[i])) { //accelerating
	if(fabs(ddq[i]) > mirror.amax[i]) {
	  fprintf(stderr,"CheckState: Acc[%d]=%g > %g\n",i,ddq[i],mirror.amax[i]);
	  return false;
	}
      }
      else {
	if(fabs(ddq[i]) > mirror.dmax[i]) {
	  fprintf(stderr,"CheckState: Dec[%d]=%g > %g\n",i,ddq[i],mirror.dmax[i]);
	  return false;
	}
      }
    }
  }
  return true;
}



void SafeTrajClient::FlushSendQueue()
{
  while (sendIndex < (int)sendqs.size()) {
    double sleept = SendPoll();
    if(sleept > 0) 
      usleep(sleept*1000000);
  }
}


double SafeTrajClient::SendPoll()
{
  if(sendIndex == (int)sendqs.size()) return 0;
  int cs;
  //make sure the mirror is sync'ed in time
  double t=this->t.GetCurrentTime();
  Timer timer;
  if(t > this->mirror.currentTime)
    this->mirror.AdvanceTime(t);
  else
    printf("SendPoll: Warning, time is %g, mirror time is %g\n",t,this->mirror.currentTime);
  while((cs= mirror.GetCurSegments()) < GetMaxSegments()) {
    printf("Current segments: %d, remaining %d\n",cs,GetMaxSegments()-cs);
    int lim = std::min(milestoneBatch,GetMaxSegments()-cs);
    int imax = std::min(sendIndex+lim,(int)sendqs.size()-1);
    printf("Sending %d milestones\n",imax-sendIndex);
    std::vector<double> tslice(sendts.begin()+sendIndex,sendts.begin()+imax+1);
    std::vector<Vector> qslice(sendqs.begin()+sendIndex,sendqs.begin()+imax+1);
    if(!virtualController) {
      bool res=this->t.AppendMilestonesQuiet(tslice,qslice);
      //make sure the mirror is sync'ed in time
      double newt = t+timer.ElapsedTime();
      if(newt > this->mirror.currentTime)
        this->mirror.AdvanceTime(newt);
      else
        printf("SendPoll: Warning, new time is %g, mirror time is %g\n",newt,this->mirror.currentTime);
    }
    this->mirror.AppendMilestonesQuiet(tslice,qslice);

    sendIndex = imax+1;
    if(sendIndex == (int)sendqs.size())
      //done
      return 0;
  }

  printf("Current # of segments: %d, approx %g seconds left\n",GetCurSegments(),this->mirror.GetTrajEndTime()-this->mirror.GetCurrentTime());
  //this is a good amount to sleep
  return 0.5*(this->mirror.GetTrajEndTime()-this->mirror.GetCurrentTime());
}
