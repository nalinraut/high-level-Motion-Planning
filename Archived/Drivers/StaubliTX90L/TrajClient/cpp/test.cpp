#include "SafeTrajClient.h"
#include "Timer.h"
#include <iostream>
#include <sstream>
using namespace std;
typedef TrajClient::Vector Vector;
typedef TrajClient::FunctionCall FunctionCall;

void Print(const Vector& v)
{
  for(size_t i=0;i<v.size();i++)
    printf("%g ",v[i]);
}

/**This function tests the communication latency/bandwidth by sending
   batched vs nonbatched gx calls.*/
void TestTiming(TrajClient& s,int iters=10,int batchsize=99)
{
  Vector qstart;
  s.GetEndConfig(qstart);
  stringstream ss;
  ss<<"0.004 "<<qstart[0]<<" "<<qstart[1]<<" "<<qstart[2]<<" "<<qstart[3]<<" "<<qstart[4]<<" "<<qstart[5];
  string milestonecmd = ss.str();

  Timer timer;
  for(int index=0;index<iters;index++) {
    timer.Reset();
    vector<FunctionCall> f(batchsize,FunctionCall("gx",""));
    s.Call(f);
    printf("Time for %d gx's (batched): %f\n",batchsize,timer.ElapsedTime());
    
    timer.Reset();
    for(int i=0;i<batchsize;i++)
      s.Call("gx","");
    printf("Time for %d gx's (unbatched): %f\n",batchsize,timer.ElapsedTime());

    timer.Reset();
    f=vector<FunctionCall>(batchsize,FunctionCall("am",milestonecmd));
    s.Call(f);
    printf("Time for %d am's (batched): %f\n",batchsize,timer.ElapsedTime());

    s.Call(f,false);
    printf("Time for %d am's (batched, noreply): %f\n",batchsize,timer.ElapsedTime());
    
    timer.Reset();
    for(int i=0;i<batchsize;i++)
      s.Call("am",milestonecmd);
    printf("Time for %d am's (unbatched): %f\n",batchsize,timer.ElapsedTime());

    timer.Reset();
    for(int i=0;i<batchsize;i++) 
      s.Call("am",milestonecmd,false);
    printf("Time for %d am's (unbatched, noreply): %f\n",batchsize,timer.ElapsedTime());
  }
}



int main ()
{
  TrajClient s("127.0.0.1");
  if(!s.IsConnected()) {
    printf("Couldn't connect, quitting\n");
    return 1;
  }
  Vector qstart;
  s.GetConfig(qstart);
  printf("Start configuration: ");
  Print(qstart);
  printf("\n");
  Vector q,qmin,qmax;
  /*
  //This tests a triangle wave on joint 2, starting at the zero position
  //Home position
  s.addMilestone(5,q);
  //triangle wave
  q = qstart;
  q[1] += 10;
  s.addMilestone(2,q);
  q[1] += -20;
  s.addMilestone(4,q);
  q[1] += 10;
  s.addMilestone(2,q);
  exit();
  */

  s.GetJointLimits(qmin,qmax);
  printf("Joint limits: ");
  Print(qmin);
  printf(",");
  Print(qmax);
  printf("\n");
  s.GetVelocityLimits(q);
  printf("Velocity limits: ");
  Print(q);
  printf("\n");


  /*
  //tests rapid sending of milestones
  for(int i=0;i<100;i++) {
    q = qstart;
    q[0] += double(i);
    s.AddMilestoneQuiet(0.04,q);
  }
  */


  /**
  //This tests the safe trajectory client
  print "Starting safe trajectory test..."
  s = SafeTrajClient('127.0.0.1')
  print "Calibrating latency..."
  s.calibrateLatency()
  print "Connection latency: ",s.latency
  s.moveto([0,0,0,0,0,0])
  s.moveto([0,30,0,0,0,0])
  s.moveto([0,0,0,0,0,0])
  */

  TestTiming(s);
}

