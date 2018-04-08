#include "motion.h"
#include <KrisLibrary/utils/threadutils.h>
#include <KrisLibrary/utils/socketutils.h>
#include <KrisLibrary/utils/ioutils.h>
#include <KrisLibrary/utils/SmartPointer.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <KrisLibrary/myfile.h>
#include <signal.h>
#include <KrisLibrary/Timer.h>
#include <string>
#include <sstream>
#include <vector>
using namespace std;

typedef bool (*PARSER_TYPE)(stringstream& in,stringstream& out);

inline int NumLimbDofs(int limb) { if(limb==BOTH) return 2*numLimbDofs; return numLimbDofs; }

map<string,PARSER_TYPE> parsers;

//this macro registers the function in the global parsers registry.
//(uses a global dummy class to add the parser to the registry on constructor)
#define REGISTER(func) \
  class func ## _dummy_class {						\
  public:								\
    func ## _dummy_class () {						\
      if(parsers.count(#func)) FatalError("Function %s already defined!\n",#func); \
      parsers[#func] = (func ## _parse); }				\
  } func ## _dummy_object; 

#define DECLBEGIN(func)						\
  bool func ## _parse(stringstream& in,stringstream& out) { 

#define DECLEND(func)				\
  return true; } REGISTER(func)

#define IN_BOOL(value) in>>value; if(!in) return false;
#define IN_int(value) in>>value; if(!in) return false;
#define IN_double(value) in>>value; if(!in) return false;
#define IN_string(value) if(!SafeInputString(in,value)) return false;
#define IN_STRING(value) if(!SafeInputString(in,value)) return false;


#define INARG(type,name)			\
  type name;					\
  IN_##type(name)	

#define INARGV(type,name,size)			\
  int n_##name = size;				\
  type name[n_##name];				\
  for(int i=0;i<n_##name;i++)			\
    { IN_##type(name[i]);  }


#define OUT_BOOL(value) out<<value;
#define OUT_int(value) out<<value;
#define OUT_double(value) out<<value;
#define OUT_STRING(value) SafeOutputString(out,value);

#define OUTRET(type,value) OUT_##type(value) 	  

#define OUTARG(type,value) out<<" "; OUT_##type(value) 	   

#define OUTARGV(type,value,size)			\
  for(int i=0;i<size;i++)			\
    { OUTARG(type,value[i]) }

#define DECLCACHED(rettype,func)					\
  bool func ## _read = false;						\
  rettype func ## _value;						\
  rettype cached_##func() {						\
    if(! func ## _read) { ( func ## _value )= func(); ( func ## _read ) = true; } \
    return func ##_value; }
#define CACHED(func) cached_##func ()

#define DECLCACHED1(rettype,func,n)					\
  bool func ## _read = false;						\
  rettype func ## _value [n];						\
  rettype cached_##func(int index) {					\
    if(! func ## _read) { for(int i=0;i<n;i++) ( func ## _value )[i] = func(i); ( func ## _read ) = true; } \
    return func ##_value[index]; }
#define CACHED1(func,arg)  cached_##func (arg)

template<class T>
T FailVal() { return 0 ;}

template<>
double FailVal() { return -1;}

#define STRING string
#define A_string const char*
#define A_int int
#define A_BOOL BOOL
#define A_double double
#define A(type) A_##type
#define E_string(val) val.c_str()
#define E_BOOL(val) val
#define E_int(val) val
#define E_double(val) val
#define E(type,val) E_##type(val)

#define DECLSEND(rettype,func)			\
  DECLBEGIN(func)				\
    OUTRET(rettype,func())			\
  DECLEND(func)

#define DECLSEND_V(rettype,func,argtype)	\
  DECLBEGIN(func)				\
    INARG(argtype,arg)			\
    OUTRET(rettype,func(E(argtype,arg)))	\
  DECLEND(func)


#define DECLSEND_VV(rettype,func,arg1type,arg2type)		\
  DECLBEGIN(func)						\
    INARG(arg1type,arg1)					\
    INARG(arg2type,arg2)					\
    OUTRET(rettype,func(E(arg1type,arg1),E(arg2type,arg2)))	\
  DECLEND(func)

#define DECLSEND_VVV(rettype,func,arg1type,arg2type,arg3type)		\
  DECLBEGIN(func)							\
    INARG(arg1type,arg1)						\
    INARG(arg2type,arg2)						\
    INARG(arg3type,arg3)						\
    OUTRET(rettype,func(E(arg1type,arg1),E(arg2type,arg2),E(arg3type,arg3))) \
  DECLEND(func)

#define DECLSEND_VVVV(rettype,func,arg1type,arg2type,arg3type,arg4type)	\
  DECLBEGIN(func) \
    INARG(arg1type,arg1)						\
    INARG(arg2type,arg2)						\
    INARG(arg3type,arg3)						\
    INARG(arg4type,arg4)						\
    OUTRET(rettype,func(E(arg1type,arg1),E(arg2type,arg2),E(arg3type,arg3),E(arg3type,arg3,arg4))) \
  DECLEND(func)


#define DECLSEND_A(rettype,func,arg1type,arg1size)	\
  DECLBEGIN(func) \
    INARGV(arg1type,arg1,arg1size)		\
    OUTRET(rettype,func(arg1))	\
  DECLEND(func)

#define DECLSEND_AA(rettype,func,arg1type,arg1size,arg2type,arg2size)		\
  DECLBEGIN(func) \
    INARGV(arg1type,arg1,arg1size)		\
    INARGV(arg2type,arg2,arg2size)		\
    OUTRET(rettype,func(arg1,arg2))		\
  DECLEND(func)

#define DECLSEND_VA(rettype,func,arg1type,arg2type,arg2size)	\
  DECLBEGIN(func) \
    INARG(arg1type,arg1)			\
    INARGV(arg2type,arg2,arg2size)		\
    OUTRET(rettype,func(E(arg1type,arg1),arg2))	\
  DECLEND(func)

#define DECLSEND_VVA(rettype,func,arg1type,arg2type,arg3type,arg3size)	\
  DECLBEGIN(func) \
    INARG(arg1type,arg1)			\
    INARG(arg2type,arg2)			\
    INARGV(arg3type,arg3,arg3size)		\
    OUTRET(rettype,func(E(arg1type,arg1),E(arg2type,arg2),arg3))	\
  DECLEND(func)

#define DECLSEND_VAA(rettype,func,arg1type,arg2type,arg2size,arg3type,arg3size)	\
  DECLBEGIN(func) \
    INARG(arg1type,arg1)				\
    INARGV(arg2type,arg2,arg2size)			\
    INARGV(arg3type,arg3,arg3size)			\
    OUTRET(rettype,func(E(arg1type,arg1),arg2,arg3))	\
  DECLEND(func)

#define DECLSEND_VAV(rettype,func,arg1type,arg2type,arg2size,arg3type)	\
  DECLBEGIN(func) \
    INARG(arg1type,arg1)			\
    INARGV(arg2type,arg2,arg2size)		\
    INARG(arg3type,arg3)			\
    OUTRET(rettype,func(E(arg1type,arg1),arg2,E(arg3type,arg3)))	\
  DECLEND(func)

#define DECLSEND_VAAA(rettype,func,arg1type,arg2type,arg2size,arg3type,arg3size,arg4type,arg4size) \
  DECLBEGIN(func) \
    INARG(arg1type,arg1)				\
    INARGV(arg2type,arg2,arg2size)			\
    INARGV(arg3type,arg3,arg3size)			\
    INARGV(arg4type,arg4,arg4size)			\
    OUTRET(rettype,func(E(arg1type,arg1),arg2,arg3,arg4))	\
  DECLEND(func)

#define DECLSEND_VVAA(rettype,func,arg1type,arg2type,arg3type,arg3size,arg4type,arg4size) \
  DECLBEGIN(func) \
    INARG(arg1type,arg1)			\
    INARG(arg2type,arg2)			\
    INARGV(arg3type,arg3,arg3size)		\
    INARGV(arg4type,arg4,arg4size)		\
    OUTRET(rettype,func(E(arg1type,arg1),E(arg2type,arg2),arg3,arg4))	\
  DECLEND(func)

#define DECLSEND_VAAVVV(rettype,func,arg1type,arg2type,arg2size,arg3type,arg3size,arg4type,arg5type,arg6type) \
  DECLBEGIN(func) \
    INARG(arg1type,arg1)				\
    INARGV(arg2type,arg2,arg2size)			\
    INARGV(arg3type,arg3,arg3size)			\
    INARG(arg4type,arg4)				\
    INARG(arg5type,arg5)				\
    INARG(arg6type,arg6)				\
    OUTRET(rettype,func(E(arg1type,arg1),arg2,arg3,arg4,arg5,arg6))	\
  DECLEND(func)


#define DECLSEND0 DECLSEND
#define DECLSEND1 DECLSEND_V
#define DECLSEND2 DECLSEND_VV
#define DECLSEND3 DECLSEND_VVV
#define DECLSEND4 DECLSEND_VVVV

#define DECLRECV_V(rettype,func,arg1type) \
  DECLBEGIN(func) \
    arg1type arg1;				\
    OUTRET(rettype,func(&arg1))			\
    OUTARG(arg1type,arg1);			\
    return true; } \
  REGISTER(func)

#define DECLRECV_VV(rettype,func,arg1type,arg2type)		    \
  DECLBEGIN(func) \
    arg1type arg1; \
    arg2type arg2; \
    OUTRET(func(&arg1,&arg2));			\
    OUTARG(arg1type,arg1);			\
    OUTARG(arg2type,arg2);			\
  DECLEND(func)

#define DECLRECV_VVV(rettype,func,arg1type,arg2type,arg3type)	    \
  DECLBEGIN(func) \
    arg1type arg1; \
    arg2type arg2; \
    arg3type arg3; \
    out<<func(&arg1,&arg2,&arg3);				\
    out<<" "<<arg1<<" "<<arg2<<" "<<arg3;		\
  DECLEND(func)

#define DECLRECV_VVVV(rettype,func,arg1type,arg2type,arg3type,arg4type)	\
  DECLBEGIN(func) \
    arg1type arg1; \
    arg2type arg2; \
    arg3type arg3; \
    arg3type arg4; \
    out<<func(&arg1,&arg2,&arg3,&arg4);				\
    out<<" "<<arg1<<" "<<arg2<<" "<<arg3<<" "<<arg4;		\
  DECLEND(func)
  

#define DECLRECV_A(rettype,func,arg1type,arg1size)		    \
  DECLBEGIN(func) \
    int n1=arg1size; \
    arg1type arg1[n1]; \
    OUTRET(rettype,func(arg1))		\
    OUTARGV(arg1type,arg1,n1);		\
  DECLEND(func)

#define DECLRECV0 DECLRECV
#define DECLRECV1 DECLRECV_V
#define DECLRECV2 DECLRECV_VV
#define DECLRECV3 DECLRECV_VVV
#define DECLRECV4 DECLRECV_VVVV

#define DECLSEND_V_RECV_A(rettype,func,arg1type,out1type,out1size)	\
  DECLBEGIN(func) \
    INARG(arg1type,arg1)			\
    int n1 = out1size;				\
    out1type out1[n1];				\
    OUTRET(rettype,func(arg1,out1))		\
    OUTARGV(out1type,out1,n1)			\
  DECLEND(func)

#define DECLSEND_V_RECV_AA(rettype,func,arg1type,out1type,out1size,out2type,out2size)	\
  DECLBEGIN(func) \
    arg1type arg1; \
    in >> arg1; if(!in) return false; \
    int n1 = out1size; \
    out1type out1[n1]; \
    int n2 = out2size; \
    out1type out2[n2]; \
    OUTRET(rettype,func(arg1,out1,out2))		\
    OUTARGV(out1type,out1,n1)			\
    OUTARGV(out2type,out2,n2)			\
  DECLEND(func)


DECLSEND1(BOOL,setKlamptModel,STRING);

DECLBEGIN(getKlamptModel)
  char buf[4096];
  buf[0] = 0;
  OUTRET(BOOL,getKlamptModel(buf,4096));
  buf[4095]=0;
  OUTARG(STRING,buf);
DECLEND(getKlamptModel)

DECLSEND1(BOOL,publishState,STRING);
DECLSEND0(BOOL,sendStartup);
DECLSEND0(BOOL,isStarted);
DECLSEND0(BOOL,sendShutdown);
DECLSEND0(double,getTime);
DECLSEND0(BOOL,stopMotion);

DECLSEND0(BOOL,isHeadNodding);
DECLSEND0(double,getHeadPan);
DECLSEND2(BOOL,sendHeadPan,double,double);

DECLSEND0(BOOL,isMobileBaseEnabled);
DECLSEND0(BOOL,isMobileBaseMoving);
DECLSEND0(double,getMobileBaseMoveTime);
DECLRECV3(BOOL,getMobileBaseTarget,double,double,double);
DECLRECV3(BOOL,getMobileBaseOdometryTarget,double,double,double);
DECLRECV3(BOOL,getMobileBaseVelocity,double,double,double);
DECLRECV3(BOOL,getMobileBaseOdometry,double,double,double);
DECLSEND3(BOOL,sendMobileBasePosition,double,double,double);
DECLSEND3(BOOL,sendMobileBaseOdometryPosition,double,double,double);
DECLSEND3(BOOL,sendMobileBaseVelocity,double,double,double);


DECLSEND1(BOOL,isLimbPositionMode,int);
DECLSEND1(BOOL,isLimbVelocityMode,int);
DECLSEND1(BOOL,isLimbEffortMode,int);
DECLSEND1(BOOL,isLimbRawPositionMode,int);

DECLSEND_V_RECV_A(BOOL,getLimbPosition,int,double,NumLimbDofs(arg1));
DECLSEND_V_RECV_A(BOOL,getLimbVelocity,int,double,NumLimbDofs(arg1));
DECLSEND_V_RECV_A(BOOL,getLimbEffort,int,double,NumLimbDofs(arg1));
DECLSEND_V_RECV_A(BOOL,getLimbCommandedPosition,int,double,NumLimbDofs(arg1));
DECLSEND_V_RECV_A(BOOL,getLimbCommandedVelocity,int,double,NumLimbDofs(arg1));

DECLSEND_VA(BOOL,sendLimbPosition,int,double,NumLimbDofs(arg1));
DECLSEND_VA(BOOL,sendLimbRawPosition,int,double,NumLimbDofs(arg1));
DECLSEND_VA(BOOL,sendLimbVelocity,int,double,NumLimbDofs(arg1));
DECLSEND_VA(BOOL,sendLimbEffort,int,double,NumLimbDofs(arg1));

DECLSEND_VA(BOOL,setEndEffectorOffset,int,double,3);
DECLSEND_V_RECV_AA(BOOL,getEndEffectorSensedTransform,int,double,9,double,3);
DECLSEND_V_RECV_AA(BOOL,getEndEffectorSensedVelocity,int,double,3,double,3);
DECLSEND_V_RECV_AA(BOOL,getEndEffectorCommandedTransform,int,double,9,double,3);
DECLSEND_V_RECV_AA(BOOL,getEndEffectorCommandedVelocity,int,double,3,double,3);
DECLSEND_VAA(BOOL,sendEndEffectorVelocity,int,double,3,double,3);
DECLSEND_VAAVVV(BOOL,sendEndEffectorMoveTo,int,double,9,double,3,double,double,double);
DECLSEND_VA(BOOL,sendEndEffectorPositionDrive,int,double,3);
DECLSEND_VAA(BOOL,sendEndEffectorDrive,int,double,3,double,3);

DECLSEND1(BOOL,isGripperEnabled,int);

DECLBEGIN(getGripperType)
  INARG(int,limb)
  char buf[1024];
  buf[0] = 0;
  OUTRET(BOOL,getGripperType(limb,buf,1024));
  buf[1023]=0;
  OUTARG(STRING,buf);
DECLEND(getGripperType)

DECLSEND1(int,numGripperDofs,int);
DECLCACHED1(int,numGripperDofs,2);
DECLSEND1(BOOL,isGripperMoving,int);
DECLSEND1(double,getGripperMoveTime,int);
DECLSEND_V_RECV_A(BOOL,getGripperPosition,int,double,CACHED1(numGripperDofs,arg1));
DECLSEND_V_RECV_A(BOOL,getGripperTarget,int,double,CACHED1(numGripperDofs,arg1));
DECLSEND_V_RECV_A(BOOL,getGripperEffort,int,double,CACHED1(numGripperDofs,arg1));
DECLSEND1(BOOL,sendCloseGripper,int);
DECLSEND1(BOOL,sendOpenGripper,int);
DECLSEND_VAAA(BOOL,sendSetGripper,int,double,CACHED1(numGripperDofs,arg1),double,CACHED1(numGripperDofs,arg1),double,CACHED1(numGripperDofs,arg1));

DECLSEND1(BOOL,isMotionQueueEnabled,int);
DECLSEND1(BOOL,isMotionQueueMoving,int);
DECLSEND1(double,getMotionQueueMoveTime,int);
DECLSEND_V_RECV_A(BOOL,getMotionQueueTarget,int,double,NumLimbDofs(arg1));
DECLSEND_VVA(BOOL,sendMotionQueueLinear,int,double,double,NumLimbDofs(arg1));
DECLSEND_VVAA(BOOL,sendMotionQueueCubic,int,double,double,NumLimbDofs(arg1),double,NumLimbDofs(arg1));
DECLSEND_VAV(BOOL,sendMotionQueueRamp,int,double,NumLimbDofs(arg1),double);
DECLSEND_VVA(BOOL,sendMotionQueueAppendLinear,int,double,double,NumLimbDofs(arg1));
DECLSEND_VVAA(BOOL,sendMotionQueueAppendCubic,int,double,double,NumLimbDofs(arg1),double,NumLimbDofs(arg1));
DECLSEND_VAV(BOOL,sendMotionQueueAppendRamp,int,double,NumLimbDofs(arg1),double);
DECLSEND_VAV(BOOL,sendMotionQueueAppendLinearRamp,int,double,NumLimbDofs(arg1),double);

/*
APIENTRY BOOL sendMotionQueueTrajectory(int limb,int numPoints,const double* times,const double* milestones,const double* vmilestones=0);
///Sends a trajectory starting from the end of the current .  Milestones are given as an array of length
///numPoints*n, where n is the dimensionality of the limb.  If vmilestones is provided, then it is an
///array of the same size designating the velocities at the milestones, and cubic interpolation is performed.
///Otherwise, linear interpolation is performed.
APIENTRY BOOL sendMotionQueueAppendTrajectory(int limb,int numPoints,const double* times,const double* milestones,const double* vmilestones=0);
*/

DECLSEND1(BOOL,isPlannerEnabled,int);
DECLSEND1(BOOL,sendPlannerObjectiveStr,STRING);
DECLSEND0(BOOL,stopPlanner);
DECLSEND0(double,plannerObjectiveValue);
DECLSEND1(BOOL,setPlannerWorldFile,STRING);
DECLSEND1(int,addPlannerObstacle,STRING);
DECLSEND1(int,deletePlannerObstacle,int);
DECLSEND2(BOOL,setPlannerObstacleMargin,int,double);
DECLSEND0(BOOL,clearPlannerWorld);
DECLSEND_VV(BOOL,enableLimbSelfCollisionAvoidance,int,BOOL);
DECLSEND_V(BOOL,enableCollisionChecking,BOOL);

DECLSEND0(int,getKlamptNumDofs);
DECLCACHED(int,getKlamptNumDofs);
DECLSEND_A(BOOL,setIKBiasConfiguration,double,CACHED(getKlamptNumDofs));
DECLRECV_A(BOOL,getKlamptSensedPosition,double,CACHED(getKlamptNumDofs));
DECLRECV_A(BOOL,getKlamptSensedVelocity,double,CACHED(getKlamptNumDofs));
DECLRECV_A(BOOL,getKlamptCommandedPosition,double,CACHED(getKlamptNumDofs));
DECLRECV_A(BOOL,getKlamptCommandedVelocity,double,CACHED(getKlamptNumDofs));
DECLSEND_A(BOOL,sendKlamptMoveVelocity,double,CACHED(getKlamptNumDofs));
DECLSEND_A(BOOL,sendKlamptMoveToTarget,double,CACHED(getKlamptNumDofs));
DECLSEND_V_RECV_A(BOOL,getKlamptLimbIndices,int,int,NumLimbDofs(arg1));
DECLSEND0(int,getKlamptHeadPanIndex);
DECLRECV3(BOOL,getKlamptMobileBaseIndices,int,int,int);
DECLSEND_V_RECV_A(BOOL,getKlamptGripperIndices,int,int,CACHED1(numGripperDofs,arg1));



class RequestReplyServer
{
public:
  RequestReplyServer(const char* addr,int maxclients);
  virtual ~RequestReplyServer();
  bool Start();
  void Stop();
  void Poll();
  void Run(double pollsleep = 0.001);
  ///do the processing here
  ///return false to indicate that the client should be shut down
  virtual bool Process(SmartPointer<File> client) =0;

  string addr;
  int serversocket;
  int maxclients;
  std::vector<SmartPointer<File> > clientsockets;
};

RequestReplyServer::RequestReplyServer(const char* _addr,int _maxclients)
  :addr(_addr),serversocket(-1),maxclients(_maxclients)
{
}

RequestReplyServer::~RequestReplyServer()
{
  Stop();
}

bool RequestReplyServer::Start()
{
  serversocket = Bind(addr.c_str(),true);
  if(serversocket < 0) {
    fprintf(stderr,"Unable to bind server socket to address %s\n",addr.c_str());
    return false;
  }
  listen(serversocket,maxclients);
  return true;
}

void RequestReplyServer::Stop()
{
  for(size_t i=0;i<clientsockets.size();i++)
    clientsockets[i] = NULL;
  clientsockets.resize(0);
  CloseSocket(serversocket);
}

void RequestReplyServer::Run(double pollsleep)
{
  while(true) {
    Poll();
    ThreadSleep(pollsleep);
  }
}

void RequestReplyServer::Poll()
{
  if(serversocket < 0) {
    fprintf(stderr,"Server is not Start()'ed yet\n");
    return;
  }
  if((int)clientsockets.size() < maxclients) {
    SOCKET clientsock = Accept(serversocket,0.0);
    if(clientsock != INVALID_SOCKET) {
      printf("Accepted new client on %s\n",addr.c_str());
      clientsockets.push_back(new File);
      clientsockets.back()->OpenTCPSocket(clientsock);
    }
    int flag = 1;
    int result = setsockopt(clientsock,            /* socket affected */
                        IPPROTO_TCP,     /* set option at TCP level */
                        TCP_NODELAY,     /* name of option */
                        (char *) &flag,  /* the cast is historical cruft */
                        sizeof(int));    /* length of option value */
  }
  if(clientsockets.empty()) {
    //tolerant of failed clients
    return;
  }

  while(true) {
    bool read = false;
    for(size_t i=0;i<clientsockets.size();i++) {
      //loop through the sockets
      if(clientsockets[i]->ReadAvailable()) {
	read = true;
	bool keepalive = Process(clientsockets[i]);
	if(!keepalive) { //close the client
	  printf("RequestReplyServer: Closed client %d\n",i);
	  //this will close the socket
	  clientsockets[i] = NULL;
	  clientsockets[i] = clientsockets.back();
	  clientsockets.resize(clientsockets.size()-1);
	  i--;
	}
      }
    }
    if(!read) return;
  }
}

class MyRequestReplyServer : public RequestReplyServer
{
public:
  int msgnum;
  double msgtime,processtime;

  MyRequestReplyServer(const char* addr,int maxclients)
    :RequestReplyServer(addr,maxclients),msgnum(0),msgtime(0),processtime(0)
  {}
  virtual bool Process(SmartPointer<File> client) {
    Timer timer;
    char buf[4096];
    if(!client->ReadString(buf,4096)) return false;
    //printf("Read message \"%s\"\n",buf);
    stringstream in(buf);
    int msg_id;
    string func_name;
    in >> msg_id >> func_name;
    stringstream out;
    out<<msg_id<<" ";
    if(parsers.count(func_name)==0) {
      fprintf(stderr,"Invalid function request %s, killing client\n",func_name.c_str());
      return false;
    }
    Timer ptimer;
    if(!parsers[func_name](in,out)) {
      fprintf(stderr,"Error servicing function request %s, killing client\n",func_name.c_str());
      fprintf(stderr,"  Offending message: \"%s\"\n",buf);
      return false;
    }
    processtime += ptimer.ElapsedTime();
    //cout<<"Writing message \""<<out.str()<<"\""<<endl;
    if(!client->WriteString(out.str().c_str())) {
      fprintf(stderr,"Error writing response to %s, killing client\n",func_name.c_str());
      return false;
    }
    msgnum ++;
    msgtime += timer.ElapsedTime();
    if(msgnum % 100 == 0) {
      printf("Average message response time: %gs\n",msgtime / msgnum);
      printf("  parsing / processing args: %gs\n",processtime / msgnum);
      msgnum = 0;
      msgtime = 0;
      processtime = 0;
    }
    return true;
  }
};

void killFunc(int dummy=0) {
  printf("Ctrl+C pressed, sending shutdown\n");
  sendShutdown();
  exit(0);
}

void run_motion_server(const char* serverAddr)
{
  printf("Running motion RPC server at %s, %d max clients... \n",serverAddr,10);
  printf("(press Ctrl+C to exit)\n");
  signal(SIGINT,killFunc);
  MyRequestReplyServer server(serverAddr,10);
  if(!server.Start()) return;
  server.Run();
}
