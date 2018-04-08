#include "motion.h"
#include <stdlib.h>
#include <string.h>
#include <KrisLibrary/myfile.h>
#include <KrisLibrary/Timer.h>
#include <KrisLibrary/errors.h>
#include <KrisLibrary/utils/AnyValue.h>
#include <KrisLibrary/utils/ioutils.h>
#include <KrisLibrary/utils/socketutils.h>
#include <KrisLibrary/utils/stringutils.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sstream>
using namespace std;

string serverAddr = "tcp://localhost:8001";
File f;
stringstream ss;
int msg_id = 0;

APIENTRY BOOL setServerAddr(const char* addr)
{
  if(!StartsWith(addr,"tcp://") && !StartsWith(addr,"udp://"))
    serverAddr = string("tcp://")+string(addr);
  else
    serverAddr = addr;
  if(strchr(addr,':')==0)
    serverAddr = serverAddr+string(":8001");
  return 1;
}

inline int NumLimbDofs(int limb) { if(limb==BOTH) return 2*numLimbDofs; return numLimbDofs; }

#define STRING const char* 

#define DECLCACHED(rettype,func) \
  bool func ## _read = false; \
  rettype func ## _value; \
  rettype cached_## func() { \
    if(! func ## _read) { ( func ## _value )= func(); ( func ## _read ) = true; } \
    return func ##_value; }
#define CACHED(func) cached_## func ()

#define DECLCACHED1(rettype,func,n)		\
  bool func ## _read = false;			\
  rettype func ## _value [n];	      \
  rettype cached_## func(int index) { \
    if(! func ## _read) { for(int i=0;i<n;i++) ( func ## _value )[i] = func(i); ( func ## _read ) = true; } \
    return func ##_value[index]; }
#define CACHED1(func,arg)  cached_ ## func (arg)

void EnsureOpen() {
  if(!f.IsOpen()) {
    SOCKET sockfd = Connect(serverAddr.c_str());
    if(sockfd == INVALID_SOCKET) {
      fprintf(stderr,"Motion client: Could not open connection to motion server %s\n",serverAddr.c_str());
      exit(1);
    } 
    int flag = 1;
    int result = setsockopt(sockfd,            /* socket affected */
                        IPPROTO_TCP,     /* set option at TCP level */
                        TCP_NODELAY,     /* name of option */
                        (char *) &flag,  /* the cast is historical cruft */
                        sizeof(int));    /* length of option value */
   if(!f.OpenTCPSocket(sockfd)) {
      fprintf(stderr,"Motion client: Could not open connection to motion server\n");
      exit(1);
    }    
  }
}

string RECV_RAW()
{
  string s;
  ss>>s;
  /*
  if(!SafeInputString(ss,s)) 
    fprintf(stderr,"Message %d has incorrect number of values!\n",msg_id-1);
  */
  return s;
}

template <class T>
bool RECV_RAW(T& val)
{
  ss>>val;
  if(!ss) {
    fprintf(stderr,"Motion client: Message %d has incorrect number of values!\n",msg_id-1);
    return false;
  }
  return true;
}

template <>
bool RECV_RAW(string& val)
{
  bool res = SafeInputString(ss,val);
  if(!res) {
    fprintf(stderr,"Motion client: Message %d has incorrect number of values!\n",msg_id-1);
    return false;
  }
  return true;
}


template <class T>
bool RECV_RETVAL(T& val)
{
  Assert(f.IsOpen());
  char buf[4096];
  if(!f.ReadString(buf,4096)) {
    fprintf(stderr,"Motion client: Error reading string, server must have died, or string too large\n");
    return false;
  }
  ss.str("");
  ss.clear();
  ss.str(buf);
  //printf("Read message \"%s\"\n",buf);
  int id;
  string str;
  ss>>id;
  if(!ss || id!=msg_id) {
    fprintf(stderr,"Motion client: Reply message has incorrect message id! (%d, should have %d)\n",id,msg_id);
    msg_id = (msg_id+1)%1000;
    return false;
  }
  ss>>val;
  msg_id = (msg_id+1)%1000;
  if(!ss) {
    fprintf(stderr,"Motion client: Invalid return type for msg %d",msg_id-1);
    return false;
  }
  return true;
}

#define SEND_CALL_BEGIN(func) EnsureOpen(); stringstream ss; ss<<msg_id<<" "<<#func;
#define SEND_CALL_ARG(type,arg) ss<<" "<<arg;
#define SEND_CALL_ARGV(type,arg,n) for(int i=0;i<n;i++) ss<<" "<<arg[i];
#define SEND_CALL_END() \
  /* printf("Sent message \"%s\"\n",ss.str().c_str());*/	\
  f.WriteString(ss.str().c_str()); 
#define RECV_CALL(type,name) type name; if(!RECV_RETVAL<type>(name)) return 0;

#define DECLSEND(rettype,func) \
  APIENTRY rettype func() {  \
    SEND_CALL_BEGIN(func); \
    SEND_CALL_END(); \
    RECV_CALL(rettype,res);				\
    return res; }

#define DECLSEND_V(rettype,func,argtype)		\
  APIENTRY rettype func(argtype arg) {  \
    SEND_CALL_BEGIN(func); \
    SEND_CALL_ARG(argtype,arg);		\
    SEND_CALL_END(); \
    RECV_CALL(rettype,res); \
    return res; }

#define DECLSEND_VV(rettype,func,arg1type,arg2type)	\
  APIENTRY rettype func(arg1type arg1,arg2type arg2) {  \
    SEND_CALL_BEGIN(func); \
    SEND_CALL_ARG(arg1type,arg1);		\
    SEND_CALL_ARG(arg2type,arg2); \
    SEND_CALL_END(); \
    RECV_CALL(rettype,res); \
    return res; }

#define DECLSEND_VVV(rettype,func,arg1type,arg2type,arg3type) \
  APIENTRY rettype func(arg1type arg1,arg2type arg2,arg3type arg3) {			\
    SEND_CALL_BEGIN(func); \
    SEND_CALL_ARG(arg1type,arg1); \
    SEND_CALL_ARG(arg2type,arg2); \
    SEND_CALL_ARG(arg3type,arg3); \
    SEND_CALL_END(); \
    RECV_CALL(rettype,res);				\
    return res; }

#define DECLSEND_VVVV(rettype,func,arg1type,arg2type,arg3type,arg4type) \
  APIENTRY rettype func(arg1type arg1,arg2type arg2,arg3type arg3,arg4type arg4) { \
    SEND_CALL_BEGIN(func); \
    SEND_CALL_ARG(arg1type,arg1); \
    SEND_CALL_ARG(arg2type,arg2); \
    SEND_CALL_ARG(arg3type,arg3); \
    SEND_CALL_ARG(arg4type,arg4); \
    SEND_CALL_END(); \
    RECV_CALL(rettype,res); \
    return res; }

#define DECLSEND_A(rettype,func,arg1type,arg1size)	\
  APIENTRY rettype func(const arg1type* arg1) {  \
    int n1=arg1size; \
    SEND_CALL_BEGIN(func); \
    for(int i=0;i<n1;i++) \
      SEND_CALL_ARG(arg1type,arg1[i]);		\
    SEND_CALL_END(); \
    RECV_CALL(rettype,res); \
    return res; }


#define DECLSEND_VA(rettype,func,arg1type,arg2type,arg2size)	\
  APIENTRY rettype func(arg1type arg1,const arg2type* arg2) {  \
    int n2=arg2size; \
    SEND_CALL_BEGIN(func); \
    SEND_CALL_ARG(arg1type,arg1); \
    for(int i=0;i<n2;i++) \
      SEND_CALL_ARG(arg2type,arg2[i]);		\
    SEND_CALL_END(); \
    RECV_CALL(rettype,res); \
    return res; }

#define DECLSEND_VAV(rettype,func,arg1type,arg2type,arg2size,arg3type)	\
  APIENTRY rettype func(arg1type arg1,const arg2type* arg2,arg3type arg3) {  \
    int n2=arg2size; \
    SEND_CALL_BEGIN(func); \
    SEND_CALL_ARG(arg1type,arg1); \
    for(int i=0;i<n2;i++) \
      SEND_CALL_ARG(arg2type,arg2[i]);		\
    SEND_CALL_ARG(arg3type,arg3); \
    SEND_CALL_END(); \
    RECV_CALL(rettype,res); \
    return res; }


#define DECLSEND_VAA(rettype,func,arg1type,arg2type,arg2size,arg3type,arg3size)	\
  APIENTRY rettype func(arg1type arg1,const arg2type* arg2,const arg3type* arg3) {  \
    int n2=arg2size; \
    int n3=arg3size; \
    SEND_CALL_BEGIN(func); \
    SEND_CALL_ARG(arg1type,arg1); \
    for(int i=0;i<n2;i++) \
      SEND_CALL_ARG(arg2type,arg2[i]);		\
    for(int i=0;i<n3;i++) \
      SEND_CALL_ARG(arg3type,arg3[i]);		\
    SEND_CALL_END(); \
    RECV_CALL(rettype,res); \
    return res; }


#define DECLSEND_VVA(rettype,func,arg1type,arg2type,arg3type,arg3size)	\
  APIENTRY rettype func(arg1type arg1,arg2type arg2,const arg3type* arg3) {  \
    int n3=arg3size; \
    SEND_CALL_BEGIN(func); \
    SEND_CALL_ARG(arg1type,arg1); \
    SEND_CALL_ARG(arg2type,arg2);  \
    for(int i=0;i<n3;i++) \
      SEND_CALL_ARG(arg3type,arg3[i]);		\
    SEND_CALL_END(); \
    RECV_CALL(rettype,res); \
    return res; }

#define DECLSEND_VVAA(rettype,func,arg1type,arg2type,arg3type,arg3size,arg4type,arg4size) \
  APIENTRY rettype func(arg1type arg1,arg2type arg2,const arg3type* arg3,const arg4type* arg4) { \
    int n3=arg3size; \
    int n4=arg4size; \
    SEND_CALL_BEGIN(func); \
    SEND_CALL_ARG(arg1type,arg1); \
    SEND_CALL_ARG(arg2type,arg2); \
    for(int i=0;i<n3;i++) \
      SEND_CALL_ARG(arg3type,arg3[i]);		\
    for(int i=0;i<n4;i++) \
      SEND_CALL_ARG(arg4type,arg4[i]);		\
    SEND_CALL_END(); \
    RECV_CALL(rettype,res); \
    return res; }


#define DECLSEND_VAAA(rettype,func,arg1type,arg2type,arg2size,arg3type,arg3size,arg4type,arg4size) \
  APIENTRY rettype func(arg1type arg1,const arg2type* arg2,const arg3type* arg3,const arg4type* arg4) { \
    int n2=arg2size; \
    int n3=arg3size; \
    int n4=arg4size; \
    SEND_CALL_BEGIN(func); \
    SEND_CALL_ARG(arg1type,arg1); \
    for(int i=0;i<n2;i++) \
      SEND_CALL_ARG(arg2type,arg2[i]);		\
    for(int i=0;i<n3;i++) \
      SEND_CALL_ARG(arg3type,arg3[i]);		\
    for(int i=0;i<n4;i++) \
      SEND_CALL_ARG(arg4type,arg4[i]);		\
    SEND_CALL_END(); \
    RECV_CALL(rettype,res); \
    return res; }

#define DECLSEND_VVAA(rettype,func,arg1type,arg2type,arg3type,arg3size,arg4type,arg4size) \
  APIENTRY rettype func(arg1type arg1,arg2type arg2,const arg3type* arg3,const arg4type* arg4) { \
    int n3=arg3size; \
    int n4=arg4size; \
    SEND_CALL_BEGIN(func); \
    SEND_CALL_ARG(arg1type,arg1); \
    SEND_CALL_ARG(arg2type,arg2); \
    for(int i=0;i<n3;i++) \
      SEND_CALL_ARG(arg3type,arg3[i]);		\
    for(int i=0;i<n4;i++) \
      SEND_CALL_ARG(arg4type,arg4[i]);		\
    SEND_CALL_END(); \
    RECV_CALL(rettype,res); \
    return res; }

#define DECLSEND_VAAVVV(rettype,func,arg1type,arg2type,arg2size,arg3type,arg3size,arg4type,arg5type,arg6type) \
  APIENTRY rettype func(arg1type arg1,const arg2type* arg2,const arg3type* arg3,arg4type arg4,arg5type arg5,arg6type arg6) { \
    int n2=arg2size; \
    int n3=arg3size; \
    SEND_CALL_BEGIN(func); \
    SEND_CALL_ARG(arg1type,arg1); \
    for(int i=0;i<n2;i++) \
      SEND_CALL_ARG(arg2type,arg2[i]);		\
    for(int i=0;i<n3;i++) \
      SEND_CALL_ARG(arg3type,arg3[i]);		\
    SEND_CALL_ARG(arg4type,arg4); \
    SEND_CALL_ARG(arg5type,arg5); \
    SEND_CALL_ARG(arg6type,arg6); \
    SEND_CALL_END(); \
    RECV_CALL(rettype,res); \
    return res; }



#define DECLSEND0 DECLSEND
#define DECLSEND1 DECLSEND_V
#define DECLSEND2 DECLSEND_VV
#define DECLSEND3 DECLSEND_VVV
#define DECLSEND4 DECLSEND_VVVV

#define DECLRECV_V(rettype,func,outtype)	\
  APIENTRY rettype func(outtype* out) {  \
    SEND_CALL_BEGIN(func); \
    SEND_CALL_END(); \
    RECV_CALL(rettype,res); \
    if(!RECV_RAW(*out)) fprintf(stderr,"Motion client: Error reading return value\n"); \
    return res; }


#define DECLRECV_VV(rettype,func,out1type,out2type)	\
  APIENTRY rettype func(out1type* out1,out2type* out2) {		\
    SEND_CALL_BEGIN(func); \
    SEND_CALL_END(); \
    RECV_CALL(rettype,res); \
    if(!RECV_RAW(*out1)) fprintf(stderr,"Motion client: Error reading return value 1\n"); \
    if(!RECV_RAW(*out2)) fprintf(stderr,"Motion client: Error reading return value 2\n"); \
    return res; }

#define DECLRECV_VVV(rettype,func,out1type,out2type,out3type)			\
  APIENTRY rettype func(out1type* out1,out2type* out2,out3type* out3) {		\
    SEND_CALL_BEGIN(func); \
    SEND_CALL_END(); \
    RECV_CALL(rettype,res); \
    if(!RECV_RAW(*out1)) fprintf(stderr,"Motion client: Error reading return value 1\n"); \
    if(!RECV_RAW(*out2)) fprintf(stderr,"Motion client: Error reading return value 2\n"); \
    if(!RECV_RAW(*out3)) fprintf(stderr,"Motion client: Error reading return value 3\n"); \
    return res; }


#define DECLRECV_A(rettype,func,outtype,outsize)	\
  APIENTRY rettype func(outtype* out) {  \
    int n=outsize; \
    SEND_CALL_BEGIN(func); \
    SEND_CALL_END(); \
    RECV_CALL(rettype,res); \
    for(int i=0;i<n;i++) \
      if(!RECV_RAW(out[i])) fprintf(stderr,"Motion client: Error reading element %d of return value\n",i); \
    return res; }

#define DECLRECV_AA(rettype,func,out1type,out1size,out2type,out2size)	\
  APIENTRY rettype func(out1type* o1,out2type* o2) {				\
    int n1=out1size;\
    int n2=out2size;\
    SEND_CALL_BEGIN(func); \
    SEND_CALL_END(); \
    RECV_CALL(rettype,res); \
    for(int i=0;i<n1;i++) \
      if(!RECV_RAW(o1[i])) fprintf(stderr,"Motion client: Error reading element %d of first return value\n",i); \
    for(int i=0;i<n2;i++) \
      if(!RECV_RAW(o2[i])) fprintf(stderr,"Motion client: Error reading element %d of second return value\n",i); \
    return res; }

#define DECLRECV_AAA(rettype,func,out1type,out1size,out2type,out2size,out3type,out3size) \
  APIENTRY rettype func(out1type* o1,out2type* o2,out3type* o3) {			\
    int n1=out1size;\
    int n2=out2size;\
    int n3=out3size;\
    SEND_CALL_BEGIN(func); \
    SEND_CALL_END(); \
    RECV_CALL(rettype,res); \
    for(int i=0;i<n1;i++) \
      if(!RECV_RAW(o1[i])) { fprintf(stderr,"Motion client: Error reading element %d of first return value\n",i); return 0; } \
    for(int i=0;i<n2;i++) \
      if(!RECV_RAW(o2[i])) { fprintf(stderr,"Motion client: Error reading element %d of second return value\n",i); return 0; } \
    for(int i=0;i<n3;i++) \
      if(!RECV_RAW(o3[i])) { fprintf(stderr,"Motion client: Error reading element %d of third return value\n",i); return 0; } \
    return res; }


#define DECLSEND_V_RECV_A(rettype,func,arg1type,outtype,outsize) \
  APIENTRY rettype func(arg1type arg1,outtype* out1) { \
    int n1=outsize;\
    SEND_CALL_BEGIN(func); \
    SEND_CALL_ARG(argtype,arg1); \
    SEND_CALL_END(); \
    RECV_CALL(rettype,res); \
    for(int i=0;i<n1;i++)  \
      if(!RECV_RAW(out1[i])) fprintf(stderr,"Motion client: Error reading element %d of return value\n",i); \
    return res; }

#define DECLSEND_V_RECV_AA(rettype,func,arg1type,out1type,out1size,out2type,out2size)	\
  APIENTRY rettype func(arg1type arg1,out1type* out1,out2type* out2) {		\
    int n1=out1size;\
    int n2=out2size;\
    SEND_CALL_BEGIN(func); \
    SEND_CALL_ARG(argtype,arg1); \
    SEND_CALL_END(); \
    RECV_CALL(rettype,res); \
    for(int i=0;i<n1;i++)  \
      if(!RECV_RAW(out1[i])) fprintf(stderr,"Motion client: Error reading element %d of first return value\n",i); \
    for(int i=0;i<n2;i++)  \
      if(!RECV_RAW(out2[i])) fprintf(stderr,"Motion client: Error reading element %d of second return value\n",i); \
    return res; }


#define DECLRECV1 DECLRECV_V
#define DECLRECV2 DECLRECV_VV
#define DECLRECV3 DECLRECV_VVV

DECLSEND1(BOOL,setKlamptModel,STRING);
APIENTRY BOOL getKlamptModel(char* buf,int bufsize)
{
  SEND_CALL_BEGIN(getKlamptModel); 
  SEND_CALL_END(); 
  RECV_CALL(BOOL,res);
  string s;
  RECV_RAW<string>(s);
  if(s.length() >= bufsize) { fprintf(stderr,"Motion client: getKlamptModel: bufsize not large enough.\n"); return 0; }
  strncpy(buf,s.c_str(),bufsize);
  return res;
}
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
APIENTRY BOOL getGripperType(int limb,char* buf,int bufsize)
{
  SEND_CALL_BEGIN(getGripperType); 
  SEND_CALL_ARG(int,limb); 
  SEND_CALL_END(); 
  RECV_CALL(BOOL,res);
  string s;
  RECV_RAW<string>(s);
  if(s.length() >= bufsize) { fprintf(stderr,"Motion client: getGripperType: bufsize not large enough.\n"); return 0; }
  strcpy(buf,s.c_str());
  return res;
}
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
DECLCACHED(int,getKlamptNumDofs)
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

APIENTRY BOOL getKlamptLimb(const double* klamptConfig,int limb,double* out)
{
  int n=NumLimbDofs(limb);
  int indices[n];
  if(!getKlamptLimbIndices(limb,indices)) return 0;
  for(int i=0;i<n;i++)
    out[i] = klamptConfig[indices[i]];
  return 1;
}
APIENTRY BOOL getKlamptHeadPan(const double* klamptConfig,double* out)
{
  int index=getKlamptHeadPanIndex();
  if(index < 0) return 0;
  *out = klamptConfig[index];
  return 1;
}
APIENTRY BOOL getKlamptMobileBase(const double* klamptConfig,double* x,double* y,double* theta)
{
  int ix,iy,iz;
  if(!getKlamptMobileBaseIndices(&ix,&iy,&iz)) return 0;
  *x = klamptConfig[ix];
  *y = klamptConfig[iy];
  *theta = klamptConfig[iz];
  return 1;
}
APIENTRY BOOL getKlamptGripper(const double* klamptConfig,int limb,double* out)
{
  int n=CACHED1(numGripperDofs,limb);
  int indices[n];
  if(!getKlamptGripperIndices(limb,indices)) return 0;
  for(int i=0;i<n;i++)
    out[i] = klamptConfig[indices[i]];
  return 1;
}
APIENTRY BOOL setKlamptLimb(const double* limbConfig,int limb,double* klamptConfig)
{
  int n=NumLimbDofs(limb);
  int indices[n];
  if(!getKlamptLimbIndices(limb,indices)) return 0;
  for(int i=0;i<n;i++)
    klamptConfig[indices[i]] = limbConfig[i];
  return 1;
}
APIENTRY BOOL setKlamptHeadPan(double pan,double* klamptConfig)
{
  int index=getKlamptHeadPanIndex();
  if(index < 0) return 0;
  klamptConfig[index] = pan;
  return 1;
}
APIENTRY BOOL setKlamptMobileBase(double x,double y,double theta,double* klamptConfig)
{
  int ix,iy,iz;
  if(!getKlamptMobileBaseIndices(&ix,&iy,&iz)) return 0;
  klamptConfig[ix] = x;
  klamptConfig[iy] = y;
  klamptConfig[iz] = theta;
  return 1;
}
APIENTRY BOOL setKlamptGripper(const double* gripperConfig,int limb,double* klamptConfig)
{
  int n=CACHED1(numGripperDofs,limb);
  int indices[n];
  if(!getKlamptGripperIndices(limb,indices)) return 0;
  for(int i=0;i<n;i++) 
    klamptConfig[indices[i]] = gripperConfig[i];
  return 1;
}

