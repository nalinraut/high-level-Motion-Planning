#include "motion.h"
#include "Planning/RealTimePlanner.h"
#include "Planning/RealTimeIKPlanner.h"
#include "Modeling/ParabolicRamp.h"
#include "Interface/RobotInterface.h"
#include "IO/XmlWorld.h"
#include <KrisLibrary/math/sparsevector.h>
#include <KrisLibrary/utils/AnyCollection.h>
#include <KrisLibrary/robotics/Rotation.h>
#include <KrisLibrary/robotics/IKFunctions.h>
#include <KrisLibrary/math/angle.h>
#include <KrisLibrary/math/SVDecomposition.h>
#include <KrisLibrary/utils/stringutils.h>
#include <sspp/Topic.h>
#include <sspp/Send.h>
#include <fstream>

const char* klampt_left_limb_names [] = {
  "left_upper_shoulder",
  "left_lower_shoulder",
  "left_upper_elbow",
  "left_lower_elbow",
  "left_upper_forearm",
  "left_lower_forearm",
  "left_wrist"
};


const char* klampt_right_limb_names [] = {
  "right_upper_shoulder",
  "right_lower_shoulder",
  "right_upper_elbow",
  "right_lower_elbow",
  "right_upper_forearm",
  "right_lower_forearm",
  "right_wrist"
};

Real error(const RigidTransform& Ta,const RigidTransform& Tb)
{
  Matrix3 Rdiff;
  Rdiff.mulTransposeB(Ta.R,Tb.R);
  AngleAxisRotation aa;
  aa.setMatrix(Rdiff);
  return Abs(aa.angle) + Ta.t.distance(Tb.t);
}

bool IsFiniteV(const Vector3& x)
{
  return ::IsFinite(x.x) && ::IsFinite(x.y) && ::IsFinite(x.z);
}

const char* klampt_left_ee_name = "left_gripper";

const char* klampt_right_ee_name = "right_gripper";

const char* klampt_left_gripper_prefix = "left_gripper:";

const char* klampt_right_gripper_prefix = "right_gripper:";

const char* klampt_left_gripper_base_name = "left_gripper:base";

const char* klampt_right_gripper_base_name = "right_gripper:base";

const char* klampt_head_name = "head";

const char* klampt_base_names [] = {
  "base_x",
  "base_y",
  "base_yaw"
};


struct ControllerUpdateData;
struct BaseCalibration;

//Typedef MyControllerUpdateData to be your own control loop

struct HeadState
{
  HeadState() { pan=panTarget=panSpeed=0; isNodding=false; sendCommand=false;}
  double pan;
  bool isNodding;

  double panTarget,panSpeed;
  bool sendCommand;
};

struct LimbState
{
  LimbState(int _limb) { limb=_limb; controlMode=NONE; enableSelfCollisionAvoidance=true; senseUpdateTime=-1; sendCommand=false; firstSendTime=-Inf; governor = Normal; motionQueueActive=false;  endEffectorOffset.setZero(); }
  void Advance(Real dt,ControllerUpdateData* context);

  int limb;

  enum {Normal,EndEffectorDrive,MotionQueue,Planner};
  int governor;

  enum {NONE,POSITION,VELOCITY,EFFORT,RAW_POSITION};
  int controlMode;
  bool enableSelfCollisionAvoidance;

  double senseUpdateTime;
  Vector sensedConfig,sensedVelocity,sensedEffort;
  Vector commandedConfig,commandedVelocity,commandedEffort;
  Vector rawCommandToSend;
  bool sendCommand;

  Vector integralError;
  double firstSendTime;

  //used when the governor is EndEffectorDrive (end effector drive command)
  Vector3 endEffectorOffset;
  RigidTransform driveTransform,achievedTransform;
  Vector3 driveAngVel,driveVel;
  Real driveSpeedAdjustment;

  //used when the governor is MotionQueue or Planner
  bool motionQueueActive;
  PolynomialMotionQueue motionQueue;
};

struct BaseState
{
  BaseState() { controlMode=NONE; enabled=false; moving=false; sendCommand=false; senseUpdateTime=-1; }
  bool GetOdometryTarget(Vector& odo) const;
  bool GetRelativeTarget(Vector& trel) const;
  void ToLocal(const Vector& odoconfig,Vector& lconfig) const;
  void ToOdometry(const Vector& lconfig,Vector& odoconfig) const;
  void IntegrateCommand(const BaseCalibration& calib,Real dt);
  void DesiredToLowLevelCommand(const BaseCalibration& calib,Real dt,Vector& twist);
  bool enabled;
  bool moving;
  Vector odometry,velocity;
  Real senseUpdateTime;

  enum {NONE,RELATIVE_POSITION,ODOMETRY_POSITION,RELATIVE_VELOCITY};
  int controlMode;
  Vector command;
  bool sendCommand;
  Vector commandedPosition;  //global, extrapolated
  Vector commandedVelocity;  //relative
  Vector estimatedVelocity;  //estimated from command
};

struct GripperState
{
  GripperState() { enabled=false; name="none"; moving=false; sendCommand=false; }
  bool enabled;
  string name;
  bool moving;
  Vector position,force;

  bool sendCommand;
  Vector positionCommand,speedCommand,forceCommand;
};

struct PlannerState
{
  enum PlannerType { None, IKPlanner };

  PlannerState() { plannerActive.resize(2,false);  currentPlanner=None; }

  //whether the planner is active for the given limb
  vector<bool> plannerActive;
  RealTimePlanningThread planningThread;
  RobotWorld world;
  WorldPlannerSettings settings;
  SmartPointer<SingleRobotCSpace> cspace;
  PolynomialMotionQueue wholeRobotMotionQueue;
  SmartPointer<DefaultMotionQueueInterface> motionQueueInterface;
  PlannerType currentPlanner;
};

struct RobotState
{
  RobotState():baxterEnabled(true),baxterEstop(false),baseEstop(false),leftLimb(LEFT),rightLimb(RIGHT) {}
  bool baxterEnabled;
  bool baxterEstop,baseEstop;
  HeadState head;
  BaseState base;
  LimbState leftLimb,rightLimb;
  GripperState leftGripper,rightGripper;
};

struct KlamptGripperMap
{
  //stores a mapping from command variables to klampt variables
  //each command variable i has a basis function q[li] = si*u+oi
  //where li is the list of links, si is the vector of scale coefficients
  //and oi is the vector of offset coefficients.
  void CommandToConfig(const double* cmd,double* q) const;
  void ConfigToCommand(const double* q,double* cmd) const;
  void FingerToConfig(int index,double value,double* q) const;
  double ConfigToFinger(const double* q,int index) const;

  string name;
  vector<vector<int> > klamptIndices;
  vector<vector<double> > klamptScaleCoefs;
  vector<vector<double> > klamptOffsetCoefs;
};

struct BaseCalibration
{
 public:
  BaseCalibration();
  Real timeDelay;
  Real xVelMax,yVelMax,angVelMax;
  Real xAccMax,yAccMax,angAccMax;
  Real xFriction,yFriction,angFriction;
};

class BaxterMotorCalibration
{
public:
  BaxterMotorCalibration();
  ///Returns true if it's loaded
  bool Enabled() const;
  ///clears the calibration
  void Clear();
  ///load from the calibration JSON file
  bool Load(const char* fn,const char* limb);
  ///Computes the command for the given q,v
  void ToCommand(const Vector& q,const Vector& v,const Vector& g,
		 const vector<Vector>& qdesired,Vector& qcmd);

  string name;
  int lookahead;
  Real rate;
  vector<SparseVector> Brows,Crows;
  Vector d;
};


///Subclasses will fill this in with your own low-level controller data.
///Make sure to lock the mutex properly when changing controller data!
///Make sure to lock the robot mutex properly when changing the robot model!
struct ControllerUpdateData
{
  ControllerUpdateData()
    :startup(false),failure(false),running(false),kill(false),t(0),last_t(0),updateSystemStateRate(20),lastUpdateSystemStateTime(0)
  {}
  //Klamp't interface helpers (assume lock is already held)
  bool GetKlamptIndices();
  void KlamptToLimb(const Vector& qklampt,int limb,Vector& qlimb) const;
  void LimbToKlampt(const Vector& qlimb,int limb,Vector& qklampt) const;
  void GetKlamptSensedConfig(Vector& qklampt) const;
  void GetKlamptSensedVelocity(Vector& dqklampt) const;
  void GetKlamptCommandedConfig(Vector& qklampt) const;
  void GetKlamptCommandedVelocity(Vector& dqklampt) const;
  void GetKlamptTargetConfig(Vector& qklampt) const;

  //planner interface helpers
  void OnWorldChange();
  void SetLimbGovernor(int limb,int governor);
  bool SolveIK(int limb,const RigidTransform& T,
	       const Vector& qmin,const Vector& qmax,Real rotationTolerance,Real positionTolerance,
	       Vector& limbJointPositions,bool doLock);
  bool SolveIK(int limb,const Vector3& endPosition,const Vector& qmin,const Vector& qmax,Real positionTolerance,
	       Vector& limbJointPositions,bool doLock);
  bool SolveIKVelocity(int limb,const Vector3& angVel,const Vector3& vel,Vector& limbJointVels,bool doLock);
  bool SolveIKVelocity(int limb,const Vector3& vel,Vector& limbJointVels,bool doLock);
  ///Returns true if the configuration self-collides or collides with obstacles in the planning world
  bool CheckCollisions(const Config& qklampt);
  ///Returns true if the limb configuration self-collides or collides with obstacles in the planning world.
  ///The base / other limbs are assumed to be in their current positions and are not checked.
  bool CheckCollisions(int limb,const Config& qlimb);

  ///Subclass should fill this out
  virtual bool MyStartup() { return true; }
  ///Subclass should fill this out
  virtual bool IsHandMoving() { return true; }

  virtual bool MyProcessSensors() { return true; }
  ///Subclass does not need to fill this out
  virtual void MyAdvanceController();
  ///Subclass should fill this out
  virtual void MySendCommands() {
    if(robotState.leftLimb.sendCommand)
      robotState.leftLimb.sendCommand = false;
    if(robotState.rightLimb.sendCommand)
      robotState.rightLimb.sendCommand = false;
    if(robotState.leftGripper.sendCommand)
      robotState.leftGripper.sendCommand = false;
    if(robotState.rightGripper.sendCommand)
      robotState.rightGripper.sendCommand = false;
    if(robotState.base.sendCommand)
      robotState.base.sendCommand = false;
  }
  ///Subclass should fill this out
  virtual void MyShutdown() {}
  ///Subclass does not need to fill this out
  virtual void MyUpdateSystemStateService();

  Mutex mutex,robotMutex;
  string systemStateAddr;
  bool startup,failure,running,kill;
  RobotState robotState;
  SmartPointer<Robot> robotModel;
  string robotModelFile;
  double t,last_t;
  File systemStateService;
  PlannerState planner;
  double updateSystemStateRate,lastUpdateSystemStateTime;

  //klamp't stuff
  vector<int> leftKlamptIndices,rightKlamptIndices;
  int headPanKlamptIndex;
  vector<int> baseKlamptIndices;
  KlamptGripperMap leftGripperMap,rightGripperMap;

  //physical calibration
  BaseCalibration baseCalibration;
  BaxterMotorCalibration leftCalibration,rightCalibration;

  //IK configuration biasing
  Config ikBiasConfig;

  //collision checking
  bool enableCollisionChecking;
};

void KlamptGripperMap::CommandToConfig(const double* cmd,double* q) const
{
  for(size_t i=0;i<klamptIndices.size();i++)
    FingerToConfig((int)i,cmd[i],q);
}

void KlamptGripperMap::ConfigToCommand(const double* q,double* cmd) const
{
  for(size_t i=0;i<klamptIndices.size();i++)
    cmd[i] = ConfigToFinger(q,(int)i);
}

void KlamptGripperMap::FingerToConfig(int index,double value,double* q) const
{
  for(size_t i=0;i<klamptIndices[index].size();i++)
    q[klamptIndices[index][i]] = value*klamptScaleCoefs[index][i]+klamptOffsetCoefs[index][i];
}

double KlamptGripperMap::ConfigToFinger(const double* q,int index) const
{
  vector<double> estimates(klamptIndices[index].size());
  for(size_t i=0;i<klamptIndices[index].size();i++)
    estimates[i] = (q[klamptIndices[index][i]]-klamptOffsetCoefs[index][i])/klamptScaleCoefs[index][i];
  double sum = 0;
  for(size_t i=0;i<estimates.size();i++)
    sum += estimates[i];
  return sum / estimates.size();
}

bool ControllerUpdateData::GetKlamptIndices()
{
  bool res = true;
  leftKlamptIndices.resize(numLimbDofs);
  rightKlamptIndices.resize(numLimbDofs);
  for(int i=0;i<numLimbDofs;i++) {
    leftKlamptIndices[i] = robotModel->LinkIndex(klampt_left_limb_names[i]);
    rightKlamptIndices[i] = robotModel->LinkIndex(klampt_right_limb_names[i]);
    if(leftKlamptIndices[i] < 0) {
      printf("Klamp't model does not contain link %s\n",klampt_left_limb_names[i]);
      res = false;
    }
    if(rightKlamptIndices[i] < 0) {
      printf("Klamp't model does not contain link %s\n",klampt_right_limb_names[i]);
      res = false;
    }
  }
  headPanKlamptIndex = robotModel->LinkIndex(klampt_head_name);
  if(headPanKlamptIndex < 0) {
    printf("Klamp't model does not contain link %s\n",klampt_head_name);
    res = false;
  }
  baseKlamptIndices.resize(3);
  for(int i=0;i<3;i++) {
    baseKlamptIndices[i] = robotModel->LinkIndex(klampt_base_names[i]);
    if(baseKlamptIndices[i] < 0) {
      printf("Klamp't model does not contain base link %s\n",klampt_base_names[i]);
      baseKlamptIndices.resize(0);
      res = false;
      break;
    }
  }
  /*
  //DEBUG: see all link names
  for(size_t i=0;i<robotModel->linkNames.size();i++)
    cout<<robotModel->linkNames[i]<<endl;
  */
  int left_gripper_base = robotModel->LinkIndex(klampt_left_gripper_base_name);
  int right_gripper_base = robotModel->LinkIndex(klampt_right_gripper_base_name);
  // printf("Gripper base links: %d %d\n",left_gripper_base,right_gripper_base);
  int left_gripper_num_links = 0;
  int right_gripper_num_links = 0;
  if(left_gripper_base >= 0) {
    for(int i=left_gripper_base;i<robotModel->q.n;i++)
      if(StartsWith(robotModel->linkNames[i].c_str(),klampt_left_gripper_prefix))
        left_gripper_num_links++;
      else
        break;
  }
  if(right_gripper_base >= 0) {
    for(int i=right_gripper_base;i<robotModel->q.n;i++)
      if(StartsWith(robotModel->linkNames[i].c_str(),klampt_right_gripper_prefix))
        right_gripper_num_links++;
      else
        break;
  }
  if(left_gripper_num_links == 3) {
    //must be rethink electric gripper
    leftGripperMap.name = "Rethink Electric Gripper";
    leftGripperMap.klamptIndices.resize(1);
    leftGripperMap.klamptScaleCoefs.resize(1);
    leftGripperMap.klamptOffsetCoefs.resize(1);
    leftGripperMap.klamptIndices[0].resize(2);
    leftGripperMap.klamptIndices[0][0] = left_gripper_base+1;
    leftGripperMap.klamptIndices[0][1] = left_gripper_base+2;
    leftGripperMap.klamptScaleCoefs[0].resize(2);
    leftGripperMap.klamptScaleCoefs[0][0] = robotModel->qMax[left_gripper_base+1];
    leftGripperMap.klamptScaleCoefs[0][1] = robotModel->qMin[left_gripper_base+2];
    leftGripperMap.klamptOffsetCoefs[0].resize(2,0.0);
  }
  else if(left_gripper_num_links == 16 || left_gripper_num_links == 43) {
    //must be reflex gripper
    double proxmax = 2.83;
    double swivelmax = 1.57;
    leftGripperMap.name = "Reflex";
    leftGripperMap.klamptIndices.resize(4);
    leftGripperMap.klamptScaleCoefs.resize(4);
    leftGripperMap.klamptOffsetCoefs.resize(4);
    //finger1
    leftGripperMap.klamptIndices[0].resize(1,robotModel->LinkIndex("left_gripper:proximal_1"));
    leftGripperMap.klamptScaleCoefs[0].resize(1,-proxmax);
    leftGripperMap.klamptOffsetCoefs[0].resize(1,proxmax);
    leftGripperMap.klamptIndices[1].resize(1,robotModel->LinkIndex("left_gripper:proximal_2"));
    leftGripperMap.klamptScaleCoefs[1].resize(1,-proxmax);
    leftGripperMap.klamptOffsetCoefs[1].resize(1,proxmax);
    leftGripperMap.klamptIndices[2].resize(1,robotModel->LinkIndex("left_gripper:proximal_3"));
    leftGripperMap.klamptScaleCoefs[2].resize(1,-proxmax);
    leftGripperMap.klamptOffsetCoefs[2].resize(1,proxmax);
    leftGripperMap.klamptIndices[3].resize(2);
    leftGripperMap.klamptIndices[3][0] = robotModel->LinkIndex("left_gripper:swivel_1");
    leftGripperMap.klamptIndices[3][1] = robotModel->LinkIndex("left_gripper:swivel_2");
    leftGripperMap.klamptScaleCoefs[3].resize(2);
    leftGripperMap.klamptScaleCoefs[3][0] = -swivelmax;
    leftGripperMap.klamptScaleCoefs[3][1] = swivelmax;
    leftGripperMap.klamptOffsetCoefs[3].resize(2);
    leftGripperMap.klamptOffsetCoefs[3][0] = swivelmax;
    leftGripperMap.klamptOffsetCoefs[3][1] = -swivelmax;
  }
  else
    printf("Motion library warning: left gripper has strange number of DOFs: %d\n",left_gripper_num_links);
  if(right_gripper_num_links == 3) {
    //must be rethink electric gripper
    rightGripperMap.name = "Rethink Electric Gripper";
    rightGripperMap.klamptIndices.resize(1);
    rightGripperMap.klamptScaleCoefs.resize(1);
    rightGripperMap.klamptOffsetCoefs.resize(1);
    rightGripperMap.klamptIndices[0].resize(2);
    rightGripperMap.klamptIndices[0][0] = right_gripper_base+1;
    rightGripperMap.klamptIndices[0][1] = right_gripper_base+2;
    rightGripperMap.klamptScaleCoefs[0].resize(2);
    rightGripperMap.klamptScaleCoefs[0][0] = robotModel->qMax[right_gripper_base+1];
    rightGripperMap.klamptScaleCoefs[0][1] = robotModel->qMin[right_gripper_base+2];
    rightGripperMap.klamptOffsetCoefs[0].resize(2,0.0);
  }
  else if(right_gripper_num_links == 16 || right_gripper_num_links == 43) {
    //must be reflex gripper
    double proxmax = 2.83;
    double swivelmax = 1.57;
    rightGripperMap.name = "Reflex";
    rightGripperMap.klamptIndices.resize(4);
    rightGripperMap.klamptScaleCoefs.resize(4);
    rightGripperMap.klamptOffsetCoefs.resize(4);
    //finger1
    rightGripperMap.klamptIndices[0].resize(1,robotModel->LinkIndex("right_gripper:proximal_1"));
    rightGripperMap.klamptScaleCoefs[0].resize(1,-proxmax);
    rightGripperMap.klamptOffsetCoefs[0].resize(1,proxmax);
    rightGripperMap.klamptIndices[1].resize(1,robotModel->LinkIndex("right_gripper:proximal_2"));
    rightGripperMap.klamptScaleCoefs[1].resize(1,-proxmax);
    rightGripperMap.klamptOffsetCoefs[1].resize(1,proxmax);
    rightGripperMap.klamptIndices[2].resize(1,robotModel->LinkIndex("right_gripper:proximal_3"));
    rightGripperMap.klamptScaleCoefs[2].resize(1,-proxmax);
    rightGripperMap.klamptOffsetCoefs[2].resize(1,proxmax);
    rightGripperMap.klamptIndices[3].resize(2);
    rightGripperMap.klamptIndices[3][0] = robotModel->LinkIndex("right_gripper:swivel_1");
    rightGripperMap.klamptIndices[3][1] = robotModel->LinkIndex("right_gripper:swivel_2");
    rightGripperMap.klamptScaleCoefs[3].resize(2);
    rightGripperMap.klamptScaleCoefs[3][0] = -swivelmax;
    rightGripperMap.klamptScaleCoefs[3][1] = swivelmax;
    rightGripperMap.klamptOffsetCoefs[3].resize(2);
    rightGripperMap.klamptOffsetCoefs[3][0] = swivelmax;
    rightGripperMap.klamptOffsetCoefs[3][1] = -swivelmax;
  }
  else
    printf("Motion library warning: right gripper has strange number of DOFs: %d\n",right_gripper_num_links);
  // printf("Left gripper: %s\n",leftGripperMap.name.c_str());
  // printf("Right gripper: %s\n",rightGripperMap.name.c_str());

  //setup the IK bias configuration to the center of the joint ranges
  ikBiasConfig = 0.5*(robotModel->qMax + robotModel->qMin);
  for(int i=0;i<ikBiasConfig.n;i++)
    if(!IsFinite(ikBiasConfig[i])) ikBiasConfig[i] = 0;
  return res;
}

void ControllerUpdateData::KlamptToLimb(const Vector& qklampt,int limb,Vector& qlimb) const
{
  if(limb==LEFT) {
    qlimb.resize(numLimbDofs);
    for(int i=0;i<numLimbDofs;i++) {
      if(i < (int)leftKlamptIndices.size() && leftKlamptIndices[i] >= 0)
	qlimb[i] = qklampt[leftKlamptIndices[i]];
      else
	qlimb[i] = 0;
    }
  }
  else if(limb==RIGHT) {
    qlimb.resize(numLimbDofs);
    for(int i=0;i<numLimbDofs;i++) {
      if(i < (int)rightKlamptIndices.size() && rightKlamptIndices[i] >= 0)
	qlimb[i] = qklampt[rightKlamptIndices[i]];
      else
	qlimb[i] = 0;
    }
  }
  else {
    qlimb.resize(numLimbDofs*2);
    for(int i=0;i<numLimbDofs;i++) {
      if(i < (int)leftKlamptIndices.size() && leftKlamptIndices[i] >= 0)
	qlimb[i] = qklampt[leftKlamptIndices[i]];
      else
	qlimb[i] = 0;
    }
    for(int i=0;i<numLimbDofs;i++) {
      if(i < (int)rightKlamptIndices.size() && rightKlamptIndices[i] >= 0)
	qlimb[i+numLimbDofs] = qklampt[rightKlamptIndices[i]];
      else
	qlimb[i+numLimbDofs] = 0;
    }
  }
}

void ControllerUpdateData::LimbToKlampt(const Vector& qlimb,int limb,Vector& qklampt) const
{
  if(limb==LEFT) {
    for(int i=0;i<numLimbDofs;i++) {
      if(i < (int)leftKlamptIndices.size() && leftKlamptIndices[i] >= 0)
	qklampt[leftKlamptIndices[i]] = qlimb[i];
    }
  }
  else if(limb==RIGHT) {
    for(int i=0;i<numLimbDofs;i++) {
      if(i < (int)rightKlamptIndices.size() && rightKlamptIndices[i] >= 0)
	qklampt[rightKlamptIndices[i]] = qlimb[i];
    }
  }
  else {
    for(int i=0;i<numLimbDofs;i++) {
      if(i < (int)leftKlamptIndices.size() && leftKlamptIndices[i] >= 0)
	qklampt[leftKlamptIndices[i]] = qlimb[i];
    }
    for(int i=0;i<numLimbDofs;i++) {
      if(i < (int)rightKlamptIndices.size() && rightKlamptIndices[i] >= 0)
	qklampt[rightKlamptIndices[i]] = qlimb[i+numLimbDofs];
    }
  }
}

void ControllerUpdateData::GetKlamptSensedConfig(Vector& qklampt) const
{
  if(!robotModel) return;
  qklampt = robotModel->q;
  for(size_t i=0;i<leftKlamptIndices.size();i++)
    if(leftKlamptIndices[i] >= 0 && !robotState.leftLimb.sensedConfig.empty())
      qklampt[leftKlamptIndices[i]] = robotState.leftLimb.sensedConfig[i];
  for(size_t i=0;i<rightKlamptIndices.size();i++)
    if(rightKlamptIndices[i] >= 0 && !robotState.rightLimb.sensedConfig.empty())
      qklampt[rightKlamptIndices[i]] = robotState.rightLimb.sensedConfig[i];
  if(robotState.base.enabled) {
    for(size_t i=0;i<baseKlamptIndices.size();i++)
      if(baseKlamptIndices[i] >= 0)
	qklampt[baseKlamptIndices[i]] = robotState.base.odometry[i];
  }
  if(headPanKlamptIndex >= 0)
    qklampt[headPanKlamptIndex] = robotState.head.pan;

  if(!leftGripperMap.name.empty() && !robotState.leftGripper.position.empty())
    leftGripperMap.CommandToConfig(&robotState.leftGripper.position[0],&qklampt[0]);
  if(!rightGripperMap.name.empty() && !robotState.rightGripper.position.empty())
    rightGripperMap.CommandToConfig(&robotState.rightGripper.position[0],&qklampt[0]);
}

void ControllerUpdateData::GetKlamptSensedVelocity(Vector& dqklampt) const
{
  if(!robotModel) return;
  dqklampt = robotModel->dq;
  for(size_t i=0;i<leftKlamptIndices.size();i++)
    if(leftKlamptIndices[i] >= 0 && !robotState.leftLimb.sensedVelocity.empty())
      dqklampt[leftKlamptIndices[i]] = robotState.leftLimb.sensedVelocity[i];
  for(size_t i=0;i<rightKlamptIndices.size();i++)
    if(rightKlamptIndices[i] >= 0 && !robotState.rightLimb.sensedVelocity.empty())
      dqklampt[rightKlamptIndices[i]] = robotState.rightLimb.sensedVelocity[i];
  if(robotState.base.enabled && robotState.base.velocity.size() == 3) {
    Vector wvelocity;
    Matrix2 R; R.setRotate(robotState.base.odometry[2]);
    Vector2 wvel = R*Vector2(robotState.base.velocity[0],robotState.base.velocity[1]);
    wvelocity.resize(3);
    wvelocity[0] = wvel.x;
    wvelocity[1] = wvel.y;
    wvelocity[2] = robotState.base.velocity[2];
    for(size_t i=0;i<baseKlamptIndices.size();i++)
      if(baseKlamptIndices[i] >= 0)
	dqklampt[baseKlamptIndices[i]] = wvelocity[i];
  }
  if(headPanKlamptIndex >= 0) {
    double sign = Sign(robotState.head.panTarget - robotState.head.pan);
    dqklampt[headPanKlamptIndex] = robotState.head.panSpeed*sign;
  }

  if(!leftGripperMap.name.empty() && !robotState.leftGripper.speedCommand.empty()) {
    Vector zero(robotState.leftGripper.speedCommand.n,0.0);
    leftGripperMap.CommandToConfig(&zero[0],&dqklampt[0]);
  }
  if(!rightGripperMap.name.empty() && !robotState.rightGripper.speedCommand.empty()) {
    Vector zero(robotState.rightGripper.speedCommand.n,0.0);
    rightGripperMap.CommandToConfig(&zero[0],&dqklampt[0]);
  }
}

void ControllerUpdateData::GetKlamptCommandedConfig(Vector& qklampt) const
{
  if(!robotModel) return;
  qklampt = robotModel->q;
  if(!robotState.leftLimb.commandedConfig.empty()) {
    for(size_t i=0;i<leftKlamptIndices.size();i++)
      if(leftKlamptIndices[i] >= 0)
	qklampt[leftKlamptIndices[i]] = robotState.leftLimb.commandedConfig[i];
  }
  if(!robotState.rightLimb.commandedConfig.empty()) {
    for(size_t i=0;i<rightKlamptIndices.size();i++)
      if(rightKlamptIndices[i] >= 0)
	qklampt[rightKlamptIndices[i]] = robotState.rightLimb.commandedConfig[i];
  }
  if(robotState.base.enabled) {
    if(robotState.base.controlMode == BaseState::NONE) {
      for(size_t i=0;i<baseKlamptIndices.size();i++)
	if(baseKlamptIndices[i] >= 0)
	  qklampt[baseKlamptIndices[i]] = robotState.base.odometry[i];
    }
    else {
      Vector pos = robotState.base.commandedPosition;
      if(pos.empty()) pos=robotState.base.odometry;
      for(size_t i=0;i<baseKlamptIndices.size();i++)
	if(baseKlamptIndices[i] >= 0)
	  qklampt[baseKlamptIndices[i]] = pos[i];
    }
  }
  if(headPanKlamptIndex >= 0)
    qklampt[headPanKlamptIndex] = robotState.head.panTarget;

  if(!leftGripperMap.name.empty() && !robotState.leftGripper.positionCommand.empty())
    leftGripperMap.CommandToConfig(&robotState.leftGripper.positionCommand[0],&qklampt[0]);
  else
    printf("No left gripper position command?\n");
  if(!rightGripperMap.name.empty() && !robotState.rightGripper.positionCommand.empty())
    rightGripperMap.CommandToConfig(&robotState.rightGripper.positionCommand[0],&qklampt[0]);
  else
    printf("No right gripper position command?\n");
}

void ControllerUpdateData::GetKlamptTargetConfig(Vector& qklampt) const
{
  if(!robotModel) return;
  qklampt = robotModel->q;
  if(!robotState.leftLimb.commandedConfig.empty()) {
    Config q = robotState.leftLimb.commandedConfig;
    if(robotState.leftLimb.motionQueueActive)
      q = robotState.leftLimb.motionQueue.Endpoint();
    for(size_t i=0;i<leftKlamptIndices.size();i++)
      if(leftKlamptIndices[i] >= 0)
  qklampt[leftKlamptIndices[i]] = q[i];
  }
  if(!robotState.rightLimb.commandedConfig.empty()) {
    Config q = robotState.rightLimb.commandedConfig;
    if(robotState.rightLimb.motionQueueActive)
      q = robotState.rightLimb.motionQueue.Endpoint();
    for(size_t i=0;i<rightKlamptIndices.size();i++)
      if(rightKlamptIndices[i] >= 0)
  qklampt[rightKlamptIndices[i]] = q[i];
  }
  if(robotState.base.enabled) {
    Vector odo(3);
    robotState.base.GetOdometryTarget(odo);
    for(size_t i=0;i<baseKlamptIndices.size();i++)
      if(baseKlamptIndices[i] >= 0)
        qklampt[baseKlamptIndices[i]] = odo[i];
  }
  if(headPanKlamptIndex >= 0)
    qklampt[headPanKlamptIndex] = robotState.head.panTarget;

  if(!leftGripperMap.name.empty() && !robotState.leftGripper.positionCommand.empty())
    leftGripperMap.CommandToConfig(&robotState.leftGripper.positionCommand[0],&qklampt[0]);
  if(!rightGripperMap.name.empty() && !robotState.rightGripper.positionCommand.empty())
    rightGripperMap.CommandToConfig(&robotState.rightGripper.positionCommand[0],&qklampt[0]);
}


void ControllerUpdateData::GetKlamptCommandedVelocity(Vector& dqklampt) const
{
  if(!robotModel) return;
  dqklampt = robotModel->dq;
  for(size_t i=0;i<leftKlamptIndices.size();i++)
    if(leftKlamptIndices[i] >= 0 && !robotState.leftLimb.commandedVelocity.empty())
      dqklampt[leftKlamptIndices[i]] = robotState.leftLimb.commandedVelocity[i];
  for(size_t i=0;i<rightKlamptIndices.size();i++)
    if(rightKlamptIndices[i] >= 0 && !robotState.rightLimb.commandedVelocity.empty())
      dqklampt[rightKlamptIndices[i]] = robotState.rightLimb.commandedVelocity[i];
  if(robotState.base.enabled) {
    if(robotState.base.controlMode != BaseState::NONE) {
      Vector pos = robotState.base.commandedPosition;
      if(pos.empty()) pos=robotState.base.odometry;
      Vector wvelocity;
      Matrix2 R; R.setRotate(pos[2]);
      Vector2 wvel = R*Vector2(robotState.base.commandedVelocity[0],robotState.base.commandedVelocity[1]);
      wvelocity.resize(3);
      wvelocity[0] = wvel.x;
      wvelocity[1] = wvel.y;
      wvelocity[2] = robotState.base.command[2];
      for(size_t i=0;i<baseKlamptIndices.size();i++)
	if(baseKlamptIndices[i] >= 0)
	  dqklampt[baseKlamptIndices[i]] = wvelocity[i];
    }
    else {
      for(size_t i=0;i<baseKlamptIndices.size();i++)
	if(baseKlamptIndices[i] >= 0)
	  dqklampt[baseKlamptIndices[i]] = 0;
    }
  }
  if(headPanKlamptIndex >= 0) {
    double sign = Sign(robotState.head.panTarget - robotState.head.pan);
    dqklampt[headPanKlamptIndex] = robotState.head.panSpeed*sign;
  }

  if(!leftGripperMap.name.empty() && !robotState.leftGripper.speedCommand.empty())
    leftGripperMap.CommandToConfig(&robotState.leftGripper.speedCommand[0],&dqklampt[0]);
  if(!rightGripperMap.name.empty() && !robotState.rightGripper.speedCommand.empty())
    rightGripperMap.CommandToConfig(&robotState.rightGripper.speedCommand[0],&dqklampt[0]);
}

void LimbState::Advance(Real dt,ControllerUpdateData* controller)
{
  bool checkCollisions = false;
  Vector x,v;
  if(motionQueueActive) {
    if(motionQueue.TimeRemaining() >= 0) {
      //update motion queue and set command
      motionQueue.Advance(dt);
      x = motionQueue.CurConfig();
      v = motionQueue.CurVelocity();
    }
  }
  //end effector driving
  if(governor == LimbState::EndEffectorDrive && (!driveVel.isZero() || !driveAngVel.isZero())) {
    Vector qlimb;
    Robot* robot = controller->robotModel;
    int ee_index = robot->LinkIndex((limb==LEFT?klampt_left_ee_name:klampt_right_ee_name));
    const vector<int>& indices = (limb==LEFT? controller->leftKlamptIndices : controller->rightKlamptIndices);
    //cout<<"Driving end effector "<<limb<<" from transform: "<<driveTransform<<endl;
    //cout<<"  dt: "<<dt<<", drive vel "<<driveVel<<", ang vel "<<driveAngVel<<endl;
    //cout<<"  Desired transform: "<<desiredTransform.t<<endl;

    //set some bounds according to maximum one-step movement
    Vector qmin,qmax;
    controller->GetKlamptCommandedConfig(qmin);
    for(size_t i=0;i<controller->baseKlamptIndices.size();i++)
      qmin[controller->baseKlamptIndices[i]]=0;
    {
      ScopedLock lock(controller->robotMutex);
      robot->UpdateConfig(qmin);
      RigidTransform originalTransform = robot->links[ee_index].T_World;
    }
    qmax = qmin;
    for(size_t i=0;i<indices.size();i++) {
      qmax[indices[i]] += robot->velMax[indices[i]]*dt;
      qmin[indices[i]] -= robot->velMax[indices[i]]*dt;
      qmax[indices[i]] = Min(qmax[indices[i]],robot->qMax[indices[i]]);
      qmin[indices[i]] = Max(qmin[indices[i]],robot->qMin[indices[i]]);
    }

    Real amount = dt * driveSpeedAdjustment;
    bool res;
    if(IsFiniteV(driveAngVel)) {
      RigidTransform increment;
      MomentRotation m(driveAngVel * amount);
      m.getMatrix(increment.R);
      increment.t = driveVel * amount;
      RigidTransform desiredTransform;
      desiredTransform.R = increment.R * driveTransform.R;
      //cout<<"Current rotation"<<endl<<driveTransform.R<<endl;
      //cout<<"Desired rotation"<<endl<<desiredTransform.R<<endl;
      desiredTransform.t = increment.t + driveTransform.t;
      //normalize drive transform rotation matrix to correct for numerical error
      //NormalizeRotation(desiredTransform.R);
      double acceptableDeviation = 5e-3;
      res = controller->SolveIK(limb,desiredTransform,qmin,qmax,acceptableDeviation,acceptableDeviation,qlimb,false);
    }
    else {
      //position only
      double acceptableDeviation = 5e-3;
      res = controller->SolveIK(limb,driveVel*amount + driveTransform.t,qmin,qmax,acceptableDeviation,qlimb,false);
    }

    if(res) {
      //cout<<"  Solved limb position: "<<qlimb<<endl;
      //update achievedTransform
      {
        ScopedLock lock(controller->robotMutex);
        for(int i=0;i<qlimb.n;i++) {
          assert(qmin[indices[i]]<=qlimb[i] && qlimb[i]<=qmax[indices[i]]);
          robot->q[indices[i]] = qlimb[i];
        }
        robot->UpdateSelectedFrames(ee_index);
        achievedTransform.R = robot->links[ee_index].T_World.R;
        achievedTransform.t = robot->links[ee_index].T_World*endEffectorOffset;
        //cout<<"  Solved limb transform: "<<achievedTransform.t<<endl;
      }
      Real distance;
      //adjust drive transform along screw to minimize distance to the achieved transform
      Vector3 trel = achievedTransform.t - driveTransform.t;
      Vector3 axis = driveVel / Max(driveVel.length(),Epsilon);
      Real tdistance = trel.dot(axis);
      //cout<<"  translation vector"<<trel<<endl;
      //printf("  Translation amount: %g\n",tdistance);
      tdistance = Clamp(tdistance,0.0,dt*driveVel.length());
      if(IsFiniteV(driveAngVel)) {
	Matrix3 Rrel;
	Rrel.mulTransposeB(achievedTransform.R,driveTransform.R);
	Vector3 rotaxis = driveAngVel / Max(driveAngVel.length(),Epsilon);
	Real Rdistance = AxisRotationMagnitude(Rrel,rotaxis);
	//printf("  Rotation amount: %g (desired %g)\n",Rdistance,dt*driveAngVel.length());
	Rdistance = Clamp(Rdistance,0.0,dt*driveAngVel.length());
	Real ut=0,uR=0;
	ut = driveVel.length();
	uR = driveAngVel.length();
	distance = (tdistance*ut+Rdistance*uR)/Max(Sqr(ut)+Sqr(uR),Epsilon);
      }
      else {
	distance = tdistance / Max(driveVel.length(),Epsilon);
      }
      //printf("Distance %g\n",distance);

      //computed error-minimizing distance along screw motion
      driveTransform.t.madd(driveVel,distance);
      if(IsFiniteV(driveAngVel)) {
	MomentRotation m;
	Matrix3 Rincrement;
	m.set(driveAngVel*distance);
	m.getMatrix(Rincrement);
	driveTransform.R = Rincrement * driveTransform.R;
	NormalizeRotation(driveTransform.R);
      }
      else
	driveTransform.R = achievedTransform.R;

      x = qlimb;
      v = (qlimb - commandedConfig) * (1.0/dt);
      sendCommand = true;
      checkCollisions = true;

      //increase drive velocity
      if(driveSpeedAdjustment < 1.0)
	driveSpeedAdjustment += 0.1;
    }
    else {
      driveSpeedAdjustment -= 0.1;
      //printf("  Solve failed, Trying with amount %g\n",driveSpeedAdjustment);
    }
    if(driveSpeedAdjustment <= Epsilon) {
      //don't adjust drive transform
      printf("  EndEffectorDrive: IK solve failed\n");
      driveAngVel.setZero();
      driveVel.setZero();
    }
  }
  //copy joint command to limb state
  if(!x.empty()) {
    if(!checkCollisions || !controller->enableCollisionChecking || !controller->CheckCollisions(limb,x)) {
      //collision check was off or passed collision check
      if(enableSelfCollisionAvoidance)
        controlMode = LimbState::POSITION;
      else
        controlMode = LimbState::RAW_POSITION;
      sendCommand = true;
      commandedConfig = x;
      commandedVelocity = v;
      rawCommandToSend = x;
    }
  }
  if(controlMode == LimbState::VELOCITY) {
    //integrate commanded config
    commandedConfig.madd(commandedVelocity,dt);
    rawCommandToSend = commandedVelocity;
  }
  if(controlMode == LimbState::EFFORT)
    rawCommandToSend = commandedEffort;
    //TODO: difference commanded config to get commanded velocity?

  //test whether to use the motor calibration
  BaxterMotorCalibration* calib = NULL;
  if(limb == LEFT)
    calib = &controller->leftCalibration;
  else if(limb == RIGHT)
    calib = &controller->rightCalibration;

  if(integralError.empty())
    integralError.resize(sensedConfig.size(),0.0);
  for(int i=0;i<integralError.n;i++) {
    //inverse weight the error by the velocity to avoid the "catch up"
    //overshoot
    double ei = commandedConfig[i] - sensedConfig[i];
    ei /= 10.0*Max(0.1,Abs(commandedVelocity[i]),Abs(sensedVelocity[i]));
    integralError[i] = Clamp(integralError[i]+ei*dt,-0.2,0.2);
  }

  //this is used to fade in from the un-calibrated
  if(sendCommand && firstSendTime < 0)
    firstSendTime = controller->t;

  if(calib && calib->Enabled() && !commandedConfig.empty() && (controlMode == LimbState::POSITION || controlMode == LimbState::RAW_POSITION)) {
    //TODO: MPC-based control
    vector<Vector> qdesired(1,commandedConfig);
    Config qsense_extrap = sensedConfig;
    if(senseUpdateTime >= 0)  {
      //cout<<"Sensed: "<<sensedConfig<<endl;
      //printf("Extrapolating velocity by %g seconds\n",controller->t-senseUpdateTime);
      qsense_extrap.madd(sensedVelocity,controller->t-senseUpdateTime);
    }
    else
      printf("Unable to extrapolate velocity\n");
    x.resize(commandedConfig.size());
    //compute gravity vector g
    Robot* robot = controller->robotModel;
    const vector<int>& indices = (limb==LEFT? controller->leftKlamptIndices : controller->rightKlamptIndices);
    Vector g((int)indices.size());
    {
      ScopedLock lock(controller->robotMutex);
      for(size_t i=0;i<indices.size();i++)
        robot->q[indices[i]] = commandedConfig[i];
      int ee_index = robot->LinkIndex((limb==LEFT?klampt_left_ee_name:klampt_right_ee_name));
      robot->UpdateSelectedFrames(ee_index);
      Vector G;
      robot->GetGravityTorques(Vector3(0,0,-9.8),G);
      for(size_t i=0;i<indices.size();i++)
        g[i] = G[indices[i]];
    }

    //compute calibrated command
    calib->ToCommand(qsense_extrap,sensedVelocity,g,qdesired,x);

    //set the raw command
    double settleTime = 2.0;
    double integralGain = 0.5;
    if(controller->t - firstSendTime < settleTime) {
      double fadein = (controller->t - firstSendTime) / settleTime;
      rawCommandToSend.madd(x-rawCommandToSend, fadein);
      rawCommandToSend.madd(integralError,integralGain*fadein);
    }
    else {
      rawCommandToSend = x;
      rawCommandToSend.madd(integralError,integralGain);
    }
    //Baxter will ignore the command if it's out of joint limits
    for(int i=0;i<rawCommandToSend.n;i++)
      rawCommandToSend[i] = Clamp(rawCommandToSend[i],robot->qMin(indices[i]),
				  robot->qMax(indices[i]));

    sendCommand = true;
  }
}

void ControllerUpdateData::MyAdvanceController()
{
  if(!robotState.baxterEnabled) return;
  ScopedLock lock(mutex);
  if(planner.plannerActive[0] || planner.plannerActive[1]) {
    if(planner.planningThread.SendUpdate(planner.motionQueueInterface)) {
      //copy the update to the limbs' motion queues
      if(planner.plannerActive[LEFT]) {
	robotState.leftLimb.motionQueue.pathOffset = planner.wholeRobotMotionQueue.pathOffset;
	for(size_t i=0;i<leftKlamptIndices.size();i++)
	  robotState.leftLimb.motionQueue.path.elements[i] = planner.wholeRobotMotionQueue.path.elements[leftKlamptIndices[i]];
      }
      if(planner.plannerActive[RIGHT]) {
	robotState.rightLimb.motionQueue.pathOffset = planner.wholeRobotMotionQueue.pathOffset;
	for(size_t i=0;i<rightKlamptIndices.size();i++)
	  robotState.rightLimb.motionQueue.path.elements[i] = planner.wholeRobotMotionQueue.path.elements[rightKlamptIndices[i]];
      }
    }
  }
  double dt = t-last_t;
  robotState.leftLimb.Advance(dt,this);
  robotState.rightLimb.Advance(dt,this);
}

void ControllerUpdateData::MyUpdateSystemStateService()
{
  if(!systemStateService.IsOpen()) {
    return;
  }
  if(t < lastUpdateSystemStateTime + 1.0/updateSystemStateRate)
    //printf("Waiting for update %g < %g\n",t,lastUpdateSystemStateTime + 1.0/updateSystemStateRate);
    return;
  lastUpdateSystemStateTime = t;

  //provide feedback to the state service about the time
  {
    AnyCollection msg;
    {
      ScopedLock lock(mutex);
      msg["t"] = t;
      msg["dt"] = t-last_t;
    }
    AnyCollection command;
    command["type"] = string("change");
    command["path"] = string(".robot");
    command["data"] = msg;
    bool res=SSPP::Send(systemStateService,command);
    Assert(res);
  }

  //provide feedback about the command
  {
    Vector q,dq;
    Vector larmq,larmv,rarmq,rarmv,baseq,basev;
    double hpan,hpanspeed;
    {
      ScopedLock lock(mutex);
      larmq = robotState.leftLimb.commandedConfig;
      larmv = robotState.leftLimb.commandedVelocity;
      rarmq = robotState.rightLimb.commandedConfig;
      rarmv = robotState.rightLimb.commandedVelocity;
      hpan = robotState.head.panTarget;
      hpanspeed = robotState.head.panSpeed;
      baseq.resize(3,0.0);
      if(robotState.base.enabled && robotState.base.controlMode != BaseState::NONE) {
	if(!robotState.base.GetOdometryTarget(baseq))
	  basev = robotState.base.command;
      }
      GetKlamptCommandedConfig(q);
      GetKlamptCommandedVelocity(dq);
    }
    AnyCollection commandData;
    if(!q.empty()) {
      commandData["q"] = vector<double>(q);
      commandData["dq"] = vector<double>(dq);
    }
    if(robotState.baxterEnabled) {
      commandData["left"] = AnyCollection();
      commandData["left"]["q"] = vector<double>(larmq);
      commandData["left"]["dq"] = vector<double>(larmv);
      commandData["right"] = AnyCollection();
      commandData["right"]["q"] = vector<double>(rarmq);
      commandData["right"]["dq"] = vector<double>(rarmv);
      commandData["head"] = AnyCollection();
      commandData["head"]["pan"] = hpan;
      commandData["head"]["panspeed"] = hpanspeed;
    }
    if(!baseq.empty() || !basev.empty()) {
      commandData["base"]["odometry"] = vector<double>(baseq);
      commandData["base"]["velocity"] = vector<double>(basev);
    }
    AnyCollection command;
    command["type"] = string("set");
    command["path"] = string(".robot.command");
    command["data"] = commandData;
    bool res=SSPP::Send(systemStateService,command);
    Assert(res);
  }

  {
    //gripper feedback
    ScopedLock lock(mutex);
    AnyCollection gripperData,leftGripperData,rightGripperData;
    leftGripperData["name"] = robotState.leftGripper.name;
    leftGripperData["enabled"] = (int)robotState.leftGripper.enabled;
    leftGripperData["position"] = vector<double>(robotState.leftGripper.position);
    leftGripperData["force"] = vector<double>(robotState.leftGripper.force);
    leftGripperData["positionCommand"] = vector<double>(robotState.leftGripper.positionCommand);
    leftGripperData["speedCommand"] = vector<double>(robotState.leftGripper.speedCommand);
    leftGripperData["forceCommand"] = vector<double>(robotState.leftGripper.forceCommand);
    rightGripperData["name"] = robotState.rightGripper.name;
    rightGripperData["enabled"] = (int)robotState.rightGripper.enabled;
    rightGripperData["position"] = vector<double>(robotState.rightGripper.position);
    rightGripperData["force"] = vector<double>(robotState.rightGripper.force);
    rightGripperData["positionCommand"] = vector<double>(robotState.rightGripper.positionCommand);
    rightGripperData["speedCommand"] = vector<double>(robotState.rightGripper.speedCommand);
    rightGripperData["forceCommand"] = vector<double>(robotState.rightGripper.forceCommand);
    gripperData["left"] = leftGripperData;
    gripperData["right"] = rightGripperData;
    AnyCollection command;
    command["type"] = string("set");
    command["path"] = string(".robot.gripper");
    command["data"] = gripperData;
    bool res=SSPP::Send(systemStateService,command);
    Assert(res);
  }

  //provide feedback about the sensed
  {
    Vector q,dq;
    Vector larmq,larmv,rarmq,rarmv,baseq,basev;
    double hpan;
    int hnod;
    {
      ScopedLock lock(mutex);
      larmq = robotState.leftLimb.sensedConfig;
      larmv = robotState.leftLimb.sensedVelocity;
      rarmq = robotState.rightLimb.sensedConfig;
      rarmv = robotState.rightLimb.sensedVelocity;
      hpan = robotState.head.pan;
      hnod = robotState.head.isNodding;
      baseq.resize(3,0.0);
      if(robotState.base.enabled) {
	baseq = robotState.base.odometry;
	basev = robotState.base.velocity;
      }
      GetKlamptSensedConfig(q);
      GetKlamptSensedVelocity(dq);
    }
    AnyCollection sensedData;
    if(!q.empty()) {
      sensedData["q"] = vector<double>(q);
      sensedData["dq"] = vector<double>(dq);
    }
    if(robotState.baxterEnabled) {
      sensedData["left"]["q"] = vector<double>(larmq);
      sensedData["left"]["dq"] = vector<double>(larmv);
      sensedData["right"]["q"] = vector<double>(rarmq);
      sensedData["right"]["dq"] = vector<double>(rarmv);
      sensedData["head"]["pan"] = hpan;
      sensedData["head"]["nodding"] = hnod;
    }
    if(!baseq.empty()) {
      sensedData["base"]["odometry"] = vector<double>(baseq);
      sensedData["base"]["velocity"] = vector<double>(basev);
    }
    AnyCollection command;
    command["type"] = string("set");
    command["path"] = string(".robot.sensed");
    command["data"] = sensedData;
    bool res=SSPP::Send(systemStateService,command);
    Assert(res);
  }

  if (robotState.leftLimb.motionQueueActive) {
    PolynomialMotionQueue* queue = &robotState.leftLimb.motionQueue;
    //provide feedback about the motion queue
    AnyCollection command;
    command["type"] = string("change");
    command["path"] = string(".controller.left");
    AnyCollection data;
    {
      ScopedLock lock(mutex);
      double tstart = queue->CurTime();
      double tend = queue->TimeRemaining()+tstart;
      data["traj_t_end"] = tend;
      //read out the path?
      Config q;
      Real dt = 0.05;
      int istart=(int)Ceil(tstart/dt);
      int iend=(int)Ceil(tend/dt);
      AnyCollection path;
      vector<double> times;
      for(int i=istart;i<iend;i++) {
        Real t=i*dt;
        times.push_back(t);
        queue->Eval(t,q,false);
        path[i-istart] = vector<double>(q);
      }
      q = queue->Endpoint();
      path[iend-istart] = vector<double>(q);
      times.push_back(tend);
      /*
      data["traj"]["milestones"] = path;
      data["traj"]["times"] = times;
      */
      data["traj_q_end"] = vector<double>(q);
    }
    command["data"] = data;
    bool res=SSPP::Send(systemStateService,command);
    Assert(res);
  }
  else {
    AnyCollection command;
    command["type"] = string("change");
    command["path"] = string(".controller.left");
    command["data"] = AnyCollection();
    bool res=SSPP::Send(systemStateService,command);
    Assert(res);
  }
  if (robotState.rightLimb.motionQueueActive) {
    PolynomialMotionQueue* queue = &robotState.rightLimb.motionQueue;
    //provide feedback about the motion queue
    AnyCollection command;
    command["type"] = string("change");
    command["path"] = string(".controller.right");
    AnyCollection data;
    {
      ScopedLock lock(mutex);
      double tstart = queue->CurTime();
      double tend = queue->TimeRemaining()+tstart;
      data["traj_t_end"] = tend;
      //read out the path?
      Config q;
      Real dt = 0.05;
      int istart=(int)Ceil(tstart/dt);
      int iend=(int)Ceil(tend/dt);
      AnyCollection path;
      vector<double> times;
      for(int i=istart;i<iend;i++) {
        Real t=i*dt;
        times.push_back(t);
        queue->Eval(t,q,false);
        path[i-istart] = vector<double>(q);
      }
      q = queue->Endpoint();
      path[iend-istart] = vector<double>(q);
      times.push_back(tend);
      /*
        data["traj"]["milestones"] = path;
        data["traj"]["times"] = times;
      */
      data["traj_q_end"] = vector<double>(q);
    }
    command["data"] = data;
    bool res=SSPP::Send(systemStateService,command);
    Assert(res);
  }
  else {
    AnyCollection command;
    command["type"] = string("change");
    command["path"] = string(".controller.right");
    command["data"] = AnyCollection();
    bool res=SSPP::Send(systemStateService,command);
    Assert(res);
  }


  //provide feedback about the end effectors
  if(robotModel != NULL) {
    ScopedLock lock(mutex);
    ScopedLock lock2(robotMutex);
    int ee[2] = {robotModel->LinkIndex(klampt_left_ee_name),robotModel->LinkIndex(klampt_right_ee_name)};
    AnyCollection data;
    data.resize(2);
    GetKlamptSensedConfig(robotModel->q);
    for(size_t i=0;i<baseKlamptIndices.size();i++)
      robotModel->q[baseKlamptIndices[i]]=0;
    robotModel->UpdateFrames();
    Vector3 ofs[2] = {robotState.leftLimb.endEffectorOffset,robotState.rightLimb.endEffectorOffset};
    for(int i=0;i<2;i++) {
      AnyCollection tcp;
      tcp.resize(3);
      tcp[0] = ofs[i].x;
      tcp[1] = ofs[i].y;
      tcp[2] = ofs[i].z;
      data[i]["toolCenterPoint"] = tcp;
    }
    for(int i=0;i<2;i++) {
      AnyCollection xform;
      vector<double> R(9);
      vector<double> t(3);
      robotModel->links[ee[i]].T_World.R.get(&R[0]);
      (robotModel->links[ee[i]].T_World*ofs[i]).get(&t[0]);
      xform.resize(2);
      xform[0] = R;
      xform[1] = t;
      data[i]["xform"]["sensed"] = xform;
    }
    GetKlamptCommandedConfig(robotModel->q);
    for(size_t i=0;i<baseKlamptIndices.size();i++)
      robotModel->q[baseKlamptIndices[i]]=0;
    robotModel->UpdateFrames();
    for(int i=0;i<2;i++) {
      AnyCollection xform;
      vector<double> R(9);
      vector<double> t(3);
      robotModel->links[ee[i]].T_World.R.get(&R[0]);
      (robotModel->links[ee[i]].T_World*ofs[i]).get(&t[0]);
      xform.resize(2);
      xform[0] = R;
      xform[1] = t;
      data[i]["xform"]["commanded"] = xform;
    }
    //trajectory destinations
    if(robotState.leftLimb.motionQueueActive) {
      PolynomialMotionQueue* queue = &robotState.leftLimb.motionQueue;
      Config q=queue->Endpoint();
      for(size_t i=0;i<leftKlamptIndices.size();i++)
        robotModel->q[leftKlamptIndices[i]] = q[i];
      robotModel->UpdateSelectedFrames(ee[0]);
      AnyCollection xform;
      vector<double> R(9);
      vector<double> t(3);
      robotModel->links[ee[0]].T_World.R.get(&R[0]);
      (robotModel->links[ee[0]].T_World*ofs[0]).get(&t[0]);
      xform.resize(2);
      xform[0] = R;
      xform[1] = t;
      data[0]["xform"]["destination"] = xform;
    }
    else
      data[0]["xform"]["destination"] = data[0]["xform"]["commanded"];
    if(robotState.rightLimb.motionQueueActive) {
      PolynomialMotionQueue* queue = &robotState.rightLimb.motionQueue;
      Config q=queue->Endpoint();
      for(size_t i=0;i<rightKlamptIndices.size();i++)
        robotModel->q[rightKlamptIndices[i]] = q[i];
      robotModel->UpdateSelectedFrames(ee[1]);
      AnyCollection xform;
      vector<double> R(9);
      vector<double> t(3);
      robotModel->links[ee[1]].T_World.R.get(&R[0]);
      (robotModel->links[ee[1]].T_World*ofs[1]).get(&t[0]);
      xform.resize(2);
      xform[0] = R;
      xform[1] = t;
      data[1]["xform"]["destination"] = xform;
    }
    else
      data[1]["xform"]["destination"] = data[1]["xform"]["commanded"];

    AnyCollection command;
    command["type"] = string("set");
    command["path"] = string(".robot.endEffectors");
    command["data"] = data;
    bool res=SSPP::Send(systemStateService,command);
    if(!res) {
      printf("Warning: could not send to system state service, state server may have crashed?\n");
      printf("Socket open: %d\n",(int)systemStateService.IsOpen());
    }
  }
}


bool BaseState::GetRelativeTarget(Vector& trel) const
{
  if(controlMode == RELATIVE_VELOCITY || controlMode == NONE)
    //relative velocity mode
    return 0;
  else if(controlMode == RELATIVE_POSITION) {
    trel = command;
  }
  else {
    //do transformation to local coordinates
    ToLocal(command,trel);
  }
  return 1;
}

bool BaseState::GetOdometryTarget(Vector& odo) const
{
  if(controlMode == RELATIVE_VELOCITY || controlMode == NONE)
    //relative velocity mode
    return 0;
  else if(controlMode == RELATIVE_POSITION) {
    //do transformation from local to odometry coordinates
    ToOdometry(command,odo);
  }
  else {
    odo = command;
  }
  return 1;
}


void BaseState::ToLocal(const Vector& odoconfig,Vector& lconfig) const
{
  double xo=odometry[0];
  double yo=odometry[1];
  double thetao=odometry[2];
  double xabs=odoconfig[0];
  double yabs=odoconfig[1];
  double thetaabs=odoconfig[2];
  Vector2 dpabs (xabs-xo,yabs-yo);
  lconfig.resize(3);
  lconfig[2] = AngleDiff(thetaabs,thetao);
  Matrix2 rrel; rrel.setRotate(-thetao);
  Vector2 dprel = rrel*dpabs;
  dprel.get(lconfig[0],lconfig[1]);
}

void BaseState::ToOdometry(const Vector& lconfig,Vector& odoconfig) const
{
  double xo=odometry[0];
  double yo=odometry[1];
  double thetao=odometry[2];
  double xrel=lconfig[0];
  double yrel=lconfig[1];
  double thetarel=lconfig[2];
  Vector2 dprel (xrel,yrel);
  Matrix2 R; R.setRotate(thetao);
  Vector2 dpabs = R*dprel;
  odoconfig.resize(3);
  odoconfig[0] = xo+dpabs.x;
  odoconfig[1] = yo+dpabs.y;
  odoconfig[2] = thetao+thetarel;
}

/** @brief Produce a smooth ramping velocity from a start state to a
 * goal configuration given velocity/acceleration bounds.
 *
 * Arguments:
 * - xerr: starting error (xcur-xdesired)
 * - vcur: starting velocity
 * - vmax: maximum velocity
 * - amax: maximum acceleration/deceleration
 * - dt: time step
 * Output:
 * - velocity command for the next time step.
 *
 * For more information about the ramping trajectory, and to have control
 * over the final velocity, use the classes in Klampt/Modeling/ParabolicRamp.h.
 */
Real RampVel(Real xerr,Real vcur,Real vmax,Real amax,Real dt)
{
  if(xerr < 0) return -RampVel(-xerr,-vcur,vmax,amax,dt);
  Real v = 0;
  if(vcur > 0) //going upwards, accelerate down
    v = vcur - amax*dt;
  else {
    //vcur is negative
    //determine whether to accelerate down, begin braking, or coast
    Real tstop = -vcur/amax;
    if(xerr + vcur*tstop + 0.5*amax*Sqr(tstop) < 0) //begin braking
      v = vcur + amax*dt;
    else { //accelerate
      v = vcur - amax*dt;
      if(v < -vmax) v = -vmax;  //coast
    }
  }
  //determine when to halt on next time step via a zero crossing
  if(Sign(xerr) != Sign(xerr + v*dt))
    v = Clamp(-xerr/dt,vcur-amax*dt,vcur+amax*dt);
  return v;
}

void BaseState::IntegrateCommand(const BaseCalibration& calib,Real dt)
{
  commandedVelocity.resize(3);
  if(commandedPosition.empty())
    commandedPosition = odometry;
  if(controlMode == NONE || command.empty()) commandedVelocity.set(0.0);
  else if (controlMode == RELATIVE_VELOCITY) {
    commandedVelocity(0) = Clamp(command(0),-calib.xVelMax,calib.xVelMax);
    commandedVelocity(1) = Clamp(command(1),-calib.yVelMax,calib.yVelMax);
    commandedVelocity(2) = Clamp(command(2),-calib.angVelMax,calib.angVelMax);
    Matrix2 R; R.setRotate(commandedPosition(2));
    Vector2 vworld = R*Vector2(commandedVelocity(0),commandedVelocity(1));
    commandedPosition(0) += vworld.x*dt;
    commandedPosition(1) += vworld.y*dt;
    commandedPosition(2) += commandedVelocity(2)*dt;
  }
  else {
    cout<<endl;
    cout<<"Current odometry "<<odometry<<", last command position "<<commandedPosition<<endl;
    Vector relativeTarget(3),lastCommandPositionRelativeToCurrent;
    ToLocal(commandedPosition,lastCommandPositionRelativeToCurrent);
    GetRelativeTarget(relativeTarget);
    if(relativeTarget(2) - lastCommandPositionRelativeToCurrent(2) > Pi)
      relativeTarget(2) -= TwoPi;
    if(relativeTarget(2) - lastCommandPositionRelativeToCurrent(2) < -Pi)
      relativeTarget(2) += TwoPi;
    cout<<"Last command position "<<lastCommandPositionRelativeToCurrent<<endl;
    cout<<"Relative target "<<relativeTarget<<endl;

    //TODO: use better optimal path limits.  Take coupling between DOFs into
    //account.  Look up literature on optimal paths for omnidirectional bases.
    ParabolicRamp::ParabolicRampND ramp;
    ramp.x0 = lastCommandPositionRelativeToCurrent;
    ramp.dx0.resize(3,0.0);
    if(!this->commandedVelocity.empty())
      ramp.dx0 = this->commandedVelocity;
    ramp.x1.resize(3);
    ramp.x1 = relativeTarget;
    ramp.dx1.resize(3,0.0);
    Vector amax(3),vmax(3);
    amax(0) = calib.xAccMax;
    amax(1) = calib.yAccMax;
    amax(2) = calib.angAccMax;
    vmax(0) = calib.xVelMax;
    vmax(1) = calib.yVelMax;
    vmax(2) = calib.angVelMax;
    /*
    cout<<"Dt "<<dt<<endl;
    cout<<"Odometry "<<this->odometry<<endl;
    cout<<"Relative target pose "<<relativeTarget<<endl;
    cout<<"Relative velocity "<<this->velocity<<endl;
    */
    bool res;
    res = ramp.SolveMinTime(amax,vmax);
    if(res) {
      ParabolicRamp::Vector xnext,vnext;
      if(ramp.endTime < dt) {
	xnext = ramp.x1;
	vnext = ramp.dx1;
      }
      else {
	ramp.Evaluate(dt,xnext);
	ramp.Derivative(dt,vnext);
      }
      cout<<"Trying to get to "<<relativeTarget<<" from "<<ramp.x0<<", start vel "<<ramp.dx0<<endl;
      cout<<"  Result vel "<<vnext<<", next pos "<<xnext<<", duration "<<ramp.endTime<<endl;
      //getchar();
      /*
      cout<<"One-step movement "<<xnext<<endl;
      cout<<"Stop time: "<<ramp.endTime<<endl;
      */
      ToOdometry(xnext,commandedPosition);

      cout<<"Desired next position (relative) "<<xnext<<endl;
      cout<<"                      (global) "<<commandedPosition<<endl;
      cout<<endl;
      commandedVelocity = vnext;
    }
    else {
      fprintf(stderr,"Base: Unable to solve for ramp. Sending zero twist?\n");
      commandedVelocity.set(0.0);
    }
    /*
    commandedVelocity(0) = RampVel(-relativeDesired(0),this->velocity(0),calib.xVelMax,calib.xAccMax,dt);
    commandedVelocity(1) = RampVel(-relativeDesired(1),this->velocity(1),calib.yVelMax,calib.yAccMax,dt);
    commandedVelocity(2) = RampVel(-relativeDesired(2),this->velocity(2),calib.angVelMax,calib.angAccMax,dt);
    */
    //cout<<"Ramp twist: "<<commandedVelocity<<endl;
  }
}

void BaseState::DesiredToLowLevelCommand(const BaseCalibration& calib,Real dt,Vector& twist)
{
  if(commandedPosition.empty()) {
    twist = commandedVelocity;
    return;
  }
  //have both a position and a velocity correction term
  //TODO: tweak this
  Real kp = 0.1;
  Real kv = 1.0;
  Vector plocal;
  ToLocal(commandedPosition,plocal);
  cout<<"CommandedPosition: "<<commandedPosition<<endl;
  cout<<"   local: "<<plocal<<endl;
  if(estimatedVelocity.empty()) estimatedVelocity = commandedVelocity;
  plocal -= (estimatedVelocity*calib.timeDelay);
  cout<<"   local extrapolated: "<<plocal<<endl;
  //apply deadband
  for(int i=0;i<3;i++)
    if(Abs(plocal[i]) < 5e-3) plocal[i] = 0;
  Vector pvel = plocal/dt;
  cout<<"Desired position velocity "<<pvel<<", desired velocity "<<commandedVelocity<<endl;
  twist = kp*pvel + kv*commandedVelocity;
  //twist = commandedVelocity;
  estimatedVelocity = twist;
  if(estimatedVelocity.maxAbsElement() > 10) {
    printf("Whoa, estimated velocity is super huge! press enter to continue\n");
    getchar();
  }

  twist[0] = Clamp(twist[0],-calib.xVelMax,calib.xVelMax);
  twist[1] = Clamp(twist[1],-calib.yVelMax,calib.yVelMax);
  twist[2] = Clamp(twist[2],-calib.angVelMax,calib.angVelMax);

  //friction compensation
  if(Abs(twist[0]) > 1e-3)
    twist[0] += Sign(twist[0])*calib.xFriction;
  if(Abs(twist[1]) > 1e-3)
    twist[1] += Sign(twist[1])*calib.yFriction;
  if(Abs(twist[2]) > 1e-3)
    twist[2] += Sign(twist[2])*calib.angFriction;
}

void ControllerUpdateData::SetLimbGovernor(int limb,int governor)
{
  if((limb == LEFT && robotState.leftLimb.governor == governor) ||
     (limb == RIGHT && robotState.rightLimb.governor == governor))
    return;
  if(limb == BOTH && robotState.leftLimb.governor == governor && robotState.rightLimb.governor == governor)
    return;

  //activate motion queue, if not already
  if(limb == LEFT || limb == BOTH) {
    //first, determine whether to activate or deactivate motion queue
    if(governor == LimbState::MotionQueue || governor == LimbState::Planner) {
      if(!robotState.leftLimb.motionQueueActive) {
	robotState.leftLimb.motionQueue.SetConstant(robotState.leftLimb.commandedConfig);
	if(robotModel) {
	  KlamptToLimb(robotModel->qMin,LEFT,robotState.leftLimb.motionQueue.qMin);
	  KlamptToLimb(robotModel->qMax,LEFT,robotState.leftLimb.motionQueue.qMax);
	  KlamptToLimb(robotModel->velMax,LEFT,robotState.leftLimb.motionQueue.velMax);
	  KlamptToLimb(robotModel->accMax,LEFT,robotState.leftLimb.motionQueue.accMax);
	}
      }
      robotState.leftLimb.motionQueueActive = true;
    }
    else
      robotState.leftLimb.motionQueueActive = false;

    if(governor == LimbState::EndEffectorDrive) {
      ScopedLock lock(robotMutex);
      //begin drive mode
      double R[9];
      Vector3 t;
      GetKlamptCommandedConfig(robotModel->q);
      //ignore base
      for(size_t i=0;i<baseKlamptIndices.size();i++)
	robotModel->q[baseKlamptIndices[i]] = 0;
      int index=robotModel->LinkIndex(klampt_left_ee_name);
      robotModel->UpdateSelectedFrames(index);
      RigidTransform Tcur;
      Tcur.R = robotModel->links[index].T_World.R;
      Tcur.t = robotModel->links[index].T_World*robotState.leftLimb.endEffectorOffset;
      robotState.leftLimb.driveTransform = Tcur;
      robotState.leftLimb.achievedTransform = Tcur;
    }

    robotState.leftLimb.governor = governor;
  }
  if(limb == RIGHT || limb == BOTH) {
    //first, determine whether to activate or deactivate motion queue
    if(governor == LimbState::MotionQueue || governor == LimbState::Planner) {
      if(!robotState.rightLimb.motionQueueActive) {
	robotState.rightLimb.motionQueue.SetConstant(robotState.rightLimb.commandedConfig);
	if(robotModel) {
	  KlamptToLimb(robotModel->qMin,RIGHT,robotState.rightLimb.motionQueue.qMin);
	  KlamptToLimb(robotModel->qMax,RIGHT,robotState.rightLimb.motionQueue.qMax);
	  KlamptToLimb(robotModel->velMax,RIGHT,robotState.rightLimb.motionQueue.velMax);
	  KlamptToLimb(robotModel->accMax,RIGHT,robotState.rightLimb.motionQueue.accMax);
	}
      }
      robotState.rightLimb.motionQueueActive = true;
    }
    else {
      robotState.rightLimb.motionQueueActive = false;
    }

    if(governor == LimbState::EndEffectorDrive) {
      //begin drive mode
      ScopedLock lock(robotMutex);
      double R[9];
      Vector3 t;
      GetKlamptCommandedConfig(robotModel->q);
      //ignore base
      for(size_t i=0;i<baseKlamptIndices.size();i++)
	robotModel->q[baseKlamptIndices[i]] = 0;
      int index=robotModel->LinkIndex(klampt_right_ee_name);
      robotModel->UpdateSelectedFrames(index);
      RigidTransform Tcur;
      Tcur.R = robotModel->links[index].T_World.R;
      Tcur.t = robotModel->links[index].T_World*robotState.rightLimb.endEffectorOffset;
      robotState.rightLimb.driveTransform = Tcur;
      robotState.rightLimb.achievedTransform = Tcur;
    }

    robotState.rightLimb.governor = governor;
  }

  if(governor == LimbState::Planner) {
    if(!planner.plannerActive[0]) {
      planner.plannerActive[0] = true;
      planner.plannerActive[1] = true;
      Vector qstart;
      GetKlamptCommandedConfig(qstart);
      planner.planningThread.SetStartConfig(qstart);
      planner.planningThread.SetPlanner(new RealTimePlanner);
      planner.planningThread.SetCSpace(planner.cspace);
      planner.planningThread.Start();
    }
  }
  else {
    if(planner.plannerActive[0] && planner.plannerActive[1]) {
      planner.plannerActive[0] = false;
      planner.plannerActive[1] = false;
      planner.planningThread.Stop();
    }
  }
}

bool ControllerUpdateData::SolveIK(int limb,const RigidTransform& T,const Vector& qmin,const Vector& qmax,Real rotationTolerance,Real positionTolerance,
				   Vector& limbJointPositions,bool doLock)
{
  Assert(rotationTolerance > 0 && positionTolerance > 0);
  int index;
  const vector<int>* limbIndices = NULL;
  Vector3 offset;
  if(limb == LEFT) {
    index=robotModel->LinkIndex(klampt_left_ee_name);
    limbIndices = &leftKlamptIndices;
    offset = robotState.leftLimb.endEffectorOffset;
  }
  else if(limb == RIGHT) {
    index=robotModel->LinkIndex(klampt_right_ee_name);
    limbIndices = &rightKlamptIndices;
    offset = robotState.rightLimb.endEffectorOffset;
  }
  else
    return false;
  if(index < 0) return false;
  Vector dlimb;
  Matrix J,Jlimb;

  IKGoal goal;
  goal.link = index;
  goal.SetFixedRotation(T.R);
  if(goal.rotConstraint != IKGoal::RotFixed) {
    fprintf(stderr,"SolveIK: failed because input rotation matrix was not a proper rotation matrix?\n");
    return false;
  }
  goal.localPosition = offset;
  goal.SetFixedPosition(T.t);

  if(doLock) mutex.lock();
  robotMutex.lock();
  //GetKlamptCommandedConfig(robotModel->q);
  GetKlamptTargetConfig(robotModel->q);
  if(!ikBiasConfig.empty()) {
    //IK biasing enabled: move target toward it by some small amount
    double timestep = 0.05;
    Vector ikBiasDir = ikBiasConfig - robotModel->q;
    for(int i=0;i<limbIndices->size();i++) {
      int k=(*limbIndices)[i];
      if(robotModel->q[k] + ikBiasDir[k]*timestep < qmin[k])
        timestep = (qmin[k]-robotModel->q[k])/ikBiasDir[k];
      else if(robotModel->q[k] + ikBiasConfig[k]*timestep > qmax[k])
        timestep = (qmax[k]-robotModel->q[k])/ikBiasDir[k];
    }
    for(int i=0;i<limbIndices->size();i++) {
      int k=(*limbIndices)[i];
      robotModel->q[k] += ikBiasDir[k]*timestep;
    }
  }
  for(size_t i=0;i<baseKlamptIndices.size();i++)
    robotModel->q[baseKlamptIndices[i]] = 0;
  //GetKlamptCommandedVelocity(robotModel->dq);
  robotModel->UpdateSelectedFrames(index);

  RobotIKFunction function(*robotModel);
  //function.UseIK(goal);
  IKGoalFunction* goalfunc = new IKGoalFunction(*robotModel,goal,function.activeDofs);
  if(IsFinite(positionTolerance) && IsFinite(rotationTolerance)) {
    goalfunc->rotationScale = positionTolerance/(positionTolerance+rotationTolerance);
    goalfunc->positionScale = rotationTolerance/(positionTolerance+rotationTolerance);
  }
  else if(!IsFinite(positionTolerance) && !IsFinite(rotationTolerance)) {
    //keep equal tolerance?  or weight position more heavily?
    goalfunc->rotationScale = 0.1/Pi;
  }
  else {
    goalfunc->rotationScale = Min(positionTolerance,rotationTolerance)/rotationTolerance;
    goalfunc->positionScale = Min(positionTolerance,rotationTolerance)/positionTolerance;
  }
  //printf("Position scale %g, rotation scale %g\n",goalfunc->positionScale,goalfunc->rotationScale);
  function.functions.push_back(goalfunc);
  function.activeDofs.mapping = *limbIndices;

  Vector x0(limbIndices->size()),err0(6);
  function.GetState(x0);
  function(x0,err0);
  Real quality0 = err0.normSquared();

  Real tolerance = Min(1e-7,Min(positionTolerance,rotationTolerance)/Sqrt(3.0));
  int iters = 100;
  int verbose = 0;
  RobotIKSolver solver(function);
  //use greater dampening than the default 0.01
  solver.solver.lambda = 1.0;
  if(qmin.empty())
    solver.UseJointLimits(TwoPi);
  else
    solver.UseJointLimits(qmin,qmax);
  solver.solver.verbose = verbose;
  bool res = solver.Solve(tolerance,iters);
  if(!qmin.empty()) {
    //check joint limits
    for(size_t i=0;i<limbIndices->size();i++) {
      int k=(*limbIndices)[i];
      if(robotModel->q[k] < qmin[k] || robotModel->q[k] > qmax[k]) {
        printf("Warning, result from IK solve is out of bounds: index %d, %g <= %g <= %g\n",k,qmin[k],robotModel->q[k],qmax[k]);
        robotModel->q[k] = Clamp(robotModel->q[k],qmin[k],qmax[k]);
        robotModel->UpdateFrames();
      }
    }
  }
  //bool res = ::SolveIK(function,tolerance,iters,verbose);
  KlamptToLimb(robotModel->q,limb,limbJointPositions);

  //now evaluate quality of the solve
  function.GetState(x0);
  function(x0,err0);
  Real qualityAfter = err0.normSquared();
  if(!(qualityAfter <= quality0)) {
    printf("Solve failed: original configuration was better\n");
    res = false;
  }
  else {
    //test
    Vector3 perr,rerr;
    goal.GetError(robotModel->links[index].T_World,perr,rerr);
    if(perr.norm() < positionTolerance && rerr.norm() < rotationTolerance)
      res = true;
    else {
      res = false;
      printf("Position error: %g, rotation error: %g not under tolerances %g, %g\n",perr.norm(),rerr.norm(),positionTolerance,rotationTolerance);
      printf("Solve tolerance %g, result %d\n",tolerance,(int)res);
    }
  }

  if(doLock) mutex.unlock();
  robotMutex.unlock();
  return res;
}

bool ControllerUpdateData::SolveIK(int limb,const Vector3& pos,const Vector& qmin,const Vector& qmax,Real positionTolerance,
				   Vector& limbJointPositions,bool doLock)
{
  int index;
  const vector<int>* limbIndices = NULL;
  Vector3 offset;
  if(limb == LEFT) {
    index=robotModel->LinkIndex(klampt_left_ee_name);
    limbIndices = &leftKlamptIndices;
    offset = robotState.leftLimb.endEffectorOffset;
  }
  else if(limb == RIGHT) {
    index=robotModel->LinkIndex(klampt_right_ee_name);
    limbIndices = &rightKlamptIndices;
    offset = robotState.rightLimb.endEffectorOffset;
  }
  else
    return false;
  if(index < 0) return false;
  Vector dlimb;
  Matrix J,Jlimb;

  if(doLock) mutex.lock();
  robotMutex.lock();
  //GetKlamptCommandedConfig(robotModel->q);
  GetKlamptTargetConfig(robotModel->q);
  if(!ikBiasConfig.empty()) {
    //IK biasing enabled: move target toward it by some small amount
    double timestep = 0.05;
    for(int i=0;i<limbIndices->size();i++) {
      int k=(*limbIndices)[i];
      if(robotModel->q[k] + ikBiasConfig[k]*timestep < qmin[k])
        timestep = (qmin[k]-robotModel->q[k])/ikBiasConfig[k];
      else if(robotModel->q[k] + ikBiasConfig[k]*timestep > qmax[k])
        timestep = (qmax[k]-robotModel->q[k])/ikBiasConfig[k];
    }
    for(int i=0;i<limbIndices->size();i++) {
      int k=(*limbIndices)[i];
      robotModel->q[k] += ikBiasConfig[k]*timestep;
    }
  }
  for(size_t i=0;i<baseKlamptIndices.size();i++)
    robotModel->q[baseKlamptIndices[i]] = 0;
  //GetKlamptCommandedVelocity(robotModel->dq);
  robotModel->UpdateSelectedFrames(index);
  IKGoal goal;
  goal.link = index;
  goal.SetFreeRotation();
  goal.localPosition = offset;
  goal.SetFixedPosition(pos);

  //evaluate quality of start config
  Vector3 perr,rerr;
  goal.GetError(robotModel->links[index].T_World,perr,rerr);
  Real quality0 = perr.normSquared();

  RobotIKFunction function(*robotModel);
  function.UseIK(goal);
  function.activeDofs.mapping = *limbIndices;
  Real tolerance = Min(1e-7,positionTolerance/Sqrt(3.0));
  int iters = 100;
  int verbose = 0;
  RobotIKSolver solver(function);
  //use greater dampening than the default 0.01
  solver.solver.lambda = 1.0;
  if(qmin.empty())
    solver.UseJointLimits(TwoPi);
  else
    solver.UseJointLimits(qmin,qmax);
  solver.solver.verbose = verbose;
  bool res = solver.Solve(tolerance,iters);
  //bool res = ::SolveIK(function,tolerance,iters,verbose);
  KlamptToLimb(robotModel->q,limb,limbJointPositions);

  //now evaluate quality of the solve
  goal.GetError(robotModel->links[index].T_World,perr,rerr);
  Real qualityAfter = perr.normSquared();
  if(!(qualityAfter <= quality0)) {
    printf("Solve failed: original configuration was better\n");
    res = false;
  }
  else {
    if(perr.norm() < positionTolerance)
      res = true;
    else {
      res = false;
      printf("Position error: %g not under tolerance %g\n",perr.norm(),positionTolerance);
    }
  }

  if(doLock) mutex.unlock();
  robotMutex.unlock();
  return res;
}


bool ControllerUpdateData::SolveIKVelocity(int limb,const Vector3& angVel,const Vector3& vel,Vector& limbJointVels,bool doLock)
{
  int index;
  const vector<int>* limbIndices = NULL;
  Vector3 offset;
  if(limb == LEFT) {
    index=robotModel->LinkIndex(klampt_left_ee_name);
    limbIndices = &leftKlamptIndices;
    offset = robotState.leftLimb.endEffectorOffset;
  }
  else if(limb == RIGHT) {
    index=robotModel->LinkIndex(klampt_right_ee_name);
    limbIndices = &rightKlamptIndices;
    offset = robotState.rightLimb.endEffectorOffset;
  }
  else
    return false;
  if(index < 0) return false;
  Vector dlimb;
  Matrix J,Jlimb;

  if(doLock) mutex.lock();
  robotMutex.lock();
  GetKlamptCommandedConfig(robotModel->q);
  GetKlamptCommandedVelocity(robotModel->dq);
  for(size_t i=0;i<baseKlamptIndices.size();i++) {
    robotModel->q[baseKlamptIndices[i]]=0;
    robotModel->dq[baseKlamptIndices[i]]=0;
  }
  robotModel->UpdateSelectedFrames(index);
  robotModel->GetFullJacobian(offset,index,J);
  robotMutex.unlock();
  if(doLock) mutex.unlock();

  Jlimb.resize(6,limbIndices->size());
  for(int i=0;i<limbIndices->size();i++)
    Jlimb.copyCol(i,J.col((*limbIndices)[i]));
  Vector twist(6);
  twist[0] = angVel[0];
  twist[1] = angVel[1];
  twist[2] = angVel[2];
  twist[3] = vel[0];
  twist[4] = vel[1];
  twist[5] = vel[2];
  RobustSVD<Real> svd;
  if(!svd.set(Jlimb)) {
    printf("sendEndEffectorVelocity: failed to solve for Jacobian pseudoinverse\n");
    return false;
  }
  svd.dampedBackSub(twist,1e-3,limbJointVels);
  //limit to joint velocities
  Real scale = 1.0;
  for(int i=0;i<limbJointVels.n;i++) {
    int k=(*limbIndices)[i];
    if(limbJointVels[i]*scale < robotModel->velMin[k])
      scale = robotModel->velMin[k]/limbJointVels[i];
    if(limbJointVels[i]*scale > robotModel->velMax[k])
      scale = robotModel->velMax[k]/limbJointVels[i];
  }
  limbJointVels *= scale;
  return true;
}

bool ControllerUpdateData::SolveIKVelocity(int limb,const Vector3& vel,Vector& limbJointVels,bool doLock)
{
  int index;
  const vector<int>* limbIndices = NULL;
  Vector3 offset;
  if(limb == LEFT) {
    index=robotModel->LinkIndex(klampt_left_ee_name);
    limbIndices = &leftKlamptIndices;
    offset = robotState.leftLimb.endEffectorOffset;
  }
  else if(limb == RIGHT) {
    index=robotModel->LinkIndex(klampt_right_ee_name);
    limbIndices = &rightKlamptIndices;
    offset = robotState.rightLimb.endEffectorOffset;
  }
  else
    return false;
  if(index < 0) return false;
  Vector dlimb;
  Matrix J,Jlimb;

  if(doLock) mutex.lock();
  robotMutex.lock();
  GetKlamptCommandedConfig(robotModel->q);
  GetKlamptCommandedVelocity(robotModel->dq);
  for(size_t i=0;i<baseKlamptIndices.size();i++) {
    robotModel->q[baseKlamptIndices[i]]=0;
    robotModel->dq[baseKlamptIndices[i]]=0;
  }
  robotModel->UpdateSelectedFrames(index);
  robotModel->GetPositionJacobian(offset,index,J);
  if(doLock) mutex.unlock();
  robotMutex.unlock();

  Jlimb.resize(3,limbIndices->size());
  for(int i=0;i<limbIndices->size();i++)
    Jlimb.copyCol(i,J.col((*limbIndices)[i]));
  Vector twist(3);
  twist[0] = vel[0];
  twist[1] = vel[1];
  twist[2] = vel[2];
  RobustSVD<Real> svd;
  if(!svd.set(Jlimb)) {
    printf("sendEndEffectorVelocity: failed to solve for Jacobian pseudoinverse\n");
    return false;
  }
  svd.dampedBackSub(twist,1e-3,limbJointVels);
  //limit to joint velocities
  Real scale = 1.0;
  for(int i=0;i<limbJointVels.n;i++) {
    int k=(*limbIndices)[i];
    if(limbJointVels[i]*scale < robotModel->velMin[k])
      scale = robotModel->velMin[k]/limbJointVels[i];
    if(limbJointVels[i]*scale > robotModel->velMax[k])
      scale = robotModel->velMax[k]/limbJointVels[i];
  }
  limbJointVels *= scale;
  return true;
}

void ControllerUpdateData::OnWorldChange()
{
  if(planner.world.robots.empty()) {
    if(robotModel) {
      planner.world.robots.resize(1);
      planner.world.robotViews.resize(1);
      planner.world.robots[0] = robotModel;
      planner.world.robotViews[0].robot = robotModel;
      planner.world.robotViews[0].SetGrey();
    }
  }
  else {
    if(robotModel) {
      printf("Warning, world file has a robot in it... replacing robot 0 with previously loaded model\n");
      planner.world.robots[0] = robotModel;
      planner.world.robotViews[0].robot = robotModel;
      planner.world.robotViews[0].SetGrey();
    }
    else {
      printf("Warning, world file has a robot in it... is it the same as the Klamp't model?\n");
      robotModel = planner.world.robots[0];
    }
  }
  planner.settings.InitializeDefault(planner.world);
  planner.cspace = new SingleRobotCSpace(planner.world,0,&planner.settings);
  planner.planningThread.SetCSpace(planner.cspace);
}


bool ControllerUpdateData::CheckCollisions(const Config& qklampt)
{
  if(!robotModel) {
    fprintf(stderr,"CheckCollisions: no robot model set\n");
    return false;
  }
  if(planner.world.robots.size()==0) {
    fprintf(stderr,"CheckCollisions: planner doesn't have any robots in world\n");
    return false;
  }
  //update robot config
  ScopedLock lock(robotMutex);
  planner.world.robots[0]->UpdateConfig(qklampt);
  planner.world.robots[0]->UpdateGeometry();
  //check collision
  if(planner.settings.CheckCollision(planner.world,planner.world.RobotID(0),planner.world.RobotID(0))) return true;
  return false;
}

bool ControllerUpdateData::CheckCollisions(int limb,const Config& qlimb)
{
  printf("Checking collisions with limb %d\n",limb);
  if(!robotModel) {
    fprintf(stderr,"CheckCollisions: no robot model set\n");
    return false;
  }
  if(planner.world.robots.size()==0) {
    fprintf(stderr,"CheckCollisions: planner doesn't have any robots in world\n");
    return false;
  }
  vector<bool> onLimb;
  vector<int> indices,worldids;
  if(limb == LEFT) {
    indices = leftKlamptIndices;
    robotModel->GetDescendants(leftKlamptIndices[0],onLimb);
    for(size_t i=0;i<onLimb.size();i++)
      if(onLimb[i]) worldids.push_back(planner.world.RobotLinkID(0,i));
  }
  else if(limb == RIGHT) {
    indices = rightKlamptIndices;
    robotModel->GetDescendants(rightKlamptIndices[0],onLimb);
    for(size_t i=0;i<onLimb.size();i++)
      if(onLimb[i]) worldids.push_back(planner.world.RobotLinkID(0,i));
  }
  else {
    indices = leftKlamptIndices;
    indices.insert(indices.end(),rightKlamptIndices.begin(),rightKlamptIndices.end());
    robotModel->GetDescendants(leftKlamptIndices[0],onLimb);
    for(size_t i=0;i<onLimb.size();i++)
      if(onLimb[i]) worldids.push_back(planner.world.RobotLinkID(0,i));
    robotModel->GetDescendants(rightKlamptIndices[0],onLimb);
    for(size_t i=0;i<onLimb.size();i++)
      if(onLimb[i]) worldids.push_back(planner.world.RobotLinkID(0,i));
  }
  set<int> iset;
  vector<int> otherids;
  for(size_t i=0;i<worldids.size();i++) iset.insert(worldids[i]);
  iset.insert(planner.world.RobotID(0));
  int numids = planner.world.NumIDs();
  for(int i=0;i<numids;i++)
    if(iset.count(i)==0) otherids.push_back(i);

  //update robot config
  ScopedLock lock(robotMutex);
  GetKlamptCommandedConfig(robotModel->q);
  for(size_t i=0;i<indices.size();i++)
    robotModel->q[indices[i]] = qlimb[i];
  robotModel->UpdateFrames();
  robotModel->UpdateGeometry();
  //check limb self collision
  Timer timer;
  if(planner.settings.CheckCollision(planner.world,worldids).first >= 0) {
    printf("Found limb self collision in time %g\n",timer.ElapsedTime());
    return true;
  }
  //check collision with the limb vs robot, and limb vs environment
  if(planner.settings.CheckCollision(planner.world,worldids,otherids).first >= 0) {
    printf("Found limb robot or environment collision in time %g\n",timer.ElapsedTime());
    return true;
  }
  printf("Verified no collision in time %g\n",timer.ElapsedTime());
  return false;
}



BaseCalibration::BaseCalibration()
{
  //this should be about 0.2 in the physical robot?
  timeDelay = 0.0;
  /*
    //full speed?
  xAccMax = 2.0;
  yAccMax = 4.0;
  xVelMax = 0.5;
  yVelMax = 1.0;
  angAccMax = 4.0;
  angVelMax = 0.6;
  */
  /*
  //very slow speed
  xAccMax = 0.05;
  yAccMax = 0.1;
  xVelMax = 0.05;
  yVelMax = 0.1;
  angAccMax = 0.1;
  angVelMax = 0.05;
  */
  //half speed , half accel
  xAccMax = 1.0;
  yAccMax = 2.0;
  xVelMax = 0.25;
  yVelMax = 0.5;
  angAccMax = 2.0;
  angVelMax = 0.3;
  //these values are calibrated via base_calibrate.py
  xFriction = 0.04;
  yFriction = 0.01;
  angFriction = 0.02;
}

BaxterMotorCalibration::BaxterMotorCalibration()
: lookahead(1), rate(50)
{}

bool BaxterMotorCalibration::Enabled() const
{
  return !Brows.empty();
}

void BaxterMotorCalibration::Clear()
{
  Brows.clear();
  Crows.clear();
  d.clear();
}

bool BaxterMotorCalibration::Load(const char* fn,const char* limb)
{
  name = limb;
  AnyCollection c;
  ifstream in(fn,ios::in);
  if(!in) {
    fprintf(stderr,"Unable to open calibration file %s\n",fn);
    return false;
  }
  in >> c;
  if(!in) {
    fprintf(stderr,"Unable to parse calibration from %s\n",fn);
    return false;
  }
  if(c.find(limb) == NULL) {
    fprintf(stderr,"Unable to load calibration for limb %s from %s\n",limb,fn);
    return false;
  }
  AnyCollection climb = c[limb];
  AnyCollection B = climb["B"];
  AnyCollection C = climb["C"];
  AnyCollection d = climb["d"];
  Assert(B.size()==7);
  Assert(C.size()==7);
  Assert(d.size()==7);
  Brows.resize(B.size());
  Assert(B.isarray());
  Assert(C.isarray());
  Assert(d.isarray());
  Crows.resize(C.size());
  this->d.resize(d.size());
  for(int i=0;i<7;i++) {
    Assert(B[i].ismap());
    Assert(C[i].ismap());
    Brows[i].clear();
    Crows[i].clear();
    vector<int> keys;
    vector<Real> values;
    bool res=B[i]["columns"].asvector<int>(keys);
    if(!res) {
      fprintf(stderr,"Invalid key in B matrix %s\n",fn);
      return false;
    }
    res = B[i]["values"].asvector<Real>(values);
    if(!res) {
      fprintf(stderr,"Invalid value in B matrix %s\n",fn);
      return false;
    }
    Assert(keys.size()==values.size());
    //printf("Loaded %d elements from B matrix entry %d\n",keys.size(),i);
    for(size_t j=0;j<keys.size();j++) {
      Brows[i][keys[j]] = values[j];
    }
    keys.resize(0);
    values.resize(0);
    res=C[i]["columns"].asvector<int>(keys);
    if(!res) {
      fprintf(stderr,"Invalid key in C matrix %s\n",fn);
      return false;
    }
    res = C[i]["values"].asvector<Real>(values);
    if(!res) {
      fprintf(stderr,"Invalid value in C matrix %s\n",fn);
      return false;
    }
    Assert(keys.size()==values.size());
    //printf("Loaded %d elements from C matrix entry %d\n",keys.size(),i);
    for(size_t j=0;j<keys.size();j++) {
      Crows[i][keys[j]] = values[j];
    }
    this->d[i] = Real(d[i]);
  }
  return true;
}

void BaxterMotorCalibration::ToCommand(const Vector& q,const Vector& v,const Vector& g,const vector<Vector>& qdesired,Vector& qcmd)
{
  Assert(qdesired.size() > 0);
  if(!Enabled()) {
    printf("Calibration not enabled\n");
    qcmd = qdesired[0];
    return;
  }
  qcmd.resize(q.n);
  qcmd.set(0.0);
  for(size_t i=0;i<Brows.size();i++) {
    if(Brows[i].numEntries()==0) {
      qcmd[i] = qdesired[0][i];
    }
    for(SparseVector::iterator e=Brows[i].begin();e!=Brows[i].end();e++) {
      int index = e->first;
      int future = index / q.size();
      int joint = index % q.size();
      if(future >= (int)qdesired.size())
	future = (int)qdesired.size()-1;
      qcmd[i] += qdesired[future][joint]*e->second;
    }
    for(SparseVector::iterator e=Crows[i].begin();e!=Crows[i].end();e++) {
      int index = e->first;
      int item = index / q.size();
      int joint = index % q.size();
      if(item == 0)
	qcmd[i] += q[joint]*e->second;
      else if(item == 1)
	qcmd[i] += v[joint]*e->second;
      else if(item == 2) {
	if(!g.empty())
	  qcmd[i] += g[joint]*e->second;
      }
      else {
	printf("Warning: calibration refers to an invalid state feature\n");
      }
    }
  }
  qcmd += d;
  /*
  cout<<name<<endl;
  cout<<"Q: "<<q<<endl;
  cout<<"V: "<<v<<endl;
  cout<<"G: "<<g<<endl;
  cout<<"Desired: "<<qdesired[0]<<endl;
  cout<<"Output: "<<qcmd<<endl;
  */
}
