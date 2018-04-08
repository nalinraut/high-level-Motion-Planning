#include <signal.h>

inline int NumLimbDofs(int limb) { if(limb == BOTH) return numLimbDofs*2; else return numLimbDofs; }

void* controller_thread_func(void* vdata)
{
  ControllerUpdateData* data = static_cast<ControllerUpdateData*>(vdata);
  if(!data->MyStartup()) {
    data->failure = true;
    return NULL;
  }
  if(!data->systemStateAddr.empty()) {
    printf("Motion Module: Connecting to system state server...\n");
    if(!data->systemStateService.Open(data->systemStateAddr.c_str(),FILECLIENT)) {
      FatalError("Motion Module: Could not open socket to system state service %s",data->systemStateAddr.c_str());
    }
  }
  {
    ScopedLock lock(data->mutex);
    data->t=data->last_t=0;
    data->lastUpdateSystemStateTime=0;
    data->startup = false;
    data->running = true;
  }

  //RUNNING.....!
  // printf("Ebolabot Motion: Running low-level control loop\n");
  Timer mytimer;
  while(!data->kill && !data->failure) {
    data->last_t = data->t;
    data->t = mytimer.ElapsedTime();
    //process sensors
    if(!data->MyProcessSensors()) {
      data->failure = true;
      stopMotion();
      break;
    }

    //update motion queue control loop, if using
    data->MyAdvanceController();

    //process new commands
    data->MySendCommands();

    //update the system state service, if using
    data->MyUpdateSystemStateService();

    //how much should we sleep?
    double dt = mytimer.ElapsedTime() - data->t;

    if(dt < 0.005)
      ThreadSleep(0.005);
    else
      ThreadYield();
  }

  if(!data->systemStateAddr.empty()) 
    data->systemStateService.Close();
  data->MyShutdown();
  data->running = false;
  return NULL;
}



//global data 
MyControllerUpdateData gData;
Thread gControllerUpdateThread;

bool loadCalibration(const char* fn)
{
  if(fn == NULL) {
    gData.leftCalibration.Clear();
    gData.rightCalibration.Clear();
    return false;
  }
  if(!gData.leftCalibration.Load(fn,"left")) {
    gData.leftCalibration.Clear();
    gData.rightCalibration.Clear();
    return false;
  }
  if(!gData.rightCalibration.Load(fn,"right")) {
    gData.leftCalibration.Clear();
    gData.rightCalibration.Clear();
    return false;
  }  
  return true;
}


BOOL setIKBiasConfiguration(const double* klamptConfig)
{
  if(klamptConfig) {
    gData.ikBiasConfig.clear();
    return true;
  }
  else {
    if(!gData.robotModel) {
      fprintf(stderr,"setIKBiasConfiguration: no robot model set\n");
      return false;
    }
    gData.ikBiasConfig.resize(gData.robotModel->q.n);
    gData.ikBiasConfig.copy(klamptConfig);
    return true;
  }
}

BOOL setKlamptModel(const char* klampt_model)
{
  if(gData.robotModel) {
    if(gData.robotModelFile == string(klampt_model)) {
      printf("setKlamptModel(): Note: robot model was already set before\n");
      return true;
    }
    else {
      printf("setKlamptModel(): Warning, robot model was set to %s before,\n",gData.robotModelFile.c_str());
      printf("  changing to %s\n",klampt_model);
    }
  }
  gData.robotModelFile = klampt_model;
  gData.robotModel = new Robot;
  if(!gData.robotModel->Load(klampt_model)) {
    printf("setKlamptModel() failed, robot could not be loaded\n");
    gData.robotModel = NULL;
    return 0;
  }
  gData.OnWorldChange();
  return 1;
}

BOOL getKlamptModel(char* buf,int bufsize)
{
  if((int)gData.robotModelFile.length() >= bufsize)
    return false;
  strncpy(buf,gData.robotModelFile.c_str(),bufsize);
  return true;
}

BOOL publishState(const char* system_state_addr)
{
  gData.systemStateAddr = system_state_addr;
}

void my_interrupt_handler(int s){
  printf("Caught signal %d, stopping robot motion\n",s);
  gData.mutex.try_lock();
  gData.mutex.unlock();
  stopMotion();
  sendShutdown();
  exit(1); 
}

///Starts up the robot
BOOL sendStartup()
{
  if(gData.startup ) {
    printf("Motion Module: sendStartup(): Motion Module already in startup phase, call has no effect\n");
    return 1;
  }
  if(gData.running) {
    printf("Motion Module: sendStartup(): Motion Module already running, call has no effect\n");
    return 1;
  }

  struct sigaction sigIntHandler;

  sigIntHandler.sa_handler = my_interrupt_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigIntHandler.sa_sigaction = 0;
  
  sigaction(SIGINT, &sigIntHandler, NULL);

  gData.kill = false;
  gData.failure = false;
  gData.startup = true;
  gControllerUpdateThread = ThreadStart(controller_thread_func,&gData);
  while(!gData.failure && !gData.running) {
    ThreadSleep(1);
  }
  if(gData.failure) return 0;
  return 1;
}

BOOL sendShutdown()
{
  stopMotion();
  gData.kill = true;
  ThreadJoin(gControllerUpdateThread);

  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = SIG_DFL;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigIntHandler.sa_sigaction = 0;

  sigaction(SIGINT, &sigIntHandler, NULL);
  printf("Motion Module: sendShutdown completed.\n");

  return true;
}

double getTime()
{
  return gData.t;
}

///Returns true if the robot is started
BOOL isStarted()
{
  return gData.running;
}

BOOL stopMotion()
{
  printf("stopMotion() called, sending 0 velocity commands...\n");
  sendMobileBaseVelocity(0,0,0);
  vector<double> zeros(numLimbDofs*2,0.0);
  if(gData.robotState.leftLimb.controlMode == LimbState::VELOCITY || gData.robotState.leftLimb.controlMode == LimbState::EFFORT) {
    sendLimbVelocity(LEFT,&zeros[0]);
  }
  else {
    if(gData.robotState.leftLimb.commandedConfig.size() != 7) {
      printf("Weird, no commanded config on left arm?\n");
      sendLimbVelocity(LEFT,&zeros[0]);
    }
    else {
      //notice a droop whenever switching to velocity control mode
      sendLimbPosition(LEFT,&gData.robotState.leftLimb.commandedConfig[0]);
    }
  }
  if(gData.robotState.rightLimb.controlMode == LimbState::VELOCITY || gData.robotState.rightLimb.controlMode == LimbState::EFFORT) {
    sendLimbVelocity(RIGHT,&zeros[0]);
  }
  else {
    if(gData.robotState.rightLimb.commandedConfig.size() != 7) {
      printf("Weird, no commanded config on right arm?\n");
      sendLimbVelocity(RIGHT,&zeros[0]);
    }
    else {
      sendLimbPosition(RIGHT,&gData.robotState.rightLimb.commandedConfig[0]);
    }
  }
  double dx=0,dy=0,dtheta=0;

  double veltol = 0.01;
  int count = 20;
  printf("Waiting for motion to actually stop..."); fflush(stdout);
  while(--count > 0) {
    if(isMobileBaseEnabled())
      getMobileBaseCommandedVelocity(&dx,&dy,&dtheta);
    if(fabs(dx) < veltol && fabs(dy) < veltol && fabs(dtheta) < veltol) {
      getLimbCommandedVelocity(BOTH,&zeros[0]);
      bool stopped = true;
      for(size_t i=0;i<zeros.size();i++)
	if(fabs(zeros[i]) > veltol ) {
    printf("Arm joint %d still moving, speed %g\n",(int)i,zeros[i]);
	  stopped = false;
	  break;
	}
      if(stopped) {
        printf("Done. \n");
        return 1;
      }
    }
    ThreadSleep(0.1);
    printf("."); fflush(stdout);
  }
  printf("\n");
  printf("Waited for 2 seconds, didn't actually stop?\n");
  printf("Base velocity %g %g %g\n",dx,dy,dtheta);
  printf("Arm velocity: ");
  cout<<zeros<<endl;
  return 0;
}

BOOL isTorsoEnabled()
{
  return gData.robotState.baxterEnabled;
}

////// HEAD ///////

///Returns true if the head is nodding
BOOL isHeadNodding()
{
  return gData.robotState.head.isNodding;
}

double getHeadPan()
{
  return gData.robotState.head.pan;
}

BOOL sendHeadPan(double target,double speed)
{
  if(!gData.robotState.baxterEnabled) return 0;
  ScopedLock lock(gData.mutex);
  if(gData.robotState.head.sendCommand) {
    printf("Motion Module: Warning, last sendHeadPan command wasn't processed\n");
  }
  gData.robotState.head.panTarget = target;
  gData.robotState.head.panSpeed = speed;
  gData.robotState.head.sendCommand = true;
  return 1;
}


////// MOBILE BASE ///////

///Returns true if the mobile base is enabled
BOOL isMobileBaseEnabled()
{
  return gData.robotState.base.enabled;
}

///Returns true if the mobile base is moving
BOOL isMobileBaseMoving()
{
  return gData.robotState.base.moving;
}
/*
///Returns the time until the mobile base stops, or -1 if it's doing a velocity command
double getMobileBaseMoveTime()
{
  return 0;
}
*/
///Returns the target for the mobile base in local coordinates
BOOL getMobileBaseTarget(double* xrel,double* yrel,double* thetarel)
{
  if(!isMobileBaseEnabled()) return 0;
  ScopedLock lock(gData.mutex);
  Vector tgt;
  if(!gData.robotState.base.GetRelativeTarget(tgt)) return 0;
  *xrel = tgt[0];
  *yrel = tgt[1];
  *thetarel = tgt[2];
  return 1;
}
///Returns the target for the mobile base in absolute (odometry) coordinates
BOOL getMobileBaseOdometryTarget(double* x,double* y,double* theta)
{
  if(!isMobileBaseEnabled()) return 0;
  ScopedLock lock(gData.mutex);
  Vector tgt;
  if(!gData.robotState.base.GetOdometryTarget(tgt)) return 0;
  *x = tgt[0];
  *y = tgt[1];
  *theta = tgt[2];
  return 1;
}

///Returns mobile base velocity in local coordinates
BOOL getMobileBaseVelocity(double* dx,double* dy,double* dtheta)
{
  if(!isMobileBaseEnabled()) return 0;
  ScopedLock lock(gData.mutex);
  *dx=gData.robotState.base.velocity[0];
  *dy=gData.robotState.base.velocity[1];
  *dtheta=gData.robotState.base.velocity[2];
  return 1;
}
///Returns odometry coordinates of the mobile base
BOOL getMobileBaseOdometry(double* x,double* y,double* theta)
{
  if(!isMobileBaseEnabled()) return 0;
  ScopedLock lock(gData.mutex);
  *x=gData.robotState.base.odometry[0];
  *y=gData.robotState.base.odometry[1];
  *theta=gData.robotState.base.odometry[2];
  return 1;
}

///Returns commanded velocity of the mobile base
BOOL getMobileBaseCommandedVelocity(double* dx,double* dy,double* dtheta)
{
  if(!isMobileBaseEnabled()) return 0;
  if(gData.robotState.base.commandedVelocity.empty()) return 0;
  ScopedLock lock(gData.mutex);
  *dx=gData.robotState.base.commandedVelocity[0];
  *dy=gData.robotState.base.commandedVelocity[1];
  *dtheta=gData.robotState.base.commandedVelocity[2];
  return 1;
}

///Sends a move target for the mobile base, in local coordinates
BOOL sendMobileBasePosition(double xrel,double yrel,double thetarel)
{
  if(!isMobileBaseEnabled()) return 0;
  ScopedLock lock(gData.mutex);
  gData.robotState.base.controlMode = BaseState::RELATIVE_POSITION;
  gData.robotState.base.command.resize(3);
  gData.robotState.base.command[0] = xrel;
  gData.robotState.base.command[1] = yrel;
  gData.robotState.base.command[2] = thetarel;
  gData.robotState.base.sendCommand = true;
  return 1;
}
///Sends a move target for the mobile base, in absolute (odometry) coordinates
BOOL sendMobileBaseOdometryPosition(double x,double y,double theta)
{
  if(!isMobileBaseEnabled()) return 0;
  ScopedLock lock(gData.mutex);
  gData.robotState.base.controlMode = BaseState::ODOMETRY_POSITION;
  gData.robotState.base.command.resize(3);
  gData.robotState.base.command[0] = x;
  gData.robotState.base.command[1] = y;
  gData.robotState.base.command[2] = theta;
  gData.robotState.base.sendCommand = true;
  return 1;
}
///Sends a move velocity for the mobile base, in local coordinates
BOOL sendMobileBaseVelocity(double dxrel,double dyrel,double dthetarel)
{
  if(!isMobileBaseEnabled()) return 0;
  ScopedLock lock(gData.mutex);
  gData.robotState.base.controlMode = BaseState::RELATIVE_VELOCITY;
  gData.robotState.base.command.resize(3);
  gData.robotState.base.command[0] = dxrel;
  gData.robotState.base.command[1] = dyrel;
  gData.robotState.base.command[2] = dthetarel;
  gData.robotState.base.sendCommand = true;
  return 1;
}

////// LIMB (LOW-LEVEL) ///////
///Returns true the limb is being commanded in position mode
BOOL isLimbPositionMode(int limb)
{
  if(limb==LEFT) return (gData.robotState.leftLimb.controlMode == LimbState::POSITION);
  else if(limb==RIGHT) return (gData.robotState.rightLimb.controlMode == LimbState::POSITION);
  else return (gData.robotState.rightLimb.controlMode == LimbState::POSITION) && (gData.robotState.leftLimb.controlMode == LimbState::POSITION);
}
///Returns true the limb is being commanded in velocity mode
BOOL isLimbVelocityMode(int limb)
{
  if(limb==LEFT) return (gData.robotState.leftLimb.controlMode == LimbState::VELOCITY);
  else if(limb==RIGHT) return (gData.robotState.rightLimb.controlMode == LimbState::VELOCITY);
  else return (gData.robotState.rightLimb.controlMode == LimbState::VELOCITY) && (gData.robotState.leftLimb.controlMode == LimbState::VELOCITY);
}
///Returns true the limb is being commanded in effort mode
BOOL isLimbEffortMode(int limb)
{
  if(limb==LEFT) return (gData.robotState.leftLimb.controlMode == LimbState::EFFORT);
  else if(limb==RIGHT) return (gData.robotState.rightLimb.controlMode == LimbState::EFFORT);
  else return (gData.robotState.rightLimb.controlMode == LimbState::EFFORT) && (gData.robotState.leftLimb.controlMode == LimbState::EFFORT);
}
///Returns true the limb is being commanded in raw position mode
BOOL isLimbRawPositionMode(int limb)
{
  if(limb==LEFT) return (gData.robotState.leftLimb.controlMode == LimbState::RAW_POSITION);
  else if(limb==RIGHT) return (gData.robotState.rightLimb.controlMode == LimbState::RAW_POSITION);
  else return (gData.robotState.rightLimb.controlMode == LimbState::RAW_POSITION) && (gData.robotState.leftLimb.controlMode == LimbState::RAW_POSITION);
}

inline void Copy(const Vector& v,double* out)
{
  for(int i=0;i<v.n;i++) out[i]=v[i];
}

inline void Copy(const double* out,Vector& v)
{
  for(int i=0;i<v.n;i++) v[i]=out[i];
}

///Returns the sensed limb position
BOOL getLimbPosition(int limb,double* angles)
{
  ScopedLock lock(gData.mutex);
  if(limb==LEFT) {
    if(gData.robotState.leftLimb.sensedConfig.empty()) return 0;
    Copy(gData.robotState.leftLimb.sensedConfig,angles);
    return 1;
  }
  else if(limb==RIGHT) {
    if(gData.robotState.rightLimb.sensedConfig.empty()) return 0;
    Copy(gData.robotState.rightLimb.sensedConfig,angles);
    return 1;
  }
  else {
    if(gData.robotState.leftLimb.sensedConfig.empty() || gData.robotState.rightLimb.sensedConfig.empty()) return 0;
    Copy(gData.robotState.leftLimb.sensedConfig,angles+numLimbDofs);
    Copy(gData.robotState.rightLimb.sensedConfig,angles+numLimbDofs);
    return 1;
  }
}
///Returns the sensed limb velocity
BOOL getLimbVelocity(int limb,double* dangles)
{
  ScopedLock lock(gData.mutex);
  if(limb==LEFT) {
    if(gData.robotState.leftLimb.sensedVelocity.empty()) return 0;
    Copy(gData.robotState.leftLimb.sensedVelocity,dangles);
    return 1;
  }
  else if(limb==RIGHT) {
    if(gData.robotState.rightLimb.sensedVelocity.empty()) return 0;
    Copy(gData.robotState.rightLimb.sensedVelocity,dangles);
    return 1;
  }
  else {
    if(gData.robotState.leftLimb.sensedVelocity.empty() || gData.robotState.rightLimb.sensedVelocity.empty()) return 0;
    Copy(gData.robotState.leftLimb.sensedVelocity,dangles+numLimbDofs);
    Copy(gData.robotState.rightLimb.sensedVelocity,dangles+numLimbDofs);
    return 1;
  }
}
///Returns the sensed limb velocity
BOOL getLimbEffort(int limb,double* efforts)
{
  ScopedLock lock(gData.mutex);
  if(limb==LEFT) {
    if(gData.robotState.leftLimb.sensedEffort.empty()) return 0;
    Copy(gData.robotState.leftLimb.sensedEffort,efforts);
    return 1;
  }
  else if(limb==RIGHT) {
    if(gData.robotState.rightLimb.sensedEffort.empty()) return 0;
    Copy(gData.robotState.rightLimb.sensedEffort,efforts);
    return 1;
  }
  else {
    if(gData.robotState.leftLimb.sensedEffort.empty() || gData.robotState.rightLimb.sensedEffort.empty()) return 0;
    Copy(gData.robotState.leftLimb.sensedEffort,efforts+numLimbDofs);
    Copy(gData.robotState.rightLimb.sensedEffort,efforts+numLimbDofs);
    return 1;
  }
}
///Returns the commanded limb position
BOOL getLimbCommandedPosition(int limb,double* angles)
{
  ScopedLock lock(gData.mutex);
  if(limb==LEFT) {
    if(gData.robotState.leftLimb.commandedConfig.empty()) return 0;
    Copy(gData.robotState.leftLimb.commandedConfig,angles);
    return 1;
  }
  else if(limb==RIGHT) {
    if(gData.robotState.rightLimb.commandedConfig.empty()) return 0;
    Copy(gData.robotState.rightLimb.commandedConfig,angles);
    return 1;
  }
  else {
    if(gData.robotState.leftLimb.commandedConfig.empty() || gData.robotState.rightLimb.commandedConfig.empty()) return 0;
    Copy(gData.robotState.leftLimb.commandedConfig,angles+numLimbDofs);
    Copy(gData.robotState.rightLimb.commandedConfig,angles+numLimbDofs);
    return 1;
  }
}
///Returns the commanded limb velocity
BOOL getLimbCommandedVelocity(int limb,double* dangles)
{
  ScopedLock lock(gData.mutex);
  if(limb==LEFT) {
    if(gData.robotState.leftLimb.commandedVelocity.empty()) return 0;
    Copy(gData.robotState.leftLimb.commandedVelocity,dangles);
    return 1;
  }
  else if(limb==RIGHT) {
    if(gData.robotState.rightLimb.commandedVelocity.empty()) return 0;
    Copy(gData.robotState.rightLimb.commandedVelocity,dangles);
    return 1;
  }
  else {
    if(gData.robotState.leftLimb.commandedVelocity.empty() || gData.robotState.rightLimb.commandedVelocity.empty()) return 0;
    Copy(gData.robotState.leftLimb.commandedVelocity,dangles+numLimbDofs);
    Copy(gData.robotState.rightLimb.commandedVelocity,dangles+numLimbDofs);
    return 1;
  }
}
///Sends a limb position command
BOOL sendLimbPosition(int limb,const double* angles)
{
  if(!gData.robotState.baxterEnabled) return 0;
  if(gData.enableCollisionChecking && gData.CheckCollisions(limb,Vector(NumLimbDofs(limb),angles))) return 0;
  if(limb==LEFT) {
    ScopedLock lock(gData.mutex);
    gData.SetLimbGovernor(LEFT,LimbState::Normal);
    gData.robotState.leftLimb.controlMode = LimbState::POSITION;
    gData.robotState.leftLimb.commandedConfig.resize(numLimbDofs);
    gData.robotState.leftLimb.commandedConfig.copy(angles);
    fill(gData.robotState.leftLimb.commandedVelocity.begin(),gData.robotState.leftLimb.commandedVelocity.end(),0.0);
    gData.robotState.leftLimb.sendCommand = true;
  }
  else if(limb==RIGHT) {
    ScopedLock lock(gData.mutex);
    gData.SetLimbGovernor(RIGHT,LimbState::Normal);
    gData.robotState.rightLimb.controlMode = LimbState::POSITION;
    gData.robotState.rightLimb.commandedConfig.resize(numLimbDofs);
    gData.robotState.rightLimb.commandedConfig.copy(angles);
    fill(gData.robotState.rightLimb.commandedVelocity.begin(),gData.robotState.rightLimb.commandedVelocity.end(),0.0);
    gData.robotState.rightLimb.sendCommand = true;
  }
  else {
    if(!sendLimbPosition(LEFT,angles)) return 0;
    if(!sendLimbPosition(RIGHT,angles+numLimbDofs)) return 0;
  }
  return 1;
}
///Sends a limb raw position command
BOOL sendLimbRawPosition(int limb,const double* angles)
{
  if(!gData.robotState.baxterEnabled) return 0;
  if(limb==LEFT) {
    ScopedLock lock(gData.mutex);
    if(gData.planner.plannerActive[LEFT]) {
      gData.planner.plannerActive[LEFT]=false;
      gData.planner.planningThread.SetObjective(NULL);
    }
    if(gData.robotState.leftLimb.motionQueueActive)
      gData.robotState.leftLimb.motionQueueActive = false;
    gData.robotState.leftLimb.controlMode = LimbState::RAW_POSITION;
    gData.robotState.leftLimb.commandedConfig.resize(numLimbDofs);
    gData.robotState.leftLimb.commandedConfig.copy(angles);
    gData.robotState.leftLimb.sendCommand = true;
  }
  else if(limb==RIGHT) {
    ScopedLock lock(gData.mutex);
    if(gData.planner.plannerActive[RIGHT]) {
      gData.planner.plannerActive[RIGHT]=false;
      gData.planner.planningThread.SetObjective(NULL);
    }
    if(gData.robotState.rightLimb.motionQueueActive)
      gData.robotState.rightLimb.motionQueueActive = false;
    gData.robotState.rightLimb.controlMode = LimbState::RAW_POSITION;
    gData.robotState.rightLimb.commandedConfig.resize(numLimbDofs);
    gData.robotState.rightLimb.commandedConfig.copy(angles);
    gData.robotState.rightLimb.sendCommand = true;
  }
  else {
    if(!sendLimbRawPosition(LEFT,angles)) return 0;
    if(!sendLimbRawPosition(RIGHT,angles+numLimbDofs)) return 0;
  }
  return 1;
}
///Sends a limb velocity command
BOOL sendLimbVelocity(int limb,const double* angles)
{
  if(!gData.robotState.baxterEnabled) return 0;
  if(limb==LEFT) {
    ScopedLock lock(gData.mutex);
    gData.SetLimbGovernor(LEFT,LimbState::Normal);
    gData.robotState.leftLimb.controlMode = LimbState::VELOCITY;
    gData.robotState.leftLimb.commandedVelocity.resize(numLimbDofs);
    gData.robotState.leftLimb.commandedVelocity.copy(angles);
    gData.robotState.leftLimb.sendCommand = true;
  }
  else if(limb==RIGHT) {
    ScopedLock lock(gData.mutex);
    gData.SetLimbGovernor(RIGHT,LimbState::Normal);
    gData.robotState.rightLimb.controlMode = LimbState::VELOCITY;
    gData.robotState.rightLimb.commandedVelocity.resize(numLimbDofs);
    gData.robotState.rightLimb.commandedVelocity.copy(angles);
    gData.robotState.rightLimb.sendCommand = true;
  }
  else {
    if(!sendLimbVelocity(LEFT,angles)) return 0;
    if(!sendLimbVelocity(RIGHT,angles+numLimbDofs)) return 0;
  }
  return 1;
}

///Sends a limb effort command
BOOL sendLimbEffort(int limb,const double* angles)
{
  if(!gData.robotState.baxterEnabled) return 0;
  if(limb==LEFT) {
    ScopedLock lock(gData.mutex);
    gData.SetLimbGovernor(LEFT,LimbState::Normal);
    gData.robotState.leftLimb.controlMode = LimbState::EFFORT;
    gData.robotState.leftLimb.commandedEffort.resize(numLimbDofs);
    gData.robotState.leftLimb.commandedEffort.copy(angles);
    gData.robotState.leftLimb.sendCommand = true;
  }
  else if(limb==RIGHT) {
    ScopedLock lock(gData.mutex);
    gData.SetLimbGovernor(RIGHT,LimbState::Normal);
    gData.robotState.rightLimb.controlMode = LimbState::EFFORT;
    gData.robotState.rightLimb.commandedEffort.resize(numLimbDofs);
    gData.robotState.rightLimb.commandedEffort.copy(angles);
    gData.robotState.rightLimb.sendCommand = true;
  }
  else {
    if(!sendLimbEffort(LEFT,angles)) return 0;
    if(!sendLimbEffort(RIGHT,angles+numLimbDofs)) return 0;
  }
  return 1;
}

APIENTRY BOOL enableLimbSelfCollisionAvoidance(int limb,BOOL enabled)
{
  ScopedLock lock(gData.mutex);
  if(limb==LEFT || limb==BOTH) 
    gData.robotState.leftLimb.enableSelfCollisionAvoidance = enabled;
  if(limb==RIGHT || limb==BOTH) 
    gData.robotState.rightLimb.enableSelfCollisionAvoidance = enabled;
  return 1;
}

APIENTRY BOOL setEndEffectorOffset(int limb,const double* localPosition)
{
  ScopedLock lock(gData.mutex);
  if(limb == LEFT || limb == BOTH) 
    gData.robotState.leftLimb.endEffectorOffset.set(localPosition);
  if(limb == RIGHT || limb == BOTH)
    gData.robotState.rightLimb.endEffectorOffset.set(localPosition);
  return true;
}

APIENTRY BOOL getEndEffectorSensedTransform(int limb,double* rotation,double* position)
{
  if(!gData.robotModel) return 0;
  int index;
  Vector3 offset;
  if(limb == LEFT)  {
    index=gData.robotModel->LinkIndex(klampt_left_ee_name);
    offset = gData.robotState.leftLimb.endEffectorOffset;
  }
  else if(limb == RIGHT) {
    index=gData.robotModel->LinkIndex(klampt_right_ee_name);
    offset = gData.robotState.rightLimb.endEffectorOffset;
  }
  else 
    return 0;
  if(index < 0) return 0;
  ScopedLock lock(gData.mutex);
  ScopedLock lock2(gData.robotMutex);
  gData.GetKlamptSensedConfig(gData.robotModel->q);
  //ignore base
  for(size_t i=0;i<gData.baseKlamptIndices.size();i++) {
    gData.robotModel->q[gData.baseKlamptIndices[i]] = 0;
  }
  gData.robotModel->UpdateSelectedFrames(index);
  gData.robotModel->links[index].T_World.R.get(rotation);
  (gData.robotModel->links[index].T_World*offset).get(position);
  return 1;
}
APIENTRY BOOL getEndEffectorSensedVelocity(int limb,double* angVel,double* vel)
{
  if(!gData.robotModel) return 0;
  int index;
  Vector3 offset;
  if(limb == LEFT)  {
    index=gData.robotModel->LinkIndex(klampt_left_ee_name);
    offset = gData.robotState.leftLimb.endEffectorOffset;
  }
  else if(limb == RIGHT)  {
    index=gData.robotModel->LinkIndex(klampt_right_ee_name);
    offset = gData.robotState.rightLimb.endEffectorOffset;
    }
  else 
    return 0;
  if(index < 0) return 0;
  ScopedLock lock(gData.mutex);
  ScopedLock lock2(gData.robotMutex);
  gData.GetKlamptSensedConfig(gData.robotModel->q);
  gData.GetKlamptSensedVelocity(gData.robotModel->dq);
  //ignore base
  for(size_t i=0;i<gData.baseKlamptIndices.size();i++)
    gData.robotModel->q[gData.baseKlamptIndices[i]] = gData.robotModel->dq[gData.baseKlamptIndices[i]] = 0;
  gData.robotModel->UpdateSelectedFrames(index);
  Vector3 dp,dw;
  gData.robotModel->GetWorldVelocity(offset,index,gData.robotModel->dq,dp);
  gData.robotModel->GetWorldAngularVelocity(index,gData.robotModel->dq,dw);
  dp.get(vel);
  dw.get(angVel);
  return 1;
}
APIENTRY BOOL getEndEffectorCommandedTransform(int limb,double* rotation,double* position)
{
  if(!gData.robotModel) return 0;
  int index;
  Vector3 offset;
  if(limb == LEFT)  {
    index=gData.robotModel->LinkIndex(klampt_left_ee_name);
    offset = gData.robotState.leftLimb.endEffectorOffset;
  }
  else if(limb == RIGHT)  {
    index=gData.robotModel->LinkIndex(klampt_right_ee_name);
    offset = gData.robotState.rightLimb.endEffectorOffset;
    }
  else 
    return 0;
  if(index < 0) return 0;
  ScopedLock lock(gData.mutex);
  ScopedLock lock2(gData.robotMutex);
  gData.GetKlamptCommandedConfig(gData.robotModel->q);
  //ignore base
  for(size_t i=0;i<gData.baseKlamptIndices.size();i++)
    gData.robotModel->q[gData.baseKlamptIndices[i]] = 0;
  gData.robotModel->UpdateSelectedFrames(index);
  gData.robotModel->links[index].T_World.R.get(rotation);
  (gData.robotModel->links[index].T_World*offset).get(position);
  return 1;
}
APIENTRY BOOL getEndEffectorCommandedVelocity(int limb,double* angVel,double* vel)
{
  if(!gData.robotModel) return 0;
  int index;
  Vector3 offset;
  if(limb == LEFT)  {
    index=gData.robotModel->LinkIndex(klampt_left_ee_name);
    offset = gData.robotState.leftLimb.endEffectorOffset;
  }
  else if(limb == RIGHT)  {
    index=gData.robotModel->LinkIndex(klampt_right_ee_name);
    offset = gData.robotState.rightLimb.endEffectorOffset;
    }
  else 
    return 0;
  ScopedLock lock(gData.mutex);
  ScopedLock lock2(gData.robotMutex);
  gData.GetKlamptCommandedConfig(gData.robotModel->q);
  gData.GetKlamptCommandedVelocity(gData.robotModel->dq);
  //ignore base
  for(size_t i=0;i<gData.baseKlamptIndices.size();i++)
    gData.robotModel->q[gData.baseKlamptIndices[i]] = gData.robotModel->dq[gData.baseKlamptIndices[i]] = 0;
  gData.robotModel->UpdateSelectedFrames(index);
  Vector3 dp,dw;
  gData.robotModel->GetWorldVelocity(offset,index,gData.robotModel->dq,dp);
  gData.robotModel->GetWorldAngularVelocity(index,gData.robotModel->dq,dw);
  dp.get(vel);
  dw.get(angVel);
  return 1;
}
APIENTRY BOOL sendEndEffectorVelocity(int limb,const double* angVel,const double* vel)
{
  if(!gData.robotModel) return 0;
  Vector dlimb;
  gData.SolveIKVelocity(limb,Vector3(angVel),Vector3(vel),dlimb,true);
  return sendLimbVelocity(limb,dlimb);
}
APIENTRY BOOL sendEndEffectorPositionVelocity(int limb,const double* vel)
{
  double err[3] = {Inf,Inf,Inf};
  return sendEndEffectorVelocity(limb,err,vel);
}
APIENTRY BOOL sendEndEffectorMoveTo(int limb,const double* rotation,const double* position,double maxRotationDeviation,double maxPositionDeviation,double maxJointDeviation)
{
  if(!gData.robotModel) return 0;
  vector<int> indices;
  if(limb == LEFT) {
    indices = gData.leftKlamptIndices;
  }
  else if(limb == RIGHT) {
    indices = gData.rightKlamptIndices;
  }
  else 
    return false;
  Vector qlimb;
  RigidTransform T;
  T.R = Matrix3(rotation);
  T.t = Vector3(position);
  if(maxJointDeviation <= 0) maxJointDeviation = Inf;
  if(maxPositionDeviation <= 0) maxPositionDeviation = Inf;
  if(maxRotationDeviation <= 0) maxRotationDeviation = Inf;
  //set bounds
  Robot* robot = gData.robotModel;
  Vector qmin,qmax;
  {
    ScopedLock lock(gData.mutex);
    gData.GetKlamptCommandedConfig(qmin);
  }
  qmax = qmin;
  for(size_t i=0;i<indices.size();i++) {
    qmax[indices[i]] += maxJointDeviation;
    qmin[indices[i]] -= maxJointDeviation;
    qmax[indices[i]] = Min(qmax[indices[i]],robot->qMax[indices[i]]);
    qmin[indices[i]] = Max(qmin[indices[i]],robot->qMin[indices[i]]);
  }
  if(!gData.SolveIK(limb,T,qmin,qmax,maxRotationDeviation,maxPositionDeviation,qlimb,true)) return false;
  //solved, now send it as a ramp
  return sendMotionQueueRamp(limb,qlimb);
}
APIENTRY BOOL sendEndEffectorPositionDrive(int limb,const double* vel)
{
  double err[3] = {Inf,Inf,Inf};
  return sendEndEffectorDrive(limb,err,vel);
}
APIENTRY BOOL sendEndEffectorDrive(int limb,const double* angVel,const double* vel)
{
  if(!gData.robotModel) return 0;

  gData.mutex.lock();
  LimbState* limbState;
  if(limb == LEFT) {
    gData.SetLimbGovernor(LEFT,LimbState::EndEffectorDrive);
    limbState = &gData.robotState.leftLimb;
  }
  else if(limb == RIGHT) {
    gData.SetLimbGovernor(RIGHT,LimbState::EndEffectorDrive);
    limbState = &gData.robotState.rightLimb;
  }
  else {
    gData.mutex.unlock();
    return 0;
  }
  limbState->driveAngVel.set(angVel);
  limbState->driveVel.set(vel);
  limbState->driveSpeedAdjustment = 1.0;
  gData.mutex.unlock();
  return 1;
}

BOOL isEndEffectorDriveEnabled(int limb)
{
  if(limb == LEFT) return gData.robotState.leftLimb.governor == LimbState::EndEffectorDrive;
  else if(limb == RIGHT) return gData.robotState.rightLimb.governor == LimbState::EndEffectorDrive;
  else return gData.robotState.leftLimb.governor == LimbState::EndEffectorDrive && gData.robotState.rightLimb.governor == LimbState::EndEffectorDrive;
}

////// GRIPPER ///////

///Returns true the gripper is enabled
BOOL isGripperEnabled(int limb)
{
  if(limb == LEFT) return gData.robotState.leftGripper.enabled;
  else if(limb == RIGHT) return gData.robotState.rightGripper.enabled;
  else return gData.robotState.leftGripper.enabled && gData.robotState.rightGripper.enabled;
}


///Returns the type string of the gripper in the given char* buffer
APIENTRY BOOL getGripperType(int limb,char* buf,int bufsize)
{
  if(limb == LEFT) { 
    if(gData.robotState.leftGripper.name.size() >= bufsize) return 0;
    strncpy(buf,gData.robotState.leftGripper.name.c_str(),gData.robotState.leftGripper.name.size());
    buf[gData.robotState.leftGripper.name.size()] = 0;
    return 1;
  }
  else if(limb == RIGHT) {
    if(gData.robotState.rightGripper.name.size() >= bufsize) return 0;
    strncpy(buf,gData.robotState.rightGripper.name.c_str(),gData.robotState.rightGripper.name.size());
    buf[gData.robotState.rightGripper.name.size()] = 0;
    return 1;
  }
  else return 0;
}

///Returns the number of DOFs for controlling the gripper
APIENTRY int numGripperDofs(int limb)
{
  if(limb == LEFT) return gData.robotState.leftGripper.position.n;
  else if(limb == RIGHT) return gData.robotState.rightGripper.position.n;
  else return gData.robotState.leftGripper.position.n + gData.robotState.rightGripper.position.n;
}

///Returns true the gripper is moving to the desired setpoint
BOOL isGripperMoving(int limb)
{
  if(limb == LEFT) return gData.robotState.leftGripper.moving;
  else if(limb == RIGHT) return gData.robotState.rightGripper.moving;
  else return gData.robotState.leftGripper.moving && gData.robotState.rightGripper.moving;
}
/*
///Returns the estimated time until the gripper stops
double getGripperMoveTime(int limb)
{
  return 0;
}
*/
///Returns the gripper position
BOOL getGripperPosition(int limb,double* config)
{
  if(!isGripperEnabled(limb)) return 0;
  ScopedLock lock(gData.mutex);
  if(limb == LEFT) {
    Copy(gData.robotState.leftGripper.position,config);
    return 1;
  }
  else if(limb == RIGHT) {
    Copy(gData.robotState.rightGripper.position,config);
    return 1;
  }
  else {
    Copy(gData.robotState.leftGripper.position,config);
    Copy(gData.robotState.rightGripper.position,config+numGripperDofs(LEFT));
    return 1;
  }
}
///Returns the gripper target
BOOL getGripperTarget(int limb,double* config)
{
  if(!isGripperEnabled(limb)) return 0;
  ScopedLock lock(gData.mutex);
  if(limb == LEFT) {
    Copy(gData.robotState.leftGripper.positionCommand,config);
    return 1;
  }
  else if(limb == RIGHT) {
    Copy(gData.robotState.rightGripper.positionCommand,config);
    return 1;
  }
  else {
    Copy(gData.robotState.leftGripper.positionCommand,config);
    Copy(gData.robotState.rightGripper.positionCommand,config+numGripperDofs(LEFT));
    return 1;
  }
}
///Returns the gripper effort
BOOL getGripperEffort(int limb,double* config)
{
  if(!isGripperEnabled(limb)) return 0;
  ScopedLock lock(gData.mutex);
  if(limb == LEFT) {
    if(gData.robotState.leftGripper.force.empty()) return 0;
    Copy(gData.robotState.leftGripper.force,config);
    return 1;
  }
  else if(limb == RIGHT) {
    if(gData.robotState.rightGripper.force.empty()) return 0;
    Copy(gData.robotState.rightGripper.force,config);
    return 1;
  }
  else {
    if(gData.robotState.leftGripper.force.empty() || gData.robotState.leftGripper.force.empty()) return 0;
    Copy(gData.robotState.leftGripper.force,config);
    Copy(gData.robotState.rightGripper.force,config+numGripperDofs(LEFT));
    return 1;
  }
}
///Tells the gripper to close
BOOL sendCloseGripper(int limb)
{
  if(limb == BOTH) return 0;
  if(numGripperDofs(limb) == 1) {
    double position = 0;
    double speed = 1;
    double effort = 1;
    return sendSetGripper(limb,&position,&speed,&effort);
  }
  else {
    double position [4] = {0,0,0,1};
    double speed [4] = {1,1,1,1};
    double effort [4] = {1,1,1,1};
    return sendSetGripper(limb,position,speed,effort);
  }
}
///Tells the gripper to open
BOOL sendOpenGripper(int limb)
{
  if(limb == BOTH) return 0;
  if(numGripperDofs(limb) == 1) {
    double position = 1;
    double speed = 1;
    double effort = 1;
    return sendSetGripper(limb,&position,&speed,&effort);
  }
  else {
    double position [4] = {1,1,1,1};
    double speed [4] = {1,1,1,1};
    double effort [4] = {1,1,1,1};
    return sendSetGripper(limb,position,speed,effort);
  }
}
///Tells the gripper to go to the desired position with the given speed and effort
BOOL sendSetGripper(int limb,const double* position,const double* speed,const double* effort)
{
  if(!isGripperEnabled(limb)) return 0;
  ScopedLock lock(gData.mutex);
  if(limb == LEFT) {
    Copy(position,gData.robotState.leftGripper.positionCommand);
    Copy(speed,gData.robotState.leftGripper.speedCommand);
    Copy(effort,gData.robotState.leftGripper.forceCommand);
    gData.robotState.leftGripper.sendCommand = true;
    return 1;
  }
  else if(limb == RIGHT) {
    Copy(position,gData.robotState.rightGripper.positionCommand);
    Copy(speed,gData.robotState.rightGripper.speedCommand);
    Copy(effort,gData.robotState.rightGripper.forceCommand);
    gData.robotState.rightGripper.sendCommand = true;
    return 1;
  }
  else {
    Copy(position,gData.robotState.leftGripper.positionCommand);
    Copy(speed,gData.robotState.leftGripper.speedCommand);
    Copy(effort,gData.robotState.leftGripper.forceCommand);
    Copy(position+numGripperDofs(LEFT),gData.robotState.rightGripper.positionCommand);
    Copy(speed+numGripperDofs(LEFT),gData.robotState.rightGripper.speedCommand);
    Copy(effort+numGripperDofs(LEFT),gData.robotState.rightGripper.forceCommand);
    gData.robotState.leftGripper.sendCommand = true;
    gData.robotState.rightGripper.sendCommand = true;
    return 1;
  }  
}


////// LIMB MOTION QUEUE ///////

///Returns true the limb is being controlled by the motion queue
BOOL isMotionQueueEnabled(int limb)
{
  if(limb == LEFT) return gData.robotState.leftLimb.motionQueueActive;
  else if(limb == RIGHT) return gData.robotState.rightLimb.motionQueueActive;
  else return gData.robotState.leftLimb.motionQueueActive && gData.robotState.rightLimb.motionQueueActive;
}
///Returns true the limb is moving according to the motion queue
BOOL isMotionQueueMoving(int limb)
{
  if(limb == LEFT) {
    ScopedLock lock(gData.mutex);
    return gData.robotState.leftLimb.motionQueueActive && (gData.robotState.leftLimb.motionQueue.TimeRemaining() >= 0);
  }
  else if(limb == RIGHT) {
    ScopedLock lock(gData.mutex);
    return gData.robotState.rightLimb.motionQueueActive && (gData.robotState.rightLimb.motionQueue.TimeRemaining() >= 0);
  }
  else return isMotionQueueMoving(LEFT) && isMotionQueueMoving(RIGHT);
}
///Returns the estimated amount of time until the motion queue stops
double getMotionQueueMoveTime(int limb)
{
  if(limb == LEFT) {
    ScopedLock lock(gData.mutex);
    if(gData.robotState.leftLimb.motionQueueActive)
      return Max(gData.robotState.leftLimb.motionQueue.TimeRemaining(),0.0);
    return 0;
  }
  else if(limb == RIGHT) {
    ScopedLock lock(gData.mutex);
    if(gData.robotState.rightLimb.motionQueueActive)
      return Max(gData.robotState.rightLimb.motionQueue.TimeRemaining(),0.0);
    return 0;
  }
  else return Max(getMotionQueueMoveTime(LEFT),getMotionQueueMoveTime(RIGHT));
}
///Returns the end configuration of the motion queue
BOOL getMotionQueueTarget(int limb,double* angles)
{
  if(limb == LEFT) {
    ScopedLock lock(gData.mutex);
    if(!gData.robotState.leftLimb.motionQueueActive) return 0;
    Vector q=gData.robotState.leftLimb.motionQueue.Endpoint();
    q.getCopy(angles);
    return 1;
  }
  else if(limb == RIGHT) {
    ScopedLock lock(gData.mutex);
    if(!gData.robotState.rightLimb.motionQueueActive) return 0;
    Vector q=gData.robotState.rightLimb.motionQueue.Endpoint();
    q.getCopy(angles);
    return 1;
  }
  else {
    return getMotionQueueTarget(LEFT,angles) && getMotionQueueTarget(RIGHT,angles+numLimbDofs);
  }
}
BOOL _sendMotionQueueLinear(int limb,double duration,const double* angles,bool immediate)
{
  if(limb == LEFT) {
    gData.SetLimbGovernor(LEFT,LimbState::MotionQueue);
    Config q(numLimbDofs);
    Copy(angles,q);
    if(immediate) gData.robotState.leftLimb.motionQueue.Cut(0);
    gData.robotState.leftLimb.motionQueue.AppendLinear(q,duration);
  }
  else if(limb == RIGHT) {
    gData.SetLimbGovernor(RIGHT,LimbState::MotionQueue);
    Config q(numLimbDofs);
    Copy(angles,q);
    if(immediate) gData.robotState.rightLimb.motionQueue.Cut(0);
    gData.robotState.rightLimb.motionQueue.AppendLinear(q,duration);
  }
  else {
    return _sendMotionQueueLinear(LEFT,duration,angles,immediate) && _sendMotionQueueLinear(RIGHT,duration,angles+numLimbDofs,immediate);
  }
  return 1;
}
BOOL _sendMotionQueueCubic(int limb,double duration,const double* angles,const double* dangles,bool immediate)
{
  if(limb == LEFT) {
    gData.SetLimbGovernor(LEFT,LimbState::MotionQueue);
    Config q(numLimbDofs),dq(numLimbDofs);
    Copy(angles,q);
    Copy(dangles,dq);
    if(immediate) gData.robotState.leftLimb.motionQueue.Cut(0);
    gData.robotState.leftLimb.motionQueue.AppendCubic(q,dq,duration);
  }
  else if(limb == RIGHT) {
    gData.SetLimbGovernor(RIGHT,LimbState::MotionQueue);
    Config q(numLimbDofs),dq(numLimbDofs);
    Copy(angles,q);
    Copy(dangles,dq);
    if(immediate) gData.robotState.rightLimb.motionQueue.Cut(0);
    gData.robotState.rightLimb.motionQueue.AppendCubic(q,dq,duration);
  }
  else {
    return _sendMotionQueueCubic(LEFT,duration,angles,dangles,immediate) && _sendMotionQueueCubic(RIGHT,duration,angles+numLimbDofs,dangles+numLimbDofs,immediate);
  }
  return 1;
}
//no-lock version
BOOL _sendMotionQueueRamp(int limb,const double* angles,double speed,bool immediate)
{
  if(!gData.robotModel) return 0;
  if(limb == LEFT) {
    gData.SetLimbGovernor(LEFT,LimbState::MotionQueue);
    Config q(numLimbDofs);
    Copy(angles,q);
    if(speed != 1.0) {
      //modify limits
      Vector oldvmax=gData.robotState.leftLimb.motionQueue.velMax,oldamax=gData.robotState.leftLimb.motionQueue.accMax;
      gData.robotState.leftLimb.motionQueue.velMax *= speed;
      gData.robotState.leftLimb.motionQueue.accMax *= speed;
      if(immediate) gData.robotState.leftLimb.motionQueue.Cut(0);
      gData.robotState.leftLimb.motionQueue.AppendRamp(q);
      //restore limits
      gData.robotState.leftLimb.motionQueue.velMax = oldvmax;
      gData.robotState.leftLimb.motionQueue.accMax = oldamax;
    }
    else {
      if(immediate) gData.robotState.leftLimb.motionQueue.Cut(0);
      gData.robotState.leftLimb.motionQueue.AppendRamp(q);
    }
  }
  else if(limb == RIGHT) {
    gData.SetLimbGovernor(RIGHT,LimbState::MotionQueue);
    Config q(numLimbDofs);
    Copy(angles,q);
    if(speed != 1.0) {
      //modify limits
      Vector oldvmax=gData.robotState.rightLimb.motionQueue.velMax,oldamax=gData.robotState.rightLimb.motionQueue.accMax;
      gData.robotState.rightLimb.motionQueue.velMax *= speed;
      gData.robotState.rightLimb.motionQueue.accMax *= speed;
      if(immediate) gData.robotState.rightLimb.motionQueue.Cut(0);
      gData.robotState.rightLimb.motionQueue.AppendRamp(q);
      //restore limits
      gData.robotState.rightLimb.motionQueue.velMax = oldvmax;
      gData.robotState.rightLimb.motionQueue.accMax = oldamax;
    }
    else {
      if(immediate) gData.robotState.rightLimb.motionQueue.Cut(0);
      gData.robotState.rightLimb.motionQueue.AppendRamp(q);
    }
  }
  else {
    printf("sendMotionQueueRamp(): warning, can't ramp arms simultaneously yet\n");
    return _sendMotionQueueRamp(LEFT,angles,speed,immediate) && _sendMotionQueueRamp(RIGHT,angles+numLimbDofs,speed,immediate);
  }
  return 1;
}
//no-lock version
BOOL _sendMotionQueueLinearRamp(int limb,const double* angles,double speed,bool immediate)
{
  if(!gData.robotModel) return 0;
  if(limb == LEFT) {
    gData.SetLimbGovernor(LEFT,LimbState::MotionQueue);
    Config q(numLimbDofs);
    Copy(angles,q);
    if(speed != 1.0) {
      //modify limits
      Vector oldvmax=gData.robotState.leftLimb.motionQueue.velMax,oldamax=gData.robotState.leftLimb.motionQueue.accMax;
      gData.robotState.leftLimb.motionQueue.velMax *= speed;
      gData.robotState.leftLimb.motionQueue.accMax *= speed;
      if(immediate) gData.robotState.leftLimb.motionQueue.Cut(0);
      gData.robotState.leftLimb.motionQueue.AppendLinearRamp(q);
      //restore limits
      gData.robotState.leftLimb.motionQueue.velMax = oldvmax;
      gData.robotState.leftLimb.motionQueue.accMax = oldamax;
    }
    else {
      if(immediate) gData.robotState.leftLimb.motionQueue.Cut(0);
      gData.robotState.leftLimb.motionQueue.AppendLinearRamp(q);
    }
  }
  else if(limb == RIGHT) {
    gData.SetLimbGovernor(RIGHT,LimbState::MotionQueue);
    Config q(numLimbDofs);
    Copy(angles,q);
    if(speed != 1.0) {
      //modify limits
      Vector oldvmax=gData.robotState.rightLimb.motionQueue.velMax,oldamax=gData.robotState.rightLimb.motionQueue.accMax;
      gData.robotState.rightLimb.motionQueue.velMax *= speed;
      gData.robotState.rightLimb.motionQueue.accMax *= speed;
      if(immediate) gData.robotState.rightLimb.motionQueue.Cut(0);
      gData.robotState.rightLimb.motionQueue.AppendLinearRamp(q);
      //restore limits
      gData.robotState.rightLimb.motionQueue.velMax = oldvmax;
      gData.robotState.rightLimb.motionQueue.accMax = oldamax;
    }
    else {
      if(immediate) gData.robotState.rightLimb.motionQueue.Cut(0);
      gData.robotState.rightLimb.motionQueue.AppendLinearRamp(q);
    }
  }
  else {
    printf("sendMotionQueueLinearRamp(): warning, can't ramp arms simultaneously\n");
    return _sendMotionQueueLinearRamp(LEFT,angles,speed,immediate) && _sendMotionQueueLinearRamp(RIGHT,angles+numLimbDofs,speed,immediate);
  }
  return 1;
}
BOOL sendMotionQueueLinear(int limb,double duration,const double* angles)
{
  if(gData.enableCollisionChecking && gData.CheckCollisions(limb,Vector(NumLimbDofs(limb),angles))) return 0;
  ScopedLock lock(gData.mutex);
  return _sendMotionQueueLinear(limb,duration,angles,true);
}
BOOL sendMotionQueueCubic(int limb,double duration,const double* angles,const double* dangles)
{
  if(gData.enableCollisionChecking && gData.CheckCollisions(limb,Vector(NumLimbDofs(limb),angles))) return 0;
  ScopedLock lock(gData.mutex);
  return _sendMotionQueueCubic(limb,duration,angles,dangles,true);
}
BOOL sendMotionQueueRamp(int limb,const double* angles,double speed)
{
  if(gData.enableCollisionChecking && gData.CheckCollisions(limb,Vector(NumLimbDofs(limb),angles))) return 0;
  ScopedLock lock(gData.mutex);
  return _sendMotionQueueRamp(limb,angles,speed,true);
}
BOOL sendMotionQueueAppendLinear(int limb,double duration,const double* angles)
{
  if(gData.enableCollisionChecking && gData.CheckCollisions(limb,Vector(NumLimbDofs(limb),angles))) return 0;
  ScopedLock lock(gData.mutex);
  return _sendMotionQueueLinear(limb,duration,angles,false);  
}
BOOL sendMotionQueueAppendCubic(int limb,double duration,const double* angles,const double* dangles)
{
  if(gData.enableCollisionChecking && gData.CheckCollisions(limb,Vector(NumLimbDofs(limb),angles))) return 0;
  ScopedLock lock(gData.mutex);
  return _sendMotionQueueCubic(limb,duration,angles,dangles,false);  
}
///Appends the given milestone to the motion queue
BOOL sendMotionQueueAppendRamp(int limb,const double* angles,double speed)
{
  if(gData.enableCollisionChecking && gData.CheckCollisions(limb,Vector(NumLimbDofs(limb),angles))) return 0;
  ScopedLock lock(gData.mutex);
  return _sendMotionQueueRamp(limb,angles,speed,false);
}
///Sends a trajectory.  Milestones are given as an array of length numPoints*n,
///where n is the dimensionality of the limb.
BOOL sendMotionQueueTrajectory(int limb,int numPoints,const double* times,const double* milestones,const double* vmilestones)
{
  if(numPoints < 1) return 0;
  int size = NumLimbDofs(limb);
  if(gData.enableCollisionChecking) {
    //check collisions on the entire trajectory
    for(int i=0;i<numPoints;i++)
      if(gData.CheckCollisions(limb,Vector(size,milestones+i*size))) return 0;
  }
  ScopedLock lock(gData.mutex);
  if(vmilestones) {
    if(!_sendMotionQueueCubic(limb,times[0],milestones,vmilestones,true)) return 0;
    for(int i=1;i<numPoints;i++)
      _sendMotionQueueCubic(limb,times[i],milestones+i*size,vmilestones+i*size,false);
    return 1;
  }
  else {
    if(!_sendMotionQueueLinear(limb,times[0],milestones,true)) return 0;
    for(int i=1;i<numPoints;i++)
      _sendMotionQueueLinear(limb,times[i],milestones+i*size,false);
    return 1;
  }
}
BOOL sendMotionQueueAppendTrajectory(int limb,int numPoints,const double* times,const double* milestones,const double* vmilestones)
{
  if(numPoints <= 0) return 1;
  int size = NumLimbDofs(limb);
  if(gData.enableCollisionChecking) {
    //check collisions on the entire trajectory
    for(int i=0;i<numPoints;i++)
      if(gData.CheckCollisions(limb,Vector(size,milestones+i*size))) return 0;
  }
  ScopedLock lock(gData.mutex);
  if(vmilestones) {
    if(!_sendMotionQueueCubic(limb,times[0],milestones,vmilestones,false)) return 0;
    for(int i=1;i<numPoints;i++)
      _sendMotionQueueCubic(limb,times[i],milestones+i*size,vmilestones+i*size,false);
    return 1;
  }
  else {
    if(!_sendMotionQueueLinear(limb,times[0],milestones,false)) return 0;
    for(int i=1;i<numPoints;i++)
      _sendMotionQueueLinear(limb,times[i],milestones+i*size,false);
    return 1;
  }
}


///Appends the given milestone to the motion queue
BOOL sendMotionQueueAppendLinearRamp(int limb,const double* angles,double speed)
{
  if(gData.enableCollisionChecking && gData.CheckCollisions(limb,Vector(NumLimbDofs(limb),angles))) return 0;
  ScopedLock lock(gData.mutex);
  return _sendMotionQueueLinearRamp(limb,angles,speed,false);
}

////// LIMB PLANNER  ///////

///Returns true the if limb's motion queue is being controlled by the planner
BOOL isPlannerEnabled(int limb)
{
  return gData.planner.plannerActive[limb];
}
///Sends a pointer to a Klampt/Planning/PlannerObjective object to theplanner
APIENTRY BOOL sendPlannerObjective(void* pPlannerObjective)
{
  gData.mutex.lock();
  if(pPlannerObjective == NULL) {
    gData.SetLimbGovernor(BOTH,LimbState::Normal);
    gData.mutex.unlock();
    return 1;
  }
  if(!gData.robotModel) {
    printf("Cannot start planner, do not have a robot model\n");
    gData.mutex.unlock();
    return 0;
  }
  PlannerObjectiveBase* p = reinterpret_cast<PlannerObjectiveBase*>(pPlannerObjective);
  if(!gData.planner.planningThread.IsPlanning()) {
    //launch the planning thread
    gData.SetLimbGovernor(BOTH,LimbState::MotionQueue);
    while(Max(gData.robotState.leftLimb.motionQueue.TimeRemaining(),gData.robotState.rightLimb.motionQueue.TimeRemaining())>0) {
      double tsleep = Max(gData.robotState.leftLimb.motionQueue.TimeRemaining(),gData.robotState.rightLimb.motionQueue.TimeRemaining());
      gData.mutex.unlock();
      printf("Waiting %gs for current motion to quit before starting planner...\n",tsleep);
      ThreadSleep(tsleep);
      gData.mutex.lock();
    }
    gData.SetLimbGovernor(BOTH,LimbState::Planner);

    if(gData.planner.currentPlanner != PlannerState::IKPlanner) {
      gData.planner.planningThread.SetPlanner(new DynamicIKPlanner);
      gData.planner.planningThread.SetCSpace(gData.planner.cspace);
      gData.planner.currentPlanner = PlannerState::IKPlanner;
    }
  }
  gData.planner.planningThread.SetObjective(p);
  gData.mutex.unlock();
  return 1;
}
///Sends a string in JSON PlannerObjective format to the planner
APIENTRY BOOL sendPlannerObjectiveStr(const char* pPlannerObjective)
{
  if(pPlannerObjective == NULL || strlen(pPlannerObjective)==0) {
    sendPlannerObjective(NULL);
  }
  stringstream ss(pPlannerObjective);
  PlannerObjectiveBase* p = LoadPlannerObjective(ss,gData.robotModel);
  if(!p) {
    printf("Error parsing planner objective string %s\n",pPlannerObjective);
    return 0;
  }
  return sendPlannerObjective(p);
}
///Stops the planner
APIENTRY BOOL stopPlanner()
{
  if(gData.planner.planningThread.IsPlanning())
    gData.planner.planningThread.Stop();
  return 1;
}
///Returns the objective function obtained by the planner
APIENTRY double plannerObjectiveValue()
{
  gData.planner.planningThread.ObjectiveValue();
}
///Sets the world file used by the planner
BOOL setPlannerWorldFile(const char* worldFile)
{
  XmlWorld xmlWorld;
  if(!xmlWorld.Load(worldFile)) {
    printf("Error loading world file %s\n",worldFile);
    return 0;
  }
  if(!xmlWorld.GetWorld(gData.planner.world)) {
    printf("Error loading world from %s\n",worldFile);
    return 0;
  }
  gData.OnWorldChange();
  return 1;
}

///Adds an obstacle to the planner, returns its ID
int addPlannerObstacle(const char* obstacleFile)
{
  XmlWorld xmlWorld;
  const char* ext=FileExtension(obstacleFile);
  if(0==strcmp(ext,"env") || 0==strcmp(ext,"tri")) {
    if(gData.planner.world.LoadTerrain(obstacleFile)<0) {
      printf("Error loading terrain file %s\n",obstacleFile);
      return 0;
    }
  }
  else if(0==strcmp(ext,"obj")) {
    if(gData.planner.world.LoadRigidObject(obstacleFile)<0) {
      printf("Error loading rigid object file %s\n",obstacleFile);
      return 0;
    }
  }
  else if(0==strcmp(ext,"xml")) {
    if(!xmlWorld.Load(obstacleFile)) {
      printf("Error loading world file %s\n",obstacleFile);
      return 0;
    }
    if(!xmlWorld.GetWorld(gData.planner.world)) {
      printf("Error loading world from %s\n",obstacleFile);
      return 0;
    }
  }
  else {
    printf("Unknown file extension %s on file %s\n",ext,obstacleFile);
    return 0;
  }
  gData.OnWorldChange();
  return 1;
}
///Deletes an obstacle from the planner, given its id
int deletePlannerObstacle(int id)
{
  fprintf(stderr,"TODO: deletePlannerObstacle not implemented yet\n");
  return 0;
}

///Sets the collision avoidance margin used by the planner
BOOL setPlannerObstacleMargin(int id,double margin)
{
  if(id < 0 || id >= gData.planner.world.NumIDs()) {
    fprintf(stderr,"setPlannerObstacleMargin: id %d is invalid, valid range [0,%d]\n",id,gData.planner.world.NumIDs()-1);
    return 0;
  }
  gData.planner.world.GetGeometry(id)->margin = margin;
  return 1;
}
///Clears the world used by the planner
BOOL clearPlannerWorld()
{
  gData.planner.world.robots.resize(0);
  gData.planner.world.rigidObjects.resize(0);
  gData.planner.world.terrains.resize(0);
  gData.OnWorldChange();
  return 1;
}

BOOL enableCollisionChecking(BOOL enabled)
{
  gData.enableCollisionChecking = bool(enabled);
}







///Retrieves the number of DOFs in the Klamp't model.  setKlamptModel() must be called first.
APIENTRY int getKlamptNumDofs()
{
  if(!gData.robotModel) return -1;
  return gData.robotModel->q.n;
}
///Retrieves the sensed Klamp't configuration.  setKlamptModel() must be called first.
APIENTRY BOOL getKlamptSensedPosition(double* klamptConfig)
{
  if(!gData.robotModel) return 0;
  ScopedLock lock(gData.mutex);
  Config q;
  gData.GetKlamptSensedConfig(q);
  Copy(q,klamptConfig);
  return 1;
}
///Retrieves the sensed Klamp't velocity.  setKlamptModel() must be called first.
APIENTRY BOOL getKlamptSensedVelocity(double* klamptVelocity)
{
  if(!gData.robotModel) return 0;
  ScopedLock lock(gData.mutex);
  Config q;
  gData.GetKlamptSensedVelocity(q);
  Copy(q,klamptVelocity);
  return 1;
}
///Retrieves the commanded Klamp't configuration.  setKlamptModel() must be called first.
APIENTRY BOOL getKlamptCommandedPosition(double* klamptConfig)
{
  if(!gData.robotModel) return 0;
  ScopedLock lock(gData.mutex);
  Config q;
  gData.GetKlamptCommandedConfig(q);
  Copy(q,klamptConfig);
  return 1;
}
///Retrieves the commanded Klamp't velocity.  setKlamptModel() must be called first.
APIENTRY BOOL getKlamptCommandedVelocity(double* klamptVelocity)
{
  if(!gData.robotModel) return 0;
  ScopedLock lock(gData.mutex);
  Config q;
  gData.GetKlamptCommandedVelocity(q);
  Copy(q,klamptVelocity);
  return 1;
}
///Sends a velocity command with the given Klamp't velocities.  setKlamptModel() must be called first.
APIENTRY BOOL sendKlamptMoveVelocity(const double* klamptVelocity)
{
  if(!gData.robotModel) return 0;
  Config dqklampt(gData.robotModel->q.n);
  Copy(klamptVelocity,dqklampt);
  Vector dqlimb;
  Vector dbase;
  double headPan;
  gData.KlamptToLimb(dqklampt,BOTH,dqlimb);
  if(!sendLimbVelocity(BOTH,dqlimb)) return 0;
  dbase.resize(3);
  if(getKlamptMobileBase(klamptVelocity,&dbase[0],&dbase[1],&dbase[2])) {
    Vector dbaselocal(3);
    {
      ScopedLock lock(gData.mutex);
      gData.robotState.base.ToLocal(dbase,dbaselocal);
    }
    sendMobileBaseVelocity(dbaselocal[0],dbaselocal[1],dbase[2]);
  }
  int ngl = numGripperDofs(LEFT), ngr = numGripperDofs(RIGHT);
  Vector dgripper;
  dgripper.resize(ngl);
  if(getKlamptGripper(dqklampt,LEFT,dgripper)) {
    Vector dest(ngl),force(ngl,1.0);
    for(int i=0;i<ngl;i++) {
      if(dgripper[i] < 0) dest[i] = 0;
      else if(dgripper[i] > 0) dest[i] = 1;
      else dest[i] = gData.robotState.leftGripper.position[i];
    }
    sendSetGripper(LEFT,dest,dgripper,force);
  }
  dgripper.resize(ngr);
  if(getKlamptGripper(dqklampt,RIGHT,dgripper)) {
    Vector dest(ngr),force(ngr,1.0);
    for(int i=0;i<ngr;i++) {
      if(dgripper[i] < 0) dest[i] = 0;
      else if(dgripper[i] > 0) dest[i] = 1;
      else dest[i] = gData.robotState.rightGripper.position[i];
    }
    sendSetGripper(RIGHT,dest,dgripper,force);
  }
  return 1;
}
///Sends a smooth motion command to the given target configuration.  setKlamptModel() must be called first.
APIENTRY BOOL sendKlamptMoveToTarget(const double* klamptConfig)
{
  if(!gData.robotModel) return 0;
  Config qklampt(gData.robotModel->q.n);
  Copy(klamptConfig,qklampt);
  Vector qlimb;
  Vector base;
  double headPan;
  gData.KlamptToLimb(qklampt,BOTH,qlimb);
  if(!sendMotionQueueRamp(BOTH,qlimb)) return 0;
  base.resize(3);
  if(getKlamptMobileBase(klamptConfig,&base[0],&base[1],&base[2])) {
    sendMobileBasePosition(base[0],base[1],base[2]);
  }
  int ngl = numGripperDofs(LEFT), ngr = numGripperDofs(RIGHT);
  Vector gripper;
  gripper.resize(ngl);
  if(getKlamptGripper(qklampt,LEFT,gripper)) {
    Vector speed(ngl,1.0),force(ngl,1.0);
    sendSetGripper(LEFT,gripper,speed,force);
  }
  gripper.resize(ngr);
  if(getKlamptGripper(qklampt,RIGHT,gripper)) {
    Vector speed(ngr,1.0),force(ngr,1.0);
    sendSetGripper(RIGHT,gripper,speed,force);
  }
  return 1;
}
///Retrieves the limb indices for the Klampt model.  setKlamptModel() must be called first
APIENTRY BOOL getKlamptLimbIndices(int limb,int* indices)
{
  if(!gData.robotModel) return 0;
  if(limb == LEFT) copy(gData.leftKlamptIndices.begin(),gData.leftKlamptIndices.end(),indices);
  else if(limb == RIGHT) copy(gData.rightKlamptIndices.begin(),gData.rightKlamptIndices.end(),indices);
  else {
    copy(gData.leftKlamptIndices.begin(),gData.leftKlamptIndices.end(),indices);
    copy(gData.rightKlamptIndices.begin(),gData.rightKlamptIndices.end(),indices+numLimbDofs);
  }
  return 1;
}
///Retrieves the head pan index for the Klampt model, or -1 if setKlamptModel() was not called first.
APIENTRY int getKlamptHeadPanIndex()
{
  if(!gData.robotModel) return -1;
  return gData.headPanKlamptIndex;
}
///Retrieves the mobile base indices for the Klampt model, returns 0 if no base is defined or
///setKlamptModel() is not called first
APIENTRY BOOL getKlamptMobileBaseIndices(int* x,int* y,int* theta)
{
  if(gData.baseKlamptIndices.empty()) return 0;
  *x = gData.baseKlamptIndices[0];
  *y = gData.baseKlamptIndices[1];
  *theta = gData.baseKlamptIndices[2];
  return 1;
}
///Retrieves the gripper indices for the Klampt model, or -1 if the gripper is not defined or
///setKlamptModel() was not called first.
APIENTRY BOOL getKlamptGripperIndices(int limb,int* indices)
{
  if(limb == LEFT) {
    if(gData.leftGripperMap.name.empty()) return 0;
    for(size_t i=0;i<gData.leftGripperMap.klamptIndices.size();i++)
      indices[i] = gData.leftGripperMap.klamptIndices[i][0];
    return 1;
  }
  else if(limb == RIGHT) {
    if(gData.rightGripperMap.name.empty()) return 0;
    for(size_t i=0;i<gData.rightGripperMap.klamptIndices.size();i++)
      indices[i] = gData.rightGripperMap.klamptIndices[i][0];
    return 1;
  }
  else {
    int k=0;
    for(size_t i=0;i<gData.leftGripperMap.klamptIndices.size();i++,k++)
      indices[k] = gData.leftGripperMap.klamptIndices[i][0];
    for(size_t i=0;i<gData.rightGripperMap.klamptIndices.size();i++,k++)
      indices[k] = gData.rightGripperMap.klamptIndices[i][0];
    return 1;
  }
}
///Retrieves the limb configuration for a given Klamp't configuration.
///Also works for velocities.  setKlamptModel() must be called first.
APIENTRY BOOL getKlamptLimb(const double* klamptConfig,int limb,double* out)
{
  if(!gData.robotModel) return 0;
  Vector q(gData.robotModel->q.n),qlimb;
  Copy(klamptConfig,q);
  gData.KlamptToLimb(q,limb,qlimb);
  Copy(qlimb,out);
  return 1;
}
///Retrieves the head pan for a given Klamp't configuration.
///Also works for velocities.  setKlamptModel() must be called first.
APIENTRY BOOL getKlamptHeadPan(const double* klamptConfig,double* out)
{
  if(!gData.robotModel) return 0;
  *out = klamptConfig[gData.headPanKlamptIndex];
  return 1;
}
///Retrieves the mobile base coordinates for a given Klamp't configuration.
///Also works for velocities.  setKlamptModel() must be called first.
APIENTRY BOOL getKlamptMobileBase(const double* klamptConfig,double* x,double* y,double* theta)
{
  if(!gData.robotModel || gData.baseKlamptIndices.empty()) return 0;
  *x = klamptConfig[gData.baseKlamptIndices[0]];
  *y = klamptConfig[gData.baseKlamptIndices[1]];
  *theta = klamptConfig[gData.baseKlamptIndices[2]];
}
///Retrieves the gripper config for a given Klamp't configuration.
///Also works for velocities.  setKlamptModel() must be called first.
APIENTRY BOOL getKlamptGripper(const double* klamptConfig,int limb,double* out)
{
  if(limb == LEFT) {
    if(gData.leftGripperMap.name.empty()) return 0;
    for(size_t i=0;i<gData.leftGripperMap.klamptIndices.size();i++) 
      out[i] = gData.leftGripperMap.ConfigToFinger(klamptConfig,i);
    return 1;
  }
  else if(limb == RIGHT) {
    if(gData.rightGripperMap.name.empty()) return 0;
    for(size_t i=0;i<gData.rightGripperMap.klamptIndices.size();i++) 
      out[i] = gData.rightGripperMap.ConfigToFinger(klamptConfig,i);
    return 1;
  }
  else {
    int k=0;
    for(size_t i=0;i<gData.leftGripperMap.klamptIndices.size();i++,k++) 
      out[k] = gData.leftGripperMap.ConfigToFinger(klamptConfig,i);
    for(size_t i=0;i<gData.rightGripperMap.klamptIndices.size();i++,k++) 
      out[k] = gData.rightGripperMap.ConfigToFinger(klamptConfig,i);
    return 1;
  }
}
///Copies the limb configuration into a given Klamp't configuration.
///Also works for velocities.  setKlamptModel() must be called first.
APIENTRY BOOL setKlamptLimb(const double* limbConfig,int limb,double* klamptConfig)
{
  if(!gData.robotModel) return 0;
  Vector q(gData.robotModel->q.n,klamptConfig),qlimb(NumLimbDofs(limb),limbConfig);
  if(limb == LEFT || limb == RIGHT) {
    gData.LimbToKlampt(qlimb,limb,q);
    Copy(q,klamptConfig);
  }
  else {
    gData.LimbToKlampt(qlimb,limb,q);
    Copy(q,klamptConfig);
  }
  return 1;
}
///Copies the head pan into a given Klamp't configuration.
///Also works for velocities.  setKlamptModel() must be called first.
APIENTRY BOOL setKlamptHeadPan(double pan,double* klamptConfig)
{
  if(!gData.robotModel) return 0;
  klamptConfig[gData.headPanKlamptIndex] = pan;
  return 1;
}
///Copies the mobile base coordinates for a given Klamp't configuration.
///Also works for velocities.  setKlamptModel() must be called first.
APIENTRY BOOL setKlamptMobileBase(double x,double y,double theta,double* klamptConfig)
{
  if(!gData.robotModel || gData.baseKlamptIndices.empty()) return 0;  
  klamptConfig[gData.baseKlamptIndices[0]] = x;
  klamptConfig[gData.baseKlamptIndices[1]] = y;
  klamptConfig[gData.baseKlamptIndices[2]] = theta;
  return 1;
}
///Copies the gripper configuration for a given Klamp't configuration.
///Also works for velocities.  setKlamptModel() must be called first.
APIENTRY BOOL setKlamptGripper(const double* gripperConfig,int limb,double* klamptConfig)
{
  if(limb == LEFT) {
    if(gData.leftGripperMap.name.empty()) return 0;
    for(size_t i=0;i<gData.leftGripperMap.klamptIndices.size();i++) 
      gData.leftGripperMap.FingerToConfig(gripperConfig[i],i,klamptConfig);
    return 1;
  }
  else if(limb == RIGHT) {
    if(gData.rightGripperMap.name.empty()) return 0;
    for(size_t i=0;i<gData.rightGripperMap.klamptIndices.size();i++) 
      gData.rightGripperMap.FingerToConfig(gripperConfig[i],i,klamptConfig);
    return 1;
  }
  else {
    int k=0;
    for(size_t i=0;i<gData.leftGripperMap.klamptIndices.size();i++,k++) 
      gData.leftGripperMap.FingerToConfig(gripperConfig[k],i,klamptConfig);
    for(size_t i=0;i<gData.rightGripperMap.klamptIndices.size();i++,k++) 
      gData.rightGripperMap.FingerToConfig(gripperConfig[k],i,klamptConfig);
    return 1;
  }
}
