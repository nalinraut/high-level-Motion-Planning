#include "motion_state.h"
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <baxter_core_msgs/AssemblyState.h>
#include <sensor_msgs/JointState.h>
#include <baxter_core_msgs/HeadState.h>
#include <baxter_core_msgs/EndEffectorState.h>
#include <baxter_core_msgs/JointCommand.h>
#include <baxter_core_msgs/HeadPanCommand.h>
#include <baxter_core_msgs/EndEffectorCommand.h>
#include <baxter_core_msgs/DigitalIOState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <reflex_msgs/Command.h>
#include <reflex_msgs/Hand.h>
#include <reflex_msgs/PoseCommand.h>
#include <reflex_msgs/VelocityCommand.h>
#include <ros/callback_queue.h>


#define MOTOR_COUNT 4 //number of motors for the gripper
#define TOLERANCE 0 //tolerance for motor velocity to consider a hand moving.

const static double reflex_offset [MOTOR_COUNT] = {3,3,3,1.5};
const static double reflex_scaling [MOTOR_COUNT] = {-3,-3,-3,-1.5};
//const static double reflex_offset [MOTOR_COUNT] = { 0,0,0,0};
//const static double reflex_scaling [MOTOR_COUNT] = { 4.5, 4.5, 4.5, 2.5 };

//bool New_left_reflex=false; bool New_right_reflex=false;

//IMPORTANT: turn this to 1 if you are doing base calibration
#define BASE_RAW_COMMANDS 1



#if BASE_RAW_COMMANDS
const static double base_x_scaling = 1;
const static double base_y_scaling = 1;
const static double base_theta_scaling = 1;
#else
//these were calibrated using base_calibrate.csv
const static double base_x_scaling = 1.5*0.81;
const static double base_y_scaling = 1.5*0.97;
const static double base_theta_scaling = 1.15;
#endif //BASE_RAW_COMMANDS
//if the heading is given in CCW angles, set this to 1.0
//otherwise, set this to -1.0
const static double base_heading_flip = 1.0;

//50 Hz
const double g_send_command_delay = 0.02;

// Class template for topic subscribers that save messages to a Stateful class
// and move on. Algorithm handles it on its own loop instead of with callbacks
template <class Msg>
class AutoMsg
{
public:
  AutoMsg(): msgCount(0),lastReadMsg(0) {}
  void subscribe(ros::NodeHandle& nh,const char* topic,int queueSize = 100) {
    this->topic = topic;
    sub = nh.subscribe(topic,queueSize,&AutoMsg<Msg>::callback, this);
  }
  void unsubscribe() {
    this->topic = "";
    sub = ros::Subscriber();
  }
  ///Use this or numUnread() to tell if a new message arrived since the last read
  bool changed() const { return msgCount != lastReadMsg; }
  ///Use this to tell how many new messages arrived since the last read
  int numUnread() const { return msgCount - lastReadMsg; }
  ///Use this to tell if a new message arrived
  void markRead() { lastReadMsg = msgCount; }

  void callback(const Msg& msg) {
    value = msg; msgCount++;
  }
  ///Returns true if a message arrived within before timeout
  ///seconds have elapsed, false if not.  pollFreq gives the polling
  ///frequency in Hz.  If targetCount < 0, this just waits for the next message.
  ///If it is >= 0 this waits until msgCount >= targetCount.
  ///If killPtr is given, the bool that it points to is a termination flag.
  ///If it is set to true by some external thread, then this waiting thread is
  ///killed.
  bool waitForMessage(double timeout,double pollFreq = 10,int targetCount=-1,bool* killPtr = NULL,const char* message=NULL) {
    if(targetCount < 0) targetCount = msgCount+1;
    bool nokill = false;
    if(killPtr == NULL) killPtr = &nokill;
    double t = 0;
    ros::Rate r(pollFreq);
    while((t < timeout) && !(*killPtr) && ros::ok()) {
      if(msgCount >= targetCount) {
	return true;
      }
      ros::spinOnce();
      if(message) printf("%s (skipping in %gs)\n",message,timeout-t);
      r.sleep();
      t += 1.0/pollFreq;
    }
    return false;
  }
  operator const Msg&  () const { return value; }
  operator Msg& () { return value; }
  const Msg* operator ->  () const { return &value; }
  Msg* operator -> () { return &value; }

  ros::Subscriber sub;
  string topic;
  Msg value;
  int msgCount;
  int lastReadMsg;
};

const char* baxter_left_limb_names [] = { "left_s0","left_s1","left_e0","left_e1","left_w0","left_w1","left_w2","left_w3"};
const char* baxter_right_limb_names [] = { "right_s0","right_s1","right_e0","right_e1","right_w0","right_w1","right_w2","right_w3"};

class MyControllerUpdateData : public ControllerUpdateData
{
public:
  //ROS stuff
  SmartPointer<ros::NodeHandle> nh;
  ros::Publisher larm_pub,rarm_pub,head_pan_pub,head_nod_pub,left_gripper_pub,right_gripper_pub,base_twist_pub;
  double last_send_time;
  AutoMsg<baxter_core_msgs::AssemblyState> assemblyState;
  AutoMsg<sensor_msgs::JointState> jointState;
  AutoMsg<baxter_core_msgs::HeadState> headState;
  AutoMsg<baxter_core_msgs::EndEffectorState> leftGripperState,rightGripperState;
  AutoMsg<reflex_msgs::Hand> leftReFlexGripper_State,rightReFlexGripper_State;
  AutoMsg<baxter_core_msgs::DigitalIOState> leftCuffState,rightCuffState;
  AutoMsg<nav_msgs::Odometry> baseState;
  Config lastBaseDifferencePos;
  double lastBaseDifferenceTime;

  //other physical robot state
  bool baxterEstop;

  map<string,int> baxter_left_limb_map,baxter_right_limb_map;

  MyControllerUpdateData()
    :last_send_time(-1),baxterEstop(false)
  {}

  virtual bool MyStartup();
  virtual bool IsHandMoving(AutoMsg<reflex_msgs::Hand> gripper);
  virtual bool MyProcessSensors();
  virtual void MySendCommands();
  virtual void MyShutdown();
};

#include "motion_common.h"


bool MyControllerUpdateData::MyStartup() {
  if(robotModel) {
    if(!GetKlamptIndices()) {
      // printf("Will proceed without some Klamp't indices\n");
    }
  }
  char* argv [] = { "apc_motion" };
  int argc=1;
  //ros::init(argc, &argv[0], "apc_motion", ros::init_options::NoSigintHandler);
  ros::init(argc, &argv[0], "apc_motion");

  //set up calibration information here
  baseCalibration.timeDelay = 0.2;

  //set up map from ros names to indices
  for(int i=0;i<numLimbDofs;i++)
    baxter_left_limb_map[baxter_left_limb_names[i]]=i;
  for(int i=0;i<numLimbDofs;i++)
    baxter_right_limb_map[baxter_right_limb_names[i]]=i;

  nh = new ros::NodeHandle();
  larm_pub = nh->advertise<baxter_core_msgs::JointCommand>(string("robot/limb/left/joint_command"),10,false);
  rarm_pub = nh->advertise<baxter_core_msgs::JointCommand>(string("robot/limb/right/joint_command"),10,false);
  head_pan_pub = nh->advertise<baxter_core_msgs::HeadPanCommand>(string("robot/head/command_head_pan"),10,false);
  head_nod_pub = nh->advertise<std_msgs::Bool>("robot/head/command_head_nod",10,false);
  base_twist_pub = nh->advertise<geometry_msgs::Twist>("hstar_amp1/base/auto/cmd/ts",10,false);

  assemblyState.subscribe(*nh,"robot/state",10);
  jointState.subscribe(*nh,"robot/joint_states",10);
  headState.subscribe(*nh,"robot/head/head_state",10);
  leftCuffState->state = 0;
  rightCuffState->state = 0;
  leftCuffState.subscribe(*nh,"robot/digital_io/left_lower_cuff/state",10);
  rightCuffState.subscribe(*nh,"robot/digital_io/right_lower_cuff/state",10);
  leftGripperState.subscribe(*nh,"robot/end_effector/left_gripper/state",10);
  rightGripperState.subscribe(*nh,"robot/end_effector/right_gripper/state",10);

  //try to subscribe to ReFlex hand status messages
  //TODO: PARAMETERIZE hand names

  const string leftPrefix = "left_hand";
  const string rightPrefix = "right_hand";
  leftReFlexGripper_State.subscribe(*nh,(leftPrefix+string("/hand_state")).c_str(),10);
  rightReFlexGripper_State.subscribe(*nh,(rightPrefix+string("/hand_state")).c_str(),10);
  baseState.subscribe(*nh,"hstar_amp1/odom");

  //DO STARTUP
  //TODO:: PARAMETERIZE all node names, paths
  ros::Publisher enabler = nh->advertise<std_msgs::Bool>("robot/set_super_enable",10);
  ros::Publisher resetter = nh->advertise<std_msgs::Empty>("robot/set_super_reset",10);
  ros::Publisher baseCmdMux = nh->advertise<std_msgs::String>("hstar_amp/base/cmd/mode",10);
  //wait for robot to be enabled
  robotState.baxterEnabled = false;
  while(true) {
    double maxWait = 2.0;
    double waitRate = 10.0;
    if(!assemblyState.waitForMessage(maxWait,waitRate,-1,&kill,"Waiting for Baxter state messages...")) {
      printf("No longer waiting for Baxter, timeout met\n");
      robotState.baxterEnabled = false;
      break;
    }
    else {
      if(assemblyState->enabled) {
	if(assemblyState->stopped) {
	  printf("Baxter ESTOP is on!  Turn off EStop and manually reset.\n");
	  resetter.publish(std_msgs::Empty());
	}
	else {
	  // printf("Baxter is enabled and ready to run\n");
	  robotState.baxterEnabled = true;
	  break;
	}
      }
      else {
	std_msgs::Bool msg;
	msg.data = true;
	enabler.publish(msg);
	printf("Waiting for Baxter to be enabled...\n");
      }
    }
  }
  if(kill || !ros::ok()) return false;
  //wait for robot joint and head messages
  if(robotState.baxterEnabled) {
    double timeout = 30;
    double rate = 10;
    if(!jointState.waitForMessage(timeout,rate,0,&kill,"Waiting for joint state update message..."))
      robotState.baxterEnabled = false;
    else if(!headState.waitForMessage(timeout,rate,0,&kill,"Waiting for head state  update message..."))
      robotState.baxterEnabled = false;
    else {
      MyProcessSensors();
    }
  }
  if(kill || !ros::ok()) return false;
//TODO: instead of piping Baxter topics to ReFlex topics, remap topics!
  //test for gripper connection and enable / calibrate if available
  int timeout = 10;
  while((timeout--)>0 && !kill && ros::ok()) {
    if(leftGripperState.msgCount > 0 && robotState.leftGripper.enabled == 0) {
      if(leftGripperState->enabled == 1) {
	if(leftGripperState->calibrated == 2) //not attached
	  robotState.leftGripper.enabled = 0;
	else {
	  robotState.leftGripper.enabled = 1;
	  robotState.leftGripper.name = "Rethink Electric Gripper";
	  robotState.leftGripper.positionCommand.resize(1);
	  robotState.leftGripper.speedCommand.resize(1,0);
	  robotState.leftGripper.forceCommand.resize(1,0);
	  robotState.leftGripper.positionCommand[0] = leftGripperState->position/100.0;
	  left_gripper_pub = nh->advertise<baxter_core_msgs::EndEffectorCommand>("robot/end_effector/left_gripper/command",10,false);
	  if(leftGripperState->calibrated != 1) {
	    baxter_core_msgs::EndEffectorCommand msg;
	    msg.id = rightGripperState->id;
	    msg.command = "clear_calibration";
	    left_gripper_pub.publish(msg);
	    ros::spinOnce();
	    ThreadSleep(0.5);

	    msg.id = leftGripperState->id;
	    msg.command = "calibrate";
	    left_gripper_pub.publish(msg);
	    printf("Sending left gripper calibrate message\n");
	  }
	}
      }
      else {
	robotState.leftGripper.enabled = 0;
      }
    }
    if(rightGripperState.msgCount > 0 && robotState.rightGripper.enabled == 0) {
      if(rightGripperState->enabled == 1) {
	if(rightGripperState->calibrated == 2) { //not attached
	  robotState.rightGripper.enabled = 0;
	}
	else {
	  robotState.rightGripper.enabled = 1;
	  robotState.rightGripper.name = "Rethink Electric Gripper";
	  robotState.rightGripper.positionCommand.resize(1,0);
	  robotState.rightGripper.speedCommand.resize(1,0);
	  robotState.rightGripper.forceCommand.resize(1,0);
	  robotState.rightGripper.positionCommand[0] = rightGripperState->position/100.0;
	  right_gripper_pub = nh->advertise<baxter_core_msgs::EndEffectorCommand>("robot/end_effector/right_gripper/command",10,false);
	  if(rightGripperState->calibrated != 1) {
	    baxter_core_msgs::EndEffectorCommand msg;
	    msg.id = rightGripperState->id;
	    msg.command = "clear_calibration";
	    right_gripper_pub.publish(msg);
	    ros::spinOnce();
	    ThreadSleep(0.5);

	    msg.id = rightGripperState->id;
	    msg.command = "calibrate";
	    right_gripper_pub.publish(msg);
	    printf("Sending right gripper calibrate message\n");
	  }
	}
      }
      else {
	robotState.rightGripper.enabled = 0;
      }
    }
     if(leftReFlexGripper_State.msgCount > 0) {
      printf("Got Reflex gripper on left hand\n");
      robotState.leftGripper.enabled = 1;
      robotState.leftGripper.name = "ReFlex";
      robotState.leftGripper.positionCommand.resize(MOTOR_COUNT,0);
      robotState.leftGripper.speedCommand.resize(MOTOR_COUNT,0);
      robotState.leftGripper.forceCommand.resize(MOTOR_COUNT,0);
      for(int i=0;i<MOTOR_COUNT;i++)
	robotState.leftGripper.positionCommand[i] = (leftReFlexGripper_State->motor[i].joint_angle-reflex_offset[i])/reflex_scaling[i];
      left_gripper_pub = nh->advertise<reflex_msgs::PoseCommand>(leftPrefix+string("/command_position").c_str(),10,false);
      //unsubscribe from Baxter messages
      leftGripperState.unsubscribe();
    }
    if(rightReFlexGripper_State.msgCount > 0) {
       printf("Got Reflex gripper on right hand\n");
      robotState.rightGripper.enabled = 1;
      robotState.rightGripper.name = "ReFlex";
      robotState.rightGripper.positionCommand.resize(MOTOR_COUNT,0);
      robotState.rightGripper.speedCommand.resize(MOTOR_COUNT,0);
      robotState.rightGripper.forceCommand.resize(MOTOR_COUNT,0);
      for(int i=0;i<MOTOR_COUNT;i++)
	robotState.rightGripper.positionCommand[i] = (rightReFlexGripper_State->motor[i].joint_angle-reflex_offset[i])/reflex_scaling[i];
      right_gripper_pub = nh->advertise<reflex_msgs::PoseCommand>(rightPrefix+string("/command_position").c_str(),2,false);
//unsubscribe from Baxter messages
      rightGripperState.unsubscribe();
    }
 if(leftReFlexGripper_State.msgCount > 0 && rightReFlexGripper_State.msgCount > 0) {
      //done waiting for grippers
      printf("Done waiting for grippers (got new reflex messages)\n");
      break;
    }


if(leftGripperState.msgCount > 0 && rightGripperState.msgCount > 0) {
      if((leftGripperState->enabled == 0 || leftGripperState->calibrated==2 || leftGripperState->calibrated == 1) && (rightGripperState->enabled == 0 || rightGripperState->calibrated==2 || rightGripperState->calibrated == 1)) {
	//done waiting for grippers
	// printf("Done waiting for grippers\n");
	break;
      }
    }
    printf("Waiting for gripper messages and gripper calibration... (%d steps left)\n",timeout);
    if(leftGripperState.msgCount > 0)
      printf("Left gripper enabled: %d, calibrated %d\n",leftGripperState->enabled,leftGripperState->calibrated);
    if(rightGripperState.msgCount > 0)
      printf("Right gripper enabled: %d, calibrated %d\n",rightGripperState->enabled,rightGripperState->calibrated);
    ros::spinOnce();
    ThreadSleep(0.1);
  }
  if(kill || !ros::ok()) return false;

  //test for base connection and enable if available
  if(baseState.waitForMessage(0.5,10,1,&kill,"Waiting for base messages...")) {
    robotState.base.enabled = 1;
    assert(baseState.changed());

    //set up velocity differencer (HStar AMP-1 velocities aren't reliable)
    lastBaseDifferencePos.resize(3,0.0);
    lastBaseDifferenceTime = ros::Time::now().toSec();
    MyProcessSensors();

    //set command MUX to auto
    std_msgs::String msg;
    msg.data = "auto";
    baseCmdMux.publish(msg);
    ros::spinOnce();
    ThreadSleep(0.1);
  }
  else {
    assert(robotState.base.enabled==0);
    assert(!baseState.changed());
  }
  if(robotState.base.enabled)
    printf("Mobile base is enabled.\n");
  else
    printf("Mobile base is not enabled.\n");
  return true;
}

bool MyControllerUpdateData::IsHandMoving(AutoMsg<reflex_msgs::Hand> gripper){
  bool is_moving = false;
  for(int i=0;i<MOTOR_COUNT;i++)
    is_moving|=(abs(gripper->motor[i].velocity)<=TOLERANCE);

  return is_moving;
}

bool MyControllerUpdateData::MyProcessSensors()
{
  bool processed = false;
  //ros::spinOnce();
  ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.01));
  if(!ros::ok()) {
    kill = true;
    return false;
  }
  if(baxterEstop) {
    kill = true;
    return false;
  }
  ScopedLock lock(mutex);
  if(baseState.changed() && robotState.base.enabled) {
    robotState.base.odometry.resize(3);
    robotState.base.odometry(0) = baseState->pose.pose.position.y;
    robotState.base.odometry(1) = -baseState->pose.pose.position.x;
    robotState.base.odometry(2) = base_heading_flip*2.0*Atan2(baseState->pose.pose.orientation.z,baseState->pose.pose.orientation.w);
    //do differencing for velocity since the odometry twist is meaningless
    if(robotState.base.velocity.empty())
      robotState.base.velocity.resize(3,0.0);
    double t = ros::Time::now().toSec();
    if(t - lastBaseDifferenceTime >= 0.1)  {
      double dt = t - lastBaseDifferenceTime;
      double c = Cos(lastBaseDifferencePos(2));
      double s = Sin(lastBaseDifferencePos(2));
      //transform global to local difference
      Vector gdiff = (robotState.base.odometry-lastBaseDifferencePos);
      robotState.base.velocity(0) = (c*gdiff(0)+s*gdiff(1))/dt;
      robotState.base.velocity(1) = (-s*gdiff(0)+c*gdiff(1))/dt;
      robotState.base.velocity(2) = gdiff(2)/dt;
      //cout<<"Updating differenced velocity to "<<robotState.base.velocity<<endl;
      lastBaseDifferencePos = robotState.base.odometry;
      lastBaseDifferenceTime = t;
    }
    if(robotState.base.velocity.norm() > 1e-3 || robotState.base.sendCommand)
      robotState.base.moving = true;
    else
      robotState.base.moving = false;
  }
  if(assemblyState.changed()) {
    baxterEstop = assemblyState->stopped;
    assemblyState.markRead();
  }
  if(jointState.changed()) {
    //update left and right arm states
    jointState.markRead();
    robotState.leftLimb.sensedConfig.resize(numLimbDofs);
    robotState.leftLimb.sensedVelocity.resize(numLimbDofs);
    robotState.leftLimb.sensedEffort.resize(numLimbDofs);
    robotState.rightLimb.sensedConfig.resize(numLimbDofs);
    robotState.rightLimb.sensedVelocity.resize(numLimbDofs);
    robotState.rightLimb.sensedEffort.resize(numLimbDofs);
    robotState.leftLimb.senseUpdateTime = t;
    robotState.rightLimb.senseUpdateTime = t;
    for(size_t i=0;i<jointState->name.size();i++) {
      if(baxter_left_limb_map.count(jointState->name[i]) != 0) {
	int k=baxter_left_limb_map[jointState->name[i]];
	robotState.leftLimb.sensedConfig[k] = jointState->position[i];
	robotState.leftLimb.sensedVelocity[k] = jointState->velocity[i];
	robotState.leftLimb.sensedEffort[k] = jointState->effort[i];
      }
      else if(baxter_right_limb_map.count(jointState->name[i]) != 0) {
	int k=baxter_right_limb_map[jointState->name[i]];
	robotState.rightLimb.sensedConfig[k] = jointState->position[i];
	robotState.rightLimb.sensedVelocity[k] = jointState->velocity[i];
	robotState.rightLimb.sensedEffort[k] = jointState->effort[i];
      }
    }
    if(robotState.leftLimb.commandedConfig.empty() || leftCuffState->state==1 /*startup or cuff pressed, reset commanded*/) {
      SetLimbGovernor(LEFT,LimbState::Normal);
      robotState.leftLimb.controlMode = LimbState::POSITION;
      robotState.leftLimb.sendCommand = false;
      robotState.leftLimb.commandedConfig = robotState.leftLimb.sensedConfig;
      robotState.leftLimb.commandedVelocity = robotState.leftLimb.sensedVelocity;
      robotState.leftLimb.commandedEffort = robotState.leftLimb.sensedEffort;
    }
    if(robotState.rightLimb.commandedConfig.empty() || rightCuffState->state==1 /*startup or cuff pressed, reset commanded*/) {
      SetLimbGovernor(RIGHT,LimbState::Normal);
      robotState.rightLimb.controlMode = LimbState::POSITION;
      robotState.rightLimb.sendCommand = false;
      robotState.rightLimb.commandedConfig = robotState.rightLimb.sensedConfig;
      robotState.rightLimb.commandedVelocity = robotState.rightLimb.sensedVelocity;
      robotState.rightLimb.commandedEffort = robotState.rightLimb.sensedEffort;
    }
  }
  if(headState.changed()) {
    //update head states
    robotState.head.pan = headState->pan;
    robotState.head.isNodding = headState->isNodding;
    headState.markRead();
  }
  if(leftGripperState.changed()) {
    //update gripper states
    if(leftGripperState->moving==1)
      robotState.leftGripper.moving = true;
    else
      robotState.leftGripper.moving = false;
    if(leftGripperState->enabled && leftGripperState->calibrated==1)
      robotState.leftGripper.enabled = true;
    else
      robotState.leftGripper.enabled = false;
    robotState.leftGripper.position.resize(1);
    robotState.leftGripper.force.resize(1);
    robotState.leftGripper.position[0] = leftGripperState->position/100.0;
    robotState.leftGripper.force[0] = leftGripperState->force/100.0;
    leftGripperState.markRead();
  }
  if(rightGripperState.changed()) {
    //update gripper states
    if(rightGripperState->moving==1)
      robotState.rightGripper.moving = true;
    else
      robotState.rightGripper.moving = false;
    if(rightGripperState->enabled && rightGripperState->calibrated==1)
      robotState.rightGripper.enabled = true;
    else
      robotState.rightGripper.enabled = false;
    robotState.rightGripper.position.resize(1);
    robotState.rightGripper.force.resize(1);
    robotState.rightGripper.position[0] = rightGripperState->position/100.0;
    robotState.rightGripper.force[0] = rightGripperState->force/100.0;
    rightGripperState.markRead();
  }

 if(leftReFlexGripper_State.changed())  {
    robotState.leftGripper.moving = this->IsHandMoving(leftReFlexGripper_State);
    robotState.leftGripper.position.resize(MOTOR_COUNT);
    //robotState.leftGripper.speed.resize(MOTOR_COUNT,0);
    robotState.leftGripper.force.resize(MOTOR_COUNT);
    for(int i=0;i<MOTOR_COUNT;i++) {
      robotState.leftGripper.position[i] = (leftReFlexGripper_State->motor[i].joint_angle-reflex_offset[i])/reflex_scaling[i];
      //robotState.leftGripper.speed[i] = leftReflexGripperState->velocity[i];
      robotState.leftGripper.force[i] = leftReFlexGripper_State->motor[i].load;
    }
    leftReFlexGripper_State.markRead();
  }
  if(rightReFlexGripper_State.changed())  {
    robotState.rightGripper.moving = this->IsHandMoving(rightReFlexGripper_State);
    robotState.rightGripper.position.resize(MOTOR_COUNT);
    //robotState.rightGripper.speed.resize(MOTOR_COUNT,0);
    robotState.rightGripper.force.resize(MOTOR_COUNT);
    for(int i=0;i<MOTOR_COUNT;i++) {
      robotState.rightGripper.position[i] = (rightReFlexGripper_State->motor[i].joint_angle-reflex_offset[i])/reflex_scaling[i];
      //robotState.rightGripper.speed[i] = rightReflexGripperState->velocity[i];
      robotState.rightGripper.force[i] = rightReFlexGripper_State->motor[i].load;
     //robotState.rightGripper.position[i] = (rightReflexGripperState->current_pos[i]-reflex_offset[i])/reflex_scaling[i];
      //robotState.rightGripper.speed[i] = rightReflexGripperState->velocity[i];
     // robotState.rightGripper.force[i] = rightReflexGripperState->load[i];
 }
    rightReFlexGripper_State.markRead();
  }
  return true;
}

void MyControllerUpdateData::MySendCommands()
{
  double dt = t-last_send_time;
  //control rate here
  if(t - last_send_time < g_send_command_delay) return;
  last_send_time = t;
  if(robotState.leftLimb.sendCommand || !robotState.leftLimb.rawCommandToSend.empty()) {
    baxter_core_msgs::JointCommand msg;
    msg.names.resize(numLimbDofs);
    for(int i=0;i<numLimbDofs;i++) msg.names[i] = baxter_left_limb_names[i];
    mutex.lock();
    const Config* x=NULL;
    if(robotState.leftLimb.controlMode == LimbState::POSITION) {
      //x=&robotState.leftLimb.commandedConfig;
      x=&robotState.leftLimb.rawCommandToSend;
      msg.mode = baxter_core_msgs::JointCommand::POSITION_MODE;
    }
    else if(robotState.leftLimb.controlMode == LimbState::RAW_POSITION) {
      //x=&robotState.leftLimb.commandedConfig;
      x=&robotState.leftLimb.rawCommandToSend;
      msg.mode = baxter_core_msgs::JointCommand::RAW_POSITION_MODE;
    }
    else if(robotState.leftLimb.controlMode == LimbState::VELOCITY) {
      //x=&robotState.leftLimb.commandedVelocity;
      x=&robotState.leftLimb.rawCommandToSend;
      msg.mode = baxter_core_msgs::JointCommand::VELOCITY_MODE;
    }
    else if(robotState.leftLimb.controlMode == LimbState::EFFORT) {
      x=&robotState.leftLimb.commandedEffort;
      //update command feedback from sensors??
      robotState.leftLimb.commandedConfig = robotState.leftLimb.sensedConfig;
      robotState.leftLimb.commandedVelocity = robotState.leftLimb.sensedVelocity;
      msg.mode = baxter_core_msgs::JointCommand::TORQUE_MODE;
    }
    robotState.leftLimb.sendCommand = false;

    if(x) {
      msg.command.resize(x->n);
      for(int i=0;i<x->n;i++)
	msg.command[i] = (*x)(i);
      mutex.unlock();
      larm_pub.publish(msg);
    }
    else
      mutex.unlock();
  }
  if(robotState.rightLimb.sendCommand  || !robotState.rightLimb.rawCommandToSend.empty()) {
    baxter_core_msgs::JointCommand msg;
    msg.names.resize(numLimbDofs);
    for(int i=0;i<numLimbDofs;i++) msg.names[i] = baxter_right_limb_names[i];

    mutex.lock();
    const Config* x=NULL;
    if(robotState.rightLimb.controlMode == LimbState::POSITION) {
      //x=&robotState.rightLimb.commandedConfig;
      x=&robotState.rightLimb.rawCommandToSend;
      msg.mode = baxter_core_msgs::JointCommand::POSITION_MODE;
    }
    else if(robotState.rightLimb.controlMode == LimbState::RAW_POSITION) {
      //x=&robotState.rightLimb.commandedConfig;
      x=&robotState.rightLimb.rawCommandToSend;
      msg.mode = baxter_core_msgs::JointCommand::RAW_POSITION_MODE;
    }
    else if(robotState.rightLimb.controlMode == LimbState::VELOCITY) {
      //x=&robotState.rightLimb.commandedVelocity;
      x=&robotState.rightLimb.rawCommandToSend;
      msg.mode = baxter_core_msgs::JointCommand::VELOCITY_MODE;
    }
    else if(robotState.rightLimb.controlMode == LimbState::EFFORT) {
      x=&robotState.rightLimb.commandedEffort;
      //update command feedback from sensors??
      robotState.rightLimb.commandedConfig = robotState.rightLimb.sensedConfig;
      robotState.rightLimb.commandedVelocity = robotState.rightLimb.sensedVelocity;
      msg.mode = baxter_core_msgs::JointCommand::TORQUE_MODE;
    }
    robotState.rightLimb.sendCommand = false;
    if(x) {
      msg.command.resize(x->n);
      for(int i=0;i<x->n;i++)
	msg.command[i] = (*x)(i);
      mutex.unlock();
      rarm_pub.publish(msg);
    }
    else
      mutex.unlock();
  }
  if(robotState.head.sendCommand) {
    ScopedLock lock(mutex);
    baxter_core_msgs::HeadPanCommand msg;
    msg.target = robotState.head.panTarget;
    msg.speed_ratio = double(robotState.head.panSpeed);
    head_pan_pub.publish(msg);
    robotState.head.sendCommand = false;
  }
  if(robotState.leftGripper.sendCommand) {
    if(robotState.leftGripper.position.size()==1) { //electric gripper
      ScopedLock lock(mutex);
      ///TODO: gripper force and velocity control
      baxter_core_msgs::EndEffectorCommand msg;
      msg.id = leftGripperState->id;
      {
	stringstream ss;
	ss<<"{\"velocity\":"<<robotState.leftGripper.speedCommand[0]*100<<",\"moving_force\":"<<robotState.leftGripper.forceCommand[0]*110<<",\"holding_force\":"<<robotState.leftGripper.forceCommand[0]*100<<",\"dead_zone\": 5.0 }";
	msg.command = "configure";
	msg.args = ss.str();
	//left_gripper_pub.publish(msg);
      }
      {
	msg.command = "go";
	stringstream ss;
	ss<<"{\"position\":"<<robotState.leftGripper.positionCommand[0]*100<<"}";

	msg.args = ss.str();
	left_gripper_pub.publish(msg);
      }
    }
    else if(robotState.leftGripper.position.size()==MOTOR_COUNT) {  //reflex gripper
      ScopedLock lock(mutex);
      //Right now is only using position controll. If you want to use position and velocity control, please change the left_gripper_pub to publish with hand/command
      reflex_msgs::Command cmd;
      reflex_msgs::PoseCommand pose_cmd;
      reflex_msgs::VelocityCommand v_cmd;
      //printf("motion_physical:LEFT preshape is: %f\n",robotState.leftGripper.positionCommand[3]);
      pose_cmd.f1=reflex_offset[0]+reflex_scaling[0]*robotState.leftGripper.positionCommand[0];
      pose_cmd.f2=reflex_offset[1]+reflex_scaling[1]*robotState.leftGripper.positionCommand[1];
      pose_cmd.f3=reflex_offset[2]+reflex_scaling[2]*robotState.leftGripper.positionCommand[2];
      pose_cmd.preshape=reflex_offset[3]+reflex_scaling[3]*robotState.leftGripper.positionCommand[3];
      v_cmd.f1=robotState.leftGripper.speedCommand[0];
      v_cmd.f2=robotState.leftGripper.speedCommand[1];
      v_cmd.f3=robotState.leftGripper.speedCommand[2];
      v_cmd.preshape=robotState.leftGripper.speedCommand[3];
      cmd.pose=pose_cmd;
      cmd.velocity=v_cmd;
      left_gripper_pub.publish(cmd.pose);

    }
    robotState.leftGripper.sendCommand = false;
  }
  if(robotState.rightGripper.sendCommand) {
    if(robotState.rightGripper.position.size()==1) { //electric gripper
      ScopedLock lock(mutex);
      baxter_core_msgs::EndEffectorCommand msg;
      msg.id = rightGripperState->id;
      {
	stringstream ss;
	ss<<"{\"velocity\":"<<robotState.rightGripper.speedCommand[0]*100<<",\"moving_force\":"<<robotState.rightGripper.forceCommand[0]*110<<",\"holding_force\":"<<robotState.rightGripper.forceCommand[0]*100<<",\"dead_zone\": 5.0 }";
	msg.command = "configure";
	msg.args = ss.str();
	//right_gripper_pub.publish(msg);
      }
      {
	msg.command = "go";
	stringstream ss;
	ss<<"{\"position\":"<<robotState.rightGripper.positionCommand[0]*100<<"}";
	msg.args = ss.str();
	right_gripper_pub.publish(msg);
      }
    }
    else if(robotState.rightGripper.position.size()==MOTOR_COUNT) {  //reflex gripper
      ScopedLock lock(mutex);

      reflex_msgs::Command cmd;
      reflex_msgs::PoseCommand pose_cmd;
      reflex_msgs::VelocityCommand v_cmd;
      //printf("motion_physical:right preshape is: %f\n",robotState.rightGripper.positionCommand[3]);
      pose_cmd.f1=reflex_offset[0]+reflex_scaling[0]*robotState.rightGripper.positionCommand[0];
      pose_cmd.f2=reflex_offset[1]+reflex_scaling[1]*robotState.rightGripper.positionCommand[1];
      pose_cmd.f3=reflex_offset[2]+reflex_scaling[2]*robotState.rightGripper.positionCommand[2];
      pose_cmd.preshape=reflex_offset[3]+reflex_scaling[3]*robotState.rightGripper.positionCommand[3];
      v_cmd.f1=robotState.rightGripper.speedCommand[0];
      v_cmd.f2=robotState.rightGripper.speedCommand[1];
      v_cmd.f3=robotState.rightGripper.speedCommand[2];
      v_cmd.preshape=robotState.rightGripper.speedCommand[3];
      cmd.pose=pose_cmd;
      cmd.velocity=v_cmd;
      right_gripper_pub.publish(cmd.pose);
      }
      robotState.rightGripper.sendCommand = false;
  }
  //process base commands
  if(robotState.base.sendCommand) {
    printf("Sending base command mode %d\n",robotState.base.controlMode);
    cout<<"    Value: "<<robotState.base.command<<endl;
    Assert(robotState.base.command.size()==3);
    if(robotState.base.controlMode == BaseState::NONE) {
      robotState.base.sendCommand = false;
    }
    else if(robotState.base.controlMode == BaseState::RELATIVE_POSITION || robotState.base.controlMode == BaseState::ODOMETRY_POSITION) {
      if(robotState.base.controlMode == BaseState::RELATIVE_POSITION) {
	//first transform to global odometry
	robotState.base.controlMode = BaseState::ODOMETRY_POSITION;
	Vector newCommand(3);
	robotState.base.ToOdometry(robotState.base.command,newCommand);
	robotState.base.command = newCommand;
      }
    }
    else if(robotState.base.controlMode == BaseState::RELATIVE_VELOCITY) {
      /*
      if(robotState.base.commandedVelocity.norm() < 1e-3) //done!
	robotState.base.sendCommand = false;
      */
    }
    robotState.base.IntegrateCommand(baseCalibration,dt);
    /*
    if(robotState.base.controlMode == BaseState::RELATIVE_POSITION || robotState.base.controlMode == BaseState::ODOMETRY_POSITION)  {
      if(robotState.base.commandedPosition.distance(robotState.base.command)<1e-3 && robotState.base.commandedVelocity.norm() < 1e-3)
	robotState.base.sendCommand = false;
    }
    */

    Vector twist(3,0.0);
    #if BASE_RAW_COMMANDS
      twist = robotState.base.commandedVelocity;
    #else
      cout<<"Desired twist "<<robotState.base.commandedVelocity<<endl;
      robotState.base.DesiredToLowLevelCommand(baseCalibration,dt,twist);
      cout<<"Commanded twist after PID + friction compensation "<<twist<<endl;
    #endif //!BASE_RAW_COMMANDS

    if(!twist.empty()) {
      geometry_msgs::Twist cmd;
      cmd.linear.x = twist(1)*base_x_scaling;
      cmd.linear.y = -twist(0)*base_y_scaling;
      cmd.linear.z = 0.0;
      cmd.angular.x = 0.0;
      cmd.angular.y = 0.0;
      cmd.angular.z = base_heading_flip*twist(2)*base_theta_scaling;
      base_twist_pub.publish(cmd);
    }
  }
  ros::spinOnce();
}

void MyControllerUpdateData::MyShutdown()
{
  //KH: don't disable robot on shutdown
  /*
  ros::Publisher enabler = nh->advertise<std_msgs::Bool>("/robot/set_super_enable",10);
  while(ros::ok()) {
    if(!assemblyState->enabled) {
      printf("Successful shutdown, robot is now disabled\n");
      break;
    }
    else {
      std_msgs::Bool msg;
      msg.data = false;
      enabler.publish(msg);
    }
    ros::spinOnce();
    ThreadSleep(0.5);
  }
  */

  ros::shutdown();
}


double getMobileBaseMoveTime()
{
  return 0;
}

///Returns the estimated time until the gripper stops
double getGripperMoveTime(int limb)
{
  return 0;
}
