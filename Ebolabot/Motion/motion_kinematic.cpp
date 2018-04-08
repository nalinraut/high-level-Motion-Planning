#include "motion_state.h"

class MyControllerUpdateData : public ControllerUpdateData
{
public:
  double lastSensorTime;
  virtual bool MyStartup() {
    ScopedLock lock(mutex);
    if(!robotModel) {
      printf("Motion(Kinematic): Klamp't model is not provided, cannot start\n");
      return false;
    }
    if(!GetKlamptIndices()) {
      //check whether to return false -- accept loss of gripper and base functionality
      for(int i=0;i<numLimbDofs;i++) {
	if(leftKlamptIndices[i] < 0 || rightKlamptIndices[i] < 0) {
	  printf("Motion(Kinematic): Klamp't model does not have appropriate limb indices\n");
	  leftKlamptIndices.resize(0);
	  rightKlamptIndices.resize(0);
	  //accept loss of arm functionality
	  break;
	  return false;
	}
      }
      /*
      if(headPanKlamptIndex < 0) {
	printf("Motion(Kinematic): Klamp't model does not have appropriate head indices\n");
	return false;
      }
      */
      for(int i=0;i<3;i++) {
	if(baseKlamptIndices[i] < 0) {
	  baseKlamptIndices.resize(0);
	  break;
	}
      }
    }
    if(rightGripperMap.name.empty())
      robotState.rightGripper.enabled = false;
    else {
      robotState.rightGripper.enabled = true;
      robotState.rightGripper.name = rightGripperMap.name;
      int numGripperDofs = (int)rightGripperMap.klamptIndices.size();
      //assume open gripper
      robotState.rightGripper.position.resize(numGripperDofs,1.0);
      robotState.rightGripper.positionCommand.resize(numGripperDofs,1.0);
      robotState.rightGripper.speedCommand.resize(numGripperDofs,0.0);
      robotState.rightGripper.forceCommand.resize(numGripperDofs,0.0);
    }
    if(leftGripperMap.name.empty()) 
      robotState.leftGripper.enabled = false;
    else {
      robotState.leftGripper.enabled = true;
      robotState.leftGripper.name = leftGripperMap.name;
      int numGripperDofs = (int)leftGripperMap.klamptIndices.size();
      //assume open gripper
      robotState.leftGripper.position.resize(numGripperDofs,1.0);
      robotState.leftGripper.positionCommand.resize(numGripperDofs,1.0);
      robotState.leftGripper.speedCommand.resize(numGripperDofs,0.0);
      robotState.leftGripper.forceCommand.resize(numGripperDofs,0.0);
    }
    if(!baseKlamptIndices.empty()) {
      robotState.base.enabled = true;
      robotState.base.odometry.resize(3,0.0);
      robotState.base.velocity.resize(3,0.0);
      robotState.base.command.resize(3,0.0);
      robotState.base.senseUpdateTime = 0;
    }
    robotState.leftLimb.senseUpdateTime = 0;
    robotState.rightLimb.senseUpdateTime = 0;
    KlamptToLimb(planner.world.robots[0]->q,LEFT,robotState.leftLimb.sensedConfig);
    KlamptToLimb(planner.world.robots[0]->q,RIGHT,robotState.rightLimb.sensedConfig);
    robotState.leftLimb.commandedConfig = robotState.leftLimb.sensedConfig;
    robotState.rightLimb.commandedConfig = robotState.rightLimb.sensedConfig;
    KlamptToLimb(planner.world.robots[0]->dq,LEFT,robotState.leftLimb.sensedVelocity);
    KlamptToLimb(planner.world.robots[0]->dq,RIGHT,robotState.rightLimb.sensedVelocity);
    robotState.leftLimb.commandedVelocity = robotState.leftLimb.sensedVelocity;
    robotState.rightLimb.commandedVelocity = robotState.rightLimb.sensedVelocity;
    return true;
  }
  virtual bool MyProcessSensors() {
    ScopedLock lock(mutex);
    double dt = t - lastSensorTime;
    //limb sensors
    if(dt > 0) {
      robotState.leftLimb.sensedVelocity = (robotState.leftLimb.commandedConfig - robotState.leftLimb.sensedConfig) / dt;
      robotState.rightLimb.sensedVelocity = (robotState.rightLimb.commandedConfig - robotState.rightLimb.sensedConfig) / dt;
    }
    robotState.leftLimb.sensedConfig = robotState.leftLimb.commandedConfig;
    robotState.rightLimb.sensedConfig = robotState.rightLimb.commandedConfig;
    robotState.leftLimb.senseUpdateTime = t;
    robotState.rightLimb.senseUpdateTime = t;

    //base moving flag
    if(robotState.base.enabled) {
      if(robotState.base.velocity.norm() > 1e-3 || robotState.base.sendCommand)
	robotState.base.moving = true;
      else
	robotState.base.moving = false;
    }

    lastSensorTime = t;
    return true;
  }
  virtual void MySendCommands() {
    ScopedLock lock(mutex);
    double dt = t - last_t;
    //advance limbs
    if(robotState.leftLimb.sendCommand) {
      if(robotState.leftLimb.controlMode == LimbState::POSITION || robotState.leftLimb.controlMode == LimbState::RAW_POSITION)
	robotState.leftLimb.sendCommand = false;
      else if(robotState.leftLimb.controlMode == LimbState::VELOCITY) 
	robotState.leftLimb.commandedConfig.madd(robotState.leftLimb.commandedVelocity,dt);
      else  //effort mode not supported
	robotState.leftLimb.sendCommand = false;
      //handle joint limits
      if(robotModel) {
	for(size_t i=0;i<leftKlamptIndices.size();i++) {
	  if(robotState.leftLimb.commandedConfig[i] < robotModel->qMin[leftKlamptIndices[i]] ||
	     robotState.leftLimb.commandedConfig[i] > robotModel->qMax[leftKlamptIndices[i]]) {
	    robotState.leftLimb.commandedConfig[i] = Clamp(robotState.leftLimb.commandedConfig[i],robotModel->qMin[leftKlamptIndices[i]],robotModel->qMax[leftKlamptIndices[i]]);
	    robotState.leftLimb.commandedVelocity[i] = 0;
	  }
	}
      }
    }
    if(robotState.rightLimb.sendCommand) {
      if(robotState.rightLimb.controlMode == LimbState::POSITION || robotState.rightLimb.controlMode == LimbState::RAW_POSITION)
	robotState.rightLimb.sendCommand = false;
      else if(robotState.rightLimb.controlMode == LimbState::VELOCITY)
	robotState.rightLimb.commandedConfig.madd(robotState.rightLimb.commandedVelocity,dt);
      else  //effort mode not supported
	robotState.rightLimb.sendCommand = false;
      //handle joint limits
      if(robotModel) {
	for(size_t i=0;i<rightKlamptIndices.size();i++) {
	  if(robotState.rightLimb.commandedConfig[i] < robotModel->qMin[rightKlamptIndices[i]] ||
	     robotState.rightLimb.commandedConfig[i] > robotModel->qMax[rightKlamptIndices[i]]) {
	    robotState.rightLimb.commandedConfig[i] = Clamp(robotState.rightLimb.commandedConfig[i],robotModel->qMin[rightKlamptIndices[i]],robotModel->qMax[rightKlamptIndices[i]]);
	    robotState.rightLimb.commandedVelocity[i] = 0;
	  }
	}
      }
    }
    //advance grippers
    if(robotState.leftGripper.sendCommand) 
      robotState.leftGripper.sendCommand = false;
    robotState.leftGripper.moving = false;
    for(int i=0;i<robotState.leftGripper.position.n;i++) {
      double pdes = Clamp(robotState.leftGripper.positionCommand[i],0.0,1.0);
      double v = Clamp(robotState.leftGripper.speedCommand[i],0.0,1.0);
      double old = robotState.leftGripper.position[i];
      robotState.leftGripper.position[i] += dt*Sign(pdes-robotState.leftGripper.position[i])*v;
      if(Sign(old-pdes) != Sign(robotState.leftGripper.position[i]-pdes))
	//stop
	robotState.leftGripper.position[i] = pdes;
      if(robotState.leftGripper.position[i] != pdes)
	robotState.leftGripper.moving=true;
    }
    if(robotState.rightGripper.sendCommand) 
      robotState.rightGripper.sendCommand = false;
    robotState.rightGripper.moving = false;
    for(int i=0;i<robotState.rightGripper.position.n;i++) {
      double pdes = Clamp(robotState.rightGripper.positionCommand[i],0.0,1.0);
      double v = Clamp(robotState.leftGripper.speedCommand[i],0.0,1.0);
      double old = robotState.rightGripper.position[i];
      robotState.rightGripper.position[i] += dt*Sign(pdes-robotState.rightGripper.position[i])*v;
      if(Sign(old-pdes) != Sign(robotState.rightGripper.position[i]-pdes))
	//stop
	robotState.rightGripper.position[i] = pdes;
      if(robotState.rightGripper.position[i] != pdes)
	robotState.rightGripper.moving = true;
    }
    //process base commands
    if(robotState.base.sendCommand) {
      Assert(robotState.base.command.size()==3);
      if(robotState.base.controlMode == BaseState::NONE) {
	robotState.base.sendCommand = false;
  robotState.base.moving = false;
  robotState.base.velocity.set(0.0);
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
	//physical mode latches the command ... kinematic mode doesnt
	//robotState.base.sendCommand = false;
	if(robotState.base.command.norm() < 1e-3) { //done!
	  robotState.base.sendCommand = false;
    robotState.base.moving = false;
    robotState.base.velocity.set(0.0);
  }
      }
      robotState.base.IntegrateCommand(baseCalibration,dt);
      //cout<<"Desired twist "<<robotState.base.commandedVelocity<<endl;

      if(robotState.base.controlMode == BaseState::RELATIVE_POSITION || robotState.base.controlMode == BaseState::ODOMETRY_POSITION)  {
	if(robotState.base.commandedPosition.distance(robotState.base.command)<1e-3 && robotState.base.commandedVelocity.norm() < 1e-3) {
    printf("Arrived, setting sendCommand/moving = false\n");
    robotState.base.sendCommand = false;
    robotState.base.moving = false;
    robotState.base.velocity.set(0.0);
  }
      }
    }
    //simulate base
    if(robotState.base.enabled && robotState.base.sendCommand) {
      //simulate friction
      Vector twist(3,0.0);
      robotState.base.DesiredToLowLevelCommand(baseCalibration,dt,twist);
      cout<<"Commanded twist "<<twist<<", dt "<<endl;
      if(Abs(twist(0)) < baseCalibration.xFriction)
	twist(0) = 0;
      else
	twist(0) -= Sign(twist(0))*baseCalibration.xFriction;
      if(Abs(twist(1)) < baseCalibration.yFriction)
	twist(1) = 0;
      else
	twist(1) -= Sign(twist(1))*baseCalibration.yFriction;
      if(Abs(twist(2)) < baseCalibration.angFriction)
	twist(2) = 0;
      else
	twist(2) -= Sign(twist(2))*baseCalibration.angFriction;
      cout<<"Friction changed twist to "<<twist<<endl;
      robotState.base.velocity = twist;
      Matrix2 R; R.setRotate(robotState.base.odometry(2));
      Vector2 vworld = R*Vector2(twist(0),twist(1));
      robotState.base.odometry(0) += vworld.x*dt;
      robotState.base.odometry(1) += vworld.y*dt;
      robotState.base.odometry(2) += twist(2)*dt;
      robotState.base.senseUpdateTime = t;
      assert(robotState.base.sendCommand == true);
    }
  }
};

#include "motion_common.h"

double getMobileBaseMoveTime()
{
  return 0;
}

///Returns the estimated time until the gripper stops
double getGripperMoveTime(int limb)
{
  return 0;
}

