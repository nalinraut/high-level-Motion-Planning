#include "Robotiq.h"
#include "Robotiq.hpp"
using namespace Robotiq;

ROBOTIQ_HANDLE robotiqCreate_Modbus(const char* port)
{
  ModbusRTU::Gripper* gripper = new ModbusRTU::Gripper(port);
  if(!gripper->Connected()) {
    delete gripper;
    return NULL;
  }
  return gripper;
}
void robotiqDestroy(ROBOTIQ_HANDLE h)
{
  GripperBase* g=(GripperBase*)h;
  delete g;
}


#define FUNC0(RETTYPE,NAME)	       \
  RETTYPE robotiq##NAME(ROBOTIQ_HANDLE h) \
  { \
  GripperBase* g=(GripperBase*)h; \
  return g->NAME(); \
  }

#define FUNC1(RETTYPE,NAME,TYPE)				\
  RETTYPE robotiq##NAME(ROBOTIQ_HANDLE h,TYPE arg)		\
  { \
  GripperBase* g=(GripperBase*)h; \
  return g->NAME(arg); \
  }


FUNC0(BOOL,Activate)
FUNC0(BOOL,Deactivate)
FUNC0(BOOL,UpdateStatus)

FUNC1(void,PrintStatus,FILE*)
FUNC0(BOOL,IsReset)
FUNC0(BOOL,IsActivated)
FUNC0(BOOL,IsChangingMode)
FUNC0(BOOL,IsActivating)
FUNC0(BOOL,IsReady)
FUNC0(uint8_t,GetFaultStatus)
FUNC1(BOOL,SetMode,int)
FUNC0(BOOL,SetBasicMode)
FUNC0(BOOL,SetPinchMode)
FUNC0(BOOL,SetWideMode)
FUNC0(BOOL,SetScissorMode)
FUNC0(uint8_t,GetMode)

BOOL robotiqGotoSimple(ROBOTIQ_HANDLE h,uint8_t position)
{
  GripperBase* g=(GripperBase*)h;
  return g->Goto(position);
}

BOOL robotiqGoto(ROBOTIQ_HANDLE h,uint8_t position,uint8_t speed,uint8_t force)
{
  GripperBase* g=(GripperBase*)h;
  return g->Goto(position,speed,force);
}
BOOL robotiqActuatorGoto(ROBOTIQ_HANDLE h,int actuator,uint8_t position,uint8_t speed,uint8_t force)
{
  GripperBase* g=(GripperBase*)h;
  return g->Goto(actuator,position,speed,force);
}

//the following queue up commands until the next SendActuatorCommands or
//Goto command
BOOL robotiqSetActuatorPosition(ROBOTIQ_HANDLE h,int actuator,uint8_t position)
{
  GripperBase* g=(GripperBase*)h;
  return g->SetActuatorPosition(actuator,position);
}
BOOL robotiqSetActuatorSpeed(ROBOTIQ_HANDLE h,int actuator,uint8_t speed)
{
  GripperBase* g=(GripperBase*)h;
  return g->SetActuatorSpeed(actuator,speed);
}
BOOL robotiqSetActuatorForce(ROBOTIQ_HANDLE h,int actuator,uint8_t force)
{
  GripperBase* g=(GripperBase*)h;
  return g->SetActuatorForce(actuator,force);
}
//sends queued up SetActuatorX commands
FUNC0(BOOL,SendActuatorCommands)
FUNC1(uint8_t,GetActuatorPosition,int)
FUNC1(uint8_t,GetActuatorSpeed,int)
FUNC1(uint8_t,GetActuatorPositionRequest,int)
FUNC1(uint8_t,GetActuatorForce,int)
FUNC0(BOOL,AreAllFingersMoving)
FUNC0(BOOL,AreSomeFingersMoving)
FUNC0(BOOL,AreSomeFingersStopped)
FUNC0(BOOL,AreAllFingersStopped)
FUNC0(BOOL,AreAllFingersDone)
FUNC1(BOOL,IsActuatorMoving,int)
FUNC1(BOOL,IsActuatorStopped,int)
FUNC1(BOOL,IsActuatorStoppedClosing,int)
FUNC1(BOOL,IsActuatorStoppedOpening,int)
FUNC1(BOOL,IsActuatorDone,int)
FUNC0(BOOL,SetAutomaticRelease)
FUNC0(BOOL,GetAutomaticRelease)
FUNC1(BOOL,SetIndividualFingerControl,BOOL)
FUNC1(BOOL,SetIndividualScissorControl,BOOL)
FUNC0(BOOL,GetIndividualFingerControl)
FUNC0(BOOL,GetIndividualScissorControl)

//Helpers.
//Returns true if ready state was reached
BOOL robotiqWaitForReady(ROBOTIQ_HANDLE h,double maxseconds,int pollfreq)
{
  GripperBase* g=(GripperBase*)h;
  return g->WaitForReady(maxseconds,pollfreq);
}
//Returns 1 if fingers reached destination, 0 if stopped, -1 if maxseconds time elapsed
int robotiqWaitForGoto(ROBOTIQ_HANDLE h,double maxseconds,int pollfreq)
{
  GripperBase* g=(GripperBase*)h;
  return g->WaitForGoto(maxseconds,pollfreq);
}
