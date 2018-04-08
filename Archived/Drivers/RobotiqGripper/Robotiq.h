#ifndef ROBOTIQ_API_H
#define ROBOTIQ_API_H

#include <stdio.h>
#include <stdint.h>

typedef void* ROBOTIQ_HANDLE;
#ifndef BOOL
typedef int BOOL;
#define TRUE 1
#define FALSE 0
#endif //BOOL

//finger preshape modes (active when scissor mode is not active)
enum {ROBOTIQ_MODE_BASIC=0,
      ROBOTIQ_MODE_PINCH=1,
      ROBOTIQ_MODE_WIDE=2,
      ROBOTIQ_MODE_SCISSOR=3};

//actuator indices
enum {ROBOTIQ_FINGER1=0,
      ROBOTIQ_FINGER2=1,
      ROBOTIQ_FINGER3=2,
      ROBOTIQ_SCISSOR=3};

#ifdef _WIN32
   #error "TODO: dllexport/import stuff"
#else
   #ifdef __cplusplus
     #define APIENTRY extern "C"
   #else
     #define APIENTRY extern 
   #endif //__cplusplus
#endif

APIENTRY ROBOTIQ_HANDLE robotiqCreate_Modbus(const char* port);
APIENTRY void robotiqDestroy(ROBOTIQ_HANDLE);

//activate / deactivate 
APIENTRY BOOL robotiqActivate(ROBOTIQ_HANDLE);
APIENTRY BOOL robotiqDeactivate(ROBOTIQ_HANDLE);

//status querying. Need to call robotiqUpdateStatus first before getting
//most up-to-date status / finger state information.
APIENTRY BOOL robotiqUpdateStatus(ROBOTIQ_HANDLE);
APIENTRY void robotiqPrintStatus(ROBOTIQ_HANDLE,FILE* f);
APIENTRY BOOL robotiqIsReset(ROBOTIQ_HANDLE);
APIENTRY BOOL robotiqIsActivated(ROBOTIQ_HANDLE);
APIENTRY BOOL robotiqIsChangingMode(ROBOTIQ_HANDLE);
APIENTRY BOOL robotiqIsActivating(ROBOTIQ_HANDLE);
APIENTRY BOOL robotiqIsReady(ROBOTIQ_HANDLE);
APIENTRY uint8_t robotiqGetFaultStatus(ROBOTIQ_HANDLE);

//change the finger mode
APIENTRY BOOL robotiqSetMode(ROBOTIQ_HANDLE,int mode);
APIENTRY BOOL robotiqSetBasicMode(ROBOTIQ_HANDLE);
APIENTRY BOOL robotiqSetPinchMode(ROBOTIQ_HANDLE);
APIENTRY BOOL robotiqSetWideMode(ROBOTIQ_HANDLE);
APIENTRY BOOL robotiqSetScissorMode(ROBOTIQ_HANDLE);
APIENTRY uint8_t robotiqGetMode(ROBOTIQ_HANDLE);

//sends the given basic mode command
APIENTRY BOOL robotiqGotoSimple(ROBOTIQ_HANDLE,uint8_t position);
//sends the given basic mode command
APIENTRY BOOL robotiqGoto(ROBOTIQ_HANDLE,uint8_t position,uint8_t speed,uint8_t force);
//sends the following basic mode command
APIENTRY BOOL robotiqActuatorGoto(ROBOTIQ_HANDLE,int actuator,uint8_t position,uint8_t speed,uint8_t force);
//the following queue up commands until the next SendActuatorCommands or
//Goto command
APIENTRY BOOL robotiqSetActuatorPosition(ROBOTIQ_HANDLE,int actuator,uint8_t position);
APIENTRY BOOL robotiqSetActuatorSpeed(ROBOTIQ_HANDLE,int actuator,uint8_t speed);
APIENTRY BOOL robotiqSetActuatorForce(ROBOTIQ_HANDLE,int actuator,uint8_t force);
//sends queued up SetActuatorX commands
APIENTRY BOOL robotiqSendActuatorCommands(ROBOTIQ_HANDLE);
APIENTRY uint8_t robotiqGetActuatorPosition(ROBOTIQ_HANDLE,int actuator);
APIENTRY uint8_t robotiqGetActuatorSpeed(ROBOTIQ_HANDLE,int actuator);
APIENTRY uint8_t robotiqGetActuatorPositionRequest(ROBOTIQ_HANDLE,int actuator);
APIENTRY uint8_t robotiqGetActuatorForce(ROBOTIQ_HANDLE,int actuator);
APIENTRY BOOL robotiqAreAllFingersMoving(ROBOTIQ_HANDLE);
APIENTRY BOOL robotiqAreSomeFingersMoving(ROBOTIQ_HANDLE);
APIENTRY BOOL robotiqAreSomeFingersStopped(ROBOTIQ_HANDLE);
APIENTRY BOOL robotiqAreAllFingersStopped(ROBOTIQ_HANDLE);
APIENTRY BOOL robotiqAreAllFingersDone(ROBOTIQ_HANDLE);
APIENTRY BOOL robotiqIsActuatorMoving(ROBOTIQ_HANDLE,int actuator);
APIENTRY BOOL robotiqIsActuatorStopped(ROBOTIQ_HANDLE,int actuator);
APIENTRY BOOL robotiqIsActuatorStoppedClosing(ROBOTIQ_HANDLE,int actuator);
APIENTRY BOOL robotiqIsActuatorStoppedOpening(ROBOTIQ_HANDLE,int actuator);
APIENTRY BOOL robotiqIsActuatorDone(ROBOTIQ_HANDLE,int actuator);
//on emergency stop -- can't address gripper after this
APIENTRY BOOL robotiqSetAutomaticRelease(ROBOTIQ_HANDLE);
APIENTRY BOOL robotiqGetAutomaticRelease(ROBOTIQ_HANDLE);
//turn on individual finger / scissor control
APIENTRY BOOL robotiqSetIndividualFingerControl(ROBOTIQ_HANDLE,BOOL enabled);
APIENTRY BOOL robotiqSetIndividualScissorControl(ROBOTIQ_HANDLE,BOOL enabled);
APIENTRY BOOL robotiqGetIndividualFingerControl(ROBOTIQ_HANDLE);
APIENTRY BOOL robotiqGetIndividualScissorControl(ROBOTIQ_HANDLE);

//Helpers for writing state machines.

//Returns true if ready state was reached. Suggest defaults maxseconds = 10, pollfreq = 50
APIENTRY BOOL robotiqWaitForReady(ROBOTIQ_HANDLE,double maxseconds,int pollfreq);

//Returns 1 if fingers reached destination, 0 if stopped, -1 if maxseconds time elapsed.
//Suggest defaults maxseconds = 10, pollfreq = 50
APIENTRY int robotiqWaitForGoto(ROBOTIQ_HANDLE,double maxseconds,int pollfreq);

#endif
