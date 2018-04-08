#include "Robotiq.h"
#include <stdlib.h>

int main(int argc,char** argv)
{
  ROBOTIQ_HANDLE g;
  if(argc <= 1) {
    printf("Usage: test-lib PORT\n");
    return 0;
  }
  g = robotiqCreate_Modbus(argv[1]);
  if(g == 0) {
    printf("Error connecting to robotiq on serial port %s\n",argv[1]);
    return 1;
  }
  printf("Activating...\n");
  if(!robotiqActivate(g)) printf("Error Activating\n");
  if(!robotiqWaitForReady(g,10,50)) printf("Error waiting for ready\n");

  printf("Setting pinch mode...\n");
  robotiqSetPinchMode(g);
  robotiqWaitForReady(g,10,50);

  printf("Setting basic mode...\n");
  robotiqSetBasicMode(g);
  robotiqWaitForReady(g,10,50);

  printf("Closing robotiq..\n");
  robotiqGoto(g,255,255,255);
  robotiqWaitForGoto(g,10,50);

  printf("Opening robotiq..\n");
  robotiqGotoSimple(g,0);
  robotiqWaitForGoto(g,10,50);

  printf("Enabling individual finger control...\n");
  robotiqSetIndividualFingerControl(g,TRUE);
  robotiqSetActuatorPosition(g,0,255);
  robotiqSetActuatorSpeed(g,0,255);
  robotiqSetActuatorForce(g,0,255);
  robotiqSetActuatorPosition(g,1,196);
  robotiqSetActuatorSpeed(g,1,128);
  robotiqSetActuatorForce(g,1,128);
  robotiqSetActuatorPosition(g,2,128);
  robotiqSetActuatorSpeed(g,2,64);
  robotiqSetActuatorForce(g,2,64);
  robotiqSendActuatorCommands(g);
  robotiqWaitForGoto(g,10,50);

  printf("Disabling individual finger control and opening robotiq..\n");
  robotiqSetIndividualFingerControl(g,FALSE);
  robotiqGotoSimple(g,0);
  robotiqWaitForGoto(g,10,50);

  robotiqDestroy(g);
  return 0;
}
