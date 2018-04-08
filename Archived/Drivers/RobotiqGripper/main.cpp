#include "Robotiq.hpp"
#include <stdlib.h>

using namespace Robotiq::ModbusRTU;

int main(int argc,char** argv)
{
  if(argc <= 1) {
    printf("Usage: test PORT\n");
    return 0;
  }
  Gripper gripper(argv[1]);
  if(!gripper.Connected()) {
    printf("Error connecting to Robotiq on serial port %s\n",argv[1]);
    return 1;
  }
  printf("Activating...\n");
  if(!gripper.Activate()) printf("Error Activating\n");
  if(!gripper.WaitForReady()) printf("Error waiting for ready\n");

  printf("Setting pinch mode...\n");
  gripper.SetPinchMode();
  gripper.WaitForReady();

  printf("Setting basic mode...\n");
  gripper.SetBasicMode();
  gripper.WaitForReady();

  printf("Closing gripper...\n");
  gripper.Goto(255,255,255);
  gripper.WaitForGoto();

  printf("Opening gripper...\n");
  gripper.Goto(0);
  gripper.WaitForGoto();

  printf("Enabling individual finger control...\n");
  gripper.SetIndividualFingerControl(true);
  gripper.SetActuatorPosition(0,255);
  gripper.SetActuatorSpeed(0,255);
  gripper.SetActuatorForce(0,255);
  gripper.SetActuatorPosition(1,196);
  gripper.SetActuatorSpeed(1,128);
  gripper.SetActuatorForce(1,128);
  gripper.SetActuatorPosition(2,128);
  gripper.SetActuatorSpeed(2,64);
  gripper.SetActuatorForce(2,64);
  gripper.SendActuatorCommands();
  gripper.WaitForGoto();

  printf("Disabling individual finger control and opening gripper...\n");
  gripper.SetIndividualFingerControl(false);
  gripper.Goto(0);
  gripper.WaitForGoto();

  return 0;
}
