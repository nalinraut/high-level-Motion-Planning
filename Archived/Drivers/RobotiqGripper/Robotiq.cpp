#include "Robotiq.hpp"
#include <modbus/modbus.h>
#include <stdlib.h>
#include <string.h>
#include <endian.h>
#include <unistd.h>


namespace Robotiq
{
  const static int NumInputBytes = 16;
  const static int NumOuputBytes = 16;
  static const char* InputNames [] = {
    "ACTION REQUEST",
    "GRIPPER OPTION 1",
    "GRIPPER OPTION 2",
    "POSITION REQUEST",
    "SPEED",
    "FORCE",
    "FINGER B POS. REQUEST",
    "FINGER B SPEED",
    "FINGER B FORCE",
    "FINGER C POS. REQUEST",
    "FINGER C SPEED",
    "FINGER C FORCE",
    "SCISSOR POS. REQUEST",
    "SCISSOR SPEED",
    "SCISSOR FORCE",
    "RESERVED"
  };
  static const char* OutputNames [] = {
    "GRIPPER STATUS",
    "OBJECT DETECTION",
    "FAULT_STATUS",
    "POS. REQUEST ECHO",
    "FINGER A POSITION",
    "FINGER A CURRENT",
    "FINGER B POS. REQUEST ECHO",
    "FINGER B POSITION",
    "FINGER B CURRENT",
    "FINGER C POS. REQUEST ECHO",
    "FINGER C POSITION",
    "FINGER C CURRENT",
    "SCISSOR POS. REQUEST ECHO",
    "SCISSOR POSITION",
    "SCISSOR CURRENT",
    "RESERVED"
  };
  //input register indices
  const static int ACTION_REQUEST_Index = 0;
  const static int GRIPPER_OPTION_1_Index = 1;
  const static int GRIPPER_OPTION_2_Index = 2;
  const static int POSITION_REQUEST_Index = 3;
  const static int SPEED_Index = 4;
  const static int FORCE_Index = 5;
  const static int FINGER_B_POS_REQUEST_Index = 6;
  const static int FINGER_B_SPEED_Index = 7;
  const static int FINGER_B_FORCE_Index = 8;
  const static int FINGER_C_POS_REQUEST_Index = 9;
  const static int FINGER_C_SPEED_Index = 10;
  const static int FINGER_C_FORCE_Index = 11;
  const static int SCISSOR_POS_REQUEST_Index = 12;
  const static int SCISSOR_SPEED_Index = 13;
  const static int SCISSOR_FORCE_Index = 14;
  //output register indices
  const static int GRIPPER_STATUS_Index = 0;
  const static int OBJECT_DETECTION_Index = 1;
  const static int FAULT_STATUS_Index = 2;
  const static int POS_REQUEST_ECHO_Index = 3;
  const static int FINGER_A_POSITION_Index = 4;
  const static int FINGER_A_CURRENT_Index = 5;
  const static int FINGER_B_POS_REQUEST_ECHO_Index = 6;
  const static int FINGER_B_POSITION_Index = 7;
  const static int FINGER_B_CURRENT_Index = 8;
  const static int FINGER_C_POS_REQUEST_ECHO_Index = 9;
  const static int FINGER_C_POSITION_Index = 10;
  const static int FINGER_C_CURRENT_Index = 11;
  const static int SCISSOR_POS_REQUEST_ECHO_Index = 12;
  const static int SCISSOR_POSITION_Index = 13;
  const static int SCISSOR_CURRENT_Index = 14;
  //both input and output
  const static int RESERVED_Index = 15;


  const static uint8_t ACTIVATE_GRIPPER_MASK = 0x1;
  const static uint8_t GRASPING_MODE_MASK = 0x6;
  const static uint8_t GRASPING_MODE_SHIFT = 1;
  const static uint8_t GOTO_MASK = 0x8;
  const static uint8_t GOTO_SHIFT = 3;
  const static uint8_t AUTOMATIC_RELEASE_MASK = 0xA;
  const static uint8_t AUTOMATIC_RELEASE_SHIFT = 4;
  const static uint8_t INDIVIDUAL_FINGER_CONTROL_MASK = 0x4;
  const static uint8_t INDIVIDUAL_FINGER_CONTROL_SHIFT = 2;
  const static uint8_t INDIVIDUAL_SCISSOR_CONTROL_MASK = 0x8;
  const static uint8_t INDIVIDUAL_SCISSOR_CONTROL_SHIFT = 3;

  const static uint8_t GRIPPER_STATUS_STATUS_MASK = 0x30;
  const static uint8_t GRIPPER_STATUS_STATUS_SHIFT = 4;
  const static uint8_t GRIPPER_MOTION_STATUS_MASK = 0xC0;
  const static uint8_t GRIPPER_MOTION_STATUS_SHIFT = 6;

  GripperBase::GripperBase()
  {
    memset(status.bytes,0,16);
    memset(command,0,16);
    command[SPEED_Index] = 100;
    command[FORCE_Index] = 100;
  }
  bool GripperBase::Activate()
  {
    if(command[ACTION_REQUEST_Index] & ACTIVATE_GRIPPER_MASK)
      //already activated
      return true;
    command[ACTION_REQUEST_Index] |= ACTIVATE_GRIPPER_MASK;
    return SendCommand(command);
  }
  
  bool GripperBase::Deactivate()
  {
    if(!(command[ACTION_REQUEST_Index] & ACTIVATE_GRIPPER_MASK))
      //already activated
      return true;
    command[ACTION_REQUEST_Index] &= ~ACTIVATE_GRIPPER_MASK;
    return SendCommand(command);
  }

  void GripperBase::PrintStatus(FILE* f)
  {
    for(int i=0;i<16;i++)
      fprintf(f,"%s (%d): %04x\n",OutputNames[i],i,(int)status.bytes[i]);
  }

  bool GripperBase::IsActivated() const
  {
    return (status.bytes[GRIPPER_STATUS_Index] & ACTIVATE_GRIPPER_MASK) != 0;
  }

  bool GripperBase::IsReset() const
  {
    return ((status.bytes[GRIPPER_STATUS_Index] & GRIPPER_STATUS_STATUS_MASK)>>GRIPPER_STATUS_STATUS_SHIFT) == 0;
  }

  bool GripperBase::IsActivating() const
  {
    return ((status.bytes[GRIPPER_STATUS_Index] & GRIPPER_STATUS_STATUS_MASK)>>GRIPPER_STATUS_STATUS_SHIFT) == 1;
  }

  bool GripperBase::IsChangingMode() const
  {
    return ((status.bytes[GRIPPER_STATUS_Index] & GRIPPER_STATUS_STATUS_MASK)>>GRIPPER_STATUS_STATUS_SHIFT) == 2;
  }

  bool GripperBase::IsReady() const
  {
    return ((status.bytes[GRIPPER_STATUS_Index] & GRIPPER_STATUS_STATUS_MASK)>>GRIPPER_STATUS_STATUS_SHIFT) == 3;
  }

  bool GripperBase::AreAllFingersMoving() const
  {
    return ((status.bytes[GRIPPER_STATUS_Index] & GRIPPER_MOTION_STATUS_MASK) >> GRIPPER_MOTION_STATUS_SHIFT) == 0;
  }
  bool GripperBase::AreSomeFingersMoving() const
  {
    return AreAllFingersMoving() || AreSomeFingersStopped();
  }
  bool GripperBase::AreSomeFingersStopped() const
  {
    return ((status.bytes[GRIPPER_STATUS_Index] & GRIPPER_MOTION_STATUS_MASK) >> GRIPPER_MOTION_STATUS_SHIFT) == 1;
  }
  bool GripperBase::AreAllFingersStopped() const
  {
    return ((status.bytes[GRIPPER_STATUS_Index] & GRIPPER_MOTION_STATUS_MASK) >> GRIPPER_MOTION_STATUS_SHIFT) == 2;
  }
  bool GripperBase::AreAllFingersDone() const
  {
    return ((status.bytes[GRIPPER_STATUS_Index] & GRIPPER_MOTION_STATUS_MASK) >> GRIPPER_MOTION_STATUS_SHIFT) == 3;
  }
  bool GripperBase::IsActuatorMoving(int actuator) const
  {
    return status.bytes[OBJECT_DETECTION_Index] >> (actuator*2) == 0;
  }
  bool GripperBase::IsActuatorStopped(int actuator) const
  {
    return IsActuatorStoppedClosing(actuator) || IsActuatorStoppedOpening(actuator);
  }
  bool GripperBase::IsActuatorStoppedClosing(int actuator) const
  {
    return status.bytes[OBJECT_DETECTION_Index] >> (actuator*2) == 1;
  }
  bool GripperBase::IsActuatorStoppedOpening(int actuator) const
  {
    return status.bytes[OBJECT_DETECTION_Index] >> (actuator*2) == 2;
  }
  bool GripperBase::IsActuatorDone(int actuator) const
  {
    return status.bytes[OBJECT_DETECTION_Index] >> (actuator*2) == 3;
  }
  uint8_t GripperBase::GetFaultStatus() const
  {
    return status.bytes[FAULT_STATUS_Index];
  }

  bool GripperBase::SetMode(int mode)
  {
    if(mode < 0 || mode >= 4) {
      fprintf(stderr,"GripperBase::SetMode(%d): mode must be 0-3\n",mode);
      return false;
    }
    command[ACTION_REQUEST_Index] &= ~GRASPING_MODE_MASK;
    command[ACTION_REQUEST_Index] |= (mode << GRASPING_MODE_SHIFT);
    return SendCommand(command);
  }
  uint8_t GripperBase::GetMode() const
  {
    uint8_t v = status.bytes[GRIPPER_STATUS_Index];
    return (v & GRASPING_MODE_MASK) >> GRASPING_MODE_SHIFT;
  }
  bool GripperBase::Goto(uint8_t position)
  {
    command[POSITION_REQUEST_Index] = position;
    return SendActuatorCommands();
  }
  bool GripperBase::Goto(uint8_t position,uint8_t speed,uint8_t force)
  {
    command[POSITION_REQUEST_Index] = position;
    command[SPEED_Index] = speed;
    command[FORCE_Index] = force;
    return SendActuatorCommands();
  }
  bool GripperBase::Goto(int actuator,uint8_t position,uint8_t speed,uint8_t force)
  {
    if(actuator < 0 || actuator >= NumActuators) {
      fprintf(stderr,"GripperBase::Goto: invalid actuator %d\n",actuator);
      return false;
    }
    command[POSITION_REQUEST_Index+actuator*3] = position;
    command[SPEED_Index+actuator*3] = speed;
    command[FORCE_Index+actuator*3] = force;
    return SendActuatorCommands();
  }

  bool GripperBase::SetActuatorPosition(int actuator,uint8_t position)
  {
    if(actuator < 0 || actuator >= NumActuators) {
      fprintf(stderr,"GripperBase::SetActuatorPosition: invalid actuator %d\n",actuator);
      return false;
    }
    command[POSITION_REQUEST_Index+actuator*3] = position;
    return true;
  }
  bool GripperBase::SetActuatorSpeed(int actuator,uint8_t speed)
  {
    if(actuator < 0 || actuator >= NumActuators) {
      fprintf(stderr,"GripperBase::SetActuatorSpeed: invalid actuator %d\n",actuator);
      return false;
    }
    command[SPEED_Index+actuator*3] = speed;
    return true;
  }
  bool GripperBase::SetActuatorForce(int actuator,uint8_t force)
  {
    if(actuator < 0 || actuator >= NumActuators) {
      fprintf(stderr,"GripperBase::SetActuatorForce: invalid actuator %d\n",actuator);
      return false;
    }
    command[FORCE_Index+actuator*3] = force;
    return true;
  }

  bool GripperBase::SendActuatorCommands()
  {
    command[ACTION_REQUEST_Index] |= GOTO_MASK;
    bool res = SendCommand(command);
    command[ACTION_REQUEST_Index] &= ~GOTO_MASK;
    return res;
  }
  uint8_t GripperBase::GetActuatorPosition(int actuator) const
  {
    if(actuator < 0 || actuator >= NumActuators) {
      fprintf(stderr,"GripperBase::GetActuatorPosition: invalid actuator %d\n",actuator);
      return 0;
    }
    return status.bytes[FINGER_A_POSITION_Index+actuator*3];
  }
  uint8_t GripperBase:: GetActuatorSpeed(int actuator) const
  {
    if(actuator < 0 || actuator >= NumActuators) {
      fprintf(stderr,"GripperBase::GetActuatorSpeed: invalid actuator %d\n",actuator);
      return 0;
    }
    return command[SPEED_Index+actuator*3];
  }
  uint8_t GripperBase::GetActuatorPositionRequest(int actuator) const
  {
    if(actuator < 0 || actuator >= NumActuators) {
      fprintf(stderr,"GripperBase::GetActuatorPositionRequest: invalid actuator %d\n",actuator);
      return 0;
    }
    return status.bytes[POS_REQUEST_ECHO_Index+actuator*3];
  }

  uint8_t GripperBase::GetActuatorForce(int actuator) const
  {
    if(actuator < 0 || actuator >= NumActuators) {
      fprintf(stderr,"GripperBase::GetActuatorForce: invalid actuator %d\n",actuator);
      return 0;
    }
    return command[FORCE_Index+actuator*3];
  }

  bool GripperBase::SetAutomaticRelease()
  {
    command[ACTION_REQUEST_Index] |= AUTOMATIC_RELEASE_MASK;
    return SendCommand(command);
  }

  bool GripperBase::GetAutomaticRelease() const 
  {
    return (command[ACTION_REQUEST_Index] & AUTOMATIC_RELEASE_MASK) != 0;
  }


  bool GripperBase::SetIndividualFingerControl(bool enabled)
  {
    if(enabled) {
      if(command[GRIPPER_OPTION_1_Index] & INDIVIDUAL_FINGER_CONTROL_MASK)
	return true;
      command[FINGER_B_POSITION_Index] = command[POSITION_REQUEST_Index];
      command[FINGER_B_SPEED_Index] = command[SPEED_Index];
      command[FINGER_B_FORCE_Index] = command[FORCE_Index];
      command[FINGER_C_POSITION_Index] = command[POSITION_REQUEST_Index];
      command[FINGER_C_SPEED_Index] = command[SPEED_Index];
      command[FINGER_C_FORCE_Index] = command[FORCE_Index];
      command[GRIPPER_OPTION_1_Index] |= INDIVIDUAL_FINGER_CONTROL_MASK;
      return SendCommand(command);
    }
    else {
      if(command[GRIPPER_OPTION_1_Index] & INDIVIDUAL_FINGER_CONTROL_MASK) {
	command[GRIPPER_OPTION_1_Index] &= ~INDIVIDUAL_FINGER_CONTROL_MASK;
	return SendCommand(command);
      }
      return true;
    }
  }
  bool GripperBase::SetIndividualScissorControl(bool enabled)
  {
    if(enabled) {
      if(command[GRIPPER_OPTION_1_Index] & INDIVIDUAL_SCISSOR_CONTROL_MASK)
	return true;
      //TODO: set scissor value according to mode
      command[SCISSOR_POSITION_Index] = command[POSITION_REQUEST_Index];
      command[SCISSOR_SPEED_Index] = command[SPEED_Index];
      command[SCISSOR_FORCE_Index] = command[FORCE_Index];
      command[GRIPPER_OPTION_1_Index] |= INDIVIDUAL_SCISSOR_CONTROL_MASK;
      return SendCommand(command);
    }
    else {
      if(command[GRIPPER_OPTION_1_Index] & INDIVIDUAL_SCISSOR_CONTROL_MASK) {
	command[GRIPPER_OPTION_1_Index] &= ~INDIVIDUAL_SCISSOR_CONTROL_MASK;
	return SendCommand(command);
      }
      return true;
    }
  }
  bool GripperBase::GetIndividualFingerControl() const
  {
    return (command[GRIPPER_OPTION_1_Index] & INDIVIDUAL_FINGER_CONTROL_MASK) != 0;
  }
  bool GripperBase::GetIndividualScissorControl() const
  {
    return (command[GRIPPER_OPTION_1_Index] & INDIVIDUAL_SCISSOR_CONTROL_MASK) != 0;
  }


  bool GripperBase::WaitForReady(double maxseconds,int pollfreq)
  {
    int sleepus = 1000000/pollfreq;
    usleep(sleepus);
    if(!UpdateStatus()) return false;
    if(IsReady()) return true;
    int numiters = int(maxseconds*pollfreq);
    for(int i=0;i<numiters;i++) {
      usleep(sleepus);
      if(!UpdateStatus()) return false;
      if(IsReady()) return true;
    }
    return false;
  }
  int GripperBase::WaitForGoto(double maxseconds,int pollfreq)
  {
    int sleepus = 1000000/pollfreq;
    usleep(sleepus);
    if(!UpdateStatus()) return false;
    if(status.bytes[POS_REQUEST_ECHO_Index]==command[POSITION_REQUEST_Index] && !AreSomeFingersMoving()) {
      if(AreAllFingersDone()) return 1;
      return 0;
    }
    int numiters = int(maxseconds*pollfreq);
    for(int i=0;i<numiters;i++) {
      usleep(sleepus);
      if(!UpdateStatus()) return false;
      if(status.bytes[POS_REQUEST_ECHO_Index]==command[POSITION_REQUEST_Index] && !AreSomeFingersMoving()) {
	if(AreAllFingersDone()) return 1;
	return 0;
      }
    }
    return -1;
  }

  namespace ModbusRTU {

    const static int Baud = 115200;
    const static char Parity = 'N';
    const static int DataBits = 8;
    const static int StopBit = 1;
    const static int SlaveID = 9; 
    const static int InputFirstReg = 1000; 
    const static int OutputFirstReg = 2000; 


    Gripper::Gripper(const char* host)
    {
      mb = modbus_new_rtu(host, Baud, Parity, DataBits, StopBit);
      //modbus_set_debug(mb,1);
      int res = modbus_set_slave(mb, SlaveID);
      //printf("modbus_set_slave(9): %d (0 is OK)\n",res);
      if(res != 0) {
	modbus_close(mb);
	modbus_free(mb);
	mb = NULL;
      }
      res = modbus_connect(mb);
      //printf("modbus_connect(): %d (0 is OK)\n",res);
      if(res != 0) {
	modbus_close(mb);
	modbus_free(mb);
	mb = NULL;
      }
    }
    Gripper::~Gripper()
    {
      if(mb) {
	modbus_close(mb);
	modbus_free(mb);
      }
    }
    bool Gripper::Connected()
    {
      return mb != NULL && UpdateStatus() != false;
    }
    bool Gripper::SendCommand(const uint8_t bytes[16])
    {
      uint16_t words[8];
      memcpy(words,bytes,16);
      //big endian to host
      for (int i=0;i<8;i++)
	words[i]=htobe16(words[i]);//from endian.h
      int res = modbus_write_registers(mb, InputFirstReg, 8,words);
      //printf("modbus_write_registers(): %d (8 is OK)\n",res);
      if(res != 8) return false;
      return true;
    }
    bool Gripper::ReadStatus(Status& s)
    {
      /* Read 16 input registers from the address 1000 
	 only 8 are read because these are 16-bit words
      */
      int res = modbus_read_registers(mb, OutputFirstReg, 8, s.words);
      //printf("modbus_read_registers(): %d (8 is OK)\n",res);
      if(res != 8) return false;

      //big endian to host
      for (int i=0;i<8;i++)
	s.words[i]=be16toh(s.words[i]);//from endian.h
      return true;
    }
  } //namespace ModbusRTU
}//namespace Robotiq
