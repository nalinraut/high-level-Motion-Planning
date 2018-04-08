#ifndef ROBOTIQ_API_HPP
#define ROBOTIQ_API_HPP

#include <stdio.h>
#include <stdint.h>

//forward declaration of modbus
struct _modbus;
typedef struct _modbus modbus_t;

//these are defined by Robotiq
namespace Robotiq {
  const static char* Name = "3-finger Adaptive Gripper";
  const static int NumFingers = 3;
  const static int NumActuators = 4;

  typedef union {
    uint8_t bytes[16];
    uint16_t words[8];
  } Status;

  class GripperBase
  {
  public:
    GripperBase();
    virtual ~GripperBase() {}

    //high level API
    bool Activate();
    bool Deactivate();
    bool UpdateStatus() { return ReadStatus(status); }
    void PrintStatus(FILE* f=stdout);

    //status querying
    bool IsReset() const;
    bool IsActivated() const;
    bool IsChangingMode() const;
    bool IsActivating() const;
    bool IsReady() const;
    uint8_t GetFaultStatus() const;
    //change the mode
    bool SetMode(int mode);
    bool SetBasicMode() { return SetMode(0); }
    bool SetPinchMode() { return SetMode(1); }
    bool SetWideMode() { return SetMode(2); }
    bool SetScissorMode() { return SetMode(3); }
    uint8_t GetMode() const;
    //sends the given basic mode command
    bool Goto(uint8_t position);
    //sends the given basic mode command
    bool Goto(uint8_t position,uint8_t speed,uint8_t force);
    //sends the following basic mode command
    bool Goto(int actuator,uint8_t position,uint8_t speed,uint8_t force);
    //the following queue up commands until the next SendActuatorCommands or
    //Goto command
    bool SetActuatorPosition(int actuator,uint8_t position);
    bool SetActuatorSpeed(int actuator,uint8_t speed);
    bool SetActuatorForce(int actuator,uint8_t force);
    //sends queued up SetActuatorX commands
    bool SendActuatorCommands();
    uint8_t GetActuatorPosition(int actuator) const;
    uint8_t GetActuatorSpeed(int actuator) const;
    uint8_t GetActuatorPositionRequest(int actuator) const;
    uint8_t GetActuatorForce(int actuator) const;
    bool AreAllFingersMoving() const;
    bool AreSomeFingersMoving() const;
    bool AreSomeFingersStopped() const;
    bool AreAllFingersStopped() const;
    bool AreAllFingersDone() const;
    bool IsActuatorMoving(int actuator) const;
    bool IsActuatorStopped(int actuator) const;
    bool IsActuatorStoppedClosing(int actuator) const;
    bool IsActuatorStoppedOpening(int actuator) const;
    bool IsActuatorDone(int actuator) const;
    //on emergency stop -- can't address gripper after this
    bool SetAutomaticRelease();
    bool GetAutomaticRelease() const;
    //turn on individual finger / scissor control
    bool SetIndividualFingerControl(bool enabled=true);
    bool SetIndividualScissorControl(bool enabled=true);
    bool GetIndividualFingerControl() const;
    bool GetIndividualScissorControl() const;

    //Helpers.
    //Returns true if ready state was reached
    bool WaitForReady(double maxseconds=10,int pollfreq = 50);
    //Returns 1 if fingers reached destination, 0 if stopped, -1 if maxseconds time elapsed
    int WaitForGoto(double maxseconds=10,int pollfreq = 50);

    //must be overridden by subclasses
    virtual bool Connected()=0;
    virtual bool SendCommand(const uint8_t bytes[16])=0;
    virtual bool ReadStatus(Status& s)=0;

    Status status;
    uint8_t command[16];
  };

  namespace ModbusRTU { 

    class Gripper : public GripperBase
    {
    public:
      Gripper(const char* port);
      virtual ~Gripper();
      virtual bool Connected();
      virtual bool SendCommand(const uint8_t bytes[16]);
      virtual bool ReadStatus(Status& s);

      modbus_t *mb;
    };

  } //namespace ModbusRTU

} //namespace Robotiq

#endif
