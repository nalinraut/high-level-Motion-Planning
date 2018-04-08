/* 
 By Peter Moran.
 
 Dependencies:
   * Arduino 1.0.6 ( http://learn.trossenrobotics.com/index.php/getting-started-with-the-arbotix/7-arbotix-quick-start-guide )
   * ArbotiX-M Hardware and Library Files
       - IMPORTANT: libraries/Bioloid/ax12.cpp needs to be modified. In ax12SetRegister() and ax12SetRegister2(), uncomment
         the ax12ReadPacket() and make it ax12ReadPacket(6). This will prevent many errors that occur when mixing fast read
         and write speeds.
 
 Notes:
   * Make sure to set menu option "Tools->Board" to "ArbotiX"
   * If first upload does not work, try again. The Arduino should restart (with a chirp!) and then upload the second time.
 
 */

#include <Servo.h> 
#include <BioloidController.h>
#include "packagehandler.h"

///////////////////////////////////////////////////////
///                   Parameters                   ////
///////////////////////////////////////////////////////
// Dynamixel
const int DYNA_MSG_MS = 0;  // time needed between writing/reading to either Dynamixel. Do not set too low.

// Linear actuator
const int LIN_ISMOVE_DZN = 5;

// Messaging
int inByteLens[] = {2,2,2,1,1,2};
int outByteLens[] = {2,2,2,1,1,1};
// extra intialization must be done in packagehandler.h, under "Initalization Parameters"

///////////////////////////////////////////////////////
///             Hardware Definitions               ////
///////////////////////////////////////////////////////
// Dynamixel
BioloidController bioloid = BioloidController(1000000);
const int PAN_ID = 1;   // dynamixel ID of pan motor
const int TILT_ID = 2;  // dynamixel ID of tilt motor
const int CUR_VEL_RGSTR = 38; // register address for reading velocity

// Linear actuator
Servo linac;
const int LINAC_PIN = 12;
const int LIN_FDBK_PIN = 6; // purple wire to A7

// Buzzer
const int BUZZER_PIN = 16;

// Mode codes
const int PASS = 1;   // command to discard data when we recieve it, but continue with requests
const int MOVE = 0;   // for direct control of pan, tilt, ext
const int PRESS = 2;  // for extending until a specified force is detected

///////////////////////////////////////////////////////
///                    State                       ////
///////////////////////////////////////////////////////
PackageHandler hndlr = ConstructPkgHandler(outByteLens, inByteLens, 1000);
int cmdPan;    // long term pan target
int cmdTilt;   // long term tilt target
int cmdExt;    // long term ext target
int stepSize = 30;  // maximum change in written configuration per call of moveDynaTowards()
int mode = 0;  // desired mode based on mode codes
int modeParam;   // parameter to pass to mode (used only in some cases)
int lastWrittenConfig[] = {0, 0};  // last [pan, tilt] values written to Dynamixel registers
unsigned long nextWriteTime[] = {0, 0};  // next time to write to [pan, tilt] Dynamixel registers

///////////////////////////////////////////////////////
///                   MAIN                         ////
///////////////////////////////////////////////////////
void setup(){
  // Beep
  pinMode(BUZZER_PIN, OUTPUT);
  beep();
  
  Serial.begin(115200);
  linac.attach(LINAC_PIN);
  
  delay(50);
  internalHalt();
}

void loop(){
  updateDataOut(&hndlr);
  handleSerialAndPerformOnNewMsg(&hndlr, &processDataIn);
  doCommands();
}

///////////////////////////////////////////////////////
///                     Control                    ////
///////////////////////////////////////////////////////
void doCommands() {
  /* Executes on commands save into State variables. Should be called regularly. */
  if(mode == PRESS) {
    
  }
  else if(mode == MOVE) {
    moveDynaTowards(PAN_ID, cmdPan);
    moveDynaTowards(TILT_ID, cmdTilt);
    setExt(cmdExt);
  }
}

void processDataIn(PackageHandler *hndlr) {
  int newMode = hndlr->dataIn[4];
  if (newMode != PASS && cmdPan != NULL) { // assume full message if cmdPan isnt null. Intended so that nulls are no used on startup, rather than for error checking.
    cmdPan = hndlr->dataIn[0];
    cmdTilt = hndlr->dataIn[1];
    cmdExt = hndlr->dataIn[2];
    stepSize = hndlr->dataIn[3];
    mode = newMode;
    modeParam = hndlr->dataIn[5];
  }
}

void updateDataOut(PackageHandler *hndlr) {
  hndlr->dataOut[0] = getDynaPos(PAN_ID);
  hndlr->dataOut[1] = getDynaPos(TILT_ID);
  hndlr->dataOut[2] = currExt();
  hndlr->dataOut[3] = moving();
  hndlr->dataOut[4] = extVel();
  hndlr->dataOut[5] = linOverload();
}

///////////////////////////////////////////////////////
///                    Movement                    ////
///////////////////////////////////////////////////////
void moveDynaTowards(int id, int pos) {
  /* Commands Dynamixel to move 1 step size in the direction towards the desired configuration. By calling this continually
  untill the desired position is reached, the Dynamixel motors will move at a safe and stable speed */
  unsigned long time = millis();
  int lastPosWrittenToDyna = lastWrittenConfig[id-1];
  if(time >= nextWriteTime[id-1]) {
    // If the step is small enough, go there immediatly
    if((lastPosWrittenToDyna - pos) <= stepSize && (lastPosWrittenToDyna - pos) >= -stepSize) {
      commandDynaTo(id, pos);
    }
    // If step is too large, just go in the right direction
    else {
      if(pos > lastPosWrittenToDyna) {
        commandDynaTo(id, lastPosWrittenToDyna+stepSize);
      }
      if(pos < lastPosWrittenToDyna) {
        commandDynaTo(id, lastPosWrittenToDyna-stepSize);
      }
    }
    nextWriteTime[id-1] = time + DYNA_MSG_MS;
  }
}

void commandDynaTo(int id, int pos) {
  // Sets target position for dynamixel directly to its register
  SetPosition(id, pos);
  lastWrittenConfig[id-1] = pos;
}

void setExt(int pulseWidth) {
  if(pulseWidth < 1000) pulseWidth = 1000;
  if(pulseWidth > 2000) pulseWidth = 2000;
  linac.writeMicroseconds(pulseWidth);
}

void pressTo(int overloadLim) {
  // needs overload function to be fixed
}

void internalHalt() {
  /* Sets goal position to current position, effectivly causing a stop on doCommands().
  In practice, doCommands() needs to be continually called in main loop for halt to work.
  This will be overwritten by the next message recieved, and thus only works in specific cases*/
  cmdPan = getDynaPos(PAN_ID);
  cmdTilt = getDynaPos(TILT_ID);
  cmdExt = currExt();
  lastWrittenConfig[0] = cmdPan;
  lastWrittenConfig[1] = cmdTilt;
  mode = MOVE; // to prevent drifting
  doCommands();
}

///////////////////////////////////////////////////////
///                    Feedback                    ////
///////////////////////////////////////////////////////
void beep() {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(8);
  digitalWrite(BUZZER_PIN, LOW);
}

int currExt() {
  // Emperically determined analog reading to pulse-width command conversion
  int analogLow = 650;
  int pwmLow = 50;
  int analogHigh = 441;
  int pwmHigh = 500;
  return map(analogRead(LIN_FDBK_PIN), analogLow, analogHigh, pwmLow, pwmHigh)+1000; // +1000 is to correct for the different value range when map was emperically found
}

int getDynaPos(int motorID) {
  return ax12GetRegister(motorID,36,2);
}

boolean moving() {
  boolean panMov = ax12GetRegister(PAN_ID, CUR_VEL_RGSTR, 2) != 0;
  boolean tiltMov = ax12GetRegister(TILT_ID, CUR_VEL_RGSTR, 2) != 0;
  int curExt = currExt();
  boolean linacMov = extVel() > LIN_ISMOVE_DZN;
  return (panMov<<2) || (tiltMov<<1) || linacMov;
}

int extVel() {
  // TODO: replace with Kalman filter
  return 0; // for compatability until extension position and velocity are fixed
}

int linOverload() {
  return 0; // for compatability until extension position and velocity are fixed
}
