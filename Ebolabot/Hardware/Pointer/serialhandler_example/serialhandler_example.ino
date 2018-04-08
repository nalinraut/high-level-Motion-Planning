#include <Servo.h>
#include "packagehandler.h"

////////// Setup Messaging ///////////////
int inByteLens[] = {1,2,1};
int outByteLens[] = {1,2,1};
PackageHandler hndlr = ConstructPkgHandler(outByteLens, inByteLens, 1000); // extra intialization must be done in packagehandler.h, under "Initalization Parameters"
////////// Done messaging setup ////////// 

void setup(){
  Serial.begin(115200);
  
  echoTest(&hndlr);
}

void loop(){
  
}


//////////  Test functions ///////////
void testOutput(PackageHandler *hndlr) {
  /*
    Sends the data [119,30841,122] over serial, which the Arduino serial monitor can display as ascii characters.
    Viewing the serial monitor, you should recieve 'dwxyz'.
  */
  int newDataOut[dataOut_LEN] = {119,30841,122};
  memcpy(hndlr->dataOut, newDataOut, sizeof(hndlr->dataOut));
  sendOutData(hndlr);
}

void echoDataIn(PackageHandler *hndlr)  {
  memcpy(hndlr->dataOut, hndlr->dataIn, sizeof(hndlr->dataOut));
}

void echoTest(PackageHandler *hndlr) {
  /*
    Allows for testing input and output over serial using serial monitor. Send data to arduino in the serial monitor by using
    strings starting with the DATA_MSG_START bit, and get that data echo'd back by writing the DATA_REQUEST. The message should
    be the DATA_MSG_START bit followed by 4 chars, per the intialization of the struct.
    
    Example: Enter 'd1234' in the serial monitor (it will return 'r'), then enter 'r'. You should recieve 'd1234'
  */
  
  while(true) {
    handleSerialAndPerformOnNewMsg(hndlr, &echoDataIn);
  }
}
