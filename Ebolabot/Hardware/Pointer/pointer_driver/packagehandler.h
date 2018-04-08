/* Defintion of packagehandler struct, its initalizer, and some constants. */

#define byte uint8_t
#include <string.h> //needed for memcpy
#include <Arduino.h>
#include "nonblocktimer.h"

//Message identifiers
const char DATA_REQUEST = 'r';
const char DATA_MSG_START = 'd';

// Initalization Parameters
const int dataOut_LEN = 6;  // number of data elements (ints) to send
const int dataIn_LEN = 6;   // number of data elements (ints) to send
const int bytesIn_LEN = 10;  // number of bytes expected to read in from other device per complete message (equal to sum of inByteLens)

struct packagehandler {
  unsigned int dataOut[dataOut_LEN];  // data to send the other device upon its request
  unsigned int dataIn[dataIn_LEN];  // complete, up-to-date data recieved from the other device
  const int outByteLens[dataOut_LEN];  // byte length each data element being sent to other device
  const int inByteLens[dataIn_LEN];  // byte length of each data element being recieved from other device
  byte bytesIn[bytesIn_LEN];  // current input message being collected. should be zero intialized and have length sum inByteLens
  bool collectingData;
  int collectionIndx;
  bool readyForData;  // when True, handleSerial() will request more data on its next call. Turn true using doneWithDataIn() after handling dataIn.
  bool newDataIn;  // turns True when handleSerial() finishes collecting a data message. Turn false using doneWithDataIn() after handling dataIn.
  NonBlockTimer RequestWaitTimer;
};
typedef struct packagehandler PackageHandler; // must reference stuct within this file, but typedef will work externally

// PackageHandler Initalizer
struct packagehandler ConstructPkgHandler(int outByteLens[], int inByteLens[], int requestWaitTimeout) {
  /*
    Initalizes and returns a packagehandler struct, according to the Initalization Parameters
    
    requestWaitTimeout - maximum time in milliseconds handleSerial() will wait for new data (after sending a request) before requesting data again
  */
  NonBlockTimer RequestWaitTimer = {0, requestWaitTimeout};
  struct packagehandler ret_struct = { {0},{0},{0},{0},{0},false,0,true,false,RequestWaitTimer }; // initalize and fill all arrays and values with zero
  memcpy((void*) ret_struct.outByteLens, outByteLens, sizeof(ret_struct.outByteLens));
  memcpy((void*) ret_struct.inByteLens, inByteLens, sizeof(ret_struct.inByteLens));
  return ret_struct;
}

// Function prototypes (for organization)
void sendOutData(struct packagehandler *hndlr);
void updateDataIn(struct packagehandler *hndlr);
unsigned int unpack(byte msg[], int startIndex, int endIndex);
void handleSerial(struct packagehandler *hndlr);
void doneWithDataIn(struct packagehandler *hndlr);


void handleSerialAndPerformOnNewMsg(struct packagehandler *hndlr, void (*doOnNewMsg)(struct packagehandler*) ) {
  /* 
  Handles all aspects of serial by sending out the data from dataOut, reading in data to dataIn, and replaying to 
  data requests from the other device. This function should be called frequently (eg in a non-blocking main loop).
  
  Args:
    *hndlr - a pointer to a PackageHandler structure
    func - a function that accepts a PackageHandler
  */
  handleSerial(hndlr);
  if (hndlr->newDataIn) {
    doOnNewMsg(hndlr);
    doneWithDataIn(hndlr);
  }
}

///////////// Internal functions /////////////////////////////

void handleSerial(struct packagehandler *hndlr) {
  /* Interface between serial and dataIn and dataOut */
  if(hndlr->readyForData || hasElapsed(&hndlr->RequestWaitTimer)) {
    Serial.write(DATA_REQUEST);
    update(&hndlr->RequestWaitTimer);
    hndlr->readyForData = false;
  }
  while (Serial.available() > 0) {
    char rb = Serial.read();
    if(!hndlr->collectingData) {
      if (rb == DATA_REQUEST) {
          sendOutData(hndlr);
          continue;
      }
      else if (rb == DATA_MSG_START) {
          hndlr->collectingData = true;
      }
    }
    else {
      if (hndlr->collectionIndx < bytesIn_LEN-1) {
          hndlr->bytesIn[hndlr->collectionIndx] = rb;
          hndlr->collectionIndx += 1;
      }
      else {
          // recieved the final byte
          hndlr->bytesIn[hndlr->collectionIndx] = rb;
          updateDataIn(hndlr);
          hndlr->collectingData = false;
          hndlr->collectionIndx = 0;
          hndlr->newDataIn = true;
          return;
      }
    }
  }
}

void sendOutData(struct packagehandler *hndlr) {
  Serial.write(DATA_MSG_START);
  for(int i = 0; i < dataOut_LEN; i++) {
    if (hndlr->outByteLens[i] == 2) {
      Serial.write((hndlr->dataOut[i] >> 8) & 255);
      Serial.write(hndlr->dataOut[i] & 255);
    }
    else { // outByteLens[data_i] equals 1 (or is miswritten). This must be true b/c an int must be 1 or 2 bytes on the Arduino.
      Serial.write(hndlr->dataOut[i] & 255);
    }
  }
}

void doneWithDataIn(struct packagehandler *hndlr) {
  /* Marks current dataIn as being processed into external program and flags handler as being ready for 
  more data from other device, which will be retrieved on the next call of handleSerial() and overwrite
  dataIn. Should be called after processesing and handling dataIn */
  hndlr->readyForData = true;
  hndlr->newDataIn = false;
}

void updateDataIn(struct packagehandler *hndlr) {
  /* Converts completed *hndlr.bytesIn[] to integers and stores them in *hndlr.dataIn[] */
  int byte_i = 0;
  for(int data_i = 0; data_i < dataIn_LEN; data_i++) {
    int fieldLen = hndlr->inByteLens[data_i];
    hndlr->dataIn[data_i] = unpack(hndlr->bytesIn, byte_i, byte_i+fieldLen-1);
    byte_i += fieldLen;
  }
}

unsigned int unpack(byte msg[], int startIndex, int endIndex) {
  /* Given an array of bytes, a starting index, and endIndex, returns the unsigned integer value of the concatenation of
  those bytes (including those at the start and end indux). Thus, can be used to "unpack" byte arrays back to the origonal
  values that were stored there. */
  int retval = 0;
  for(int indx = startIndex; indx <= endIndex; indx++) {
    int pow = endIndex-indx;
    int add = msg[indx] << pow*8;
    retval += add;
  }
  return retval;
}
