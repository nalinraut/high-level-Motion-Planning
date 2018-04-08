/*
  A motorized slide potentiometer controller for reading potentiometer, touch,
 and button states and for setting motor positions via serial communication.
 
 Currently designed for a 7 slider, 3 button board. Simple changes to the code
 can expand to more sliders and buttons by adding more instances of stuctures
 and expanding the print and read statements.
 
 Peter Moran
 August 2015
 */
#include "WConstants.h"
#include <Bounce2.h>
#include <PID_v1.h>
#include "v2_teensy.h"
#include "slider.h"

/////////////////////////////////////////
////     HARDWARE     ////
#define NUM_SLIDERS 7

// Messaging parameters
int lastMsgSnd = 0;
#define MSSG_MS 10
const byte numChars = 32;
char message[numChars] = "500,500,500,500,500,500,500";
char tempChars[numChars];
char rc;
char startMarker = '<';
char endMarker = '>';

// Debouncers
Bounce d0 = Bounce();
Bounce d1 = Bounce();
Bounce d2 = Bounce();
#define BOUNCE_INTVL 3

// Sliders
Slider s0 = {P0, M0_SPD, M0_DIR, T0};
Slider s1 = {P1, M1_SPD, M1_DIR, T1};
Slider s2 = {P2, M2_SPD, M2_DIR, T2};
Slider s3 = {P3, M3_SPD, M3_DIR, T3};
Slider s4 = {P4, M4_SPD, M4_DIR, T4};
Slider s5 = {P5, M5_SPD, M5_DIR, T5};
Slider s6 = {P6, M6_SPD, M6_DIR, T6};
Slider *sliders[NUM_SLIDERS] = {&s0, &s1, &s2, &s3, &s4, &s5, &s6};

// PID struts
PID pid0(&s0.currPos, &s0.pid_out, &s0.target, kp*kScale, ki*kScale, kd*kScale, DIRECT);
PID pid1(&s1.currPos, &s1.pid_out, &s1.target, kp*kScale, ki*kScale, kd*kScale, DIRECT);
PID pid2(&s2.currPos, &s2.pid_out, &s2.target, kp*kScale, ki*kScale, kd*kScale, DIRECT);
PID pid3(&s3.currPos, &s3.pid_out, &s3.target, kp*kScale, ki*kScale, kd*kScale, DIRECT);
PID pid4(&s4.currPos, &s4.pid_out, &s4.target, kp*kScale, ki*kScale, kd*kScale, DIRECT);
PID pid5(&s5.currPos, &s5.pid_out, &s5.target, kp*kScale, ki*kScale, kd*kScale, DIRECT);
PID pid6(&s6.currPos, &s6.pid_out, &s6.target, kp*kScale, ki*kScale, kd*kScale, DIRECT);
PID *pids[NUM_SLIDERS] = {&pid0, &pid1, &pid2, &pid3, &pid4, &pid5, &pid6};

/////////////////////////////////////////

void setup() {
  // Begin serial communication
  Serial.begin(115200); 

  // Configure buttons
  pinMode(BTN_0, INPUT);
  pinMode(BTN_1, INPUT);
  pinMode(BTN_2, INPUT);
  d0.attach(BTN_0);
  d1.attach(BTN_1);
  d2.attach(BTN_2);
  d0.interval(BOUNCE_INTVL);
  d1.interval(BOUNCE_INTVL);
  d2.interval(BOUNCE_INTVL);

  // Configure PID
  for (int i = 0; i < NUM_SLIDERS; i++) {
    pids[i]->SetMode(AUTOMATIC);
    pids[i]->SetOutputLimits(-PID_OUT_MAG, PID_OUT_MAG);
    pids[i]->SetSampleTime(PID_MS);
  }

  // Configure sliders and initilize states
  for (int i = 0; i < NUM_SLIDERS; i++) {
    sliders[i]->touch_count = 0;
    sliders[i]->pid = pids[i];
    sliders[i]->target = 500;
    updateState(sliders[i]);
    enableSlider(sliders[i]);
  }
}

void loop() {
  // Read data from computer and go
  getCmdPos();
  updateStateAndGo();

  // Read buttons
  d0.update();
  d1.update();
  d2.update();
  boolean b1 = d0.read();
  boolean b2 = d1.read();
  boolean b3 = d2.read();
  
  // Send message to computer
  unsigned long timeNow = millis();
  if (timeNow - lastMsgSnd > MSSG_MS) {
    for (int i = 0; i < NUM_SLIDERS; i++) {
      int num = (int) sliders[i]->currPos;
      boolean touch = sliders[i]->touched;
      Serial.write(touch << 7 | num >> 8);
      Serial.write(num);
    }
    Serial.write(0);
    Serial.write(b1 << 2 | b2 << 1 | b3);
    Serial.print("\n");
    lastMsgSnd = timeNow;
  }
}

//////////////////////////////////////////////////////////

// Update each slider struct to the physical readings and go to its commanded position 
void updateStateAndGo() {
  for (int i = 0; i < NUM_SLIDERS; i++) {
    updateState(sliders[i]);
    slideToTarget(sliders[i]);
  }
}

// Set all sliders to follow another slider. Helpful for debugging.
void follow(int num, Slider *sldr) {
  for (int i = 0; i < NUM_SLIDERS; i++) {
    if (i != num) {
      sliders[i]->target = sldr->currPos;
    }
  }
}

///////////////////////////////////////////////////////////////////////
////////////           MESSAGING            /////////////
///////////////////////////////////////////////////////////////////////

void getCmdPos() {
  strcpy(tempChars, message);
  readMessage();
  parseMessage();
}

void readMessage() {
  static boolean recvInProgress = false;
  static byte ndx = 0;

  while (Serial.available() > 0) {
    rc = Serial.read();
    if (recvInProgress == true) {
      if (rc != endMarker) {
        message[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      }
      else {
        message[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
      }
    }

    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}

void parseMessage() {

  // split the data into its parts

  char * strtokIndx; // this is used by strtok() as an index

  strtokIndx = strtok(tempChars,",");      // Get the first field,
  s0.target = atoi(strtokIndx);                    // interpret and save as an integer

  strtokIndx = strtok(NULL, ",");          // Get the next field,
  s1.target = atoi(strtokIndx);               // interpret and save as an integer

  strtokIndx = strtok(NULL, ",");          // Get the next field,
  s2.target = atoi(strtokIndx);               // interpret and save as an integer

  strtokIndx = strtok(NULL, ",");          // Get the next field,
  s3.target = atoi(strtokIndx);               // interpret and save as an integer

  strtokIndx = strtok(NULL, ",");          // Get the next field,
  s4.target = atoi(strtokIndx);               // interpret and save as an integer

  strtokIndx = strtok(NULL, ",");          // Get the next field,
  s5.target = atoi(strtokIndx);               // interpret and save as an integer

  strtokIndx = strtok(NULL, ",");          // Get the next field,
  s6.target = atoi(strtokIndx);              // interpret and save as an integer
  
  strtokIndx = strtok(NULL, ",");          // Get the next field,
  forceFollow = atoi(strtokIndx) != 0;              // interpret and save as an integer

}

