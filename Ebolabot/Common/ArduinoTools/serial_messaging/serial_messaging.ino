/*
 Robust, non blocking way of transfering data from a PC to Arduino.
 String commands are sent over serial to the Arduino. Only data inisde
 the wedge brackets (ie "<data>") will be processed.
 
 To extend / modify the capabilities, simply follow the pattern seen
 in parseMessage() to add more fields or to change the data type parsing.
 
 Modified from code posted by [Robin2] at http://forum.arduino.cc/index.php?topic=288234.0
 */

// Messaging parameters
#define INIT_MSG "13,3.7"
#define SERIAL_BAUD 9600

// Messaging
const byte numChars = 32;
char message[numChars] = INIT_MSG;
char tempChars[numChars];
char rc;
char startMarker = '<';
char endMarker = '>';

// Message variable
char someString[32] = {0};
int someInteger = 0;
float someFloat = 0.0;

char rc; // Recieved character


void setup() {
    Serial.begin(SERIAL_BAUD);
    Serial.println("[ Arduino is ready ]");
}

void loop() {
    updateValues();
    showParsedData(); // DEMO, remove when copied
    delay(2000);      // DEMO, remove when copied
}

void updateValues() {
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

/* This is where the data sent from the computer is stored into our
messaging values. To change the store values and types, make sure to
follow the conventions below. The lines are in pairs. The first gets
the field, the second converts to the correct type.

IMPORTANT: Make sure you get the fields the correct way. Note how we
get the first field differently from subsequent ones. How we convert
the fields does not matter though.
/*
void parseMessage() {

    // split the data into its parts

    char * strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(tempChars,",");      // Get the first field,
    strcpy(messageFromPC, strtokIndx);       // interpret and save as a string

    strtokIndx = strtok(NULL, ",");          // Get the next field,
    someInteger = atoi(strtokIndx);        // interpret and save as an integer

    strtokIndx = strtok(NULL, ",");          // Get the next field,
    someFloat = atof(strtokIndx);          // interpret and save as a float

}

// DEMO, remove when copied
void showParsedData() {
    Serial.print("Message ");
    Serial.println(messageFromPC);
    Serial.print("Integer ");
    Serial.println(integerFromPC);
    Serial.print("Float ");
    Serial.println(floatFromPC);
}
