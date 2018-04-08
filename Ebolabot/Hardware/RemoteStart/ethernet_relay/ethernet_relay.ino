/*
Turns Baxter on and off via Ethernet commands, intended to be sent by accompanying Python code.
This is done by simulating a button press using a relay.

Peter Moran
August 2015
 
 Based on code from http://www.toptechboy.com/tutorial/python-with-arduino-lesson-16-simple-client-server-configuration-over-ethernet/
 */

#include <Ethernet.h>
#include <EthernetUdp.h>
#include <SPI.h>

const int CTRL = 8;

unsigned int localPort = 5000;
IPAddress ip(192, 168, 1, 120); // 192.168.1.120
byte macAddr[] = {  
  0x00, 0xAA, 0xBB, 0xCC, 0xDE, 0x02 };

EthernetUDP Ether;
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];
String datReq;
int packetSize;

void setup() {
  Serial.begin(9600);
  Ethernet.begin(macAddr);
  Ether.begin(localPort);
  delay(1500);

  pinMode(CTRL, OUTPUT);
  digitalWrite(CTRL, LOW);
}

void loop() {
  packetSize = Ether.parsePacket();
  if(packetSize>0) {
    Ether.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
    String datReq(packetBuffer);

    if (datReq =="<TOGGLE_POWER>") {
      // Use relay
      digitalWrite(CTRL, HIGH);
      delay(500);
      digitalWrite(CTRL, LOW);

      // Respond
      Ether.beginPacket(Ether.remoteIP(), Ether.remotePort()); 
      Ether.print(">POWER_TOGGLED<");
      Ether.endPacket();
    }
    if (datReq =="<PING>") {
      // Respond
      Ether.beginPacket(Ether.remoteIP(), Ether.remotePort()); 
      Ether.print(">PONG<");
      Ether.endPacket();
    }
  }
  memset(packetBuffer, 0, UDP_TX_PACKET_MAX_SIZE);
}



