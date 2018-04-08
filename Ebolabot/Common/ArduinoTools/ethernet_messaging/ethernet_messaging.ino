/*
Simple script for reading string data over ethernet using an Arduino and an Arduino Ethernet Shield.

Based on code from http://www.toptechboy.com/tutorial/python-with-arduino-lesson-16-simple-client-server-configuration-over-ethernet/
*/

#include <Ethernet.h>
#include <EthernetUdp.h>
#include <SPI.h>

unsigned int localPort = 5000;
IPAddress ip(192, 168, 1, 120); // 192.168.1.120
byte mac[] = {  
  0x00, 0xAA, 0xBB, 0xCC, 0xDE, 0x02 };

EthernetUDP Udp;
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];
String datReq;
int packetSize;

void setup() {
  Serial.begin(9600);
  Ethernet.begin(mac);
  Udp.begin(localPort);
  delay(1500);
}

void loop() {
  packetSize = Udp.parsePacket();
  if(packetSize>0) {
    Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
    String datReq(packetBuffer);
    
    // datReq now equals a string of data sent from the PC

    if (datReq =="Red") {
      // To respond, use
      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort()); 
      Udp.print("You are Asking for Red");
      Udp.endPacket();
    }
  }
  memset(packetBuffer, 0, UDP_TX_PACKET_MAX_SIZE);
}


