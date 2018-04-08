__author__ = 'Peter Moran'
"""
Sends Ethernet messages to an Arduino at a specified IP address in order to turn Baxter
on and off via a relay. See README for more operation details.

Peter Moran, August 2015.
Based on code from http://www.toptechboy.com/tutorial/python-with-arduino-lesson-16-simple-client-server-configuration-over-ethernet/

NOTE:
If the IP address of the Arduino changes, make sure to change the address below. To find
the new IP address of the Arduino first, follow the instructions in the README.
"""

from socket import socket, AF_INET, SOCK_DGRAM

arduino_addr = ('192.168.1.120', 5000)  # Change this address if the Arduino IP changes
arduino_socket = socket(AF_INET, SOCK_DGRAM)
arduino_socket.settimeout(1)

def enabled():
    """Returns True if Arduino is set up and can receive and respond to messages."""
    data = "<PING>"
    arduino_socket.sendto(data, arduino_addr)
    try:
        rec_data, addr = arduino_socket.recvfrom(2048)
        if rec_data == ">PONG<":
            return True
    except:
        return False

def togglePower():
    """Command the Arduino to toggle power. Returns True if the Arduino receives the message."""
    data = "<TOGGLE_POWER>"
    arduino_socket.sendto(data, arduino_addr)
    try:
        rec_data, addr = arduino_socket.recvfrom(2048)
        if rec_data == ">POWER_TOGGLED<":
            return True
    except:
        return False