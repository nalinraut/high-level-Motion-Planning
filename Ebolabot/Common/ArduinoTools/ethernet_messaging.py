__author__ = 'Peter Moran'
""" Simple script for sending string data to an Arduino over Ethernet """

from socket import socket, AF_INET, SOCK_DGRAM
import time

arduino_addr = ('192.168.1.120', 5000)
arduino_socket = socket(AF_INET, SOCK_DGRAM)
arduino_socket.settimeout(1)

data = "<Your Message To Arduino>"
arduino_socket.sendto(data, arduino_addr)
try:
    rec_data, addr = arduino_socket.recvfrom(2048)
    print rec_data
except:
    pass