#!/usr/bin/python
from sspp.topic import *
import json
import socket

v = TopicValue(('localhost',4568),'.')
try:
    res = v.get()
    #pretty print JSON message
    print json.dumps(res, sort_keys=True, indent=4, separators=(',', ': '))
except socket.error:
    print
    print "Did you forget to start the system state server?"
    print
