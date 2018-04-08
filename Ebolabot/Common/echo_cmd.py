#!/usr/bin/python
from sspp.topic import *
import json
import socket

class TopicEchoService2 (TopicSubscriberBase):
    """An example class for how to subclass TopicSubscriberBase."""
    def onMessage(self,msg):
        print msg

v = TopicEchoService2(('localhost',4568),'.robot.command.qcmd')
try:
    v.run()
except socket.error:
    print
    print "Did you forget to start the system state server?"
    print


