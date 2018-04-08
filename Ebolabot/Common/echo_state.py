#!/usr/bin/python
from sspp.topic import *
import json
import socket
import os
import sys
ebolabot_root = os.getenv("EBOLABOT_PATH",".")
sys.path.append(ebolabot_root)
from Common.system_config import EbolabotSystemConfig

system_state_addr = EbolabotSystemConfig.getdefault_ip('state_server_computer_ip',('localhost',4568))

class TopicEchoService2 (TopicSubscriberBase):
    """An example class for how to subclass TopicSubscriberBase."""
    def onMessage(self,msg):
        print json.dumps(msg,sort_keys="True",indent=4,separators=(',',': '))

topic = sys.argv[1] if len(sys.argv) > 1 else '.'
v = TopicEchoService2(system_state_addr,topic)
try:
    v.run()
except socket.error:
    print
    print "Did you forget to start the system state server? searching on",system_state_addr
    print


