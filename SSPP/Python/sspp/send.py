#!/usr/bin/python

from service import Service
import time
import sys
import json

class SendService (Service):
    """A service that sends a message as a client to a server"""
    def __init__(self,addr,message,repeat=1,asServer=False):
        Service.__init__(self)
        
        self.open(addr,asServer=asServer)
        time.sleep(0.01)
        for i in xrange(repeat):
            print "Sending",message
            self.sendMessage(message)
        time.sleep(0.01)
        return
    def onUpdate(self):
        if not self.writable(): #done sending
            self.close()
            return False
        return True

if __name__ == "__main__":
    addr = ('localhost',4567)
    repeat = 1
    asServer = False
    i = 1
    while i < len(sys.argv):
        if sys.argv[i][0] != '-':
            #not an option
            break
        if sys.argv[i] == '-r':
            repeat = int(sys.argv[i+1])
            i += 1
        elif sys.argv[i] == '-s':
            addr = sys.argv[i+1].split(':')
            if len(addr)==1:
                addr = [addr,4567]
            addr[1] = int(addr[1])            
            asServer = True
        elif sys.argv[i] == '-c':
            addr = sys.argv[i+1].split(':')
            if len(addr)==1:
                addr = [addr,4567]
            addr[1] = int(addr[1])
            asServer = True
        elif sys.argv[i] == '-h':
            print "USAGE: send.py [options] message1 message2 ..."
            print "OPTIONS:"
            print " -r n: repeat n times"
            print " -s addr: listen as server on the given address and send message to clients"
            print " -c addr: connect as client to the given address"
        else:
            print "Unknown option",sys.argv[i]
        i += 1
    if i == len(sys.argv):
        print "No messages specified on command line"
        exit(-1)
    while i < len(sys.argv):
        msg = json.loads(sys.argv[i])
        s = SendService(addr,msg,repeat,asServer)
        s.run(0.01)
        i += i
