"""Automatic remote hosting of variables. See spiffy_test.py for an example
of how to use it.

Spiffy servers may perform logging, relay messages between clients, and
manage exclusive writes.

Each process should set a context on startup which will be used as
the namespace for all created variables.  A process can change to a different
context using the spiffy.setContext function.

Usage:
    from sspp import spiffy
    
    #connect to the spiffy server
    spiffy.start((server_ip,server,port),context=".my_context_name")
    
    foo = Spiffy('foo') #creates a variable named foo on the spiffy server under .my_context_name
    print foo.get() #hasn't been set yet, so this returns None
    foo.set(100)
    print foo.get() #will return 100

    #disconnect from the server
    spiffy.stop()
"""

import service
from threading import Thread,Lock
from collections import defaultdict
import time

_map = {}
_lock = Lock()
_context = ""
_pushVariables = defaultdict(list)
_processID = None
_variableCounter = 0
_writeOnly = {}
_pushThread = None
_pushService = None
_syncService = None

def setContext(context=""):
    global _context
    _context = context

def getContext():
    global _context
    return _context

class _PushService(service.Service):
    def __init__(self,host):
        global _map
        service.Service.__init__(self,map=_map)
        self.open(host)
        self.sendMessage({'type':'name','data':'Spiffy push thread'})
        self.sendMessage({'type':'subscribe','path':'._spiffy.null'})
    def checkWriteOnly(self,path):
        self.sendMessage({'type':'subscribe','path':'._spiffy.writeOnly'+path})
    def addPush(self,path,var):
        global _lock,_pushVariables
        with _lock:
            _pushVariables[path].append(var)
            self.sendMessage({'type':'subscribe','path':path})
    def removePush(self,path,var):
        global _lock,_pushVariables
        with _lock:
            _pushVariables[path].erase(var)
            if len(_pushVariables[path])==0:
                self.sendMessage({'type':'unsubscribe','path':path})
    def onMessage(self,msg):
        global _lock,_writeOnly,_pushVariables
        with _lock:
            if msg == None:
                #first message
                return
            if hasattr(msg,'__iter__') and 'path' in msg:
                if msg["path"].startswith('._spiffy.writeOnly'):
                    ofs = len('._spiffy.writeOnly')
                    _writeOnly[msg["path"][ofs:]] = msg["data"]
                else:
                    if msg["path"] not in _pushVariables:
                        print msg["path"]
                        raise RuntimeError("Got a message for a non-push variable")
                    for i in _pushVariables[msg["path"]]:
                        i._update(msg["data"])
            else:
                raise RuntimeError("PushService: invalid push message "+str(msg))

class _SyncService(service.Service):
    def __init__(self,host):
        global _map
        service.Service.__init__(self,map=_map)
        self.open(host)
        self.sendMessage({'type':'name','data':'Spiffy pull thread'})
        self.sendMessage({'type':'get','path':'_spiffy.clientCounter'})
        #print "Waiting for numclients get..."
        result = self.waitForMessage(sameThread=False)
        #print "  done"
        if result==None:
            self.id = 0
        else:
            self.id = result
        self.sendMessage({'type':'set','path':'_spiffy.clientCounter','data':(self.id+1)})

    def isWriteable(self,path,id):
        global _lock,_writeOnly
        _lock.acquire()
        #wait for first writeOnly message to be received
        while True:
            if path in _writeOnly:
                rights = _writeOnly[path]
                _lock.release()
                if rights == None: return True
                return rights['client']==self.id and rights['instance']==id
            _lock.release()
            print "Waiting for access rights for",path
            time.sleep(0.1)
            _lock.acquire()
        _lock.release()
        return True
        
    def markExclusiveWrite(self,path,enabled,id):
        global _lock,_writeOnly
        if enabled:
            if not self.isWriteable(path,id):
                print "Access violation: can't get exclusive write to",path,"for instance ID",id
                return False
            rights = {'client':self.id,'instance':id}
            self.sendMessage({'type':'set','path':'_spiffy.writeOnly'+path,'data':rights})
            with _lock:
                _writeOnly[path] = rights
        else:
            with _lock:
                if path not in _writeOnly:
                    return True
                if _writeOnly[path]['client'] != self.id or _writeOnly[path]['instance'] != id:
                    return True
                del _writeOnly[path]
            self.sendMessage({'type':'delete','path':'_spiffy.writeOnly'+path})
        return True


def _run_push_service(host):
    global _pushService
    _pushService.run()

def _interrupt_handler(signum,frame):
    print "Keyboard interrupt called, joining Spiffy thread..."
    stop()

def start(host=("localhost",4567),context=""):
    global _context,_pushThread,_pushService,_syncService
    _context = context
    _pushService = _PushService(host)
    #the push thread starts the asyncore loop
    _pushThread = Thread(target=_run_push_service,args=(host,))
    _pushThread.start()
    _syncService = _SyncService(host)

def stop():
    global _pushThread,_pushService,_syncService,_writeOnly,_pushVariables
    #unmark any write-only messages, unsubscribe from all messages
    for path,rights in _writeOnly.iteritems():
        if rights['client'] == _syncService.id:
            _pushService.sendMessage({'type':'delete','path':'._spiffy.writeOnly'+path})
    #hopefully the delte messages all get sent within this time?
    time.sleep(0.1)
    _writeOnly = {}
    _pushVariables = {}
    _pushService.terminate()
    _pushThread.join()
    _syncService = None
    _pushService = None

class Spiffy:
    """ A variable on a spiffy server.  Initialize it with a given name and context.

    They can be set to push mode, when the server sends updates, or pull mode, where
    the request is obtained only on a get() call.

    They can be set to exclusive write from a particular Spiffy variable instance
    to exclude other processes/instances from writing to them."""
    def __init__(self,name):
        global _lock,_context,_variableCounter,_pushService
        with _lock:
            self.id = _variableCounter
            _variableCounter += 1
        self.name = name
        self.value = None
        self.path = _context+"."+name
        self.pushMode = False
        _pushService.checkWriteOnly(self.path)
    def set(self,value):
        global _syncService,_pushService
        if not _syncService.isWriteable(self.path,self.id):
            print "Tried writing on an unwritable variable",self.path
            return False
        _syncService.sendMessage({'type':'set','path':self.path,'data':value})
        self.value = value
        return True
    def get(self):
        if self.pushMode:
            return self.value
        global _syncService
        _syncService.sendMessage({'type':'get','path':self.path})
        #print "Waiting for get",self.path,"..."
        self.value = _syncService.waitForMessage(sameThread=False)
        #print "Got",self.path
        return self.value
    def _update(self,value):
        #called by internal push thread for push variables
        self.value = value
    def setPullMode(self):
        global _pushService
        if self.pushMode:
            _pushService.removePush(self.path,self)
            self.pushMode = False
    def setPushMode(self):
        global _pushService
        if not self.pushMode:
            _pushService.addPush(self.path,self)
            self.pushMode = True
    def setExclusiveWrite(self,enabled=True):
        global _syncService
        return _syncService.markExclusiveWrite(self.path,enabled,self.id)
