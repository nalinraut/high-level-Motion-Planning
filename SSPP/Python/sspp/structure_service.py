from service import Service
import time
import json

#verbosity levels
QUIET = 0
ERRORS = 1
NOTIFICATIONS = 2
ALL = 3
DEBUG = 4

class SubscriberData:
    def __init__(self,client,path,rate=0):
        """Sets up client to subscribe to a sub-object, specified by a path.
        A rate of <= 0 indicates that a message should only be sent when
        that sub-object changes"""
        self.client = client
        self.path = path
        self.rate = rate
        if rate <= 0:
            self.nextSendTime = None
        else:
            self.nextSendTime = time.time()

def toKeys(path):
    keys = path.split('.')
    if len(keys) == 0:
        return keys
    if keys[0] == '':
        #starts with a ., signifies first element
        keys = keys[1:]
        return keys
    return keys

def isancestorpath(path1,path2):
    """Returns True if path1 is an ancestor of path2 (including itself)"""
    if not path2.startswith(path1): return False
    #false positives: if path2 is "abcd" and path1 is "abc", the prior test
    #passes
    if len(path2) == len(path1): return True
    return path2[len(path1)]=='.'

def safeget(o,element,add = False):
    if isinstance(o,list):
        index = int(element)
        if index < 0:
            raise "Invalid array index",index
        if index >= len(o) and add == False: return None
        while index >= len(o):
            o.append({})
        return  o[index]
    else:
        if element not in o:
            if add == False: return None
            o[element] = {}
        return o[element]

def safeset(o,element,value):
    if isinstance(o,list):
        index = int(element)
        if index < 0:
            raise "Invalid array index",index
        while index >= len(o):
            o.append({})
        o[index] = value
    else:
        if element not in o:
            o[element] = {}
        o[element] = value

def safedel(o,element):
    if isinstance(o,list):
        index = int(element)
        if index < 0:
            raise "Invalid array index",index
        if index >= len(o):
            return False
        o[index] = {}
    else:
        if element not in o:
            return False
        del o[element]
    return True

class StructureService (Service):
    """A service that allows subscribers to observe a structure and
    publishers to change the structure.

    All incoming messages can specify a path, so that only a sub-structure
    given is given or changed.  The path of the desired sub-structure is
    given in the form '.key1.key2.key3' or '' to specify the whole structure.

    For example, if the structure is {'foo':1,'bar':[2,3,4]} then
    the 'get' message with path '.bar.2' would return 4.

    Subscribers are either sent messages at a regular rate, or are only
    notified asynchronously when the object has changed.  It will be smart
    about sending messages only when the given object is touched.  So, in
    the above example, if client 1 is subscribed to '.bar.2' and client 2
    changes 'foo', '.bar.X' where X!=2, or adds a new element to the root,
    client 1 will not be notified.

    Subscribers are sent a message "SSPP StructureService" on connect.

    Incoming messages are of the form:
        - { type:'name',[data:NAME] }: Sets the name of the client for debug
          messages. If data is not provided, then it uses the default
        - { type:'subscribe',[path:PATH],[rate:RATE (in Hz)] }:
          Begins sending messages about the structure to the client.
          If rate is specified and positive, the object is sent at a regular
          rate.  Otherwise, it is sent only when changed.
        - { type:'unsubscribe',[path:PATH] }:
          Stops sending messages about the structure to the client.
        - { type:'get',[path:PATH] }: Sends one message about the
          structure to the client.
        - { type:'set',[path:PATH],data:DATA }: Sets the specified object to
          DATA.  Notifies subscribers.
        - { type:'change',[path:PATH],data:DATA }: Changes entries in the
          specified object to the entries in DATA.  Only the keys in DATA
          are modified.  Notifies subscribers.
        - { type:'delete',[path:PATH} }: deletes a specified object.
        - { type:'resize',[path:PATH},data:VALUE }: resizes the specified
          object to a given (nonnegative integer) size.
    """
    def __init__(self):
        Service.__init__(self)
        self.verbosity = ERRORS
        self.object = {}
        self.subscribers = {}
        self.numInMessages = 0
        self.timeInMessageProcess = 0
        self.numOutMessages = 0
        self.timeOutMessageProcess = 0

    def onClientAccept(self,client):
        #if self.verbosity >= DEBUG: print 'SSPP structure service client accepted, sending "SSPP StructureService" initialization message'
        #client.sendMessage("SSPP StructureService")
        pass

    def onClientClose(self,client):
        if client.addr in self.subscribers:
            if self.verbosity >= NOTIFICATIONS:
                print "Normal disconnect of",str(client),"from topics",
            for s in self.subscribers[client.addr]:
                print s.path,
            print
            del self.subscribers[client.addr]
        Service.onClientClose(self,client)
        return
        
    def onClientMessage(self,client,msg):
        if 'type' not in msg:
            if self.verbosity >= ERRORS: print "Message from client",str(client),"does not have 'type' member"
            return
        t0 = time.time()
        path = ''
        if 'path' in msg:
            path = msg['path']
            if len(path) > 0 and path[0] == '.':
                path = path[1:]
        #print "Message type",msg['type']
        if msg['type'] == 'name':
            if self.verbosity >= NOTIFICATIONS: print "Renaming client",str(client),"to",
            if 'data' in msg:
                client.name = msg['data']
            else:
                client.name = None
            if self.verbosity >= NOTIFICATIONS: print str(client)
        elif msg['type'] == 'subscribe':
            defaultRate = 0
            rate = (msg['rate'] if 'rate' in msg else defaultRate)
            self.subscribe(client,path,rate)
        elif msg['type'] == 'unsubscribe':
            self.unsubscribe(client,path)
        elif msg['type'] == 'get':
            value = self.getPath(path)
            if value == None:
                if self.verbosity >= NOTIFICATIONS: print "Client",str(client),"asked for nonexistent path",path,", returning None"
            else:
                if self.verbosity >= ALL: print "Client",str(client),"/",path,"getting",value
                pass
            client.sendMessage(value)
        elif msg['type'] == 'set':
            if 'data' not in msg:
                if self.verbosity >= ERRORS: print "Client",str(client),"/",path,"sent an improper 'set' message with no 'data' element"
                return
            #print "Client",str(client),"/",path,"setting",msg['data']
            self.setPath(path,msg['data'])
        elif msg['type'] == 'change':
            if 'data' not in msg:
                if self.verbosity >= ERRORS: print "Client",str(client),"/",path,"sent an improper 'change' message with no 'data' element"
                return
            #print "Client",str(client),"/",path,"changing",msg['data']
            self.updatePath(path,msg['data'])
        elif msg['type'] == 'delete':
            self.deletePath(path)
        elif msg['type'] == 'resize':
            if 'data' not in msg:
                if self.verbosity >= ERRORS: print "Client",str(client),"/",path,"sent an improper 'resize' message with no 'data' element"
                return
            self.resizePath(path,msg['data'])
        else:
            if self.verbosity >= ERRORS: print "Client",str(client),"sent invalid message type",msg['type']
        self.numInMessages += 1
        self.timeInMessageProcess += time.time()-t0

    def subscribe(self,client,path,rate):
        if self.verbosity >= NOTIFICATIONS: print "Client",str(client),"subscribing to",'"'+path+'"',("at rate "+str(rate) if rate >= 0 else "asynchronous")
        if client.addr not in self.subscribers:
            #add this address to the subscriber list
            self.subscribers[client.addr] = []
        self.subscribers[client.addr].append(SubscriberData(client,path,rate))
        #send first message if nonempty
        if rate == 0:
            value = self.getPath(path)
            if self.verbosity >= DEBUG: print "  First subscribe message",value
            if len(self.subscribers[client.addr])==1:
                if self.verbosity >= DEBUG: print "  Sending (raw) reply",json.dumps(value)
                client.sendMessage(value)
            else:
                if self.verbosity >= DEBUG: print "  Sending (with path info) reply",json.dumps(value)
                client.sendMessage({'path':'.'+path,'data':value})
        return

    def unsubscribe(self,client,path):
        if self.verbosity >= NOTIFICATIONS: print "Client",str(client),"unsubscribing from",'"'+path+'"'
        if client.addr not in self.subscribers:
            if self.verbosity >= ERRORS: print "Unsubscribing client",str(client),"not currently a subscriber?"
            return
        if path == '':
            #erase all
            del self.subscribers[client.addr]
        else:
            #erase those that start with the given path
            self.subscribers[client.addr] = [cdata for cdata in self.subscribers[client.addr] if not isancestorpath(path,cdata.path)]
            if len(self.subscribers[client.addr])==0:
                del self.subscribers[client.addr]
        return
            
    def onUpdate(self):
        """This is called periodically from run()"""
        if not Service.onUpdate(self):
            return False
        t = time.time()
        for addr,subscriptions in self.subscribers.iteritems():
            for clientData in subscriptions:
                if clientData.nextSendTime != None and t >= clientData.nextSendTime:
                    value = self.getPath(clientData.path)
                    if len(subscriptions)==1:
                        self.numOutMessages += 1
                        clientData.client.sendMessage(value)
                    else:
                        self.numOutMessages += 1
                        clientData.client.sendMessage({'path':'.'+path,'data':value})
                    clientData.nextSendTime += 1.0/clientData.rate
        self.timeOutMessageProcess += time.time()-t
        if self.numInMessages > 1000:
            #print self.numInMessages,"incoming messages, processing time",self.timeInMessageProcess/self.numInMessages
            self.numInMessages = 0
            self.timeInMessageProcess = 0
        if self.numOutMessages > 1000:
            #print self.numOutMessages,"outgoing messages, processing time",self.timeOutMessageProcess/self.numOutMessages
            self.numOutMessages = 0
            self.timeOutMessageProcess = 0
        return True

    def onStructureChange(self,path,exact=False,ancestors=True,descendants=True):
        """This is called any time a structure changes. Default updates any
        clients watching ancestors or descendants of the path.  Setting exact
        to True only updates clients watching the exact path.
        """
        for addr,subscriptions in self.subscribers.iteritems():
            for clientData in subscriptions:
                if clientData.rate <= 0:
                    #need to check if the updated path if a subset of client's
                    #path or vice versa
                    cpath=clientData.path
                    if exact:
                        if path == cpath:
                            t0 = time.time()
                            value = self.getPath(cpath)
                            #print "Notifying watcher of",cpath,"of update"
                            if len(subscriptions)==1:
                                if self.verbosity >= DEBUG: print "Sending update (raw)",path,"to",str(clientData.client)
                                clientData.client.sendMessage(value)
                            else:
                                if self.verbosity >= DEBUG: print "Sending update (with path info)",path,"to",str(clientData.client)
                                clientData.client.sendMessage({'path':'.'+path,'data':value})
                            self.numOutMessages += 1
                            self.timeOutMessageProcess += time.time()-t0
                    else:
                        if (ancestors and isancestorpath(cpath,path)) or (descendants and isancestorpath(path,cpath)):
                            #print "Notifying watcher of",cpath,"of update on",path
                            t0 = time.time()
                            value = self.getPath(cpath)
                            if len(subscriptions)==1:
                                if self.verbosity >= DEBUG: print "Sending update of",cpath,"raw to",str(clientData.client)
                                clientData.client.sendMessage(value)
                            else:
                                if self.verbosity >= DEBUG: print "Sending update of",cpath,"(with path info) to",str(clientData.client)
                                clientData.client.sendMessage({'path':'.'+cpath,'data':value})
                            self.numOutMessages += 1
                            self.timeOutMessageProcess += time.time()-t0


    def lookup(self,keys,add = False):
        """Looks up a nested element in self.object given a list of keys"""
        o = self.object
        for element in keys:
            o = safeget(o,element,add)
            if o == None: return o
        return o
                
    def setPath(self,path,obj):
        """Sets the sub-structure at path to obj"""
        items = toKeys(path)
        if len(items)==0:
            #empty string, set whole object
            self.object = obj
        else:
            parent = self.lookup(items[:-1],add=True)
            safeset(parent,items[-1],obj)
        #check listeners
        self.onStructureChange(path)
        return

    def getPath(self,path):
        """Gets the sub-structure at path or None if it doesn't exist"""
        items = toKeys(path)
        return self.lookup(items)

    def updatePath(self,path,obj):
        """Updates the sub-keys of the sub-structure at path to the sub-
        keys of obj."""
        items = toKeys(path)
        #print "Structure updating",items
        try:
            if len(items)==0:
                self._update(self.object,items,obj)
            else:
                parent = self.lookup(items[:-1],add=True)
                self._update(parent,items,obj)
        except Exception:
            print "Error running updatePath on",'"'+path+'",',"object",obj
            raise
        #the _update call notified all changed descendants, now
        #update all ancestors
        #print "Notifying watchers of",'.'.join(items),"and ancestors"
        self.onStructureChange('.'.join(items),exact=False,descendants=False,ancestors=True)

    def _update(self,dest,subkeys,obj):
        """Internal use only"""
        subitem = (subkeys[-1] if len(subkeys) > 0 else None)
        if subitem != None:
            if subitem not in dest or dest[subitem]==None:
                if isinstance(obj,list):
                    dest[subitem] = [{} for v in obj]
                else:
                    dest[subitem] = {}
        if isinstance(obj,dict):
            for (key,value) in obj.iteritems():
                if subitem == None:
                    #root
                    self._update(dest,subkeys+[key],value)
                else:
                    self._update(dest[subitem],subkeys+[key],value)
        elif isinstance(obj,list):
            for (key,value) in enumerate(obj):
                if subitem == None:
                    #root
                    self._update(dest,subkeys+[key],value)
                else:
                    self._update(dest[subitem],subkeys+[key],value)
        else:
            if subitem == None:
                #root
                self.object = obj
            else:
                dest[subitem] = obj
        #print "Notifying watchers of",'.'.join(subkeys)
        self.onStructureChange('.'.join(str(v) for v in subkeys),True)
        return

    def deletePath(self,path):
        """Deletes the given sub-structure at path."""
        items = toKeys(path)
        if len(items)==0:
            #empty string, get whole object
            self.object = {}
        parent = self.lookup(items[:-1])
        if parent == None:
            return
        if safedel(parent,items[-1]):
            self.onStructureChange(path)
        return

if __name__ == '__main__':
    print "Starting structure service on localhost:4567, verbose=ERRORS"
    service = StructureService()
    service.open(('localhost',4567),asServer=True)
    service.run(100)
