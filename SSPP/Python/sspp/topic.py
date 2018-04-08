"""Implements a notion of "topics" as members of a StructureService
/ TopicServer.  Using topics on a single service to communicate data
is often easier than directly sending/receiving data from multiple
services.  There is an overhead in this method in that the topic server
must read the data and then re-send it to any subscribers.
"""

from service import Service
from structure_service import StructureService
import time
import asyncore
import socket

class TopicClient (Service):
    """A base class for a service that accesses a topic on a StructureService.
    Accepts name, subscribe, unsubscribe, set, get, change, delete, and
    resize."""
    def __init__(self):
        Service.__init__(self)
        self.firstTopic = None

    def open(self,addr):
        return Service.open(self,addr,asServer=False)

    def setName(self,name):
        """Helper function: may be used by subclasses / callers"""
        self.sendMessage({'type':'name','data':name})

    def subscribe(self,topic,rate=0):
        """Helper function: may be used by subclasses / callers"""
        self.sendMessage({'type':'subscribe','path':topic,'rate':rate})
        if self.firstTopic == None:
            self.firstTopic = topic

    def unsubscribe(self,topic):
        """Helper function: may be used by subclasses / callers"""
        self.sendMessage({'type':'unsubscribe','path':topic})

    def set(self,topic,value):
        """Helper function: may be used by subclasses / callers"""
        self.sendMessage({'type':'set','path':topic,'data':value})

    def get(self,topic):
        """Helper function: may be used by subclasses / callers"""
        self.sendMessage({'type':'get','path':topic})
        if self.firstTopic == None:
            self.firstTopic = topic

    def change(self,topic,value):
        """Helper function: may be used by subclasses / callers"""
        self.sendMessage({'type':'change','path':topic,'data':value})

    def delete(self,topic):
        """Helper function: may be used by subclasses / callers"""
        self.sendMessage({'type':'delete','path':topic})

    def resize(self,topic,n):
        """Helper function: may be used by subclasses / callers"""
        self.sendMessage({'type':'resize','path':topic,'data':n})

    def onMessage(self,msg):
        """Overload this to handle responses to subscribe() and get()"""
        if msg == "SSPP StructureService": pass
        if isinstance(msg,dict) and 'path' in msg:
            return self.onTopicMessage(msg['path'],msg['data'])
        else:
            return self.onTopicMessage(self.firstTopic,msg)
    
    def onTopicMessage(self,path,msg):
        """Overload this to handle responses to subscribe() and get()"""
        pass

class TopicSubscriberBase(TopicClient):
    """Automatically subscribes to a topic on the given address.
    subscriber = TopicSubscriberBase(addr,topic,rate).
    
    Subclass still must overload onMessage()
    """
    def __init__(self,addr,topic,rate=0):
        TopicClient.__init__(self)
        self.topic = topic
        self.rate = rate
        print "TopicSubscriberBase",self.topic,"initializing"
        TopicClient.open(self,addr)
        print "TopicSubscriberBase",self.topic,"subscribing"
        self.subscribe(topic,rate)

class TopicEchoService (TopicSubscriberBase):
    """An example class for how to subclass TopicSubscriberBase."""
    def onMessage(self,msg):
        print "Received message:",msg

class TopicLogService (TopicSubscriberBase):
    """Dumps a log to disk."""
    def __init__(self,logfn,addr,topic='.'):
        TopicSubscriberBase.__init__(self,addr,topic)
        self.sendMessage({'type':'name','data':'Topic Log Service'})
        self.fout = open(logfn,'w')
    def onMessage(self,msg):
        fout.write(json.dumps(msg))
        fout.write('\n')
    def handle_close(self):
        self.fout.close()

class TopicValue (TopicClient):
    """This class allows you to easily get and set a value on a
    StructureService. The value may be changed by external processes, and the
    stored value will be automatically updated (thread safe).

    Usage is
    v = TopicValue(addr=('localhost',4567),topic='.foo')
    v.set({'bar':[4,5,6]})
    print v.get()
    """
    def __init__(self,addr,topic='.'):
        TopicClient.__init__(self)
        try:
            self.open(addr)
        except Exception as e:
            print "Couldn't open..."            
        self.setName('TopicValue('+topic+')')
        self.topic = topic
    def set(self,value):
        TopicClient.set(self,self.topic,value)
    def get(self,timeout=None):
        TopicClient.get(self,topic)
        return self.waitForMessage(timeout)

class TopicListener (TopicSubscriberBase):
    """This class allows you to easily get and set a value on a
    StructureService. The value may be changed by external processes, and the
    stored value will be automatically updated (thread safe).

    Usage is
    v = TopicListener(addr=('localhost',4567),topic='.foo')
    v.set({'bar':[4,5,6]})
    print v.get()

    The main difference between TopicListener and TopicValue is that
    this class will not block on get().  As a result it may be
    somewhat faster if gets are performed frequently while the
    structure stored by the StructureService changes infrequently.
    """
    def __init__(self,addr,topic='.',rate=0):
        self.value = None
        TopicSubscriberBase.__init__(self,addr,topic)
        self.setName('TopicListener('+topic+')')
    def get(self):
        return self.value
    def set(self,value):
        TopicClient.set(self,self.topic,value)
        self.value = value
    def onMessage(self,msg):
        #overload of Service method
        #print "TopicListener",self.topic,"got message",msg
        self.value = msg

class MultiTopicGetterSetter:
    """A convenience class that lets you avoid providing the topic"""
    def __init__(self,client,topic):
        self.client = client
        self.topic = topic
    def set(self,value):
        return self.client.set(self.topic,value)
    def get(self):
        return self.client.get(self.topic)

class MultiTopicValue (TopicClient):
    """This class allows you to easily get and set multiple values on a
    StructureService. The value may be changed by external processes, and the
    stored value will be automatically updated (thread safe).

    Usage is
    v = MultiTopicValue(addr=('localhost',4567))
    v.set('.foo',{'bar':[4,5,6]})
    print v.get()
    """
    def __init__(self,addr):
        TopicClient.__init__(self)
        try:
            self.open(addr)
        except Exception as e:
            print "Couldn't open..."            
        self.setName('MultiTopicValue')
        self.topic = topic
    def set(self,topic,value):
        return TopicClient.set(topic,value)
    def get(self,topic,timeout=None):
        TopicClient.get(topic,self)
        return self.waitForMessage(timeout)
    def __getattr__(self,topic):
        return MultiTopicGetterSetter(self,topic)

class MultiTopicListener (TopicClient):
    """This class allows you to easily get and set multiple values on a
    TopicServer. The value may be changed by external processes, and the
    stored value will be automatically updated (thread safe).

    Usage is
    v = MultiTopicListener(addr=('localhost',4567),topics=['.foo'])
    getter = v.listen('.foo')
    v.set('.foo',{'bar':[4,5,6]})
    print v.get('.foo')
    #equivalently...
    getter.set({'bar':[4,5,6]})
    print getter.get()

    The main difference between MultiTopicListener and MultiTopicValue is that
    this class will not block on get().  As a result it may be
    somewhat faster if gets are performed frequently while the
    structure stored by the StructureService changes infrequently.
    """
    def __init__(self,addr,topics=None,rate=0):
        TopicClient.__init__(self)
        TopicClient.open(self,addr)
        self.values = dict()
        if topics is not None:
            for t in topics:
                self.listen(t)
    def listen(self,topic):
        if topic in self.values: return self[topic]
        self.values[topic] = None
        TopicClient.subscribe(self,topic)
        self.setName('MultiTopicListener('+",".join(self.values.keys())+')')
        return self[topic]
    def get(self,topic):
        return self.values[topic]
    def set(self,topic,value):
        assert topic in self.values,"Can't set a non-subscribed value"
        TopicClient.set(self,topic,value)
        self.values[topic] = value
    def onTopicMessage(self,topic,msg):
        #overload of TopicClient method
        #print "MultiTopicListener",topic,"got message",msg
        self.values[topic] = msg
    def __getitem__(self,topic):
        return MultiTopicGetterSetter(self,topic)

class TopicTransformService (TopicSubscriberBase):
    """A service that will subscribe to a topic, transform it, and publish it
    back to a different topic.
    """
    def __init__(self,func,addr,sourceTopic,destTopic):
        self.func = func
        self.lastSentMsg = None
        self.sourceTopic = sourceTopic
        self.destTopic = destTopic
        self.checkLoop = (sourceTopic == destTopic)
        TopicSubscriberBase.__init__(self,addr,sourceTopic)
        self.setName('TopicListener('+sourceTopic+','+destTopic+')')
    def onMessage(self,msg):
        if self.checkLoop:
            if msg == self.lastSentMsg:
                #avoid infinite loops if sourceTopic == destTopic
                return
        #process and send
        res = self.func(msg)
        self.sendMessage({'type':'set','path':self.destTopic,'data':res})
        if self.checkLoop:
            self.lastSentMsg = msg

class TopicServer(StructureService):
    """A convenience class that runs a topic server on the initialized
    address."""
    def __init__(self,addr=None):
        StructureService.__init__(self)
        if addr:
            self.open(addr)
    def open(self,addr):
        StructureService.open(self,addr,asServer=True)

if __name__ == '__main__':
    service = TopicEchoService(('localhost',4568),'.')
    service.run()
