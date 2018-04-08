"""An implementation of the Simple Structure Passing Protocol
"""
import asyncore,socket
import errno
import json
import time
import sys
import traceback

headerlen = 4

def packStrlen(s):
    l = len(s)
    assert(l <= 0xffffffff)
    bytes = [None]*4
    bytes[0] = chr(l&0xff)
    bytes[1] = chr((l>>8)&0xff)
    bytes[2] = chr((l>>16)&0xff)
    bytes[3] = chr((l>>24)&0xff)
    return ''.join(bytes)

def unpackStrlen(s):
    assert len(s)==headerlen
    return (ord(s[3])<<24)|(ord(s[2])<<16)|(ord(s[1])<<8)|ord(s[0])

def writeSocket(socket,msg):
    totalsent = 0
    while totalsent < len(msg):
        sent = socket.send(msg[totalsent:])
        if sent == 0:
            raise IOError("socket connection closed")
        totalsent = totalsent + sent
    return

def readSocket(socket,length):
    chunk = socket.recv(length)
    msg = chunk
    while len(msg) < length:
        chunk = socket.recv(length-len(msg))
        if chunk == '':
            raise IOError("socket connection closed")
        msg = msg + chunk
    return msg

class ClientHandler(asyncore.dispatcher):
    """Don't use this externally... Used when the Service is
    used in server mode to handle connected clients."""
    def __init__(self,server,sock,addr):
        sock.setblocking(True)
        sock.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
        asyncore.dispatcher.__init__(self,sock)
        self.addr = addr
        self.name = None
        self.buffer = ""
        self.server = server

    def __str__(self):
        if self.name==None:
            return self.addr[0]+":"+str(self.addr[1])
        else:
            return '"'+self.name+'" ('+self.addr[0]+":"+str(self.addr[1])+')'

    def writable(self):
        """Called to determine whether there's any data left to be sent.
        Do not override."""
        return (len(self.buffer) > 0)

    def handle_error(self):
        print "ClientHandler: an error occurred:",sys.exc_info()[1]
        self.close()
        raise
        #traceback.print_exc()

    def writable(self):
        """Called to determine whether there's any data left to be sent.
        Do not override."""
        return (len(self.buffer) > 0)

    def handle_write(self):
        """Called to send data when available.  Do not override."""
        sent = self.send(self.buffer)
        self.buffer = self.buffer[sent:]

    def handle_read(self):
        if self.server == None: return
        try:
            lenstr = self.read(headerlen)
            msglen = unpackStrlen(lenstr)
            msg = self.read(msglen)
            try:
                output = json.loads(msg)
            except ValueError:
                print "Error parsing JSON object from message '"+msg+"'"
                return
            self.server.onClientMessage(self,output)
        except IOError as e:
            #print "ClientHandler.handle_read: connection from",str(self),"broken"
            print "Closing connection from",str(self)
            print "     Reason:",e
            self.close()
            return     
        except AssertionError:
            print "ClientHandler.handle_read: assertion error"
            return            

    def recv(self, buffer_size):
        """Fix for windows sockets throwing EAGAIN crashing asyncore"""
        while True:
            try:
                data = self.socket.recv(buffer_size)
                if not data:
                    # a closed connection is indicated by signaling
                    # a read condition, and having recv() return 0.
                    self.handle_close()
                    return ''
                else:
                    return data
            except socket.error, why:
                # winsock sometimes throws ENOTCONN
                if why.args[0] in (errno.EAGAIN, errno.EWOULDBLOCK):
                    #print "EGAIN or EWOULDBLOCK returned... spin waiting"
                    time.sleep(0.001)
                    continue
                elif why.args[0] == errno.ENOTCONN:
                    self.handle_close()
                    return ''
                else:
                    print "socket.error in recv",buffer_size
                    print why
                    raise

    def read(self,length):
        chunk = self.recv(length)
        msg = chunk
        while len(msg) < length:
            chunk = self.recv(length-len(msg))
            if chunk == '':
                raise IOError("socket connection broken")
            msg = msg + chunk
        return msg
        
    def handle_close(self):
        if self.server == None: return
        self.server.onClientClose(self)
        self.server.clients.remove(self)
        self.server = None
        #elf.close()

    def sendMessage(self,msg):
        smsg = json.dumps(msg)
        #print "JSON message:",smsg
        #print "message prefix:",packStrlen(smsg)
        self.buffer = self.buffer + packStrlen(smsg) + smsg
        #print "buffer now:",self.buffer

class Service(asyncore.dispatcher):
    """A service that transmits JSON messages in the simple structure
    passing protocol. Sends/receives variable-length messages such that
    the first 4 bytes are the length of the message (in binary) and the remainder is
    the payload.

    Subclasses should override onMessage, which accepts with arbitrary
    Python objects that can be serialized by the json module.
    Subclasses should use sendMessage to send a message.
    Subclasses should override onUpdate() if you wish to send periodic
    messages

    To open a socket, call open().
    
    To run, call run().  If run(updateFreq) is called, the onUpdate method
    will be called every 1/updateFreq seconds.
    """
    def __init__(self, sock = None, addr = None, map = None):
        if sock != None:
            sock.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY)
            self.opened = True
            asyncore.dispatcher.__init__(self,sock,map=map)
        else:
            self.opened = False
            asyncore.dispatcher.__init__(self,map=map)
        self.connected = False
        self.map = map
        self.addr = addr
        self.buffer = ""
        self.kill = False
        
    def open(self,addr,asServer=False):
        self.opened = True
        self.addr = addr
        if asServer:
            self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
            self.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
            self.set_reuse_addr()
            self.bind(addr)
            self.listen(5)
            self.clients = []
        else:
            self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
            self.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
            self.connect( addr )

    def handle_accept(self):
        pair = self.accept()
        if pair is not None:
            sock, addr = pair
            self.clients.append(ClientHandler(self,sock,addr))
            print 'Accepted connection from %s' % str(self.clients[-1])
            self.onClientAccept(self.clients[-1])

    def handle_connect(self):
        """Called on socket connect.  May be overridden."""
        self.connected = True
        return

    def handle_close(self):
        """Called on socket close.  May be overridden."""
        self.close()
        self.opened = False
        self.connected = False
        return

    def handle_error(self):
        print "Service: an error occurred:",sys.exc_info()[1]
        traceback.print_exc()
        self.close()
        self.opened = False
        raise

    def handle_read(self):
        """Called on read.  Do not override; override onMessage instead."""
        try:
            lenstr = self.read(headerlen)
            msglen = unpackStrlen(lenstr)
            msg = self.read(msglen)
            try:
                output = json.loads(msg)
            except ValueError:
                print "Error parsing JSON object from message '"+msg+"'"
                return
            self.onMessage(output)
        except IOError:
            print "Service.handle_read: connection to",self.addr,"broken"
            return     
        except AssertionError:
            print "Service.handle_read: assertion error"
            return

    def writable(self):
        """Called to determine whether there's any data left to be sent.
        Do not override."""
        return (len(self.buffer) > 0)

    def handle_write(self):
        """Called to send data when available.  Do not override."""
        sent = self.send(self.buffer)
        self.buffer = self.buffer[sent:]

    def waitForMessage(self,timeout=None,sameThread=True):
        """Call this to wait for the next message, or for the timeout to elapse.
        Returns the waited for message, or None if the timeout was reached.

        If sameThread=False, assumes the asyncore loop has already started in a different
        thread (e.g., using run()).  If sameThread=False and this function is called in the
        same thread as run(), it will block!
        """
        self._waitMessage = None
        self._waitMessageReceived = False
        def waitOnMessage(msg):
            self._waitMessage = msg
            self._waitMessageReceived = True
        oldOnMessage = self.onMessage
        self.onMessage = waitOnMessage
        time.sleep(0.001)
        t0 = time.time()
        while True:
            timeleft = (timeout - (time.time()-t0) if timeout != None else 1.0)
            if timeleft <= 0:
                break
            if self._waitMessageReceived:
                self.onMessage = oldOnMessage
                return self._waitMessage
            if not self.opened:
                print "waitForMessage(): read error occurred, returning None"
                self.onMessage = oldOnMessage
                return None
            if sameThread:
                asyncore.loop(timeout = 0.01, count=1, map=self.map)
            else:
                time.sleep(0.01)
        self.onMessage = oldOnMessage
        print "waitForMessage(): hit timeout, returning None"
        return None

    def onMessage(self,msg):
        """Override this to handle an incoming message"""
        pass

    def onClientAccept(self,client):
        """Override this to handle accepting a client.  Default does nothing."""
        pass

    def onClientMessage(self,client,msg):
        """Override this to handle an incoming message from a client.
        Default just calls onMessage. """
        self.onMessage(msg)

    def onClientClose(self,client):
        """Override this to handle a client closing.  Default does nothing. """
        pass
    
    def sendMessage(self,msg):
        """Call this to send an outgoing message"""
        if hasattr(self,'clients'):
            #this is in server mode, send the message to clients
            smsg = json.dumps(msg)
            #print "server->client JSON message:",smsg
            #print "  msg header",packStrlen(smsg)
            for c in self.clients:
                c.buffer = c.buffer + packStrlen(smsg) + smsg
                #print "client buffer now:",c.buffer
            return

        smsg = json.dumps(msg)
        #print "Client->server JSON message:",smsg
        self.buffer = self.buffer + packStrlen(smsg) + smsg
        #print "buffer now:",self.buffer

    def onUpdate(self):
        """Override this to run periodic sends.  Return False to quit."""
        return not self.kill

    def read(self,length):
        chunk = self.recv(length)
        if chunk == '':
            raise IOError("socket connection broken")
        msg = chunk
        while len(msg) < length:
            chunk = self.recv(length-len(msg))
            if chunk == '':
                raise IOError("socket connection broken")
            msg = msg + chunk
        return msg

    def recv(self, buffer_size):
        """Fix for windows sockets throwing EAGAIN crashing asyncore"""
        while True:
            try:
                data = self.socket.recv(buffer_size)
                if not data:
                    # a closed connection is indicated by signaling
                    # a read condition, and having recv() return 0.
                    self.handle_close()
                    return ''
                else:
                    return data
            except socket.error, why:
                # winsock sometimes throws ENOTCONN
                if why.args[0] in (errno.EAGAIN, errno.EWOULDBLOCK):
                    #print "EGAIN or EWOULDBLOCK returned... spin waiting"
                    time.sleep(0.001)
                    continue
                elif why.args[0] == errno.ENOTCONN:
                    self.handle_close()
                    return ''
                else:
                    print "socket.error in recv",buffer_size
                    print why
                    raise
               
    def run(self,updateFreq=0):
        if not self.opened:
            raise ValueError("Calling run on service without first calling open()")
        if updateFreq == 0:
            while not self.kill:
                asyncore.loop(timeout = 0.01, count=1, map=self.map)
        else:
            while not self.kill:
                asyncore.loop(timeout = 1.0/updateFreq, count=1, map=self.map)
                if self.onUpdate() == False:
                    return

    def terminate(self):
        self.kill = True
