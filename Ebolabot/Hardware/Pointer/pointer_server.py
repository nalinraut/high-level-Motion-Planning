import sys
import pointer
from sspp import service

serverAddr = ('192.168.0.101',6006)
PPU_PORT = '/dev/ttyUSB0'

def handle_accept(self):
        pair = self.accept()
        if pair is not None:
            sock, addr = pair
            self.clients.append(ClientHandler(self,sock,addr))
            print 'Accepted connection from %s' % str(self.clients[-1])
            self.onClientAccept(self.clients[-1])

class PPUServer(service.Service):
    """Implements an RPC using the pointer library"""
    def __init__(self,port=PPU_PORT):
        service.Service.__init__(self)
        print "Opening PPU on port",port
        self.ppu = pointer.PPU(port)
        if not self.ppu.initToPhysicalConfig(timeout=5):
            raise RuntimeError("PPU didn't seem to want to connect")
        print('Connected')
        print self.ppu.getSnsConfig()
    def onClientMessage(self,client,msg):
        func = msg['func']
        args = msg['args']
        f = getattr(self.ppu,func)
        if f is None:
            print "Client called an invalid function!",f
            return False
        for i,a in enumerate(args):
            if isinstance(a,unicode):
                args[i] = a.decode('ascii')
        returnValue = f(*args)
        returnMessage = [returnValue]
        client.sendMessage(returnMessage)
    def onUpdate(self):
        self.ppu.update()

if __name__ == "__main__":
    if len(sys.argv) > 1:
        server = PPUServer(sys.argv[1])
    else:
        server = PPUServer()

    server.open(serverAddr,asServer=True)
    server.run(20.0)
