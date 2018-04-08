from sspp import service

class PPUClient(service.Service):
    def __init__(self,serverAddr = ('192.168.0.101',6006)):
        service.Service.__init__(self)
        self.open(serverAddr,asServer=False)
        if not self.opened:
            raise RuntimeError("PPU server has not started?")
    def callGenericFunction(self,func,args):
        self.sendMessage({'func':func,'args':args})
        result = self.waitForMessage(3)
        if result is None:
            return result
        return result[0]
    def setStepSize(self,size):
        return self.callGenericFunction('setStepSize',[size])
    def setCmdConfig(self, config, mode = 'raw'):
        return self.callGenericFunction('setCmdConfig',[config,mode])
    def setCmdPos(self, coord):
        return self.callGenericFunction('setCmdPos',[coord])
    def getCmdConfig(self,mode = 'raw'):
        return self.callGenericFunction('getCmdConfig',[mode])
    def getSnsConfig(self,mode = 'raw'):
        return self.callGenericFunction('getSnsConfig',[mode])
    def setHalt(self,retval='raw'):
        return self.callGenericFunction('setHalt',[retval])
    def setCmdRad(self, motor, rads):
        return self.callGenericFunction('setCmdRad',[motor,rads])
    def getCmdRad(self, motor):
        return self.callGenericFunction('getCmdRad',[motor])
    def setCmdStrokPrct(self,pct):
        return self.callGenericFunction('setCmdStrokPrct',[pct])
    def getCmdStrokPrct(self):
        return self.callGenericFunction('getCmdStrokPrct',[])

if __name__ == "__main__":
    import time
    print "Testing pointer_client"
    ppu = PPUClient()
    pan,tilt,ext = ppu.getCmdRad('pan'),ppu.getCmdRad('tilt'),ppu.getCmdStrokPrct()
    print "Commanded config:",pan,tilt,ext
    ppu.setCmdConfig((pan,tilt,1),'rad/prct')
    for i in range(50):
        print "Config:",ppu.getSnsConfig('rad/prct')
        time.sleep(0.1)
    ppu.setCmdConfig((pan,tilt,0),'rad/prct')
    for i in range(50):
        print "Config:",ppu.getSnsConfig('rad/prct')
        time.sleep(0.1)
