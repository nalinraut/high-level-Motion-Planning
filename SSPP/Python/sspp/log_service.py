from service import Service,packStrlen

class LogService (Service):
    def __init__(self,fn=None):
        Service.__init__(self)
        self.file = None
        if fn != None:
            self.setLogFile(fn)
    def setLogFile(self,fn):
        print "Saving to",fn
        self.file = open(fn,'w')
    def onMessage(self,msg):
        msgstr = json.dumps(msg)
        self.file.write(packStrlen(msgstr))
        self.file.write(msgstr)
    def handle_close(self):
        Service.handle_close(self)
        self.file.close()
        self.file = None

if __name__ == '__main__':
    service = LogService()
    service.setLogFile('log.txt')
    service.open(('localhost',4567),False)
    service.run()
