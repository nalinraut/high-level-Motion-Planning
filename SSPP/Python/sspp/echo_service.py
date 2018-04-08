from service import Service

class EchoService (Service):
    def onMessage(self,msg):
        print msg

if __name__ == '__main__':
    service = EchoService()
    service.open(('localhost',4567))
    service.run()
