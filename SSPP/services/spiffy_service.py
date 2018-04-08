#!/usr/bin/python

from sspp.structure_service import StructureService
import json
import time


class SpiffyService(StructureService):
    def __init__(self,host,initFile=None):
        StructureService.__init__(self)
        self.open(host,asServer=True)
        self.logDelay = None
        self.nextLogTime = None
        self.logPrefix = ''
        if initFile:
            print "SpiffyService: loading initial value from",initFile
            f = open(initFile,'r')
            self.object = json.loads(''.join(f.readlines()))
            f.close()
            #clear any ._spiffy data
            if '_spiffy' in self.object:
                del self.object['_spiffy']
    def save(self,fileName=None):
        if fileName == None:
            timestr = time.strftime("%Y%m%d-%H%M%S")
            fileName = self.logPrefix+timestr+'.json'
        print "Saving Spiffy dump to",fileName
        f = open(fileName,'w')
        f.write(json.dumps(self.object))
        f.write('\n')
        f.close()

    def onUpdate(self):
        if self.nextLogTime!=None and time.time() >= self.nextLogTime:
            self.save()
            self.nextLogTime = time.time()+self.logDelay
            
    def beginLogging(self,delay_in_seconds,prefix=''):
        self.logDelay = delay_in_seconds
        self.logPrefix = prefix
        self.nextLogTime = 0

if __name__ == '__main__':
    import sys
    import os
    persistent_file = 'spiffy_service_persistent.json'
    oldfile = None
    if len(sys.argv) > 1:
        oldfile = sys.argv[1]
    if oldfile == 'persistent':
        oldfile = persistent_file
        if not os.path.exists(persistent_file):
            oldfile = None
    try:
        service = SpiffyService(('localhost',4567),oldfile)
        service.beginLogging(60,'spiffy_service_')
        service.run(1.0)
    except KeyboardInterrupt:
        if len(sys.argv) > 1 and sys.argv[1] == 'persistent':
            service.save(persistent_file)
        print "Spiffy server killed via Ctrl+C"
