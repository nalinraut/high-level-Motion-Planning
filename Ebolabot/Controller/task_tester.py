import os
import sys
from sspp.topic import *
ebolabot_root = os.getenv("EBOLABOT_PATH",".")
sys.path.append(ebolabot_root)
from Common.system_config import EbolabotSystemConfig
import threading
import time
import signal

#Configuration variable: where's the state server?
system_state_addr = EbolabotSystemConfig.getdefault_ip('state_server_computer_ip',('localhost',4568))

class ServiceThread(threading.Thread):
    """This thread takes care of running asyncore while the GUI is running"""
    def __init__(self):
        threading.Thread.__init__(self)
        self.updateFreq = 100
        self._kill = False
        self.taskStatusGetter = TopicListener(system_state_addr,'.controller.task_status')
        self.taskSetter = TopicServiceBase('.controller.task')
        self.taskSetter.open(system_state_addr)
    def run(self):
        while(not self._kill):
            asyncore.loop(timeout = 1.0/self.updateFreq, count=1)
    def kill(self):
        self._kill = True


class TaskTester:
    def __init__(self):
        self.serviceThread = ServiceThread()
        self.serviceThread.start()
        self.lastTask = None

        #cleanly handle Ctrl+C
        def handler(signal,frame):
            print "Exiting due to Ctrl+C"
            self.serviceThread.kill()
            exit(0)
        signal.signal(signal.SIGINT,handler)

    def close(self):
        self.serviceThread.kill()
        self.serviceThread = None
    def sendJointPose(self,limb,position,speed=None,safe=None,**args):
        self.sendMsg('JointPose',limb=limb,position=position,speed=speed,safe=safe,**args)
    def sendJointVelocity(self,limb,velocity,maxTime=None,safe=None,**args):
        self.sendMsg('JointVelocity',limb=limb,velocity=velocity,maxTime=maxTime,safe=safe,**args)
    def sendJointVelocity(self,limb,velocity,maxTime=None,safe=None,**args):
        self.sendMsg('JointVelocity',limb=limb,velocity=velocity,maxTime=maxTime,safe=safe,**args)
    def sendGripperCommand(self,limb,command):
        self.sendMsg('Gripper',limb=limb,command=command)
    def sendGripperPosition(self,limb,position,speed=None,force=None):
        self.sendMsg('Gripper',limb=limb,position=position,speed=speed,force=force)
    def sendMsg(self,type,**args):
        msg = {}
        for k,v in args.iteritems():
            if v != None:
                msg[k] = v
        msg['type'] = type
        print "Starting task",type,"with params",msg
        self.serviceThread.taskSetter.set(msg)
        time.sleep(0.05)
        self.lastTask = type
    def waitForTaskCompletion(self,pollFreq = 0.1,timeout = 10.0):
        """Waits for the task to complete, and returns the status code"""
        t = 0
        while t < timeout:
            status = self.serviceThread.taskStatusGetter.get()
            if status != 'running':
                print "Task",self.lastTask,"returned with status",status
                return status
            time.sleep(pollFreq)
            t += pollFreq
        return 'timeout'

if __name__=='__main__':
    #in real usage you would not want to import motion due to the
    #risk of clobbering other task controllers.  You would rather
    #setup your controller to read from the state service.
    from Motion import motion
    from Motion import config
    config.setup()
    res = motion.robot.startup()
    if not res:
        print "Error starting up Motion Module"
        exit(1)
    time.sleep(0.1)

    tester = TaskTester()
    try:
        tester.sendJointPose('left',position=[,0,0,0,0,0,0])
        tester.waitForTaskCompletion()
        #tester.sendJointPose('left',position=[0,0,0,1,0,0,0])
        #tester.waitForTaskCompletion()
        #tester.sendJointPose('left',position=[0,0,0,0,0,0,0],speed=0.5)
        #tester.waitForTaskCompletion()
        tester.sendGripperCommand('left','open')
        tester.waitForTaskCompletion()
        tester.sendGripperCommand('left','close')
        tester.waitForTaskCompletion()
        tester.sendGripperCommand('left','open')
        tester.waitForTaskCompletion()
    except Exception as e:
        raise
    finally:
        tester.close()
