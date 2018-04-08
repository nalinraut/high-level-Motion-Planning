from klampt import *
from klampt import WidgetSet,RobotPoser
from klampt.glprogram import *
from klampt.glcommon import GLWidgetPlugin
import math
import time
import signal
from sspp.topic import *
ebolabot_root = os.getenv("EBOLABOT_PATH",".")
sys.path.append(ebolabot_root)
from Common.system_config import EbolabotSystemConfig
import threading

#Configuration variable: where's the state server?
system_state_addr = EbolabotSystemConfig.getdefault_ip('state_server_computer_ip',('localhost',4568))



class GLWidgetProgram(GLPluginProgram):
    """A program that uses a widget plugin"""
    def __init__(self,name):
        GLPluginProgram.__init__(self,"Manual poser")
        self.widgetPlugin = GLWidgetPlugin()
    def initialize(self):
        GLPluginProgram.initialize(self)
        self.setPlugin(self.widgetPlugin)
    def addWidget(self,widget):
        self.widgetPlugin.addWidget(widget)

class MyGLViewer(GLWidgetProgram):
    def __init__(self,world,serviceThread):
        GLWidgetProgram.__init__(self,"Manual poser")

        self.world = world
        self.serviceThread = serviceThread

        for i in range(10):
            q = self.serviceThread.qcmdGetter.get()
            if q != None:
                break
            time.sleep(0.1)
        if q == None:
            print "Warning, system state service doesn't seem to have .robot.command.q"
            print "Press enter to continue"
            raw_input()
        else:
            world.robot(0).setConfig(q)
        self.robotPoser = RobotPoser(world.robot(0))
        self.addWidget(self.robotPoser)
        
        robot = world.robot(0)
        left_arm_link_names = ['left_upper_shoulder','left_lower_shoulder','left_upper_elbow','left_lower_elbow','left_upper_forearm','left_lower_forearm','left_wrist']
        right_arm_link_names = ['right_upper_shoulder','right_lower_shoulder','right_upper_elbow','right_lower_elbow','right_upper_forearm','right_lower_forearm','right_wrist']
        self.left_arm_link_indices = [robot.link(l).index for l in left_arm_link_names]
        self.right_arm_link_indices = [robot.link(l).index for l in right_arm_link_names]

    def display(self):
        #Put your display handler here
        #the example draws the posed robot in grey, the sensed
        #configuration in transparent red and the commanded
        #configuration in transparent green

        #this line will draw the world
        GLWidgetProgram.display(self)
        
        q = self.serviceThread.qsnsGetter.get()
        if q:
            glEnable(GL_BLEND)
            glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
            glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[1,0,0,0.5])
            self.world.robot(0).setConfig(q)
            self.world.robot(0).drawGL(False)

        #draw commanded configuration
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
        glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[0,1,0,0.5])
        q = self.serviceThread.qcmdGetter.get()
        if q:
            self.world.robot(0).setConfig(q)
            self.world.robot(0).drawGL(False)
            glDisable(GL_BLEND)

    def keyboardfunc(self,c,x,y):
        #Put your keyboard handler here
        #the current example toggles simulation / movie mode
        if c == 'h':
            print '[space]: send the current posed milestone'
            print 'q: clean quit'
        elif c == ' ':
            q = self.robotPoser.get()
            #CONSTRUCT THE TASK HERE
            msg = {}
            msg['type'] = 'JointPose'
            msg['limb'] = 'both'
            msg['position'] = [q[i] for i in self.left_arm_link_indices + self.right_arm_link_indices]
            msg['speed'] = 1
            msg['safe'] = 0
            print "Sending message",msg
            #SEND THE TASK TO THE SYSTEM STATE SERVICE
            self.serviceThread.taskSetter.set(msg)
        elif c == 'q':
            self.serviceThread.kill()
            exit(0)
        else:
            GLWidgetProgram.keyboardfunc(self,c,x,y)
            self.refresh()


class ServiceThread(threading.Thread):
    """This thread takes care of running asyncore while the GUI is running"""
    def __init__(self):
        threading.Thread.__init__(self)
        self._kill = False
        self.updateFreq = 100
        #set up the listeners: sensed, commanded, and destination configs
        self.qsnsGetter = TopicListener(system_state_addr,'.robot.sensed.q',20)
        self.qcmdGetter = TopicListener(system_state_addr,'.robot.command.q',20)
        self.qdstGetter = TopicListener(system_state_addr,'.controller.traj_q_end',20)
        self.statusGetter = TopicListener(system_state_addr,'.controller.task_status',20)
        self.taskSetter = TopicServiceBase('.controller.task')
        self.taskSetter.open(system_state_addr)
    def run(self):
        while(not self._kill):
            asyncore.loop(timeout = 1.0/self.updateFreq, count=1)
    def kill(self):
        self._kill = True

if __name__ == "__main__":

    print "widget_ui.py: manually sends configurations to the ControllerDispatcher"
    print "Press [space] to send milestones.  Press q to exit."
    print
    klampt_model = EbolabotSystemConfig.get("klampt_model",lambda s:s.encode('ascii','ignore'))
    world = WorldModel()
    res = world.readFile(klampt_model)
    if not res:
        raise RuntimeError("Unable to load Ebolabot model "+klampt_model)

    serviceThread = ServiceThread()
    serviceThread.start()

    #cleanly handle Ctrl+C
    def handler(signal,frame):
        print "Exiting due to Ctrl+C"
        serviceThread.kill()
        exit(0)
    signal.signal(signal.SIGINT,handler)
    
    viewer = MyGLViewer(world,serviceThread)
    viewer.run()

    #cleanup
    serviceThread.kill()
