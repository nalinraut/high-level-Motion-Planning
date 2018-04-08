'''
    A Task Generator for forwarding motion commands into the TRINA stack.
    Based heavily (copied) from Kris hauser's Oct 2015 game_pad.py TaskGenerator.

    Converted by Gunnar Horve and Max Merlin Jan 2015.

    Subscribes to a robot OTP handover topic.  Aimed at being used for autonomous
    robotic handovers.
'''
# Import Modules
import os
import sys
ebolabot_root = os.getenv("EBOLABOT_PATH",".")
sys.path.append(ebolabot_root)
from task_generator import TaskGenerator

from OpenGL.GL import *
from klampt.vis import gldraw
from klampt.vis.glinterface import GLPluginInterface as GLPluginBase
import rospy
from otp.msg import pos


class MyWidgetPlugin(GLPluginBase):
    ''' Hook class to allow event passing to/from Klampt && OpenGL UI '''
    def __init__(self,taskGen): #taskGen is an instance ofSkittlesTaskGenerator
        GLPluginBase.__init__(self)
        self.taskGen = taskGen

    def initialize(self):
        GLPluginBase.initialize(self)
        return True

    def keyboardfunc(self,c,x,y):
        print("Someone typed " + c)

    def display(self):
        pass

    def display_screen(self):
        pass

    def eventfunc(self,type,args):
        print("we got an EVENT!!!!!!!!!!!!")

class SkittlesTaskGenerator(TaskGenerator):
    '''
        Logic class for sending commands to TRINA.
        See TaskGenerator for method definitions
    '''

    def __init__(self):
        self.limb = 'left'
        self.lastState = {}
        self.plugin = None

        # set up OTP subscriber && defalt values
        rospy.Subscriber("/position", pos, self.updateOTP)
        self._x = 1
        self._y = 0
        self._z = 1.2

    def name(self): return "GSkittles"

    def init(self,world):
        self.world = world
        return True

    def start(self):
        self.limb = 'left'
        self._status = 'ok'
        self.plugin = MyWidgetPlugin(self)
        self.lastState = {}
        return True

    def status(self):
        return 'ok'

    def messages(self):
        return ["Controlling "+self.limb]

    def controlMode(self):
        return "Cartesian position"

    def stop(self):
        self._status=''
        self.plugin = None

    def get(self):
        #+x is forward, +z is up, +y is left
        #For details on the args go to cartesian_pose.py in controller/tasks
        return {"type":"CartesianPose", "limb":"left", "position":[self._x,self._y,self._z], "rotation":[0,1,0,0,0,1,1,0,0], "speed":1, "maxJointDeviation":100, "safe":0}

    def glPlugin(self):
        return self.plugin

    def updateOTP(self, data):
        self._x = data.x
        self._y = data.y
        self._z = data.z    

def make():
    ''' Hook function that C++ aspects of Ebolabot stack grab. '''
    return SkittlesTaskGenerator()
