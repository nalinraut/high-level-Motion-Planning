from klampt import *
from klampt import WidgetSet,RobotPoser
from klampt import vectorops
from klampt.glprogram import *
from klampt.glcommon import GLWidgetPlugin
import math
import time
import sys
import os
ebolabot_root = os.getenv("EBOLABOT_PATH",".")
sys.path.append(ebolabot_root)
from Motion import motion
from Motion import config

def selfcollision(robot,qstart,qgoal):
    """returns whether a self collision is predicted along the route from qstart to qgoal"""
    distance = vectorops.distance(qstart,qgoal)
    epsilon = math.radians(2)
    numsteps = int(math.ceil(distance/epsilon))
    for i in xrange(numsteps+1):
        u = float(i)/numsteps
        q = robot.interpolate(qstart,qgoal,u)
        robot.setConfig(q)
        if robot.selfCollides():
            return True
    return False

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
    def __init__(self,world):
        GLWidgetProgram.__init__(self,"Manual poser")

        self.world = world
        self.robotPoser = RobotPoser(world.robot(0))
        self.addWidget(self.robotPoser)

        robot = world.robot(0)
        left_arm_link_names = ['left_upper_shoulder','left_lower_shoulder','left_upper_elbow','left_lower_elbow','left_upper_forearm','left_lower_forearm','left_wrist']
        right_arm_link_names = ['right_upper_shoulder','right_lower_shoulder','right_upper_elbow','right_lower_elbow','right_upper_forearm','right_lower_forearm','right_wrist']
        self.left_arm_link_indices = [robot.link(l).index for l in left_arm_link_names]
        self.right_arm_link_indices = [robot.link(l).index for l in right_arm_link_names]
        self.firstTimeHack = True

    def display(self):
        #Put your display handler here
        #the current example draws the sensed robot in grey and the
        #commanded configurations in transparent green

        #this line with draw the world
        robot = motion.robot
        q = robot.getKlamptSensedPosition()
        self.world.robot(0).setConfig(q)
        self.world.robot(0).drawGL()
        GLWidgetProgram.display(self)

        #draw commanded configuration
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
        glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[0,1,0,0.5])
        q = robot.getKlamptCommandedPosition()
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
            q0 = motion.robot.getKlamptCommandedPosition()
            if not self.firstTimeHack and selfcollision(self.world.robot(0),q0,q):
                print "Invalid configuration, it self-collides"
            else:
                self.firstTimeHack = False
                robot = motion.robot
                robot.left_mq.setRamp(robot.left_limb.configFromKlampt(q))
                robot.right_mq.setRamp(robot.right_limb.configFromKlampt(q))
                qlg = robot.left_gripper.configFromKlampt(q)
                qrg = robot.right_gripper.configFromKlampt(q)
                robot.left_gripper.command(qlg,[1]*len(qlg),[1]*len(qlg))
                robot.right_gripper.command(qrg,[1]*len(qrg),[1]*len(qrg))
                #robot.left_mq.setRamp([q[i] for i in self.left_arm_link_indices])
                #robot.right_mq.setRamp([q[i] for i in self.right_arm_link_indices])
        elif c == 'q':
            motion.robot.shutdown()
            exit(0)
        else:
            GLWidgetProgram.keyboardfunc(self,c,x,y)
            self.refresh()


if __name__ == "__main__":
    print "collision_test.py: manually sends configurations to the Motion module"
    print "Press [space] to send milestones.  Press q to exit."
    print

    config.setup()
    res = motion.robot.startup()
    if not res:
        print "Error starting up Motion Module"
        exit(1)
    time.sleep(0.1)
    world = WorldModel()
    res = world.readFile(config.klampt_model)
    if not res:
        raise RuntimeError("Unable to load Motion Module model "+fn)
    
    viewer = MyGLViewer(world)
    viewer.run()
    motion.robot.shutdown()
