# Import Modules
import os
import sys
import rospy
ebolabot_root = os.getenv("EBOLABOT_PATH",".")
sys.path.append(ebolabot_root)
#for logitech module
sys.path.append(os.path.join(ebolabot_root,'InputDevices/GamePad'))
from Common.system_config import EbolabotSystemConfig
from task_generator import TaskGenerator
from klampt import RobotPoser
from klampt.math import so3,se3,vectorops

from OpenGL.GL import *
from klampt.vis import gldraw
from klampt.vis.glinterface import GLPluginInterface as GLPluginBase
from klampt.vis.glcommon import GLWidgetPlugin

from visualization_msgs.msg import Marker
import threading
from sspp.service import Service
from sspp.topic import TopicListener
import asyncore


# parameters for initial robot pose
q_init_left = [-0.08897088559570313, -0.9675583808532715, -1.2034079267211915, 1.7132575355041506, 0.6776360122741699, 1.0166457660095216, -0.5065971546203614]
q_init_right = [0.08897088559570313, -0.9675583808532715, 1.2034079267211915, 1.7132575355041506, -0.6776360122741699, 1.0166457660095216, 0.5065971546203614]

q_tuckarm_left = [0.58897088559570313, -0.9675583808532715, -1.2034079267211915, 1.7132575355041506, 0.6776360122741699, 1.0166457660095216, -0.5065971546203614]
q_tuckarm_right = [-0.58897088559570313, -0.9675583808532715, 1.2034079267211915, 1.7132575355041506, -0.6776360122741699, 1.0166457660095216, 0.5065971546203614]


q = (0.564775,0.425383,0.426796,0.563848)
R1 = so3.from_quaternion(q)
t1 = (0.227677,0.0916972,0.0974174)
kinect_frame_to_pedestal = (R1, t1)



#Configuration variable: where's the state server?
system_state_addr = EbolabotSystemConfig.getdefault_ip('state_server_computer_ip',('localhost',4568))

#imaging stuff
try:
    from PIL import Image
except ImportError, err:
    import Image

class GLTexture:
    def __init__(self,fn=None):
        self.glid = None
        if fn:
            self.loadImage(fn)
    def destroy(self):
        glDeleteTextures([self.glid])

    def setBytes(self,w,h,buffer,glformat=GL_RGBA):
        self.w,self.h = w,h
        if self.glid == None:
            self.glid = glGenTextures(1)
        glBindTexture(GL_TEXTURE_2D,self.glid)
        glPixelStorei(GL_UNPACK_ALIGNMENT,1)
        glTexImage2D(
            GL_TEXTURE_2D, 0, glformat, w, h, 0,
            glformat, GL_UNSIGNED_BYTE, buffer
        )
    def loadImage(self,fn):
        im = Image.open(fn)
        try:
            self.w,self.h,image = im.size[0],im.size[1],im.tobytes("raw","RGBA",0,-1)
        except SystemError:
            self.w,self.h,image = im.size[0],im.size[1],im.tobytes("raw","RGBX",0,-1)
        if self.glid == None:
            self.glid = glGenTextures(1)
        glBindTexture(GL_TEXTURE_2D,self.glid)
        glPixelStorei(GL_UNPACK_ALIGNMENT,1)
        glTexImage2D(
            GL_TEXTURE_2D, 0, GL_RGBA, self.w, self.h, 0,
            GL_RGBA, GL_UNSIGNED_BYTE, image
        )
        return True
    def enable(self,smooth=True,glmode=GL_MODULATE):
        glEnable(GL_TEXTURE_2D)
        if smooth:
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
        else:
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
        glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, glmode)
        glBindTexture(GL_TEXTURE_2D,self.glid)
    def disable(self):
        glDisable(GL_TEXTURE_2D)
    def blit(self,x,y,w=None,h=None):
        if w==None: w = self.w
        if h==None: h = self.h
        self.enable()
        glDisable(GL_LIGHTING)
        glColor4f(1,1,1,1)
        glBegin(GL_QUADS)
        glTexCoord2f(0,1)
        glVertex2f(x,y)
        glTexCoord2f(0,0)
        glVertex2f(x,y+h)
        glTexCoord2f(1,0)
        glVertex2f(x+w,y+h)
        glTexCoord2f(1,1)
        glVertex2f(x+w,y)
        glEnd()
        self.disable()

class MyWidgetPlugin(GLWidgetPlugin):
    def __init__(self,taskGen):
        GLWidgetPlugin.__init__(self)
        self.world = taskGen.world
        self.sendMilestone = False
        self.initialPose = False
        self.tuckArmPose = False
        self.selectedLimb = "both"
        self.gripperControl = False
        self.gripperState = "open"
        self.trackPosition = False
        self.trackPosition_lastState = False


    def initialize(self):
        GLWidgetPlugin.initialize(self)
        self.images = {}
        self.images['left'] = GLTexture("UI/Resources/left-arm.png")
        self.images['right'] = GLTexture("UI/Resources/right-arm.png")
        self.images['both'] = GLTexture("UI/Resources/both-arm.png")
        return True

    def display(self):

        robot = self.world.robot(0)
        oldColors = []
        for i in range(robot.numLinks()):
            #c = robot.link(i).appearance().getColor()
            c = [0.5,0.5,0.5,1.0]
            oldColors.append(c)
            robot.link(i).appearance().setColor(c[0],c[1],c[2],0.5)
        GLWidgetPlugin.display(self)
        for i in range(robot.numLinks()):
            c = oldColors[i]
            robot.link(i).appearance().setColor(c[0],c[1],c[2],c[3])

    def display_screen(self):
        glRasterPos(20,30)
        glColor3f(1,1,1)
        glDisable(GL_LIGHTING)
        if self.selectedLimb in self.images:
            self.images[self.selectedLimb].blit(20,40)

    def print_usage(self):
        print "keyboard functions: "
        print "s - swtich between left arm, right arm, and both arm"
        print "g - swtich between open and close gripper"
        print "i - set arm to initial position"
        print "p - switch between start and stop position tracking"

    def keyboardfunc(self,c,x,y):
        if c==' ':
            self.sendMilestone = True
        # == initialization
        elif c == 'i':
            self.initialPose = True
            print "Set to initial position: Untuck arm"
        elif c == 's':
            if self.selectedLimb == "both":
                self.selectedLimb = "right"
            elif self.selectedLimb == "right":
                self.selectedLimb = "left"
            elif self.selectedLimb == "left":
                self.selectedLimb = "both"
            print "Controlling the", self.selectedLimb, "arm"
        elif c == 't':
            self.tuckArmPose = True
            print "Tuck arm ... "
        elif c == 'h':
            self.print_usage()
        elif c == 'p':
            if self.trackPosition == False:
                self.trackPosition = True
                print "Start tracking position"
            # if self.trackPosition == False:
            #     self.trackPosition = True
            #     print "Start tracking position"
            # elif self.trackPosition == True:
            #     self.trackPosition = False
            #     print "Stop tracking position"
        elif c == 'g':
            if self.gripperState == "open":
                self.gripperControl = True
                self.gripperState = "close"
                print "Close gripper"
            elif self.gripperState == "close":
                self.gripperControl = True
                self.gripperState = "open"
                print "Open gripper"


    def eventfunc(self,type,args):
        """TODO: connect this up to GUI buttons"""
        if type=='button':
            if args=='send':
                self.sendMilestone = True


class ServiceThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self._kill = False
        self.updateFreq = 60
        self.eeGetter_left = TopicListener(system_state_addr,'.robot.endEffectors.0.xform.sensed', self.updateFreq)
        self.eeGetter_right = TopicListener(system_state_addr,'.robot.endEffectors.1.xform.sensed', self.updateFreq)

    def run(self):
        while(not self._kill):
            asyncore.loop(timeout = 1.0/self.updateFreq, count=1)
    def kill(self):
        self._kill = True

class KeyboardAutoTrackTaskGenerator(TaskGenerator):
    """Allows the user to interact with the model by right clicking
    using the mouse and pressing space bar to send the milestone.

    Also demonstrates how to write a Python plugin task with an OpenGL
    widget.
    """
    def __init__(self):
        self.plugin = None
        self.q = None

    def callback(self, data):
        self.marker = data
        self.marker_pos[0] = self.marker.pose.position.x
        self.marker_pos[1] = self.marker.pose.position.y
        self.marker_pos[2] = self.marker.pose.position.z
        #=== transform to world coordinate
        marker = se3.apply(kinect_frame_to_pedestal, tuple(self.marker_pos))
        self.marker_pos_tf = list(marker)
        self.marker_pos_tf[2] = self.marker_pos_tf[2] + self.base_height
        # print "marker_pos_tf = ", self.marker_pos_tf

    def name(self): return "Keyboard"

    def init(self,world):
        self.world = world
        self.robotPoser = RobotPoser(world.robot(0))
        self.plugin = MyWidgetPlugin(self)
        self.plugin.addWidget(self.robotPoser)
        self.robot = world.robot(0)

        left_arm_link_names = ['left_upper_shoulder','left_lower_shoulder','left_upper_elbow','left_lower_elbow','left_upper_forearm','left_lower_forearm','left_wrist']
        right_arm_link_names = ['right_upper_shoulder','right_lower_shoulder','right_upper_elbow','right_lower_elbow','right_upper_forearm','right_lower_forearm','right_wrist']
        base_link_names = ['base_x','base_y','base_yaw']
        self.left_arm_link_indices = [self.robot.link(l).index for l in left_arm_link_names]
        self.right_arm_link_indices = [self.robot.link(l).index for l in right_arm_link_names]
        self.base_link_indices = [self.robot.link(l).index for l in base_link_names]
        if any(v < 0 for v in self.left_arm_link_indices+self.right_arm_link_indices):
            raise ValueError("Robot in world does not have Baxter's torso?")

        # == subscribe marker position
        self.marker = Marker()
        self.base_height = 1.1
        self.marker_pos = [0.0, 0.0, 0.0]
        self.marker_pos_tf = [0.0, 0.0, 0.0]
        rospy.init_node('marker_listener', anonymous=True)
        rospy.Subscriber("/Marker_glove", Marker, self.callback, None)

        #== auto tracking parameter
        self.tracking_speed_ratio = 1.0
        self.tracking_vel = [0.0, 0.0, 0.0]
        self.left_ee_t = [0.0, 0.0, 0.0]
        self.right_ee_t = [0.0, 0.0, 0.0]
        self.reactive_vel_left_ee = [0.0, 0.0, 0.0]
        self.reactive_vel_right_ee = [0.0, 0.0, 0.0]


        return True

    def start(self):
        self._status = 'ok'
        self.robotPoser.set(self.world.robot(0).getConfig())
        self.serviceThread = ServiceThread()
        self.serviceThread.start()
        return True

    def stop(self):
        if self.serviceThread:
            self.serviceThread.kill()
            print "Waiting for thread join..."
            self.serviceThread.join()
            print "Done"
            self.serviceThread = None

    def getEE(self):
        R1,t1 = self.serviceThread.eeGetter_left.get()
        R2,t2 = self.serviceThread.eeGetter_right.get()
        self.left_ee_t = list(t1)
        self.right_ee_t = list(t2)
        print "left hand EE = ", self.left_ee_t
        print "right hand EE = ", self.right_ee_t

    def cal_reactive_vel(self, pos_hand, pos_target, vel_ratio):
        reactive_vel = [0.0, 0.0, 0.0]
        pos_diff = vectorops.sub(tuple(pos_target), tuple(pos_hand))
        pos_diff_norm = vectorops.norm(pos_diff)
        if pos_diff_norm >= 0.02:
            vel = vectorops.mul(pos_diff, vel_ratio)
            reactive_vel = list(vel)
        return reactive_vel


    def get(self):
        res = self.do_logic()
        return res


    def do_logic(self):

        rospy.Subscriber("/Marker_glove", Marker, self.callback, None)

        ''' retract arm to initial position '''

        if self.plugin.initialPose:
            self.plugin.initialPose = False
            self.q = self.robotPoser.get()
            if self.plugin.selectedLimb == "both":
                for i in range(len(self.left_arm_link_indices)):
                    idx = self.left_arm_link_indices[i]
                    self.q[idx] = q_init_left[i]
                for i in range(len(self.right_arm_link_indices)):
                    idx = self.right_arm_link_indices[i]
                    self.q[idx] = q_init_right[i]
                print "set both arms to initial pose"
            if self.plugin.selectedLimb == "left":
                for i in range(len(self.left_arm_link_indices)):
                    idx = self.left_arm_link_indices[i]
                    self.q[idx] = q_init_left[i]
                print "set left arms to initial pose"
            if self.plugin.selectedLimb == "right":
                for i in range(len(self.right_arm_link_indices)):
                    idx = self.right_arm_link_indices[i]
                    self.q[idx] = q_init_right[i]
                print "set right arms to initial pose"
            self.robotPoser.set(self.q)

        if self.plugin.tuckArmPose:
            self.plugin.tuckArmPose = False
            self.q = self.robotPoser.get()
            if self.plugin.selectedLimb == "both":
                for i in range(len(self.left_arm_link_indices)):
                    idx = self.left_arm_link_indices[i]
                    self.q[idx] = q_tuckarm_left[i]
                for i in range(len(self.right_arm_link_indices)):
                    idx = self.right_arm_link_indices[i]
                    self.q[idx] = q_tuckarm_right[i]
                print "Tuck both arms"
            if self.plugin.selectedLimb == "left":
                for i in range(len(self.left_arm_link_indices)):
                    idx = self.left_arm_link_indices[i]
                    self.q[idx] = q_tuckarm_left[i]
                print "Tuck left arm"
            if self.plugin.selectedLimb == "right":
                for i in range(len(self.right_arm_link_indices)):
                    idx = self.right_arm_link_indices[i]
                    self.q[idx] = q_tuckarm_right[i]
                print "Tuck right arm"
            self.robotPoser.set(self.q)


        ''' Control - Robot arm and mobile base'''

        if self.plugin.sendMilestone:
            self.plugin.sendMilestone = False
            self.q = self.robotPoser.get()
            qcmd = self.world.robot(0).getConfig()
            baseMoved = any(qcmd[i] != self.q[i] for i in self.base_link_indices)
            leftArmMoved = any(qcmd[i] != self.q[i] for i in self.left_arm_link_indices)
            rightArmMoved = any(qcmd[i] != self.q[i] for i in self.right_arm_link_indices)
            moved = []
            if baseMoved: moved.append('base')
            if leftArmMoved: moved.append('left')
            if rightArmMoved: moved.append('right')
            targets = []
            if baseMoved: targets.append([self.q[i] for i in self.base_link_indices])
            if leftArmMoved: targets.append([self.q[i] for i in self.left_arm_link_indices])
            if rightArmMoved: targets.append([self.q[i] for i in self.right_arm_link_indices])
            print "Sending milestone",self.q,"parts",",".join(moved)
            #CONSTRUCT THE TASK HERE
            msg = {}
            msg['type'] = 'JointPose'
            msg['parts'] = moved
            msg['positions'] = targets
            msg['speed'] = 1
            msg['safe'] = 0
            return msg

        ''' Control - gripper '''
        if self.plugin.gripperControl == True:
            self.plugin.gripperControl = False
            if self.plugin.gripperState == "open":
                p = [1.0, 1.0, 1.0, 1.0]
                if self.plugin.selectedLimb == "both":
                    p = p + p
            elif self.plugin.gripperState == "close":
                p = [0.3, 0.3, 0.3, 1.0]
                if self.plugin.selectedLimb == "both":
                    p = p + p
            msg = {}
            msg['type'] = 'Gripper'
            msg['limb'] = self.plugin.selectedLimb
            msg['position'] = p
            msg['force'] = 0.2
            msg['speed'] = 0.5
            return msg


        ''' Auto tracking '''

        if self.plugin.trackPosition == True:
            self.plugin.trackPosition_lastState = True
            self.plugin.trackPosition = False
            print "move to target: ", self.marker_pos_tf
            self.getEE()
            if self.plugin.selectedLimb == "both":
                self.reactive_vel_left_ee = self.cal_reactive_vel(self.left_ee_t, self.marker_pos_tf, self.tracking_speed_ratio)
                self.reactive_vel_right_ee = self.cal_reactive_vel(self.right_ee_t, self.marker_pos_tf, self.tracking_speed_ratio)
                print "Use both left or right arm to track"
                linear_vel = self.reactive_vel_left_ee + self.reactive_vel_right_ee
                angular_vel = [0.0, 0.0, 0.0] + [0.0, 0.0, 0.0]
            if self.plugin.selectedLimb == "left":
                self.reactive_vel_left_ee = self.cal_reactive_vel(self.left_ee_t, self.marker_pos_tf, self.tracking_speed_ratio)
                print "Move left arm to target at velocity: ", self.reactive_vel_left_ee
                linear_vel = self.reactive_vel_left_ee
                angular_vel = [0.0, 0.0, 0.0]
            if self.plugin.selectedLimb == "right":
                self.reactive_vel_right_ee = self.cal_reactive_vel(self.right_ee_t, self.marker_pos_tf, self.tracking_speed_ratio)
                print "Move right arm to target"
                linear_vel = self.reactive_vel_right_ee
                angular_vel = [0.0, 0.0, 0.0]
            msg = {}
            msg['type'] = 'CartesianVelocity'
            msg['limb'] = self.plugin.selectedLimb
            msg['linear'] = linear_vel
            msg['angular'] = angular_vel
            return msg

        if self.plugin.trackPosition_lastState == True:
            self.plugin.trackPosition_lastState = False
            print "last command is cartesian velocity ... "
            if self.plugin.selectedLimb == "both":
                linear_vel = [0.0, 0.0, 0.0] + [0.0, 0.0, 0.0]
                angular_vel = [0.0, 0.0, 0.0] + [0.0, 0.0, 0.0]
            if self.plugin.selectedLimb == "left":
                linear_vel = [0.0, 0.0, 0.0]
                angular_vel = [0.0, 0.0, 0.0]
            if self.plugin.selectedLimb == "right":
                linear_vel = [0.0, 0.0, 0.0]
                angular_vel = [0.0, 0.0, 0.0]
            msg = {}
            msg['type'] = 'CartesianVelocity'
            msg['limb'] = self.plugin.selectedLimb
            msg['linear'] = linear_vel
            msg['angular'] = angular_vel
            return msg

        return None







    def glPlugin(self):
        return self.plugin

def make():
    return KeyboardAutoTrackTaskGenerator()


# === send raw_joint_position command
