# Import Modules
from task_generator import TaskGenerator
import rospy
import time
import os
import sys
ebolabot_root = os.getenv("EBOLABOT_PATH",".")
sys.path.append(ebolabot_root)
from Common.system_config import EbolabotSystemConfig
import threading
import copy
import math

from OpenGL.GL import *
from klampt.vis import gldraw
from klampt.vis.glinterface import GLPluginInterface as GLPluginBase
from klampt.vis.glcommon import GLWidgetPlugin

from visualization_msgs.msg import Marker

from klampt.math import so3,se3,vectorops
from sspp.service import Service
from sspp.topic import MultiTopicListener
import asyncore
import time
from UI.utils.gltexture import *


#Configuration variable: where's the state server?
system_state_addr = EbolabotSystemConfig.getdefault_ip('state_server_computer_ip',('localhost',4568))

#Configuration variable: where's the haptic service?
try:
    #default 'tcp://192.168.1.128:3456' ?
    haptic_service_addr = EbolabotSystemConfig.get_ip('haptic_server_computer_ip')
except Exception:
    print "Haptic device server not configured. Please configure the appropriate variable in",EbolabotSystemConfig.system_config_fn
    exit(1)

modes = ['normal','gripper', 'tracking', 'switchPose', 'basePosition', 'baseOrientation', 'gripperIncremental', 'sync']

#TODO: not working at the moment
#set this to -1 to have view-centric control, looking at the face of the robot
viewToWorldScaleXY = 1

defaultDeviceState = {'mode':'normal',
                      'time': 0.0,
                      'buttonDown':False,
                      'position': [0.0, 0.0, 0.0],
                      'rotationMoment': [0.0, 0.0, 0.0],
                      'jointAngle': [0.0, 0.0, 0.0], 
                      'gimbalAngle': [0.0, 0.0, 0.0], 
                      "velocity": [0.0, 0.0, 0.0],
                      "angularVelocity": [0.0, 0.0, 0.0],
                      'rotationScale':3,
                      'positionScale':7.5, 
                      'deviceInitialTransform': None,
                      'devicePreviousTransform': None,
                      'deviceCurrentTransform': None,
                      'linearVel': [0.0, 0.0, 0.0],
                      'angularVel': [0.0, 0.0, 0.0],
                      'newupdate': False
                      }

scale_m2cm = 100
endEffectorIndices = [25,45]


#Transform from device to robot/simulation (in column format) 
#in tool handle frame
#+z is backward axis, rotates around handle (roll)
#+x is right, rotation ins pitch
#+y is up, rotation is yaw
worldToHandle = ([0,-1,0,   0,0,1,  -1,0,0 ],  [0,0,0])
#in the world frame, +z is up (yaw), +y is left (pitch), and +x is forward (roll)

#then, the handle transform to the user's reference (handle pointing forward in the) transform is
handleToUser = ([-1,0,0,   0,0,1,  0,1,0],[0,0,0])
#This is the output of the user frame
worldToUser = (so3.inv([0,1,0,   0,0,1,  1,0,0 ]),  [0,0,0])
userToWorld = (so3.inv([0,1,0,   -1,0,0,  0,0,1]),   [0,0,0])

debugHapticTransform = False

def se3translation(v):
    return (so3.identity(),v)

def deviceToViewTransform(R,t):
    Ruser = so3.mul(so3.inv(handleToUser[0]),R)
    R = so3.mul(userToWorld[0],so3.mul(Ruser,worldToUser[0]))
    return (R,so3.apply([0,1,0,    0,0,1,   1,0,0],t))

def deviceToWidgetTransform(R,t):
    """Given a device transform in the viewpoint-local frame, map to the device transform in world coordinates."""
    Tview = so3.rotation((0,0,1),math.pi),[0,0,0]
    return se3.mul(Tview,deviceToViewTransform(R,t))

def baseVelocityViewToWorld(twist):
    """Given dx,dy,dheta, maps this from view-centric coordinates to world coordinates"""
    return [-twist[0],-twist[1],twist[2]]

endEffectorLocalTransforms = [(so3.identity(),(0,0,0.08)),
                              (so3.identity(),(0,0,0.08))]

''' frame transformation: from kienct frame to pedestal frame  - determined by extrinsic calibration '''
q = (0.564775,0.425383,0.426796,0.563848)
R1 = so3.from_quaternion(q)
t1 = (0.227677,0.0916972,0.0974174)
kinect_frame_to_pedestal = (R1, t1)

''' default pose:
- front: move arm to the front of robot (equal to untuck arm)
- side: default - move arm to side so the view of kinect will not be blocked; if recorded arm posture exist, switch to recorded arm posture


'''
alignArmPose = {}

alignArmPose['front'] = [[-0.08897088559570313, -0.9675583808532715, -1.2034079267211915, 1.7132575355041506, 0.6776360122741699, 1.0166457660095216, -0.5065971546203614],
                           [0.08897088559570313, -0.9675583808532715, 1.2034079267211915, 1.7132575355041506, -0.6776360122741699, 1.0166457660095216, 0.5065971546203614]]

alignArmPose['side'] = [[0.192255, 0.232187, -1.63754, 0.99, 0.281169, 1.41366, 1.48787],
                           [-0.192255, 0.232187, 1.63754, 0.99, -0.281169, 1.41366, -1.48787]]


alignArmPose['push'] = [[-0.160088, -1.10271, -1.12767, 1.84711, 0.629936, 0.990431, 0.81607],
                           [0.160088, -1.10271, 1.12767, 1.84711, -0.629936, 0.990431, -0.851607]]


# alignArmPose['straight'] = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
#                            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]


recordArmPose = {}

recordArmPose['front'] = [[-0.08897088559570313, -0.9675583808532715, -1.2034079267211915, 1.7132575355041506, 0.6776360122741699, 1.0166457660095216, -0.5065971546203614],
                           [0.08897088559570313, -0.9675583808532715, 1.2034079267211915, 1.7132575355041506, -0.6776360122741699, 1.0166457660095216, 0.5065971546203614]]

recordArmPose['record'] = [[0.58897088559570313, -0.9675583808532715, -1.2034079267211915, 1.7132575355041506, 0.6776360122741699, 1.0166457660095216, -0.5065971546203614],
                           [-0.58897088559570313, -0.9675583808532715, 1.2034079267211915, 1.7132575355041506, -0.6776360122741699, 1.0166457660095216, 0.5065971546203614]]



armPoseSign = [-1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0]
endEffectorSign = [1.0, -1.0, 1.0]


# ===========================================================


''' haptic mode: arm, base, auto, fix, joint '''
HapticModeButton = "arm"
RecordPoseButton = False
CollisionDetectionEnabled = True



class MyWidgetPlugin(GLPluginBase):
    def __init__(self,taskGen):
        GLPluginBase.__init__(self)
        self.taskGen = taskGen
        self.displayHelpFlag = False
        self.BaseControlMode = False

    def initialize(self):
        GLPluginBase.initialize(self)
        global HapticModeButton
        global RecordPoseButton

        self.images = {}
        self.images['Cartesian position'] = GLTexture("UI/Resources/cartesian-translation.png")
        self.images['Cartesian rotation'] = GLTexture("UI/Resources/cartesian-rotation.png")
        self.images['Joint angles'] = GLTexture("UI/Resources/joint.png")
        self.images['left'] = GLTexture("UI/Resources/left-arm.png")
        self.images['right'] = GLTexture("UI/Resources/right-arm.png")

        self.images['normal'] = GLTexture("UI/Resources/EndEffectorControl.png")
        self.images['gripper'] = GLTexture("UI/Resources/GripperControl.png")
        self.images['switchPose'] = GLTexture("UI/Resources/SwitchPose.png")
        self.images['tracking'] = GLTexture("UI/Resources/AutoTracking.png")
        self.images['gripperIncremental'] = GLTexture("UI/Resources/GripperIncremental.png")
        self.images['help'] = GLTexture("UI/Resources/Help.png")

        self.images['basePosition'] = GLTexture("UI/Resources/BasePosition.png")
        self.images['baseOrientation'] = GLTexture("UI/Resources/BaseOrientation.png")

        
        self.images['handCoupling'] = GLTexture("UI/Resources/HandCoupling.png")
        self.images['sync'] = GLTexture("UI/Resources/sync.png")
        self.images['armCoupling'] = GLTexture("UI/Resources/ArmCoupling.png")
        self.images['armShift'] = GLTexture("UI/Resources/ArmShift.png")
        self.images['recordPose'] = GLTexture("UI/Resources/RecordPose.png")

        self.images['horizontalalign'] = GLTexture("UI/Resources/HorizontalAlign.png")
        self.images['verticalalign'] = GLTexture("UI/Resources/VerticalAlign.png")
        self.images['frontalign'] = GLTexture("UI/Resources/Knob.png")
        self.images['tablealign'] = GLTexture("UI/Resources/TableAlign.png")
        self.images['onlyPosition'] = GLTexture("UI/Resources/cartesian-translation.png")

        self.images['lowerArm'] = GLTexture("UI/Resources/LowerArm.png")
        self.images['upperArm'] = GLTexture("UI/Resources/UpperArm.png")
        self.images['wrist'] = GLTexture("UI/Resources/Wrist.png")

        self.images['ArmMode'] = GLTexture("UI/Resources/ArmMode.png")
        self.images['BaseMode'] = GLTexture("UI/Resources/BaseMode.png")
        self.images['AutoMode'] = GLTexture("UI/Resources/AutoMode.png")
        self.images['JointMode'] = GLTexture("UI/Resources/JointMode.png")
        self.images['FixMode'] = GLTexture("UI/Resources/Fix.png")

        return True

    def keyboardfunc(self,c,x,y):
        global RecordPoseButton
        global HapticModeButton
        global CollisionDetectionEnabled

        if c=='h':
            if self.displayHelpFlag == False:
                print "Display help ... "
                self.displayHelpFlag = True
            else:
                self.displayHelpFlag = False

        elif c=='b':
            if HapticModeButton == 'arm':
                HapticModeButton = 'base'
            elif HapticModeButton == 'base':
                HapticModeButton = 'auto'
            elif HapticModeButton == 'auto':
                HapticModeButton = 'fix'
            elif HapticModeButton == 'fix':
                HapticModeButton = 'joint'
            elif HapticModeButton == 'joint':
                HapticModeButton = 'arm'

        elif c=='r':
            if RecordPoseButton == False:
                RecordPoseButton = True
            else:
                RecordPoseButton = False

        elif c=='c':
            CollisionDetectionEnabled = not CollisionDetectionEnabled
            print "Collision detection toggled to:",CollisionDetectionEnabled


    def display(self):
        T1 = self.taskGen.getDesiredCartesianPose('left',0)
        T2 = self.taskGen.getDesiredCartesianPose('right',1)
        baseTransform = self.taskGen.world.robot(0).link(2).getTransform()
        glEnable(GL_LIGHTING)
        if T1 is not None:
            gldraw.xform_widget(se3.mul(baseTransform,T1),0.2,0.03,fancy=True,lighting=True)
        if T2 is not None:
            gldraw.xform_widget(se3.mul(baseTransform,T2),0.2,0.03,fancy=True,lighting=True)

        dstate = self.taskGen.serviceThread.hapticupdater.deviceState

        if debugHapticTransform:
            for deviceState in dstate:
                #debugging
                #this is the mapping from the handle to the user frame
                Traw = (so3.from_moment(deviceState['rotationMoment']),deviceState['position'])
                Tuser = se3.mul(se3.inv(handleToUser),Traw)
                T = se3.mul(userToWorld,se3.mul(Tuser,worldToUser))
                T = (T[0],so3.apply([0,-1,0,    0,0,1,   -1,0,0],T[1]))
                T = deviceToViewTransform(Traw[0],Traw[1])
                gldraw.xform_widget(se3.mul(se3translation([-1,0,0]),Traw),0.5,0.05,lighting=True,fancy=True)
                gldraw.xform_widget(se3.mul(se3translation([-0.5,0,0]),Tuser),0.5,0.05,lighting=True,fancy=True)
                #gldraw.xform_widget(se3.mul(se3translation([-0.5,0,0]),se3.mul(Traw,worldToHandle)),0.5,0.05,lighting=True,fancy=True)
                gldraw.xform_widget(T,0.5,0.05,lighting=True,fancy=True)

                break

    def display_screen(self):

        global HapticModeButton
        glRasterPos(20,30)
        glColor3f(1,1,1)
        glDisable(GL_LIGHTING)

        dstateMode = self.taskGen.controlMode()
        for i in range(0, len(dstateMode)):
            if i == 0:
                self.images[dstateMode[i]].blit(20,40)
            elif i == 1:
                self.images[dstateMode[i]].blit(40+64,40)

        if self.displayHelpFlag == True:
            self.images['help'].blit(40+64,60+64)

        if HapticModeButton == 'arm':
            self.images['ArmMode'].blit(20,60+64)
        elif HapticModeButton == 'base':
            self.images['BaseMode'].blit(20,60+64)
        elif HapticModeButton == 'auto':
            self.images['AutoMode'].blit(20,60+64)
        elif HapticModeButton == 'fix':
            self.images['FixMode'].blit(20,60+64)
        elif HapticModeButton == 'joint':
            self.images['JointMode'].blit(20,60+64)


    def eventfunc(self,type,args):
        """TODO: connect this up to GUI buttons"""
        pass
# modes = ['normal','gripper', 'tracking', 'switchPose', 'basePosition', 'baseOrientation', 'gripperIncremental']


def toggleMode(dstate):
    global HapticModeButton
    if HapticModeButton == 'arm':
        if dstate['mode'] == 'normal':
            dstate['mode'] = 'gripper'
        elif dstate['mode'] == 'gripper':
            dstate['mode'] = 'gripperIncremental'
        elif dstate['mode'] == 'gripperIncremental':
            dstate['mode'] = 'switchPose'
        elif dstate['mode'] == 'switchPose':
            dstate['mode'] = 'normal'    
        else:
            dstate['mode'] = 'normal'
    elif HapticModeButton == 'base':
        if dstate['mode'] == 'basePosition':
            dstate['mode'] = 'baseOrientation'
        elif dstate['mode'] == 'baseOrientation':
            dstate['mode'] = 'basePosition'
        else:
            dstate['mode'] = 'basePosition'
    elif HapticModeButton == 'auto':
        if dstate['mode'] == 'switchPose':
            dstate['mode'] = 'sync'
        elif dstate['mode'] == 'sync':
            dstate['mode'] = 'armCoupling'
        elif dstate['mode'] == 'armCoupling':
            dstate['mode'] = 'armShift'
        elif dstate['mode'] == 'armShift':
            dstate['mode'] = 'handCoupling'
        elif dstate['mode'] == 'handCoupling':
            dstate['mode'] = 'recordPose'
        elif dstate['mode'] == 'recordPose':
            dstate['mode'] = 'switchPose'
        else:
            dstate['mode'] = 'switchPose'
    elif HapticModeButton == 'fix':
        if dstate['mode'] == 'switchPose':
            dstate['mode'] = 'tablealign'
        elif dstate['mode'] == 'tablealign':
            dstate['mode'] = 'onlyPosition'
        elif dstate['mode'] == 'onlyPosition':
            dstate['mode'] = 'frontalign'
        elif dstate['mode'] == 'frontalign':
            dstate['mode'] ='horizontalalign'            
        elif dstate['mode'] == 'horizontalalign':
            dstate['mode'] = 'verticalalign'
        elif dstate['mode'] == 'verticalalign':
            dstate['mode'] = 'switchPose'
        else:
            dstate['mode'] = 'switchPose'
    elif HapticModeButton == 'joint':
        if dstate['mode'] == 'upperArm':
            dstate['mode'] = 'lowerArm'
        elif dstate['mode'] == 'lowerArm':
            dstate['mode'] = 'wrist'
        elif dstate['mode'] == 'wrist':
            dstate['mode'] = 'upperArm'
        else:
            dstate['mode'] = 'upperArm'


class HapticWidgetUpdateService (Service):
    """Reads from a HapticService to update the device state"""
    def __init__(self,addr):
        Service.__init__(self)
        self.open(addr,asServer=False)
        self.viewer = None
        self.widgetGetters = None
        self.deviceState = []
        self.numMessages = 0
        self.lastUpdateTime = time.time()
        self.sentHapticFeedback = False

    def close(self):
        if self.sentHapticFeedback:
            #turn off haptic feedback
            msg = {}
            msg['type'] = 'HapticForceCommand'
            msg['device'] = 0
            msg['enabled'] = 0
            self.sendMessage(msg)
            msg['device'] = 1
            self.sendMessage(msg)
            #loop until all messages are sent
            while self.writable():
                asyncore.loop(timeout = 0.02, count=100, map=self.map)
        Service.close(self)

    def onMessage(self,msg):
        global defaultDeviceState
        #print "Getting haptic message"
        #print msg
        self.numMessages += 1
        if msg['type'] != 'MultiHapticState':
            print "Strange message type",msg['type']
            return
        if len(self.deviceState)==0:
            print "Adding",len(msg['devices'])-len(self.deviceState),"haptic devices"
            while len(self.deviceState) < len(msg['devices']):
                self.deviceState.append(defaultDeviceState.copy())

        if len(msg['devices']) != len(self.deviceState):
            print "Incorrect number of devices in message:",len(msg['devices'])
            return

        #change overall state depending on button 2
        if 'events' in msg:
            for e in msg['events']:
                #print e['type']
                if e['type']=='b2_down':
                    dstate = self.deviceState[e['device']]
                    toggleMode(dstate)
                    print 'Changing device',e['device'],'to state',dstate['mode']

        for i in range(len(self.deviceState)):
            dmsg = msg['devices'][i]
            dstate = self.deviceState[i]
            #  ===== read from msg =====
            dstate['position'] = dmsg['position']
            dstate['rotationMoment'] = dmsg['rotationMoment']
            dstate['velocity'] = dmsg['velocity']
            dstate['angularVelocity'] = dmsg['angularVelocity']
            dstate['jointAngles'] = dmsg['jointAngles']
            #dstate['gimbalAngle'] = dmsg['gimbalAngle']
            oldTime = dstate['time']
            dstate['time'] = dmsg['time']
            #print "position",dmsg['position']
            #print "rotation moment",dmsg['rotationMoment']
            #print "angular velocity",dmsg['angularVelocity']
            dstate['deviceCurrentTransform'] = deviceToWidgetTransform(so3.from_moment(dmsg['rotationMoment']),dmsg['position'])
            if dmsg['button1'] == 1:
                #drag widget if button 1 is down
                oldButtonDown = dstate['buttonDown']
                dstate['buttonDown'] = True
                #if oldButtonDown == False:
                #    print "start haptic motion ... "
                #movingmb

                if dstate['buttonDown']:
                    # --- take initial position when button 1 is pressed----
                    if not oldButtonDown:
                        dstate['devicePreviousTransform'] = dstate['deviceCurrentTransform']
                        dstate['deviceInitialTransform'] = dstate['devicePreviousTransform']
                        continue

                    newTime = dstate['time']
                    if newTime != oldTime:
                        # print "previous position = ", dstate['devicePreviousTransform'][1]
                        # print "current position = ", dstate['deviceCurrentTransform'][1]
                        timeInterval = newTime - oldTime
                        #print "========================"
                        #print "time = ", timeInterval

                        delta_Pos = vectorops.mul(vectorops.sub(dstate['deviceCurrentTransform'][1], dstate['devicePreviousTransform'][1]), dstate['positionScale'])
                        vel   = vectorops.div(delta_Pos, timeInterval)

                        delta_Moment = vectorops.mul(tuple(so3.moment(so3.mul(dstate['deviceCurrentTransform'][0], so3.inv(dstate['devicePreviousTransform'][0])))), dstate['rotationScale'])
                        angvel = vectorops.div(delta_Moment, timeInterval)

                        # print "vel = [%2.4f %2.4f %2.4f]" % (vel[0], vel[1], vel[2])
                        # print "angvel = [%2.4f %2.4f %2.4f]" % (angvel[0], angvel[1], angvel[2])
                        dstate['linearVel'] = list(vel)
                        dstate['angularVel'] = list(angvel)
                        
                        dstate['newupdate'] = True

                #special modes set "newupdate" is true even when button is not down
                if dstate['mode'] == 'tracking' or dstate['mode'] == 'switchPose':
                    dstate['time'] = dmsg['time']
                    newTime = dstate['time']
                    if newTime != oldTime:
                        dstate['newupdate'] = True
                    else:
                        dstate['newupdate'] = False
                else:
                    pass 
                    
            else:
                dstate['buttonDown'] = False 

            #end of loop, store previous transform
            dstate['devicePreviousTransform'] = dstate['deviceCurrentTransform']

    def onUpdate(self):
        t = time.time()
        #print self.numMessages,"haptic messages read over time",t-self.lastUpdateTime
        self.numMessages = 0
        self.lastUpdateTime = t

    def activateDrag(self,kD,device='all'):
        self.sentHapticFeedback = True
        if device == 'all':
            devices = [0,1]
        else:
            devices = [device]
        msg = {}
        msg['type'] = 'HapticForceCommand'
        msg['enabled'] = 1
        msg['damping'] = [-kD]
        msg['forceCenter'] = [0,0,0]
        for device in devices:
            msg['device'] = device
            self.sendMessage(msg)
        
    def activateSpring(self,kP,kD,center=None,device='all'):
        self.sentHapticFeedback = True
        if device == 'all':
            devices = [0,1]
        else:
            devices = [device]
        msg = {}
        msg['type'] = 'HapticForceCommand'
        msg['enabled'] = 0
        msg['linear'] = [-kP]
        msg['damping'] = [-kD]
        if center is not None:
            msg['center'] = center
        for device in devices:
            msg['device'] = device
            if center is None:
                countdown = 10
                while self.numMessages == 0 and countdown > 0:
                    #need to wait for an update
                    time.sleep(0.05)
                    countdown -= 1
                if self.numMessages != 0:
                    print "Setting center to",self.deviceState[device]['position']
                    msg['center'] = self.deviceState[device]['position']
            self.sendMessage(msg)

class ServiceThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self._kill = False
        self.stateUpdateFreq = 50
        self.hapticupdater = HapticWidgetUpdateService(haptic_service_addr)

        self.state_listener = MultiTopicListener(system_state_addr,topics=None,rate=self.stateUpdateFreq)
        self.baseVelocitySensed = self.state_listener.listen('.robot.sensed.base.velocity')
        self.baseVelocityCommand = self.state_listener.listen('.robot.command.base.velocity')
        self.ArmPosition_left = self.state_listener.listen('.robot.sensed.left.q')
        self.ArmPosition_right = self.state_listener.listen('.robot.sensed.right.q')
        self.eeGetter_left = self.state_listener.listen('.robot.endEffectors.0.xform.destination')
        self.eeGetter_right = self.state_listener.listen('.robot.endEffectors.1.xform.destination')
        self.gripperGetter_left = self.state_listener.listen('.robot.gripper.left.positionCommand')
        self.gripperGetter_right = self.state_listener.listen('.robot.gripper.right.positionCommand')
        self.state_listener.setName("HapticGUIListener")

    def run(self):
        """Note: don't call ServiceThread.run(), call ServiceThread.start()"""
        #self.hapticupdater.run(1)
        while not self.hapticupdater.kill:
            #read from the haptic service
            asyncore.loop(timeout = 0.02, count=100, map=self.hapticupdater.map)
            self.hapticupdater.onUpdate()
            #listen to state topics
            asyncore.loop(timeout = 1.0/self.stateUpdateFreq, count=10)

        print "Closing haptic widget update service..."
        self.hapticupdater.close()
        self.hapticupdater = None
    def kill(self):
        self.hapticupdater.terminate()
        self._kill = True

    # def run(self):
    #     while(not self._kill):
    #         asyncore.loop(timeout = 1.0/self.updateFreq, count=1)
    # def kill(self):
    #     self._kill = True

class HapticTaskGenerator(TaskGenerator):
    def __init__(self):
        self.serviceThread = None
        self.lastMsg = None
        self.startTransforms = [None,None]
        self.CartesianPoseControl = False

        #=== gripper open and close control
        self.gripperStatus = [None, None]
        self.gripperControlFlag = False
        self.gripperControlTimer = [time.time(), time.time()]

        #=== gripper incremental control
        self.gripperSizeFlag = False
        self.gripperSizeTimer = [time.time(), time.time()]
        self.gripperControlRatio = 0.3
        self.gripperPosition = [[0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]]


        self.poseStatus = [None, None]
        self.poseControlFlag = False
        self.poseControlTimer = [time.time(), time.time()]

        #  === plugin function ===
        self.plugin = None
        self.dstateMode = []

        # === base control ===
        self.baseDevice = False
        self.basePosControlFlag = False
        self.baseOrtControlFlag = False
        self.basePosVelocityRatio = 0.5
        self.baseOrtVelocityRatio = 0.25
        self.basePosCommandTimer = [time.time(), time.time()]
        self.baseOrtCommandTimer = [time.time(), time.time()]

        self.baseSensedVelocity = [0.0, 0.0, 0.0]
        self.baseCommandVelocity = [0.0, 0.0, 0.0]

        # === joint control ===
        self.jointControlRatio = 0.25

        # == arm position ===
        self.ArmPosition = [[0.0]*7, [0.0]*7]

        # == subscribe marker position
        self.marker = Marker()
        self.base_height = 1.1
        self.marker_pos = [0.0, 0.0, 0.0]
        self.marker_pos_tf = [0.0, 0.0, 0.0]
        rospy.init_node('marker_listener', anonymous=True)

        #== auto tracking parameter
        self.autoTrackingControlFlag = False
        self.autoTrackingStatus = [None, None]
        self.autoTrackingTimer = [time.time(), time.time()]

        #== synchroniztion arm pose parameter
        self.syncControlFlag = False
        self.syncControlTimer = [time.time(), time.time()]

        #== coupling two hands parameter
        self.handCouplingControlFlag = False
        self.handCouplingControlTimer = [time.time(), time.time()]

        #== coupling two arms parameter
        self.armCouplingControlFlag = False
        self.armCouplingControlTimer = [time.time(), time.time()]

        #== arm shift parameter
        self.armShiftControlFlag = False
        self.armShiftControlTimer = [time.time(), time.time()]

        self.alignControlFlag = False
        self.alignControlTimer = [time.time(), time.time()]

        #== record pose parameter
        self.recordPoseControlFlag = False
        self.recordPoseStatus = [None, None]
        self.recordPoseTimer = [time.time(), time.time()]

        #== upper and lower limb joint control
        self.upperLimbControlFlag = False
        self.upperLimbTimer = [time.time(), time.time()]

        self.lowerLimbControlFlag = False
        self.lowerLimbTimer = [time.time(), time.time()]

        self.wristControlFlag = False
        self.wristTimer = [time.time(), time.time()]

        self.tracking_speed_ratio = 1.0
        self.robotEndEffectorPosition = [[0.0, 0.0, 0.0],[0.0, 0.0, 0.0]]
        self.robotEndEffectorTransform = [se3.identity(),se3.identity()]
        self.EEReactiveVelocity = [[0.0, 0.0, 0.0],[0.0, 0.0, 0.0]]



    def name(self): return "Haptic"

    def init(self,world):
        self.world = world
        self.robot = world.robot(0)

        self.gripperControlFlag = False
        self.gripperSizeFlag = False
        self.poseControlFlag = False
        self.syncControlFlag = False
        self.handCouplingControlFlag = False
        self.armCouplingControlFlag = False
        self.armShiftControlFlag = False
        self.alignControlFlag = False
        self.recordPoseControlFlag = False
        self.autoTrackingControlFlag = False
        self.basePosControlFlag = False
        self.baseOrtControlFlag = False
        self.upperLimbControlFlag = False
        self.lowerLimbControlFlag = False
        self.wristControlFlag = False


        return True

    def start(self):
        self.plugin = MyWidgetPlugin(self)
        self.lastMsg = None
        if self.serviceThread==None:
            self.serviceThread = ServiceThread()
            self.serviceThread.start()
        attached = False
        for i in range(5):
            time.sleep(0.1)
            if len(self.serviceThread.hapticupdater.deviceState)!=0:
                attached = True
                break
            if i % 5 == 1:
                print "HapticTaskGenerator: Waiting for haptic device tasks to be read..."
        if not attached:
            print "Haptic server doesn't appear to be running"
            self.serviceThread.kill()
            print "Waiting for thread join..."
            self.serviceThread.join()
            print "Done"
            self.serviceThread = None
        else:
            print "Haptic server is running"
            print "Turning on drag..."
            #2 is probably the highest damping that doesnt cause vibration
            #self.serviceThread.hapticupdater.activateDrag(2)
            self.serviceThread.hapticupdater.activateSpring(kP=20,kD=0,center=None)

        self.ros_subscribers = []
        self.ros_subscribers.append(rospy.Subscriber("/Marker_glove", Marker, self.marker_glove_callback, None))

        return attached

    def status(self):
        if self.serviceThread != None:
            return 'ok'
        else:
            return 'error'

    def messages(self):
        return []

    def stop(self):
        if self.serviceThread:
            self.serviceThread.kill()
            print "Waiting for thread join..."
            self.serviceThread.join()
            print "Done"
            self.serviceThread = None
        for s in self.ros_subscribers:
            s.unregister()
        self.plugin = None
        self.gripperControlFlag = False
        self.gripperSizeFlag = False
        self.poseControlFlag = False
        self.handCouplingControlFlag = False
        self.armCouplingControlFlag = False
        self.armShiftControlFlag = False
        self.alignControlFlag = False
        self.recordPoseControlFlag = False
        self.syncControlFlag = False
        self.autoTrackingControlFlag = False
        self.basePosControlFlag = False
        self.baseOrtControlFlag = False
        self.upperLimbControlFlag = False
        self.lowerLimbControlFlag = False
        self.wristControlFlag = False


    def controlMode(self):
        deviceState = self.serviceThread.hapticupdater.deviceState
        if len(deviceState)==0:
            print "No device state"
            return None

        self.dstateMode = []

        for i, dstate in enumerate(deviceState):
            self.dstateMode = self.dstateMode + [dstate['mode']]
        return self.dstateMode


    def marker_glove_callback(self, data):
        self.marker = data
        self.marker_pos[0] = self.marker.pose.position.x
        self.marker_pos[1] = self.marker.pose.position.y
        self.marker_pos[2] = self.marker.pose.position.z
        #=== transform to world coordinate
        marker = se3.apply(kinect_frame_to_pedestal, tuple(self.marker_pos))
        self.marker_pos_tf = list(marker)
        self.marker_pos_tf[2] = self.marker_pos_tf[2] + self.base_height
        # print "marker_pos_tf = ", self.marker_pos_tf

    def calReactiveVel(self, pos_hand, pos_target, vel_ratio):
        reactive_vel = [0.0, 0.0, 0.0]
        pos_diff = vectorops.sub(tuple(pos_target), tuple(pos_hand))
        pos_diff_norm = vectorops.norm(pos_diff)
        if pos_diff_norm >= 0.02:
            vel = vectorops.mul(pos_diff, vel_ratio)
            reactive_vel = list(vel)
        return reactive_vel


    def get(self):

        # === get robot status from system state service: robot end-effector position, gripper position
        self.getRobotStatus()

        msg = self.do_logic()
        self.last_message = msg
        return msg

    def gripperController(self):
        deviceState = self.serviceThread.hapticupdater.deviceState
        if len(deviceState)==0:
            print "No device state"
            return None
        msg = {}
        p = []
        self.gripperControlFlag = False
        for i in range(0,2):
            if self.gripperPosition[i]:
                p = p + self.gripperPosition[i]
            else:
                p = p + [1.0, 1.0, 1.0, 0.4]

        for i, dstate in enumerate(deviceState):
            if dstate['mode'] != 'gripper': continue
            if dstate['buttonDown']:
                time_now = time.time()
                if time_now - self.gripperControlTimer[i] > 0.05:
                    if i == 1:
                        gripperVel = -dstate['linearVel'][1]
                    else:
                        gripperVel = dstate['linearVel'][1]

                    if abs(gripperVel) < 0.005:
                            gripperVel = 0.0

                    self.gripperPosition[i] =  [v + gripperVel*self.gripperControlRatio for v in self.gripperPosition[i][0:3]] + [0.3]

                    for j in range(0,3):
                        if self.gripperPosition[i][j] > 1.0:
                            self.gripperPosition[i][j] = 1.0
                        elif self.gripperPosition[i][j] < 0.4:
                            self.gripperPosition[i][j] = 0.4

                    p[0+i*4:4+i*4] = self.gripperPosition[i]
                    msg['limb'] = "both"
                    msg['position'] = p
                    msg['type'] = 'Gripper'
                    msg['force'] = 0.4
                    msg['speed'] = 0.3
                    self.gripperControlTimer[i] = time_now
                    self.gripperControlFlag = True
        if self.gripperControlFlag == True:
            return msg
        else:
            return None


    def gripperIncrementalControl(self):
        deviceState = self.serviceThread.hapticupdater.deviceState
        if len(deviceState)==0:
            print "No device state"
            return None
        msg = {}
        p = []
        self.gripperSizeFlag = False

        for i in range(0,2):
            if self.gripperPosition[i]:
                p = p + self.gripperPosition[i]
            else:
                p = p + [1.0, 1.0, 1.0, 1.0]

        for i, dstate in enumerate(deviceState):
            if dstate['mode'] != 'gripperIncremental': continue

            if dstate['buttonDown']:
                if dstate['newupdate'] == True:
                    time_now = time.time()
                    if time_now - self.gripperSizeTimer[i] > 0.02:
                        if i == 1:
                            gripperVel = -dstate['linearVel'][1]
                        else:
                            gripperVel = dstate['linearVel'][1]
                        if abs(gripperVel) < 0.001:
                            gripperVel = 0.0

                        self.gripperPosition[i] =  [v + gripperVel*self.gripperControlRatio for v in self.gripperPosition[i][0:3]] + [1.0]

                        for j in range(0,3):
                            if self.gripperPosition[i][j] > 1.0:
                                self.gripperPosition[i][j] = 1.0
                            elif self.gripperPosition[i][j] < 0.3:
                                self.gripperPosition[i][j] = 0.3

                        p[0+i*4:4+i*4] = self.gripperPosition[i]
                        msg['limb'] = "both"
                        msg['position'] = p
                        msg['type'] = 'Gripper'
                        msg['force'] = 0.4
                        msg['speed'] = 0.3
                        self.gripperSizeTimer[i] = time_now
                        self.gripperSizeFlag = True
                        self.baseOrtControlFlag = False
        if self.gripperSizeFlag == True:
            return msg
        else:
            return None

    def autoTracking(self):
        deviceState = self.serviceThread.hapticupdater.deviceState
        if len(deviceState)==0:
            print "No device state"
            return None
        msg = {}

        self.autoTrackingControlFlag = False

        for i, dstate in enumerate(deviceState):
            if dstate['mode'] != 'tracking': continue
            if dstate['buttonDown']:
                # if dstate['newupdate'] == True:
                if self.autoTrackingControlFlag == False:
                    time_now = time.time()
                    if time_now - self.autoTrackingTimer[i] > 0.02:
                        print "Start auto tracking ... "
                        self.EEReactiveVelocity[i] = self.calReactiveVel(self.robotEndEffectorPosition[i], self.marker_pos_tf, self.tracking_speed_ratio)
                        linear_vel = self.EEReactiveVelocity[i]
                        angular_vel = [0.0, 0.0, 0.0]
                        msg['type'] = 'CartesianVelocity'
                        if i == 0:
                            msg['limb'] = "left"
                        if i == 1:
                            msg['limb'] = "right"
                        msg['linear'] = linear_vel
                        msg['angular'] = angular_vel
                        self.autoTrackingTimer[i] = time.time()
                        self.autoTrackingControlFlag = True
        if self.autoTrackingControlFlag == True:
            return msg
        else:
            return None

    def switchPoseController(self):
        deviceState = self.serviceThread.hapticupdater.deviceState
        if len(deviceState)==0:
            print "No device state"
            return None
        msg = {}
        pose = self.ArmPosition

        self.poseControlFlag = False

        for i, dstate in enumerate(deviceState):
            if dstate['mode'] != 'switchPose':
                self.poseStatus[i] = "push"
                continue
            if dstate['buttonDown']:
                time_now = time.time()
                if dstate['newupdate'] == True:
                    if time_now - self.poseControlTimer[i] > 0.5:
                        if self.poseStatus[i] == "front":
                            print "switch to side pose"
                            self.poseStatus[i] = "side"
                            pose[i] = alignArmPose['side'][i]
                        elif self.poseStatus[i] == "side":
                            print "switch to push pose"
                            self.poseStatus[i] = "push"
                            pose[i] = alignArmPose['push'][i]
                        elif self.poseStatus[i] == "push" or self.poseStatus[i] == None:
                            print "switch to front pose"
                            self.poseStatus[i] = "front"
                            pose[i] = alignArmPose['front'][i]
                        msg['type'] = "JointPose"
                        msg['parts'] = ['left', 'right']
                        msg['positions'] = pose
                        msg['speed'] = 1
                        msg['safe'] = int(CollisionDetectionEnabled)
                        self.poseControlTimer[i] = time_now
                        self.poseControlFlag = True
        if self.poseControlFlag == True:
            return msg
        else:
            return None

    def upperLimbController(self):
        deviceState = self.serviceThread.hapticupdater.deviceState
        if len(deviceState)==0:
            print "No device state"
            return None
        msg = {}
        pose = self.ArmPosition

        self.upperLimbControlFlag = False
        
        for i, dstate in enumerate(deviceState):
            if dstate['mode'] != 'upperArm':
                continue
            if dstate['buttonDown']:
                time_now = time.time()
                if dstate['newupdate'] == True:
                    if time_now - self.upperLimbTimer[i] > 0.2:
                        jointVel = dstate['linearVel']
                        for j in range(0,3):
                            if abs(jointVel[j]) > 0.5:   
                                pose[i][j] = jointVel[j]*self.jointControlRatio + self.ArmPosition[i][j]
                        msg['type'] = "JointPose"
                        msg['parts'] = ['left', 'right']
                        msg['positions'] = pose
                        msg['speed'] = 1
                        msg['safe'] = int(CollisionDetectionEnabled)
                        self.upperLimbTimer[i] = time_now
                        self.upperLimbControlFlag = True
        if self.upperLimbControlFlag == True:
            return msg
        else:
            return None

    def lowerLimbController(self):
        deviceState = self.serviceThread.hapticupdater.deviceState
        if len(deviceState)==0:
            print "No device state"
            return None
        msg = {}
        pose = self.ArmPosition

        self.lowerLimbControlFlag = False
        
        for i, dstate in enumerate(deviceState):
            if dstate['mode'] != 'lowerArm':
                continue
            if dstate['buttonDown']:
                time_now = time.time()
                if dstate['newupdate'] == True:
                    if time_now - self.lowerLimbTimer[i] > 0.2:
                        jointVel = dstate['linearVel']
                        for j in range(0,3):
                            if abs(jointVel[j]) > 0.5:
                                pose[i][j+3] = jointVel[j]*self.jointControlRatio + self.ArmPosition[i][j+3]

                        msg['type'] = "JointPose"
                        msg['parts'] = ['left', 'right']
                        msg['positions'] = pose
                        msg['speed'] = 1
                        msg['safe'] = int(CollisionDetectionEnabled)
                        self.upperLimbTimer[i] = time_now
                        self.upperLimbControlFlag = True
        if self.upperLimbControlFlag == True:
            return msg
        else:
            return None

    def wristController(self):
        deviceState = self.serviceThread.hapticupdater.deviceState
        if len(deviceState)==0:
            print "No device state"
            return None
        msg = {}
        pose = self.ArmPosition

        self.wristControlFlag = False
        
        for i, dstate in enumerate(deviceState):
            if dstate['mode'] != 'wrist':
                continue
            if dstate['buttonDown']:
                time_now = time.time()
                if dstate['newupdate'] == True:
                    if time_now - self.wristTimer[i] > 0.02:
                        wristVel = dstate['linearVel'][2]
                        print "wristVel = ", wristVel
                        if abs(wristVel) > 0.5:
                            pose[i][6] = wristVel*self.jointControlRatio + self.ArmPosition[i][6]

                        msg['type'] = "JointPose"
                        msg['parts'] = ['left', 'right']
                        msg['positions'] = pose
                        msg['speed'] = 1
                        msg['safe'] = int(CollisionDetectionEnabled)
                        self.wristTimer[i] = time_now
                        self.wristControlFlag = True
        if self.wristControlFlag == True:
            return msg
        else:
            return None


    def switchRecordPose(self):
        deviceState = self.serviceThread.hapticupdater.deviceState
        if len(deviceState)==0:
            print "No device state"
            return None
        msg = {}
        pose = self.ArmPosition

        self.recordPoseControlFlag = False
        global RecordPoseButton
        global recordArmPose

        if RecordPoseButton == True:
            for i, dstate in enumerate(deviceState):
                if dstate['mode'] == 'recordPose':
                    recordArmPose['record'][i] = self.ArmPosition[i]
                    print "recordArmPose = ", recordArmPose 
                RecordPoseButton = False

        for i, dstate in enumerate(deviceState):
            if dstate['mode'] != 'recordPose':
                self.recordPoseStatus[i] = "record"
                continue
            if dstate['buttonDown']:
                time_now = time.time()
                if dstate['newupdate'] == True:
                    if time_now - self.recordPoseTimer[i] > 0.5:
                        if self.recordPoseStatus[i] == "front":
                            print "switch to recorded pose"
                            self.recordPoseStatus[i] = "record"
                            pose[i] = recordArmPose['record'][i]
                        elif self.recordPoseStatus[i] == "record" or self.recordPoseStatus[i] == None:
                            print "switch to front pose"
                            self.recordPoseStatus[i] = "front"
                            pose[i] = recordArmPose['front'][i]
                        msg['type'] = "JointPose"
                        msg['parts'] = ['left', 'right']
                        msg['positions'] = pose
                        msg['speed'] = 1
                        msg['safe'] = int(CollisionDetectionEnabled)
                        self.recordPoseTimer[i] = time_now
                        self.recordPoseControlFlag = True
        if self.recordPoseControlFlag == True:
            return msg
        else:
            return None

    def syncPoseController(self):
        deviceState = self.serviceThread.hapticupdater.deviceState
        if len(deviceState)==0:
            print "No device state"
            return None
        msg = {}
        pose = self.ArmPosition
        self.syncControlFlag = False
        global armPoseSign

        for i, dstate in enumerate(deviceState):
            if dstate['mode'] != 'sync':
                continue
            if dstate['buttonDown']:
                time_now = time.time()
                if dstate['newupdate'] == True:
                    if time_now - self.syncControlTimer[i] > 0.5:
                        if i == 0:
                            pose[1] = [a*b for a,b in zip(pose[0],armPoseSign)]
                            print "Sync with right arm"
                        else:
                            pose[0] = [a*b for a,b in zip(pose[1],armPoseSign)]
                            print "Sync with left arm"

                        msg['type'] = "JointPose"
                        msg['parts'] = ['left', 'right']
                        msg['positions'] = pose
                        msg['speed'] = 1
                        msg['safe'] = int(CollisionDetectionEnabled)
                        self.syncControlTimer[i] = time_now
                        self.syncControlFlag = True
        if self.syncControlFlag == True:
            return msg
        else:
            return None

    def handCouplingController(self):
        deviceState = self.serviceThread.hapticupdater.deviceState
        if len(deviceState)==0:
            print "No device state"
            return None
        msg = {}
        p = []
        gripperVel = 0.0

        for i in range(0,2):
            if self.gripperPosition[i]:
                p = p + self.gripperPosition[i]
            else:
                p = p + [1.0, 1.0, 1.0, 1.0]

        self.handCouplingControlFlag = False
        leading_idx = -1

        for i, dstate in enumerate(deviceState):
            if dstate['mode'] != 'handCoupling':
                continue
            if leading_idx == -1:
                leading_idx = i
            if dstate['buttonDown']:
                time_now = time.time()
                if dstate['newupdate'] == True:
                    print "new update"
                    if time_now - self.handCouplingControlTimer[i] > 0.0:
                        if i == 1:
                            gripperVel = -dstate['linearVel'][1]
                        else:
                            gripperVel = dstate['linearVel'][1]
                        

                        self.handCouplingControlTimer[i] = time_now
                        self.handCouplingControlFlag = True
                        self.baseOrtControlFlag = False
            if leading_idx == 0:
                self.gripperPosition[1] = self.gripperPosition[0]
            elif leading_idx == 1:
                self.gripperPosition[0] = self.gripperPosition[1]   
                                 
        if self.handCouplingControlFlag == True:
            for i in range(0,2):
                self.gripperPosition[i] =  [v + gripperVel*self.gripperControlRatio for v in self.gripperPosition[i][0:3]] + [self.gripperPosition[i][3]]
                for j in range(0,3):
                    if self.gripperPosition[i][j] > 1.0:
                        self.gripperPosition[i][j] = 1.0
                    elif self.gripperPosition[i][j] < 0.2:
                        self.gripperPosition[i][j] = 0.2

                p[0+i*4:4+i*4] = self.gripperPosition[i]
            
            msg['limb'] = "both"
            msg['position'] = p
            msg['type'] = 'Gripper'
            msg['force'] = 0.2
            msg['speed'] = 0.5

            return msg
        else:
            return None

    def getDesiredCartesianPose(self,limb,device):
        """Returns a pair (R,t) of the desired EE pose if the limb should have
        a cartesian pose message, or None if it should not.

        Implementation-wise, this reads from self.startTransforms and the deviceState
        dictionary to determine the correct desired end effector transform.  The delta
        from devices[device]['deviceCurrentTransform'] to devices[device]['deviceInitialTransform']
        is scaled, then offset by self.startTransforms[device].  (self.startTransforms is
        the end effector transform when deviceInitialTransform is set)
        """
        if limb=='left':
            T = self.serviceThread.eeGetter_left.get()
        else:
            T = self.serviceThread.eeGetter_right.get()
        if T is  None:
            T = se3.identity()
        R,t=T
        deviceState = self.serviceThread.hapticupdater.deviceState
        if deviceState == None: return T
        if self.startTransforms[device] == None: return T
        dstate = deviceState[device]
        Tcur = dstate['deviceCurrentTransform']
        T0 = dstate['deviceInitialTransform']
        if T0 == None:
            T0 = Tcur
        #print "Button is down and mode is",dstate['mode']
        #print dstate
        assert T0 != None,"T0 is null"
        assert Tcur != None,"Tcur is null"
        if dstate['mode'] in ['normal','armCoupling','armShift', 'onlyPosition']: 
            relRot = so3.mul(Tcur[0],so3.inv(T0[0]))
            relTrans = vectorops.sub(Tcur[1],T0[1])
            #print "Rotation moment",so3.moment(relRot)
            translationScaling = 3.0
            if dstate['mode'] == 'onlyPosition':
                relRot = so3.from_moment([0.0, 0.0, 0.0])
            desRot = so3.mul(relRot,self.startTransforms[device][0])
            desPos = vectorops.add(vectorops.mul(relTrans,translationScaling),self.startTransforms[device][1])
            #TEST: just use start rotation
            #desRot = self.startTransforms[i][0]
            return (desRot,desPos)

        else:
            # print "how to render mode",dstate['mode'],"?"
            return T

    def armCouplingController(self):
        deviceState = self.serviceThread.hapticupdater.deviceState
        if len(deviceState)==0:
            print "No device state"
            return None

        self.armCouplingControlFlag = False
        leading_idx = -1

        T1 = self.serviceThread.eeGetter_left.get()
        if T1 is  None:
            T1 = se3.identity()
        T2=self.serviceThread.eeGetter_right.get()
        if T2 is  None:
            T2 = se3.identity()
        
        transforms = [T1, T2]

        update = False
        for i,dstate in enumerate(deviceState):
            if dstate['mode'] != 'armCoupling': continue
            if leading_idx == -1:
                leading_idx = i
                if dstate['buttonDown']:
                    if self.startTransforms[i] == None:
                        self.startTransforms[i] = transforms[i]
                        print "Start transform",transforms[i]
                    if dstate['newupdate']:
                        update = True
                        dstate['newupdate'] = False
                else:
                    self.startTransforms[i] = None
        
        if update and leading_idx != -1:
            i = leading_idx
            j = 1 - leading_idx

            msg = {}
            msg['type'] = 'CartesianPose'
            msg['limb'] = 'both'
            T1=self.getDesiredCartesianPose('left',0)
            R1,t1=T1
            T2=self.getDesiredCartesianPose('right',1)
            R2,t2=T2
            msg['position'] = t1+t2
            msg['rotation'] = R1+R2
            msg['maxJointDeviation']=0.5
            msg['safe'] = int(CollisionDetectionEnabled)
            leadPos = msg['position'][0+i*3:3+i*3]
            leadRot = msg['rotation'][0+i*9:9+i*9]

            coupledPos = [a*b for a,b in zip(leadPos,endEffectorSign)]

            leadMoment = list(so3.moment(leadRot))
            coupledMoment = [-1.0*a*b for a,b in zip(leadMoment,endEffectorSign)]
            coupledRot = so3.from_moment(tuple(coupledMoment))
            # coupledRot = leadRot
            msg['position'][0+j*3:3+j*3] = coupledPos
            msg['rotation'][0+j*9:9+j*9] = coupledRot

            # print "========="
            # print "leadPos = ", leadPos
            # print "coupledPos = ", coupledPos
            
            # print "leadMoment = ", leadMoment
            # print "coupledMoment = ", coupledMoment

            self.armCouplingControlFlag = True
            return msg    
        else:
            return None

    def armShiftController(self):
        deviceState = self.serviceThread.hapticupdater.deviceState
        if len(deviceState)==0:
            print "No device state"
            return None
        msg = {}

        msg['type'] = 'CartesianPose'
        msg['limb'] = 'both'
        T1 = self.serviceThread.eeGetter_left.get()
        if T1 is  None:
            T1 = se3.identity()
        R1,t1=T1
        T2=self.serviceThread.eeGetter_right.get()
        if T2 is  None:
            T2 = se3.identity()
        R2,t2=T2
        transforms = [(R1,t1),(R2,t2)]
        msg['position'] = t1+t2
        msg['rotation'] = R1+R2
        msg['maxJointDeviation']=0.5
        msg['safe'] = int(CollisionDetectionEnabled)

        dist = [0.0, 0.0, 0.0]

        self.armShiftControlFlag = False
        leading_idx = -1


        update = False
        for i,dstate in enumerate(deviceState):
            if dstate['mode'] != 'armShift': 
                dist = [a-b for a,b in zip(t1, t2)]
                continue
            if leading_idx == -1:
                leading_idx = i
                if dstate['buttonDown']:
                    if self.startTransforms[i] == None:
                        self.startTransforms[i] = transforms[i]
                        print "Start transform",transforms[i]
                    if dstate['newupdate']:
                        update = True
                        dstate['newupdate'] = False
                    Tcur = dstate['deviceCurrentTransform']
                    T0 = dstate['deviceInitialTransform']
                    if T0 == None:
                        T0 = Tcur
                    print "Button is down and mode is",dstate['mode']
                    print dstate
                    assert T0 != None,"T0 is null"
                    assert Tcur != None,"Tcur is null"
                    relRot = so3.mul(Tcur[0],so3.inv(T0[0]))
                    relTrans = vectorops.sub(Tcur[1],T0[1])
                    print "Rotation moment",so3.moment(relRot)
                    oldMoment = so3.moment(relRot)
                    newMoment = (0.0,0.0, oldMoment[2])
                    newrelRot = so3.from_moment(newMoment)
                    desRot = so3.mul(newrelRot,self.startTransforms[i][0])
                    desPos = vectorops.add(relTrans,self.startTransforms[i][1])
                    #TEST: just use start rotation
                    #desRot = self.startTransforms[i][0]
                    msg['position'][0+i*3:3+i*3] = desPos
                    msg['rotation'][0+i*9:9+i*9] = desRot

                else:
                    self.startTransforms[i] = None
        
        if leading_idx != -1:
            i = leading_idx
            j = 1 - leading_idx

            

            leadPos = msg['position'][0+i*3:3+i*3]
            leadRot = msg['rotation'][0+i*9:9+i*9]
            if i == 0:
                coupledPos = [a-b for a,b in zip(leadPos,dist)]
            else:
                coupledPos = [a+b for a,b in zip(leadPos,dist)]
            

            leadMoment = list(so3.moment(leadRot))
            coupledMoment = [-1.0*a*b for a,b in zip(leadMoment,endEffectorSign)]
            coupledRot = so3.from_moment(tuple(coupledMoment))
            # coupledRot = leadRot
            msg['position'][0+j*3:3+j*3] = coupledPos
            msg['rotation'][0+j*9:9+j*9] = coupledRot


        if update:
            self.armShiftControlFlag = True
            return msg    
        else:
            return None


    def alignController(self):
        deviceState = self.serviceThread.hapticupdater.deviceState
        if len(deviceState)==0:
            print "No device state"
            return None
        msg = {}

        self.alignControlFlag = False            
        
        msg['type'] = 'CartesianPose'
        msg['limb'] = 'both'
        T1 = self.serviceThread.eeGetter_left.get()
        if T1 is  None:
            T1 = se3.identity()
        R1,t1=T1
        T2=self.serviceThread.eeGetter_right.get()
        if T2 is  None:
            T2 = se3.identity()
        R2,t2=T2
        transforms = [(R1,t1),(R2,t2)]
        msg['position'] = t1+t2
        msg['rotation'] = R1+R2
        msg['maxJointDeviation']=0.5
        msg['safe'] = int(CollisionDetectionEnabled)

   
        for i,dstate in enumerate(deviceState):
            if dstate['mode'] not in ['horizontalalign', 'verticalalign', 'frontalign', 'tablealign']: continue
            if dstate['buttonDown']:
                if self.startTransforms[i] == None:
                    self.startTransforms[i] = transforms[i]
                    print "Start transform",transforms[i]
                if dstate['newupdate']:
                    self.alignControlFlag = True
                    dstate['newupdate'] = False
                Tcur = dstate['deviceCurrentTransform']
                T0 = dstate['deviceInitialTransform']
                if T0 == None:
                    T0 = Tcur
                print "Button is down and mode is",dstate['mode']
                print dstate
                assert T0 != None,"T0 is null"
                assert Tcur != None,"Tcur is null"
                relRot = so3.mul(Tcur[0],so3.inv(T0[0]))
                relTrans = vectorops.sub(Tcur[1],T0[1])
                print "Rotation moment",so3.moment(relRot)
                desRot = so3.mul(relRot,self.startTransforms[i][0])
                desPos = vectorops.add(relTrans,self.startTransforms[i][1])

                if dstate['mode'] == 'horizontalalign':
                    zeroRot = [0.0, -1.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0]
                    rotationAboutY = so3.mul(desRot,so3.inv(zeroRot))
                    oldMoment = so3.moment(rotationAboutY)
                    theta_y = oldMoment[1]

                    # rotation matrix is in column matrix 
                    desRot = [0.0, -1.0, 0.0, -math.sin(theta_y), 0.0, -math.cos(theta_y), math.cos(theta_y), 0.0, -math.sin(theta_y)];
                    #this should be equal to desRot
                    #desRot2 = so3.mul(so3.rotation([0,1,0],theta_y),zeroRot)

                elif dstate['mode'] == 'verticalalign':
                    zeroRot = [0.0, 0.0, -1.0, 0, 1, 0.0, 1, 0, 0.0];
                    rotationAboutZ = so3.mul(desRot,so3.inv(zeroRot))
                    oldMoment = so3.moment(rotationAboutZ)
                    theta_z = oldMoment[2]

                    # rotation matrix is in column matrix 
                    desRot = [0.0, 0.0, -1.0, -math.sin(theta_z), math.cos(theta_z), 0.0, math.cos(theta_z), math.sin(theta_z), 0.0];
                    #this should be equal to desRot
                    #desRot2 = so3.mul(so3.rotation([0,0,1],theta_z),zeroRot)
                elif dstate['mode'] == 'frontalign':
                    # zeroRot: math.cos(theta_x) = 1, math.sin(theta_x) = 0
                    zeroRot = [0.0, 0.0, -1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0];
                    rotationAboutX = so3.mul(desRot,so3.inv(zeroRot))
                    oldMoment = so3.moment(rotationAboutX)
                    theta_x = oldMoment[0]

                    print "theta_x: ==== ", theta_x

                    # rotation matrix is in column matrix 
                    desRot = [0.0, math.sin(theta_x), -math.cos(theta_x), 0.0, math.cos(theta_x), math.sin(theta_x), 1.0, 0.0, 0.0];
                    #this should be equal to desRot
                    #desRot2 = so3.mul(so3.rotation([0,0,1],theta_z),zeroRot)
                elif dstate['mode'] == 'tablealign':
                    zeroRot = [1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0];
                    rotationAboutZ = so3.mul(desRot,so3.inv(zeroRot))
                    oldMoment = so3.moment(rotationAboutZ)
                    theta_z = oldMoment[2]

                    # rotation matrix is in column matrix 
                    desRot = [math.cos(theta_z), math.sin(theta_z), 0.0, math.sin(theta_z), -math.cos(theta_z), 0.0, 0.0, 0.0, -1.0];
                    #this should be equal to desRot
                    #desRot2 = so3.mul(so3.rotation([0,0,1],theta_z),zeroRot)

                else: 
                    pass



                msg['position'][0+i*3:3+i*3] = desPos
                msg['rotation'][0+i*9:9+i*9] = desRot

            else:
                self.startTransforms[i] = None
        
        if self.alignControlFlag == True:
            return msg    
        else:
            return None


    def BasePositionControl(self):
        deviceState = self.serviceThread.hapticupdater.deviceState
        if len(deviceState)==0:
            print "No device state"
            return None
        msg = {}
        baseVel = [0.0, 0.0, 0.0]
        self.basePosControlFlag = False
        self.baseDevice = False

        for i, dstate in enumerate(deviceState):
            if dstate['mode'] != 'basePosition': continue
            if self.baseDevice == False:
                if dstate['buttonDown']:
                    #print "SumVelocity",dstate['sumVelocity']
                    if dstate['newupdate'] == True:
                        if 'sumVelocity' not in dstate:
                            dstate['sumVelocity'] = [0,0,0]
                            dstate['countVelocity'] = 0
                        dstate['sumVelocity'] = vectorops.add(dstate['sumVelocity'],dstate['linearVel'])
                        dstate['countVelocity'] += 1
                        time_now = time.time()
                        if time_now - self.basePosCommandTimer[i] > 0.1:
                            averageVelocity = vectorops.div(dstate['sumVelocity'],dstate['countVelocity'])
                            #print "Average velocity",averageVelocity
                            for i in range(0,2):
                                if averageVelocity[i] > 0.01 or averageVelocity[i] < -0.01:
                                    baseVel[i] = -averageVelocity[i]*self.basePosVelocityRatio
                                else:
                                    baseVel[i] = 0.0
                            msg['type'] = 'BaseVelocity'
                            msg['velocity'] = baseVelocityViewToWorld(baseVel)
                            # msg['velocity'] = [v * self.baseVelocityRatio for v in baseVel]
                            self.basePosControlFlag = True
                            self.gripperSizeFlag = False
                            self.baseDevice = True
                            self.basePosCommandTimer[i] = time.time()
                            dstate['sumVelocity'] = [0,0,0]
                            dstate['countVelocity'] = 0
                else: 
                    self.basePosControlFlag = True
                    msg['type'] = 'BaseVelocity'
                    msg['velocity'] = [0,0,0]

        if self.basePosControlFlag == True:
            return msg
        else:
            return None

    def BaseOrientationControl(self):
        deviceState = self.serviceThread.hapticupdater.deviceState
        if len(deviceState)==0:
            print "No device state"
            return None
        msg = {}
        baseVel = [0.0, 0.0, 0.0]
        self.baseOrtControlFlag = False

        for i, dstate in enumerate(deviceState):
            if dstate['mode'] != 'baseOrientation': continue
            # if i == 0:
            if dstate['buttonDown']:
                if dstate['newupdate'] == True:
                    if 'sumVelocity' not in dstate:
                        dstate['sumVelocity'] = [0,0,0]
                        dstate['countVelocity'] = 0
                    dstate['sumVelocity'] = vectorops.add(dstate['sumVelocity'],dstate['linearVel'])
                    dstate['countVelocity'] += 1
                    time_now = time.time()
                    if time_now - self.baseOrtCommandTimer[i] > 0.1:
                        averageVelocity = vectorops.div(dstate['sumVelocity'],dstate['countVelocity'])
                        if averageVelocity[1] > 0.01:
                            baseVel[2] = averageVelocity[1]
                        elif averageVelocity[1] < -0.01:
                            baseVel[2] = averageVelocity[1]
                            # baseVel[2] = dstate['linearVel'][1]*self.baseOrtVelocityRatio
                        else:
                            baseVel[2] = 0.0
                        msg['type'] = 'BaseVelocity'
                        msg['velocity'] = baseVel
                        # msg['velocity'] = [v * self.baseVelocityRatio for v in baseVel]
                        self.baseOrtControlFlag = True
                        self.basePosControlFlag = False
                        self.gripperSizeFlag = False
                        self.baseOrtCommandTimer[i] = time.time()
                        dstate['sumVelocity'] = [0,0,0]
                        dstate['countVelocity'] = 0
                else: 
                    self.baseOrtControlFlag = True
                    msg['type'] = 'BaseVelocity'
                    msg['velocity'] = [0,0,0]
        if self.baseOrtControlFlag == True:
            return msg
        else:
            return None


    def renderCartesianVelocityMsg(self):
        """DEPRECATED"""
        deviceState = self.serviceThread.hapticupdater.deviceState
        if len(deviceState)==0: 
            print "No device state"
            return None
        msg = {}
        msg['type'] = 'CartesianVelocity'
        msg['limb'] = 'both'
        msg['linear'] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg['angular'] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg['safe'] = 0

        for i,dstate in enumerate(deviceState):
            if dstate['buttonDown']:
                if dstate['newupdate']:
                    print "Device",i,"linear",dstate['linearVel'],"angular",dstate['angularVel']
                    msg['linear'][0+i*3:3+i*3] = dstate['linearVel']
                    msg['angular'][0+i*3:3+i*3] = dstate['angularVel']
                    dstate['newupdate'] = False
        return msg



    def renderCartesianPoseMsg(self):
        deviceState = self.serviceThread.hapticupdater.deviceState
        if len(deviceState)==0: 
            print "No device state"
            return None
        transforms = [self.serviceThread.eeGetter_left.get(),self.serviceThread.eeGetter_right.get()]
        update = False
        for i,dstate in enumerate(deviceState):
            if dstate['mode'] not in ['normal', 'onlyPosition'] : continue
            if dstate['buttonDown']:
                if self.startTransforms[i] == None:
                    self.startTransforms[i] = transforms[i]
                    #RESET THE HAPTIC FORCE FEEDBACK CENTER
                    self.serviceThread.hapticupdater.activateSpring(kP=20,kD=0,center=None)
                    #print "Start transform",transforms[i]
                if dstate['newupdate']:
                    update = True
                    dstate['newupdate'] = False
            else:
                self.startTransforms[i] = None
        if update:
            msg = {}
            msg['type'] = 'CartesianPose'
            msg['limb'] = 'both'
            T1 = self.getDesiredCartesianPose('left',0)
            R1,t1=T1
            T2 = self.getDesiredCartesianPose('right',1)
            R2,t2=T2
            transforms = [(R1,t1),(R2,t2)]
            msg['position'] = t1+t2
            msg['rotation'] = R1+R2
            msg['maxJointDeviation']=0.5
            msg['safe'] = int(CollisionDetectionEnabled)
            self.CartesianPoseControl = True
            return msg    
        else:
            return None


    def getRobotStatus(self):
        T1 = self.serviceThread.eeGetter_left.get()
        if T1 is  None:
            T1 = se3.identity()
        R1,t1=T1
        T2 = self.serviceThread.eeGetter_right.get()
        if T2 is  None:
            T2 = se3.identity()
        R2,t2=T2
        gripperStatus_left = self.serviceThread.gripperGetter_left.get()
        gripperStatus_right = self.serviceThread.gripperGetter_right.get()

        self.baseSensedVelocity = self.serviceThread.baseVelocitySensed.get()
        self.baseCommandVelocity = self.serviceThread.baseVelocityCommand.get()

        self.ArmPosition[0] = self.serviceThread.ArmPosition_left.get()
        self.ArmPosition[1] = self.serviceThread.ArmPosition_right.get()

        self.robotEndEffectorPosition[0] = t1
        self.robotEndEffectorPosition[1] = t2

        self.robotEndEffectorTransform[0] = T2
        self.robotEndEffectorTransform[1] = T2

        self.gripperPosition[0] = gripperStatus_left
        self.gripperPosition[1] = gripperStatus_right


        for i in range(0,2):
            if self.gripperPosition[i]:
                if all(v > 0.9 for v in self.gripperPosition[i]):
                    self.gripperStatus[i] = "open"
                elif all(v < 0.3 for v in self.gripperPosition[i]):
                    self.gripperStatus[i] = "close"

    def do_logic(self):

        if HapticModeButton == 'arm':
            msg = self.renderCartesianPoseMsg()
            if self.CartesianPoseControl:
                self.CartesianPoseControl = False
                return msg

            msg = self.gripperController()
            if self.gripperControlFlag:
                self.gripperControlFlag = False
                return msg

            msg = self.gripperIncrementalControl()
            if self.gripperSizeFlag:
                self.gripperSizeFlag = False
                return msg

            msg = self.autoTracking()
            if self.autoTrackingControlFlag:
                self.autoTrackingControlFlag = False
                return msg

            msg = self.switchPoseController()
            if self.poseControlFlag:
                self.poseControlFlag = False
                return msg

        elif HapticModeButton == 'base':
            msg = self.BasePositionControl()
            if self.basePosControlFlag:
                self.basePosControlFlag = False
                return msg

            msg = self.BaseOrientationControl()
            if self.baseOrtControlFlag:
                self.baseOrtControlFlag = False
                return msg

        elif HapticModeButton == 'auto':

            msg = self.switchPoseController()
            if self.poseControlFlag:
                self.poseControlFlag = False
                return msg

            msg = self.switchRecordPose()
            if self.recordPoseControlFlag:
                self.recordPoseControlFlag = False
                return msg        

            msg = self.syncPoseController()
            if self.syncControlFlag:
                self.syncControlFlag = False
                return msg

            msg = self.handCouplingController()
            if self.handCouplingControlFlag:
                self.handCouplingControlFlag = False
                return msg

            msg = self.armCouplingController()
            if self.armCouplingControlFlag:
                self.armCouplingControlFlag = False
                return msg

            msg = self.armShiftController()
            if self.armShiftControlFlag:
                self.armShiftControlFlag = False
                return msg
            
        elif HapticModeButton == 'fix':

            msg = self.renderCartesianPoseMsg()
            if self.CartesianPoseControl:
                self.CartesianPoseControl = False
                return msg

            msg = self.switchPoseController()
            if self.poseControlFlag:
                self.poseControlFlag = False
                return msg

            msg = self.alignController()
            if self.alignControlFlag:
                self.alignControlFlag = False
                return msg

        elif HapticModeButton == 'joint':
            
            msg = self.upperLimbController()
            if self.upperLimbControlFlag:
                self.upperLimbControlFlag = False
                return msg

            msg = self.lowerLimbController()
            if self.lowerLimbControlFlag:
                self.lowerLimbControlFlag = False
                return msg

            msg = self.wristController()
            if self.wristControlFlag:
                self.wristControlFlag = False
                return msg

        else: 
            msg = {}
            

        return msg


    def glPlugin(self):
        return self.plugin

def make():
    return HapticTaskGenerator()

