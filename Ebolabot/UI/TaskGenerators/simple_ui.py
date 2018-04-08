"""
Controls Baxter using any game pad / joystick via the logitech and Motion modules.
Designed for Logitech controllers, may work for other controllers too.
Converted by Kris Hauser Oct 2015 from a script written by Peter Moran,
July 2015.

SETUP:
Before running this script, you must run the system state service and the
controller dispatcher.

CONTROLS:
    Left stick x -- move left/right
    Left stick y -- move forward
    Right stick x -- turn
    A -- tuck
    B -- neutral
    X -- original
"""

# Import Modules
import os
import sys
import time
import threading
import asyncore

import copy
import math

ebolabot_root = os.getenv("EBOLABOT_PATH",".")
sys.path.append(ebolabot_root)
from Common.system_config import EbolabotSystemConfig

#for logitech module
sys.path.append(os.path.join(ebolabot_root,'InputDevices/GamePad'))
import logitech
from task_generator import TaskGenerator
from klampt.io import resource

from OpenGL.GL import *
from klampt.vis import gldraw
from klampt.vis.glinterface import GLPluginInterface as GLPluginBase
from klampt.vis.glcommon import GLWidgetPlugin

from klampt.math import so3,se3,vectorops
from sspp.service import Service
from sspp.topic import MultiTopicListener
from UI.utils.gltexture import *
from UI.utils.haptic_client import *

#Configuration variable: where's the state server?
system_state_addr = EbolabotSystemConfig.getdefault_ip('state_server_computer_ip',('localhost',4568))

#Configuration variable: where's the haptic service?
try:
    #default 'tcp://192.168.1.128:3456' ?
    haptic_service_addr = EbolabotSystemConfig.get_ip('haptic_server_computer_ip')
except Exception:
    print "Haptic device server not configured. Please configure the appropriate variable in",EbolabotSystemConfig.system_config_fn
    exit(1)

debugHapticTransform = False

#set this -1 for view-centric control, looking at the face of the robot
viewToWorldScaleXY = 1

gamepadBaseTranslationSpeedScale = 0.2
gamepadBaseRotationSpeedScale = 0.2

CollisionDetectionEnabled = True

hapticArmTranslationScaling = 3.0
hapticGripperSpringStrength = 20
hapticArmSpringStrength = 20

#Configuration variable: where's the state server?
system_state_addr = EbolabotSystemConfig.getdefault_ip('state_server_computer_ip',('localhost',4568))

#Configuration variable: where's the haptic service?
try:
    #default 'tcp://192.168.1.128:3456' ?
    haptic_service_addr = EbolabotSystemConfig.get_ip('haptic_server_computer_ip')
except Exception:
    print "Haptic device server not configured. Please configure the appropriate variable in",EbolabotSystemConfig.system_config_fn
    exit(1)



class ServiceThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self._kill = False
        self.stateUpdateFreq = 50
        # self.stateUpdateFreq=20
        self.hapticupdater = HapticClient(haptic_service_addr)

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



class MyWidgetPlugin(GLPluginBase):
    def __init__(self,taskGen):
        GLPluginBase.__init__(self)
        self.taskGen = taskGen

    def initialize(self):
        GLPluginBase.initialize(self)
        return True

    def keyboardfunc(self,c,x,y):
        global CollisionDetectionEnabled

        if c=='c':
            CollisionDetectionEnabled = not CollisionDetectionEnabled
            print "Collision detection toggled to:",CollisionDetectionEnabled

    def draw_wand(self,xform,size = 0.2):
        #draw the wand pointing along the -x axis (+x toward the user)
        gldraw.setcolor(1,0.8,0.8)
        mat = zip(*se3.homogeneous(xform))
        mat = sum([list(coli) for coli in mat],[])
        glPushMatrix()
        glMultMatrixf(mat)
        gldraw.box([-size,-size*0.05,-size*0.05],[size*0.5,size*0.05,size*0.05])
        glPopMatrix()

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
        if T1 is not None and dstate[0] != None:
            R = dstate[0].widgetCurrentTransform[0]
            T = se3.mul(baseTransform,(R,T1[1]))
            self.draw_wand(T)
        if T2 is not None and dstate[1] != None:
            R = dstate[1].widgetCurrentTransform[0]
            T = se3.mul(baseTransform,(R,T2[1]))
            self.draw_wand(T)

        if debugHapticTransform:
            for deviceState in dstate:
                #debugging
                #this is the mapping from the handle to the user frame
                Traw = (so3.from_moment(deviceState.rotationMoment),deviceState.position)
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
        global CollisionDetectionEnabled
        """
        glRasterPos(20,30)
        glColor3f(1,1,1)
        glDisable(GL_LIGHTING)
        """
        #TODO: render status of the task generator

    def eventfunc(self,type,args):
        """TODO: connect this up to GUI buttons"""
        pass

class SimpleUITaskGenerator(TaskGenerator):
    def __init__(self):
        self.j = None

    def name(self): return "SimpleUI"

    def init(self,world):
        assert self.j == None,"Init may only be called once"
        self.world = world
        left_arm_link_names = ['left_upper_shoulder','left_lower_shoulder','left_upper_elbow','left_lower_elbow','left_upper_forearm','left_lower_forearm','left_wrist']
        right_arm_link_names = ['right_upper_shoulder','right_lower_shoulder','right_upper_elbow','right_lower_elbow','right_upper_forearm','right_lower_forearm','right_wrist']
        robot = world.robot(0)
        self.left_arm_link_indices = [robot.link(l).index for l in left_arm_link_names]
        self.right_arm_link_indices = [robot.link(l).index for l in right_arm_link_names]
        self.endEffectorIndices = [25,45]
        self.plugin = None
        self.activeDevice = None
        if not self.init_gamepad():
            return False
        if not self.init_haptic():
            return False
        return True

    def init_gamepad(self):
        """Initializes the gamepad"""
        self.lastGamepadState = {}
        self.originalConfig = None
        self.tuckConfig = resource.get("tuck.config",directory="UI/Resources",doedit=False)
        self.neutralConfig = resource.get("neutral.config",directory="UI/Resources",doedit=False)
        # Connect to controller
        try:
            self.j = logitech.Joystick(0)
        except:
            print "Joystick not found"
            print "Note: Pygame reports there are " + str(logitech.numJoys()) + " joysticks"
            return False
        return True

    def start_gamepad(self):
        self.update_gamepad_start_config()
        return

    def stop_gamepad(self):
        return

    def update_gamepad(self):
        """Returns a command message from the gamepad, or None if no message should be sent."""
        j = self.j
        j.updateState()
        rstick = j.get_stick_R()
        lstick = j.get_stick_L()
        #LB = j.get_LB()
        #RB = j.get_RB()
        #LT = j.get_LT()
        #RT = j.get_RT()

        if j.get_A() and self.tuckConfig is not None:
            return self.makeArmsTask(self.tuckConfig)
        if j.get_B() and self.neutralConfig is not None:
            return self.makeArmsTask(self.neutralConfig)
        if j.get_X() and self.originalConfig is not None:
            return self.makeArmsTask(self.originalConfig)       

        state = {}
        state['rstick'] = rstick
        state['lstick'] = lstick
        #state['LB'] = LB
        #state['RB'] = RB
        #state['LT'] = LT
        #state['RT'] = RT
        deadband = 0.02
        self.lastGamepadState = state

        if len(state) > 0:        
            rstick = state['rstick']
            lstick = state['lstick']
            vel = [-viewToWorldScaleXY*float(lstick[1])*gamepadBaseTranslationSpeedScale,-viewToWorldScaleXY*float(lstick[0])*gamepadBaseTranslationSpeedScale,-float(rstick[0])*gamepadBaseRotationSpeedScale]
            if any(abs(v) > deadband for v in vel):
                msg = {'type':'BaseVelocity'}
                msg['velocity'] = vel 
                return msg
            elif self.activeDevice == 'gamepad':
                msg = {'type':'BaseVelocity'}
                msg['velocity'] = [0,0,0]
                return msg
        return None

    def update_gamepad_start_config(self):
        q = self.world.robot(0).getConfig()
        if self.originalConfig == None:
            tol = 1e-2
            if all(abs(q[i]-self.tuckConfig[i])<tol for i in self.left_arm_link_indices+self.right_arm_link_indices):
                pass
            elif all(abs(q[i]-self.neutralConfig[i])<tol for i in self.left_arm_link_indices+self.right_arm_link_indices):
                pass
            else:
                #remember original configuration
                self.originalConfig = q

    def init_haptic(self):
        self.serviceThread = None
        self.startTransforms = [None,None]

        self.gripperControlRatio = 1
        self.startGripperPosition = [None,None]
        self.hapticTrackGripperPosition = [None,None]
        self.hapticFeedbackMode = [None,None]
        self.startGripperConfigs = [[0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]]
        self.gripperValues = [None,None]
        self.gripperShapes = [None,None]
        self.gripperChangeMode = [False,False]
        self.gripperHapticFeedbackStatus = [None,None]
        return True

    def start_haptic(self):
        if self.serviceThread==None:
            self.serviceThread = ServiceThread()
            self.serviceThread.start()
            self.haptic_client = self.serviceThread.hapticupdater
        attached = False
        for i in range(5):
            time.sleep(0.1)
            if len(self.haptic_client.deviceState)!=0:
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
            #print "Turning on drag..."
            #2 is probably the highest damping that doesnt cause vibration
            #self.serviceThread.hapticupdater.activateDrag(2)
            #self.haptic_client.activateSpring(kP=20,kD=0,center=None)

    def update_haptic(self):
        msg = self.gripperIncrementalControl()
        if msg is None:
            msg = self.makeHapticCartesianPoseMsg()
            if msg: 
                self.update_gamepad_start_config()
        #finish
        for dstate in self.haptic_client.deviceState:
            dstate.mark_as_read()
        return msg

    def gripperConfigToShape(self,config):
        if config[3] == 1.0:
            u = sum(config[0:3])/3
            return ('power',(u-0.1)/0.9)
        elif config[3] == 0.0:
            u = (config[0]+config[1])/2
            return ('pinch',(u-0.3)/0.7)
        else:
            u = sum(config[0:3])/3
            return ('claw',(u-0.3)/0.7)

    def shapeToGripperConfig(self,shape,value):
        if shape=='power':
            c = 0.1+value*0.9
            return [c,c,c,1]
        elif shape=='pinch':
            c = 0.3+value*0.7
            return [c,c,1,0]
        elif shape == 'claw':
            c = 0.3+value*0.7
            return [c,c,c,0.4]

    def gripperIncrementalControl(self):
        deviceState = self.haptic_client.deviceState
        if len(deviceState)==0:
            print "No device state"
            return None
        sendGripperPosition = False
        gripperConfigs = [self.serviceThread.gripperGetter_left.get(),self.serviceThread.gripperGetter_right.get()]
        # nextShape = {'power':'claw','claw':'pinch','pinch':'power'}
        nextShape = {'power':'claw','claw':'power'}
        prevShape = dict((v,k) for (k,v) in nextShape.items())
        
        #left-right range: 10cm
        grange = 0.1
        apertureControlDim = 0
        modeControlDim = 1
        hapticStrength = [0,hapticGripperSpringStrength,hapticGripperSpringStrength*4]
        sidewallStrength = [0,hapticGripperSpringStrength*4,hapticGripperSpringStrength*4]
        vertwallStrength = [hapticGripperSpringStrength*4,hapticGripperSpringStrength,hapticGripperSpringStrength*4]

        #look for gripper control
        for i, dstate in enumerate(deviceState):
            # i=1-index
            if not dstate.newupdate: continue
            if gripperConfigs[i] == None or len(gripperConfigs[i]) == 0:
                #not attached
                continue
            if dstate.buttonDown[1]:
                if self.startGripperConfigs[i] is None:
                    self.startGripperConfigs[i] = gripperConfigs[i]
                    self.hapticTrackGripperPosition[i] = dstate.position
                    self.startGripperPosition[i] = dstate.position[:]
                    self.gripperShapes[i],self.gripperValues[i] = self.gripperConfigToShape(gripperConfigs[i])
                    self.startGripperPosition[i][apertureControlDim] = self.startGripperPosition[i][apertureControlDim] - self.gripperValues[i]*grange

                #haptic device position relative to 0 position
                wpos = vectorops.sub(dstate.position,self.startGripperPosition[i])
                if wpos[modeControlDim] > 0.06:
                    #moved up, change shape
                    if not self.gripperChangeMode[i]:
                        self.gripperChangeMode[i] = True
                        self.gripperShapes[i] = nextShape[self.gripperShapes[i]]
                        print "Switched to shape",self.gripperShapes[i]
                        self.haptic_client.activateSpring(kP = sidewallStrength, kD=0.4,center=self.hapticTrackGripperPosition[i],device=i)
                        print "Setting spring to strength",sidewallStrength,"center",self.hapticTrackGripperPosition[i]
                        self.hapticFeedbackMode[i] = 'sidewall'
                    else:
                        pass
                elif wpos[modeControlDim] < -0.06:
                    #moved down, change shape
                    if not self.gripperChangeMode[i]:
                        self.gripperChangeMode[i] = True
                        self.gripperShapes[i] = prevShape[self.gripperShapes[i]]
                        print "Switched to shape",self.gripperShapes[i]
                        self.haptic_client.activateSpring(kP = sidewallStrength,kD=0.4,center=self.hapticTrackGripperPosition[i],device=i)
                        print "Setting spring to strength",sidewallStrength,"center",self.hapticTrackGripperPosition[i]
                        self.hapticFeedbackMode[i] = 'sidewall'
                    else:
                        pass
                else:
                    if abs(wpos[modeControlDim]) < 0.04:  #0.02 deadband
                        self.gripperChangeMode[i] = False
                        if self.hapticFeedbackMode[i] == 'sidewall':
                            self.haptic_client.activateSpring(kP = hapticStrength,kD=0.1,center=self.hapticTrackGripperPosition[i],device=i)
                            print "A Setting spring to normal strength",hapticStrength,"center",self.hapticTrackGripperPosition[i]
                            self.hapticFeedbackMode[i] = 'normal'
                
                #Clamp the range of the haptic tracker
                self.hapticTrackGripperPosition[i] = self.startGripperPosition[i][:]
                self.hapticTrackGripperPosition[i][apertureControlDim] = self.startGripperPosition[i][apertureControlDim] + max(min(wpos[apertureControlDim],grange),0)

                gripperValue = (dstate.position[apertureControlDim] - self.startGripperPosition[i][apertureControlDim])/grange
                # if i==0:
                #     gripperValue=-gripperValue

                if gripperValue <= 0 or gripperValue >= 1:
                    if self.hapticFeedbackMode[i] not in ['vertwall','sidewall']:
                        self.hapticFeedbackMode[i] = 'vertwall'
                        print "Setting spring to strength",vertwallStrength,"center",self.hapticTrackGripperPosition[i]
                        self.haptic_client.activateSpring(kP = vertwallStrength, kD = 0.4, center=self.hapticTrackGripperPosition[i], device = i)
                else:
                    if self.hapticFeedbackMode[i] in [None,'vertwall']:
                        self.hapticFeedbackMode[i] = 'normal'
                        self.haptic_client.activateSpring(kP = hapticStrength, kD = 0.1, center=self.hapticTrackGripperPosition[i], device = i)
                        print "B Setting spring to normal strength",hapticStrength,"center",self.hapticTrackGripperPosition[i]

                #clamp the range of the gripper value
                self.gripperValues[i] = min(max(gripperValue,0),1)
                gripperConfigs[i] = self.shapeToGripperConfig(self.gripperShapes[i],self.gripperValues[i])
                sendGripperPosition = True
            else:
                if self.hapticFeedbackMode[i] != None:
                    self.haptic_client.deactivateHapticFeedback(device = i)
                    self.hapticFeedbackMode[i] = None
                self.startGripperConfigs[i] = None


        if sendGripperPosition:
            msg = {}
            msg['limb'] = "both"
            msg['position'] = gripperConfigs[0]+gripperConfigs[1]
            msg['type'] = 'Gripper'
            msg['force'] = 0.3
            msg['speed'] = 10.0
            print msg['position']
            return msg
        return None

    def getDesiredCartesianPose(self,limb,device):
        """Returns a pair (R,t) of the desired EE pose if the limb should have
        a cartesian pose message, or None if it should not.

        - limb: either 'left' or 'right'
        - device: an index of the haptic device

        Implementation-wise, this reads from self.startTransforms and the deviceState
        to determine the correct desired end effector transform.  The delta
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
        deviceState = self.haptic_client.deviceState
        if deviceState == None: return T
        dstate = deviceState[device]

        if dstate.widgetInitialTransform[0] == None or self.startTransforms[device] == None: return T
        Tcur = dstate.widgetCurrentTransform
        T0 = dstate.widgetInitialTransform[0]
        if T0 == None:
            T0 = Tcur
        #print "Button is down and mode is",dstate['mode']
        #print dstate
        assert T0 != None,"T0 is null"
        assert Tcur != None,"Tcur is null"
        relRot = so3.mul(Tcur[0],so3.inv(T0[0]))
        relTrans = vectorops.sub(Tcur[1],T0[1])

        #print "Rotation moment",so3.moment(relRot)
        desRot = so3.mul(relRot,self.startTransforms[device][0])
        desPos = vectorops.add(vectorops.mul(relTrans,hapticArmTranslationScaling),self.startTransforms[device][1])
        #TEST: just use start rotation
        #desRot = self.startTransforms[i][0]
        return (desRot,desPos)


    def makeHapticCartesianPoseMsg(self):
        deviceState = self.haptic_client.deviceState
        if len(deviceState)==0: 
            print "No device state"
            return None
        transforms = [self.serviceThread.eeGetter_left.get(),self.serviceThread.eeGetter_right.get()]
        update = False
        commanding_arm=[0,0]
        for i,dstate in enumerate(deviceState):
            if dstate.buttonDown[0]:
                commanding_arm[i]=1
                if self.startTransforms[i] == None:
                    self.startTransforms[i] = transforms[i]
                    #RESET THE HAPTIC FORCE FEEDBACK CENTER
                    #self.serviceThread.hapticupdater.activateSpring(kP=hapticArmSpringStrength,kD=0,center=dstate.position,device=i)
                    
                #UPDATE THE HAPTIC FORCE FEEDBACK CENTER FROM ROBOT EE POSITION
                #need to invert world coordinates to widget coordinates to haptic device coordinates
                #widget and world translations are the same
                dx = vectorops.sub(transforms[i][1],self.startTransforms[i][1]);
                dxwidget = vectorops.div(dx,hapticArmTranslationScaling)
                R,device_center = widgetToDeviceTransform(so3.identity(),vectorops.add(dxwidget,dstate.widgetInitialTransform[0][1]))
                #print "Device position error",vectorops.sub(dstate.position,device_center)
                if vectorops.distance(dstate.position,device_center) > 0.03:
                    c = vectorops.madd(device_center,vectorops.unit(vectorops.sub(dstate.position,device_center)),0.025)
                    self.serviceThread.hapticupdater.activateSpring(kP=hapticArmSpringStrength,kD=0,center=c,device=i)
                else:
                    #deactivate
                    self.serviceThread.hapticupdater.activateSpring(kP=0,kD=0,device=i)
                if dstate.newupdate:
                    update = True
            else:
                if self.startTransforms[i] != None:
                    self.serviceThread.hapticupdater.deactivateHapticFeedback(device=i)
                self.startTransforms[i] = None
            dstate.newupdate = False
        if update:
            msg = {}
            msg['type'] = 'CartesianPose'
     
            T1 = self.getDesiredCartesianPose('left',0)
            R1,t1=T1
            T2 = self.getDesiredCartesianPose('right',1)
            R2,t2=T2
            transforms = [(R1,t1),(R2,t2)]
            if commanding_arm[0] and commanding_arm[1]:
                msg['limb'] = 'both'
                msg['position'] = t1+t2
                msg['rotation'] = R1+R2
                msg['maxJointDeviation']=0.5
                msg['safe'] = int(CollisionDetectionEnabled)
                self.CartesianPoseControl = True
                return msg
            elif commanding_arm[0] ==0:
                msg['limb'] = 'right'
                msg['position'] = t2
                msg['rotation'] = R2
                msg['maxJointDeviation']=0.5
                msg['safe'] = int(CollisionDetectionEnabled)
                self.CartesianPoseControl = True
                return msg
            else:
                msg['limb'] = 'left'
                msg['position'] = t1
                msg['rotation'] = R1
                msg['maxJointDeviation']=0.5
                msg['safe'] = int(CollisionDetectionEnabled)
                self.CartesianPoseControl = True
                return msg
        else:
            return None

    def stop_haptic(self):
        if self.serviceThread:
            self.serviceThread.kill()
            print "Waiting for thread join..."
            self.serviceThread.join()
            print "Done"
            self.serviceThread = None
        self.gripperControlFlag = False
        self.gripperSizeFlag = False

    def start(self):
        if not self.j: return False
        self._status = 'ok'
        self.plugin = MyWidgetPlugin(self)
        self.start_gamepad()
        self.start_haptic()
        return True

    def status(self):
        if self.j and self.serviceThread:
            return 'ok'
        else:
            return 'error'

    def messages(self):
        return []

    def stop(self):
        self.stop_gamepad()
        self.stop_haptic()
        self.plugin = None
        self._status=''


    def get(self):
        # Read gamepad, if there's a message override the haptic
        res = self.update_gamepad()
        if res is not None:
            self.activeDevice = 'gamepad'
            if 'velocity' in res and res['velocity'] == [0,0,0]:
                #stop message sent
                self.activeDevice = None
            return res
        
        res = self.update_haptic()
        if res is not None:
            self.activeDevice = 'haptic'
            return res

        self.activeDevice = None
        return res

    def makeArmsTask(self,q):
        """returns a task message that sends the robot to a desired configuration"""
        msg = {}
        msg['type'] = 'JointPose'
        msg['parts'] = ["left","right"]
        msg['positions'] = [[q[i] for i in self.left_arm_link_indices],
                            [q[i] for i in self.right_arm_link_indices]]
        msg['speed'] = 1
        msg['safe'] = 0
        return msg

    def glPlugin(self):
        return self.plugin

def make():
    return SimpleUITaskGenerator()
