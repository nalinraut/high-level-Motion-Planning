#!/usr/bin/env python
import klampt
from klampt import *
from klampt import WidgetSet,RobotPoser
from klampt.glprogram import *
from klampt.glcommon import GLWidgetPlugin

import math
import time
import signal
import copy
import os
import sys
import asyncore

from sspp.topic import *
from sspp.service import Service

ebolabot_root = os.getenv("EBOLABOT_PATH",".")
sys.path.append(ebolabot_root)

from Common.system_config import EbolabotSystemConfig
import threading


# ==========================
# Global parameters: Network Connection
# ==========================

#Configuration variable: where's the haptic service?
try:
    #default 'tcp://192.168.1.128:3456' ?
    haptic_service_addr = EbolabotSystemConfig.get_ip('haptic_server_computer_ip')
except Exception:
    print "Haptic device server not configured. Please configure the appropriate variable in",EbolabotSystemConfig.system_config_fn
    exit(1)

#Configuration variable: where's the state server?
system_state_addr = EbolabotSystemConfig.getdefault_ip('state_server_computer_ip',('localhost',4568))


# ================================
# Global variables: Modes and Transforms
# ================================

# reads from an external haptic_service and makes a simple UI that provides
# system state
# -.controller.task
#
# reads in from system state
# - .command.qcmd
# - .controller.traj_q_end
# - .robot.endEffectors.0.dest_xform
# - .robot.endEffectors.1.dest_xform

modes = ['normal','scaling','gripper']

defaultDeviceState = {'mode':'normal',
                      'time': 0.0,
                      'buttonDown':False,
                      'position': [0.0, 0.0, 0.0],
                      'rotationMoment': [0.0, 0.0, 0.0],
                      'jointAngle': [0.0, 0.0, 0.0], 
                      'gimbalAngle': [0.0, 0.0, 0.0], 
                      "velocity": [0.0, 0.0, 0.0],
                      "angularVelocity": [0.0, 0.0, 0.0],
                      'rotationScale':1.0,
                      'positionScale':3.0, 
                      'devicePreviousTransform': None,
                      'deviceCurrentTransform': None,
                      'moveToTransform': None,
                      'linearVel': [0.0, 0.0, 0.0],
                      'angularVel': [0.0, 0.0, 0.0],
                      'newupdate': False
                      }

endEffectorIndices = [25,45]

scale_m2cm = 100

''' Transform from device to robot/simulation (in column format) ''' 
# === For robot === 
# mappingTransform = ([0,-1,0,   0,0,1,   -1,0,0],
#                                [0,0,0])
# === For simulation === 
mappingDeviceToWidget = ([0,1,0,   -1,0,0,   0,0,1],
                           [0,0,0])
mappingDevice = ([0,-1,0,   0,0,1,   -1,0,0],
                               [0,0,0])

mappingTransform = se3.mul(mappingDeviceToWidget,mappingDevice)

endEffectorLocalTransforms = [(so3.identity(),(0,0,0.08)),
                              (so3.identity(),(0,0,0.08))]
# =========================================================================

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
        self.oldTime = 0.0


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

        for i in range(10):
            ee1 = self.serviceThread.ee1Getter.get()
            ee2 = self.serviceThread.ee1Getter.get()
            if ee1 != None and ee2 != None:
                break
            time.sleep(0.1)
        if ee1 == None:
            print "Warning, system state service doesn't seem to have .robot.endEffectors.0.xform.sensed"
            print "Press enter to continue"
            raw_input()
        if ee2 == None:
            print "Warning, system state service doesn't seem to have .robot.endEffectors.1.xform.sensed"
            print "Press enter to continue"
            raw_input()


    def display(self):
        #Put your display handler here
        #the example draws the posed robot in grey, the sensed
        #configuration in transparent red and the commanded
        #configuration in transparent green

        #this line will draw the world
        GLWidgetProgram.display(self)
        
        q = self.serviceThread.qsnsGetter.get()
        ee1 = self.serviceThread.ee1Getter.get()
        ee2 = self.serviceThread.ee1Getter.get()

        # print "End Effector 1 = ", ee1

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

        self.getMsg()


    def getMsg(self):
        deviceState = self.serviceThread.updater.deviceState
        ee1 = self.serviceThread.ee1Getter.get()
        ee2 = self.serviceThread.ee1Getter.get()
        ee = [ee1, ee2]
        haptic_task_type = "cartesian"
        if haptic_task_type == "cartesian":
            msg = self.renderCartesianVelocityMsg(deviceState)
        elif haptic_task_type =="multiIK":
            msg = self.renderMultiIKMsg(deviceState, ee)

        # === Generate cartesian velocity task === 
        for i, dstate in enumerate(deviceState):
            if dstate['buttonDown']:
                if dstate['newupdate']:    
                    if haptic_task_type == "cartesian":
                        print "send cartesian control msg: ", msg
                    elif haptic_task_type =="multiIK":    
                        print "send Multi-IK control msg: ", msg
                    self.serviceThread.taskSetter.set(msg)

        # === Generate Multi-IK task ===



    def renderCartesianVelocityMsg(self, deviceState):
        """
        Prepare a task message for Controller dispatcher - Send a multiIK Task
        """
        msg = {}
        msg['type'] = 'CartesianVelocity'
        msg['limb'] = 'both'
        # msg['speed'] = 1
        msg['linear'] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg['angular'] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
        msg['safe'] = 0

        if len(deviceState)==0: return

        for i,dstate in enumerate(deviceState):
            if dstate['buttonDown']:
                if dstate['newupdate']:
                    msg['linear'][0+i*3:3+i*3] = dstate['linearVel']
                    msg['angular'][0+i*3:3+i*3] = dstate['angularVel']
        return msg


    def renderMultiIKMsg(self, deviceState, ee):
        """
        Prepare a task message for Controller dispatcher - Send a multiIK Task
        """
        msg = {}
        msg['components'] = []

        if len(deviceState)==0: return

        for i,dstate in enumerate(deviceState):
            if dstate['buttonDown']: 
                if dstate['newupdate']:
                    ikgoal = {}
                    ikgoal['link'] = endEffectorIndices[i]
                    Rlocal,tlocal = endEffectorLocalTransforms[i]
                    ikgoal['localPosition'] = tlocal
                    ikgoal['endPosition'] = [dstate['moveToTransform'][1][k] + ee[i][1][k]  for k in range(0,3) ]         
        msg['components'].append(ikgoal)
        return msg

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

        elif c == 'i':
            ql = [0.25, -0.5, -0.5, 1.5, -0.25, 1.0, 0.5] 
            qr = [-0.25, -0.5, 0.5, 1.5, 0.25, 1.0, 0.5]
            msg = {}
            msg['type'] = 'JointPose'
            msg['limb'] = 'both'
            msg['position'] = ql + qr
            msg['speed'] = 1
            msg['safe'] = 0
            print "Sending message",msg
            self.serviceThread.taskSetter.set(msg)

        else:
            GLWidgetProgram.keyboardfunc(self,c,x,y)
            self.refresh()


def toggleMode(dstate):
    if dstate['mode'] == 'normal':
        dstate['mode'] = 'scaling'
    elif dstate['mode'] == 'scaling':
        dstate['mode'] = 'gripper'
    else:
        dstate['mode'] = 'normal'


def mapPosition(vector):
    x = vector[0]
    y = vector[1]
    z = vector[2]
    vector[0] = -z 
    vector[1] = -x
    vector[2] = y


class HapticWidgetUpdateService(Service):
    """
    Reads from a HapticService to update the device state
    - mapping transform from haptic device to cartesian workspace 
    - switch mode
    - update deviceState
    """
    def __init__(self,addr):
        Service.__init__(self)
        self.open(addr,asServer=False)
        self.updateFreq = 50
        self.deviceState = []

    def onMessage(self, msg):
        deviceState = self.deviceState      
        #print "Getting haptic message"
        #print msg
        if msg['type'] != 'MultiHapticState':
            print "Strange message type",msg['type']
            return
        if len(deviceState)==0:
            print "Adding",len(msg['devices'])-len(deviceState),"haptic devices"
            while len(deviceState) < len(msg['devices']):
                deviceState.append(defaultDeviceState.copy())

        if len(msg['devices']) != len(deviceState):
            print "Incorrect number of devices in message:",len(msg['devices'])
            return

        #change overall state depending on button 2
        if 'events' in msg:
            for e in msg['events']:
                #print e['type']
                if e['type']=='b2_down':
                    dstate = deviceState[e['device']]
                    toggleMode(dstate)
                    print 'Changing device',e['device'],'to state',dstate['mode']

        for i in range(len(deviceState)):
            dmsg = msg['devices'][i]
            dstate = deviceState[i]
            
            if dmsg['button1'] == 1:
                #drag widget if button 1 is down
                oldButtonDown = dstate['buttonDown']
                oldTime = dstate['time']
                dstate['buttonDown'] = True
                dstate['newupdate'] = False
                if oldButtonDown == False:
                    print "start haptic motion ... "
                #moving
                if dstate['mode'] == 'normal':
                    if dstate['buttonDown']:
                        #  ===== read from msg =====
                        dstate['position'] = dmsg['position']
                        dstate['rotationMoment'] = dmsg['rotationMoment']
                        dstate['velocity'] = dmsg['velocity']
                        dstate['angularVelocity'] = dmsg['angularVelocity']
                        dstate['jointAngle'] = dmsg['jointAngle']
                        dstate['gimbalAngle'] = dmsg['gimbalAngle']
                        dstate['time'] = dmsg['time']
                        newTime = dstate['time']
                        
                        # --- take initial position when button 1 is pressed----
                        if not oldButtonDown:
                            dstate['devicePreviousTransform'] = se3.mul(mappingTransform,(so3.from_moment(dmsg['rotationMoment']),dmsg['position']))
                            continue

                        dstate['deviceCurrentTransform'] = se3.mul(mappingTransform, (so3.from_moment(dmsg['rotationMoment']),dmsg['position']))
                        if newTime != oldTime:
                            # print "previous position = ", dstate['devicePreviousTransform'][1]
                            # print "current position = ", dstate['deviceCurrentTransform'][1]
                            timeInterval = newTime - oldTime
                            print "========================" 
                            print "time = ", timeInterval
                            delta_Pos = vectorops.mul(vectorops.sub(dstate['deviceCurrentTransform'][1], dstate['devicePreviousTransform'][1]), dstate['positionScale'])
                            vel   = vectorops.div(delta_Pos, timeInterval)

                            delta_Moment = vectorops.mul(tuple(so3.moment(so3.mul(dstate['deviceCurrentTransform'][0], so3.inv(dstate['devicePreviousTransform'][0])))), dstate['rotationScale'])
                            angvel = vectorops.div(delta_Moment, timeInterval)

                            dstate['moveToTransform'] = (so3.from_moment(delta_Moment),delta_Pos)

                            # print "dstate['moveToTransform'] = ", dstate['moveToTransform']
                            # print "vel = [%2.4f %2.4f %2.4f]" % (vel[0], vel[1], vel[2])
                            # print "angvel = [%2.4f %2.4f %2.4f]" % (angvel[0], angvel[1], angvel[2])
                            dstate['linearVel'] = list(vel)
                            dstate['angularVel'] = list(angvel)
                            
                            dstate['newupdate'] = True
                        dstate['devicePreviousTransform'] = dstate['deviceCurrentTransform']
                    else:
                        pass
                elif dstate['mode'] == 'gripper':
                    pass
                elif dstate['mode'] == 'scale':
                    pass 
                    
            else:
                dstate['buttonDown'] = False    


        
class ServiceThread(threading.Thread):
    """This thread takes care of running asyncore while the GUI is running"""
    def __init__(self):
        threading.Thread.__init__(self)
        self.updater = HapticWidgetUpdateService(haptic_service_addr)

        # ===== Set up the listeners: sensed, commanded, and destination configs ---> to display in GUI
        self.qsnsGetter = TopicListener(system_state_addr,'.robot.sensed.q')
        self.qcmdGetter = TopicListener(system_state_addr,'.robot.command.q')
        self.qdstGetter = TopicListener(system_state_addr,'.controller.traj_q_end')
        self.statusGetter = TopicListener(system_state_addr,'.controller.task_status')
        self.ee1Getter = TopicListener(system_state_addr,'.robot.endEffectors.0.xform.sensed')
        self.ee2Getter = TopicListener(system_state_addr,'.robot.endEffectors.1.xform.sensed')

        # ===== open system_state_server to send tasks
        self.taskSetter = TopicServiceBase('.controller.task')
        self.taskSetter.open(system_state_addr)

    def run(self):
        self.updater.run()

    def kill(self):
        self.updater.terminate()


if __name__ == "__main__":

    print "widget_haptics: use haptic devices as input devices"
    print "Press [space] to send milestones.  Press q to exit."

    # ===== Load Klampt Model =====
    klampt_model = EbolabotSystemConfig.get("klampt_model",lambda s:s.encode('ascii','ignore'))
    world = WorldModel()
    res = world.readFile(klampt_model)
    if not res:
        raise RuntimeError("Unable to load Ebolabot model "+klampt_model)

    print "Connecting to state server on",system_state_addr

    # ===== Start service thread =====
    serviceThread = ServiceThread()
    serviceThread.start()

    # ===== cleanly handle Ctrl+C =====
    def handler(signal,frame):
        print "Exiting due to Ctrl+C"
        serviceThread.kill()
        exit(0)
    signal.signal(signal.SIGINT,handler)
    
    viewer = MyGLViewer(world,serviceThread)
    viewer.run()

    #cleanup
    serviceThread.kill()
