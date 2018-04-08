from sspp.service import Service
from klampt.math import so3,se3,vectorops
import time
import math
import asyncore

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

def se3translation(v):
    return (so3.identity(),v)

def deviceToViewTransform(R,t):
    Ruser = so3.mul(so3.inv(handleToUser[0]),R)
    R = so3.mul(userToWorld[0],so3.mul(Ruser,worldToUser[0]))
    return (R,so3.apply([0,1,0,    0,0,1,   1,0,0],t))

def viewToDeviceTransform(R,t):
    Ruser = so3.mul(worldToUser[0],so3.mul(R,userToWorld[0]))
    R = so3.mul(handleToUser[0],Ruser)
    return (R,so3.apply(so3.inv([0,1,0,    0,0,1,   1,0,0]),t))

def deviceToWidgetTransform(R,t):
    """Given a device transform, map to the widget transform world coordinates."""
    Tview = so3.rotation((0,0,1),math.pi),[0,0,0]
    return se3.mul(Tview,deviceToViewTransform(R,t))

def widgetToDeviceTransform(R,t):
    """Given a widget transform in the world coordinates, map to a device transform """
    Tview = so3.rotation((0,0,1),math.pi),[0,0,0]
    Rview,tview = se3.mul(se3.inv(Tview),(R,t))
    return viewToDeviceTransform(Rview,tview)


class HapticDeviceState:
    def __init__(self):
        self.time = 0.0
        self.buttonDown = [False,False]
        self.buttonPressed = [False,False]
        self.buttonReleased = [False,False]
        self.position = [0.0, 0.0, 0.0]
        self.rotationMoment = [0.0, 0.0, 0.0]
        self.jointAngle = [0.0, 0.0, 0.0]
        self.gimbalAngle = [0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0]
        self.angularVelocity = [0.0, 0.0, 0.0]
        self.widgetInitialTransform = [None,None]
        self.widgetPreviousTransform = None
        self.widgetCurrentTransform = None
        self.linearVel = [0.0, 0.0, 0.0]
        self.angularVel = [0.0, 0.0, 0.0]
        self.newupdate = False

    def positionDeltaFromButtonPress(self,button=0):
        """Returns the difference between the current position and the button press position"""
        return vectorops.sub(self.widgetCurrentTransform[1], self.widgetInitialTransform[button][1])

    def rotationDeltaFromButtonPress(self,button=0):
        """Returns the difference between the current rotation and the button press rotation,
        relative to the frame of the button press rotation"""
        return so3.mul(self.widgetCurrentTransform[0], so3.inv(self.widgetInitialTransform[button][0]))

    def mark_as_read(self):
        self.newupdate = False

class HapticClient (Service):
    """Reads from a HapticService to update haptic device state"""
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
            self.deactivateHapticFeedback(True)
        Service.close(self)

    def onMessage(self,msg):
        #print "Getting haptic message"
        #print msg
        self.numMessages += 1
        if msg['type'] != 'MultiHapticState':
            print "Strange message type",msg['type']
            return
        if len(self.deviceState)==0:
            print "Adding",len(msg['devices'])-len(self.deviceState),"haptic devices"
            while len(self.deviceState) < len(msg['devices']):
                self.deviceState.append(HapticDeviceState())

        if len(msg['devices']) != len(self.deviceState):
            print "Incorrect number of devices in message:",len(msg['devices'])
            return

        #read button presses
        for dstate in self.deviceState:
            dstate.buttonPressed[0] = dstate.buttonPressed[1] = False
            dstate.buttonReleased[0] = dstate.buttonReleased[1] = False
        if 'events' in msg:
            for e in msg['events']:
                #print e['type']
                if e['type']=='b1_down':
                    dstate = self.deviceState[e['device']]
                    dstate.buttonPressed[0] = True
                elif e['type']=='b2_down':
                    dstate = self.deviceState[e['device']]
                    dstate.buttonPressed[1] = True
                elif e['type']=='b1_up':
                    dstate = self.deviceState[e['device']]
                    dstate.buttonReleased[0] = True
                elif e['type']=='b2_up':
                    dstate = self.deviceState[e['device']]
                    dstate.buttonReleased[1] = True

        for i in range(len(self.deviceState)):
            dmsg = msg['devices'][i]
            dstate = self.deviceState[i]
            if dstate.time == dmsg['time']:
                #no update...
                continue

            dstate.newupdate = True

            #  ===== read from msg =====
            dstate.position = dmsg['position']
            dstate.rotationMoment = dmsg['rotationMoment']
            dstate.velocity = dmsg['velocity']
            dstate.angularVelocity = dmsg['angularVelocity']
            dstate.jointAngles = dmsg['jointAngles']
            #dstate.gimbalAngle = dmsg['gimbalAngle']
            oldTime = dstate.time
            dstate.time = dmsg['time']
            #print "position",dmsg['position']
            #print "rotation moment",dmsg['rotationMoment']
            #print "angular velocity",dmsg['angularVelocity']
            dstate.widgetCurrentTransform = deviceToWidgetTransform(so3.from_moment(dmsg['rotationMoment']),dmsg['position'])
            if dmsg['button1'] == 1:
                oldButtonDown = dstate.buttonDown[0]
                dstate.buttonDown[0] = True
                if not oldButtonDown:
                    dstate.widgetInitialTransform[0] = dstate.widgetPreviousTransform
                    continue
            else:
                dstate.buttonDown[0] = False
            if dmsg['button2'] == 1:
                oldButtonDown = dstate.buttonDown[1]
                dstate.buttonDown[1] = True
                if not oldButtonDown:
                    dstate.widgetInitialTransform[1] = dstate.widgetPreviousTransform
                    continue
            else:
                dstate.buttonDown[1] = False
 
            newTime = dstate.time
            if newTime != oldTime and dstate.widgetPreviousTransform is not None:
                # print "previous position = ", dstate.widgetPreviousTransform[1]
                # print "current position = ", dstate.widgetCurrentTransform[1]
                timeInterval = newTime - oldTime
                #print "========================"
                #print "time = ", timeInterval

                delta_Pos = vectorops.sub(dstate.widgetCurrentTransform[1], dstate.widgetPreviousTransform[1])
                vel   = vectorops.div(delta_Pos, timeInterval)

                delta_Moment = so3.moment(so3.mul(dstate.widgetCurrentTransform[0], so3.inv(dstate.widgetPreviousTransform[0])))
                angvel = vectorops.div(delta_Moment, timeInterval)

                # print "vel = [%2.4f %2.4f %2.4f]" % (vel[0], vel[1], vel[2])
                # print "angvel = [%2.4f %2.4f %2.4f]" % (angvel[0], angvel[1], angvel[2])
                dstate.linearVel = list(vel)
                dstate.angularVel = list(angvel)

            #end of loop, store previous transform
            dstate.widgetPreviousTransform = dstate.widgetCurrentTransform

    def onUpdate(self):
        t = time.time()
        #print self.numMessages,"haptic messages read over time",t-self.lastUpdateTime
        self.numMessages = 0
        self.lastUpdateTime = t


    def getState(self,device):
        """Returns the HapticDeviceState corresponding to the given device"""
        return self.deviceState[device]

    def deactivateHapticFeedback(self,block=False,device='all'):
        """turns off haptic feedback.  If block is true, this blocks until
        all messages are sent.
        """
        if device == 'all':
            devices = [0,1]
        else:
            devices = [device]
        msg = {}
        msg['type'] = 'HapticForceCommand'
        msg['enabled'] = 0
        for d in devices:
            msg['device'] = d
            self.sendMessage(msg)
        self.sentHapticFeedback = False
        if block: 
            #loop until all messages are sent
            while self.writable():
                asyncore.loop(timeout = 0.02, count=100, map=self.map)

    def activateDrag(self,kD,device='all'):
        """Activates viscous damping, with the parameter kD"""
        self.sentHapticFeedback = True
        if device == 'all':
            devices = [0,1]
        else:
            devices = [device]
        msg = {}
        msg['type'] = 'HapticForceCommand'
        msg['enabled'] = 1
        if hasattr(kD,'__iter__'):
            if len(kD) == 3:
                kD = [kD[0],0,0,0,kD[1],0,0,0,kD[2]]
            msg['damping'] = [-v for v in kD]
        else:
            msg['damping'] = [-kD]
        msg['forceCenter'] = [0,0,0]
        for device in devices:
            msg['device'] = device
            self.sendMessage(msg)
        
    def activateSpring(self,kP,kD,center=None,device='all'):
        """Activates a spring with proportional and damping terms
        with the parameter kP, kD.  If center=None, this uses the device'self
        current position as the center."""
        self.sentHapticFeedback = True
        if device == 'all':
            devices = [0,1]
        else:
            devices = [device]
        msg = {}
        msg['type'] = 'HapticForceCommand'
        msg['enabled'] = 1
        if hasattr(kP,'__iter__'):
            if len(kP) == 3:
                kP = [kP[0],0,0,0,kP[1],0,0,0,kP[2]]
            msg['linear'] = [-v for v in kP]
        else:
            msg['linear'] = [-kP]
        if hasattr(kD,'__iter__'):
            if len(kD) == 3:
                kD = [kD[0],0,0,0,kD[1],0,0,0,kD[2]]
            msg['damping'] = [-v for v in kD]
        else:
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
                    #print "Setting center to",self.deviceState[device].position
                    msg['center'] = self.deviceState[device].position
            self.sendMessage(msg)
