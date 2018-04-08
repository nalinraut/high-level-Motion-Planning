#!/usr/bin/env python
from sspp.service import Service
from sspp.topic import *
import klampt
from klampt import so3,se3,vectorops
from klampt.glprogram import *
from klampt import gldraw
import threading
import copy
import math

#Configuration variable: where's the state server?
system_state_addr = ('localhost',4568)

#Configuration variable: where's the haptic service?
haptic_service_addr = ('192.168.1.128',3456)

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
                      'buttonDown':False,
                      'moveStartDeviceTransform':None,
                      'moveStartWidgetTransform':None,
                      'widgetTransform':se3.identity(),
                      'rotationScale':1,
                      'positionScale':5
                      }

deviceState = []

#device +z is gripper -z
#device +x is gripper -y
#device +y is gripper -x
deviceToWidgetTransform = ([0,-1,0,   -1,0,0,   0,0,-1],
                           [0,0,0])

endEffectorIndices = [25,45]
#widget coordinates in local frame of end effector
endEffectorLocalTransforms = [(so3.identity(),(0,0,0.08)),
                              (so3.identity(),(0,0,0.08))]

#TendEffector*localCoordinates = widget 

def toggleMode(dstate):
    if dstate['mode'] == 'normal':
        dstate['mode'] = 'scaling'
    elif dstate['mode'] == 'scaling':
        dstate['mode'] = 'gripper'
    else:
        dstate['mode'] = 'normal'

class HapticCartesianTaskService(Service):
    """
    Sends messages as a client to an StructureService
    """
    def __init__(self,addr,topic='.controller.task'):
        Service.__init__(self)
        self.topic = topic
        self.lastData = None
        self.open(addr,asServer=False)
        print "Publishing multi_ik tasks to",addr[0]+':'+str(addr[1])+'/'+topic
    def onUpdate(self):
        global deviceState
        if len(deviceState)==0: return
        cobj = {'type':'multi_ik','safe':1,'components':[]}
        for i,dstate in enumerate(deviceState):
            if not dstate['buttonDown']: continue
            ikgoal = {}
            ikgoal['link'] = endEffectorIndices[i]
            Rlocal,tlocal = endEffectorLocalTransforms[i]
            ikgoal['localPosition'] = tlocal
            ikgoal['endPosition'] = dstate['widgetTransform'][1]
            ikgoal['endRotation'] = so3.moment(so3.mul(Rlocal,dstate['widgetTransform'][0]))
            ikgoal['posConstraint'] = 'fixed'
            ikgoal['rotConstraint'] = 'fixed'
            cobj['components'].append(ikgoal)
        if cobj != self.lastData:
            self.sendMessage({'type':'set','path':self.topic,'data':cobj})
            self.lastData = cobj
        return Service.onUpdate(self)

class HapticWidgetUIService (Service):
    """Reads from a HapticService to update the device state"""
    def __init__(self,addr):
        Service.__init__(self)
        self.open(addr,asServer=False)
        self.viewer = None
        self.widgetGetters = None

    def onMessage(self,msg):
        global deviceState,defaultDeviceState
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
                dstate['buttonDown'] = True
                #moving
                if dstate['mode'] == 'normal':
                    if not oldButtonDown:
                        dstate['moveStartDeviceTransform'] = (so3.from_moment(dmsg['rotationMoment']),dmsg['position'])
                        if self.widgetGetters == None:
                            dstate['moveStartWidgetTransform'] = dstate['widgetTransform']
                        else:
                            Twidget = self.widgetGetters[i]()
                            if Twidget != None:
                                dstate['moveStartWidgetTransform'] = Twidget
                            else:
                                dstate['moveStartWidgetTransform'] = dstate['widgetTransform']

                    #perform transformation of widget
                    deviceTransform = (so3.from_moment(dmsg['rotationMoment']),dmsg['position'])
                    #compute relative motion
                    Tdev = se3.mul(deviceToWidgetTransform,deviceTransform)
                    TdevStart = se3.mul(deviceToWidgetTransform,dstate['moveStartDeviceTransform'])
                    relDeviceTransform = (so3.mul(Tdev[0],so3.inv(TdevStart[0])),vectorops.sub(Tdev[1],TdevStart[1]))
                    #scale relative motion
                    Rrel,trel = relDeviceTransform
                    wrel = so3.moment(Rrel)
                    wrel = vectorops.mul(wrel,dstate['rotationScale'])
                    trel = vectorops.mul(trel,dstate['positionScale'])
                    #print "Moving",trel,"rotating",wrel
                    relDeviceTransform = (so3.from_moment(wrel),trel)
                    #perform transform
                    dstate['widgetTransform'] = se3.mul(dstate['moveStartWidgetTransform'],relDeviceTransform)
                elif dstate['mode'] == 'scaling':
                    if not oldButtonDown:
                        dstate['moveStartDeviceTransform'] = (so3.from_moment(dmsg['rotationMoment']),dmsg['position']) 
                    #perform scaling
                    deviceTransform = (so3.from_moment(dmsg['rotationMoment']),dmsg['position'])
                    oldDeviceTransform = dstate['moveStartDeviceTransform']
                    znew = deviceTransform[1][1]
                    zold = oldDeviceTransform[1][1]
                    posscalerange = [1e-2,20]
                    rotscalerange = [0.5,2]
                    newposscale = min(max(dstate['positionScale']*math.exp((znew-zold)*10),posscalerange[0]),posscalerange[1])
                    newrotscale = min(max(dstate['rotationScale']*math.exp((znew-zold)*4),rotscalerange[0]),rotscalerange[1])
                    if any(newposscale == v for v in posscalerange) or any(newrotscale == v for v in rotscalerange):
                        pass #dont change the scale
                    else:
                        dstate['positionScale'] = newposscale
                        dstate['rotationScale'] = newrotscale
                    print "Setting position scale to",dstate['positionScale'],"rotation scale to",dstate['rotationScale']
                    #update start device transform
                    dstate['moveStartDeviceTransform'] = deviceTransform
                elif dstate['mode'] == 'gripper':
                    print "Gripper mode not available"
                    dstate['mode'] = 'normal'
                    dstate['buttonDown'] = False
            else:
                dstate['buttonDown'] = False
        if self.viewer: self.viewer.update(deviceState)
        else: print "No viewer..."

class HapticUIViewer(GLRealtimeProgram):
    def __init__(self,worldfn = None):
        GLRealtimeProgram.__init__(self,"Haptic UI Viewer")
        self.statelock = threading.Lock()
        self.state = None
        if worldfn == None:
            self.world = None
            self.robot = None
        else:
            self.world = klampt.WorldModel()
            if not self.world.readFile(worldfn):
                print "HapticUIViewer: Warning, could not load world file",worldfn
                print "Proceeding without robot feedback"
                self.robot = None
                self.world = None
            else:
                self.robot = self.world.robot(0)
                self.qcmdGetter = TopicListener(system_state_addr,'.robot.command.qcmd')
                self.qdestGetter = TopicListener(system_state_addr,'.controller.traj_q_end')

    def update(self,deviceState):
        self.statelock.acquire()
        self.state = copy.deepcopy(deviceState)
        self.statelock.release()

    def display(self):
        if self.state == None: return
        glEnable(GL_LIGHTING)
        self.statelock.acquire()
        for i,d in enumerate(self.state):
            Twidget = d['widgetTransform']
            T = (so3.mul(Twidget[0],so3.inv(deviceToWidgetTransform[0])),Twidget[1])
            width = 0.05*d['rotationScale']
            length = 0.1*d['positionScale']
            gldraw.xform_widget(T,length,width)
        self.statelock.release()
        if self.robot:
            #draw robot
            qcmd = self.qcmdGetter.get()
            if qcmd:
                self.robot.setConfig(qcmd)
                self.robot.drawGL(True)
            else:
                self.robot.drawGL(True)
            qdest = self.qdestGetter.get()
            if qdest:
                glEnable(GL_BLEND)
                glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
                glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[0,1,0,0.5])
                self.robot.setConfig(qdest)
                self.robot.drawGL(False)
                glDisable(GL_BLEND)
        glDisable(GL_LIGHTING)

    def display_screen(self):
        if self.state == None: return
        glDisable(GL_LIGHTING)
        self.statelock.acquire()
        try:
            glColor3f(0,0,0,1)
            for i,d in enumerate(self.state):
                glRasterPos2i(20+500*i,60)
                gldraw.glutBitmapString(GLUT_BITMAP_HELVETICA_18,d['mode'].capitalize())
            glRasterPos2i(0,0)
        except Exception as e:
            print "Exception called",e
            print traceback.format_exc()
        finally:
            self.statelock.release()


class ServiceThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.writer = HapticCartesianTaskService(system_state_addr)
        self.reader = HapticWidgetUIService(haptic_service_addr)
        getter1 = TopicListener(system_state_addr,'.robot.endEffectors.0.dest_xform')
        getter2 = TopicListener(system_state_addr,'.robot.endEffectors.1.dest_xform')
        self.reader.widgetGetters = [lambda :se3.mul(getter1.get(),endEffectorLocalTransforms[0]),
                                     lambda :se3.mul(getter2.get(),endEffectorLocalTransforms[1])]
    def run(self):
        self.writer.run(0.05)
    def kill(self):
        self.writer.terminate()


if __name__ == '__main__':
    program = HapticUIViewer('/home/motion/Klampt/data/baxterserial.xml')
    st = ServiceThread()
    st.reader.viewer = program
    st.start()
    try:
        program.run()
    except Exception as e:
        print "Exception called in HapticUIViewer"
        print e
        print traceback.format_exc()
    finally:
        print "Killing service thread"
        st.kill()

