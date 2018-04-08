"""Interface to the Robotiq gripper library via ctypes"""

from ctypes import *
import platform

#finger preshape modes
MODE_BASIC=0
MODE_PINCH=1
MODE_WIDE=2
MODE_SCISSOR=3
mode_names = ['basic','pinch','wide','scissor']
mode_name_to_index = dict((v,i) for (i,v) in enumerate(mode_names))

#actuator indices
FINGER1=0
FINGER2=1
FINGER3=2
SCISSOR=3
actuator_names = ['finger1','finger2','finger3','scissor']
actuator_name_to_index = dict((v,i) for (i,v) in enumerate(actuator_names))

#global variables
robotiq_lib = None

def setup(libpath=""):
    """Robotiq DLL setup.
    
    Arguments:
    - libpath: the path to the libRobotiq.so shared library
    """
    global robotiq_lib
    #check to see if setup was called before?
    if robotiq_lib != None:
        print "robotiq.setup(): Warning, already called setup before"
        return

    #load the C library and set up the return types
    dllname = "Robotiq"
    if platform.system() == 'Windows':
        robotiq_lib = windll.LoadLibrary(libpath+dllname+".dll")
    elif platform.system().startswith('CYGWIN'):
        robotiq_lib = cdll.LoadLibrary(libpath+"cyg"+dllname+".dll")
    else:
        robotiq_lib = cdll.LoadLibrary(libpath+"lib"+dllname+".so")

    #set up some return types
    robotiq_lib.robotiqCreate_Modbus.restype = c_void_p
    robotiq_lib.robotiqGetFaultStatus.restype = c_ubyte
    robotiq_lib.robotiqGetMode.restype = c_ubyte
    robotiq_lib.robotiqGetActuatorPosition.restype = c_ubyte
    robotiq_lib.robotiqGetActuatorSpeed.restype = c_ubyte
    robotiq_lib.robotiqGetActuatorPositionRequest.restype = c_ubyte
    robotiq_lib.robotiqGetActuatorForce.restype = c_ubyte
    print
    print "****** Robotiq Python API started *******"
    print 

class Actuator:
    def __init__(self,handle,index):
        self.handle = handle
        self.index = index
    def goto(self,position,speed,force):
        return robotiq_lib.robotiqActuatorGoto(self.handle,self.index,c_ubyte(position),c_ubyte(speed),c_ubyte(force));
    def setPosition(self,position):
        """Queues up an individual finger position command"""
        return robotiq_lib.robotiqSetActuatorPosition(self.handle,self.index,c_ubyte(position))
    def setSpeed(self,speed):
        """Queues up an individual finger speed command"""
        return robotiq_lib.robotiqSetActuatorSpeed(self.handle,self.index,c_ubyte(speed))
    def setForce(self,force):
        """Queues up an individual finger force command"""
        return robotiq_lib.robotiqSetActuatorForce(self.handle,self.index,c_ubyte(force))
    def sendCommands(self):
        """If there are queued up commands, sends them.  Note: sends commands for all fingers, not just
        this one"""
        return robotiq_lib.robotiqSendActuatorCommands(self.handle)
    def getPosition(self):
        return robotiqGetActuatorPosition(self.handle,self.index)
    def getSpeed(self):
        return robotiqGetActuatorSpeed(self.handle,self.index)
    def getPositionRequest(self):
        return robotiqGetActuatorPositionRequest(self.handle,self.index)
    def getForce(self):
        return robotiqGetActuatorForce(self.handle,self.index)
    def isMoving(self):
        return robotiq_lib.robotiqIsActuatorMoving(self.handle,self.index)
    def isStopped(self):
        return robotiq_lib.robotiqIsActuatorStopped(self.handle,self.index)
    def isStoppedClosing(self):
        return robotiq_lib.robotiqIsActuatorStoppedClosing(self.handle,self.index)
    def isStoppedOpening(self):
        return robotiq_lib.robotiqIsActuatorStoppedOpening(self.handle,self.index)
    def isDone(self):
        return robotiq_lib.robotiqIsActuatorDone(self.handle,self.index)


class Gripper:
    """The primary interface to the Robotiq gripper.
    Follows identically the C API, except individual
    actuator commands are given by the Actuator class
    retrieved by the Gripper.actuator() method. 

    Once a Gripper is constructed by providing the
    port to the constructor or on successful open(),
    close() must be  called to free the memory &
    connection associate with this gripper.

    See Robotiq.h for more documentation.
    """
    def __init__(self,port=None,protocol='modbus'):
        if robotiq_lib == None:
            setup()
        self.handle = None
        if port != None:
            self.open(port,protocol)
    def open(self,port,protocol='modbus'):
        if self.handle != None:
            self.close()
        if protocol!='modbus':
            raise ValueError("Only accepts modbus RTU protocol for now")
        self.handle = robotiq_lib.robotiqCreate_Modbus(c_char_p(port))
        if self.handle == None:
            raise RuntimeError("Unable to connect on "+port)
        return True
    def close(self):
        if self.handle != None:
            robotiq_lib.robotiqDestroy(self.handle)
        self.handle = None
    def activate(self):
        return robotiq_lib.robotiqActivate(self.handle)
    def deactivate(self):
        return robotiq_lib.robotiqDeactivate(self.handle)!=0
    def updateStatus(self):
        return robotiq_lib.updateStatus(self.handle)
    def printStatus(self):
        raise NotImplementedError("printStatus only available in C API")
    def isReset(self):
        return robotiq_lib.robotiqIsReset(self.handle)
    def isActivated(self):
        return robotiq_lib.robotiqIsActivated(self.handle)
    def isChangingMode(self):
        return robotiq_lib.robotiqIsChangingMode(self.handle)
    def isActivating(self):
        return robotiq_lib.robotiqIsActivating(self.handle)
    def isReady(self):
        return robotiq_lib.robotiqIsReady(self.handle)
    def getFaultStatus(self):
        """Returns the raw byte indicating the fault status of the hand"""
        return robotiq_lib.robotiqGetFaultStatus(self.handle)
    def setMode(self,mode):
        """Gets the current mode.  mode can either be a string or index 0-3"""
        global mode_name_to_index
        if isinstance(mode,str):
            mode = mode_name_to_index[mode]
        return robotiq_lib.robotiqSetMode(self.handle,mode)
    def setBasicMode(self):
        return robotiq_lib.robotiqSetBasicMode(self.handle)
    def setPinchMode(self):
        return robotiq_lib.robotiqSetPinchMode(self.handle)
    def setWideMode(self):
        return robotiq_lib.robotiqSetWideMode(self.handle)
    def setScissorMode(self):
        return robotiq_lib.robotiqSetScissorMode(self.handle)
    def getMode(self):
        """Gets the name of the current mode"""
        global mode_names
        m = robotiq_lib.robotiqGetMode(self.handle)
        return mode_names[m]
    def goto(self,position,speed=None,force=None):
        if speed==None:
            return robotiq_lib.robotiqGotoSimple(self.handle,c_ubyte(position))
        else:
            return robotiq_lib.robotiqGoto(self.handle,c_ubyte(position),c_ubyte(speed),c_ubyte(force))
    def actuator(self,actuator):
        """Retrieves a given actuator.  This can either be a string or index 0-3."""
        if isinstance(actuator,str):
            actuator = actuator_name_to_index[actuator]
        return Actuator(self.handle,actuator)
    def sendActuatorCommands(self):
        """If there are queued up actuator commands, sends them now."""
        return robotiq_lib.robotiqSendActuatorCommands(self.handle)
    def areAllFingersMoving(self):
        return robotiq_lib.robotiqAreAllFingersMoving(self.handle)
    def areSomeFingersMoving(self):
        return robotiq_lib.robotiqAreSomeFingersMoving(self.handle)
    def areSomeFingersStopped(self):
        return robotiq_lib.robotiqAreSomeFingersStopped(self.handle)
    def areAllFingersStopped(self):
        return robotiq_lib.robotiqAreAllFingersStopped(self.handle)
    def areAllFingersDone(self):
        return robotiq_lib.robotiqAreAllFingersDone(self.handle)
    def setAutomaticRelease(self):
        """Call this on emergency stop -- can't address
        gripper after this."""
        return robotiq_lib.robotiqSetAutomaticRelease(self.handle)
    def getAutomaticRelease(self):
        return robotiq_lib.robotiqGetAutomaticRelease(self.handle)
    def setIndividualFingerControl(self,enabled=True):
        """Enable/disable individual finger control"""
        return robotiq_lib.robotiqSetIndividualFingerControl(self.handle,enabled)
    def setIndividualScissorControl(self,enabled=True):
        """Enable/disable individual scissor control"""
        return robotiq_lib.robotiqSetIndividualScissorControl(self.handle,enabled)
    def getIndividualFingerControl(self):
        return robotiq_lib.robotiqGetIndividualFingerControl(self.handle)
    def getIndividualScissorControl(self):
        return robotiq_lib.robotiqGetIndividualScissorControl(self.handle)
    def waitForReady(self,maxseconds=10.0,pollfreq=50):
        """Waits until the gripper is in the ready state"""
        return robotiq_lib.robotiqWaitForReady(self.handle,c_double(maxseconds),pollfreq)
    def waitForGoto(self,maxseconds=10,pollfreq=50):
        """Returns 1 if fingers reached destination, 0 if stopped, -1 if maxseconds time elapsed."""
        return robotiq_lib.robotiqWaitForGoto(self.handle,c_double(maxseconds),pollfreq)       

if __name__=="__main__":
    import os
    import sys
    import time
    if len(sys.argv) < 2:
        print "Usage: python robotiq.py PORT"
        print "Runs a test script"
        exit(0)

    print "Testing Robotiq..."
    gripper = Gripper(sys.argv[1])

    if not gripper.activate(): print "Error Activating"
    if not gripper.waitForReady(): print "Error waiting for ready"
    
    print "Setting pinch mode..."
    gripper.setPinchMode()
    gripper.waitForReady()
    
    print "Setting basic mode..."
    gripper.setBasicMode()
    gripper.waitForReady()
    
    print "Closing gripper..."
    gripper.goto(255,255,255)
    gripper.waitForGoto()
    
    print "Opening gripper..."
    gripper.goto(0)
    gripper.waitForGoto()
    
    print "Enabling individual finger control..."
    gripper.setIndividualFingerControl(True)
    gripper.actuator('finger1').setPosition(255)
    gripper.actuator('finger1').setSpeed(255)
    gripper.actuator('finger1').setForce(255)
    gripper.actuator('finger2').setPosition(196)
    gripper.actuator('finger2').setSpeed(128)
    gripper.actuator('finger2').setForce(128)
    gripper.actuator(FINGER3).setPosition(128)
    gripper.actuator(FINGER3).setSpeed(64)
    gripper.actuator(FINGER3).setForce(64)
    gripper.sendActuatorCommands()
    gripper.waitForGoto()
    
    print "Disabling individual finger control and opening gripper..."
    gripper.setIndividualFingerControl(False)
    gripper.goto(0)
    gripper.waitForGoto()
  
    print "Closing gripper..."
    gripper.close()
