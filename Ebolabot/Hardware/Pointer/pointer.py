"""
Controls local PPU device, providing for reading its state (joint angles, cartesian position, etc) and writing 
commands (joints, position, stopping, etc).

Created by Peter Moran (Spring 2016)
with IK and FK functions by Carrina Dong
"""

from serialhandler import SerialReqReplyHandler
from serial import Serial, SerialException
from math import pi
import math
from klampt import vectorops
from time import time

class PPU(object):
    # Mode codes (for setting PPU.mode)
    PASS = 1    # tell PPU to discard data when it recieves it, but continue with requests
    MOVE = 0    # for direct control of pan, tilt, ext
    PRESS = 2   # for extending until a specified force is detected
    
    # Hardware constants
    DYNA_RAD_RES = 0.00153435538 # smallest change in radians that dynamixel motors will recognise. Equates to 1 step.

    # IK parameters
    xb=0.5175
    hb=2.5

    # IK joint limits (Degrees/Inches)
    q1LL=-179
    q1UL=180
    q2LL=-90
    q2UL=22
    q3LL=7.5
    q3UL=9.47

    def __init__(self, port="/dev/ttyUSB0"):
        # Sent to PPU device
        self.cmdPan_Raw = None  # desired angle of base motor [0 - 4095] (motor steps)
        self.cmdTilt_Raw = None # desired angle of tilt motor [0 - 4095] (motor steps)
        self.cmdExt_Raw = None  # desired stroke for linear actuator [1000 - 2000] (micro-second pulse width)
        self.stepSize = 30      # speed limit for rotation motors (encoder step size)
        self.mode = PPU.MOVE    # mode to set PPU to
        self.modeParam = 0      # parameter to pass along with mode

        # Read from PPU device
        self.sensedConfig = None # desired [pan, tilt, ext] as reported by the pointer
        self.moving = None       # boolean array for which motors are moving [isPanning, isTilting, isExtending]
        self.extVel = None       # velocity of linear actuation (arbitrary units)
        self.extOverload = None  # relative load on linear actuator, increasing with duration. 2+ means contact likely

        # Attempt to connect to PPU device
        try:
            self.ser = Serial(port, 115200, timeout=0)
        except SerialException:
            print '\nNo device is on port "' + port + '"\n    Try connecting to a different port\n    Typically follows the format "/dev/ttyUSB##" where ## is a port number\n'
            raise
        
        # Set up Serial Reqest-Reply handler
        dataOutByteLens = [2,2,2,1,1,2]
        dataInByteLens = [2,2,2,1,1,1]
        self.serHndlr = SerialReqReplyHandler(self.ser, dataOutByteLens, dataInByteLens)
        
    def initToPhysicalConfig(self,retval='raw',timeout=None):
        """
        Forces a delay until the Arduino starts reporting data, and updates cmdPan_Raw, cmdTilt_Raw, 
        and cmdExt_Raw to the sensed configuration.
        
        Returns:
            Sensed configuration [pan_Raw, tilt_Raw, ext_Raw]
        """
        self.setCmdConfig([0,0,1000]) # because mode will be changed to PASS, these will be ignored, but we need to set them because None cannot be packed
        self.mode = PPU.PASS
        t0 = time()
        while self.sensedConfig is None or any(val is None for val in self.sensedConfig):
            self.update()
            if timeout != None:
                if time()-t0 > timeout:
                    print "PPU.initToPhysicalConfig(): failed to connect after timeout",timeout
                    print "PPU.initToPhysicalConfig(): check that port is set to PPU device rather than a different serial device"
                    return None
        # Hacky fix, should be implemented on Arduino side so invalid feedback will not be sent
        if self.sensedConfig[2] > 2000:
            self.sensedConfig[2] = 2000
        
        self.setCmdConfig(self.sensedConfig)
        self.mode = PPU.MOVE
        if retval=='raw':
            return self.sensedConfig
        elif retval=='rad/prct':
            return self.getSnsConfig('rad/prct')
            
    def update(self):
        """
        Sends commands and data to PPU device and reads data from PPU device. Should be called regularly.
        """
        self._setDataOut()
        self.serHndlr.handleSerialAndPerformOn('new data-in content', self._processDataIn)
    
    def setStepSize(self,size):
        """Sets the step size for the pan/tilt.

        Args:
            size - number of ticks per timestep
        """
        self.stepSize = size

    def setCmdConfig(self, config, mode = 'raw'):
        """
        Quickly sets the commanded configuration according to the specified mode. Also sets the PPU mode to MOVE.
        
        Args:
            config - list containing comanded postions, e.g. [cmd_pan, cmd_tilt, cmd_ext], according to mode
            mode - Set to 'rad/prct' if config will be passed as [cmdPan_rads, cmdTilt_rads, cmdExt_strokePrcent]
                   Set to 'raw' if config will be passed as raw values, i.e. [cmdPan_Raw, cmdTilt_Raw, cmdExt_Raw]
        """
        self.mode = PPU.MOVE
        self.modeParam = 0
        if mode == 'raw':
            self.cmdPan_Raw = config[0]
            self.cmdTilt_Raw = config[1]
            self.cmdExt_Raw = config[2]
        elif mode == 'rad/prct':
            self.setCmdRad('pan', config[0])
            self.setCmdRad('tilt', config[1])
            self.setCmdStrokPrct(config[2])
        else:
            raise ValueError('mode is not set to a valid option')

    def getCmdConfig(self, mode = 'raw'):
        """
        Gets commanded configuration according to the specified mode. 
        
        Args:
            mode - Set to 'rad/prct' if config will be passed as [cmdPan_rads, cmdTilt_rads, cmdExt_strokePrcent]
                   Set to 'raw' if config will be passed as raw values, i.e. [cmdPan_Raw, cmdTilt_Raw, cmdExt_Raw]
        """
        if mode == 'raw':
            return (self.cmdPan_Raw,self.cmdTilt_Raw,self.cmdExt_Raw)
        elif mode == 'rad/prct':
            return (dynaMX64ToRad(self.cmdPan_Raw),dynaMX64ToRad(self.cmdTilt_Raw),firgelliToStrokePct(self.cmdExt_Raw))
        else:
            raise ValueError('mode is not set to a valid option')

    def getSnsConfig(self, mode = 'raw'):
        """
        Gets sensed configuration according to the specified mode. 
        
        Args:
            mode - Set to 'rad/prct' if config will be passed as [snsPan_rads, snsTilt_rads, snsExt_strokePrcent]
                   Set to 'raw' if config will be passed as raw values, i.e. [snsPan_Raw, snsTilt_Raw, snsExt_Raw]
        """
        if mode == 'raw':
            return self.sensedConfig[:]
        elif mode == 'rad/prct':
            return (dynaMX64ToRad(self.sensedConfig[0]),dynaMX64ToRad(self.sensedConfig[1]),firgelliToStrokePct(self.sensedConfig[2]))
        else:
            raise ValueError('mode is not set to a valid option')

    def setCmdPos(self, coord):
        """
        Sets the commanded configuration as needed to reach the point [x,y,z] relative to the PPU base frame in inches.
        
        Args:
            coord - cartesian coordinates [x,y,z] in PPU base frame in inches
        """
        config = self._solveIK(coord)
        raw_config = self._commandconversion(config)
        self.setCmdConfig(raw_config)

    ''' Need proper measurement conversion to implement
    def getCmdPos(self):
        """         
        Returns:
            The coordinates [x,y,z] of the PPU tip for the commanded configuration.
        """

    def getSnsPos(self):
        """         
        Returns:
            The coordinates [x,y,z] of the PPU tip for the sensed configuration.
        """
    '''
       
    def setHalt(self,retval='raw'):
        """
        Sets the commanded position to the last sensed position, causing a halt on the next Arduino's
        next message request.
        
        Returns:
            Configuration [cmdPan_Raw, cmdTilt_Raw, cmdExt_Raw] used to stop.
        """
        self.setCmdConfig(self.sensedConfig)
        if retval=='raw':
            return self.sensedConfig
        elif retval=='rad/prct':
            return self.getSnsConfig('rad/prct')
    
    def setPress(self):
        self.mode = PPU.PRESS
    
    def setCmdRad(self, motor, rads):
        """     
        Args:
            motor - string that dictates which motor to set. 'pan' for pan motor, 'tilt' for tilt motor
            rads - radian angle to set comanded positon to
        """
        checkRads(rads)
        if motor == 'pan':
            self.cmdPan_Raw = radToDynaMX64(rads)
        elif motor == 'tilt':
            self.cmdTilt_Raw = radToDynaMX64(rads)
        else:
            raise ValueError('motor is not set to a valid option: "%s"'%(motor,))
        
    def getCmdRad(self, motor):
        """      
        Args:
            motor - string that dictates which motor angle to return. 'pan' for pan motor, 'tilt' for tilt motor
        Returns:
            comanded postion of the pan or tilt motor in radian angles
        """
        if motor == 'pan':
            return dynaMX64ToRad(self.cmdPan_Raw)
        if motor == 'tilt':
            return dynaMX64ToRad(self.cmdTilt_Raw)
        else:
            raise ValueError('motor is not set to a valid option')
    
    def setCmdStrokPrct(self, percent):
        """Sets cmdExt_Raw based on a percent of the max extension length"""
        checkPercent(percent)
        self.cmdExt_Raw = strokePctToFirgelli(percent)
    
    def getCmdStrokPrct(self):
        """Gets the stroke percentage for the current cmdExt_Raw"""
        return firgelliToStrokePct(self.cmdExt_Raw)
    
    def close(self):
        """For clean shutdown, call this before exiting the program"""
        self.ser.close()
        
    def _solveIK(self, coord):
        """
        Returns the degrees for the pan and tilt motor and the inches for the linear actuator needed
        to reach the point [x,y,z].

        Args:
            coord - cartesian coordinates [x,y,z] in PPU base frame in inches
        Returns:
            PPU configuration [q1,q2,q3] to reach the x,y,z coordinates, in degrees and inches where
                q1 is degree angle of pan (base) motor
                q2 is degree angle of tilt motor
                q3 is extension of linear actuator in inches
        """
        xw,yw,zw = coord
        y=yw
        x=xw+PPU.xb
        z=zw-PPU.hb
        q3=vectorops.norm((x,y,z))
        q2=math.degrees(math.asin(-z/q3))
        if q2==90 or q2==-90:
            q1=0 #infinite solutions 
        else:
            q1=math.degrees(math.atan2(y,x))
        if (q1 < PPU.q1LL) or (q1 > PPU.q1UL):
            raise ValueError("q1 is %f, which is not within IK joint limits." %q1)
        elif (q2 < PPU.q2LL) or (q2 > PPU.q2UL):
            raise ValueError("q2 is %f, which is not within IK joint limits." %q2)
        elif (q3 < PPU.q3LL) or (q3 > PPU.q3UL):
            raise ValueError("q3 is %f, which is not within IK joint limits." %q3)
        else:
            return [q1,q2,q3]

    def _solveFK(self, configDI):
        """
        Returns the cartesian coordinates [x,y,z] of the PPU tip in inches, given the PPU configuration
        [q1,q2,q3] in degrees and inches.

        Args:
            configDI - PPU configuration [q1,q2,q3] in degrees and inches where
                q1 is degree angle of pan (base) motor
                q2 is degree angle of tilt motor
                q3 is extension of linear actuator in inches
        Returns:
            Cartesian coordinates [x,y,z] of the PPU tip when in a [q1,q2,q3] configuration.
        """
        q1,q2,q3 = configDI
        if (q1 < PPU.q1LL) or (q1 > PPU.q1UL):
            raise ValueError("Please provide a value from %d to %d for q1." %(PPU.q1LL,PPU.q1UL))
        if (q2 < PPU.q2LL) or (q2 > PPU.q2UL):
            raise ValueError("Please provide a value from %d to %d for q2." %(PPU.q2LL,PPU.q2UL))
        if (q3 < PPU.q3LL) or (q3 > PPU.q3UL):
            raise ValueError("Please provide a value from %d to %d for q3." %(PPU.q3LL,PPU.q3UL))
        q1r=math.radians(q1)
        q2r=math.radians(q2)
        x=math.cos(q1r)*math.cos(q2r)*q3
        y=math.sin(q1r)*math.cos(q2r)*q3
        z=-math.sin(q2r)*q3
        xw=x-PPU.xb
        yw=y
        zw=z+PPU.hb
        return [xw,yw,zw]

    def _commandconversion(self, configDI):
        """ Given PPU configuration [q1,q2,q3] in degrees and inches, returns raw configuration [cmdPan_Raw, cmdTilt_Raw, cmdExt_Raw]"""
        q1,q2,q3 = configDI
        cf=0.08791 # angle/value conversion factor
        pan=(q1+180)/cf
        tilt=(q2+180)/cf
        ext=1000+(1000*(q3-PPU.q3LL))/(PPU.q3UL-PPU.q3LL)
        return [int(round(pan)),int(round(tilt)),int(round(ext))]

    def _processDataIn(self, givenHndlr):
        self.sensedConfig = [givenHndlr.dataIn[0], givenHndlr.dataIn[1], givenHndlr.dataIn[2]]
        self.moving = givenHndlr.dataIn[3]
        self.extVel = givenHndlr.dataIn[4]
        self.extOverload = givenHndlr.dataIn[5]
        
    def _setDataOut(self):
        self.serHndlr.dataOut[0] = self.cmdPan_Raw
        self.serHndlr.dataOut[1] = self.cmdTilt_Raw
        self.serHndlr.dataOut[2] = self.cmdExt_Raw
        self.serHndlr.dataOut[3] = self.stepSize
        self.serHndlr.dataOut[4] = self.mode
        self.serHndlr.dataOut[5] = self.modeParam
    
    # Property checkers
    @property
    def cmdPan_Raw(self):
        return self.__cmdPan_Raw
    @cmdPan_Raw.setter
    def cmdPan_Raw(self, step):
        checkDynaStep(step)
        self.__cmdPan_Raw = step
        
    @property
    def cmdTilt_Raw(self):
        return self.__cmdTilt_Raw
    @cmdTilt_Raw.setter
    def cmdTilt_Raw(self, step):
        checkDynaStep(step)
        self.__cmdTilt_Raw = step
        
    @property
    def cmdExt_Raw(self):
        return self.__cmdExt_Raw
    @cmdExt_Raw.setter
    def cmdExt_Raw(self, pulse_us):
        checkPulse(pulse_us)
        self.__cmdExt_Raw = pulse_us

def mapFromTo(val, inMin, inMax, retMin, retMax):
    """
    Linearly maps val from (inMin, inMax) range to the (retMin, retMax) range. Mapping outside of min and max values is
    allowed and properly handled.
    
    Args:
        val - value to map
        inMin - min of input range
        inMax - max of input range
        retMin - min of output range
        retMax - max of output range
    Returns:
        val mapped from (inMin, inMax) range to the (retMin, retMax) range
    """
    inputRange = inMax - inMin
    returnRange = retMax - retMin
    percent = float(val - inMin) / inputRange
    return retMin + (percent * returnRange)

def radToDynaMX64(rads):
    """
    Takes a radian angle and outputs the nearest raw Dynamixel MX-64 motor step.
    
    Args:
        rads - radian angle
    Returns:
        equivalent motor step
    """
    if rads is None:
        return None
    checkRads(rads)
    return int(round(mapFromTo(rads, pi, -pi, 0, 4095)))

def dynaMX64ToRad(step):
    """
    Takes raw Dynamixel MX-64 motor step and converts it to radian angle.
    
    Args:
        step - motor step
    Returns:
        equivalent radian angle
    """
    if step is None:
        return None
    #checkDynaStep(step)
    return mapFromTo(step, 0, 4095, pi, -pi)

def strokePctToFirgelli(percent):
    """
    Takes stroke percentage and converts it to the nearest micro-second pulse width (for Firgelli linear actuator).
    
    Args:
        percent - stroke percentage
    Returns:
        equivalent micro-second pulse width
    """
    if percent is None:
        return None
    checkPercent(percent)
    return int(round(mapFromTo(percent, 0, 1.0, 1000, 2000)))

def firgelliToStrokePct(pulse_us):
    """
    Takes micro-second pulse width and converts it to stroke percentage (for Firgelli linear actuator).
    
    Args:
        pulse_us - micro-second pulse width
    Returns:
        equivalent stroke percentage
    """
    if pulse_us is None:
        return None
    #checkPulse(pulse_us)
    return mapFromTo(pulse_us, 1000, 2000, 0, 1.0)

def checkRads(rads):
    if rads is None:
        return
    if rads < -pi or rads > pi:
        raise ValueError('rads must be between -pi and pi, but was ' + str(rads))

def checkDynaStep(step):
    if step is None:
        return
    if not isinstance(step, int):
        raise ValueError('step must be an int')
    if step < 0 or step > 4095:
        raise ValueError('step must be between 0 and 4095, but was ' + str(step))

def checkPercent(percent):
    if percent is None:
        return
    if percent < 0 or percent > 1.0:
        raise ValueError('percent must be between 0 and 1.0, but was ' + str(percent))

def checkPulse(pulse_us):
    if pulse_us is None:
        return
    if not isinstance(pulse_us, int):
        raise ValueError('pulse_us must be an int')
    if pulse_us < 1000 or pulse_us > 2000:
        raise ValueError('pulse_us must be between 1000 and 2000, but was ' + str(pulse_us))

