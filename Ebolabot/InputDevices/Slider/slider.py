__author__ = 'Peter Moran'
"""
Serial interface for communicating with one or more RoboSlider devices. Provides low level
interface with raw analog values along with methods for mapping input and outputs to any
desired range.

Peter Moran, August 2015
"""
from serial import Serial
from serial import SerialException
from time import sleep
from struct import unpack  # used for parsing binary data

ARD_INT = 2  # bytes in an int for Arduino
CAL_LOW = 100  # max value accepted while calibrating low
CAL_HIGH = 900  # minimum value excepted while calibrating high
CAL_PAUSE = 0.25  # pause in between calibrating high and low
POS_MASK = 1023  # binary mask for extracting position int
TOUCH_MASK = 32768  # binary mask for extracting position boolean


class Board:
    """
    Top level class for interfacing with a RoboSlider device. Connects over serial
    and provides access to each individual analog slider and button.

    Instance variables:
        sliders -- an array of the Board's sliders, as Pot objects
        buttons -- an array of the Board's buttons, as Button objects
    """
    def __init__(self, port=None, numSliders=7, numButtons=3):
        """Create Board object and connect to it over a serial port."""
        self.sliders = [Pot() for _ in range(numSliders)]
        self.buttons = [Button() for _ in range(numButtons)]
        self.forceFollow = False

        if port:
            while True:
                try:
                    self.com = Serial(port, 115200)
                    break
                except SerialException:
                    print 'Error: No device is on port' + port
                    print 'Type an alternative port name'
                    port = input('Port = ')
        else:
            for i in range(11):
                if i >= 11:
                    print 'Error: could not find slider device'
                    exit()
                try:
                    self.com = Serial("/dev/ttyACM"+str(i), 115200)
                    break
                except SerialException:
                    pass

        self.readBoard()

    def readBoard(self):
        """Update the board's slider and button objects to real world readings."""
        bytes = self.com.readline()
        expectedlen = (self.numSliders() + 1) * ARD_INT + 1
        if len(bytes) == expectedlen:
            ints = []
            for i in range(0, len(bytes) - 1, ARD_INT):
                ints.append(unpack('>h', bytes[i:i + ARD_INT])[0])
            for j in range(self.numSliders()):
                self.sliders[j].setPosition(ints[j] & POS_MASK)
                self.sliders[j]._touched = (ints[j] & TOUCH_MASK) != 0
            for (i, button) in enumerate(self.buttons):
                button.pressed = (ints[self.numSliders()] & (1 << i)) != 0
        else:
            try:
                self.readBoard()
            except RuntimeError:
                print "Fatal Error: State could not be found within acceptable time limits"
                exit()

    def sendCommands(self):
        """Send commanded positions and foceFollow option over serial."""
        msg = '<'
        for i in range(self.numSliders()):
            msg += str(self.sliders[i].cmdpos_raw())
            msg += ','
        msg += str(int(self.forceFollow))
        msg += '>'
        self.com.write(msg)
        return msg

    def sldrPositions(self, mode="mapped"):
        """Return list of the slider positions, according to last recorded state.

        Keyword arguments:
        mode -- "mapped" return floats mapped to slider specified range (default),
                "raw" return raw analog values
        """
        retval = []
        if mode == "raw":
            for slider in self.sliders:
                retval.append(slider.position_raw())
        elif mode == "mapped":
            for slider in self.sliders:
                retval.append(slider.position_mapped())
        return retval

    def buttonStates(self, mode="released"):
        retval = [False] * self.numButtons()
        for (i, button) in enumerate(self.buttons):
            if mode == "released":
                retval[i] = button.released()
            if mode == "pressed":
                retval[i] = button.pressed()
        return retval

    def numSliders(self):
        return len(self.sliders)

    def numButtons(self):
        return len(self.buttons)

    # TODO: Debug and fix calibrate command
    def calibrate(self, mode="both"):
        """Finds the true physical range of each individual slider and updates sliders with them.

        Arguments:
        mode -- "both" calibrates min and max (default),
                "low" calibrates the minimums,
                "high" calibrates the maximums.
        """
        if mode == "both":
            self.calibrate("low")
            sleep(CAL_PAUSE)
            self.calibrate("high")
        self.readBoard()
        positions = self.sldrPositions("raw")
        if mode == "low":
            print "Calibrating low... "
            for slider in self.sliders:
                slider.raw_cmdpos = 0
            while any((slider.position_raw() > CAL_LOW and slider.position_raw() != 0) for slider in self.sliders) or any(slider.position_raw() < 0 for slider in self.sliders):
                self.sendCommands()
                self.readBoard()
            print "\t done"
        if mode == "high":
            print "Calibrating high... "
            for slider in self.sliders:
                slider.raw_cmdpos = 1000
            while any((slider.position_raw() < CAL_HIGH and slider.position_raw() != 0) for slider in self.sliders) or any(slider.position_raw() < 0 for slider in self.sliders):
                self.sendCommands()
                self.readBoard()
            print "\t done"
        sleep(CAL_PAUSE)
        for slider in self.sliders:
            if mode == "low":
                slider.physMin = slider.position_mapped()
            if mode == "high":
                slider.physMax = slider.position_mapped()


class Pot:
    """Holds the physical state of each slider and converts between mapped positions and raw analog readings."""
    def __init__(self, mapMin=0, mapMax=1000):
        # Configurations
        self.mapMin = mapMin
        self.mapMax = mapMax
        self.physMin = 0
        self.physMax = 1023

        # State
        self._position_raw = None
        self._position_mapped = None
        self._cmdpos_raw = None
        self._cmdpos_mapped = None
        self._touched = None

    def setPosition(self, rawPos):
        """Given a raw position, updates both the raw position and mapped position states."""
        self._position_raw = rawPos
        self._position_mapped = self.__rawToMap(rawPos)

    def position_mapped(self):
        """Get the last recorded position in terms of the mapping."""
        return self._position_mapped

    def position_raw(self):
        """Get the last recorded analog position of the slider."""
        return self._position_raw

    def setCmdpos_mapped(self, target):
        """Set a desired slider position in terms of mapped values."""
        if target < self.mapMin:
            self._cmdpos_mapped = self.mapMin
            self._cmdpos_raw = self.physMin
        elif target > self.mapMax:
            self._cmdpos_mapped = self.mapMax
            self._cmdpos_raw = self.physMax
        else:
            self._cmdpos_mapped = target
            self._cmdpos_raw = self.__mapToRaw(target)

    def setCmdpos_raw(self, target):
        """Set a desired slider position in terms of analog values."""
        if target < self.physMin:
            self._cmdpos_mapped = self.mapMin
            self._cmdpos_raw = self.physMin
        elif target > self.physMax:
            self._cmdpos_mapped = self.mapMax
            self._cmdpos_raw = self.physMax
        else:
            self._cmdpos_mapped = self.__rawToMap(target)
            self._cmdpos_raw = target

    def cmdpos_mapped(self):
        """The desired position for the slider in terms of mapped values."""
        return self._cmdpos_mapped

    def cmdpos_raw(self):
        """The desired position for the slider in terms of analog values."""
        return self._cmdpos_raw

    def touched(self):
        """Returns true when the slider is touched."""
        return self._touched

    def __mapToRaw(self, pos_map):
        """Convert a mapped position to a raw position"""
        return int(self.__map(pos_map, self.mapMin, self.mapMax, self.physMin, self.physMax))

    def __rawToMap(self, pos_raw):
        """Convert a raw position to a mapped position"""
        return self.__map(pos_raw, self.physMin, self.physMax, self.mapMin, self.mapMax)

    def __map(self, val, inMin, inMax, retMin, retMax):
        """Map val from one range to a new range"""
        inputRange = inMax - inMin
        returnRange = retMax - retMin
        percent = float(val - inMin) / inputRange
        return retMin + (percent * returnRange)


class Button:
    """Keeps track of button state and history.

    Methods:
    pressed() -- True if the button is pressed down / actuated..
    released() -- True if button was just released.
    setPressed(pressed) -- Update the state of the button.
    """
    def __init__(self):
        self._pressed = False
        self._wasPressed = False

    def pressed(self):
        """True if the button is pressed down / actuated."""
        return self._pressed

    def released(self):
        """True if button was just released."""
        return not self._pressed and self._wasPressed

    def setPressed(self, pressed):
        """Update the state of the button to True or False."""
        self._wasPressed = self._pressed
        self._pressed = pressed


if __name__ == "__main__":
    b = Board()

    baxter_joint_mins = [-1.7017, -2.147, -3.0542,  -.05, -3.059, -1.5708, -3.059]
    baxter_joint_maxs = [ 1.7017,  1.047,  3.0542, 2.618,  3.059,   2.094,  3.059]
    for i in range(7):
        b.sliders[i].mapMin = baxter_joint_mins[i]
        b.sliders[i].mapMax = baxter_joint_maxs[i]
    b.readBoard()

    for i in range(7):
        b.sliders[i].setCmdpos_mapped(1)

    b.sendCommands()