"""
Connects to and returns readings from any Logitech controller
(or controllers using the same button mappings)

Peter Moran, July 2015

SETUP:
1. Create a Joystick object to connect to the joystick. If only one controller is connected, the ID should be 0.
2. Call updateState() to read and update the current controller configuration.
    - Make sure to call this regularly, such as at the beginning of a loop.
3. Access the button and analog states from the last update via methods below.

"""

import pygame  # Joystick interface

# Set up pygame
pygame.display.init()
pygame.joystick.init()

# Button id's
A = 0
B = 1
X = 2
Y = 3
LB = 4
RB = 5
BACK = 6
START = 7
L_STICK_CLICK = 9
R_STICK_CLICK = 10

# Axis id's
LX = 0
LY = 1
RX = 3
RY = 4
PAD_X = 6
PAD_Y = 7
LT = 2
RT = 5


def numJoys():
    """Returns the number of recognized joysticks plugged into the computer"""
    return pygame.joystick.get_count()

class Joystick:
    """Provides easy access to joystick button and analog values"""
    analgdeadzone = 0.1

    def __init__(self, id):
        """Connects to Joystick with specified id"""
        self.joy = pygame.joystick.Joystick(id)
        self.joy.init()

    def updateState(self):
        """Updates the current state of the controller"""
        pygame.event.pump()

    def get_A(self):
        return self.joy.get_button(A)

    def get_B(self):
        return self.joy.get_button(B)

    def get_X(self):
        return self.joy.get_button(X)

    def get_Y(self):
        return self.joy.get_button(Y)

    def get_LB(self):
        return self.joy.get_button(LB)

    def get_RB(self):
        return self.joy.get_button(RB)

    def get_Back(self):
        return self.joy.get_button(BACK)

    def get_Start(self):
        return self.joy.get_button(START)

    def get_L3(self):
        return self.joy.get_button(L_STICK_CLICK)

    def get_R3(self):
        return self.joy.get_button(R_STICK_CLICK)

    def get_stick_L(self):
        """Returns [x, y] value of left joystick, ranging 0 to 1. Value will be 0 inside deadzone"""
        pos = [self.joy.get_axis(LX), self.joy.get_axis(LY)]
        for axis in pos:
            if axis < self.analgdeadzone:
                axis = 0
        return pos

    def get_stick_R(self):
        """Returns [x, y] value of right joystick, ranging 0 to 1. Value will be 0 inside deadzone"""
        pos = [self.joy.get_axis(RX), self.joy.get_axis(RY)]
        for axis in pos:
            if axis < self.analgdeadzone:
                axis = 0
        return pos

    def get_dpad(self):
        return [self.joy.get_axis(PAD_X), self.joy.get_axis(PAD_Y)]

    def get_LT(self):
        """Returns how much the left trigger is pressed (range 0 to 1)"""
        return self.joy.get_axis(LT)

    def get_RT(self):
        """Returns how much the right trigger is pressed (range 0 to 1)"""
        return self.joy.get_axis(RT)