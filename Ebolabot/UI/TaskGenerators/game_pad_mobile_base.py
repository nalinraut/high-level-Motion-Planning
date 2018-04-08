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
    Y -- safe toggle
    A -- tuck
    B -- neutral
    X -- original
"""

# Import Modules
import os
import sys
ebolabot_root = os.getenv("EBOLABOT_PATH",".")
sys.path.append(ebolabot_root)
#for logitech module
sys.path.append(os.path.join(ebolabot_root,'InputDevices/GamePad'))
import logitech
from task_generator import TaskGenerator
from klampt.io import resource

#set this -1 for view-centric control, looking at the face of the robot
viewToWorldScaleXY = 1

class GamePadBaseDriveTaskGenerator(TaskGenerator):
    def __init__(self):
        self.j = None
        self.lastState = {}

    def name(self): return "GamePadBaseDrive"

    def init(self,world):
        assert self.j == None,"Init may only be called once"
        self.world = world
        self.originalConfig = None
        self.tuckConfig = resource.get("tuck.config",directory="UI/Resources",doedit=False)
        self.neutralConfig = resource.get("neutral.config",directory="UI/Resources",doedit=False)
        left_arm_link_names = ['left_upper_shoulder','left_lower_shoulder','left_upper_elbow','left_lower_elbow','left_upper_forearm','left_lower_forearm','left_wrist']
        right_arm_link_names = ['right_upper_shoulder','right_lower_shoulder','right_upper_elbow','right_lower_elbow','right_upper_forearm','right_lower_forearm','right_wrist']
        robot = world.robot(0)
        self.left_arm_link_indices = [robot.link(l).index for l in left_arm_link_names]
        self.right_arm_link_indices = [robot.link(l).index for l in right_arm_link_names]
        # Connect to controller
        try:
            self.j = logitech.Joystick(0)
            return True
        except:
            print "Joystick not found"
            print "Note: Pygame reports there are " + str(logitech.numJoys()) + " joysticks"
            return False

    def start(self):
        if not self.j: return False
        self._status = 'ok'
        q = self.world.robot(0).getConfig()
        if self.originalConfig == None:
            if all(q[i] == self.tuckConfig[i] for i in self.left_arm_link_indices+self.right_arm_link_indices):
                pass
            elif all(q[i] == self.neutralConfig[i] for i in self.left_arm_link_indices+self.right_arm_link_indices):
                pass
            else:
                #remember original configuration
                self.originalConfig = q
        return True

    def status(self):
        if self.j:
            return 'ok'
        else:
            return 'error'

    def messages(self):
        return []

    def stop(self):
        self._status=''

    def get(self):
        # Read controller
        j = self.j
        j.updateState()
        rstick = j.get_stick_R()
        lstick = j.get_stick_L()
        LB = j.get_LB()
        RB = j.get_RB()
        LT = j.get_LT()
        RT = j.get_RT()
        Y = j.get_Y()	# For safe control

        if j.get_A() and self.tuckConfig is not None:
            return self.setArmsMsg(self.tuckConfig)
        if j.get_B() and self.neutralConfig is not None:
            return self.setArmsMsg(self.neutralConfig)
        if j.get_X() and self.originalConfig is not None:
            return self.setArmsMsg(self.originalConfig)       

        state = {}
        state['rstick'] = rstick
        state['lstick'] = lstick
        state['LB'] = LB
        state['RB'] = RB
        state['LT'] = LT
        state['RT'] = RT
        state['Y'] = Y

        if len(self.lastState) > 0:
            res = self.do_logic(self.lastState,state)
        else:
            res = None
            
        self.lastState = state
        return res

    def setArmsMsg(self,q):
        """returns a message that sends the robot to a desired configuration"""
        msg = {}
        msg['type'] = 'JointPose'
        msg['parts'] = ["left","right"]
        msg['positions'] = [[q[i] for i in self.left_arm_link_indices],
                            [q[i] for i in self.right_arm_link_indices]]
        msg['speed'] = 1
        msg['safe'] = 0
        return msg


    def do_logic(self,lastState,state):
        rstick = state['rstick']
        lstick = state['lstick']
        y = state['Y']

        return {'type':'BaseVelocity','velocity':[-viewToWorldScaleXY*float(lstick[1])/5,-viewToWorldScaleXY*float(lstick[0])/5,-float(rstick[0])/5], 'safe':y}


def make():
    return GamePadBaseDriveTaskGenerator()
