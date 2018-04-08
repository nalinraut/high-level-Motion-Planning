"""
Controls Baxter using any game pad / joystick via the logitech and Motion modules.
Designed for Logitech controllers, may work for other controllers too.
Peter Moran, July 2015

SETUP:
Before running this script, you must connect to Baxter and the reflex grippers.

CONTROLS:
Controls are modified when special control buttons are pressed to create combinations, noted below.
Always:
    Right trigger -- fully closes right gripper when pressed
No modifier (End effector velocity mode, ie translations):
    Right stick x -- move end effector along Baxter's y axis (ie side to side)
    Right stick y -- move end effector along Baxter's x axis (ie outward)
    Left stick y -- move end effector along Baxter's z axis (ie vertically)
Left bumper (End effector rotational velocity mode):
    Right stick x -- rotate end effector about its x axis
    Right stick y -- rotate end effector about its y axis
    Left stick x -- rotate end effector about its z axis
Right bumper (Joint velocity mode):
    Right stick x -- rotate gripper
    Right stick y -- bend wrist
    Left stick x -- rotate elbow
    Left stick y -- bend elbow

"""

# Import Modules
import sys
import os
ebolabot_root = "../"
sys.path.append(ebolabot_root)
sys.path.append(os.path.join(ebolabot_root,"InputDevices/GamePad")
from Motion import motion
from Motion import config
import logitech

# Connect to controller
try:
    j = logitech.Joystick(0)
except:
    print "Joystick not found"
    print "Note: Pygame reports there are " + str(logitech.numJoys()) + " joysticks"
    exit()

# Connect to Baxter
robot = motion.setup(mode=config.mode,klampt_model=config.klampt_model,libpath="./")
res = robot.startup()
if not res:
    print "Error connecting to robot"
    exit()

print "Moving to neutral configuration"
q = robot.left_limb.commandedPosition()
q[1] = -1.0
q[3] = 2.0
q[5] = 1.0
robot.left_mq.appendLinearRamp(q)
q = robot.right_limb.commandedPosition()
q[1] = -1.0
q[3] = 2.0
q[5] = 1.0
robot.right_mq.setRamp(q)

while robot.right_mq.moving() or robot.left_mq.moving():
    pass

while (True):
    # Read controller
    j.updateState()
    rstick = j.get_stick_R()
    lstick = j.get_stick_L()
    LB = j.get_LB()
    RB = j.get_RB()
    RT = j.get_RT()

    # Update velocities
    if RT > 0.1:
        robot.right_gripper.close()
    else:
        robot.right_gripper.open()
    if RB:  # Joint velocity mode
        jVel = [0, 0, 0, 0, 0, 0, 0]
        jVel[6] = rstick[0] / 5  # wrist 2
        jVel[5] = rstick[1] / 5  # wrist 1
        jVel[4] = lstick[0] / 5  # wrist 0
        jVel[3] = lstick[1] / 5  # elbow 1
        motion.robot.left_limb.velocityCommand(jVel)
    elif LB:  # End effector angular velocity mode
        driveVel = [0, 0, 0]
        driveAngVel = [0, 0, 0]
        driveAngVel[0] = -rstick[0] / 10  # about end effector x
        driveAngVel[1] = rstick[1] / 10   # about end effector y
        driveAngVel[2] = lstick[0] / 10   # about end effector z
        robot.left_ee.driveCommand(driveAngVel, driveVel)
    else:  # End effector velocity mode
        driveVel = [0, 0, 0]
        driveAngVel = [0, 0, 0]
        driveVel[1] = -rstick[0] / 10  # y
        driveVel[0] = -rstick[1] / 10  # x
        driveVel[2] = -lstick[1] / 10  # z
        robot.left_ee.driveCommand(driveAngVel, driveVel)
