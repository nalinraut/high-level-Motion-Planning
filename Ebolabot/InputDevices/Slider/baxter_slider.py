__author__ = 'Peter Moran'
"""
Enables to RoboSlider to control the joint angles of Baxter and keeps the RoboSlider device
and Baxter in sync with each other, allowing for the RoboSlider to reflect any changes
externally caused changes in Baxter's joint angles.

Peter Moran, August 2015.
"""

import sys
ebolabot_root = os.getenv("EBOLABOT_PATH","/home/motion/iml-internal/Ebolabot")
sys.path.append(ebolabot_root)
sys.path.append(os.path.join(ebolabot_root,"InputDevices/Slider")
from Motion import motion
from Motion import config
import slider

def configBoardForBaxter(board):
    baxter_joint_mins = [-1.7017, -2.147, -3.0542,  -.05, -3.059, -1.5708, -3.059]
    baxter_joint_maxs = [ 1.7017,  1.047,  3.0542, 2.618,  3.059,   2.094,  3.059]
    for i in range(motion.numLimbDofs):
        board.sliders[i].mapMin = baxter_joint_mins[i]
        board.sliders[i].mapMax = baxter_joint_maxs[i]

def moveSlidersToJoints(board, limb):
    jointAngles = limb.sensedPosition()
    for i in range(motion.numLimbDofs):
        board.sliders[i].setCmdpos_mapped(jointAngles[i])
    board.sendCommands()

if __name__ == "__main__":
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

    print "Connecting to slider"
    b = slider.Board()
    print "Configuring board"
    configBoardForBaxter(b)

    print "Moving slider with Baxter"
    while True:
        #b.readBoard()
        moveSlidersToJoints(b, robot.right_limb)
