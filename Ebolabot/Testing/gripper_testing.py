import sys
sys.path.append(".")
from Motion import motion
from Motion import config
from klampt import *
from klampt.glprogram import *
import math


class GripperTestUIViewer(GLRealtimeProgram):
    def __init__(self,world):
        GLRealtimeProgram.__init__(self,"Ebolabot gripper test")
        self.world = world
        self.gripper = 'l'
        self.presets = [[1,1,1,1],[0.5,0.5,0.5,1],[0,0,0,1],[1,1,1,0],[0,0,1,0],[1,1,1,0.5],[0,0,0,0.5]]
        self.force = 1
        self.speed = 1
        self.current_preset = 0
        print motion.robot.left_gripper.type()
        print motion.robot.right_gripper.type()

    def display(self):
        q = motion.robot.getKlamptSensedPosition()
        self.world.robot(0).setConfig(q)
        self.world.drawGL()

    def display_screen(self):
        pass

    def keyboardfunc(self,c,x,y):
        robot = motion.robot
        if c=='h':
            print "Help:"
            print "  h: print this message"
            print "  q: shutdown and exit"
            print "  s: stop motion"
            print "  l/r: switch to left or right grippers"
            print "  o/c: open or close the selected gripper"
            print "  f/F: set low or high force"
            print "  v/V: set low or high velocity"
            print "  n: go to the next preset configuration"
        elif c=='q':
            robot.shutdown()
            print "Shutdown completed"
            exit(0)
        elif c=='s':
            robot.stopMotion()
        elif c=='l':
            self.gripper = 'l'
        elif c=='r':
            self.gripper = 'r'
        elif c=='o':
            if self.gripper == 'l':
                robot.left_gripper.open()
            else:
                robot.right_gripper.open()
        elif c=='c':
            if self.gripper == 'l':
                robot.left_gripper.close()
            else:
                robot.right_gripper.close()
        elif c=='f':
            self.force = 0.1
        elif c=='F':
            self.force = 1
        elif c=='v':
            self.speed = 0.1
        elif c=='V':
            self.speed = 1
        elif c=='n':
            if self.gripper == 'l':
                robot.left_gripper.command(self.presets[self.current_preset],[self.speed]*4,[self.force]*4)
            else:
                robot.right_gripper.command(self.presets[self.current_preset],[self.speed]*4,[self.force]*4)
            self.current_preset = (self.current_preset+1) % len(self.presets)

if __name__ == '__main__':
    config.parse_args()
    print "motion_testing.py: various random tests of the motion module."""
    print "Press q to exit."
    print

    print "Loading Motion Module model",config.klampt_model
    motion.setup(mode=config.mode,klampt_model=config.klampt_model,libpath="./")
    print "Running startup"
    res = motion.robot.startup()
    if not res:
        print "Error starting up Motion Module"
        exit(1)
    time.sleep(0.1)
    world = WorldModel()
    res = world.readFile(config.klampt_model)
    if not res:
        raise RuntimeError("Unable to load Klamp't model "+fn)
    program = GripperTestUIViewer(world)
    program.run()
    motion.robot.shutdown()    
    

