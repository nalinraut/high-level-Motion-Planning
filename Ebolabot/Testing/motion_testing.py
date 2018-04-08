import sys
sys.path.append(".")
from Motion import motion
from Motion import config
from klampt import *
from klampt import so3,se3,vectorops
from klampt.glprogram import *
from klampt import gldraw
import copy
import math


class MotionTestUIViewer(GLRealtimeProgram):
    def __init__(self,world):
        GLRealtimeProgram.__init__(self,"Ebolabot motion test")
        self.world = world

    def display(self):
        q = motion.robot.getKlamptSensedPosition()
        self.world.robot(0).setConfig(q)
        self.world.drawGL()

    def display_screen(self):
        pass

    def keyboardfunc(self,c,x,y):
        if c=='h':
            print "Help:"
            print "  h: print this message"
            print "  q: shutdown and exit"
            print "  s: stop motion"
            print "  v: move on a constant velocity curve"
            print "  m: move via the motion queue"
        elif c=='q':
            motion.robot.shutdown()
            print "Shutdown completed"
            exit(0)
        elif c=='v':
            v = [0.0]*7
            v[1] = 1.0
            motion.robot.left_limb.velocityCommand(v);
        elif c=='m':
            v = [0.0]*7
            v[1] = 1.0
            motion.robot.left_mq.setRamp(v,0.5);
            v[2] = -1.5
            v[3] = 2.0
            motion.robot.left_mq.appendCubic(2.0,v,[0.0]*7);
        elif c=='s':
            motion.robot.stopMotion();


if __name__ == '__main__':
    config.parse_args()
    print "motion_testing.py: various random tests of the motion module."""
    print "Press q to exit."
    print

    print "Loading Motion Module model",config.klampt_model
    motion.setup(mode=config.mode,klampt_model=config.klampt_model,libpath="./")
    res = motion.robot.startup()
    if not res:
        print "Error starting up Motion Module"
        exit(1)
    time.sleep(0.1)
    world = WorldModel()
    res = world.readFile(config.klampt_model)
    if not res:
        raise RuntimeError("Unable to load Klamp't model "+fn)
    program = MotionTestUIViewer(world)
    program.run()
    motion.robot.shutdown()    
    

