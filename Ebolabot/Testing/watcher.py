from klampt import *
from klampt.glprogram import *
import time
import sys
import os
ebolabot_root = os.getenv("EBOLABOT_PATH",".")
sys.path.append(ebolabot_root)
from Motion import motion
from Motion import config


class MyGLViewer(GLRealtimeProgram):
    def __init__(self,world):
        GLRealtimeProgram.__init__(self,"Viewer")
        self.world = world

        robot = world.robot(0)

    def display(self):
        #Put your display handler here
        #the current example draws the sensed robot in grey and the
        #commanded configurations in transparent green

        #these lines will update the robot and draw the world
        robot = motion.robot
        q = robot.getKlamptSensedPosition()
        self.world.robot(0).setConfig(q)
        GLWidgetProgram.display(self)

        #draw commanded configuration
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
        glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[0,1,0,0.5])
        q = robot.getKlamptCommandedPosition()
        self.world.robot(0).setConfig(q)
        self.world.robot(0).drawGL(False)
        glDisable(GL_BLEND)

    def keyboardfunc(self,c,x,y):
        #Put your keyboard handler here
        #the current example toggles simulation / movie mode
        if c == 'h':
            print 'q: clean quit'
        elif c == 'q':
            motion.robot.shutdown()
            exit(0)
        else:
            GLWidgetProgram.keyboardfunc(self,c,x,y)
            self.refresh()


if __name__ == "__main__":
    config.parse_args()
    print "watcher.py: updates and visualizes the robot. Runs in client mode"
    print

    print "Loading Motion Module model",config.klampt_model
    config.mode = 'client'
    motion.setup(mode=config.mode,klampt_model=config.klampt_model,libpath="./",)
    res = motion.robot.startup()
    if not res:
        print "Error starting up Motion Module"
        exit(1)
    time.sleep(0.1)
    world = WorldModel()
    res = world.readFile(config.klampt_model)
    if not res:
        raise RuntimeError("Unable to load Klamp't model "+config.klampt_model)
    
    viewer = MyGLViewer(world)
    viewer.run()
