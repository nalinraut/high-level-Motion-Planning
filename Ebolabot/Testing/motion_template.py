import sys
import os
ebolabot_root = os.getenv("EBOLABOT_PATH",".")
sys.path.append(ebolabot_root)
from Motion import motion
from Motion import config
from klampt import *
from klampt.glprogram import *
import math
import time

class MyGLViewer(GLRealtimeProgram):
    def __init__(self,world):
        GLRealtimeProgram.__init__(self,"My GL program")
        self.world = world
        self.t0 = time.time()
        self.inPosition = False

    def display(self):
        #Put your display handler here
        #the current example draws the sensed robot in grey and the
        #commanded configurations in transparent green
        robot = motion.robot
        q = robot.getKlamptSensedPosition()
        self.world.robot(0).setConfig(q)
        self.world.drawGL()

        #draw commanded configuration
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
        glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[0,1,0,0.5])
        q = robot.getKlamptCommandedPosition()
        self.world.robot(0).setConfig(q)
        self.world.robot(0).drawGL(False)
        glDisable(GL_BLEND)

    def control_loop(self):
        robot = motion.robot
        #Put your control handler here
        #currently moves a joint according to sinusoid
        #q = robot.left_limb.commandedPosition()
        #print "Left commanded",q
        #q[1] = math.sin(time.time() )
        #robot.left_limb.positionCommand(q)
        t = time.time()-self.t0
        if robot.left_mq.moving():
            pass
        elif not self.inPosition:
            q = robot.left_limb.commandedPosition()
            q[1] = -1.0;
            q[3] = 2.0;
            q[5] = 1.0;
            print "Sent setRamp command"
            robot.left_mq.setRamp(q)
            self.inPosition = True
        else:
            # robot.left_ee.velocityCommand([0,0,0],[math.sin(t)*0.2,math.cos(t)*0.2,0])
            robot.left_ee.driveCommand([0,0,0],[math.sin(t)*0.2,math.cos(t)*0.2,0])
        return

    def idle(self):
        self.control_loop()
        self.refresh()

    def mousefunc(self,button,state,x,y):
        #Put your mouse handler here
        print "mouse",button,state,x,y
        GLRealtimeProgram.mousefunc(self,button,state,x,y)

    def motionfunc(self,x,y):
        #Put your mouse motion handler here
        GLRealtimeProgram.motionfunc(self,x,y)

    def specialfunc(self,c,x,y):
        #Put your keyboard special character handler here
        print c,"pressed"

    def keyboardfunc(self,c,x,y):
        #Put your keyboard handler here
        #the current example toggles simulation / movie mode
        print c,"pressed"
        if c == 'q':
            motion.robot.shutdown()
            exit(0)
        self.refresh()



if __name__ == "__main__":
    config.parse_args()
    print "motion_template.py: sets up a visualizer for testing Motion"
    print "control loops.  Press q to exit."
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
        raise RuntimeError("Unable to load Klamp't model "+config.klampt_model)
    
    viewer = MyGLViewer(world)
    viewer.run()
    motion.robot.shutdown()
