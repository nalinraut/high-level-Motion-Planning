from klampt import *
from klampt import vectorops
from klampt.glprogram import *
from klampt.glcommon import GLWidgetPlugin
import math
import time
import sys
import os
ebolabot_root = os.getenv("EBOLABOT_PATH",".")
sys.path.append(ebolabot_root)
from Motion import motion
from Motion import config
sys.path.append(os.path.join(ebolabot_root,"InputDevices/BaxterPuppet"))
from readArduino import *
from filters import *

#########################################################################

class MyGLViewer(GLRealtimeProgram):
    def __init__(self,world):
        GLRealtimeProgram.__init__(self,"My GL program")
        self.world = world
        self.t0 = time.time()
        self.inPosition = False
        self.ra = readArduino()

    def display(self):
        #Put your display handler here
        #the current example draws the sensed robot in grey and the
        #commanded configurations in transparent green
        robot = motion.robot
        #q = robot.getKlamptSensedPosition()
        q = robot.getKlamptCommandedPosition()
        self.world.robot(0).setConfig(q)
        self.world.drawGL()

        #draw commanded configuration
        #glEnable(GL_BLEND)
        #glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
        #glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[0,1,0,0.5])
        #q = robot.getKlamptCommandedPosition()
        #self.world.robot(0).setConfig(q)
        #self.world.robot(0).drawGL(False)
        #glDisable(GL_BLEND)

    def control_loop(self):
        robot = motion.robot
        #Put your control handler here
        #currently moves a joint according to sinusoid
        q = robot.left_limb.commandedPosition()
        r = robot.right_limb.commandedPosition()
        #print "Left commanded",q

        processed_data = self.ra.read()
        assert len(q) >= 6
        assert len(r) >= 6
        assert len(processed_data) >=12
    	
    	#joint commands for right arm
    	q[0] = 0.00388 * processed_data[0] - 2.01026 #-1.7 + processed_data[0] * 3.4/1023
    	q[1] = 0.00408 * processed_data[1] - 2.86122 #-2.15 + processed_data[1] * 3.2/1023
    	q[2] = 0.00395 * processed_data[2] - 2.40745 #-3.1 + processed_data[2] * 6.1/1023
    	q[3] = 0.00395 * processed_data[3] - 1.51872 #-0.05 + processed_data[3] * 2.65/1023
    	q[4] = 0.00367 * processed_data[4] - 2.3802 #-3.05 + processed_data[4] * 6.1/1023
    	q[5] = 0.003747 * processed_data[5] - 1.8035 #-1.6 + processed_data[5] * 3.7/1023
    	#joint commands for left arm
    	r[0] = 0.00365 * processed_data[6] - 1.98829 #-1.7 + processed_data[6] * 3.4/1023
    	r[1] = 0.003744 * processed_data[7] - 2.0879 #-2.15 + processed_data[7] * 3.2/1023
    	r[2] = 0.003713 * processed_data[8] - 2.85335 #-3.1 + processed_data[8] * 6.1/1023
    	r[3] = 0.00416 * processed_data[9] - 1.86292 #-0.05 + processed_data[9] * 2.65/1023
    	r[4] = 0.003949 * processed_data[10] - 3.30957 #-3.05 + processed_data[10] * 6.1/1023
    	r[5] = 0.00397 * processed_data[11] - 1.93171 #-1.6 + processed_data[11] * 3.7/1023
	
        robot.left_limb.positionCommand(q)
    	robot.right_limb.positionCommand(r)
        return

    def idle(self):
        self.control_loop()
        glutPostRedisplay()

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
        glutPostRedisplay()



if __name__ == "__main__":
    print "puppet_test.py: tests the RoboPuppet with the motion module"
    print "Press q to exit."
    print

    config.setup()
    res = motion.robot.startup()
    if not res:
        print "Error starting up APC Motion"
        exit(1)
    time.sleep(0.1)
    world = WorldModel()
    res = world.readFile(config.klampt_model)
    if not res:
        raise RuntimeError("Unable to load APC Motion model "+fn)
    
    viewer = MyGLViewer(world)
    viewer.run()
