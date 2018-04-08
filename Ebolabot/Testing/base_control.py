from klampt import *
from klampt.glprogram import *
import math
import time
import sys
import os
ebolabot_root = os.getenv("EBOLABOT_PATH",".")
sys.path.append(ebolabot_root)
from Motion import motion
from Motion import config

ebolabot_root = os.getenv("EBOLABOT_PATH",".")
sys.path.append(ebolabot_root)

from Common.system_config import EbolabotSystemConfig
import threading


#########################################################################

class MyBaseDriveProgram(GLRealtimeProgram):
    """Used for driving around the base and testing the Motion base control
    functionality"""
    def __init__(self,world,name="My GL Widget Program"):
        GLRealtimeProgram.__init__(self,name)
        self.world = world
        self.world.robot(0).setConfig(motion.robot.getKlamptCommandedPosition())
        self.command = [0,0,0]
        self.commandType = 'rv'
        self.increment = 0.01
        self.incrementang = 0.02

    def display(self):
        #Put your display handler here
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

        #draw command / target
        if self.commandType == 'op' or self.commandType == 'rp':
            tgt = motion.robot.base.odometryTarget()
            cmd = self.command
            if self.commandType == 'rp':
                pos = motion.robot.base.odometryPosition()
                c = math.cos(pos[2])
                s = math.sin(pos[2])
                cmdnew = [pos[0]+c*cmd[0]-s*cmd[1],pos[1]+s*cmd[0]+c*cmd[1],pos[2]+cmd[2]]
                cmd = cmdnew
            glDisable(GL_DEPTH_TEST)
            glDisable(GL_LIGHTING)
            glPointSize(7.0)
            glEnable(GL_POINT_SMOOTH)
            glColor3f(0,0.5,1)
            glBegin(GL_POINTS)
            glVertex3f(cmd[0],cmd[1],0)
            glEnd()
            glBegin(GL_LINES)
            glVertex3f(cmd[0],cmd[1],0)
            glVertex3f(cmd[0]+math.cos(cmd[2])*0.4,cmd[1]+math.sin(cmd[2])*0.4,0)
            glEnd()

            if tgt:
                glColor3f(0.5,0,1)
                glBegin(GL_POINTS)
                glVertex3f(tgt[0],tgt[1],0)
                glEnd()
                glBegin(GL_LINES)
                glVertex3f(tgt[0],tgt[1],0)
                glVertex3f(tgt[0]+math.cos(tgt[2])*0.4,tgt[1]+math.sin(tgt[2])*0.4,0)
                glEnd()
            glEnable(GL_DEPTH_TEST)

    def specialfunc(self,c,x,y):
        #Put your keyboard special character handler here
        print c,"pressed"
        if c == GLUT_KEY_LEFT:
            self.command[1] += self.increment
        elif c == GLUT_KEY_RIGHT:
            self.command[1] -= self.increment
        elif c == GLUT_KEY_UP:
            self.command[0] += self.increment
        elif c == GLUT_KEY_DOWN:
            self.command[0] -= self.increment
        self.updateCommand()
        self.refresh()

    def updateCommand(self):
        if self.commandType == 'rv':
            self.sendCommand()
    def sendCommand(self):
        if self.commandType == 'rv':
            motion.robot.base.moveVelocity(*self.command)
        elif self.commandType == 'rp':
            motion.robot.base.movePosition(*self.command)
        elif self.commandType == 'op':
            motion.robot.base.moveOdometryPosition(*self.command)

    def keyboardfunc(self,c,x,y):
        #Put your keyboard handler here
        #the current example toggles simulation / movie mode
        print c,"pressed"
        c = c.lower()
        if c=='h':
            self.print_help()
        elif c=='s':
            self.command = [0,0,0]
            motion.robot.base.stop()
        elif c==' ':
            self.sendCommand()
        elif c==',' or c=='<':
            self.command[2] += self.incrementang
            self.updateCommand()
        elif c=='.' or c=='>':
            self.command[2] -= self.incrementang
            self.updateCommand()
        elif c=='v':
            print "Switching to velocity mode"
            self.command = [0,0,0]
            self.commandType = 'rv'
        elif c=='r':
            print "Switching to relative position mode"
            self.command = [0,0,0]
            self.commandType = 'rp'
        elif c=='o':
            print "Switching to odometry position mode"
            self.command = list(motion.robot.base.odometryPosition())
            self.commandType = 'op'
        elif c=='q':
            motion.robot.shutdown()
            exit()
        self.refresh()

    def idle(self):
        GLRealtimeProgram.idle(self)
    
    def print_help(self):
        print "Press h to print this message"
        print "Press up/down arrow to move forward/backward arm"
        print "Press left/right arrow to move left/right"
        print "Press </> to rotate left/right"
        print "Press o to switch to odometry position command mode"
        print "Press r to switch to relative position command mode"
        print "Press v to switch to velocity command mode"        
        print "Press [space] to send a position command"
        print "Press s to stop motion"
        print "Press q to exit."
        print


if __name__ == "__main__":
    config.parse_args()
    print "base_control.py: allows driving a base with the Motion module"
    print
    print "Loading Motion Module model",config.klampt_model
    motion.setup(mode=config.mode,klampt_model=config.klampt_model,libpath="./",)
    res = motion.robot.startup()
    if not res:
        print "Error starting up Motion Module"
        exit(1)
    time.sleep(0.01)
    if not motion.robot.base.enabled():
        print "Base is not enabled..."
        exit(1)
    world = WorldModel()
    res = world.readFile(config.klampt_model)
    if not res:
        raise RuntimeError("Unable to load Klamp't model "+config.klampt_model)
  
    viewer = MyBaseDriveProgram(world)
    viewer.print_help()
    viewer.run()
    motion.robot.shutdown()
