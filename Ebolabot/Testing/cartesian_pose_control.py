from klampt import *
from klampt import vectorops,so3,se3
from klampt.glprogram import *
from klampt import gldraw
import math
import time
import sys
import os
ebolabot_root = os.getenv("EBOLABOT_PATH",".")
sys.path.append(ebolabot_root)
from Motion import motion
from Motion import config


def angle(a,b):
    anorm = vectorops.norm(a)
    bnorm = vectorops.norm(b)
    if anorm < 1e-6 or bnorm < 1e-6: return 0
    dp = vectorops.dot(vectorops.div(a,anorm),vectorops.div(b,bnorm))
    dp = max(min(dp,1),-1)
    return math.acos(dp)

class MyEEPoseProgram(GLRealtimeProgram):
    """A class for driving around end effectors using the keyboard.  Used
    primarily for demonstrating the end effector moveto function and for
    testing the Motion library.
    """
    def __init__(self,world,name="My GL Widget Program"):
        GLRealtimeProgram.__init__(self,name)
        self.world = world
        self.world.robot(0).setConfig(motion.robot.getKlamptCommandedPosition())
        self.drivePos = [0,0,0]
        self.driveRot = so3.identity()
        self.driveArm = 'l'
        self.useRotation = True
        self.localMotion = True
        self.increment = 0.05
        self.incrementang = 0.1
        self.trace_sensed = {'l':[],
                             'r':[]}

        self.baseCommand = [0,0,0]
        self.increment = 0.01
        self.incrementang = 0.02

        print "Moving to neutral configuration"
        q = motion.robot.left_limb.commandedPosition()
        q[1] = -1.0;
        q[3] = 2.0;
        q[5] = 1.0;
        motion.robot.left_mq.appendLinearRamp(q)
        q = motion.robot.right_limb.commandedPosition()
        q[1] = -1.0;
        q[3] = 2.0;
        q[5] = 1.0;
        motion.robot.enableCollisionChecking(True)
        motion.robot.right_mq.setRamp(q)
        motion.robot.left_ee.setOffset([0,0,0.1])
        motion.robot.right_ee.setOffset([0,0,0.1])
        self.state = 'move to start'

    def closeEvent(self,event):
        """Called when the "X" button is pressed"""
        motion.robot.shutdown()
        event.accept()

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

        #draw commanded pose
        gldraw.xform_widget((self.driveRot,self.drivePos),0.1,0.02,fancy=True)

        #draw trace
        glDisable(GL_LIGHTING)
        glColor3f(1,0.5,0)
        for arm in self.trace_sensed:
            glBegin(GL_LINE_STRIP)
            for T in self.trace_sensed[arm]:
                glVertex3f(*T[1])
            glEnd()
        glEnable(GL_LIGHTING)
        return

    def specialfunc(self,c,x,y):
        #Put your keyboard special character handler here
        print c,"pressed"
        self.refresh()

    def specialfunc(self,c,x,y):
        #Put your keyboard special character handler here
        print c,"pressed"
        if c == GLUT_KEY_LEFT:
            self.baseCommand[1] += self.increment
        elif c == GLUT_KEY_RIGHT:
            self.baseCommand[1] -= self.increment
        elif c == GLUT_KEY_UP:
            self.baseCommand[0] += self.increment
        elif c == GLUT_KEY_DOWN:
            self.baseCommand[0] -= self.increment
        self.updateBaseCommand()
        self.refresh()

    def updateBaseCommand(self):
        motion.robot.base.moveVelocity(*self.baseCommand)
        self.refresh()

    def updateCommand(self):
        jointErr = 0.5 if self.localMotion else 0.0
        positionBand = 0.2
        rotationBand = 2
        if self.useRotation:
            print "Moving to rotation (quat):",so3.quaternion(self.driveRot),", position:",self.drivePos
            if self.driveArm=='l':
                if not motion.robot.left_ee.moveTo(self.driveRot,self.drivePos,maxPositionError=positionBand,maxRotationError=rotationBand,maxJointDeviation=jointErr):
                    print "IK failure, snapping widget to current position"
                    self.driveRot,self.drivePos = motion.robot.left_ee.commandedTransform()
            elif self.driveArm=='r':
                if not motion.robot.right_ee.moveTo(self.driveRot,self.drivePos,maxPositionError=positionBand,maxRotationError=rotationBand,maxJointDeviation=jointErr):
                    print "IK failure, snapping widget to current position"
                    self.driveRot,self.drivePos = motion.robot.right_ee.commandedTransform()
            else:
                raise ValueError("Invalid value for self.driveArm")
        else:
            print "Moving to position:",self.drivePos
            if self.driveArm=='l':
                R = motion.robot.left_ee.commandedTransform()[0]
                if not motion.robot.left_ee.moveTo(R,self.drivePos,maxPositionError=positionBand,maxJointDeviation=jointErr):
                    self.driveRot,self.drivePos = motion.robot.left_ee.commandedTransform()
            elif self.driveArm=='r':
                R = motion.robot.right_ee.commandedTransform()[0]
                if not motion.robot.right_ee.moveTo(R,self.drivePos,maxPositionError=positionBand,maxJointDeviation=jointErr):
                    self.driveRot,self.drivePos = motion.robot.right_ee.commandedTransform()
            else:
                raise ValueError("Invalid value for self.driveArm")

    def keyboardfunc(self,c,x,y):
        #Put your keyboard handler here
        #the current example toggles simulation / movie mode
        print c,"pressed"
        if c.lower()=='q':
            motion.robot.shutdown()
            self.close()
        elif c.lower()=='h':
            self.print_help()
        elif c.lower()=='o':
            self.useRotation = not self.useRotation
            print "Using rotation?",self.useRotation
        elif c.lower()=='p':
            self.localMotion = not self.localMotion
            print "Local motion?",self.localMotion
        elif c.lower()=='l':
            print "Switching to left arm"
            self.driveArm = 'l'
            self.driveRot,self.drivePos = motion.robot.left_ee.commandedTransform()
        elif c.lower()=='r':
            print "Switching to right arm"
            self.driveArm = 'r'
            self.driveRot,self.drivePos = motion.robot.right_ee.commandedTransform()
        elif c==' ':
            motion.robot.stopMotion()
        elif c=='X':
            self.drivePos[0] += self.increment
            self.updateCommand()
        elif c=='x':
            self.drivePos[0] -= self.increment
            self.updateCommand()
        elif c=='Y':
            self.drivePos[1] += self.increment
            self.updateCommand()
        elif c=='y':
            self.drivePos[1] -= self.increment
            self.updateCommand()
        elif c=='Z':
            self.drivePos[2] += self.increment
            self.updateCommand()
        elif c=='z':
            self.drivePos[2] -= self.increment
            self.updateCommand()
        elif c=='!':
            R = so3.rotation([1,0,0],self.increment)
            self.driveRot = so3.mul(R,self.driveRot)
            self.updateCommand()
        elif c=='1':
            R = so3.rotation([1,0,0],-self.increment)
            self.driveRot = so3.mul(R,self.driveRot)
            self.updateCommand()
        elif c=='@':
            R = so3.rotation([0,1,0],self.increment)
            self.driveRot = so3.mul(R,self.driveRot)
            self.updateCommand()
        elif c=='2':
            R = so3.rotation([0,1,0],-self.increment)
            self.driveRot = so3.mul(R,self.driveRot)
            self.updateCommand()
        elif c=='#':
            R = so3.rotation([0,0,1],self.increment)
            self.driveRot = so3.mul(R,self.driveRot)
            self.updateCommand()
        elif c=='3':
            R = so3.rotation([0,0,1],-self.increment)
            self.driveRot = so3.mul(R,self.driveRot)
            self.updateCommand()
        elif c==',' or c=='<':
            self.baseCommand[2] += self.incrementang
            self.updateBaseCommand()
        elif c=='.' or c=='>':
            self.baseCommand[2] -= self.incrementang
            self.updateBaseCommand()

        self.refresh()

    def extendTrace(self,trace,T):
        if len(trace) < 2:
            trace.append(T)
        else:
            pprev = trace[-2]
            prev = trace[-1] 
            #check linearity
            eold = se3.error(trace[-1],trace[-2])
            enew = se3.error(T,trace[-1])
            if angle(eold,enew) > 0.1 or vectorops.normSquared(eold) > 0.02*0.02:
                trace.append(T)
            else:
                #near-linear
                trace[-1] = T

    def idle(self):
        if self.state == 'move to start':
            if motion.robot.left_mq.moving() or motion.robot.right_mq.moving():
                return
            self.state = 'posing'
            self.driveRot,self.drivePos = motion.robot.left_ee.commandedTransform()
        dt = time.time()-self.lasttime
        #integrate trace
        trace = self.trace_sensed[self.driveArm]
        if len(trace)==0:
            self.trace_sensed['l'] = [motion.robot.left_ee.commandedTransform()]
            self.trace_sensed['r'] = [motion.robot.right_ee.commandedTransform()]
        #update sensed trace
        if self.driveArm=='l':
            T = motion.robot.left_ee.commandedTransform()
        else:
            T = motion.robot.right_ee.commandedTransform()
        self.extendTrace(self.trace_sensed[self.driveArm],T)
        self.refresh()
        
    def print_help(self):
        print "Press h to print this message"
        print "Press l/r to switch to left/right arm"
        print "Press o to toggle orientation control"
        print "Press p to toggle between local and global IK solving"
        print "Press [space] to stop movement"
        print "Press X/x to increase/decrease x position"
        print "Press Y/y to increase/decrease y position"
        print "Press Z/z to increase/decrease z position"
        print "Press !/1 to increase/decrease x orientation"
        print "Press @/2 to increase/decrease y orientation"
        print "Press #/3 to increase/decrease z orientation"
        print "Press arrow keys to control base velocity"
        print "Press </> to increase/decrease base rotation speed"
        print "Press q to exit."
        print


if __name__ == "__main__":
    config.parse_args()
    print "drive_control.py: allows driving end effectors with the Motion module"
    print
    print "Loading Motion Module model",config.klampt_model
    motion.setup(mode=config.mode,klampt_model=config.klampt_model,libpath="./",)
    res = motion.robot.startup()
    if not res:
        print "Error starting up Motion Module"
        exit(1)
    time.sleep(0.01)
    world = WorldModel()
    res = world.readFile(config.klampt_model)
    if not res:
        raise RuntimeError("Unable to load Klamp't model "+config.klampt_model)
  
    viewer = MyEEPoseProgram(world)
    viewer.print_help()
    viewer.run()
    motion.robot.shutdown()
