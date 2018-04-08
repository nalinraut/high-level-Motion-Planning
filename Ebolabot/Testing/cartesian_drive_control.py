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

def angle(a,b):
    anorm = vectorops.norm(a)
    bnorm = vectorops.norm(b)
    if anorm < 1e-6 or bnorm < 1e-6: return 0
    dp = vectorops.dot(vectorops.div(a,anorm),vectorops.div(b,bnorm))
    dp = max(min(dp,1),-1)
    return math.acos(dp)

class MyEEDriveProgram(GLRealtimeProgram):
    """A class for driving around end effectors using the keyboard.  Used
    primarily for demonstrating the end effector drive function and for
    testing the Motion library.
    """
    def __init__(self,world,name="My GL Widget Program"):
        GLRealtimeProgram.__init__(self,name)
        self.world = world
        self.world.robot(0).setConfig(motion.robot.getKlamptCommandedPosition())
       
        self.driveVel = [0,0,0]
        self.driveAngVel = [0,0,0]
        self.driveArm = 'l'
        self.useRotation = True
        self.increment = 0.05
        self.incrementang = 0.1
        self.trace_int = {'l':[],
                          'r':[]}
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
        motion.robot.right_mq.setRamp(q)
        motion.robot.left_ee.setOffset([0,0,0.1])
        motion.robot.right_ee.setOffset([0,0,0.1])

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

        #draw trace
        glDisable(GL_LIGHTING)
        glColor3f(0,1,0)
        for arm in self.trace_int:
            glBegin(GL_LINE_STRIP)
            for T in self.trace_int[arm]:
                glVertex3f(*T[1])
            glEnd()
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

    def updateDrive(self):
        if motion.robot.left_mq.moving() or motion.robot.right_mq.moving():
            return
        if self.useRotation:
            print "Drive angular vel.",self.driveAngVel,", translational vel.",self.driveVel
            if self.driveArm=='l':
                motion.robot.left_ee.driveCommand(self.driveAngVel,self.driveVel)
            elif self.driveArm=='r':
                motion.robot.right_ee.driveCommand(self.driveAngVel,self.driveVel)
            else:
                raise ValueError("Invalid value for self.driveArm")
        else:
            print "Drive translational vel.",self.driveVel
            if self.driveArm=='l':
                motion.robot.left_ee.driveCommand(None,self.driveVel)
            elif self.driveArm=='r':
                motion.robot.right_ee.driveCommand(None,self.driveVel)
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
            self.updateDrive()
        elif c.lower()=='l':
            motion.robot.right_ee.driveCommand([0]*3,[0]*3)
            print "Switching to left arm"
            self.driveArm = 'l'
        elif c.lower()=='r':
            motion.robot.left_ee.driveCommand([0]*3,[0]*3)
            print "Switching to right arm"
            self.driveArm = 'r'
        elif c==' ':
            self.driveVel = [0,0,0]
            self.driveAngVel = [0,0,0]
            self.updateDrive()
            motion.robot.left_limb.velocityCommand([0]*motion.numLimbDofs)
            motion.robot.right_limb.velocityCommand([0]*motion.numLimbDofs)
            self.baseCommand = [0,0,0]
            self.updateBaseCommand()
        elif c=='X':
            self.driveVel[0] += self.increment
            self.driveVel[0] = max(self.driveVel[0],0)
            self.updateDrive()
        elif c=='x':
            self.driveVel[0] -= self.increment
            self.driveVel[0] = min(self.driveVel[0],0)
            self.updateDrive()
        elif c=='Y':
            self.driveVel[1] += self.increment
            self.driveVel[1] = max(self.driveVel[1],0)
            self.updateDrive()
        elif c=='y':
            self.driveVel[1] -= self.increment
            self.driveVel[1] = min(self.driveVel[1],0)
            self.updateDrive()
        elif c=='Z':
            self.driveVel[2] += self.increment
            self.driveVel[2] = max(self.driveVel[2],0)
            self.updateDrive()
        elif c=='z':
            self.driveVel[2] -= self.increment
            self.driveVel[2] = min(self.driveVel[2],0)
            self.updateDrive()
        elif c=='!':
            self.driveAngVel[0] += self.incrementang
            self.driveAngVel[0] = max(self.driveAngVel[0],0)
            self.updateDrive()
        elif c=='1':
            self.driveAngVel[0] -= self.incrementang
            self.driveAngVel[0] = min(self.driveAngVel[0],0)
            self.updateDrive()
        elif c=='@':
            self.driveAngVel[1] += self.incrementang
            self.driveAngVel[1] = max(self.driveAngVel[1],0)
            self.updateDrive()
        elif c=='2':
            self.driveAngVel[1] -= self.incrementang
            self.driveAngVel[1] = min(self.driveAngVel[1],0)
            self.updateDrive()
        elif c=='#':
            self.driveAngVel[2] += self.incrementang
            self.driveAngVel[2] = max(self.driveAngVel[2],0)
            self.updateDrive()
        elif c=='3':
            self.driveAngVel[2] -= self.incrementang
            self.driveAngVel[2] = min(self.driveAngVel[2],0)
            self.updateDrive()
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
        if motion.robot.left_mq.moving() or motion.robot.right_mq.moving():
            return
        dt = time.time()-self.lasttime
        #integrate trace
        trace = self.trace_int[self.driveArm]
        if len(trace)==0:
            self.trace_int['l'] = [motion.robot.left_ee.commandedTransform()]
            self.trace_int['r'] = [motion.robot.right_ee.commandedTransform()]
            self.trace_sensed['l'] = [motion.robot.left_ee.commandedTransform()]
            self.trace_sensed['r'] = [motion.robot.right_ee.commandedTransform()]
            trace = self.trace_int[self.driveArm]
        Tlast = trace[-1]
        tint = vectorops.madd(Tlast[1],self.driveVel,dt)
        Rint = so3.mul(so3.from_moment(vectorops.mul(self.driveAngVel,dt)),Tlast[0])
        self.extendTrace(self.trace_int[self.driveArm],(Rint,tint))
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
        print "Press [space] to stop movement"
        print "Press X/x to increase/decrease x velocity"
        print "Press Y/y to increase/decrease y velocity"
        print "Press Z/z to increase/decrease z velocity"
        print "Press !/1 to increase/decrease x ang velocity"
        print "Press @/2 to increase/decrease y ang velocity"
        print "Press #/3 to increase/decrease z ang velocity"
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
  
    viewer = MyEEDriveProgram(world)
    viewer.print_help()
    viewer.run()
    motion.robot.shutdown()
