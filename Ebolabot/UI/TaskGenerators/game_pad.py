"""
Controls Baxter using any game pad / joystick via the logitech and Motion modules.
Designed for Logitech controllers, may work for other controllers too.
Converted by Kris Hauser Oct 2015 from a script written by Peter Moran,
July 2015.

SETUP:
Before running this script, you must run the system state service and the
controller dispatcher.

CONTROLS:
Controls are modified when special control buttons are pressed to create combinations, noted below.
Always:
    Right trigger -- switch between precision grip and power grip
No modifier (End effector velocity mode, ie translations):
    Right stick x -- move end effector along Baxter's y axis (ie side to side)
    Right stick y -- move end effector along Baxter's x axis (ie outward)
    Left stick x -- gripper open and close
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
import os
import sys
ebolabot_root = os.getenv("EBOLABOT_PATH",".")
sys.path.append(ebolabot_root)
#for logitech module
sys.path.append(os.path.join(ebolabot_root,'InputDevices/GamePad'))
import logitech
from task_generator import TaskGenerator

from OpenGL.GL import *
from klampt.vis import gldraw
from klampt.vis.glinterface import GLPluginInterface as GLPluginBase

#imaging stuff
try:
    from PIL import Image
except ImportError, err:
    import Image

#set this -1 for view-centric control, looking at the face of the robot
viewToWorldScaleXY = 1
''' gripper Mode: power, precision '''
GripperMode = {}
GripperMode['left'] = 'power'
GripperMode['right'] = 'power'

''' Hold Mode: free, hold '''
HoldMode = {}
HoldMode['left'] = 'free'
HoldMode['right'] = 'free'

HoldPose = {}
HoldPose['left'] = [1.0, 1.0, 1.0, 1.0]
HoldPose['right'] = [1.0, 1.0, 1.0, 1.0]

TuckPose = {}
TuckPose['left'] = [-0.08897088559570313, -0.9675583808532715, -1.2034079267211915, 1.7132575355041506, 0.6776360122741699, 1.0166457660095216, -0.5065971546203614]
TuckPose['right'] = [0.08897088559570313, -0.9675583808532715, 1.2034079267211915, 1.7132575355041506, -0.6776360122741699, 1.0166457660095216, 0.5065971546203614]

TuckStatus = {}
TuckStatus['left'] = False
TuckStatus['right'] = False

class GLTexture:
    def __init__(self,fn=None):
        self.glid = None
        if fn:
            self.loadImage(fn)
    def destroy(self):
        glDeleteTextures([self.glid])

    def setBytes(self,w,h,buffer,glformat=GL_RGBA):
        self.w,self.h = w,h
        if self.glid == None:
            self.glid = glGenTextures(1)
        glBindTexture(GL_TEXTURE_2D,self.glid)
        glPixelStorei(GL_UNPACK_ALIGNMENT,1)
        glTexImage2D(
            GL_TEXTURE_2D, 0, glformat, w, h, 0,
            glformat, GL_UNSIGNED_BYTE, buffer
        )
    def loadImage(self,fn):
        im = Image.open(fn)
        try:
            self.w,self.h,image = im.size[0],im.size[1],im.tobytes("raw","RGBA",0,-1)
        except SystemError:
            self.w,self.h,image = im.size[0],im.size[1],im.tobytes("raw","RGBX",0,-1)
        if self.glid == None:
            self.glid = glGenTextures(1)
        glBindTexture(GL_TEXTURE_2D,self.glid)
        glPixelStorei(GL_UNPACK_ALIGNMENT,1)
        glTexImage2D(
            GL_TEXTURE_2D, 0, GL_RGBA, self.w, self.h, 0,
            GL_RGBA, GL_UNSIGNED_BYTE, image
        )
        return True
    def enable(self,smooth=True,glmode=GL_MODULATE):
        glEnable(GL_TEXTURE_2D)
        if smooth:
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
        else:
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
        glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, glmode)
        glBindTexture(GL_TEXTURE_2D,self.glid)
    def disable(self):
        glDisable(GL_TEXTURE_2D)
    def blit(self,x,y,w=None,h=None):
        if w==None: w = self.w
        if h==None: h = self.h
        self.enable()
        glDisable(GL_LIGHTING)
        glColor4f(1,1,1,1)
        glBegin(GL_QUADS)
        glTexCoord2f(0,1)
        glVertex2f(x,y)
        glTexCoord2f(0,0)
        glVertex2f(x,y+h)
        glTexCoord2f(1,0)
        glVertex2f(x+w,y+h)
        glTexCoord2f(1,1)
        glVertex2f(x+w,y)
        glEnd()
        self.disable()

class MyWidgetPlugin(GLPluginBase):
    def __init__(self,taskGen):
        GLPluginBase.__init__(self)
        self.taskGen = taskGen

    def initialize(self):
        GLPluginBase.initialize(self)
        self.images = {}
        self.images['Cartesian position'] = GLTexture("UI/Resources/cartesian-translation.png")
        self.images['Cartesian rotation'] = GLTexture("UI/Resources/cartesian-rotation.png")
        self.images['Joint angles'] = GLTexture("UI/Resources/joint.png")
        self.images['left'] = GLTexture("UI/Resources/left-arm.png")
        self.images['right'] = GLTexture("UI/Resources/right-arm.png")
        return True

    def keyboardfunc(self,c,x,y):
        if c=='l':
            print "Switching to left limb"
            self.taskGen.limb = 'left'
        elif c=='r':
            print "Switching to right limb"
            self.taskGen.limb = 'right'

    def display(self):
        pass

    def display_screen(self):
        glRasterPos(20,30)
        glColor3f(1,1,1)
        glDisable(GL_LIGHTING)
        if self.taskGen.limb in self.images:
            self.images[self.taskGen.limb].blit(20,40)
        if self.taskGen.controlMode() in self.images:
            self.images[self.taskGen.controlMode()].blit(40+64,40)

    def eventfunc(self,type,args):
        """TODO: connect this up to GUI buttons"""
        if type=='button':
            if args=='left':
                self.taskGen.limb = 'left'
            if args=='right':
                self.taskGen.limb = 'right'

class GamePadTaskGenerator(TaskGenerator):
    def __init__(self):
        self.j = None
        self.limb = 'left'
        self.lastState = {}
        self.plugin = None

    def name(self): return "GamePad"

    def init(self,world):
        assert self.j == None,"Init may only be called once"
        self.world = world
        # Connect to controller
        try:
            self.j = logitech.Joystick(0)
            return True
        except:
            print "Joystick not found"
            print "Note: Pygame reports there are " + str(logitech.numJoys()) + " joysticks"
            return False

    def start(self):
        if not self.j: return False
        self.limb = 'left'
        self._status = 'ok'
        self.plugin = MyWidgetPlugin(self)
        self.lastState = {}
        return True

    def status(self):
        if self.j:
            return 'ok'
        else:
            return 'error'

    def messages(self):
        return ["Controlling "+self.limb]

    def controlMode(self):
        if len(self.lastState)==0: return 'None'
        if self.lastState['RB']: return 'Joint angles'
        elif self.lastState['LB']: return "Cartesian rotation"
        else: return "Cartesian position"

    def stop(self):
        self._status=''
        self.plugin = None

    def get(self):
        # Read controller
        j = self.j
        j.updateState()
        rstick = j.get_stick_R()
        lstick = j.get_stick_L()
        LB = j.get_LB()
        RB = j.get_RB()
        LT = j.get_LT()
        RT = j.get_RT()
        B_button = j.get_B()
        A_button = j.get_A()
        X_button = j.get_X()

        state = {}
        state['rstick'] = rstick
        state['lstick'] = lstick
        state['LB'] = LB
        state['RB'] = RB
        state['LT'] = LT
        state['RT'] = RT
        state['B'] = B_button
        state['A'] = A_button
        state['X'] = X_button

        if len(self.lastState) > 0:
            res = self.do_logic(self.lastState,state)
        else:
            res = None

        self.lastState = state
        return res

    def do_logic(self,lastState,state):
        global GripperMode
        global HoldMode
        global HoldPose

        rstick = state['rstick']
        lstick = state['lstick']
        if state['LT'] > 0.1 and lastState['LT'] <= 0.1:
            if self.limb == 'left':
                self.limb = 'right'
            else:
                self.limb = 'left'

        if state['B'] == 1 and self.lastState['B'] == 0:
            if GripperMode[self.limb] == 'power':
                GripperMode[self.limb] = 'precision'
            elif GripperMode[self.limb] == 'precision':
                GripperMode[self.limb] = 'power'

        if state['A'] == 1 and self.lastState['A'] == 0:
            if HoldMode[self.limb] == 'free':
                HoldMode[self.limb] = 'hold'
            elif HoldMode[self.limb] == 'hold':
                HoldMode[self.limb] = 'free'

        if state['X'] == 1 and self.lastState['X'] == 0:
            if TuckStatus[self.limb] == False:
                TuckStatus[self.limb] = True
            elif TuckStatus[self.limb] == True:
                TuckStatus[self.limb] = False

        if TuckStatus[self.limb] == True:
            Jointmsg = {}
            Jointmsg['type'] = "JointPose"
            Jointmsg['part'] = self.limb
            Jointmsg['position'] = TuckPose[self.limb]
            Jointmsg['speed'] = 1
            Jointmsg['safe'] = 0
            # TuckStatus[self.limb] = False
            print Jointmsg
            return Jointmsg

        if state['RT'] > -1.0 and abs(state['RT']) > 0.01 :
            gripMsg = {}
            preshape = [1.0]
            if GripperMode[self.limb] == 'power':
                preshape = [1.0]
            elif GripperMode[self.limb] == 'precision':
                preshape = [0.4]

            p = [1.0,1.0, 1.0] + preshape
            gripMsg['limb'] = self.limb
            gripMsg['type'] = 'Gripper'
            gripMsg['force'] = 0.4
            gripMsg['speed'] = 0.2
            if HoldMode[self.limb] == 'free':
                gripsize = (0.2 - 1.8)*state['RT']/2.0 + 1.8/2.0
                p = [gripsize, gripsize, gripsize] + preshape
                HoldPose[self.limb] = p
                gripMsg['position'] = p
            elif HoldMode[self.limb] == 'hold':
                gripMsg['position'] = HoldPose[self.limb]
            return gripMsg



        # if state['RT'] > 0.1 and lastState['RT'] <= 0.1:
            # return {'type':'Gripper','limb':self.limb,'command':'close'}
        # elif state['RT'] <= 0.1 and lastState['RT'] > 0.1:
            # return {'type':'Gripper','limb':self.limb,'command':'open'}

        if state['RB']:
            # Joint velocity mode
            jVel = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            jVel[6] = rstick[0] / 5.0  # wrist 2
            jVel[5] = rstick[1] / 5.0  # wrist 1
            jVel[4] = lstick[0] / 5.0  # wrist 0
            jVel[3] = lstick[1] / 5.0  # elbow 1
            return {'type':'JointVelocity','limb':self.limb,'velocity':jVel}
        if state['LB']:  # End effector angular velocity mode
            driveVel = [0.0, 0.0, 0.0]
            driveAngVel = [0.0, 0.0, 0.0]
            driveAngVel[0] = viewToWorldScaleXY*rstick[0] / 5.0  # about end effector x
            driveAngVel[1] = -viewToWorldScaleXY*rstick[1] / 5.0   # about end effector y
            driveAngVel[2] = -lstick[0] / 5.0   # about end effector z
            return {'type':'CartesianVelocity','limb':self.limb,'linear':driveVel,'angular':driveAngVel}
        else:  # End effector velocity mode
            driveVel = [0.0, 0.0, 0.0]
            driveAngVel = [0.0, 0.0, 0.0]
            driveVel[1] = -viewToWorldScaleXY*rstick[0] / 5.0  # y
            driveVel[0] = -viewToWorldScaleXY*rstick[1] / 5.0  # x
            driveVel[2] = -lstick[1] / 5.0  # z
            return {'type':'CartesianVelocity','limb':self.limb,'linear':driveVel,'angular':driveAngVel}

    def glPlugin(self):
        return self.plugin

def make():
    return GamePadTaskGenerator()
