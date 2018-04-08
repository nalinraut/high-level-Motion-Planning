# Import Modules
import os
import sys
ebolabot_root = os.getenv("EBOLABOT_PATH",".")
sys.path.append(ebolabot_root)
#for logitech module
sys.path.append(os.path.join(ebolabot_root,'InputDevices/GamePad'))
from Common.system_config import EbolabotSystemConfig
from task_generator import TaskGenerator
from klampt import RobotPoser
from klampt.vis.glcommon import GLWidgetPlugin

# parameters for initial robot pose
q_init_left = [-0.08897088559570313, -0.9675583808532715, -1.2034079267211915, 1.7132575355041506, 0.6776360122741699, 1.0166457660095216, -0.5065971546203614]
q_init_right = [0.08897088559570313, -0.9675583808532715, 1.2034079267211915, 1.7132575355041506, -0.6776360122741699, 1.0166457660095216, 0.5065971546203614]

#Configuration variable: where's the state server?
system_state_addr = EbolabotSystemConfig.getdefault_ip('state_server_computer_ip',('localhost',4568))

class MyWidgetPlugin(GLWidgetPlugin):
    def __init__(self,taskGen):
        GLWidgetPlugin.__init__(self)
        self.world = taskGen.world
        self.sendMilestone = False
        self.initialPose = False

    def initialize(self):
        GLWidgetPlugin.initialize(self)
        return True
    def display(self):
        robot = self.world.robot(0)
        oldColors = []
        for i in range(robot.numLinks()):
            #c = robot.link(i).appearance().getColor()
            c = [0.5,0.5,0.5,1.0]
            oldColors.append(c)
            robot.link(i).appearance().setColor(c[0],c[1],c[2],0.5)
        GLWidgetPlugin.display(self)
        for i in range(robot.numLinks()):
            c = oldColors[i]
            robot.link(i).appearance().setColor(c[0],c[1],c[2],c[3])
    def keyboardfunc(self,c,x,y):
        if c==' ':
            self.sendMilestone = True
        # == initialization 
        elif c == 'i':
            self.initialPose = True
            print "button i is pressed"

    def eventfunc(self,type,args):
        """TODO: connect this up to GUI buttons"""
        if type=='button':
            if args=='send':
                self.sendMilestone = True

class MousePoserTaskGenerator(TaskGenerator):
    """Allows the user to interact with the model by right clicking
    using the mouse and pressing space bar to send the milestone.
    
    Also demonstrates how to write a Python plugin task with an OpenGL
    widget.
    """
    def __init__(self):
        self.plugin = None

    def name(self): return "MousePoser"

    def init(self,world):
        self.world = world
        self.robotPoser = RobotPoser(world.robot(0))
        self.plugin = MyWidgetPlugin(self)
        self.plugin.addWidget(self.robotPoser)
        robot = world.robot(0)
        left_arm_link_names = ['left_upper_shoulder','left_lower_shoulder','left_upper_elbow','left_lower_elbow','left_upper_forearm','left_lower_forearm','left_wrist']
        right_arm_link_names = ['right_upper_shoulder','right_lower_shoulder','right_upper_elbow','right_lower_elbow','right_upper_forearm','right_lower_forearm','right_wrist']
        base_link_names = ['base_x','base_y','base_yaw']
        self.left_arm_link_indices = [robot.link(l).index for l in left_arm_link_names]
        self.right_arm_link_indices = [robot.link(l).index for l in right_arm_link_names]
        self.base_link_indices = [robot.link(l).index for l in base_link_names]
        if any(v < 0 for v in self.left_arm_link_indices+self.right_arm_link_indices):
            raise ValueError("Robot in world does not have Baxter's torso?")
        return True

    def start(self):
        self._status = 'ok'
        self.robotPoser.set(self.world.robot(0).getConfig())
        return True

    def stop(self):
        pass

    def get(self):
        if self.plugin.initialPose:
            self.plugin.initialPose = False
            q = self.robotPoser.get()
            for i in range(len(self.left_arm_link_indices)):
                idx = self.left_arm_link_indices[i]
                q[idx] = q_init_left[i]
            for i in range(len(self.right_arm_link_indices)):
                idx = self.right_arm_link_indices[i]
                q[idx] = q_init_right[i]
            self.robotPoser.set(q)
            print "set up to initial pose"

        if self.plugin.sendMilestone:
            self.plugin.sendMilestone = False
            q = self.robotPoser.get()
            qcmd = self.world.robot(0).getConfig()
            baseMoved = any(qcmd[i] != q[i] for i in self.base_link_indices)
            leftArmMoved = any(qcmd[i] != q[i] for i in self.left_arm_link_indices)
            rightArmMoved = any(qcmd[i] != q[i] for i in self.right_arm_link_indices)
            moved = []
            if baseMoved: moved.append('base')
            if leftArmMoved: moved.append('left')
            if rightArmMoved: moved.append('right')
            targets = []
            if baseMoved: targets.append([q[i] for i in self.base_link_indices])
            if leftArmMoved: targets.append([q[i] for i in self.left_arm_link_indices])
            if rightArmMoved: targets.append([q[i] for i in self.right_arm_link_indices])
            print "Sending milestone",q,", moving parts:",",".join(moved)
            #CONSTRUCT THE TASK HERE
            msg = {}
            msg['type'] = 'JointPose'
            msg['parts'] = moved
            msg['positions'] = targets
            msg['speed'] = 1
            #TEST: only allow safe configurations (self collision checking)
            msg['safe'] = 1
            return msg
        return None

    def glPlugin(self):
        return self.plugin

def make():
    return MousePoserTaskGenerator()
