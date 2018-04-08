#contains constants for Baxter with the reflex grippers (not totally done yet!)
from klampt import vectorops,so3,loader
from klampt import resource
import math
import sys

sys.path.insert(0, '/home/motion/ece590-s2015/klampt_models')
#load gripper module
# sys.path.append('klampt_models')
try:
    import reflex_col as gripper
except ImportError:
    from klampt_models import reflex_col as gripper

#the .rob file name
klampt_model_name = 'baxter_with_reflex_gripper_col.rob'
gripper_model_name = gripper.klampt_model_name

#indices of the left and right cameras in the Baxter robot file
left_camera_link_name = 'left_hand_camera'
right_camera_link_name = 'right_hand_camera'

#indices of the left and right grippers in the Baxter robot file
left_gripper_link_name = 'left_gripper'
right_gripper_link_name = 'right_gripper'

left_arm_geometry_indices = [15,16,17,18,19,21,22,23,30,31]
right_arm_geometry_indices = [35,36,37,38,39,41,42,43,50,51]
left_hand_link_start = 54
right_hand_link_start = 54+gripper.numDofs
left_hand_geometry_indices = range(54,54+gripper.numDofs)
right_hand_geometry_indices = range(54+gripper.numDofs,54+gripper.numDofs*2)

#indices of the left and right arms in the Baxter robot file
left_arm_link_names = ['left_upper_shoulder','left_lower_shoulder','left_upper_elbow','left_lower_elbow','left_upper_forearm','left_lower_forearm','left_wrist']
right_arm_link_names = ['right_upper_shoulder','right_lower_shoulder','right_upper_elbow','right_lower_elbow','right_upper_forearm','right_lower_forearm','right_wrist']

#local transformations (rotation, translation pairs) of the grasp center
resource.setDirectory("resources/reflex")
left_gripper_center_xform = resource.get("left_gripper_center.xform",
                                         type='RigidTransform',
                                         description="Left gripper center",
                                         world="klampt_models/"+klampt_model_name,
                                         frame=left_gripper_link_name)
right_gripper_center_xform = resource.get("right_gripper_center.xform",
                                          type='RigidTransform',
                                          default=left_gripper_center_xform,
                                          description="Right gripper center",
                                          world="klampt_models/"+klampt_model_name,
                                          frame=right_gripper_link_name)

#resting configuration
rest_config = resource.get("baxter_rest.config",type="Config",description="Rest configuration",world="klampt_models/"+klampt_model_name)

def set_model_gripper_command(robot,limb,command):
    """Given the Baxter RobotModel 'robot' at its current configuration,
    this will set the configuration so the gripper on limb 'limb' is
    placed at the gripper command values 'command'.
    """
    qrobot = robot.getConfig()
    qhand = gripper.commandToConfig(command)
    if limb=='left':
        print "Opening left gripper to",command
        for i in range(gripper.numDofs):
            qrobot[left_hand_link_start+i] = qhand[i]
    else:
        print "Opening right gripper to",command
        for i in range(gripper.numDofs):
            qrobot[right_hand_link_start+i] = qhand[i]
    robot.setConfig(qrobot)

def set_q_gripper_command(qlimb,limb,command):
    qhand = gripper.commandToConfig(command)
    if limb=='left':
        for i in range(gripper.numDofs):
            qlimb[left_hand_link_start+i] = qhand[i]
    else:
        for i in range(gripper.numDofs):
            qlimb[right_hand_link_start+i] = qhand[i]
    
