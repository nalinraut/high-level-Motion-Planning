import rospy
 
from reflex_sf_msgs.msg import Command
 
rospy.init_node('reflex_sf_dof_tour')
pub = rospy.Publisher('/reflex_hand2/command', Command, queue_size=10)

FINGER_CLOSED =2
FINGER_PINCH = 1 

PRESHAPE_CYLINDER = 0
PRESHAPE_SPHERICAL = 1.5
PRESHAPE_PINCH = 2.5


# finger 1 (right-hand index) close
rospy.sleep(2)
command=Command()
command.position=[2,2,2,0]
command.velocity=[0,0,0,0]
command.force=[0.1,0.1,0.1,0.1]
pub.publish(command)
rospy.sleep(2)
command=Command()
command.position=[0.5,0.5,0.5,0]
command.velocity=[0,0,0,0]
command.force=[0.1,0.1,0.1,0.1]
pub.publish(command)
rospy.sleep(2)
