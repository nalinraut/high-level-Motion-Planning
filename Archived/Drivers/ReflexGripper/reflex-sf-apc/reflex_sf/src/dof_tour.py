import rospy
 
from reflex_sf_msgs.msg import Pose
 
rospy.init_node('reflex_sf_dof_tour')
pub = rospy.Publisher('/reflex_sf/command', Pose, queue_size=10)

FINGER_CLOSED = 4.6
FINGER_PINCH = 3.5 

PRESHAPE_CYLINDER = 0
PRESHAPE_SPHERICAL = 1.5
PRESHAPE_PINCH = 2.5

# finger 1 (right-hand index) close
rospy.sleep(5)
pub.publish(0,0,0,0)
rospy.sleep(1)
pub.publish(FINGER_CLOSED, 0, 0, 0)
rospy.sleep(1)
pub.publish(0, 0, 0, 0)
rospy.sleep(1)
# finger 2 (right-hand middle) close
pub.publish(0, FINGER_CLOSED, 0, 0)
rospy.sleep(1)
pub.publish(0, 0, 0, 0)
rospy.sleep(1)
# finger 3 (thumb) close
pub.publish(0, 0, FINGER_CLOSED, 0)
rospy.sleep(1)
pub.publish(0, 0, 0, 0)
rospy.sleep(1)
# preshape
pub.publish(0, 0, 0, PRESHAPE_SPHERICAL)
rospy.sleep(1)
pub.publish(0, 0, 0, PRESHAPE_PINCH)
rospy.sleep(1)
pub.publish(0, 0, 0, 0)
rospy.sleep(1)
# hand closed in cylindrical power grasp
pub.publish(FINGER_CLOSED, FINGER_CLOSED, FINGER_CLOSED, 0)
rospy.sleep(1)
# hand open
pub.publish(0, 0, 0, 0)
rospy.sleep(1)
# preshape hand for pinch
pub.publish(0, 0, 0, PRESHAPE_PINCH)
rospy.sleep(1)
# pinch grasp
pub.publish(FINGER_PINCH, FINGER_PINCH, 0, PRESHAPE_PINCH)
rospy.sleep(1)
# hand open (pinch grasp)
pub.publish(0, 0, 0, PRESHAPE_PINCH)
rospy.sleep(1)
# hand open (cylindrical grasp)
pub.publish(0, 0, 0, 0)
rospy.sleep(1)
