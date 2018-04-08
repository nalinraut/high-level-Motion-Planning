#!/usr/bin/env python

'''---------------------------------------------------------------------------

  desc:    input: ts msg 
           output: js msg
           
  author:  ks
  created: 12/05/14

---------------------------------------------------------------------------'''

import roslib
roslib.load_manifest( 'hstar_amp' )
import rospy, sys
import tf
import math

# ros common msgs
from std_msgs.msg import String
from geometry_msgs.msg import Twist 
from trajectory_msgs.msg import JointTrajectory

from ts_to_js import TsToJsConverter

NODE_NAME='ts_to_js_test'

#=======================================================#
#   
#=======================================================#
class ROSNode:

  #  constructor 
  #-----------------------------------------------------#
  def __init__(self):

    rospy.init_node( NODE_NAME )

    # params
    self.app_type = rospy.get_param( 'app_type', 'mecanum' )
    self.topic_prefix = rospy.get_param( 'topic_prefix', '/base/output/cmd' )

    # pub ( js )
    self.pub_js = rospy.Publisher( self.topic_prefix + '/ts', JointTrajectory )

    # joint names are optional
    if rospy.has_param( 'joint_names' ):
      self.joint_names = rospy.get_param( 'joint_names' )

    # intialize converter
    self.ts_to_js_converter = TsToJsConverter( self.app_type )

    # test msgs
    test_msgs = [ Twist(), Twist(), Twist() ]
    test_msgs[0].linear.x = 1.0
    test_msgs[1].linear.y = 1.0
    test_msgs[2].angular.z = 1.0

    r = rospy.Rate( 0.5 ) # hz 
    self.loop_count = 0

    print 'Initialized '+NODE_NAME+'.'

    while not rospy.is_shutdown():
      self.js_msg = self.ts_to_js_converter.convert( test_msgs[ self.loop_count % len( test_msgs ) ] )
      #print str( self.js_msg )
      self.pub_js.publish( self.js_msg )
      r.sleep()
      self.loop_count = self.loop_count + 1


def main(args):
  
  print 'Starting ['+NODE_NAME+']'
  node = ROSNode() 
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down node."


if __name__ == '__main__':
  main(sys.argv)
