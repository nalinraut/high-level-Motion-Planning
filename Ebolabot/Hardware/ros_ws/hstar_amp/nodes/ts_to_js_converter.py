#!/usr/bin/env python

'''---------------------------------------------------------------------------

  desc:    input: ts msg 
           output: js msg
           
  author:  ks
  created: 12/05/14

---------------------------------------------------------------------------'''

import roslib
import rospy, sys
import tf
import math

# ros common msgs
from std_msgs.msg import String
from geometry_msgs.msg import Twist 
from trajectory_msgs.msg import JointTrajectory

from lib.ts_to_js import TsToJsConverter

NODE_NAME='ts_to_js_converter'

#=======================================================#
#   
#=======================================================#
class ROSNode:

  #  constructor 
  #-----------------------------------------------------#
  def __init__(self):

    rospy.init_node( NODE_NAME )

    # params
    self.app_type = rospy.get_param( '~app_type', 'mecanum' )
    self.topic_prefix = rospy.get_param( '~topic_prefix', 'base/output/cmd' )
  
    print 'params: '
    print ' - app_type: ['+self.app_type+']'

    # intialize converter
    self.ts_to_js_converter = TsToJsConverter( self.app_type )
    print ' - create convert for app type: ['+self.app_type+'].'

    # sub ( ts )
    self.sub_ts = rospy.Subscriber( self.topic_prefix + '/ts', Twist, self.cmdCallback )
    print ' - sub to : ['+self.topic_prefix+'/ts].'

    # pub ( js )
    self.pub_js = rospy.Publisher( self.topic_prefix + '/js', JointTrajectory )
    print ' - pub on : ['+self.topic_prefix+'/js].'

    # joint names are optional
    if rospy.has_param( 'joint_names' ):
      self.joint_names = rospy.get_param( 'joint_names' )

    print 'Initialized '+NODE_NAME+'.'

  def cmdCallback( self, msg ):
    self.js_msg = self.ts_to_js_converter.convert( msg )
    self.pub_js.publish( self.js_msg )


def main(args):
  
  print 'Starting ['+NODE_NAME+']'
  node = ROSNode() 
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down node."


if __name__ == '__main__':
  main(sys.argv)
