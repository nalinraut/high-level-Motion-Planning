#!/usr/bin/env python

'''---------------------------------------------------------------------------

  desc:    convert joystick -> Twist msg
           input:  'joy' sensor_msgs::Joy
           output: 'base/joystick/cmd/ts' geometry_msgs::Twist
           output: 'base/joystick/cmd/js' trajectory_msgs::JointTrajectory
           output: 'base/cmd/mode' std_msgs/String        
           
  author:  ks
  created: 11/29/14

---------------------------------------------------------------------------'''

import roslib
import rospy, sys
import tf
import math

# ros common msgs
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist 
from sensor_msgs.msg import Joy 
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from lib.jp_common_apps import BaseCmdHandler
#from lib.jp_pan_tilt import PanTiltCmdHandler

NODE_NAME='app_joysick_parser'

#=======================================================#
#   
#=======================================================#
class ROSNode:

  #  constructor 
  #-----------------------------------------------------#
  def __init__(self):

    rospy.init_node( NODE_NAME )

    self.prefix = '[JP] '

    # params
    self.use_timer = False
    self.use_hold_button = False
    self.drive_type = rospy.get_param( '~drive_type', 'diff' )
    self.app_types = rospy.get_param( '~app_types', [ 'base' ] )
      
    # print params
    print 'params: '
    print ' - apps: ['+str(self.app_types)+']'

    # timer
    if ( self.use_timer ):
      rospy.Timer( rospy.Duration( 1.0 / self.pub_rate ), self.timerCallback )

    # sub 
    self.sub_joy = rospy.Subscriber( 'joy', Joy, self.joyCallback )

    # create handle for each source
    self.cmd_handlers = {}

    # init cmd handlers
    self.cmd_handlers['base'] = BaseCmdHandler( self, 'base' )
    #self.cmd_handlers['pan_tilt'] = PanTiltCmdHandler( self, 'pan_tilt' )

    print 'Initialized '+NODE_NAME+'.'

  #  timer 
  #-----------------------------------------------#
  def timerCallback( self, event ):
    self.publishBaseCmd()

  #  joy 
  #-----------------------------------------------#
  def joyCallback( self, msg ):
    self.cmd_handlers['base'].handleJoystick( msg )
    #self.cmd_handlers['pan_tilt'].handleJoystick( msg )


def main(args):
  print 'Starting ['+NODE_NAME+']'
  node = ROSNode() 
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down node."


if __name__ == '__main__':
  main(sys.argv)
