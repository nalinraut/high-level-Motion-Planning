#!/usr/bin/env python

'''---------------------------------------------------------------------------

  desc:    input:  
           output: 
           
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
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from jp_utils import CmdHandler

#-----------------------------------------------------#
#  base 
#-----------------------------------------------------#
class PanTiltCmdHandler( CmdHandler ):
  
  def __init__( self, parent, name ):
    CmdHandler.__init__( self, parent, name )
    self.initDefaults()

  def initDefaults( self ):
    self.cmd['ts'] = Twist()
    self.cmd['js'] = self.createJointMsg( 2 )
    self.pub_cmd['ts'] = rospy.Publisher( '/'+self.app_name+'/joystick/cmd/ts', Twist )
    self.pub_cmd['js'] = rospy.Publisher( '/'+self.app_name+'/joystick/cmd/js', JointTrajectory )
    self.use_deadman = True
    self.deadman_button = 'SELECT'

  def handleJoystick( self, msg ):
  
    # parse
    CmdHandler.handleJoystick( self, msg )
    j = self.joystick

    if not self.use_deadman or ( self.use_deadman and j.buttons[self.deadman_button].pressed ):
      self.cmd['ts'].linear.x = j.lx_raw
      self.cmd['ts'].linear.y = j.ly_raw
      self.current_mode = 'joystick'

      if j.buttons['X'].active or j.buttons['Y'].active:
        if j.buttons['Y'].clicked:
          self.publishMode( 'auto' )  # set auto
        elif j.buttons['X'].clicked:
          self.publishMode( 'joystick' ) # set manual
      else:
        if not self.use_timer:
          self.publishCmd()

    else:
      self.zeroCmd()

  def zeroCmd( self ):
    if self.current_mode == 'joystick':
      self.cmd['ts'].linear.x = 0.0
      self.cmd['ts'].linear.y = 0.0
      self.cmd['ts'].angular.z = 0.0
      self.publishCmd()
      self.publishCmd()
