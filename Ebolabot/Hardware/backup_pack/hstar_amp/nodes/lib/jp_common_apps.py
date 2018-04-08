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
class BaseCmdHandler( CmdHandler ):
  
  def __init__( self, parent, name ):
    CmdHandler.__init__( self, parent, name )
    self.initDefaults()

  def initDefaults( self ):
    self.cmd['ts'] = Twist()
    self.cmd['js'] = self.createJointMsg( 4 )
    self.pub_cmd['ts'] = rospy.Publisher( self.app_name+'/joystick/cmd/ts', Twist )
    self.pub_cmd['js'] = rospy.Publisher( self.app_name+'/joystick/cmd/js', JointTrajectory )
    self.use_deadman = False
    self.deadman_button = 'START'
    self.drive_type = 'mecanum'
    self.prefix = '['+self.app_name+']'

  def handleJoystick( self, msg ):

    # parse
    CmdHandler.handleJoystick( self, msg )
    j = self.joystick
    # print "in jp_common_apps: mode = ", self.drive_type 
    if ( not self.use_deadman ) or ( self.use_deadman and j.buttons[self.deadman_button].pressed ):
      if self.drive_type == 'diff':
        self.cmd['ts'].linear.x = j.lx_raw
        self.cmd['ts'].angular.z = j.ly_raw
        # print "diff: cmd['ts'].linear.x = ", self.cmd['ts'].linear.x, " cmd['ts'].angular.z = ", self.cmd['ts'].angular.z
      else:
        self.cmd['ts'].linear.x = j.lx_raw
        self.cmd['ts'].linear.y = j.ly_raw
        self.cmd['ts'].angular.z = j.ry_raw
        # print "cmd['ts'].linear.x = ", self.cmd['ts'].linear.x, " cmd['ts'].linear.y = ", self.cmd['ts'].linear.y, " cmd['ts'].angular.z = ", self.cmd['ts'].angular.z
      self.current_mode = 'joystick'

      if j.buttons['X'].active or j.buttons['Y'].active:
        if j.buttons['Y'].clicked:
          self.setMode( 'auto' )
        elif j.buttons['X'].clicked:
          self.setMode( 'joystick' )
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
