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
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

#  helper classes - Button
#-----------------------------------------------------#
class Button:

  def __init__( self, index ):
    self.index = index
    self.pressed = False
    self.pressed_last = False
    self.clicked = False
    self.active = False

  def update( self, msg ):
    self.pressed_last = self.pressed
    self.pressed = msg.buttons[self.index] == 1
    self.clicked = self.pressed_last and not self.pressed
    self.active = self.pressed or self.clicked

#  helper classes - Joystick
#-----------------------------------------------------#
class Joystick:

  def __init__( self ):
    self.initButtons()

  def initButtons( self ):
    button_names = [ 'X', 'Y', 'A', 'B', 'START', 'SELECT', 'L1', 'R1', 'L3', 'R3', 'LOGO' ]
    button_indices = [ 2, 3, 0, 1, 7, 6, 4, 5, 9, 10, 8 ]
    self.buttons = {}
    for i in range( 0, len( button_names ) ):
      self.buttons[button_names[i]] = Button( button_indices[i] )
    self.l2_init = False
    self.r2_init = False

  def parse( self, msg ):
    gain_dpad = 0.01
    gain_analog = 0.03
    gain_trigger = 0.02

    # analog sticks 
    lx_raw            = msg.axes[ 1 ]
    ly_raw            = msg.axes[ 0 ]
    rx_raw            = msg.axes[ 4 ]
    ry_raw            = msg.axes[ 3 ]
    l2_raw            = msg.axes[ 2 ]
    r2_raw            = msg.axes[ 5 ]
    lx                = msg.axes[ 1 ] * gain_analog
    ly                = msg.axes[ 0 ] * gain_analog
    rx                = msg.axes[ 4 ] * gain_analog
    ry                = msg.axes[ 3 ] * gain_analog

    # back triggers ( logitech only )
    if not self.l2_init and l2_raw >= 0.99:
      self.l2_init = True
    if not self.r2_init and r2_raw >= 0.99:
      self.r2_init = True

    trigger2 = 0.0
    if self.l2_init and self.r2_init:
      l2 = l2_raw * gain_analog
      r2 = r2_raw * gain_analog
      trigger2 = ( l2 - r2 )
    else:
      trigger2 = 0.0

    '''l3 = 1.0 if self.buttons['L3'].pressed else 0.0
    r3 = 1.0 if self.buttons['R3'].pressed else 0.0
    trigger3 = ( r3 - l3 ) * gain_trigger'''

    # dpad ( up & left each treated as single axis )
    self.dpad_x            = msg.axes[ 7 ]
    self.dpad_y            = msg.axes[ 6 ]
    self.dpad_x_float      = msg.axes[ 7 ] * gain_dpad
    self.dpad_y_float      = msg.axes[ 6 ] * gain_dpad

    # set to self
    self.lx = lx
    self.ly = ly
    self.rx = rx
    self.ry = ry
    self.lx_raw = lx_raw
    self.ly_raw = ly_raw
    self.rx_raw = rx_raw
    self.ry_raw = ry_raw
    self.trigger2 = trigger2
    self.l2_raw = l2_raw
    self.r2_raw = r2_raw

    # buttons ( all )
    for button in self.buttons.itervalues():
      button.update( msg )


#-----------------------------------------------------#
#  cmd handler 
#-----------------------------------------------------#
class CmdHandler():
  
  def __init__( self, parent, name ):
    self.app_name = name
    self.parent = parent
    self.use_timer = self.parent.use_timer
    self.joystick = Joystick()
    self.initVariables()
    self.initCommonDefaults()
    self.initPublishers()

  def createJointMsg( self, num_joints ):
    joint_trajectory = JointTrajectory()
    joint_trajectory_point = JointTrajectoryPoint()
    for i in range( 0, num_joints ):
      joint_trajectory_point.positions.append( 0.0 )
      joint_trajectory.joint_names.append( 'j'+str( i ) )
      joint_trajectory_point.time_from_start = rospy.Duration( 0.0 )
    joint_trajectory.points.append( joint_trajectory_point )
    print '\t- create joint trajectory with ['+str( len( joint_trajectory.points[0].positions ) )+'] joints'
    return joint_trajectory

  def initVariables( self ):
    self.cmd = {}
    self.pub_cmd = {}
    self.pub_mode = rospy.Publisher( self.app_name+'/cmd/mode', String )
    print 'creating publishers and commands for app ['+self.app_name+'].'

  def initCommonDefaults( self ):
    self.use_deadman = True
    self.current_space = 'ts'
    self.current_mode = 'joystick'
    self.prefix = '[core handler]'

  def initPublishers( self ):
    pass

  #  mode 
  #-----------------------------------------------#
  def setMode( self, mode ):
    self.current_mode = mode
    self.pub_mode.publish( mode )
    print self.prefix+'change mode ['+self.app_name+'] to ['+str( mode )+']'

  #  control space
  #-----------------------------------------------#
  def setSpace( self, space ):
    self.current_space = space 
    print self.prefix+'change space ['+self.app_name+'] to ['+str( space )+']'

  #  publish cmd 
  #-----------------------------------------------#
  def publishCmd( self ):
    cmd = self.cmd[self.current_space]
    publisher = self.pub_cmd[self.current_space]
    publisher.publish( cmd )

  #  parse 
  #-----------------------------------------------#
  def handleJoystick( self, msg ):
    self.joystick.parse( msg )

