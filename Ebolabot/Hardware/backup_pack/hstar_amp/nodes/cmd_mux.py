#!/usr/bin/env python

'''---------------------------------------------------------------------------

  desc:    input:  <app>/<source>/cmd/<js|ts> 
           output: <app>/output/cmd/<js|ts> 
           
  author:  ks
  created: 11/29/14

---------------------------------------------------------------------------'''

import roslib
import rospy, sys
import tf
import math

# ros common msgs
from std_msgs.msg import String
from geometry_msgs.msg import Twist 
from trajectory_msgs.msg import JointTrajectory

from lib.watchdog import Watchdog

NODE_NAME='app_cmd_mux'

#=======================================================#
#   
#=======================================================#
class AppHandler:

  def __init__( self, app_type, control_mode, msg_type, parent ):
    self.app_type = app_type
    self.control_mode = control_mode 
    self.msg_type = msg_type
    self.parent = parent
    self.cmd_sources = parent.cmd_sources
    self.cmd_priority = {}
    self.cmd_priority['joystick'] = 3
    self.cmd_priority['ui'] = 2
    self.cmd_priority['auto'] = 1
    
    # init watchdog 
    self.watchdog = Watchdog()
    self.watchdog.setTimeout( 100 )
    self.watchdog.setValidCountMin( 0 )

    self.manual = True

    # params
    self.use_timer = parent.use_timer
    self.pub_rate = parent.pub_rate

    # timer
    if ( self.use_timer ):
      rospy.Timer( rospy.Duration( 1.0 / self.pub_rate ), self.timerCallback )

    # change mode
    self.sub_mode = rospy.Subscriber( self.app_type+'/cmd/mode', String, self.modeCallback )
    self.pub_cmd = rospy.Publisher( self.app_type+'/output/cmd/'+self.control_mode, self.msg_type )

    # ad-hoc
    '''if self.app_type == 'base' and self.control_mode == 'ts':
      print '\nuse extra callback\n'
      self.use_extra = True
    else:
      self.use_extra = False

    if self.use_extra:
      self.sub_extra = rospy.Subscriber( +self.app_type+'/extra/cmd/'+self.control_mode, Twist, self.extraCallback )
      self.cmd_extra = Twist()'''

    rospy.loginfo( 'created ['+app_type+'] handler for ['+self.control_mode+'] control.' )

    self.current_command = None 
    self.current_source = 'joystick' 

    # create handle for each source
    self.cmd_handlers = {} 
    for source in self.cmd_sources: 
      self.cmd_handlers[source] = CmdHandler( source, self ) 
      rospy.loginfo( '\t- created ['+source+'] command handler.' )


  #  timer
  #-----------------------------------------------------#
  def timerCallback( self, event ):
    for handler in self.cmd_handlers.itervalues():
      pass
      '''if not handler.watchdog.isValid():
        rospy.loginfo( 'command watchdog timeout ['+self.current_source+'] for ['+self.app_type+'] ('+str(handler.watchdog.sig)+')' )
        self.current_command = Twist()'''

    if self.current_command is not None:
      self.pub_cmd.publish( self.current_command )

  #  mode 
  #-----------------------------------------------------#
  def changeMode( self, mode ):
    if ( mode in self.cmd_sources ): 
      self.current_source = mode
      rospy.loginfo( 'change source to ['+self.current_source+'] for ['+self.app_type+'].' )
    else:
      rospy.logwarn( 'requested invalid source ['+requested_source+'] for ['+self.app_type+'].' )

  #  mode callback
  #-----------------------------------------------------#
  def modeCallback( self, msg ):
    self.changeMode( msg.data )

  #  extra callback
  #-----------------------------------------------------#
  def extraCallback( self, msg ):
    self.cmd_extra = msg
    #print 'got extra cmd: '+str(self.cmd_extra.linear.x)

  #  mux command 
  #-----------------------------------------------------#
  def muxCommand( self, msg, source ):
    
    #rospy.loginfo( 'mux command current ['+self.current_source+'] - ['+source+'].' )

    # if cmd from current source
    #if source is self.current_source: 
    if source == self.current_source: 
    
      #rospy.loginfo( ' - mux command current ['+self.current_source+'] - ['+source+'].' )
  
      self.current_command = msg

      #if self.use_extra:
      #  self.current_command.linear.x += self.cmd_extra.linear.x
   
      # if not using constant rate timer, then send off directly from callback
      if not self.use_timer: 
        self.pub_cmd.publish( self.current_command )

    # if different source, check if possible override
    else:

      # if higher priority, change mode
      if self.cmd_priority[source] > self.cmd_priority[self.current_source]:
        self.changeMode( source ) 
      else:
        # exceptions to priority rule go here
   
        # if only 1 priority lower, and higher priority mode timed out, then lower priority override  
        if self.cmd_priority[self.current_source] - self.cmd_priority[source] == 1:
          if not self.cmd_handlers[self.current_source].watchdog.isValid():
            self.changeMode( source )


#=======================================================#
#   
#=======================================================#
class CmdHandler:

  def __init__( self, source, parent ):
    self.source = source
    self.parent = parent
  
    # init watchdog
    self.watchdog = Watchdog()
    self.watchdog.setTimeout( 1.0 )
    self.watchdog.setValidCountMin( 0 )

    # sub
    self.sub_cmd = rospy.Subscriber( 'base/'+source+'/cmd/'+parent.control_mode, parent.msg_type, self.cmdCallback )

  #  command callback
  #-----------------------------------------------------#
  def cmdCallback( self, msg ):
    self.parent.muxCommand( msg, self.source )
    self.watchdog.update()




#=======================================================#
#   
#=======================================================#
class ROSNode:

  #  constructor 
  #-----------------------------------------------------#
  def __init__(self):

    rospy.init_node( NODE_NAME )

    # params
    self.cmd_sources  = rospy.get_param( '~cmd_sources', [ 'joystick', 'auto' ] )
    self.app_types = rospy.get_param( '~app_types', [ 'base' ] )
    self.use_timer = rospy.get_param( '~use_timer', True )
    self.pub_rate = rospy.get_param( '~publish_rate', 20.0 )

    print 'params: '
    print '\t- sources: ['+str(self.cmd_sources)+']'
    print '\t- apps: ['+str(self.app_types)+']'
    print '\t- use timer: ['+str(self.use_timer)+']'
    print '\t- publish rate: ['+str(self.pub_rate)+']'

    # create base instance for odometry feedback
    self.apps = {}
    #self.app_types = [ 'base', 'left_arm', 'right_arm' ]
    #self.control_modes = [ 'ts', 'js' ]
    self.control_modes = [ 'ts' ]
    self.msg_types = {}
    for app_type in self.app_types: 
      self.msg_types[app_type] = {}
    self.msg_types['base']['ts'] = Twist
    #self.msg_types['base']['js'] = JointTrajectory
    #self.msg_types['left_arm']['ts'] = Twist
    #self.msg_types['left_arm']['js'] = JointTrajectory
    #self.msg_types['right_arm']['ts'] = Twist
    #self.msg_types['right_arm']['js'] = JointTrajectory

    # create app handle for each application type
    for app_type in self.app_types: 
      self.apps[app_type] = {}
      for control_mode in self.control_modes: 
        msg_type = self.msg_types[app_type][control_mode]
        self.apps[app_type][control_mode] = AppHandler( app_type, control_mode, msg_type, self ) 

    print 'Initialized '+NODE_NAME+'.'



def main(args):
  
  print 'Starting ['+NODE_NAME+']'
  node = ROSNode() 
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down node."


if __name__ == '__main__':
  main(sys.argv)
