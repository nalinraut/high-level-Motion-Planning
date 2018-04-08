#!/usr/bin/env python

'''---------------------------------------------------------------------------

  desc:    input:  
           output: 
           
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
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

#=======================================================#
#  Joint Array - base class & sub-classes   
#=======================================================#
class JointArray(): 

  def __init__( self ):
    self.joint_msg = JointTrajectory()
  
  def createJointMsg( self, num_joints ):
    joint_trajectory = JointTrajectory()
    joint_trajectory_point = JointTrajectoryPoint()
    for i in range( 0, num_joints ):
      joint_trajectory_point.positions.append( 0.0 )
      joint_trajectory.joint_names.append( 'j'+str( i ) )
      joint_trajectory_point.time_from_start = rospy.Duration( 0.0 )
    joint_trajectory.points.append( joint_trajectory_point )
    print 'create joint trajectory with ['+str( len( joint_trajectory.points[0].positions ) )+'] joints'
    return joint_trajectory

  def fillHeader( self, msg ):
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'base_link' 


class Mecanum( JointArray ): 

  def __init__( self ):
    JointArray.__init__( self ) 
    self.joint_msg = self.createJointMsg( 4 )

  def convertTsToJs( self, msg ):
    self.fillHeader( self.joint_msg )
    x = msg.linear.x
    y = msg.linear.y
    z = msg.angular.z
    self.joint_msg.points[0].positions[0] = x - y - z
    self.joint_msg.points[0].positions[1] = x + y + z 
    self.joint_msg.points[0].positions[2] = x - y + z 
    self.joint_msg.points[0].positions[3] = x + y - z
    return self.joint_msg


class Diff( JointArray ): 

  def __init__( self ):
    JointArray.__init__( self ) 
    self.joint_msg = self.createJointMsg( 2 )

  def convertTsToJs( self, msg ):
    x = msg.linear.x
    z = msg.angular.z
    #z = msg.linear.y
    self.joint_msg.points[0].positions[0] = x - z
    self.joint_msg.points[0].positions[1] = x + z
    return self.joint_msg


class TsToJsConverter:

  def __init__( self, app_type ):
    # params
    self.app_types_map = {}
    self.app_types_map['mecanum'] = Mecanum
    self.app_types_map['diff'] = Diff 
    self.app_types_map['joint array'] = JointArray 
    self.app_type = app_type
    self.converter = self.app_types_map[app_type]()

  def convert( self, msg ):
    return self.converter.convertTsToJs( msg )
  
