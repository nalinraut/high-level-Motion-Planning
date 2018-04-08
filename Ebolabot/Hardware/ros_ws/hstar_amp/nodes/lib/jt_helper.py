#!/usr/bin/env python

'''---------------------------------------------------------------------------

  desc:    input:  
           output: 
           
  author:  ks
  created: 12/12/14

---------------------------------------------------------------------------'''

import roslib
roslib.load_manifest( 'hstar_amp' )
import rospy, sys
import tf
import math

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def createJointMsg( num_joints, frame_id ):
  joint_trajectory = JointTrajectory()
  #joint_trajectory.header.frame_id = frame_id
  joint_trajectory.header.frame_id = "base_link" 
  joint_trajectory.header.stamp = rospy.Time.now()
  joint_trajectory_point = JointTrajectoryPoint()
  for i in range( 0, num_joints ):
    joint_trajectory_point.positions.append( 0.0 )
    joint_trajectory.joint_names.append( 'j'+str( i ) )
    joint_trajectory_point.time_from_start = rospy.Duration( 0.0 )
  joint_trajectory.points.append( joint_trajectory_point )
  print 'create joint trajectory with ['+str( len( joint_trajectory.points[0].positions ) )+'] joints'
  return joint_trajectory
