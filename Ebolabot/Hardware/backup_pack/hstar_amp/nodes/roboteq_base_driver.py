#!/usr/bin/env python

'''---------------------------------------------------------------------------

  desc:    input: base output cmd ( from a cmd mux )
           input: driver status, feedback
           output: roboteq_driver/j<n>/cmd
           output: /tf, odom, base wheel counts 
           
  author:  ks
  created: 11/27/14

---------------------------------------------------------------------------'''

import roslib
import rospy, sys
import tf
import math

# ros common msgs
from std_msgs.msg import String
from std_msgs.msg import Float32, Float64MultiArray
from geometry_msgs.msg import Twist 
from sensor_msgs.msg import JointState 
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# ros 3rd party msgs
from roboteq_msgs.msg import Command, Feedback, Status, Configuration

NODE_NAME='roboteq_base_driver'

#=======================================================#
#   Joint
#=======================================================#
class Joint:

  def __init__( self, index, base ):
    self.index = index
    self.pos = 0.0
    self.pos_last = None 
    self.pos_diff = 0.0
    self.vel = 0.0
    self.enc_count = 0 
    self.hall_count = 0 
    self.parent = base
    self.app = base.parent
    self.time_last_update = rospy.Time.now()

    # commands
    self.motor_cmd = 0
    self.prefix = 'j'+str( self.index )

    # config driver
    self.loadConfig()

    # set params
    self.enc_type = base.parent.enc_types[index]
    self.motor_gain = base.parent.motor_gain
    self.motor_dir = base.parent.motor_dirs[index]
    self.enc_dir = base.parent.enc_dirs[index] 
    #if ( self.enc_type == 'encoder' ):
    #  self.enc2m = base.parent.enc2m
    #elif ( self.enc_type == 'hall' ):
    #  self.enc2m = base.parent.hall2m
    self.enc2rad = base.parent.enc2rad
    self.hall2rad = base.parent.hall2rad

    self.sub_motor = rospy.Subscriber( 'roboteq_driver/j'+str( self.index )+'/feedback', Feedback, self.feedbackCallback )
    self.pub_motor = rospy.Publisher( 'roboteq_driver/j'+str( self.index )+'/cmd', Command )

    if ( self.index == 0 ):
      self.pub_battery = rospy.Publisher( 'base/battery_voltage', Float32 )

  #  get config from param server key
  #-----------------------------------------------------#
  def loadConfig( self ):

    if self.app.use_init_config:

      print '\tget init config from param server. '

      # use the individual joint config ( if exists )
      if self.app.use_joint_config:
      
        print '\t\tusing joint config'

        # look for custom config first
        param_key = 'joint_configs/'+self.prefix+'/config'
        param_lookup_key = 'joint_configs/'+self.prefix+'/config-lookup'

        if rospy.has_param( param_key ):
          print '\t\t\tusing param key ['+param_key+']'
          self.getConfigFromParamServer( param_key )

        # if doesn't exist, then look up by joint config by name
        elif rospy.has_param( param_lookup_key ):

          print '\t\t\tusing param lookup key ['+param_lookup_key+']'

          # get the name of the param to look for
          joint_lookup_key = rospy.get_param( param_lookup_key )
          print '\t\t\tusing joint config ['+joint_lookup_key+']'
          param_joint_lookup_key = 'joint_configs/'+joint_lookup_key

          if rospy.has_param( param_joint_lookup_key ):
            self.getConfigFromParamServer( param_joint_lookup_key )

      # or use the common one defined for application ( only works if all the same motor )
      else:
        if hasattr( self.app, 'config_msg' ):
          self.setConfig( self.parent.config_msg )


  #  get config from param server key
  #-----------------------------------------------------#
  def getConfigFromParamServer( self, param_key ):
    print '\t\t\tgetting config from param server by key ['+param_key+']'
    config_msg = Configuration()
    config_param = rospy.get_param( param_key )
    for key, val in config_param.iteritems():
      print '\t\t\t- [j'+str( self.index )+'] set ['+key+']: to '+str( val )
      setattr( config_msg, key, val )
    self.setConfig( config_msg )

  #  config
  #-----------------------------------------------------#
  def setConfig( self, config_msg ):
    print '\t['+self.prefix+'] setting config'
    self.pub_config = rospy.Publisher( 'roboteq_driver/j'+str( self.index )+'/cfg', Configuration )
    rospy.sleep( 1 )
    self.pub_config.publish( config_msg )
    print '\t['+self.prefix+'] sent config'

  #  command 
  #-----------------------------------------------------#
  def setCommand( self, data ):
    self.motor_cmd = data * self.motor_dir * self.motor_gain
    self.pub_motor.publish( self.motor_cmd )
  
  #  odometry 
  #-----------------------------------------------------#
  def update( self, data, sensor2rad ):
    self.pos = data * self.enc_dir * sensor2rad
    if self.pos_last is None:
      self.pos_last = self.pos 
    #self.pos = data

  def getDiff( self ):
      #self.pos_diff = ( self.pos - self.pos_last ) * self.enc_dir * self.enc2m
      self.pos_diff = self.pos - self.pos_last
      self.pos_last = self.pos
      dt = ( rospy.Time.now() - self.time_last_update ).to_sec()
      self.time_last_update = rospy.Time.now()
      self.vel = self.pos_diff / dt
      return self.pos_diff 

  #  motor driver feedback 
  #-----------------------------------------------------#
  def feedbackCallback( self, msg ):
    self.enc_count = msg.measured_position 
    self.hall_count = msg.measured_position_hall
    if ( self.enc_type == 'hall' ):
      self.update( msg.measured_position_hall, self.hall2rad )
    elif ( self.enc_type == 'encoder' ):
      self.update( msg.measured_position, self.enc2rad )
    if ( self.index == 0 ):
      self.pub_battery.publish( msg.supply_voltage )
      if self.parent.isReady():
        self.parent.update()
  



#=======================================================#
#   Base 
#=======================================================#
class Base:

  def createJointMsg( self, num_joints ):
    joint_trajectory = JointTrajectory()
    joint_trajectory_point = JointTrajectoryPoint()
    for i in range( 0, num_joints ):
      joint_trajectory_point.positions.append( 0.0 )
      joint_trajectory_point.velocities.append( 0.0 )
      joint_trajectory.joint_names.append( 'j'+str( i ) )
      joint_trajectory_point.time_from_start = rospy.Duration( 0.0 )
    joint_trajectory.points.append( joint_trajectory_point )
    print '\t- created joint trajectory with ['+str( len( joint_trajectory.points[0].positions ) )+'] joints'
    return joint_trajectory

  def createArrayMsg( self, num ):
    msg = Float64MultiArray()
    for i in range( 0, num ):
      msg.data.append( 0.0 )
    print '\t- created multi array of length ['+str( len( msg.data ) )+'].'
    return msg 


  def __init__( self, parent ):
    self.x = 0
    self.y = 0
    self.z = 0
    self.x_diff = 0
    self.y_diff = 0
    self.z_diff = 0
    self.vel_x = 0
    self.vel_y = 0
    self.vel_z = 0
    self.parent = parent
    self.drive_type = parent.base_type
    self.publish_tf = parent.publish_tf
    self.publish_wheel_tf = parent.publish_wheel_tf
    if ( self.drive_type == 'diff' ):
      self.num_joints = 2
    elif ( self.drive_type == 'mecanum' ):
      self.num_joints = 4
    print '\tcreated ['+str( self.drive_type )+'] base with ['+str( self.num_joints )+'] joints'

    self.rad2m = self.parent.wheel_dia / 2.0 

    # setup joint_states publisher
    if self.publish_wheel_tf:
      self.joint_state_msg = JointState()
      self.pub_joint_states = rospy.Publisher( 'joint_states', JointState )
      print 'get wheel transforms ...'
      for i in range( 0, self.num_joints ):
        self.joint_state_msg.name.append( 'wheel'+str(i) )
        self.joint_state_msg.position.append( 0.0 )

    # publishers
    self.pub_joint_traj = rospy.Publisher( 'base/joints', JointTrajectory )
    self.pub_enc_count = rospy.Publisher( 'base/joints/encoder_count', Float64MultiArray )
    self.pub_hall_count = rospy.Publisher( 'base/joints/hall_count', Float64MultiArray )

    # joint traj msg
    self.joint_msg = self.createJointMsg( self.num_joints )
    self.hall_count_msg = self.createArrayMsg( self.num_joints )
    self.enc_count_msg = self.createArrayMsg( self.num_joints )

    # param
    self.wheelbase_width = parent.wheelbase_width
  
    # tf broadcaster
    self.br = tf.TransformBroadcaster()
    self.odom = Odometry()
    self.odom.header.frame_id = 'odom'
    self.odom.child_frame_id = 'base_footprint'
    self.odom.twist.twist.linear.z = 0
    self.odom.twist.twist.angular.x = 0
    self.odom.twist.twist.angular.y = 0

    # odom pub
    self.pub_odom = rospy.Publisher( 'odom', Odometry )

    self.time_last_update = rospy.Time.now()
    self.time_last_debug = rospy.Time.now()

    self.frame = 0

    # subscribe to feedback
    self.joints = []
    for i in range( 0, self.num_joints ):
      joint = Joint( i, self )
      self.joints.append( joint )


  #  commands
  #-----------------------------------------------------#
  def updateJointMsg( self ):
    
    # update traj msg
    for i in range( 0, len( self.joints ) ):
      self.joint_msg.points[0].positions[i] = self.joints[i].pos
      self.joint_msg.points[0].velocities[i] = self.joints[i].vel
      self.hall_count_msg.data[i] = self.joints[i].hall_count 
      self.enc_count_msg.data[i] = self.joints[i].enc_count 

    if self.publish_wheel_tf:
      self.publishWheelTf()

    # publish
    if self.parent.publish_enc_count:
      self.pub_enc_count.publish( self.enc_count_msg )
    if self.parent.publish_hall_count:
      self.pub_hall_count.publish( self.hall_count_msg )
    if self.parent.publish_joint_trajectory:
      self.pub_joint_traj.publish( self.joint_msg )

  #  wheel tf
  #-----------------------------------------------------#
  def publishWheelTf( self ):
    for i in range( 0, len( self.joints ) ):
      self.joint_state_msg.header.frame_id = 'base_link'
      self.joint_state_msg.position[i] = -self.joints[i].pos
      self.joint_state_msg.header.stamp = rospy.Time.now()
      self.pub_joint_states.publish( self.joint_state_msg )

  #  commands
  #-----------------------------------------------------#
  def setMotorCommands( self, msg ):
    for i in range( 0, len( self.joints ) ):
      self.joints[i].setCommand( msg.points[0].positions[i] ) 

  def isReady( self ):
    ready = True
    for i in range( 0, len( self.joints ) ):
      if self.joints[i].pos_last is None:
        ready = False
        #print 'initialized joint ['+str( i )+']\n'
    return ready


  #  update pose 
  #-----------------------------------------------------#
  def update( self ):

    # update position and orientation

    # diff
    if ( self.drive_type == 'diff' ):
      self.x_diff = ( self.joints[0].pos_diff + self.joints[1].pos_diff ) / 2.0
      self.z_diff = -( self.joints[0].pos_diff - self.joints[1].pos_diff ) / self.wheelbase_width

    # mecanum 
    elif ( self.drive_type == 'mecanum' ):
      d = [ 0, 0, 0, 0 ] 
      for i in range( 0, len( self.joints ) ):
        d[i] = self.joints[i].getDiff() * self.rad2m
      self.x_diff = ( d[0] + d[1] + d[2] + d[3] ) / 4.0
      self.y_diff = ( -d[0] + d[1] - d[2] + d[3] ) / 4.0 / 1.10
      self.z_diff = ( -d[0] + d[1] + d[2] - d[3] ) / 4.0 * math.pi / 1.81

      # debug 
      #if ( self.frame % 20 < 4 ):
        #print '{:1.2f}, {:1.2f}, {:1.2f}'.format( self.x, self.y, self.z )
        #print '( '+str(d[0])+', '+str(d[1])+', '+str(d[2])+', '+str(d[3])+' )' 
      
    # update yaw
    self.z = self.z + self.z_diff
    self.oquat = tf.transformations.quaternion_from_euler( 0, 0, self.z )
    
    # update odometric pose
    self.x = self.x + self.x_diff * math.cos( self.z ) - self.y_diff * math.sin( self.z )
    self.y = self.y + self.x_diff * math.sin( self.z ) + self.y_diff * math.cos( self.z )

    # debug
    if ( self.parent.debug ):
      dt_debug = ( rospy.Time.now() - self.time_last_debug ).to_sec()
      if ( dt_debug > 1.0 / self.parent.debug_print_rate ):
        print '{:1.2f}, {:1.2f}, {:1.2f}'.format( self.x, self.y, self.z )
        self.time_last_debug = rospy.Time.now()

    # update velocity
    dt = ( rospy.Time.now() - self.time_last_update ).to_sec()
    self.vel_x = self.x_diff / dt
    self.vel_y = self.y_diff / dt
    self.vel_z = self.z_diff / dt
    self.time_last_update = rospy.Time.now()

    # publish odom & tf transform
    if self.publish_tf:
      self.publishTf()

    self.updateJointMsg()

    self.publishOdometry()
  
    self.frame = self.frame + 1 

  #  publish 
  #-----------------------------------------------------#
  def publishOdometry( self ):
    self.odom.header.stamp = rospy.Time.now()
    self.odom.pose.pose.position.x = self.x
    self.odom.pose.pose.position.y = self.y
    self.odom.pose.pose.position.z = 0.0
    self.odom.pose.pose.orientation.x = self.oquat[0]
    self.odom.pose.pose.orientation.y = self.oquat[1]
    self.odom.pose.pose.orientation.z = self.oquat[2]
    self.odom.pose.pose.orientation.w = self.oquat[3]
    self.odom.twist.twist.linear.x = self.vel_x
    self.odom.twist.twist.linear.y = self.vel_y
    self.odom.twist.twist.angular.z = self.vel_z
    self.odom.twist.twist.linear.x = 0 
    self.odom.twist.twist.linear.y = 0 
    self.odom.twist.twist.angular.z = 0 

    # need covariance if using ekf
    self.odom.pose.covariance[0] = 0.001 
    self.odom.pose.covariance[7] = 0.001 
    self.odom.pose.covariance[14] = 0.000001 
    self.odom.pose.covariance[21] = 0.000001 
    self.odom.pose.covariance[28] = 0.000001 
    self.odom.pose.covariance[35] = 0.001 
    self.odom.twist.covariance[0] = 0.001 
    self.odom.twist.covariance[7] = 0.001 
    self.odom.twist.covariance[14] = 0.000001 
    self.odom.twist.covariance[21] = 0.000001 
    self.odom.twist.covariance[28] = 0.000001 
    self.odom.twist.covariance[35] = 0.001 
  
    self.pub_odom.publish( self.odom )

  #  tf
  #-----------------------------------------------------#
  def publishTf( self ):
    self.br.sendTransform( ( self.x, self.y, 0.0 ), self.oquat, rospy.Time.now(), 'base_footprint', 'odom' )


#-----------------------------------------------------#
#  
#-----------------------------------------------------#
class ROSNode:

  #  constructor 
  #-----------------------------------------------------#
  def __init__(self):

    rospy.init_node( NODE_NAME )

    # params
    self.base_type  = rospy.get_param( '~base_type', 'diff' )
    self.publish_tf  = rospy.get_param( '~publish_tf', False )
    self.publish_wheel_tf  = rospy.get_param( '~publish_wheel_tf', False )
    self.publish_joint_trajectory  = rospy.get_param( '~publish_joint_trajectory', True )
    self.publish_hall_count  = rospy.get_param( '~publish_hall_count', True )
    self.publish_enc_count  = rospy.get_param( '~publish_enc_count', True )
    self.motor_gain  = rospy.get_param( '~motor_gain', 1 )
    self.motor_dirs  = rospy.get_param( '~motor_dirs', [ 1, -1, -1, 1 ] )
    self.enc_dirs  = rospy.get_param( '~enc_dirs', [ 1, -1, -1, 1 ] )
    self.enc_types = rospy.get_param( '~enc_types', [ 'hall', 'hall', 'hall', 'hall' ] )
    self.enc2m  = rospy.get_param( '~enc2m', 1 )
    self.hall2m  = rospy.get_param( '~hall2m', 1 )
    self.wheel_dia = rospy.get_param( '~wheel_dia', 1 )
    self.enc2rad  = rospy.get_param( '~enc2rad', 1 )
    self.hall2rad  = rospy.get_param( '~hall2rad', 1 )
    self.wheelbase_width = rospy.get_param( '~wheelbase_width', 0.6 ) 
    self.debug = rospy.get_param( '~debug', False ) 
    self.debug_print_rate = rospy.get_param( '~debug_print_rate', 1.0 ) 
    self.use_init_config = rospy.get_param( '~use_init_config', False ) 
    self.use_joint_config = rospy.get_param( '~use_joint_config', False ) 

    # handle config option
    if self.use_init_config and rospy.has_param( '~config' ):
      self.config = rospy.get_param( '~config' ) 
      print 'config param: '
      #print str( self.config ) 
      self.config_msg = Configuration()
      for key, val in self.config.iteritems():
        print '\t- set ['+key+']: to '+str( val )
        setattr(self.config_msg, key, val)

    # print params
    print 'params: '
    print '\t- base_type: ['+str(self.base_type)+']' 
    print '\t- debug: ['+str(self.debug)+']' 
    print '\t- debug_print_rate: ['+str(self.debug_print_rate)+']' 
    print '\t- type: ['+self.base_type+']' 
    print '\t- publish:'
    print '\t\t- odom tf: ['+str(self.publish_tf)+']' 
    print '\t\t- wheel tf: ['+str(self.publish_wheel_tf)+']' 
    print '\t\t- joint trajectory ['+str(self.publish_joint_trajectory)+']' 
    print '\t\t- hall count ['+str(self.publish_hall_count)+']' 
    print '\t\t- encoder count ['+str(self.publish_enc_count)+']' 
    print '\t- publish_odom_tf: ['+str(self.publish_tf)+']' 
    print '\t- motor_gain: ['+str(self.motor_gain)+']' 
    print '\t- enc_types: ['+str(self.enc_types)+']' 
    print '\t- enc2m: ['+str(self.enc2m)+']' 
    print '\t- enc2rad: ['+str(self.enc2rad)+']' 
    print '\t- hall2rad: ['+str(self.hall2rad)+']' 
    print '\t- wheel dia: ['+str(self.wheel_dia)+']' 
    print '\t- wheelbase width: ['+str(self.wheelbase_width)+']' 

    print '\t- motor directions: '
    for i in self.motor_dirs: 
      print '\t\t- '+str(self.motor_dirs[i])
    print '\t- encoder directions: ' 
    for i in self.enc_dirs: 
      print '\t\t- '+str(self.enc_dirs[i])
    print '\t- encoder types: ' 
    for i in self.enc_types: 
      print '\t\t- '+i

    # create base instance for odometry feedback
    self.base = Base( self )

    # sub
    #self.sub_cmd = rospy.Subscriber( 'base/output/cmd/ts', Twist, self.cmdCallback )
    self.sub_cmd = rospy.Subscriber( 'base/output/cmd/js', JointTrajectory, self.cmdCallback )

    print '\nInitialized ['+NODE_NAME+'].\n'


  #  base command 
  #-----------------------------------------------------#
  def cmdCallback( self, msg ):
    self.base.setMotorCommands( msg )



def main(args):
  
  print 'Starting ['+NODE_NAME+']'
  node = ROSNode() 
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down node."


if __name__ == '__main__':
  main(sys.argv)
