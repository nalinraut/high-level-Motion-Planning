#!/usr/bin/env python
#
# RightHand Robotics code for interfacing with components of the ReFlex SF hand
#

from dynamixel_msgs.msg import JointState
from dynamixel_controllers.srv import TorqueEnable
import rospy
from std_msgs.msg import Float64


class Motor(object):
    def __init__(self, name):
        '''
        Assumes that "name" is the name of the controller with a preceding
        slash, e.g. /reflex_sf_f1
        '''
        self.name = name[1:]
        self.zero_point = rospy.get_param(self.name + '/zero_point')
        self.angle_range = rospy.get_param(self.name + '/angle_range')
        self.flipped = rospy.get_param(self.name + '/flipped')
        self.current_raw_position = 0.0
        self.current_position = 0.0
        self.current_goal_position = 0.0
        self.pub = rospy.Publisher(name + '/command', Float64, queue_size=10)
        self.torque_enable_service = rospy.ServiceProxy(name + '/torque_enable',
                                                        TorqueEnable)
        self.torque_enabled = True
        self.sub = rospy.Subscriber(name + '/state', JointState,
                                    self.receiveStateCb)
        self.subscriber_hook = None

    def setMotorZeroPoint(self):
        self.zero_point = self.current_raw_position
        rospy.set_param(self.name + '/zero_point', self.current_raw_position)

    def getRawCurrentPosition(self):
        return self.current_raw_position

    def getCurrentPosition(self):
        return self.current_position

    def getCurrentGoalPosition(self):
        return self.current_goal_position

    def setRawMotorPosition(self, goal_pos):
        '''
        Sets the given position to the motor
        '''
        self.pub.publish(goal_pos)

    def setMotorPosition(self, goal_pos):
        '''
        Bounds the given motor command and sets it to the motor
        '''
        self.pub.publish(self.checkMotorCommand(goal_pos))

    def checkMotorCommand(self, angle_command):
        '''
        Returns the given command if it's within the allowable range, returns
        the bounded command if it's out of range
        '''
        angle_command = self.correctMotorOffset(angle_command)
        if self.flipped:
            bounded_command = max(min(angle_command, self.zero_point),
                                  self.zero_point - self.angle_range)
        else:
            bounded_command = min(max(angle_command, self.zero_point),
                                  self.zero_point + self.angle_range)
        return bounded_command

    def correctMotorOffset(self, angle_command):
        '''
        Adjusts for the zero point offset
        '''
        if self.flipped:
            return self.zero_point - angle_command
        else:
            return self.zero_point + angle_command

    def enableTorque(self):
        self.torque_enabled = True
        self.torque_enable_service(True)

    def disableTorque(self):
        self.torque_enabled = False
        self.torque_enable_service(False)

    def tighten(self, tighten_angle=0.05):
        '''
        Takes the given angle offset in radians and tightens the motor
        '''
        if self.flipped:
            tighten_angle *= -1
        self.setRawMotorPosition(self.current_raw_position + tighten_angle)

    def loosen(self, loosen_angle=0.05):
        '''
        Takes the given angle offset in radians and loosens the motor
        '''
        if self.flipped:
            loosen_angle *= -1
        self.setRawMotorPosition(self.current_raw_position - loosen_angle)

    def receiveStateCb(self, data):
        self.current_raw_position = data.current_pos
        if self.flipped:
            self.current_position = self.zero_point - self.current_raw_position
            self.current_goal_position = self.zero_point - data.goal_pos
        else:
            self.current_position = self.current_raw_position - self.zero_point
            self.current_goal_position = data.goal_pos - self.zero_point
        if self.subscriber_hook != None:
            self.subscriber_hook(self.name,data)
