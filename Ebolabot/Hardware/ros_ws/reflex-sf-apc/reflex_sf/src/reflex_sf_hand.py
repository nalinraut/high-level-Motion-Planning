#!/usr/bin/env python
#
# RightHand Robotics code for interfacing with a ReFlex SF hand
#

from os.path import join
import yaml

from dynamixel_msgs.msg import JointState
import rospy
import rospkg
from std_msgs.msg import Float64

from reflex_sf_msgs.msg import Command,JointState
from motor import Motor
import time

class ReflexSFHand(object):
    def __init__(self,name='hand1'):
        prefix = 'reflex_'+name
        self.prefix = prefix
        rospy.init_node(prefix)
        rospy.loginfo('Starting up the ReFlex SF hand')
        self.motors = dict((prefix+suffix,Motor('/'+prefix+suffix)) for suffix in ['_f1','_f2','_f3','_preshape'])
        #attach callbacks to motor data
        for motor in self.motors:
            self.motors[motor].subscriber_hook = lambda name,data:self.motorReceiveCb(name,data)
        rospy.Subscriber('/'+prefix+'/command', Command, self.receiveCmdCb)
        self.state = JointState()
        self.state.header.seq = 0
        self.state.header.stamp = rospy.Time.now()
        self.state.name = '/'+prefix
        self.state.motor_ids = [-1]*4;
        self.state.motor_temps = [-1]*4
        self.state.goal_pos = [0]*4
        self.state.current_pos = [0]*4
        self.state.error = [0]*4
        self.state.velocity = [0]*4
        self.state.load = [0]*4
        self.state.is_moving = False
        self.pub = rospy.Publisher('/' + prefix + '/state', JointState, queue_size=10)
        rospy.loginfo('ReFlex SF hand has started, waiting for commands...')

    def receiveCmdCb(self, data):
        self.motors[self.prefix+'_f1'].setMotorPosition(data.position[0])
        self.motors[self.prefix+'_f2'].setMotorPosition(data.position[1])
        self.motors[self.prefix+'_f3'].setMotorPosition(data.position[2])
        self.motors[self.prefix+'_preshape'].setMotorPosition(data.position[3])
        #TODO: force thresholds

    def motorReceiveCb(self,name,data):
        motor = self.motors[name]
        prefix = self.prefix
        name_to_index = {prefix+"_f1":0,prefix+"_f2":1,prefix+"_f3":2,prefix+"_preshape":3}
        index = name_to_index[name]
        if len(data.motor_ids) >= 1:
            self.state.motor_ids[index] = data.motor_ids[0]
        if len(data.motor_temps) >= 1:
            self.state.motor_temps[index] = data.motor_temps[0]
        self.state.goal_pos[index] = motor.current_position
        self.state.current_pos[index] = motor.current_goal_position
        self.state.error[index] = data.error if not motor.flipped else -data.error
        self.state.velocity[index] = data.velocity if not motor.flipped else -data.velocity
        self.state.load[index] = data.load
        if data.is_moving: self.state.is_moving = True
        self.state.header.stamp = data.header.stamp
        if index == 3:
            #send joint state
            self.pub.publish(self.state)
            self.state.header.seq += 1
            self.state.is_moving = False

    def printMotorPositions(self):
        print ""  # visually separates following print messages in the flow
        for motor in sorted(self.motors):
            print motor, " position: ", self.motors[motor].getCurrentPosition()

    def calibrate(self):
        calibrating=[1,1,1]
        count=[10,10,10]
        step =0
        while any(calibrating) and step < 20:
            step=step+1
            i=0
            for motor in sorted(self.motors):
                if i>2:
                    break
                if calibrating[i]:
                    rospy.loginfo("Calibrating motor " + motor)
                    print "loosing motor " + motor
                    print "motor load=",self.state.load[i]
                    self.motors[motor].loosen()
                    if self.state.load[i]*self.motors[motor].calibration_flip>self.motors[motor].calibration_threshold:
                        count[i]=count[i]-1
                        if count[i]==0:
                            calibrating[i]=0
                            print motor+" finished!"
                            print count
                    else:
                        count[i]=2
                i=i+1
            time.sleep(0.5)
       

        calibrating=[1,1,1]
        count=[0,0,0]
        step =0
        while any(calibrating) and step < 30:
            step=step+1
            i=0
            for motor in sorted(self.motors):
                if i>2:
                    break
                if calibrating[i]:
                    rospy.loginfo("Calibrating motor " + motor)
                    print "Tightening motor " + motor
                    print "motor load=",self.state.load[i]
                    self.motors[motor].tighten()
                    if self.state.load[i]*self.motors[motor].calibration_flip<self.motors[motor].calibration_threshold:
                        count[i]=count[i]+1
                        if count[i]>self.motors[motor].calibration_count:
                            calibrating[i]=0
                            print motor+" finished!"
                            print count
                            rospy.loginfo("Saving current position for %s as the zero point",motor)
                            self.motors[motor].setMotorZeroPoint()
                i=i+1
            time.sleep(0.3)
        print "Calibration complete, writing data to file"
        self.writeCurrentPositionsToZero()


    def writeZeroPointDataToFile(self, filename, data):
        rospack = rospkg.RosPack()
        reflex_sf_path = rospack.get_path('reflex_sf')
        yaml_path = "yaml"
        file_path = join(reflex_sf_path, yaml_path, filename)
        with open(file_path, "w") as outfile:
            outfile.write(yaml.dump(data))

    def writeCurrentPositionsToZero(self):
        data = dict()
        for item in ['_f1','_f2','_f3','_preshape']:
            data[self.prefix+item] = dict(
                zero_point=self.motors[self.prefix+item].getRawCurrentPosition()
            )
        self.writeZeroPointDataToFile(self.prefix+'_zero_points.yaml', data)

    def disableTorque(self):
        for motor in self.motors:
            self.motors[motor].disableTorque()

    def enableTorque(self):
        for motor in self.motors:
            self.motors[motor].enableTorque()


def main(handname):
    hand = ReflexSFHand(handname)
    rospy.on_shutdown(hand.disableTorque)
    rospy.spin()


if __name__ == '__main__':
    import sys
    if len(sys.argv) <= 1:
        print "USAGE: rosrun reflex_sf reflex_sf_hand.py NAME"
        exit(0)
    main(sys.argv[1])
