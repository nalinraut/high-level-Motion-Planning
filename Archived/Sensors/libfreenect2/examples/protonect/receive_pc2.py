#!/usr/bin/env python
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from roslib import message
import random

#listener
def listen():
    rospy.init_node('listen', anonymous=True)
    rospy.Subscriber("/kinect2/pc", PointCloud2, callback_kinect)
    rospy.spin()

def callback_kinect(data) :
    data_out = pc2.read_points(data, field_names=None, skip_nans=False)
    print random.choice(list(data_out))



if __name__ == '__main__':
    try:
        listen()
    except rospy.ROSInterruptException:
        pass