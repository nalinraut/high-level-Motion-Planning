#!/bin/sh

if [ $# -eq 0 ]
  then
    echo "No experiment name supplied"
else 
name=$1
rosbag record -O $name /camera/rgb/camera_info /camera/rgb/image_raw /kinect2/sd/points /kinect2/sd/camera_info /kinect2/sd/image_color_rect /kinect2/qhd/image_color /kinect2/qhd/camera_info

# /usb_cam/camera_info /usb_cam/image_raw /tf /soft_hand/joint_states /soft_hand/joint_trajectory_controller/state /kinect2/sd/image_color_rect /ar_pose_marker 
fi
