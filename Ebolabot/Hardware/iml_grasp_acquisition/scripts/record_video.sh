#!/bin/sh

if [ $# -eq 0 ]
  then
    echo "No experiment name supplied"
else 
name=$1
rosbag record -O $name /usb_cam/camera_info /usb_cam/image_raw 
fi
