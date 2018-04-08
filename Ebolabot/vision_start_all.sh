#!/bin/bash

kill_descendant_processes() {
    local pid="$1"
    local and_self="${2:-false}"
    if children="$(pgrep -P "$pid")"; then
        for child in $children; do
            kill_descendant_processes "$child" true
        done
    fi
    if [[ "$and_self" == true ]]; then
        kill -9 "$pid"
    fi
}

./RealSense_ROS_Emitter tcp://192.168.0.104:3457 left_realsense &
./RealSense_ROS_Emitter tcp://192.168.0.105:3457 right_realsense &
rosrun kinect2_bridge kinect2_bridge &


int_handler()
{
    echo "Interrupted."
    kill_descendant_processes $$
    exit 1
}
trap 'int_handler' INT

while true; do
    sleep 1
    #echo "I'm still alive!"
done