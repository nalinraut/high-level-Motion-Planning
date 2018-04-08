# reflex_sf

Interface code for ReFlex SF by RightHand Robotics

This is a simple wrapper on the ROS dynamixel-controller package that provides:
* a script to calibrate the tendons and motors, setting the zero-point
* a simple node that sets the pose (finger1, finger2, finger3, preshape) based on a message to the /reflexsf/command topic

Naming convention for fingers:
* f1 = right-hand index finger
* f2 = right-hand middle finger
* f3 = thumb
* preshape

0 = fully-open


