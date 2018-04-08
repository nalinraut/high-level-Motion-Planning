cmake_minimum_required(VERSION 2.8.3)
project(ebolabot)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  reflex_sf
  baxter_interface
  hstar_amp
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
#catkin_python_setup()

## Generate messages in the 'msg' folder
# add_message_files(
#    FILES
#    Pose.msg
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#    DEPENDENCIES
#    std_msgs
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rrlasdlf
 CATKIN_DEPENDS std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
# include_directories(
#   ${catkin_INCLUDE_DIRS}
# )
