CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

PROJECT(kinect2_emitter)
SET(CMAKE_BUILD_TYPE RelWithDebInfo)

SET(PROTONECT_DIR "../protonect")
SET(MY_DIR ${kinect2_emitter_SOURCE_DIR})

OPTION(ENABLE_CXX11 "Enable C++11 support" OFF)
OPTION(ENABLE_OPENCL "Enable OpenCL support" ON)

IF(ENABLE_CXX11)
  INCLUDE(CheckCXXCompilerFlag)
  CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
  CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
  IF(COMPILER_SUPPORTS_CXX11)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  ELSEIF(COMPILER_SUPPORTS_CXX0X)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
  ELSE()
    MESSAGE(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
  ENDIF()
ENDIF(ENABLE_CXX11)

# additional cmake modules
LIST(APPEND CMAKE_MODULE_PATH ${PROTONECT_DIR}/cmake_modules)

# setup threading
INCLUDE(SetupLibfreenect2Threading)
INCLUDE_DIRECTORIES(${LIBFREENECT2_THREADING_INCLUDE_DIR})

INCLUDE(GenerateResources)

#set the default path for built executables to the "bin" directory
SET(EXECUTABLE_OUTPUT_PATH ${MY_DIR}/bin)


#dependencies
INCLUDE_DIRECTORIES(${PROTONECT_DIR}/../../depends/glfw_src/include/)

if(APPLE)
  # libjpeg-turbo
  INCLUDE_DIRECTORIES("${MY_DIR}/../../depends/libjpeg_turbo/include/")
  LINK_DIRECTORIES("${MY_DIR}/../../depends/libjpeg_turbo/lib/")
endif()


# dependencies
FIND_PACKAGE(OpenCV REQUIRED)

# LibUSB
LINK_DIRECTORIES("${MY_DIR}/../../depends/libusb/lib/")

# GLEW
if (APPLE)
  LINK_DIRECTORIES("${MY_DIR}/../../depends/glew/lib/")
else()
  LINK_DIRECTORIES("${MY_DIR}/../../depends/glew/lib64/")
endif()

# GLFW
SET(BUILD_SHARED_LIBS ON CACHE BOOL "Build shared libraries")
SET(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "Build the GLFW example programs")
SET(GLFW_BUILD_TESTS OFF CACHE BOOL "Build the GLFW test programs")
SET(GLFW_BUILD_DOCS OFF CACHE BOOL "Build the GLFW documentation")

# libusb
LINK_DIRECTORIES("${MY_DIR}/../../depends/libusb/lib/")

LIST(APPEND LIBRARIES
  usb-1.0
  ${OpenCV_LIBS}
  turbojpeg
  GLEWmx
  glfw
  ${GLFW_LIBRARIES}
)


#ROS defines
SET(ROS_VERSION groovy CACHE STRING "ROS version")
IF(ROS_VERSION STREQUAL "")
ELSE(ROS_VERSION STREQUAL "")
SET(ROS_PATH /opt/ros/${ROS_VERSION})
FIND_LIBRARY(ROSCPPLIB NAMES roscpp PATHS ${ROS_PATH}/lib)
FIND_LIBRARY(ROSCONSOLELIB NAMES rosconsole PATHS ${ROS_PATH}/lib)
FIND_LIBRARY(ROSTIMELIB NAMES rostime PATHS ${ROS_PATH}/lib)
FIND_LIBRARY(ROSCPP_SERIALIZATIONLIB NAMES roscpp_serialization PATHS ${ROS_PATH}/lib)
SET(ROS_LIBRARIES ${ROSCPPLIB} ${ROSCONSOLELIB} ${ROSTIMELIB} ${ROSCPP_SERIALIZATIONLIB})
SET(ROS_WS /home/motion/ros_ws CACHE STRING "ROS workspace including Baxter SDK")
INCLUDE_DIRECTORIES(${ROS_PATH}/include ${ROS_WS}/install/include)
ENDIF(ROS_VERSION STREQUAL "")

INCLUDE_DIRECTORIES("{PROTONECT_DIR}/include")


ADD_EXECUTABLE(Kinect2_ROS_Emitter
  ros_emitter.cpp
)

ADD_DEPENDENCIES(Kinect2_ROS_Emitter libfreenect2)
  
TARGET_LINK_LIBRARIES(Kinect2_ROS_Emitter
  freenect2 ${ROS_LIBRARIES} ${LIBRARIES}
)


