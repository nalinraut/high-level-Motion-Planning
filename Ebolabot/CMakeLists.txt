CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
SET(KLAMPT_ROOT "$ENV{HOME}/Klampt" CACHE PATH "Klamp't path")
SET(SSPP_ROOT "${CMAKE_SOURCE_DIR}/../SSPP"  CACHE PATH "Path to iml-internal/SSPP")
SET (CMAKE_MODULE_PATH "${KLAMPT_ROOT}/CMakeModules")
FIND_PACKAGE(Klampt REQUIRED)
ADD_DEFINITIONS(${KLAMPT_DEFINITIONS})
INCLUDE_DIRECTORIES(. ${KLAMPT_INCLUDE_DIRS} ${SSPP_ROOT}/include)

#for SSPP
SET(SSPP_LIBRARIES ${SSPP_ROOT}/lib/libsspp.a)

#for ebolabot
INCLUDE_DIRECTORIES(Motion)

#necessary for Python task controllers
find_package ( PythonLibs 2.7 REQUIRED )
IF(PYTHONLIBS_FOUND)
  SET(HAVE_PYTHON 1)
ELSE(PYTHONLIBS_FOUND)
  SET(HAVE_PYTHON 0)
ENDIF(PYTHONLIBS_FOUND)
INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_DIRS})

#old test service
#ADD_EXECUTABLE(ControllerService Controller/controller_service.cpp)
#TARGET_LINK_LIBRARIES(ControllerService ${KLAMPT_LIBRARIES} ${SSPP_LIBRARIES})

#motion library: client
ADD_LIBRARY(motion_client SHARED
		Motion/motion_client.cpp)
TARGET_LINK_LIBRARIES(motion_client ${KRISLIBRARY_LIBRARIES})

#motion library: kinematic
ADD_LIBRARY(motion_kinematic SHARED
		Motion/motion_kinematic.cpp)
TARGET_LINK_LIBRARIES(motion_kinematic ${SSPP_LIBRARIES} ${KLAMPT_LIBRARIES})

#motion library: physics simulation (not done yet)
#ADD_LIBRARY(motion_klamptsim SHARED
#		Motion/motion_klamptsim.cpp)
#TARGET_LINK_LIBRARIES(motion_klamptsim ${SSPP_LIBRARIES}) ${KLAMPT_LIBRARIES})


#motion server: virtual robot
ADD_EXECUTABLE(MotionServer_kinematic
		Motion/motion_server_kinematic.cpp Motion/motion_server.cpp)
TARGET_LINK_LIBRARIES(MotionServer_kinematic ${SSPP_LIBRARIES} ${KLAMPT_LIBRARIES})

#motion server: physics simulation (not done yet)
#ADD_EXECUTABLE(MotionServer_klamptsim
#		Motion/motion_server_klamptsim.cpp Motion/motion_server.cpp)
#TARGET_LINK_LIBRARIES(MotionServer_klamptsim ${SSPP_LIBRARIES}) ${KLAMPT_LIBRARIES})

#ebolabot library: real robot
SET (CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules")
SET(ROSDEPS rosconsole roscpp roscpp_serialization rostime )
FIND_PACKAGE(ROS)
IF(ROS_FOUND)
  MESSAGE("ROS FOUND! path ${ROS_PATH}")
  SET(ROS_WS $ENV{HOME}/ros_ws CACHE STRING "ROS workspace including Baxter SDK")
  INCLUDE_DIRECTORIES(${ROS_PATH}/include ${ROS_WS}/devel/include ${ROS_WS}/install/include)

  #motion server: physical robot
  ADD_EXECUTABLE(MotionServer_physical
    Motion/motion_server_physical.cpp Motion/motion_server.cpp)
  TARGET_LINK_LIBRARIES(MotionServer_physical ${ROS_LIBRARIES} ${SSPP_LIBRARIES} ${KLAMPT_LIBRARIES})

  #motion library: physical robot
  ADD_LIBRARY(motion_physical SHARED
    Motion/motion_physical.cpp)

  TARGET_LINK_LIBRARIES(motion_physical ${ROS_LIBRARIES} ${SSPP_LIBRARIES} ${KLAMPT_LIBRARIES} )

  FIND_PACKAGE(JPEG)
  INCLUDE_DIRECTORIES(${LIBJPEG_INCLUDE_DIR} )
  FIND_PACKAGE(ZLIB)
  INCLUDE_DIRECTORIES(${ZLIB_INCLUDE_DIR})
  FIND_PACKAGE(PNG)
  INCLUDE_DIRECTORIES(${LIBPNG_INCLUDE_DIR})

  #FIND_PACKAGE(FFmpeg REQUIRED COMPONENTS avutil avcodec avformat swscale swresample)
  #INCLUDE_DIRECTORIES(${FFMPEG_INCLUDE_DIRS})

  IF(LIBJPEG_FOUND AND LIBPNG_FOUND AND ZLIB_FOUND)
    MESSAGE("LIBJPEG, ZLIB, and LIBPNG FOUND SUCCESS - proceeding with jpeg compilation")
    ADD_EXECUTABLE(RealSense_ROS_Emitter Sensors/realsense_ros_emitter.cpp)
    TARGET_LINK_LIBRARIES(RealSense_ROS_Emitter ${KRISLIBRARY_LIBRARIES} ${ROS_LIBRARIES} ${LIBJPEG_LIBRARY} ${LIBPNG_LIBRARY} ${ZLIB_LIBRARY} )
  ELSE(LIBJPEG_FOUND AND LIBPNG_FOUND AND ZLIB_FOUND)
    IF(NOT(LIBJPEG_FOUND))
      MESSAGE("LIBJPEG NOT FOUND")
    ENDIF(NOT(LIBJPEG_FOUND))
    IF(NOT(LIBPNG_FOUND))
      MESSAGE("LIBJPEG NOT FOUND")
    ENDIF(NOT(LIBPNG_FOUND))
    IF(NOT(ZLIB_FOUND))
      MESSAGE("LIBJPEG NOT FOUND")
    ENDIF(NOT(ZLIB_FOUND))
    MESSAGE("Realsense emitter will be functional except for any processes that require decompressing images such as point cloud reconstruction")
    ADD_EXECUTABLE(RealSense_ROS_Emitter Sensors/realsense_ros_emitter_no_compress.cpp)
    TARGET_LINK_LIBRARIES(RealSense_ROS_Emitter ${KRISLIBRARY_LIBRARIES} ${ROS_LIBRARIES})
  ENDIF(LIBJPEG_FOUND AND LIBPNG_FOUND AND ZLIB_FOUND)

ENDIF(ROS_FOUND)



#controller dispatcher
ADD_EXECUTABLE(ControllerDispatcher Controller/controller_dispatcher.cpp Controller/TaskController.cpp Controller/PyTaskController.cpp Controller/MultiIKController.cpp Common/system_config.cpp)
ADD_DEFINITIONS(-DHAVE_PYTHON=${HAVE_PYTHON})
TARGET_LINK_LIBRARIES(ControllerDispatcher  ${SSPP_LIBRARIES} ${KLAMPT_LIBRARIES} motion_client ${PYTHON_LIBRARIES})

ADD_EXECUTABLE(TaskGUIDemo UI/task_gui_demo.cpp UI/TaskManager.cpp UI/TaskGenerator.cpp UI/PyTaskGenerator.cpp Common/system_config.cpp)
ADD_DEFINITIONS(-DHAVE_PYTHON=${HAVE_PYTHON})
TARGET_LINK_LIBRARIES(TaskGUIDemo  ${SSPP_LIBRARIES} ${KLAMPT_LIBRARIES} ${PYTHON_LIBRARIES})

#show all the included directories for debug
get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "dir='${dir}'")
endforeach()
