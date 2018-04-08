CMAKE_MINIMUM_REQUIRED(VERSION 2.6.4)
PROJECT( RealSenseService )

SET (CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules")

# COMPILER SETTINGS (default: Release) and flags
INCLUDE(CompilerSettings)

# Set output directories for libraries and executables
SET( BASE_DIR ${CMAKE_BINARY_DIR} )
SET( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${BASE_DIR}/lib )
SET( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${BASE_DIR}/lib )
SET( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BASE_DIR}/bin )

set(INCLUDE_DIRS "${PROJECT_SOURCE_DIR}")
INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

LINK_DIRECTORIES(${CMAKE_LIBRARY_OUTPUT_DIRECTORY})


##### External libraries: KrisLibrary, its dependencies, RealSense, SSPP#####
SET(KRISLIBRARY_ROOT "../../Klampt/Library" CACHE PATH "KrisLibrary parent path")
FIND_PACKAGE(KrisLibrary REQUIRED)
ADD_DEFINITIONS(${KRISLIBRARY_DEFINITIONS})
INCLUDE_DIRECTORIES(${KRISLIBRARY_INCLUDE_DIRS})

FIND_PACKAGE(RealSense REQUIRED)
ADD_DEFINITIONS(${REALSENSE_DEFINITIONS})
INCLUDE_DIRECTORIES(${REALSENSE_INCLUDE_DIR})

FIND_PACKAGE(SSPP REQUIRED)
INCLUDE_DIRECTORIES(${SSPP_INCLUDE_DIR})


####### Applications #########
IF(NOT USE_QT4_APPS)
  #USE_QT4_APPS not set
  FIND_PACKAGE(Qt4)
  IF(QT4_FOUND)
    SET(USE_QT4_APPS TRUE)
  ELSE(QT4_FOUND)
    SET(USE_QT4_APPS FALSE)
  ENDIF(QT4_FOUND)
ENDIF(NOT USE_QT4_APPS)

ADD_EXECUTABLE(RealSenseService realsense_service.cpp )
TARGET_LINK_LIBRARIES(RealSenseService ${KRISLIBRARY_LIBRARIES} ${SSPP_LIBRARY} ${REALSENSE_LIBRARY})


# uninstall target
configure_file(
    "${CMAKE_MODULE_PATH}/CMakeUninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)


#install targets
install(TARGETS RealSenseService 
  RUNTIME DESTINATION bin
  COMPONENT apps)

# Needs to be last statement:
#INCLUDE(CPackSettings)

# Finished:
MESSAGE ("\n")
MESSAGE (STATUS "Compile everything using: make\n")
MESSAGE (STATUS "Install using: make install")
MESSAGE (STATUS "Uninstall using: make uninstall\n")
