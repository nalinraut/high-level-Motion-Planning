CMAKE_MINIMUM_REQUIRED(VERSION 2.6.4)
PROJECT( SSPP )

SET (CMAKE_MODULE_PATH "CMakeModules")

##### External libraries: KrisLibrary, its dependencies#####
SET(KRISLIBRARY_ROOT "../../Klampt/Library" CACHE PATH "KrisLibrary parent path")

FIND_PACKAGE(KrisLibrary REQUIRED)
ADD_DEFINITIONS(${KRISLIBRARY_DEFINITIONS})
INCLUDE_DIRECTORIES(${KRISLIBRARY_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${KRISLIBRARY_ROOT})


# COMPILER SETTINGS (default: Release) and flags
INCLUDE(CompilerSettings)

# Set output directories for libraries and executables
SET( BASE_DIR ${CMAKE_BINARY_DIR} )
SET( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${BASE_DIR}/lib )
SET( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${BASE_DIR}/lib )
SET( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BASE_DIR}/services )

set(INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/include")
INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

LINK_DIRECTORIES(${CMAKE_LIBRARY_OUTPUT_DIRECTORY})





####### Applications #########

FILE(GLOB SSPP_SOURCE_FILES "src/sspp/*.cpp")
ADD_LIBRARY(sspp ${SSPP_SOURCE_FILES})
TARGET_LINK_LIBRARIES(sspp ${KRISLIBRARY_LIBRARIES})
ADD_EXECUTABLE(EchoService src/services/echo_service.cpp)
ADD_EXECUTABLE(LogService src/services/log_service.cpp)
ADD_EXECUTABLE(SendService src/services/send_service.cpp)
ADD_EXECUTABLE(HeartbeatService src/services/heartbeat_service.cpp)
TARGET_LINK_LIBRARIES(EchoService sspp)
TARGET_LINK_LIBRARIES(LogService sspp)
TARGET_LINK_LIBRARIES(SendService sspp)
TARGET_LINK_LIBRARIES(HeartbeatService sspp)


# uninstall target
configure_file(
    "${CMAKE_MODULE_PATH}/CMakeUninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)


#install targets
install(TARGETS EchoService LogService SendService
  RUNTIME DESTINATION bin
  COMPONENT apps)

# Needs to be last statement:
#INCLUDE(CPackSettings)

# Finished:
MESSAGE ("\n")
MESSAGE (STATUS "Compile everything using: make\n")
MESSAGE (STATUS "Install using: make install")
MESSAGE (STATUS "Uninstall using: make uninstall\n")

get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "dir='${dir}'")
endforeach()
