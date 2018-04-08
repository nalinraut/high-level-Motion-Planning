# - Try to find RealSense
#
# May wish to define REALSENSE_ROOT
#
# Once done this will define
#  
#  REALSENSE_FOUND        - system has OpenHaptics
#  REALSENSE_INCLUDE_DIR  - the OpenHaptics include directory
#  REALSENSE_LIBRARY      - Link these to use OpenHaptics
#   


#Default: works for WIN32, academic edition 3.2
SET(REALSENSE_ROOT "C:\\Program Files (x86)\\Intel\\RSSDK" CACHE PATH "Intel RealSense SDK path")

IF (REALSENSE_INCLUDE_DIR)
  # Already in cache, be silent
  SET(REALSENSE_FIND_QUIETLY TRUE)
ENDIF (REALSENSE_INCLUDE_DIR)

 # Find the headers
 FIND_PATH( REALSENSE_INCLUDE_DIR pxcbase.h
            PATHS /usr/include "${REALSENSE_ROOT}/include" )

#path to utility classes
SET(REALSENSE_UTILS_ROOT "${REALSENSE_ROOT}/sample/common")

 # Find the headers for the utilities
 FIND_PATH( REALSENSE_UTILS_INCLUDE_DIR util_render.h
            PATHS /usr/include "${REALSENSE_UTILS_ROOT}/include" )

LIST(APPEND REALSENSE_INCLUDE_DIR ${REALSENSE_UTILS_INCLUDE_DIR})


if( WIN32 )
  IF(CMAKE_SIZEOF_VOID_P EQUAL 8)
    SET(REALSENSE_BUILD_DIR x64)
  ELSE ()
    SET(REALSENSE_BUILD_DIR Win32)
  ENDIF ()

  FIND_LIBRARY( REALSENSE_LIBRARY_RELEASE
               libpxc.lib
               PATHS "${REALSENSE_ROOT}/lib" "${REALSENSE_ROOT}/lib/${REALSENSE_BUILD_DIR}")

  FIND_LIBRARY( REALSENSE_LIBRARY_DEBUG
               NAMES libpxc_d.lib
               PATHS "${REALSENSE_ROOT}/lib/${REALSENSE_BUILD_DIR}")

  FIND_LIBRARY( REALSENSE_UTILS_LIBRARY_RELEASE
               NAMES libpxcutils.lib
               PATHS "${REALSENSE_UTILS_ROOT}/lib/${REALSENSE_BUILD_DIR}/v110")

  FIND_LIBRARY( REALSENSE_UTILS_LIBRARY_DEBUG
               NAMES libpxcutils_d.lib
               PATHS "${REALSENSE_UTILS_ROOT}/lib/${REALSENSE_BUILD_DIR}/v110")

  SET(REALSENSE_LIBRARY debug ${REALSENSE_LIBRARY_DEBUG} debug ${REALSENSE_UTILS_LIBRARY_DEBUG} optimized ${REALSENSE_LIBRARY_RELEASE} optimized ${REALSENSE_UTILS_LIBRARY_RELEASE})

else (WIN32)

 FIND_LIBRARY( REALSENSE_LIBRARY
               NAMES pxc
               PATHS /usr/lib /usr/local/lib "${REALSENSE_ROOT}/lib"  )
endif( WIN32)


IF (REALSENSE_INCLUDE_DIR AND REALSENSE_LIBRARY)
  SET(REALSENSE_FOUND TRUE)
ELSE (REALSENSE_INCLUDE_DIR AND REALSENSE_LIBRARY)
  SET( REALSENSE_FOUND FALSE )
ENDIF (REALSENSE_INCLUDE_DIR AND REALSENSE_LIBRARY)

