# - Try to find Perceptual Computing SDK
#
# May wish to define PCSDK_ROOT
#
# Once done this will define
#  
#  PCSDK_FOUND        - system has OpenHaptics
#  PCSDK_INCLUDE_DIR  - the OpenHaptics include directory
#  PCSDK_LIBRARY      - Link these to use OpenHaptics
#   


#Default: works for WIN32, academic edition 3.2
SET(PCSDK_ROOT "C:\\Program Files (x86)\\Intel\\PCSDK" CACHE PATH "Intel Perceptual Computing SDK path")

IF (PCSDK_INCLUDE_DIR)
  # Already in cache, be silent
  SET(PCSDK_FIND_QUIETLY TRUE)
ENDIF (PCSDK_INCLUDE_DIR)

 # Find the headers
 FIND_PATH( PCSDK_INCLUDE_DIR pxcbase.h
            PATHS /usr/include "${PCSDK_ROOT}/include" )

#path to utility classes
SET(PCSDK_UTILS_ROOT "${PCSDK_ROOT}/sample/common")

 # Find the headers for the utilities
 FIND_PATH( PCSDK_UTILS_INCLUDE_DIR util_capture.h
            PATHS /usr/include "${PCSDK_UTILS_ROOT}/include" )

LIST(APPEND PCSDK_INCLUDE_DIR ${PCSDK_UTILS_INCLUDE_DIR})


if( WIN32 )
  IF(CMAKE_SIZEOF_VOID_P EQUAL 8)
    SET(PCSDK_BUILD_DIR x64)
  ELSE ()
    SET(PCSDK_BUILD_DIR Win32)
  ENDIF ()

  FIND_LIBRARY( PCSDK_LIBRARY_RELEASE
               libpxc.lib
               PATHS "${PCSDK_ROOT}/lib" "${PCSDK_ROOT}/lib/${PCSDK_BUILD_DIR}")

  FIND_LIBRARY( PCSDK_LIBRARY_DEBUG
               NAMES libpxc_d.lib
               PATHS "${PCSDK_ROOT}/lib/${PCSDK_BUILD_DIR}")

  FIND_LIBRARY( PCSDK_UTILS_LIBRARY_RELEASE
               NAMES libpxcutils.lib
               PATHS "${PCSDK_UTILS_ROOT}/lib/${PCSDK_BUILD_DIR}/v110")

  FIND_LIBRARY( PCSDK_UTILS_LIBRARY_DEBUG
               NAMES libpxcutils_d.lib
               PATHS "${PCSDK_UTILS_ROOT}/lib/${PCSDK_BUILD_DIR}/v110")

  SET(PCSDK_LIBRARY debug ${PCSDK_LIBRARY_DEBUG} debug ${PCSDK_UTILS_LIBRARY_DEBUG} optimized ${PCSDK_LIBRARY_RELEASE} optimized ${PCSDK_UTILS_LIBRARY_RELEASE})

else (WIN32)

 FIND_LIBRARY( PCSDK_LIBRARY
               NAMES pxc
               PATHS /usr/lib /usr/local/lib "${PCSDK_ROOT}/lib"  )
endif( WIN32)


IF (PCSDK_INCLUDE_DIR AND PCSDK_LIBRARY)
  SET(PCSDK_FOUND TRUE)
ELSE (PCSDK_INCLUDE_DIR AND PCSDK_LIBRARY)
  SET( PCSDK_FOUND FALSE )
ENDIF (PCSDK_INCLUDE_DIR AND PCSDK_LIBRARY)

