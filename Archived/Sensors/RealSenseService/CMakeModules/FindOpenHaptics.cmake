# - Try to find OpenHaptics
#
# May wish to define OPENHAPTICS_ROOT
#
# Once done this will define
#  
#  OPENHAPTICS_FOUND        - system has OpenHaptics
#  OPENHAPTICS_INCLUDE_DIR  - the OpenHaptics include directory
#  OPENHAPTICS_LIBRARY      - Link these to use OpenHaptics
#   


#Default: works for WIN32, academic edition 3.2
SET(OPENHAPTICS_ROOT "C:\\OpenHaptics\\Academic\\3.2" CACHE PATH "OpenHaptics path")

IF (OPENHAPTICS_INCLUDE_DIR)
  # Already in cache, be silent
  SET(OPENHAPTICS_FIND_QUIETLY TRUE)
ENDIF (OPENHAPTICS_INCLUDE_DIR)

 # Find the headers
 FIND_PATH( OPENHAPTICS_INCLUDE_DIR HD/hd.h
            PATHS /usr/include "${OPENHAPTICS_ROOT}/include" )

if( WIN32 )
  IF(CMAKE_SIZEOF_VOID_P EQUAL 8)
    SET(OPENHAPTICS_BUILD_DIR x64)
  ELSE ()
    SET(OPENHAPTICS_BUILD_DIR Win32)
  ENDIF ()

  FIND_LIBRARY( OPENHAPTICS_LIBRARY_RELEASE
               NAMES hd.lib hd.dll hl.lib hl.dll
               PATHS "${OPENHAPTICS_ROOT}/lib" "${OPENHAPTICS_ROOT}/lib/${OPENHAPTICS_BUILD_DIR}" "${OPENHAPTICS_ROOT}/lib/${OPENHAPTICS_BUILD_DIR}/ReleaseAcademicEdition")

  FIND_LIBRARY( OPENHAPTICS_LIBRARY_DEBUG
               NAMES hd.lib hd.dll hl.lib hl.dll
               PATHS "${OPENHAPTICS_ROOT}/lib/${OPENHAPTICS_BUILD_DIR}/DebugAcademicEdition")

  SET(OPENHAPTICS_LIBRARY debug ${OPENHAPTICS_LIBRARY_DEBUG} optimized ${OPENHAPTICS_LIBRARY_RELEASE})

else (WIN32)

 FIND_LIBRARY( OPENHAPTICS_LIBRARY
               NAMES hd.lib hl.lib
               PATHS /usr/lib /usr/local/lib "${OPENHAPTICS_ROOT}/lib"  )
endif( WIN32)


IF (OPENHAPTICS_INCLUDE_DIR AND OPENHAPTICS_LIBRARY)
  SET(OPENHAPTICS_FOUND TRUE)
ELSE (OPENHAPTICS_INCLUDE_DIR AND OPENHAPTICS_LIBRARY)
  SET( OPENHAPTICS_FOUND FALSE )
ENDIF (OPENHAPTICS_INCLUDE_DIR AND OPENHAPTICS_LIBRARY)

