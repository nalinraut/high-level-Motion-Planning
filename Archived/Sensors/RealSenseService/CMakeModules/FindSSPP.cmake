# - Try to find SSPP
#
# May wish to define SSPP_ROOT
#
# Once done this will define
#  
#  SSPP_FOUND        - system has SSPP
#  SSPP_INCLUDE_DIR  - the SSPP include directory
#  SSPP_LIBRARY      - Link these to use SSPP
#   


SET(SSPP_ROOT "C:\\iml-internal\\SSPP" CACHE PATH "SSPP path")

IF (SSPP_INCLUDE_DIR)
  # Already in cache, be silent
  SET(SSPP_FIND_QUIETLY TRUE)
ENDIF (SSPP_INCLUDE_DIR)

 # Find the headers
 FIND_PATH( SSPP_INCLUDE_DIR sspp/Service.h
            PATHS /usr/include "${SSPP_ROOT}/include" )

if( WIN32 )

  FIND_LIBRARY( SSPP_LIBRARY_RELEASE
               NAMES sspp.lib
               PATHS "${SSPP_ROOT}/lib" "${SSPP_ROOT}/msvc/lib/Release")

  FIND_LIBRARY( SSPP_LIBRARY_DEBUG
               NAMES sspp.lib
               PATHS "${SSPP_ROOT}/lib" "${SSPP_ROOT}/msvc/lib/Debug")

  SET(SSPP_LIBRARY debug ${SSPP_LIBRARY_DEBUG} optimized ${SSPP_LIBRARY_RELEASE})

else (WIN32)

 FIND_LIBRARY( SSPP_LIBRARY
               NAMES sspp.lib
               PATHS /usr/lib /usr/local/lib "${SSPP_ROOT}/lib"  )
endif( WIN32)


IF (SSPP_INCLUDE_DIR AND SSPP_LIBRARY)
  SET(SSPP_FOUND TRUE)
ELSE (SSPP_INCLUDE_DIR AND SSPP_LIBRARY)
  SET( SSPP_FOUND FALSE )
ENDIF (SSPP_INCLUDE_DIR AND SSPP_LIBRARY)

