# - Try to find turboJPEG
#
# May wish to define TURBOJPEG_ROOT
#
# Once done this will define
#  
#  TURBOJPEG_FOUND        - system has TURBOJPEG
#  TURBOJPEG_INCLUDE_DIR  - the TURBOJPEG include directory
#  TURBOJPEG_LIBRARY      - Link these to use TURBOJPEG
#   


#Default: works for WIN32, academic edition 3.2
SET(TURBOJPEG_ROOT "C:\\libjpeg-turbo-gcc" CACHE PATH "TURBOJPEG path")
SET(CMAKE_FIND_LIBRARY_SUFFIXES ".lib" ".a")
#SET(CMAKE_IMPORT_LIBRARY_SUFFIX ".lib" ".a")

IF (TURBOJPEG_INCLUDE_DIR)
  # Already in cache, be silent
  SET(TURBOJPEG_FIND_QUIETLY TRUE)
ENDIF (TURBOJPEG_INCLUDE_DIR)

 # Find the headers
 FIND_PATH( TURBOJPEG_INCLUDE_DIR jconfig.h
            PATHS /usr/include "${TURBOJPEG_ROOT}/include" )

if( WIN32 )
  IF(CMAKE_SIZEOF_VOID_P EQUAL 8)
    SET(TURBOJPEG_BUILD_DIR x64)
  ELSE ()
    SET(TURBOJPEG_BUILD_DIR Win32)
  ENDIF ()

  FIND_LIBRARY( TURBOJPEG_LIBRARY_RELEASE
               NAMES libturbojpeg
               PATHS /usr/lib /usr/local/lib "${TURBOJPEG_ROOT}/lib")
			   
  FIND_LIBRARY( TURBOJPEG_LIBRARY_DEBUG
               NAMES libturbojpeg
               PATHS /usr/lib /usr/local/lib "${TURBOJPEG_ROOT}/lib")
			   
  SET(TURBOJPEG_LIBRARY debug ${TURBOJPEG_LIBRARY_DEBUG} optimized ${TURBOJPEG_LIBRARY_RELEASE})

else (WIN32)

 FIND_LIBRARY( TURBOJPEG_LIBRARY
               NAMES libturbojpeg.a
               PATHS /usr/lib /usr/local/lib "${TURBOJPEG_ROOT}/lib")
endif( WIN32)



IF (TURBOJPEG_INCLUDE_DIR AND TURBOJPEG_LIBRARY)
  SET(TURBOJPEG_FOUND TRUE)
ELSE (TURBOJPEG_INCLUDE_DIR AND TURBOJPEG_LIBRARY)
  SET( TURBOJPEG_FOUND FALSE )
ENDIF (TURBOJPEG_INCLUDE_DIR AND TURBOJPEG_LIBRARY)
