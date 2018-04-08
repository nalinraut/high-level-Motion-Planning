# FindLIBJPEG.cmake
# - Try to find LIBJPEG
#
# May wish to define LIBJPEG_ROOT
#
# Once done this will define
#  
#  LIBJPEG_FOUND        - system has LIBJPEG
#  LIBJPEG_INCLUDE_DIR  - the LIBJPEG include directory
#  LIBJPEG_LIBRARY      - Link these to use LIBJPEG
#   


#Default: works for WIN32, academic edition 3.2
SET(LIBJPEG_ROOT "/usr/lib/x86_64-linux-gnu" CACHE PATH "LIBJPEG path")
SET(CMAKE_FIND_LIBRARY_SUFFIXES ".lib" ".a")
#SET(CMAKE_IMPORT_LIBRARY_SUFFIX ".lib" ".a")

SET(DUMMY "this")

IF (LIBJPEG_INCLUDE_DIR)
  # Already in cache, be silent
  SET(LIBJPEG_FIND_QUIETLY TRUE)
ENDIF (LIBJPEG_INCLUDE_DIR)

 # Find the headers
 FIND_PATH( LIBJPEG_INCLUDE_DIR jconfig.h
            PATHS /usr/include "${LIBJPEG_ROOT}/include" )

if( WIN32 )
  IF(CMAKE_SIZEOF_VOID_P EQUAL 8)
    SET(LIBJPEG_BUILD_DIR x64)
  ELSE ()
    SET(LIBJPEG_BUILD_DIR Win32)
  ENDIF ()

  FIND_LIBRARY( LIBJPEG_LIBRARY_RELEASE
               NAMES libjpeg
               PATHS /usr/lib /usr/local/lib "${LIBJPEG_ROOT}")
			   
  FIND_LIBRARY( LIBJPEG_LIBRARY_DEBUG
               NAMES libjpeg
               PATHS /usr/lib /usr/local/lib "${LIBJPEG_ROOT}")
			   
  SET(LIBJPEG_LIBRARY debug ${LIBJPEG_LIBRARY_DEBUG} optimized ${LIBJPEG_LIBRARY_RELEASE})

else (WIN32)

 FIND_LIBRARY( LIBJPEG_LIBRARY
               NAMES libjpeg.a
               PATHS "${LIBJPEG_ROOT}" /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu )

endif( WIN32)

IF (LIBJPEG_INCLUDE_DIR AND LIBJPEG_LIBRARY)
  SET(LIBJPEG_FOUND TRUE)
ELSE (LIBJPEG_INCLUDE_DIR AND LIBJPEG_LIBRARY)
  SET( LIBJPEG_FOUND FALSE )
ENDIF (LIBJPEG_INCLUDE_DIR AND LIBJPEG_LIBRARY)

