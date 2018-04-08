# - Try to find ZLIB
#
# May wish to define ZLIB_ROOT
#
# Once done this will define
#  
#  ZLIB_FOUND        - system has ZLIB
#  ZLIB_INCLUDE_DIR  - the ZLIB include directory
#  ZLIB_LIBRARY      - Link these to use ZLIB
#   

#Default: works for WIN32, academic edition 3.2
SET(ZLIB_ROOT "/usr/lib/x86_64-linux-gnu" CACHE PATH "ZLIB path")
SET(CMAKE_FIND_LIBRARY_SUFFIXES ".lib" ".a" ".so")
#SET(CMAKE_IMPORT_LIBRARY_SUFFIX ".lib" ".a")

IF (ZLIB_INCLUDE_DIR)
  # Already in cache, be silent
  SET(ZLIB_FIND_QUIETLY TRUE)
ENDIF (ZLIB_INCLUDE_DIR)

 # Find the headers
 FIND_PATH( ZLIB_INCLUDE_DIR zlib.h
            PATHS /usr/include "${ZLIB_ROOT}/include" )

if( WIN32 )
  IF(CMAKE_SIZEOF_VOID_P EQUAL 8)
    SET(ZLIB_BUILD_DIR x64)
  ELSE ()
    SET(ZLIB_BUILD_DIR Win32)
  ENDIF ()

  FIND_LIBRARY( ZLIB_LIBRARY_RELEASE
               NAMES zlibstat 
               PATHS /usr/lib /usr/local/lib "${ZLIB_ROOT}/lib")
			   
  FIND_LIBRARY( ZLIB_LIBRARY_DEBUG
               NAMES zlibstatd 
               PATHS /usr/lib /usr/local/lib "${ZLIB_ROOT}/lib")
			   
  SET(ZLIB_LIBRARY debug ${ZLIB_LIBRARY_DEBUG} optimized ${ZLIB_LIBRARY_RELEASE})

else (WIN32)

  FIND_LIBRARY( ZLIB_LIBRARY
               NAMES libz.a
               PATHS /usr/lib /usr/local/lib "${ZLIB_ROOT}")
endif( WIN32)



IF (ZLIB_INCLUDE_DIR AND ZLIB_LIBRARY)
  SET(ZLIB_FOUND TRUE)
ELSE (ZLIB_INCLUDE_DIR AND ZLIB_LIBRARY)
  SET( ZLIB_FOUND FALSE )
ENDIF (ZLIB_INCLUDE_DIR AND ZLIB_LIBRARY)
