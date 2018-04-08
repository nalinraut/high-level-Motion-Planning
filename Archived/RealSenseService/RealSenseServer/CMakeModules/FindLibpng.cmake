# - Try to find Libpng
#
# May wish to define LIBPNG_ROOT
#
# Once done this will define
#  
#  LIBPNG_FOUND        - system has LIBPNG
#  LIBPNG_INCLUDE_DIR  - the LIBPNG include directory
#  LIBPNG_LIBRARY      - Link these to use LIBPNG
#   

#Default: works for WIN32, academic edition 3.2
SET(LIBPNG_ROOT "C:\\png_libs" CACHE PATH "LIBPNG path")
SET(CMAKE_FIND_LIBRARY_SUFFIXES ".lib" ".a")
#SET(CMAKE_IMPORT_LIBRARY_SUFFIX ".lib" ".a")

IF (LIBPNG_INCLUDE_DIR)
  # Already in cache, be silent
  SET(LIBPNG_FIND_QUIETLY TRUE)
ENDIF (LIBPNG_INCLUDE_DIR)

 # Find the headers
 FIND_PATH( LIBPNG_INCLUDE_DIR png.h
            PATHS /usr/include "${LIBPNG_ROOT}/include" )

if( WIN32 )
  IF(CMAKE_SIZEOF_VOID_P EQUAL 8)
    SET(LIBPNG_BUILD_DIR x64)
  ELSE ()
    SET(LIBPNG_BUILD_DIR Win32)
  ENDIF ()

  FIND_LIBRARY( LIBPNG_LIBRARY_RELEASE
               NAMES zlibstat libpng 
               PATHS /usr/lib /usr/local/lib "${LIBPNG_ROOT}/lib")
			   
  FIND_LIBRARY( LIBPNG_LIBRARY_DEBUG
               NAMES zlibstatd libpngd 
               PATHS /usr/lib /usr/local/lib "${LIBPNG_ROOT}/lib")
			   
  SET(LIBPNG_LIBRARY debug ${LIBPNG_LIBRARY_DEBUG} optimized ${LIBPNG_LIBRARY_RELEASE})

else (WIN32)

 FIND_LIBRARY( LIBPNG_LIBRARY
               NAMES libLIBPNG.a
               PATHS /usr/lib /usr/local/lib "${LIBPNG_ROOT}/lib")
endif( WIN32)



IF (LIBPNG_INCLUDE_DIR AND LIBPNG_LIBRARY)
  SET(LIBPNG_FOUND TRUE)
ELSE (LIBPNG_INCLUDE_DIR AND LIBPNG_LIBRARY)
  SET( LIBPNG_FOUND FALSE )
ENDIF (LIBPNG_INCLUDE_DIR AND LIBPNG_LIBRARY)
