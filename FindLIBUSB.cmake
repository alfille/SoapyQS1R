# - Find libusb for portable USB support
# This module will find libusb as published by
#  http://libusb.sf.net and
#  http://libusb-win32.sf.net
# 
# It will use PkgConfig if present and supported, else search
# it on its own. If the LIBUSB_ROOT_DIR environment variable
# is defined, it will be used as base path.
# The following standard variables get defined:
#  LIBUSB_FOUND:        true if LIBUSB was found
#  LIBUSB_INCLUDE_DIRS: the directory that contains the include file
#  LIBUSB_LIBRARIES:    the library

# Shameless stolen from:
# https://raw.githubusercontent.com/LuaDist/lualibusb/master/cmake/FindLIBUSB.cmake

include ( CheckLibraryExists )
include ( CheckIncludeFile )

find_package ( PkgConfig )
if ( PKG_CONFIG_FOUND )
  pkg_check_modules ( PKGCONFIG_LIBUSB libusb-1.0 )
endif ( PKG_CONFIG_FOUND )

if ( PKGCONFIG_LIBUSB_FOUND )
  set ( LIBUSB_FOUND ${PKGCONFIG_LIBUSB_FOUND} )
  set ( LIBUSB_INCLUDE_DIRS ${PKGCONFIG_LIBUSB_INCLUDE_DIRS} )
  foreach ( i ${PKGCONFIG_LIBUSB_LIBRARIES} )
    find_library ( ${i}_LIBRARY
      NAMES ${i}
      PATHS ${PKGCONFIG_LIBUSB_LIBRARY_DIRS}
    )
    if ( ${i}_LIBRARY )
      list ( APPEND LIBUSB_LIBRARIES ${${i}_LIBRARY} )
    endif ( ${i}_LIBRARY )
    mark_as_advanced ( ${i}_LIBRARY )
  endforeach ( i )

else ( PKGCONFIG_LIBUSB_FOUND )
  find_path ( LIBUSB_INCLUDE_DIRS
    NAMES
      usb.h
    PATHS
      $ENV{ProgramFiles}/LIBUSB-Win32
      $ENV{LIBUSB_ROOT_DIR}
    PATH_SUFFIXES
      include
  )
  mark_as_advanced ( LIBUSB_INCLUDE_DIRS )
#  message ( STATUS "LIBUSB include dir: ${LIBUSB_INCLUDE_DIRS}" )

  if ( ${CMAKE_SYSTEM_NAME} STREQUAL "Windows" )
    # LIBUSB-Win32 binary distribution contains several libs.
    # Use the lib that got compiled with the same compiler.
    if ( MSVC )
      if ( WIN32 )
        set ( LIBUSB_LIBRARY_PATH_SUFFIX lib/msvc )
      else ( WIN32 )
        set ( LIBUSB_LIBRARY_PATH_SUFFIX lib/msvc_x64 )
      endif ( WIN32 )          
    elseif ( BORLAND )
      set ( LIBUSB_LIBRARY_PATH_SUFFIX lib/bcc )
    elseif ( CMAKE_COMPILER_IS_GNUCC )
      set ( LIBUSB_LIBRARY_PATH_SUFFIX lib/gcc )
    endif ( MSVC )
  endif ( ${CMAKE_SYSTEM_NAME} STREQUAL "Windows" )

  find_library ( usb_LIBRARY
    NAMES
      libusb usb
    PATHS
      $ENV{ProgramFiles}/LIBUSB-Win32
      $ENV{LIBUSB_ROOT_DIR}
    PATH_SUFFIXES
      ${LIBUSB_LIBRARY_PATH_SUFFIX}
  )
  mark_as_advanced ( usb_LIBRARY )
  if ( usb_LIBRARY )
    set ( LIBUSB_LIBRARIES ${usb_LIBRARY} )
  endif ( usb_LIBRARY )

  if ( LIBUSB_INCLUDE_DIRS AND LIBUSB_LIBRARIES )
    set ( LIBUSB_FOUND true )
  endif ( LIBUSB_INCLUDE_DIRS AND LIBUSB_LIBRARIES )
endif ( PKGCONFIG_LIBUSB_FOUND )

if ( LIBUSB_FOUND )
  set ( CMAKE_REQUIRED_INCLUDES "${LIBUSB_INCLUDE_DIRS}" )
  check_include_file ( usb.h LIBUSB_FOUND )
#    message ( STATUS "LIBUSB: usb.h is usable: ${LIBUSB_FOUND}" )
endif ( LIBUSB_FOUND )
if ( LIBUSB_FOUND )
  check_library_exists ( "${LIBUSB_LIBRARIES}" usb_open "" LIBUSB_FOUND )
#    message ( STATUS "LIBUSB: library is usable: ${LIBUSB_FOUND}" )
endif ( LIBUSB_FOUND )

if ( NOT LIBUSB_FOUND )
  if ( NOT LIBUSB_FIND_QUIETLY )
    message ( STATUS "LIBUSB not found, try setting LIBUSB_ROOT_DIR environment variable." )
  endif ( NOT LIBUSB_FIND_QUIETLY )
  if ( LIBUSB_FIND_REQUIRED )
    message ( FATAL_ERROR "" )
  endif ( LIBUSB_FIND_REQUIRED )
endif ( NOT LIBUSB_FOUND )
#message ( STATUS "LIBUSB: ${LIBUSB_FOUND}" )
