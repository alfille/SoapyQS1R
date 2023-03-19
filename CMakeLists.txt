######################################################################## 
 # Build Soapy SDR support module for QS1R
 ######################################################################## 

cmake_minimum_required(VERSION 2.8.7)
project(SoapyQS1R CXX)

find_package(SoapySDR "0.4.0" NO_MODULE)
 if (NOT SoapySDR_FOUND) 
    message(FATAL_ERROR "Soapy SDR development files not found...") 
 endif () 

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
find_package(LIBUSB REQUIRED libusb-1.0)
#find_package(LIBUSB)

if (NOT LIBUSB_FOUND) 
     message(FATAL_ERROR "LIBUSB development files not found...") 
endif () 
message(STATUS "LIBUSB_INCLUDE_DIRS - ${LIBUSB_INCLUDE_DIRS}")
message(STATUS "LIBUSB_LIBRARIES - ${LIBUSB_LIBRARIES}")

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${LIBUSB_INCLUDE_DIRS})

#enable c++11 features
if(CMAKE_COMPILER_IS_GNUCXX)

    #C++11 is a required language feature for this project
    include(CheckCXXCompilerFlag)
    CHECK_CXX_COMPILER_FLAG("-std=c++11" HAS_STD_CXX11)
    if(HAS_STD_CXX11)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    else(HAS_STD_CXX11)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    endif()

    #disable warnings for unused parameters
    add_definitions(-Wno-unused-parameter)

endif(CMAKE_COMPILER_IS_GNUCXX)

if (APPLE)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wc++11-extensions")
endif(APPLE)

SOAPY_SDR_MODULE_UTIL(
    TARGET QS1RSupport
    SOURCES
    QS1R_Registration.cpp
    QS1R_Settings.cpp
    QS1R_Streaming.cpp
    QS1R_Session.cpp
    QS1R_Utilities.cpp
    LIBRARIES ${LIBUSB_LIBRARIES}
)

#Copy firmware to /usr/share
# https://stackoverflow.com/questions/15694347/installing-additional-files-with-cmake
INSTALL(DIRECTORY firmware DESTINATION /usr/share/QS1R)

#UDEV rules
INSTALL(FILES udev/10-qs1r.rules DESTINATION /etc/udev/rules.d/10-qs1r.rules)


########################################################################
# uninstall target
########################################################################
add_custom_target(uninstall
    "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

