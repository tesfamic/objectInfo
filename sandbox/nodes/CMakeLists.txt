cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()


find_path(LIBUSB_INCLUDEDIR
          NAMES libusb.h
          HINTS /usr/include/libusb-1.0 )
find_library(LIBUSB_LIBRARIES
             NAMES usb-1.0
             HINTS /usr/lib/ /usr/x86_64-linux-gnu/
             PATH_SUFFIXES lib)

message(STATUS ${LIBUSB_INCLUDEDIR})
message(STATUS ${LIBUSB_LIBRARIES})


rosbuild_add_executable(objectInfo src/objectInfo.cpp)
rosbuild_add_executable(kinectMotor src/kinectMotor.cpp)
#rosbuild_add_executable(objectLocation src/objectLocation.cpp)

target_link_libraries(kinectMotor ${LIBUSB_LIBRARIES})



























