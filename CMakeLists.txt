cmake_minimum_required(VERSION 2.8.3)
project(polhemus_connect)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  interactive_markers
  roscpp
  rospy
  std_msgs
  tf
  visualization_msgs
  message_generation
)

## System dependencies are found with CMake's conventions
 find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################


## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   polhemus_pose.msg
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   geometry_msgs   std_msgs   visualization_msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES polhemus_connect
  CATKIN_DEPENDS geometry_msgs interactive_markers roscpp rospy std_msgs tf visualization_msgs #message_generation
  DEPENDS system_lib -lusb #-L/usr/local/lib#/libusb-1.0.so
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  
)

## Declare a C++ library
add_library(polhemus_connect
  src/PingPong.cpp
  src/PiTracker.cpp
  src/PiServer.cpp

  include/PingPong.h
  include/PiTerm.h
  include/PiTracker.h
  /usr/local/lib/libusb-1.0.so
 )

target_link_libraries(polhemus_connect ${catkin_LIBRARIES} -lm -ldl)


  add_executable(PiServer src/PiServer.cpp)
  target_link_libraries(PiServer ${catkin_LIBRARIES} polhemus_connect -lusb-1.0)
  add_dependencies(PiServer polhemus_connect_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############


#############
## Testing ##
#############


