cmake_minimum_required(VERSION 3.0.2)
project(diff_teleop_keyboard)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES diff_teleop_keyboard
  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}

)

catkin_install_python(PROGRAMS
 src/diff_teleop_keyboard.py 
 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
