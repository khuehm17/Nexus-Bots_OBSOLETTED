cmake_minimum_required(VERSION 3.0.2)
project(control_hardware)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES control_hardware
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
# add library python
# add_library(parsers SHARED ${SOURCE_FILES})

add_executable(control_hardware src/control_hardware.cpp)

add_dependencies(control_hardware control_hardware) #${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(control_hardware ${catkin_LIBRARIES})


