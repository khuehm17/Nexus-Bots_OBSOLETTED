execute_process(COMMAND "/home/blackwind/workspace/Nexus-Bots/nexus-ros/jetnano/build/robot_teleop/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/blackwind/workspace/Nexus-Bots/nexus-ros/jetnano/build/robot_teleop/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
