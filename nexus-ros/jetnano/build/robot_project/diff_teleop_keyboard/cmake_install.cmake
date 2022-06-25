# Install script for directory: /home/blackwind/workspace/Nexus-Bots/nexus-ros/jetnano/src/robot_project/diff_teleop_keyboard

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/blackwind/workspace/Nexus-Bots/nexus-ros/jetnano/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/blackwind/workspace/Nexus-Bots/nexus-ros/jetnano/build/robot_project/diff_teleop_keyboard/catkin_generated/installspace/diff_teleop_keyboard.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/diff_teleop_keyboard/cmake" TYPE FILE FILES
    "/home/blackwind/workspace/Nexus-Bots/nexus-ros/jetnano/build/robot_project/diff_teleop_keyboard/catkin_generated/installspace/diff_teleop_keyboardConfig.cmake"
    "/home/blackwind/workspace/Nexus-Bots/nexus-ros/jetnano/build/robot_project/diff_teleop_keyboard/catkin_generated/installspace/diff_teleop_keyboardConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/diff_teleop_keyboard" TYPE FILE FILES "/home/blackwind/workspace/Nexus-Bots/nexus-ros/jetnano/src/robot_project/diff_teleop_keyboard/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/diff_teleop_keyboard" TYPE PROGRAM FILES "/home/blackwind/workspace/Nexus-Bots/nexus-ros/jetnano/build/robot_project/diff_teleop_keyboard/catkin_generated/installspace/diff_teleop_keyboard.py")
endif()

