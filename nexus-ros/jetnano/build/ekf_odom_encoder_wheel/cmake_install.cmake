# Install script for directory: /home/blackwind/workspace/Nexus-Bots/nexus-ros/jetnano/src/ekf_odom_encoder_wheel

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/blackwind/workspace/Nexus-Bots/nexus-ros/jetnano/build/ekf_odom_encoder_wheel/catkin_generated/installspace/ekf_odom_encoder_wheel.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ekf_odom_encoder_wheel/cmake" TYPE FILE FILES
    "/home/blackwind/workspace/Nexus-Bots/nexus-ros/jetnano/build/ekf_odom_encoder_wheel/catkin_generated/installspace/ekf_odom_encoder_wheelConfig.cmake"
    "/home/blackwind/workspace/Nexus-Bots/nexus-ros/jetnano/build/ekf_odom_encoder_wheel/catkin_generated/installspace/ekf_odom_encoder_wheelConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ekf_odom_encoder_wheel" TYPE FILE FILES "/home/blackwind/workspace/Nexus-Bots/nexus-ros/jetnano/src/ekf_odom_encoder_wheel/package.xml")
endif()

