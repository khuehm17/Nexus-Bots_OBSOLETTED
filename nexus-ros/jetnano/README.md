# Nexus ROS

## Environments information
- OS version: Ubuntu 18.04
- ROS version: Melodic
- Board: Jetson Nano J47
- Prerequisite packages
  - rosserial

## Run guideline  
### Keyboard control
  1. Start ROS
  ```
  roscore
  ```
  2. Start ROSSERIAL node
  ```
  rosrun rosserial_arduino serial_node.py /dev/ttyUSB0
  ```
  3. Start keyboard_teleop node
  ```
  rosrun turtlesim turtle_teleop_key
  ```
  4. Initialize control_hardware node
  ```
  - cd workspace/Control_hardware_ROS
  - catkin_make
  - source devel/setup.bash
  ```
  4. Start hardware_control node
  ```
  rosrun control_hardware control_hardware
  ```
