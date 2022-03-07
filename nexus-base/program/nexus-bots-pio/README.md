# Nexus PlatformIO project usage guideline

## Installation
- Install VSCode Download page [https://code.visualstudio.com/download]
- When VSCode was installed:
  - install C/C++ plugin
  - install PlatformIO IDE plugin

## Usage guideline
- Open project folder in VSCode
- Build program: In the left panel, choose **PlatformIO** -> Choose **Build**
- Upload code: In the left panel, choose **PlatformIO** -> Choose **Upload**

### *Note: This project was configured for development on Nexus robot, no need for any extra steps for usage*

## Test guideline
### ROSSER_SUB_TEST
- On ROS machine, launch the roscore in a new terminal window:
  ```
  roscore
  ```
- Run the rosserial client application that forwards your Arduino messages to the rest of ROS. Make sure to use the correct serial port:
  ```
  rosrun rosserial_python serial_node.py /dev/ttyUSB0
  ```
- If you want the ability to programmatically reset your Arduino, run using:
  ```
  rosrun rosserial_arduino serial_node.py _port:=/dev/ttyUSB0
  ```
- You can toggle the motor direction using rostopic:
  ```
  rostopic pub led_blink std_msgs/Empty --once
  ```
## Issues:
1. Compilation error: No \<cstring\> 
  ```
  fatal error: cstring: No such file or directory
   #include <cstring>
  compilation terminated.
  ```
To solve the issue:
- Change #include \<cstring\> to #include \<string.h\>
- Change std::memcpy() to memcpy()