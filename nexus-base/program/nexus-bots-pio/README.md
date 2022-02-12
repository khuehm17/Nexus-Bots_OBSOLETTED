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