# Nexus robotics base

- documents/: technical configuration, manual,...
- lib/: Nexus Arduino libraries
- program/: testing code
- resources/: images for taking notes

## Nexus hardware
### Controller information
- Board: Arduino Nano
- Processor: ATmega328P (Old Bootloader)

### IMU information
- Hardware: BMX160 9-axis Sensor
- Hardware information: https://www.dfrobot.com/product-1928.html
- Useful resources
  - https://wiki.dfrobot.com/Gravity%3A%20BMX160%2BBMP388%2010%20DOF%20Sensor%20SKU%3A%20SEN0252
  - https://github.com/DFRobot/DFRobot_BMX160
  - https://github.com/DFRobot/DFRobot_BMP388

### Omni wheel odometry
- Useful resources
  - https://github.com/GuiRitter/OpenBase

### Hardware configuration
|      | Motor Wheel 1 | Motor Wheel 2 | Motor Wheel 3 |
|------|---------------|---------------|---------------|
| PWM  |       5       |       6       |       10      |
| DIR  |       4       |       7       |       11      |
| ENCA |       2       |       8       |       12      |
| ENCB |       3       |       9       |       13      |

### Localization Algorithm
- Useful resources
  - https://www.michaelskupien.com/robot-localization-ros
  - https://github.com/mkhuthir/RoboND-Robot-Localization-Project
  - https://automaticaddison.com/sensor-fusion-using-the-ros-robot-pose-ekf-package/
  - https://github.com/cra-ros-pkg/robot_localization
  - https://github.com/chiprobotics/chip_imu_driver
  - https://kapernikov.com/the-ros-robot_localization-package/