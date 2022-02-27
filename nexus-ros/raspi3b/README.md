# Nexus ROS

## Environment information
- Ubiquity Robotics Raspberry Pi image
- OS version: Ubuntu 16.04 
- ROS version: Kinetic
- Installed packages:
    - ...

## Additional Installation
- Adding Ubiquity Repositories [https://packages.ubiquityrobotics.com]
  - Issue: Fix apt-get “KEYEXPIRED: The following signatures were invalid [https://futurestud.io/tutorials/fix-ubuntu-debian-apt-get-keyexpired-the-following-signatures-were-invalid]
  - Issue: Failed to fetch https://packages.ubiquityrobotics.com/ubuntu/ubiquity/dists/xenial/main/binary-armhf/Packages  server certificate verification failed. CAfile: /etc/ssl/certs/ca-certificates.crt CRLfile: none when run sudo apt-get update [run sudo apt-get upgrade -> sudo apt-get update again]
- Install Rosserial Arduino: sudo apt-get install ros-kinetic-rosserial-arduino
- Install Rosserial: sudo apt-get install ros-kinetic-rosserial

## Useful information
- https://bksimotec.com/khac-phuc-loi-cap-nhat-tren-ubuntu-os/
