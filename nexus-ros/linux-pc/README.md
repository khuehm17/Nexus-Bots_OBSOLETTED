# ROS on Linux-PC
## Installation guideline 
### System prerequisite:
- Linux-PC: 20.04 LTS.
- ROS version: Noetic.

### Configuration
On Jetson:
1. Add Master_IP_URI and ROS_IP to ~/.bashrc environment variables. To check Jetson IP, run command ```$ ifconfig``` in advanced.
    ```
        export ROS_MASTER_URI=http://<Jetson_IP>:<Your_port>
        export ROS_IP=<Jetson_IP>
    ```
    Default port is 11311.
    e.g.
    ```
        export ROS_MASTER_URI=http://192.168.1.113:11311
        export ROS_IP=192.168.1.113
    ```
    Then, source bash configuration file using command ```$ source ~/.bashrc```

On Linux-PC:

2. Add Master_IP_URI and ROS_IP to ~/.bashrc environment variables.
    ```
        export ROS_MASTER_URI=http://<Jetson_IP>:<Your_port>
        export ROS_IP=<Linux_PC_IP>
    ```
    Default port is 11311.
    e.g.
    ```
        export ROS_MASTER_URI=http://192.168.1.113:11311
        export ROS_IP=192.168.1.139
    ```
    Then, source bash configuration file using command ```$ source ~/.bashrc```

### Test ROS_IP
1. On Jetson: running ```$ roscore```
2. On Linux-PC: using command ```$ rostopic list``` and display ```/rosout /rosout_agg```. 
## Running Slam mapping.
1. On Jetson: Everytime enter workspace, must using command ```$ cd Nexus-Bots/nexus-ros/jetnano```, ```$ catkin_make && source devel/setup.bash```. And then ```roslaunch hector_slam_launch tutorial.launch```.
2. On Linux-PC: Using command ```$ cd Nexus-Bots/nexus-ros/linux-pc```, ```$ catkin_make && source devel/setup.bash```. The next, ```$ roslaunch slam_rviz slam_rviz.launch```
