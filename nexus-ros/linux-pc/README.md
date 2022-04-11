# ROS on Linux-PC
## Installation guideline 
### System prerequisite:
- Linux-PC: 20.04 LTS.
- ROS version: Noetic.

### Configuration
We want to access the ROS communication messages from our laptop. This will also let us visualize things in a convenient manner (with rviz). There are a couple of steps to follow here.

On Jetson:
1. Add Master_IP_URI and ROS_IP to ~/.bashrc environment variables. To check Jetson IP, run command ```$ ifconfig``` in advanced to find the ROS_IP (being the IP address of the master computer).
    ```
        export ROS_MASTER_URI=http://<Jetson_IP>:<Your_port>
        export ROS_IP=<Jetson_IP>
    ```
    Default port is 11311.

    For e.g.
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

    For e.g.
    ```
        export ROS_MASTER_URI=http://192.168.1.113:11311
        export ROS_IP=192.168.1.139
    ```
    Then, source bash configuration file using command ```$ source ~/.bashrc```

### To Test ROS_IP
1. On Jetson: running ```$ roscore``` to start ROS on Jetson.
2. On Linux-PC: using command ```$ rostopic list``` to list all topics and run ```/rosout /rosout_agg``` to check if ROS_IP was correctly configured.
## Running Slam mapping Visualization
1. On Jetson: Whenever enter workspace, using bellow commands to rebuild ROS workspace on Jetson.
   ```
   $ cd Nexus-Bots/nexus-ros/jetnano
   $ catkin_make && source devel/setup.bash
   ```

   Then launch Hector Slam node by using bellow command:
   ```
   roslaunch hector_slam_launch tutorial.launch
   ```
2. On Linux-PC: Using bellow commands to rebuild ROS workspace on PC.
    ```
   $ cd Nexus-Bots/nexus-ros/linux-pc
   $ catkin_make && source devel/setup.bash
   ```

   After Hector Slam node was launched on Jetson, we can launch Rviz visualization node by using bellow commnand:
   ```
   $ roslaunch slam_rviz slam_rviz.launch
   ```
