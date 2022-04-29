#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt16.h"
#include <geometry_msgs/Twist.h>
#include <sstream>

#define CMD_STOP          (1)
#define CMD_ADVANCE       (2)
#define CMD_BACKOFF       (3)
#define CMD_LEFT          (4)
#define CMD_RIGHT         (5)
#define CMD_ROTATELEFT    (6)
#define CMD_ROTATERIGHT   (7)
#define COMPARE_VEL_ANG   (1)
#define COMPARE_VEL_LIN   (0.5) 
#define COMPARE_VEL_STOP  (0)

std_msgs::UInt16 msgControl;
float lin_vel_x, ang_vel_z;

// Create a command callback
void cmdvelCallback(const geometry_msgs::Twist::ConstPtr& keyboard_control)
{
    lin_vel_x = keyboard_control->linear.x;
    ang_vel_z = keyboard_control->angular.z;
    if (lin_vel_x == COMPARE_VEL_LIN)
    {
        // go traight ahead
        msgControl.data = CMD_ADVANCE;
    }
    else if (lin_vel_x == - COMPARE_VEL_LIN) 
    {
        // back off
        msgControl.data = CMD_BACKOFF;
    }
    else if (ang_vel_z == COMPARE_VEL_ANG)
    {   
        // Rotate left
        msgControl.data = CMD_ROTATELEFT;
    }
    else if (ang_vel_z == - COMPARE_VEL_ANG)
    {   
        // Rotate right
        msgControl.data = CMD_ROTATERIGHT;
    }
    else if (lin_vel_x == COMPARE_VEL_STOP && ang_vel_z == COMPARE_VEL_STOP)
    {
        // Stop robot
        msgControl.data = CMD_STOP;
    }

    ROS_INFO("linear.x = %0.2f", lin_vel_x);
    ROS_INFO("angular.z = %0.2f", ang_vel_z);
}

#define ORIGINAL_VERSION

#ifdef ORIGINAL_VERSION
int main(int argc, char** argv)
{
    ros::init(argc, argv, "control_hardware");

    ros::NodeHandle hNode;
    
    // Create a publisher
    ros::Publisher pub_control = hNode.advertise<std_msgs::UInt16>("cmd_motor", 10);
    ros::Rate loop_rate(10);
    
    // Subscriber. Teleop_keyboard transfer data to.
    ros::Subscriber sub_teleop = hNode.subscribe("/cmd_vel", 100, &cmdvelCallback); 
    
    while(ros::ok())
    {
        pub_control.publish(msgControl);

        ros::spinOnce();
        loop_rate.sleep();        
    }
}
#endif  //ORIGINAL_VERSION

#ifdef VERSION_TEST
int main(int argc, char** argv)
{
    ros::init(argc, argv, "control_hardware");

    ros::NodeHandle hNode;
    
    // Create a publisher
    ros::Publisher pub_control = hNode.advertise<std_msgs::UInt16>("servo", 10);
    ros::Rate loop_rate(10);
    
    // Subscriber. Teleop_keyboard transfer data to.
    ros::Subscriber sub_teleop = hNode.subscribe("/cmd_vel", 100, &cmdvelCallback); 
    
    while(ros::ok())
    {
        pub_control.publish(msgControl);

        ros::spinOnce();
        loop_rate.sleep();        
    }
}
#endif  //VERSION_TEST
