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
#define COMPARE_VEL       (2) 

std_msgs::UInt16 msgControl;
float lin_vel_x, lin_vel_y, ang_vel_z;

// Create a command callback
void cmdvelCallback(const geometry_msgs::Twist::ConstPtr& keyboard_control)
{
    lin_vel_x = keyboard_control->linear.x;
    lin_vel_y = keyboard_control->linear.y;
    ang_vel_z = keyboard_control->angular.z;
    if (lin_vel_x == COMPARE_VEL)
    {
        // go traight ahead
        msgControl.data = CMD_ADVANCE;
    }
    else if (lin_vel_x == - COMPARE_VEL)
    {   
        // back off
        msgControl.data = CMD_BACKOFF;
    }
    else if (ang_vel_z == COMPARE_VEL)
    {   
        // Rotate left
        msgControl.data = CMD_ROTATELEFT;
    }
    else if (ang_vel_z == - COMPARE_VEL)
    {   
        // Rotate right
        msgControl.data = CMD_ROTATERIGHT;
    }

    ROS_INFO("linear.x = %0.2f", lin_vel_x);
    ROS_INFO("angular.z = %0.2f", ang_vel_z);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "control_hardware");

    ros::NodeHandle hNode;
    
    // Create a publisher
    ros::Publisher pub_control = hNode.advertise<std_msgs::UInt16>("servo", 10);
    ros::Rate loop_rate(10);
    
    // Subscriber. Teleop_keyboard transfer data to.
    ros::Subscriber sub_teleop = hNode.subscribe("turtle1/cmd_vel", 100, &cmdvelCallback); 
    
    while(ros::ok())
    {
        pub_control.publish(msgControl);

        ros::spinOnce();
        loop_rate.sleep();        
    }
}
