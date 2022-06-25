#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <sstream>
#include <math.h>

std_msgs::Float32 msgControl;

// Create a command callback
void cmdvelCallback(const geometry_msgs::Twist::ConstPtr& keyboard_control)
{
    if (keyboard_control->linear.x == 0)
    {
        msgControl.data = 0;
    }   
    if (keyboard_control->linear.x == 0.02)
    {
        msgControl.data = round(0.02);
    }  
    if (keyboard_control->linear.x == 0.04)
    {
        msgControl.data = round(0.04 * 100) / 100;
    }  
    if (keyboard_control->linear.x == 0.06)
    {
        msgControl.data = round(0.06 * 100) / 100;
    }  
    if (keyboard_control->linear.x == 0.08)
    {
        msgControl.data = round(0.08 * 100) / 100;
    }  
    if (keyboard_control->linear.x == 0.1)
    {
        msgControl.data = round(0.1 * 100) / 100;
    }  
    if (keyboard_control->linear.x == -0.02)
    {
        msgControl.data = round(-0.02 * 100) / 100;
    }  
    if (keyboard_control->linear.x == -0.04)
    {
        msgControl.data = round(-0.04 * 100) / 100;
    }  
    if (keyboard_control->linear.x == -0.06)
    {
        msgControl.data = round(-0.06 * 100) / 100;
    }  
    if (keyboard_control->linear.x == -0.08)
    {
        msgControl.data = round(-0.08 * 100) / 100;
    }  
    if (keyboard_control->linear.x == -0.1)
    {
        msgControl.data = round(-0.1 * 100) / 100;
    }  
    if (keyboard_control->angular.z == 0.1)
    {
        msgControl.data = 0.1;
    }  
    if (keyboard_control->angular.z == 0.2)
    {
        msgControl.data = 0.2;
    }  
    if (keyboard_control->angular.z == 0.3)
    {
        msgControl.data = 0.3;
    }  
    if (keyboard_control->angular.z == 0.4)
    {
        msgControl.data = 0.4;
    }  
    if (keyboard_control->angular.z == 0.5)
    {
        msgControl.data = 0.5;
    }  
    if (keyboard_control->angular.z == -0.1)
    {
        msgControl.data = -0.1;
    }  
    if (keyboard_control->angular.z == -0.2)
    {
        msgControl.data = -0.2;
    }  
    if (keyboard_control->angular.z == -0.3)
    {
        msgControl.data = -0.3;
    }  
    if (keyboard_control->angular.z == -0.4)
    {
        msgControl.data = -0.4;
    }  
    if (keyboard_control->angular.z == -0.5)
    {
        msgControl.data = -0.5;
    }  
}

#define ORIGINAL_VERSION

#ifdef ORIGINAL_VERSION
int main(int argc, char** argv)
{
    ros::init(argc, argv, "control_hardware");

    ros::NodeHandle hNode;
    
    // Create a publisher
    ros::Publisher pub_control = hNode.advertise<std_msgs::Float32>("cmd_motor", 10);
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
