#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <iostream>

// Initialize ROS publishers
ros::Publisher pub_goal;
ros::Publisher pub_initial_2d;

// Take move_base_simple/goal as input and publish goal_2d
void goalCallback(const geometry_msgs::PoseStamped &goal)
{
  geometry_msgs::PoseStamped rpyGoal;
  rpyGoal.header.frame_id = "map";
  rpyGoal.header.stamp = goal.header.stamp;
  rpyGoal.pose.position.x = goal.pose.position.x;
  rpyGoal.pose.position.y = goal.pose.position.y;
  rpyGoal.pose.position.z = 0;
  tf::Quaternion q(0, 0, goal.pose.orientation.z, goal.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  rpyGoal.pose.orientation.x = 0;
  rpyGoal.pose.orientation.y = 0;
  rpyGoal.pose.orientation.z = yaw;
  rpyGoal.pose.orientation.w = 0;
  pub_goal.publish(rpyGoal);
}

// Take initialpose as input ad publish iitial_2d
void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped &pose)
{
  geometry_msgs::PoseStamped rpyPose;
  rpyPose.header.frame_id = "map";
  rpyPose.header.stamp = pose.header.stamp;
  rpyPose.pose.position.x = pose.pose.pose.position.x;
  rpyPose.pose.position.y = pose.pose.pose.position.y;
  rpyPose.pose.position.z = 0;
  tf::Quaternion q(0, 0, pose.pose.pose.orientation.z, pose.pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  rpyPose.pose.orientation.x = 0;
  rpyPose.pose.orientation.y = 0;
  rpyPose.pose.orientation.z = yaw;
  rpyPose.pose.orientation.w = 0;
  pub_initial_2d.publish(rpyPose);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rviz_click_to_2d");
  ros::NodeHandle h_Node;
  pub_goal = h_Node.advertise<geometry_msgs::PoseStamped>("goal_2d", 0);
  pub_initial_2d = h_Node.advertise<geometry_msgs::PoseStamped>("initial_2d", 0);
  ros::Subscriber sub_goal = h_Node.subscribe("move_base_simple/goal", 0, goalCallback);
  ros::Subscriber sub_initial_2d = h_Node.subscribe("initialpose", 0, initialPoseCallback);
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
