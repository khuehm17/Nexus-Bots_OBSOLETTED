#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

tf::TransformBroadcaster *p_odom_broadcaster;

void odomCallback(const geometry_msgs::PoseWithCovarianceStamped& odom_stamp)
{
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = ros::Time::now();
  odom_trans.header.frame_id = "base_footprint";
  odom_trans.child_frame_id = "omni_robot";
  // odom_trans.child_frame_id = "base_footprint";

  odom_trans.transform.translation.x = odom_stamp.pose.pose.position.x;
  odom_trans.transform.translation.y = odom_stamp.pose.pose.position.y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_stamp.pose.pose.orientation;
  p_odom_broadcaster->sendTransform(odom_trans);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_odometry");
  ros::NodeHandle hNode;

  tf::TransformBroadcaster odom_broadcaster;
  p_odom_broadcaster = &odom_broadcaster;

  /* Subscribe odom topic from microcontroller */
  ros::Subscriber odom_sub = hNode.subscribe("robot_pose_ekf/odom_combined", 1000, odomCallback);
  while(ros::ok())
  {
    ros::spinOnce();
  }
  return 0;
}
