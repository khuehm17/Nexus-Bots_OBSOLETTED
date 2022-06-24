#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

void odomCallback(const nav_msgs::Odometry& odom_msg)
{
  geometry_msgs::TransformStamped odom_tf;
  tf::TransformBroadcaster tf_br;
  odom_tf.header.stamp = ros::Time::now();
  odom_tf.header.frame_id = "odom";
  odom_tf.child_frame_id = "omni_robot";

  odom_tf.transform.translation.x = odom_msg.pose.pose.position.x;
  odom_tf.transform.translation.y = odom_msg.pose.pose.position.y;
  odom_tf.transform.translation.z = odom_msg.pose.pose.position.z;
  odom_tf.transform.rotation = odom_msg.pose.pose.orientation;
  tf_br.sendTransform(odom_tf);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom_tf");
  ros::NodeHandle h_Node;
  ros::Subscriber odom_tf_sub = h_Node.subscribe("/odom", 100, odomCallback);

  ros::spin();
  return 0;
}
