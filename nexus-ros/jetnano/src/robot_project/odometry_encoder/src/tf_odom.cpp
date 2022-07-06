#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

void odomCallback(const nav_msgs::Odometry& odom_msgs)
{
  geometry_msgs::TransformStamped odom_tf;
  tf::TransformBroadcaster odom_broadcaster;
  odom_tf.header.frame_id = "odom";
  odom_tf.child_frame_id = "base_footprint";
  
  odom_tf.transform.translation.x = odom_msgs.pose.pose.position.x;
  odom_tf.transform.translation.y = odom_msgs.pose.pose.position.y;
  odom_tf.transform.translation.z = odom_msgs.pose.pose.position.z;
  odom_tf.transform.rotation = odom_msgs.pose.pose.orientation;
  odom_broadcaster.sendTransform(odom_tf);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_odom");
  ros::NodeHandle h_Node;
  ros::Subscriber sub_odom = h_Node.subscribe("/odom", 50, odomCallback);
  ros::spin();
  return 0;
}
