#!/usr/bin/env python  
import rospy
import tf_conversions
import tf2_ros
import geometry_msgs.msg
from nav_msgs.msg import Odometry 


def handle_odom_position(msg):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "base_footprint"
    t.child_frame_id = "odom_encoder"
    t.transform.translation.x = msg.position.x
    t.transform.translation.y = msg.position.y
    t.transform.translation.z = 0
    t.transform.rotation.x = 0 
    t.transform.rotation.y = 0
    t.transform.rotation.z = 0
    t.transform.rotation.w = 0

    br.sendTransform(t)

if __name__ == '__main__':
      rospy.init_node('tf_broadcaster_odom')
      rospy.Subscriber('/odom', Odometry, handle_odom_position)
      rospy.spin()
