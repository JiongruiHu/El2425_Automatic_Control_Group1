#!/usr/bin/env python

import rospy

import tf
from nav_msgs.msg import Odometry

def transform_to_odom(msg):
    br = tf.TransformBroadcaster()
    br.sendTransform(msg.pose.pose.position,
                     msg.pose.pose.orientation,
                     rospy.Time.now(),
                     "odom",
                     "SVEA1")

if __name__ == '__main__':
    rospy.init_node('odom_pose_transformer')
    rospy.Subscriber('SVEA1/odom',
                     Odometry,
                     transform_to_odom)
    rospy.spin()