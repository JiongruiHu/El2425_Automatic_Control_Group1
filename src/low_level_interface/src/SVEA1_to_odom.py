#!/usr/bin/env python

import rospy

import tf
from nav_msgs.msg import Odometry

def transform_to_odom(msg):
    br = tf.TransformBroadcaster()
    pos = msg.pose.pose.position
    ori = msg.pose.pose.orientation
    br.sendTransform((-1*pos.x,-1*pos.y,-1*pos.z),
                     -1*(ori.x,ori.y,ori.z,ori.w),
                     rospy.Time.now(),
                     "odom",
                     "SVEA1")

if __name__ == '__main__':
    rospy.init_node('odom_pose_transformer')
    rospy.Subscriber('SVEA1/odom',
                     Odometry,
                     transform_to_odom)
    rospy.spin()