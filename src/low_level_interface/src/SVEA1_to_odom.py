#!/usr/bin/env python

import rospy

import tf
from tf import transformations as t
##from nav_msgs.msg import Odometry

# def transform_to_odom(msg):
#     br = tf.TransformBroadcaster()
#     pos = msg.pose.pose.position
#     ori = msg.pose.pose.orientation
#     br.sendTransform((pos.x,pos.y,pos.z),
#                      (ori.x,ori.y,ori.z,ori.w),
#                      rospy.Time.now(),
#                      "odom",
#                      "SVEA1")

if __name__ == '__main__':
    rospy.init_node('odom_inv_transformer')

    listener = tf.TransformListener()


    br = tf.TransformBroadcaster()

    rate = rospy.Rate(10)
    print("So far so good")
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('SVEA1', 'qualisys', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("Translation Error")
            continue
        transform = t.concatenate_matrices(t.translation_matrix(trans), t.quaternion_matrix(rot))
        inversed_transform = t.inverse_matrix(transform)
        translation = t.translation_from_matrix(inversed_transform)
        quaternion = t.quaternion_from_matrix(inversed_transform)
        br.sendTransform(translation, quaternion, rospy.Time.now(), "qualisys", "SVEA1")

        rate.sleep()