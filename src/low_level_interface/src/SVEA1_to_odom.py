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

    rate = rospy.Rate(10.0)
    print("So far so good")
    listener.waitForTransform("/qualisys", "SVEA1", rospy.Time(), rospy.Duration(4.0))
    while not rospy.is_shutdown():
        try:
            now = rospy.Time.now()
            listener.waitForTransform("SVEA1", "/qualisys", now, rospy.Duration(4.0))
            (trans,rot) = listener.lookupTransform('SVEA1', 'qualisys', rospy.Time(0))
        except (tf.LookupException):
            print("Lookup")
            continue
        except (tf.ConnectivityException):
            print("Connectivity")
            continue
        except (tf.ExtrapolationException ):
            print("Extrapolation")
        transform = t.concatenate_matrices(t.translation_matrix(trans), t.quaternion_matrix(rot))
        inversed_transform = t.inverse_matrix(transform)
        translation = t.translation_from_matrix(inversed_transform)
        quaternion = t.quaternion_from_matrix(inversed_transform)
        br.sendTransform(translation, quaternion, rospy.Time.now(), "odom", "base_link")
        print("Sleeping")
        rate.sleep()
        print("Slept")