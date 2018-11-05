import rospy
from numpy import *
from path_points.py import path_points
from low_level_interface.msgs import lli_ctrl_request
from nav_msgs.msg import Odometry


class PurePursuit(object):
    def __init__(self):
        self.path = path_points('linear')

        # Subscribe to the topics
        self.car_pose_sub = rospy.Subscriber("carname/odom", Odemetry, self.car_pose_cb)

        # init Publisher
        self.car_control_pub = rospy.Publisher("lli/ctrl_request", lli_ctrl_request, queue_size=10)

        lli_msg = lli_ctrl_request()
        lli_msg.steering = self.controller()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.car_control_pub.pub(lli_msg)
            rate.sleep()

    def controller(self):
        xr, yr = self.car_pose.pose.pose.position.x, self.car_pose.pose.pose.position.y
        heading = self.car_pose.twist.twist.angular.z
        xg, yg = 1, 1  # self.path
        L = 0.42
        ld = sqrt((xg - xr)**2 + (yg - yr)**2)
        des_heading = arctan2((xg - xr), (yg - yr))
        phi = des_heading - heading
        curv = 2 * sin(phi) / ld
        des_phi = arctan(L * curv)

        if des_phi > pi/4:  # or 100
            phi = pi/4
        elif des_phi < -pi/4:  # or -100
            phi = -pi/4
        else:
            phi = des_phi
        return phi

    def car_pose_cb(self, car_pose_msg):
        self.car_pose = car_pose_msg


if __name__ == "__main__":

    rospy.init_node('path_follow')
    try:
        PurePursuit()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
