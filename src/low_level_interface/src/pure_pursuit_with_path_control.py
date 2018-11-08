#!/usr/bin/env python
import rospy
from tr.transformations import euler_from_quaternion
from numpy import *
from path_points import path_points
from low_level_interface.msg import lli_ctrl_request
from nav_msgs.msg import Odometry


class PurePursuit(object):
    def __init__(self):
        self.path = path_points('circle')
        # self.path = [[-1.439,-1.3683],[0.245,-1.88676]]
        # path = self.path
        # Subscribe to the topics
        self.car_pose_sub = rospy.Subscriber("SVEA1/odom", Odometry, self.car_pose_cb)

        rospy.loginfo(self.car_pose_sub)
        # init Publisher
        self.car_control_pub = rospy.Publisher("lli/ctrl_request", lli_ctrl_request, queue_size=10)
        rate = rospy.Rate(10)
        # goal = self.path[0]
        lli_msg = lli_ctrl_request()
        lli_msg.velocity = speed
        self.ld = 0.2

        if hasattr(self, 'car_pose'):
            while not (rospy.is_shutdown() or len(self.path) == 0):
                goal = self.choose_point()
                lli_msg.steering = -self.controller(goal)
                self.car_control_pub.publish(lli_msg)
                rate.sleep()
                # goal = self.path[0]
        lli_msg.velocity = 0
        self.car_control_pub.publish(lli_msg)

    def controller(self,goal):
        xr, yr = self.car_pose.pose.pose.position.x, self.car_pose.pose.pose.position.y
        # heading = self.car_pose.twist.twist.angular.z
        xo, yo = self.car_pose.pose.pose.orientation.x, self.car_pose.pose.pose.orientation.y
        zo, w = self.car_pose.pose.pose.orientation.z, self.car_pose.pose.pose.orientation.w
        heading = euler_from_quaternion([xo, yo, zo, w])[2]
        xg, yg = goal[0],goal[1]  # self.path
        L = 0.32
        ld = sqrt((xg - xr)**2 + (yg - yr)**2)
        des_heading = arctan2((yg - yr), (xg - xr))
        print('des_head',des_heading)
        headErr = desHeading - currentHeading
        # print("headErrOriginal", headErr)
        if headErr > np.pi:
            headErr = -2 * np.pi + headErr
        if headErr < -1 * np.pi:
            headErr = 2 * np.pi + headErr
        print('phi',headErr)
        # print('difference_phi',phi*180/pi)
        curv = 2 * sin(headErr) / ld
        des_phi = arctan(L * curv)
        print('des_phi',des_phi)

        if phi > pi/2 or des_phi > pi/4:  # or 100
            phi = pi/4
        elif phi < -pi/2 or des_phi < -pi/4:  # or -100
            phi = -pi/4
        else:
            phi = des_phi
        # print('real phi',(phi*180/pi))
        return int(100/(pi/4)*phi)

    def reach_goal(self, goal):
        xr, yr = self.car_pose.pose.pose.position.x, self.car_pose.pose.pose.position.y
        tol = 0.4
        if dist((xr,yr), goal) <= tol:
            return True
        else:
            return False


    def car_pose_cb(self, car_pose_msg):
        self.car_pose = car_pose_msg

    def choose_point(self):
        xr, yr = self.car_pose.pose.pose.position.x, self.car_pose.pose.pose.position.y
        while(len(self.path) > 1):
            examined_point = self.path[0]
            distance = dist((xr,yr), examined_point)
            if distance > self.ld:
                return examined_point
            self.path.remove(examined_point)
        goal_point = self.path[0]
        self.path.remove(goal_point)
        return goal_point


def dist(p1, p2):
    return sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)


if __name__ == "__main__":

    rospy.init_node('path_follow')
    speed = 10
    try:
        PurePursuit()
    except rospy.ROSInterruptException:
        pass
    
