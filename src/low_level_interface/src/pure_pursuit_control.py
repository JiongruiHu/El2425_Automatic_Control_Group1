#!/usr/bin/env python
import rospy
from tf.transformations import euler_from_quaternion
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
        while self.path != []:
            goal = self.path[0]
            if hasattr(self, 'car_pose'):
                while not (rospy.is_shutdown() or self.reach_goal(goal)):
                    lli_msg.steering = -self.controller(goal)
                    self.car_control_pub.publish(lli_msg)
                    rate.sleep()
                self.path.remove(goal)
                # goal = self.path[0]
        lli_msg.velocity = 0
        self.car_control_pub.publish(lli_msg)

    def controller(self,goal):
        xr, yr = self.car_pose.pose.pose.position.x, self.car_pose.pose.pose.position.y
        # heading = self.car_pose.twist.twist.angular.z
        xo, yo = self.car_pose.pose.pose.orientation.x, self.car_pose.pose.pose.orientation.y
        zo, w = self.car_pose.pose.pose.orientation.z, self.car_pose.pose.pose.orientation.w
        current_heading = euler_from_quaternion([xo, yo, zo, w])[2]
        print('goal',goal)
        L = 0.32
        d = dist((xr,yr), goal)
        for g in self.path:
            distance = dist((xr,yr), g)
            if distance < d:
                d = distance
                goal = g
                self.path.remove(g)
                break
        xg, yg = goal[0], goal[1]
        # ld = sqrt((xg - xr)**2 + (yg - yr)**2)
        ld = d
        des_heading = arctan2((yg - yr), (xg - xr))
        print('des_head',des_heading*180/pi)
        headErr = des_heading - current_heading
        # print("headErrOriginal", headErr)
        if headErr > pi:
            headErr = -2 * pi + headErr
        if headErr < -1 * pi:
            headErr = 2 * pi + headErr
        #print('phi',headErr)
        # print('difference_phi',phi*180/pi)
        curv = 2 * sin(headErr) / ld
        des_phi = arctan(L * curv)
        #print('des_phi',des_phi)

        if headErr > pi/2 or des_phi > pi/4:  # or 100
            steering = pi/4
        elif headErr < -pi/2 or des_phi < -pi/4:  # or -100
            steering = -pi/4
        else:
            steering = des_phi
        print('real steering',(steering*180/pi))
        print('\n')
        return int(100/(pi/4)*steering)

    def reach_goal(self, goal):
        xr, yr = self.car_pose.pose.pose.position.x, self.car_pose.pose.pose.position.y
        tol = 0.2
        if dist((xr,yr), goal) <= tol:
            return True
        else:
            return False


    def car_pose_cb(self, car_pose_msg):
        self.car_pose = car_pose_msg


def dist(p1, p2):
    return sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)


if __name__ == "__main__":

    rospy.init_node('path_follow')
    speed = 15
    try:
        PurePursuit()
    except rospy.ROSInterruptException:
        pass
    
