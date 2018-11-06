import rospy
from numpy import *
from path_points.py import path_points
from low_level_interface.msgs import lli_ctrl_request
from nav_msgs.msg import Odometry


class PurePursuit(object):
    def __init__(self):
        self.path = path_points('linear')
        path = self.path
        # Subscribe to the topics
        self.car_pose_sub = rospy.Subscriber("SVEA1/odom", Odemetry, self.car_pose_cb)

        # init Publisher
        self.car_control_pub = rospy.Publisher("lli/ctrl_request", lli_ctrl_request, queue_size=10)
        rate = rospy.Rate(10)
        goal = self.path[0]
        lli_msg = lli_ctrl_request()
        lli_msg.velocity = speed
        while self.path != []:
            
            while not rospy.is_shutdown() and not self.reach_goal(goal):
                lli_msg.steering = self.controller(goal)
                self.car_control_pub.pub(lli_msg)
                rate.sleep()
            self.path.remove(goal)
            goal = self.path[0]

    def controller(self,goal):
        xr, yr = self.car_pose.pose.pose.position.x, self.car_pose.pose.pose.position.y
        heading = self.car_pose.twist.twist.angular.z

        xg, yg = goal[0],goal[1]  # self.path
        L = 0.32
        ld = sqrt((xg - xr)**2 + (yg - yr)**2)
        des_heading = arctan2((yg - yr),(xg - xr))
        phi = des_heading - heading
        curv = 2 * sin(phi) / ld
        des_phi = arctan(L * curv)

        if des_phi > pi/4:  # or 100
            phi = pi/4
        elif des_phi < -pi/4:  # or -100
            phi = -pi/4
        else:
            phi = des_phi
        return int(100/(pi/4)*phi)

    def reach_goal(self, goal):
        xr, yr = self.car_pose.pose.pose.position.x, self.car_pose.pose.pose.position.y
        tol = 0.1
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
    speed = 10
    try:
        PurePursuit()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
