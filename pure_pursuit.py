from numpy import *
import path_points.py
import rospy
from nav_msgs.msg import Odometry




def pure_pursuit():
    path = path_points
    phi = 0 # the steering angle

    #x, y = car.posistion
    #heading = car.heading

    #Subscribe to topics
    robot_pose_sub = rospy.Subscriber('carname/odom', Odemetry, robot_pose_cb)



    #Call back functions
    return phi


def robot_pose_cb(robot_pose_msg):
    xr = robot_pose_msg.pose.pose.position.x
    yr = robot_pose_msg.pose.pose.position.y
    theta = robot_pose_msg.twist.twist.angular.z
    L = 0.42
    #xg, yg = goal(x,y)
    xg, yg = 1,1
    desired_theta = arctan2(xg-xr, yg-yr)
    alpha = desired_theta - theta  # ??????
    ld = sqrt((xg - xr)**2 + (yg - yr)**2)
    curv = 2 * sin(alpha)/ld
    delta_desired = arctan(L * curv)
    if delta_desired > pi/4:
        delta = pi/4
    elif delta_desired < -pi/4:
        delta = -pi/4
    else: delta = delta_desired
    return []

def goal(xr,yr):

    return xr,yr

class robot():
    def __init__(self):
        self.L = 0.42


