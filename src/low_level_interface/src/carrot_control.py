#! /usr/bin/env python 
# -*- coding: utf-8 -*-
import rospy as rp
import numpy as np
from path_points import path_points
from low_level_interface.msg import lli_ctrl_request
from nav_msgs.msg import Odometry

allPoints = path_points('linear')
pointIndex = 0

## Arguments of data need adjustment based on Mocap
def controller(data):
    
    kP = 1*100/(np.pi/4)
    global allPoints
    global pointIndex
    global toleranceLimit
    curr_point = allPoints[pointIndex]
    data_point = (data.pose.pose.position.x, data.pose.pose.position.y)
    while distance(data_point, curr_point) < toleranceLimit:
        pointIndex += 1
        if pointIndex >= allPoints.size():
            control.publish(0,0,0,0,0,0)
            exit()
        curr_point = allPoints[pointIndex]

    nextPoint = curr_point
    xdiff = nextPoint[0] - data.pose.pose.position.x
    ydiff = nextPoint[1] - data.pose.pose.position.y
    desHeading = np.arctan2(ydiff,xdiff)
    currentHeading = data.pose.pose.orientation.z

    headErr = desHeading - currentHeading
    if headErr > np.pi:
        headErr = 2*np.pi - headErr
    if headErr < -1*np.pi:
        headErr = -2*np.pi - headErr
    print("headErr",headErr)
    steering = int(-kP*(headErr))
    print("Steering",steering)
    return steering

def distance(point1, point2):
    sqr_sum = (point1[0] - point2[0])**2 + (point1[1] - point2[1])**2
    return np.sqrt(sqr_sum)

def callback(data):
    global speed
    steering = controller(data)
    if steering > 100:
	    steering = 100
    if steering < -100:
	    steering = -100
    lli_msg = lli_ctrl_request()
    lli_msg.velocity = speed
    lli_msg.steering = steering
    control.publish(lli_msg)

def listener():
    rp.init_node("carrot_control", anonymous = True)
    rp.Subscriber("SVEA1/odom", Odometry, callback)
    rp.spin()


if __name__ == "__main__":
    allPoints = tuple(path_points("linear"))
    assert(len(allPoints) == 36)
    toleranceLimit = 0.2
    speed = 10
    control = rp.Publisher("lli/ctrl_request", lli_ctrl_request, queue_size = 10)
    listener()

