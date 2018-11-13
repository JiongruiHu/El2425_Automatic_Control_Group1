#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from low_level_interface.msg import lli_ctrl_request
from numpy import *

rospy.init_node('E_stop')
car_speed = rospy.Publisher("lli/ctrl_request", lli_ctrl_request, queue_size=10)

def lidar_cb(data):
    speed = 10
    msg = lli_ctrl_request()
    msg.velocity = speed
    angles = arange(data.angle_min, data.angle_max+data.angle_increment, data.angle_increment)
    #print(angles)     
    ranges = data.ranges
    threshold_dist = 1
    for i in range(len(angles)):
        if abs(angles[i]) > pi-pi/4:
            if ranges[i] < threshold_dist:
                speed = -10
    msg.velocity = speed
    car_speed.publish(msg)

 

Lidar_sub = rospy.Subscriber("/scan", LaserScan, lidar_cb)
rospy.spin()

