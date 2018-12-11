#!/usr/bin/env python
import rospy
from low_level_interface.msg import lli_ctrl_request as msg_out
from tf.transformations import euler_from_quaternion
from numpy import *
from nav_msgs.msg import Odometry

ros_out = msg_out()

ros_out.velocity = 10

vel = 0
steer =0

STEP_VELOCITY = 5
STEP_STEERING = 20
NPOINTS = 100

current_point=0
path=[]
yr = 0
xr = 0
xo = 0
yo = 0
zo = 0
w = 0

def generate():
    global path
    xmin = -1
    xmax = 1
    ymin = -1.5
    ymax = 1.5
    xrange = linspace(xmin,xmax,25)
    yrange = linspace(ymin,ymax,25)
    yrange_ = linspace(ymax,ymin,25)
    xrange_ = linspace(xmax,xmin,25)
    for i in arange(25):
        path.append([xrange[i],ymin])
    for i in arange(25):
        path.append([xmax,yrange[i]])
    for i in arange(25):
        path.append([xrange_[i],ymax])
    for i in arange(25):
        path.append([xmin,yrange_[i]])


def updatePos(msg):
    global xr,yr, xo, yo, zo, w
    xr, yr = msg.pose.pose.position.x, msg.pose.pose.position.y
    xo, yo = msg.pose.pose.orientation.x, msg.pose.pose.orientation.y
    zo, w = msg.pose.pose.orientation.z, msg.pose.pose.orientation.w

def trace_path():
    global current_point, path
    while(dist((xr,yr),path[current_point])<0.5):
        current_point=(current_point+1)%len(path)
    move(path[current_point])

def move(goal):
    global pub, xo, yo, zo, w, xr, yr, ros_out
    current_heading = euler_from_quaternion([xo, yo, zo, w])[2]
    xg, yg = goal[0],goal[1]
    des_heading = arctan2((yg - yr), (xg - xr))
    ld = dist((xr, yr), (xg, yg))
    headErr = des_heading - current_heading
    if headErr > pi:
        headErr = -2 * pi + headErr
    if headErr < -1 * pi:
        headErr = 2 * pi + headErr
    # print('difference_phi',phi*180/pi)
    curv = 2 * sin(headErr) / ld
    des_phi = arctan(0.32 * curv)

    if headErr > pi/2 or des_phi > pi/4:  # or 100
        phi = pi/4
    elif headErr < -pi/2 or des_phi < -pi/4:  # or -100
        phi = -pi/4
    else:
        phi = des_phi
    print('real phi',(phi*180/pi))
    steer= -1*int(100/(pi/4)*phi)
    ros_out.steering = steer
    pub.publish(ros_out)


def dist(p1, p2):
    return sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)


if __name__ == '__main__':
    rospy.init_node('test_control',anonymous=True)
    pub = rospy.Publisher('/lli/ctrl_request', msg_out, queue_size=10)
    rospy.Subscriber("SVEA1/odom", Odometry, updatePos)
    rate = rospy.Rate(10)
    generate()
    while not rospy.is_shutdown():
        if abs(xr) > 0 or abs(yr) > 0:#zero_exception
            trace_path()
        rate.sleep()
        
