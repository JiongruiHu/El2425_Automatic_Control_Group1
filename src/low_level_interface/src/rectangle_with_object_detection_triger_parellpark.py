#!/usr/bin/env python
import rospy
from low_level_interface.msg import lli_ctrl_request as msg_out
from tf.transformations import euler_from_quaternion
from numpy import *
from nav_msgs.msg import Odometry
from Parallel_parking_v3 import FollowThenPark
from sensor_msgs.msg import LaserScan

ros_out = msg_out()

ros_out.velocity = 10#speed can be regualted for demo

vel = 0;
steer =0;

NPOINTS = 100
DETECTED = False
LOOP = True

current_point=0
path=[]
yr = 0
xr = 0
xo = 0
yo = 0
zo = 0
w = 0
pub= rospy.Publisher('/lli/lli_ctrl_request',msg_out,queue_size=10)

def generate():
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
        path.append([xrange_[i],ymax)
    for i in arange(25):
        path.append([xmin,yrange_[i]])




def updatePos(msg):
    xr, yr = msg.pose.pose.position.x, msg.pose.pose.position.y
    xo, yo = msg.pose.pose.orientation.x, msg.pose.pose.orientation.y
    zo, w = msg.pose.pose.orientation.z, msg.pose.pose.orientation.w


def trace_path():
    if(dist((xr,xy),path[current_point])>0.5):
        current_point=(current_point+1)%len(path)
    move(path[current_point])

def move(goal):
    current_heading = euler_from_quaternion([xo, yo, zo, w])[2]
    xg, yg = goal[0],goal[1]
    des_heading = arctan2((yg - yr), (xg - xr))
    headErr = des_heading - current_heading
    if headErr > pi:
        headErr = -2 * pi + headErr
    if headErr < -1 * pi:
        headErr = 2 * pi + headErr
    print('difference_phi',phi*180/pi)
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


def lidar_cb(data):
    angles = arange(data.angle_min, data.angle_max + data.angle_increment, data.angle_increment)
    ranges = data.ranges
    obstacle_detected(angles,ranges)

def obstacle_detected(angles,ranges):
    threshold_dist = 0.6
    for i in range(len(angles)):
        if abs(angles[i]) < pi/6:
            if ranges[i] < threshold_dist:
                Estop = 1
        if abs(angles[i]) > pi/4:
            if ranges[i] < threshold_dist:
                DETECTED=True


if __name__ == '__main__':
    rospy.init_node('test_control',anonymous=True)
    rospy.Subscriber("SVEA1/odom", Odometry, updatePos)
    rospy.Subscriber("/scan", LaserScan, lidar_cb)
    rate = rospy.Rate(10)
    _point=[]
    while not rospy.is_shutdown() and LOOP:
        if( xr != 0 and yr != 0 ):
            if not DETECTED:
                trace_path()
            else:
                _x,_y = path[current_point]
                _x1,_y1 = path[(current_point+1)%NPOINTS]
                if(_x == _x1):#if this is the case that means car is in y direction
                    if(_y < _y1):#it is moving right(lower line of rectangle)
                        point=[xr,_y-1]
                        _point = point
                    elif(_y > _y1):
                        point=[xr,_y+1]
                        _point = point
                elif(_y == _y1):#car is moving in x direction
                    if(_x < _x1):
                        point=[_x-1,yr]
                        _point = point
                    elif(_x > _x1):#its goving forward
                        point=[_x+1,yr]
                        _point = point
                LOOP = False
                move(_point)
            rate.sleep()
            FollowThenPark()#not sure if its right way to call -as this and above code are using same subscribers. 
#TODO:Call Parallel Parking
