#!/usr/bin/env python
import rospy
from low_level_interface.msg import lli_ctrl_request as msg_out
from tf.transformations import euler_from_quaternion
from numpy import *
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from demo_part_2_A< import FollowThenPark


ros_out = msg_out()

ros_out.velocity = 14

vel = 0;
steer =0;

NPOINTS = 100
DETECTED = False
LOOP = True
WAITING_FOR_START = False

#current_point
path=[]
yr = 0
xr = 0
xo = 0
yo = 0
zo = 0
w = 0
pub= rospy.Publisher('lli/ctrl_request',msg_out,queue_size=10)

def generate():
    xmin = -1
    xmax = 1.5
    ymin = -1.8
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


def closest_point():

  global xr,yr,path
  closest=dist((xr,yr),path[0])
  point=0
  for i in range(0,len(path)):
    distance=dist((xr,yr),path[i])
    if(distance<closest):
      closest = distance
      point=i
  return point



def updatePos(msg):
    global xr,yr, xo, yo, zo, w
    xr, yr = msg.pose.pose.position.x, msg.pose.pose.position.y
    xo, yo = msg.pose.pose.orientation.x, msg.pose.pose.orientation.y
    zo, w = msg.pose.pose.orientation.z, msg.pose.pose.orientation.w


def trace_path():
    global current_point,path
    ld = 0.6
    if(dist((xr,yr),path[current_point])<ld):
        current_point=(current_point+1)%len(path)
    move(path[current_point])

def move(goal):
    global pub, xo, yo, zo, w, xr, yr, ros_out
    current_heading = euler_from_quaternion([xo, yo, zo, w])[2]
    xg, yg = goal[0],goal[1]
    des_heading = arctan2((yg - yr), (xg - xr))
    ld = dist((xr,yr),(xg,yg))
    headErr = des_heading - current_heading
    if headErr > pi:
        headErr = -2 * pi + headErr
    if headErr < -1 * pi:
        headErr = 2 * pi + headErr
    #print('difference_phi',phi*180/pi)
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
    global DETECTED, Estop, WAITING_FOR_START
    threshold_dist = 0.4
    none_detected = True
    for i in range(len(angles)):
        if abs(angles[i]) < pi/6:
            if ranges[i] < threshold_dist:
                Estop = 1
        if abs(angles[i] - pi/2) < pi/20:
            if ranges[i] < threshold_dist:
                if not WAITING_FOR_START:
                    DETECTED = True
                else:
                    none_detected = False
    if none_detected:
         WAITING_FOR_START = False



if __name__ == '__main__':
    rospy.init_node('test_control',anonymous=True)
    rospy.Subscriber("SVEA1/odom", Odometry, updatePos)
    generate()
    rospy.Subscriber("/scan", LaserScan, lidar_cb)
    rate = rospy.Rate(10)
    _point=[]
    while(xr == 0 and yr == 0):
        rate.sleep()
    current_point = closest_point()

    while not rospy.is_shutdown() and LOOP:
        if( xr != 0 and yr != 0 ):
            if (not DETECTED) or WAITING_FOR_START:
                print("Not yet detected")
                print("closest point", path[current_point])
                trace_path()
            else:
                _x,_y = path[current_point]
                _x1,_y1 = path[(current_point+1)%NPOINTS]
                if(_x == _x1):
                    if(_y < _y1):
                        point=[xr,_y-1]
                        _point = point
                    elif(_y > _y1):
                        point=[xr,_y+1]
                        _point = point
                elif(_y == _y1):
                    if(_x < _x1):
                        point=[_x-1,yr]
                        _point = point
                    elif(_x > _x1):
                        point=[_x+1,yr]
                        _point = point

                # LOOP = False
                move(_point)
                print("current pose", xr,yr)
                print("follow point",_point)
                FollowThenPark()
                current_point = closest_point()
                WAITING_FOR_START = True
                DETECTED = False
            rate.sleep()
            print("before follow then park")


    #TODO:Call Parallel Parking
