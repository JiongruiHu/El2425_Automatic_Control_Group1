import rospy as rp
import numpy as np
from path_points.py import path_points
from low_level_interface/msgs import lli_ctrl_request
from nav_msgs/msg import Odometry

## Arguments of data need adjustment based on Mocap
def controller(data):
    
    kP = 1*100/(pi/4)

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
    desHeading = np.arctan(ydiff/xdiff)
    currentHeading = data.twist.twist.angular.z
    steering = int(-kP*(desHeading - currentHeading))
    return steering

def distance(point1, point2):
    sqr_sum = (point1[0] - point2[0])**2 + (point1[1] - point2[1])**2
    return np.sqrt(sqr_sum)

def callback(data):
    
    steering = controller(data)
    control.publish(speed, steering, 0,0,0,0)

def listener():
    rp.init_node("carrot_control", anonymous = True)
    rp.Subscriber(Odometry, "CARNAME/odom", callback)         #Needs CARNAME
    rp.spin()


if __name__ == "__main__":
    allPoints = tuple(path_points("linear"))
    assert(allPoints.size == 36)
    pointIndex = 0
    toleranceLimit = 0.2
    speed = 10
    control = rp.Publisher("lli/ctrl_request", lli_ctrl_request, queue_size = 10)
    listener()
