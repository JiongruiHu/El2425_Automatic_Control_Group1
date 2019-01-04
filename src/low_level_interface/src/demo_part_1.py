#!/usr/bin/env python
import rospy
from tf.transformations import euler_from_quaternion
from numpy import *
import matplotlib as mp
from sensor_msgs.msg import LaserScan
from path_points import path_points
from low_level_interface.msg import lli_ctrl_request
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray


class ParkingControl(object):
    ## Initializes the node and creates publishers, subscribers, and runs the rest of the program
    def __init__(self):
        # self.path = path_points('figure-8')
        self.path = []
        self.Estop = 0
        self.car_heading = 0
        # Subscribe to the MOCAP, lidar and race course topics
        self.car_pose_sub = rospy.Subscriber("SVEA1/odom", Odometry, self.car_pose_cb)
        self.Lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_cb)
        self.race_path_sub = rospy.Subscriber("/race_course", PoseArray, self.race_path_cb)

        rospy.loginfo(self.car_pose_sub)
        # init Publisher
        self.car_control_pub = rospy.Publisher("lli/ctrl_request", lli_ctrl_request, queue_size=10)
        self.rate = rospy.Rate(10)
        # goal = self.path[0]
        self.steering_angle = 0

        self.ld = 0.6
        self.xs = []
        self.ys = []
        while len(self.path) == 0:
            self.rate.sleep()
        self.__pure_pursuit()

    ## Runs full pure pursuit algorithm, including finding the next goal point, finding correct control signals,
    ## publishing and taking care of auxillary functions.
    def __pure_pursuit(self):
        lli_msg = lli_ctrl_request()
        lli_msg.velocity = speed
        savetxt("/home/nvidia/El2425_Automatic_Control_Group1/planned_path.csv", array(self.path), delimiter=",")
        while len(self.path) > 0:
            if hasattr(self, 'car_pose'):
                while not (rospy.is_shutdown() or len(self.path) == 0):
                    goal = self.choose_point()
                    lli_msg.velocity,lli_msg.steering = self.controller(goal)
                    self.car_control_pub.publish(lli_msg)
                    self.rate.sleep()
                # goal = self.path[0]
        lli_msg.velocity = 0
        self.car_control_pub.publish(lli_msg)
        pose_arr = array([self.xs, self.ys])
        savetxt("/home/nvidia/El2425_Automatic_Control_Group1/real_path.csv", pose_arr, delimiter=",")

    ## Pure pursuit controller.Takes a point given by choose point and calculates the curvature necessary to reach it.
    def controller(self,goal):
        xr, yr = self.car_pose.pose.pose.position.x, self.car_pose.pose.pose.position.y
        self.xs.append(xr)      
        self.ys.append(yr)      # Saves pose in vector
        xo, yo = self.car_pose.pose.pose.orientation.x, self.car_pose.pose.pose.orientation.y
        zo, w = self.car_pose.pose.pose.orientation.z, self.car_pose.pose.pose.orientation.w

        self.current_heading = euler_from_quaternion([xo, yo, zo, w])[2]    # Compensates for quaternion format
        xg, yg = goal[0],goal[1]  # Goal point coordinates
        L = 0.32            # Distance between wheel axes
        ld = sqrt((xg - xr)**2 + (yg - yr)**2)  # Distance between car and goal point
        des_heading = arctan2((yg - yr), (xg - xr))
        headErr = des_heading - self.current_heading    
        
        # Compensate for potential angle singularity
        if headErr > pi:
            headErr = -2 * pi + headErr
        if headErr < -1 * pi:
            headErr = 2 * pi + headErr
        # Calculate necessary curvature and corresponding steering angle
        curv = 2 * sin(headErr) / ld
        des_phi = arctan(L * curv)
        
        # Compensate for control limit
        if headErr > pi/2 or des_phi > pi/4:  
            phi = pi/4
        elif headErr < -pi/2 or des_phi < -pi/4: 
            phi = -pi/4
        else:
            phi = des_phi
        v = self.speed_control(phi)     # Get velocity signal
        self.steering_angle = phi
        return v, -int(100/(pi/4)*phi)  # Convert to signal units

    ## Speed control, including emergency stop whenever necessary
    def speed_control(self, phi):
        if self.Estop == 0:
            speed = self.__choose_speed(phi)
        else:
            speed = -100
        return speed

    ## Pure speed control. 
    ## Currently returns constant speed, but can be modified to give velocity as a ReLU function of angle by changing parameters.
    def __choose_speed(self, phi):
        max_speed = 20
        min_speed = 20
        min_ang = pi / 48
        max_ang = pi / 6
        if abs(phi) < min_ang:
            speed = max_speed
        elif abs(phi) > max_ang:
            speed = min_speed
        else:
            speed = max_speed - (max_ang - min_ang) * (max_speed - min_speed)
        return speed

    ## Returns boolean, saying whether we are close enough to a given goal point.
    def reach_goal(self, goal):
        xr, yr = self.car_pose.pose.pose.position.x, self.car_pose.pose.pose.position.y
        tol = 0.2
        if dist((xr,yr), goal) <= tol:
            return True
        else:
            return False
        
    ## Look-ahead distance part of pure_pursuit algorithm. Returns a goal point.
    def choose_point(self):
        xr, yr = self.car_pose.pose.pose.position.x, self.car_pose.pose.pose.position.y
        while(len(self.path) > 1):
            examined_point = self.path[0]
            distance = dist((xr,yr), examined_point)
            if distance > self.ld:
                return examined_point
            self.path.remove(examined_point)
            self.path.append(examined_point)
        goal_point = self.path[0]
        self.path.remove(goal_point)
        return goal_point

    ## Updates car_pose based on MOCAP.
    def car_pose_cb(self, car_pose_msg):
        self.car_pose = car_pose_msg

    ## Emergency stop. Looks in a cone around the current heading angle, given by bicycle model, 8 dm in front of the lidar.
    def lidar_cb(self, data):
        threshold_dist = 0.8
        beta = arctan(tan(self.steering_angle) * 0.5)       # Heading angle
        angles = arange(data.angle_min, data.angle_max + data.angle_increment, data.angle_increment)
        ranges = data.ranges
        Estop = 0               # False
        for i in range(len(angles)):
            if angles[i] < -pi + beta + pi / 11 or angles[i] > -pi + beta - pi / 11:    # Angle within cone
                if ranges[i] < threshold_dist:      
                    Estop = 1
                    print("E-stop at dist:" + str(ranges[i]) + " and angle: " + str(angles[i]))
        self.Estop = Estop


    ## Callback function of the race course topic. Saves the message as an array of 2D vectors if there is none
    def race_path_cb(self, data):
        if len(self.path) == 0:
            path = []
            for pose in data.poses:
                path.append((pose.position.x, pose.position.y))
            self.path = path
            print(path[0])

 

## Euclidian distance
def dist(p1, p2):
    return sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)


if __name__ == "__main__":

    rospy.init_node('path_follow')
    speed = 10
    try:
        ParkingControl()
    except rospy.ROSInterruptException:
        pass
    
