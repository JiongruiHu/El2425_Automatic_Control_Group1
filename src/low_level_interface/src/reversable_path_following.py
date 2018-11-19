#!/usr/bin/env python
import rospy
from tf.transformations import euler_from_quaternion
from numpy import *
import matplotlib as mp
from sensor_msgs.msg import LaserScan
from path_points import path_points
from low_level_interface.msg import lli_ctrl_request
from nav_msgs.msg import Odometry


class PurePursuit(object):
    def __init__(self):
        self.path = path_points('figure-8')
        self.Estop = 0
        self.car_heading = 0
        # Subscribe to the topics
        self.car_pose_sub = rospy.Subscriber("SVEA1/odom", Odometry, self.car_pose_cb)
        self.Lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_cb)
        self.reversed = True
        rospy.loginfo(self.car_pose_sub)
        # init Publisher
        self.car_control_pub = rospy.Publisher("lli/ctrl_request", lli_ctrl_request, queue_size=10)

        # goal = self.path[0]

        self.ld = 0.5
        self.xs = []
        self.ys = []
        self.__pure_pursuit()

    def __pure_pursuit(self):
        rate = rospy.Rate(10)
        lli_msg = lli_ctrl_request()
        lli_msg.velocity = speed
        while len(self.path) > 0:
            if hasattr(self, 'car_pose'):
                while not (rospy.is_shutdown() or len(self.path) == 0):
                    goal = self.choose_point()
                    lli_msg.velocity,lli_msg.steering = self.reversed_controller(goal)
                    self.car_control_pub.publish(lli_msg)
                    rate.sleep()
                # goal = self.path[0]
        lli_msg.velocity = 0
        self.car_control_pub.publish(lli_msg)
        pose_arr = array([self.xs, self.ys])
        savetxt("/home/nvidia/catkin_ws/real_path.csv", pose_arr, delimiter=",")

    ##Both following functions choose between reversed and forward depending on self.reversed
    def controller(self,goal):
        if self.reversed:
            return self.reversed_controller(goal)
        else:
            return self.forward_controller(goal)

    def lidar_cb(self,data):
        if self.reversed:
            return self.reversed_lidar_cb(data)
        else:
            return self.forward_lidar_cb(data)

    def change_to_reversed(self):
        lli_msg = lli_ctrl_request()
        lli_msg.velocity = -10
        self.car_control_pub.publish(lli_msg)
        lli_msg.velocity = 0
        self.car_control_pub.publish(lli_msg)
        self.reversed = True

    def change_to_forward(self):
        self.reversed = False


    def reversed_controller(self,goal):
        xr, yr = self.car_pose.pose.pose.position.x, self.car_pose.pose.pose.position.y
        #self.car_heading = self.car_pose.twist.twist.angular.z
        self.xs.append(xr)
        self.ys.append(yr)
        xo, yo = self.car_pose.pose.pose.orientation.x, self.car_pose.pose.pose.orientation.y
        zo, w = self.car_pose.pose.pose.orientation.z, self.car_pose.pose.pose.orientation.w

        recieved_heading = euler_from_quaternion([xo, yo, zo, w])[2]
        self.current_heading =  recieved_heading - pi if recieved_heading > 0 else recieved_heading + pi        #Ensures that the heading is 180 away from the recorded
        xg, yg = goal[0],goal[1]  # self.path
        L = 0.32
        ld = sqrt((xg - xr)**2 + (yg - yr)**2)
        des_heading = arctan2((yg - yr), (xg - xr))
        print('des_head',des_heading)
        headErr = des_heading - self.current_heading
        # print("headErrOriginal", headErr)
        if headErr > pi:
            headErr = -2 * pi + headErr
        if headErr < -1 * pi:
            headErr = 2 * pi + headErr
        print('phi',headErr)
        # print('difference_phi',phi*180/pi)
        curv = 2 * sin(headErr) / ld
        des_phi = arctan(L * curv)
        print('des_phi',des_phi)

        if headErr > pi/2 or des_phi > pi/4:  # or 100
            phi = pi/4
        elif headErr < -pi/2 or des_phi < -pi/4:  # or -100
            phi = -pi/4
        else:
            phi = des_phi
        v = self.reversed_speed_control(phi)
        # print('real phi',(phi*180/pi))
        return v, int(100/(pi/4)*phi)      ##MAY NEED TO ADD MINUS SIGN

    def forward_controller(self, goal):
        xr, yr = self.car_pose.pose.pose.position.x, self.car_pose.pose.pose.position.y
        # self.car_heading = self.car_pose.twist.twist.angular.z
        self.xs.append(xr)
        self.ys.append(yr)
        xo, yo = self.car_pose.pose.pose.orientation.x, self.car_pose.pose.pose.orientation.y
        zo, w = self.car_pose.pose.pose.orientation.z, self.car_pose.pose.pose.orientation.w

        self.current_heading = euler_from_quaternion([xo, yo, zo, w])[2]
        xg, yg = goal[0], goal[1]  # self.path
        L = 0.32
        ld = sqrt((xg - xr) ** 2 + (yg - yr) ** 2)
        des_heading = arctan2((yg - yr), (xg - xr))
        print('des_head', des_heading)
        headErr = des_heading - self.current_heading
        # print("headErrOriginal", headErr)
        if headErr > pi:
            headErr = -2 * pi + headErr
        if headErr < -1 * pi:
            headErr = 2 * pi + headErr
        print('phi', headErr)
        # print('difference_phi',phi*180/pi)
        curv = 2 * sin(headErr) / ld
        des_phi = arctan(L * curv)
        print('des_phi', des_phi)

        if headErr > pi / 2 or des_phi > pi / 4:  # or 100
            phi = pi / 4
        elif headErr < -pi / 2 or des_phi < -pi / 4:  # or -100
            phi = -pi / 4
        else:
            phi = des_phi
        v = self.speed_control(phi)
        # print('real phi',(phi*180/pi))
        return v, -int(100 / (pi / 4) * phi)

    def speed_control(self, phi):
        if self.Estop == 0:
            if abs(phi) < pi/12:
                speed = 20
            else:
                speed = 15
        else:
            speed = -10
        #speed = E_stop(speed)
        return speed

    def reversed_speed_control(self, phi):
        if self.Estop == 0:
            if abs(phi) < pi/12:
                speed = -25
            else:
                speed = -20
        else:
            speed = 0
        #speed = E_stop(speed)
        return speed

    def reach_goal(self, goal):
        xr, yr = self.car_pose.pose.pose.position.x, self.car_pose.pose.pose.position.y
        tol = 0.2
        if dist((xr,yr), goal) <= tol:
            return True
        else:
            return False


    def car_pose_cb(self, car_pose_msg):
        self.car_pose = car_pose_msg

    def choose_point(self):
        xr, yr = self.car_pose.pose.pose.position.x, self.car_pose.pose.pose.position.y
        while(len(self.path) > 1):
            examined_point = self.path[0]
            distance = dist((xr,yr), examined_point)
            if distance > self.ld:
                return examined_point
            self.path.remove(examined_point)
        goal_point = self.path[0]
        self.path.remove(goal_point)
        return goal_point

    
    def reversed_lidar_cb(self,data):
        #msg = lli_ctrl_request()
        #msg.velocity = speed
        #if not hasattr(self, 'car_pose'):
        #return
        #vx, vy = self.car_pose.twist.twist.linear.x, self.car_pose.twist.twist.linear.y
        #self.car_heading = arctan2(vy, vx)
        #if vx**2+vy**2 < 10**(-3):
        #    return
        #beta = self.car_heading - self.current_heading
        angles = arange(data.angle_min, data.angle_max+data.angle_increment, data.angle_increment)
        #print(angles)     
        ranges = data.ranges
        threshold_dist = 0.6
        Estop = 0
        for i in range(len(angles)):
            if abs(angles[i]) < pi/6:
                if ranges[i] < threshold_dist:
                    Estop = 1
        self.Estop = Estop
        #msg.velocity = speed
        #car_speed.publish(msg)

    def forward_lidar_cb(self,data):
        # msg = lli_ctrl_request()
        # msg.velocity = speed
        # if not hasattr(self, 'car_pose'):
        # return
        # vx, vy = self.car_pose.twist.twist.linear.x, self.car_pose.twist.twist.linear.y
        # self.car_heading = arctan2(vy, vx)
        # if vx**2+vy**2 < 10**(-3):
        #    return
        # beta = self.car_heading - self.current_heading
        angles = arange(data.angle_min, data.angle_max + data.angle_increment, data.angle_increment)
        # print(angles)
        ranges = data.ranges
        threshold_dist = 1
        Estop = 0
        for i in range(len(angles)):
            if abs(angles[i]) > pi - pi / 6:
                if ranges[i] < threshold_dist:
                    Estop = 1
        self.Estop = Estop
        # msg.velocity = speed
        # car_speed.publish(msg)

 


def dist(p1, p2):
    return sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)


if __name__ == "__main__":

    rospy.init_node('path_reversed_follow')
    speed = 10
    try:
        PurePursuit()
    except rospy.ROSInterruptException:
        pass
    
