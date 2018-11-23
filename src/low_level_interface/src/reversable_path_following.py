#!/usr/bin/env python
import rospy
import time
from tf.transformations import euler_from_quaternion
from numpy import *
import matplotlib as mp
from sensor_msgs.msg import LaserScan
from path_points import path_points, adjustable_path_points
from low_level_interface.msg import lli_ctrl_request
from nav_msgs.msg import Odometry
from path_planning import Path


# Creates a follow then park implementation in SVEA1 from MOCAP
class FollowThenPark(object):
    def __init__(self):
        self.path = path_points('figure-8')
        self.Estop = 0
        self.car_heading = 0
        # Subscribe to the topics
        self.car_pose_sub = rospy.Subscriber("SVEA1/odom", Odometry, self.car_pose_cb)
        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_cb)
        self.reversed = False

        self.has_parking_spot = False
        self.parking_identified = 0
        self.parking_lot_start = [0, 0]
        self.parking_lot_dist = 0
        self.pp_goal = [0, 0]
        self.obs_list = []
        self.pp_range = None
        self.pp_angle = None

        rospy.loginfo(self.car_pose_sub)
        # init Publisher
        self.car_control_pub = rospy.Publisher("lli/ctrl_request", lli_ctrl_request, queue_size=10)

        # goal = self.path[0]

        self.ld = 0.5
        self.xs = []
        self.ys = []
        self.__follow_then_park()

    def __backward_then_forward(self):
        self.change_to_reversed()
        self.path = path_points('reversed_circle')
        self.__pure_pursuit()
        self.change_to_forward()
        self.path = path_points('circle')
        self.__pure_pursuit()

    def __follow_then_park(self):
        self.change_to_forward()
        self.path = path_points('linear')
        self.__pure_pursuit()
        if self.has_parking_spot:
            self.parallell_parking_start(self.pp_angle, self.pp_range)
            self.parallell_parking_backwards()

    def __pure_pursuit(self):
        rate = rospy.Rate(50)
        lli_msg = lli_ctrl_request()
        while len(self.path) > 0:
            if hasattr(self, 'car_pose'):
                while not (rospy.is_shutdown() or len(self.path) == 0):
                    goal = self.choose_point()
                    lli_msg.velocity, lli_msg.steering = self.controller(goal)
                    if not self.has_parking_spot:
                        self.car_control_pub.publish(lli_msg)
                        rate.sleep()
                    else:
                        return
                # goal = self.path[0]
        lli_msg.velocity = 0
        self.car_control_pub.publish(lli_msg)
        pose_arr = array([self.xs, self.ys])
        savetxt("/home/nvidia/catkin_ws/real_path.csv", pose_arr, delimiter=",")

    # Both following functions choose between reversed and forward depending on self.reversed
    def controller(self, goal):
        if self.reversed:
            return self.reversed_controller(goal)
        else:
            return self.forward_controller(goal)

    def lidar_cb(self,data):
        if self.reversed:
            pass
            # self.reversed_lidar_cb(data)
        else:
            pass
            # self.forward_lidar_cb(data)
        self.parking_stop(data)

    def change_to_reversed(self):
        lli_msg = lli_ctrl_request()
        lli_msg.velocity = -10
        self.car_control_pub.publish(lli_msg)
        lli_msg.velocity = 0
        self.car_control_pub.publish(lli_msg)
        self.reversed = True

    def change_to_forward(self):
        self.reversed = False

    def reversed_controller(self, goal):
        xr, yr = self.car_pose.pose.pose.position.x, self.car_pose.pose.pose.position.y
        # self.car_heading = self.car_pose.twist.twist.angular.z
        self.xs.append(xr)
        self.ys.append(yr)
        xo, yo = self.car_pose.pose.pose.orientation.x, self.car_pose.pose.pose.orientation.y
        zo, w = self.car_pose.pose.pose.orientation.z, self.car_pose.pose.pose.orientation.w

        recieved_heading = euler_from_quaternion([xo, yo, zo, w])[2]
        self.current_heading =  recieved_heading - pi if recieved_heading > 0 else recieved_heading + pi        # Ensures that the heading is 180 away from the recorded
        xg, yg = goal[0],goal[1]  # self.path
        L = 0.32
        ld = sqrt((xg - xr)**2 + (yg - yr)**2)
        des_heading = arctan2((yg - yr), (xg - xr))
        print('des_head', des_heading)
        head_err = des_heading - self.current_heading
        # print("headErrOriginal", headErr)
        if head_err > pi:
            head_err = -2 * pi + head_err
        if head_err < -1 * pi:
            head_err = 2 * pi + head_err
        print('phi', head_err)
        # print('difference_phi', phi*180/pi)
        curv = 2 * sin(head_err) / ld
        des_phi = arctan(L * curv)
        print('des_phi', des_phi)

        if head_err > pi/2 or des_phi > pi/4:  # or 100
            phi = pi/4
        elif head_err < -pi/2 or des_phi < -pi/4:  # or -100
            phi = -pi/4
        else:
            phi = des_phi
        v = self.reversed_speed_control(phi)
        # print('real phi',(phi*180/pi))
        return v, int(100/(pi/4)*phi)      # MAY NEED TO ADD MINUS SIGN

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
        head_err = des_heading - self.current_heading
        # print("headErrOriginal", headErr)
        if head_err > pi:
            head_err = -2 * pi + head_err
        if head_err < -1 * pi:
            head_err = 2 * pi + head_err
        print('phi', head_err)
        # print('difference_phi',phi*180/pi)
        curv = 2 * sin(head_err) / ld
        des_phi = arctan(L * curv)
        print('des_phi', des_phi)

        if head_err > pi / 2 or des_phi > pi / 4:  # or 100
            phi = pi / 4
        elif head_err < -pi / 2 or des_phi < -pi / 4:  # or -100
            phi = -pi / 4
        else:
            phi = des_phi
        v = self.speed_control(phi)
        # print('real phi',(phi*180/pi))
        return v, -int(100 / (pi / 4) * phi)

    def speed_control(self, phi):
        if self.Estop == 0:
            if abs(phi) < pi/12:
                speed = 12
            else:
                speed = 12
        else:
            speed = 0
        # speed = E_stop(speed)
        return speed

    def reversed_speed_control(self, phi):
        if self.Estop == 0:
            if abs(phi) < pi/12:
                speed = -25
            else:
                speed = -20
        else:
            speed = 0
        # speed = E_stop(speed)
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
        if dist((xr, yr), goal_point) < 0.05:
            self.path.remove(goal_point)
        return goal_point

    def parallell_parking_start(self, angle, range):
        parallell_distance = 0.25        # Distance in the car's direction between corner and starting point
        # outward_distance = 0.3      # Same, but to the left
        # parallell_distance_to_travel = parallell_distance - cos(angle) * range
        # outward_distance_to_travel = outward_distance - sin(angle) * range
        # # Rotation into global frame
        # x_distance_to_travel = cos(self.current_heading) * parallell_distance_to_travel - \
        #                        sin(self.current_heading) * outward_distance_to_travel
        # y_distance_to_travel = sin(self.current_heading) * parallell_distance_to_travel + \
        #                        cos(self.current_heading) * outward_distance_to_travel
        xr, yr = self.car_pose.pose.pose.position.x, self.car_pose.pose.pose.position.y
        # xg, yg = xr + x_distance_to_travel, yr + y_distance_to_travel
        # start_path = adjustable_path_points("linear", (xr, yr), (xg, yg))
        # self.path = start_path

        parallell_distance_to_goal = -0.45 - cos(angle) * range  # Heavily subject to change
        outward_distance_to_goal = -0.08 - sin(angle) * range
        x_distance_to_goal = cos(self.current_heading) * parallell_distance_to_goal - \
                             sin(self.current_heading) * outward_distance_to_goal
        y_distance_to_goal = sin(self.current_heading) * parallell_distance_to_goal + \
                             cos(self.current_heading) * outward_distance_to_goal
        xp, yp = xr + x_distance_to_goal, yr + y_distance_to_goal
        self.pp_goal = (xp, yp)

        head = self.current_heading
        parallell_start = xr * cos(head) + yr * sin(head)
        parallell_position = parallell_start
        lli_msg = lli_ctrl_request()
        lli_msg.velocity = 12
        while parallell_position < parallell_start + parallell_distance:
            self.car_control_pub.publish(lli_msg)
            rospy.sleep(0.1)
            xr, yr = self.car_pose.pose.pose.position.x, self.car_pose.pose.pose.position.y
            parallell_position = xr * cos(head) + yr * sin(head)



    def parallell_parking_backwards(self):
        xr, yr = self.car_pose.pose.pose.position.x, self.car_pose.pose.pose.position.y
        print("Planning path...")
        parking_path = Path((xr, yr), self.pp_goal, self.obs_list, self.current_heading)
        print("Building path...")
        steerings, times = parking_path.build_path()
        self.change_to_reversed()
        # self.__pure_pursuit()

        self.steer_from_lists(steerings, times)

    def steer_from_lists(self, steerings, times):
        start = time.time()
        lli_msg = lli_ctrl_request()
        lli_msg.velocity = -17
        time_elapsed = 0
        i = 0
        while time_elapsed < times[-1]:
            while times[i] < time_elapsed:
                i += 1
            lli_msg.steering = int(100*steerings[i]/(pi/4))
            self.car_control_pub.publish(lli_msg)
            rospy.sleep(0.05)
            time_elapsed = time.time() - start

    def parking_stop(self, data):
        angles = arange(data.angle_min, data.angle_max + data.angle_increment, data.angle_increment)
        ranges = data.ranges
        parking_threshold = 0.5
        pp_len_threshold = 0.7          # Length of gap, subject to change
        for i in range(len(angles)):
            if (angles[i] < pi / 2 + pi / 50) and (angles[i] > pi / 2 - pi / 50):
                if self.parking_identified == 0:            # No lot identified
                    if ranges[i] < parking_threshold:
                        return
                    elif angles[i+1] > pi / 2 + pi / 50 or angles[i+1] < pi / 2 - pi / 50:    # All relevant angles passed test
                        self.parking_lot_start = [self.car_pose.pose.pose.position.x, self.car_pose.pose.pose.position.y]
                        self.parking_identified = 1
                        print("START:"+str(self.parking_lot_start))
                elif ranges[i] < parking_threshold and self.parking_identified == 1:    # Start but not end of lot identified
                    parking_lot_end = [self.car_pose.pose.pose.position.x, self.car_pose.pose.pose.position.y]
                    self.parking_lot_dist = dist(parking_lot_end, self.parking_lot_start)
                    print("Dist"+str(self.parking_lot_dist))
                    if self.parking_lot_dist > pp_len_threshold:
                        self.has_parking_spot = True
                        self.parking_identified = 2             # parking_stop will detect no more lots
                        self.generate_obs_list(angles, ranges)
                        self.pp_range = ranges[i]
                        self.pp_angle = angles[i]
                        # self.parallell_parking_start(angles[i], ranges[i])
                    else:
                        self.parking_identified = 0

    # Uses MOCAP to transform obstacles from polar local coordinates to
    # cartesian global coordinates
    def generate_obs_list(self, angles, ranges):
        xr, yr = self.car_pose.pose.pose.position.x, self.car_pose.pose.pose.position.y
        head = self.current_heading
        obs_list = []
        for i in range(len(ranges)):
            if ranges[i] < 12:           # Maximum range of LIDAR
                parallell_distance = -cos(angles[i]) * ranges[i]
                outward_distance = -sin(angles[i]) * ranges[i]
                x_distance = cos(head) * parallell_distance - sin(head) * outward_distance
                y_distance = sin(head) * parallell_distance + cos(head) * outward_distance
                obs_list.append([xr + x_distance, yr + y_distance])
        self.obs_list = obs_list
    
    def reversed_lidar_cb(self,data):
        # msg = lli_ctrl_request()
        # msg.velocity = speed
        # if not hasattr(self, 'car_pose'):
        # return
        # vx, vy = self.car_pose.twist.twist.linear.x, self.car_pose.twist.twist.linear.y
        # self.car_heading = arctan2(vy, vx)
        # if vx**2+vy**2 < 10**(-3):
        #    return
        # beta = self.car_heading - self.current_heading
        angles = arange(data.angle_min, data.angle_max+data.angle_increment, data.angle_increment)
        # print(angles)
        ranges = data.ranges
        threshold_dist = 0.6
        Estop = 0
        for i in range(len(angles)):
            if abs(angles[i]) < pi/6:
                if ranges[i] < threshold_dist:
                    Estop = 1
        self.Estop = Estop
        # msg.velocity = speed
        # car_speed.publish(msg)

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
    try:
        FollowThenPark()
    except rospy.ROSInterruptException:
        pass
    
