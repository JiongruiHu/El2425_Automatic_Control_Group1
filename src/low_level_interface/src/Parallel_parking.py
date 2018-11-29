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
        self.path = adjustable_path_points('parking', (1.5, 1.5), (0, 0), heading = pi/2)
        self.Estop = 0
        self.car_heading = 0
        # Subscribe to the topics
        self.car_pose_sub = rospy.Subscriber("SVEA1/odom", Odometry, self.car_pose_cb)
        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_cb)
        self.reversed = False

        self.has_parking_spot = False
        self.preparing_to_park = False
        self.going_backwards = False
        self.parking_identified = 0
        self.parking_lot_start = [0, 0]
        self.parking_lot_dist = 0
        self.Atan_start = []
        self.current_start_distance = 0.2    # Distance to first obstacle
        self.pp_range = None
        self.pp_angle = None
        self.pp_coner = None
        self.pp_heading = 0

        rospy.loginfo(self.car_pose_sub)
        # init Publisher
        self.car_control_pub = rospy.Publisher("lli/ctrl_request", lli_ctrl_request, queue_size=10)

        # goal = self.path[0]

        self.ld = 0.32
        self.xs = []
        self.ys = []
        self.__follow_then_park()

    def __forward_then_backward(self):
        self.change_to_forward()
        self.path = path_points('linear')
        self.__pure_pursuit()
        self.change_to_reversed()
        self.path = path_points('reversed_linear')
        self.__pure_pursuit()

    def __backward_then_forward(self):
        self.change_to_reversed()
        self.path = path_points('small_circle')
        self.ld = 0.25
        self.has_parking_spot = True
        self.__pure_pursuit()
        return
        self.change_to_forward()
        self.path = path_points('linear')
        self.__pure_pursuit()

    def __follow_then_park(self):
        self.change_to_forward()
        self.path = path_points('linear')
        self.__pure_pursuit()
        if self.has_parking_spot:

            self.parallell_parking_backwards()
            print(self.pp_corner)

    def __pure_pursuit(self):
        savetxt("/home/nvidia/catkin_ws/planned_path.csv", array(self.path), delimiter = ",")
        rate = rospy.Rate(50)
        lli_msg = lli_ctrl_request()
        while len(self.path) > 0:
            if hasattr(self, 'car_pose'):
                while not (rospy.is_shutdown() or len(self.path) == 0):
                    goal = self.choose_point()
                    lli_msg.velocity, lli_msg.steering = self.controller(goal)

                    # if not self.has_parking_spot:
                    self.car_control_pub.publish(lli_msg)
                    rate.sleep()
                    # else:
                        # return
                    if self.going_backwards:
                        self.__check_backwards_done()
                # goal = self.path[0]
        lli_msg.velocity = 0
        self.car_control_pub.publish(lli_msg)
        pose_arr = array([self.xs, self.ys])
        print(self.xs[-1])
        print(self.ys[-1])
        savetxt("/home/nvidia/catkin_ws/real_path.csv", transpose(pose_arr), delimiter=",")

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
        lli_msg.velocity = - 10
        self.car_control_pub.publish(lli_msg)
        rospy.sleep(0.1)
        lli_msg.velocity = 0
        self.car_control_pub.publish(lli_msg)
        rospy.sleep(0.1)
        lli_msg.velocity = - 10
        self.car_control_pub.publish(lli_msg)
        rospy.sleep(0.1)
        self.reversed = True

    def change_to_forward(self):
        self.reversed = False

    def reversed_controller(self, goal):
        xr, yr, self.current_heading = self.__find_current_position(True)
        self.xs.append(xr)
        self.ys.append(yr)
        xg, yg = goal[0],goal[1]  # self.path
        L = 0.32
        lf = 0.17
        ld = dist((xr, yr), (xg, yg))
        des_heading = arctan2((yg - yr), (xg - xr))
        # print('des_head', des_heading)
        head_err = des_heading - self.current_heading
        # print("headErrOriginal", headErr)
        if head_err > pi:
            head_err = -2 * pi + head_err
        if head_err < -1 * pi:
            head_err = 2 * pi + head_err
        # print('phi', head_err)
        # print('difference_phi', phi*180/pi)
        curv = 2 * sin(head_err) / ld
        des_phi = arctan(L * curv)
        # des_phi = arctan(L / lf * tan(arcsin(lf * curv)))
        # print('des_phi', des_phi)

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
        xr, yr, self.current_heading = self.__find_current_position()
        # self.car_heading = self.car_pose.twist.twist.angular.z
        self.xs.append(xr)
        self.ys.append(yr)

        xg, yg = goal[0], goal[1]  # self.path
        L = 0.32
        ld = sqrt((xg - xr) ** 2 + (yg - yr) ** 2)
        des_heading = arctan2((yg - yr), (xg - xr))
        # print('des_head', des_heading)
        head_err = des_heading - self.current_heading
        # print("headErrOriginal", headErr)
        if head_err > pi:
            head_err = -2 * pi + head_err
        if head_err < -1 * pi:
            head_err = 2 * pi + head_err
        # print('phi', head_err)
        # print('difference_phi',phi*180/pi)
        curv = 2 * sin(head_err) / ld
        des_phi = arctan(L * curv)
        # print('des_phi', des_phi)

        if head_err > pi / 2 or des_phi > pi / 4:  # or 100
            phi = pi / 4
        elif head_err < -pi / 2 or des_phi < -pi / 4:  # or -100
            phi = -pi / 4
        else:
            phi = des_phi
        v = self.speed_control(phi)
        # print('real phi',(phi*180/pi))
        return v, -int(100 / (pi / 4) * phi)

    def __find_current_position(self, reversed = False):
        assert hasattr(self, "car_pose")
        dist_diff = 0.05
        xo, yo = self.car_pose.pose.pose.orientation.x, self.car_pose.pose.pose.orientation.y
        zo, w = self.car_pose.pose.pose.orientation.z, self.car_pose.pose.pose.orientation.w
        heading = euler_from_quaternion([xo, yo, zo, w])[2]
        xp, yp = self.car_pose.pose.pose.position.x, self.car_pose.pose.pose.position.y
        xr, yr = xp + dist_diff * cos(heading), yp + dist_diff * sin(heading)
        if reversed:
            heading = heading - pi if heading > 0 else heading + pi
        return xr, yr, heading

    def speed_control(self, phi):
        if self.Estop == 0:
            if abs(phi) < pi / 12:
                speed = 10
            else:
                speed = 10
        else:
            speed = 0
        # if self.has_parking_spot:
        # speed = -10
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

        if self.has_parking_spot:
            speed = -19
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
        xr, yr,  __= self.__find_current_position()
        while len(self.path) > 1:
            examined_point = self.path[0]
            distance = dist((xr,yr), examined_point)
            if distance > self.ld:
                return examined_point
            self.path.remove(examined_point)
        goal_point = self.path[0]
        if dist((xr, yr), goal_point) < 0.15:
            self.path.remove(goal_point)
        return goal_point

    def parallell_parking_start(self):
        #Car positioning
        self.pp_heading = self.current_heading
        parallell_distance = 0.5        # Distance in the car's direction between corner and starting point
        outward_distance = 0.3      # Same, but to the left
        xg = self.pp_corner[0] + (parallell_distance * cos(self.current_heading)) - (outward_distance*sin(self.current_heading))
        yg = self.pp_corner[1] + (parallell_distance * sin(self.current_heading)) + (outward_distance*cos(self.current_heading))
        print("GoalPos: "+ str((xg,yg)))
        xr, yr, __ = self.__find_current_position()
        print("CarPos: ", str((xr,yr)))
        start_path = adjustable_path_points("linear", (xg, yg), (xg, yg))
        self.path = start_path
        #Arctan start point
        Atan_parallell_distance = 0.3        # Distance in the car's direction between corner and starting point
        Atan_outward_distance = 0.4      # Same, but to the left
        Atan_x = self.pp_corner[0] + (Atan_parallell_distance * cos(self.current_heading)) - (Atan_outward_distance*sin(self.current_heading))
        Atan_y = self.pp_corner[1] + (Atan_parallell_distance * sin(self.current_heading)) + (Atan_outward_distance*cos(self.current_heading))
        self.Atan_start = [Atan_x, Atan_y]
        self.preparing_to_park = True

        
        

        # head = self.current_heading
        # parallell_start = xr * cos(head) + yr * sin(head)
        # parallell_position = parallell_start
        # lli_msg = lli_ctrl_request()
        # lli_msg.velocity = 12
        # while parallell_position < parallell_start + parallell_distance:
        #     self.car_control_pub.publish(lli_msg)
        #     rospy.sleep(0.1)
        #     xr, yr = self.car_pose.pose.pose.position.x, self.car_pose.pose.pose.position.y
        #     parallell_position = xr * cos(head) + yr * sin(head)



    def parallell_parking_backwards(self):
        self.preparing_to_park = False
        self.going_backwards = True
        rospy.sleep(1.0)
        xr, yr = self.car_pose.pose.pose.position.x, self.car_pose.pose.pose.position.y
        print("Planning path...")
        savetxt("/home/nvidia/catkin_ws/obs_without.csv", self.obs_list, delimiter=",")
        self.path = adjustable_path_points("parking", self.Atan_start, heading = self.pp_heading)
        print("Building path...")
        #self.path = steerings
        self.ld = 0.32
        self.change_to_reversed()
        self.__pure_pursuit()

        # self.steer_from_lists(steerings, times)

    def steer_from_lists(self, steerings, times):
        self.change_to_reversed()
        start = time.time()
        lli_msg = lli_ctrl_request()
        lli_msg.velocity = -17
        time_elapsed = 0
        i = 0
        while time_elapsed < times[-1]:
            while times[i] < time_elapsed:
                i += 1
            lli_msg.steering = -int(100 * steerings[i]/(pi/4))
            self.car_control_pub.publish(lli_msg)
            rospy.sleep(0.05)
            time_elapsed = time.time() - start

    def __check_backwards_done(self):
        xr, yr, heading = self.__find_current_position()
        enough_backwards = ((self.pp_corner[0] - xr) * cos(self.pp_heading) + (self.pp_corner[1] - yr) * sin(self.pp_heading) > 0.25)
        enough_inwards = (- (self.pp_corner[0] - xr) * sin(self.pp_heading) + (self.pp_corner[1] - yr) * cos(
            self.pp_heading) > 0.06)
        small_heading = (abs(self.pp_heading - heading) < pi/20)
        if enough_backwards and enough_inwards and small_heading:
            self.path = []

    # Changes a linear path to another in a certain outward_distance away
    def change_lane(self, parallell_distance, outward_distance):
        initial_p_dist = 0.2
        xr, yr, __ = self.__find_current_position()
        heading = arctan2(self.path[1][1]-self.path[0][1], self.path[1][0] - self.path[0][0])
        x_init = initial_p_dist * cos(heading) - outward_distance * sin(heading)
        y_init = initial_p_dist * sin(heading) + outward_distance * cos(heading)
        x_distance = parallell_distance * cos(heading) - outward_distance * sin(heading)
        y_distance = parallell_distance * sin(heading) + outward_distance * cos(heading)
        x_start, y_start = xr + x_init, yr + y_init
        x_goal, y_goal = xr + x_distance, yr + y_distance
        self.path = adjustable_path_points("linear", (x_start, y_start), (x_goal, y_goal))

    def parking_stop(self, data):
        angles = arange(data.angle_min, data.angle_max + data.angle_increment, data.angle_increment)
        ranges = data.ranges
        parking_threshold = 0.5
        pp_len_threshold = 0.7          # Length of gap, subject to change
        for i in range(len(angles)):
            if (angles[i] < pi / 2 + pi / 50) and (angles[i] > pi / 2 - pi / 50):
                if self.parking_identified == 0:            # No lot identified
                    if ranges[i] < parking_threshold:
                        self.current_start_distance = ranges[i] * sin(angles[i])
                        print("Angle: " + str(angles[i]))
                        print("Range: " + str(ranges[i]))
                        return
                    elif angles[i+1] > pi / 2 + pi / 50 or angles[i+1] < pi / 2 - pi / 50:    # All relevant angles passed test
                        self.parking_lot_start = [self.car_pose.pose.pose.position.x, self.car_pose.pose.pose.position.y]
                        self.parking_identified = 1
                        print("START:"+str(self.parking_lot_start))
                        # Change lane to be sufficiently close to parked vehicles
                        outward_distance_to_move = 0.3 - self.current_start_distance
                        print("Correction: " + str(outward_distance_to_move))
                        self.change_lane(parallell_distance=2.0, outward_distance=outward_distance_to_move)
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
                        self.__set_pp_corner(data)
                        print("corner: " + str(self.pp_corner))
                        print("car: " + str([self.car_pose.pose.pose.position.x, self.car_pose.pose.pose.position.y]))
                        self.parallell_parking_start()
                        # self.parallell_parking_start(angles[i], ranges[i])
                    else:
                        self.parking_identified = 0
                print("Angle: "+str(angles[i]))
                print("Range: "+str(ranges[i]))

     # Decides the corner of the front obstacle as the closest obstacle to the right of the vehicle
    # Used in parking_stop(data)
    def __set_pp_corner(self, data):
        xr, yr, self.pp_heading = self.__find_current_position()
        angles = arange(data.angle_min, data.angle_max + data.angle_increment, data.angle_increment)
        ranges = data.ranges
        min_range = 12
        corner_angle = 0
        for i in range(len(angles)):
            if (angles[i] < pi/2 + pi/4) and (angles[i] > pi/2 - pi/4):
                if ranges[i] < min_range:
                    min_range = ranges[i]
                    corner_angle = angles[i]
        if corner_angle > 0:
            parallell_corner_distance = -min_range * cos(corner_angle)
            outward_corner_distance = -min_range * sin(corner_angle)
            x_corner_pos = xr + parallell_corner_distance * cos(self.pp_heading) - outward_corner_distance * sin(self.pp_heading)
            y_corner_pos = yr + parallell_corner_distance * sin(self.pp_heading) + outward_corner_distance * cos(self.pp_heading)
            self.pp_corner = (x_corner_pos, y_corner_pos)
            print("Car's position: "+str((xr,yr)))
            print("Corner position: "+str((x_corner_pos,y_corner_pos)))
            print("Distance: "+str(min_range))
        else:                       # No corner detected!
            print("Corner detection error!")


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
    
