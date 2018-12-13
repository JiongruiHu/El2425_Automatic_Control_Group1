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
        self.going_forwards = False
        self.doing_forward_parking = False
        self.parking_identified = -1
        self.parking_lot_start = [0, 0]
        self.parking_lot_dist = 0
        self.Atan_start = []
        self.current_start_distance = 0.2    # Distance to first obstacle
        self.pp_range = None
        self.pp_angle = None
        self.pp_corner = None
        self.pp_heading = 0
        self.fp_corner = (0, 0)
        self.parked = False
        self.scan = None
        self.pp_path = None
        self.found_path = False

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
        print("I am here 1")
        if self.has_parking_spot:

            self.parallell_parking_backwards()
            self.parallell_parking_forwards()
        rospy.sleep(1.0)
        print("I am here 2")
        self.leave_from_parking("backward_parking")
            # print(self.pp_corner)

    def __pure_pursuit(self):
        #savetxt("/home/nvidia/catkin_ws/planned_path.csv", array(self.path), delimiter = ",")
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
                    if self.doing_forward_parking:
                        self.__check_backwards_done(forwards=True)
                # goal = self.path[0]
        lli_msg.velocity = 0
        self.car_control_pub.publish(lli_msg)
        pose_arr = array([self.xs, self.ys])
        print(self.xs[-1])
        print(self.ys[-1])
        #savetxt("/home/nvidia/catkin_ws/real_path.csv", transpose(pose_arr), delimiter=",")

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
        if hasattr(self, "car_pose"):
            self.parking_stop(data)
            self.scan = data

    def change_to_reversed(self):
        lli_msg = lli_ctrl_request()
        lli_msg.velocity = - 10
        self.car_control_pub.publish(lli_msg)
        rospy.sleep(1.0)
        lli_msg.velocity = 0
        self.car_control_pub.publish(lli_msg)
        rospy.sleep(1.0)
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
                speed = 14
            else:
                speed = 14
        else:
            speed = 0
        if self.going_forwards:
            speed = 13
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
            speed = -21
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
        # print("Pose: ", (self.car_pose.pose.pose.position.x, self.car_pose.pose.pose.position.y))

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
        Atan_parallell_distance = 0.29        # Distance in the car's direction between corner and starting point
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
        #savetxt("/home/nvidia/catkin_ws/obs_without.csv", self.obs_list, delimiter=",")
        self.path = adjustable_path_points("parking", self.Atan_start, heading=self.pp_heading)
        self.pp_path = copy(self.path)
        print("Building path...")
        #self.path = steerings
        self.ld = 0.35
        self.change_to_reversed()
        self.__pure_pursuit()
        self.parked = True

        # self.steer_from_lists(steerings, times)

    def parallell_parking_forwards(self):
        self.going_backwards = False
        self.going_forwards = True
        rospy.sleep(1.0)
        xr, yr, __ = self.__find_current_position()
        outward_distance = -0.15
        parallell_distance = -0.2
        x_goal = self.pp_corner[0] + parallell_distance * cos(self.pp_heading) - outward_distance * sin(self.pp_heading)
        y_goal = self.pp_corner[1] + parallell_distance * sin(self.pp_heading) + outward_distance * cos(self.pp_heading)
        self.path = adjustable_path_points("linear", (xr, yr), goal=(x_goal, y_goal))
        print("Building path...")
        # self.path = steerings
        self.ld = 0.25
        self.change_to_forward()
        self.__pure_pursuit()



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

    def __check_backwards_done(self, forwards = False):
        xr, yr, heading = self.__find_current_position()
        if not forwards:
            enough_backwards = ((self.pp_corner[0] - xr) * cos(self.pp_heading) + (self.pp_corner[1] - yr) * sin(self.pp_heading) > 0.25)
            enough_inwards = (- (self.pp_corner[0] - xr) * sin(self.pp_heading) + (self.pp_corner[1] - yr) * cos(
                self.pp_heading) > 0.06)
            small_heading = (abs(self.pp_heading - heading) < pi/20)
            print(abs(self.pp_heading - heading))
            if enough_backwards and enough_inwards and small_heading:
                print("Early stop")
                self.path = []
            if small_heading:
                print("Small heading")
        else:
            enough_forwards = ((self.fp_corner[0] - xr) * cos(self.pp_heading) + (self.fp_corner[1] - yr) * sin(self.pp_heading) < -0.25)
            enough_inwards = (- (self.fp_corner[0] - xr) * sin(self.pp_heading) + (self.fp_corner[1] - yr) * cos(
                self.pp_heading) > 0.06)
            small_heading = (abs(self.pp_heading - heading) < pi / 20)
            print(abs(self.pp_heading - heading))
            if enough_forwards and enough_inwards and small_heading:
                print("Early stop")
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

    def preemptive_corner_finding(self, ranges, angles):
        i = 0
        filtered_ranges = []
        filtered_angles = []
        for i in range(len(ranges)):
            if ranges[i] < 12:
                filtered_ranges.append(ranges[i])
                filtered_angles.append(angles[i])
        tmpAngles, tmpRanges = [], []
        for i in range(len(filtered_angles)):
            if (0<= angles[i] < pi):
                tmpAngles.append(filtered_angles[i])
                tmpRanges.append(filtered_ranges[i])
        newAngles = tmpAngles[argmin(tmpRanges):]  # newAngles is from the angles where the shortest range starts
        newRanges = tmpRanges[argmin(tmpRanges):]  # newRanges is from the shortest range
        DeltaRanges = [j - i for i, j in zip(newRanges[:-1], newRanges[1:])]  # first derivative of ranges
        first_corner_x, first_corner_y = None, None
        second_corner_x, second_corner_y = None, None
        for i in range(len(DeltaRanges)):
            if DeltaRanges[i] > 0.15:
                print("Angle: ", newAngles[i])
                print("Next angle: ", newAngles[i+1])
                #print("Range: ",newRanges[i])
                #print("MinAngle: ",newAngles[0])
                #print("MinRange: ",newRanges[0])
                #print("DeltaRange",DeltaRanges)
                first_corner_idx = i
                l1 = newRanges[first_corner_idx]
                # in the car frame
                first_corner_x = -l1 * cos(newAngles[first_corner_idx])
                first_corner_y = -l1 * sin(newAngles[first_corner_idx])
                # reverse the DeltaRanges: the first corner should be the first 1st derivative with negative
                # sign in the reversed DeltaRanges? Is it true
                if first_corner_x and not self.found_path:
                    xr, yr, heading = self.__find_current_position()
                    first_corner_x_real = xr + first_corner_x * cos(heading) - first_corner_y * sin(heading)
                    first_corner_y_real = yr + first_corner_x * sin(heading) + first_corner_y * cos(heading)
                    close_point_x = -newRanges[0] * cos(newAngles[0])
                    close_point_y = -newRanges[0] * sin(newAngles[0])
                    close_point_x_real = xr + close_point_x * cos(heading) - close_point_y * sin(heading)
                    close_point_y_real = yr + close_point_x * sin(heading) + close_point_y * cos(heading)
                    vehicle_heading = arctan2(first_corner_y_real - close_point_y_real,
                                              first_corner_x_real - close_point_x_real)
                    path_length = 2
                    outward_distance = 0.3
                    start_point_x = close_point_x_real - outward_distance * sin(vehicle_heading)
                    start_point_y = close_point_y_real + outward_distance * cos(vehicle_heading)
                    goal_point_x = start_point_x + path_length * cos(heading)
                    goal_point_y = start_point_y + path_length * sin(heading)
                    self.path = adjustable_path_points("linear", (start_point_x, start_point_y),
                                                       (goal_point_x, goal_point_y))
                    self.found_path = True
                DeltaRanges.reverse()
                for j in range(len(DeltaRanges)):
                    #print("rev DeltaRange",DeltaRanges[j])
                    if DeltaRanges[j] < -0.01:
                        second_corner_idx = len(DeltaRanges) - 1 - j
                        l2 = newRanges[second_corner_idx]
                        if l2 < 2.5:
                            test_corner_y = -l2 * sin(newAngles[second_corner_idx])
                            if abs(test_corner_y - first_corner_y) <= 0.1:
                                print("Angle: ",newAngles[second_corner_idx])
                                print("Range: ",newRanges[second_corner_idx])
                                print("MinAngle: ",newAngles[0])
                                print("MinRange: ",newRanges[0])
                                second_corner_x = -l2 * cos(newAngles[second_corner_idx])
                                second_corner_y = -l2 * sin(newAngles[second_corner_idx])
                                break
                break

        # calculate the distance between the 1st corner and 2nd corner
        if second_corner_x:
            self.parking_lot_dist = dist((first_corner_x, first_corner_y), (second_corner_x, second_corner_y))
        else:
            return
        #print("distance of parking spot",self.parking_lot_dist)
        if 0.7 <= self.parking_lot_dist < 1.2:
            self.has_parking_spot = True
            self.current_start_distance = -second_corner_y       # May need to add minus sign
            self.parking_identified = 0     # find a small parking spot
            print("Preparing parallel parking, the distance <0.7")
            xr, yr, heading = self.__find_current_position()
            second_corner_x_real = xr + second_corner_x * cos(heading) - second_corner_y * sin(heading)
            second_corner_y_real = yr + second_corner_x * sin(heading) + second_corner_y * cos(heading)
            self.pp_corner = (second_corner_x_real, second_corner_y_real)
            print("Second corner :", (second_corner_x_real, second_corner_y_real))

        elif self.parking_lot_dist >= 1.2: # find large parking spot
            self.has_parking_spot = True
            self.parking_identified = 3     # find large parking spot
            xr, yr, heading = self.__find_current_position()
            first_corner_x_real = xr + first_corner_x * cos(heading) - first_corner_y * sin(heading)
            first_corner_y_real = yr + first_corner_x * sin(heading) + first_corner_y * cos(heading)
            second_corner_x_real = xr + second_corner_x * cos(heading) - second_corner_y * sin(heading)
            second_corner_y_real = yr + second_corner_x * sin(heading) + second_corner_y * cos(heading)
            self.fp_corner = (first_corner_x_real, first_corner_y_real)
            print("Preparing forward parking,dist > 1.2")
            print("First corner local: ",(first_corner_x, first_corner_y))
            print("First corner :", self.fp_corner)
            print("Distance :", self.parking_lot_dist)
            print("Second corner :", (second_corner_x_real, second_corner_y_real))
            self.ld = 0.32
            self.doing_forward_parking = True
            self.forward_parking()
            # just drive in directly

    # if the car back parked, then it should use the same path it parks
    # if the car forward parked, then it should drive backwards as close as possible to
    # the car behind , then follow a path generated by backwards parking function.
    def leave_from_parking(self, howparked):
        #preprocess the scan data, take away the inf datas
        self.doing_forward_parking = False
        self.has_parking_spot = True
        data = self.scan
        filtered_ranges, filtered_angles = [], []
        ranges = data.ranges
        angles = arange(data.angle_min, data.angle_max + data.angle_increment, data.angle_increment)
        for i in range(len(ranges)):
            if ranges[i] < 12:
                filtered_ranges.append(ranges[i])
                filtered_angles.append(angles[i])
        xr, yr, heading = self.__find_current_position(reversed=False)
        idx_minangles = argmin(abs(array(filtered_angles)+(heading - self.pp_heading)))
        backward_dist = filtered_ranges[idx_minangles]
        goal_pos_x = xr - (backward_dist - 0.28) * cos(self.pp_heading)
        goal_pos_y = yr - (backward_dist - 0.28) * sin(self.pp_heading)
        self.path = adjustable_path_points("linear", (xr, yr), goal=(goal_pos_x, goal_pos_y))
        print("I am here 3")
        self.change_to_reversed()
        print("i am stucked")
        self.__pure_pursuit()
        print("I am here 4")
        rospy.sleep(1.0)

        if howparked == "forward_parking":
            self.path = adjustable_path_points("parking",(goal_pos_x, goal_pos_y), heading=self.pp_heading + pi)

        if howparked == "backward_parking":
            print("i am here 5")
            print(self.pp_heading)
            self.path = adjustable_path_points("parking",(goal_pos_x, goal_pos_y), heading=self.pp_heading + pi)
            self.ld = 0.32

        self.change_to_forward()
        self.__pure_pursuit()

    def forward_parking(self):
        outward_distance = 0.3
        parallel_distance = -0.4
        heading = arctan2(self.path[1][1] - self.path[0][1], self.path[1][0] - self.path[0][0])
        start_x = parallel_distance * cos(heading) - outward_distance * sin(heading) + self.fp_corner[0]
        start_y = parallel_distance * sin(heading) + outward_distance * cos(heading) + self.fp_corner[1]
        print("Start position: ", (start_x, start_y))
        self.path = adjustable_path_points("parking_forward", (start_x, start_y), heading=heading)
        self.has_parking_spot = False
        self.pp_heading = heading
        self.parked = True

    def parking_stop(self, data):
        angles = arange(data.angle_min, data.angle_max + data.angle_increment, data.angle_increment)
        ranges = data.ranges
        parking_threshold = 0.7
        pp_len_threshold = 0.7  # Length of gap, subject to change
        for i in range(len(angles)):
            if (angles[i] < pi / 2 + pi / 50) and (angles[i] > pi / 2 - pi / 50):
                if self.parking_identified == -1:
                    self.preemptive_corner_finding(ranges, angles)
                elif self.parking_identified == 0:            # No lot identified
                    if ranges[i] < parking_threshold:
                        # self.current_start_distance = ranges[i] * sin(angles[i])
                        # print("Angle: " + str(angles[i]))
                        # print("Range: " + str(ranges[i]))
                        return
                    elif angles[i+1] > pi / 2 + pi / 50 or angles[i+1] < pi / 2 - pi / 50:    # All relevant angles passed test
                        self.parking_lot_start = [self.car_pose.pose.pose.position.x, self.car_pose.pose.pose.position.y]
                        self.parking_identified = 1
                        print("START:"+str(self.parking_lot_start))
                        # Change lane to be sufficiently close to parked vehicles
                        path_heading = arctan2(self.path[1][1] - self.path[0][1], self.path[1][0] - self.path[0][0])
                        xr,yr,_ = self.__find_current_position()
                        gamma = arctan2(self.pp_corner[1]-yr, self.pp_corner[0]-xr)
                        dl = dist((xr,yr),self.pp_corner)
                        self.current_start_distance = dl*sin(path_heading-gamma)
                        outward_distance_to_move = 0.3 - self.current_start_distance
                        print("Correction: " + str(outward_distance_to_move))
                        self.change_lane(parallell_distance=2.0, outward_distance=outward_distance_to_move)
                elif ranges[i] < parking_threshold and self.parking_identified == 1:    # Start but not end of lot identified
                    parking_lot_end = [self.car_pose.pose.pose.position.x, self.car_pose.pose.pose.position.y]
                    self.parking_lot_dist = dist(parking_lot_end, self.parking_lot_start)
                    print("Dist"+str(self.parking_lot_dist))
                    if self.parking_lot_dist > pp_len_threshold:
                        self.has_parking_spot = True
                    if self.has_parking_spot:
                        self.parking_identified = 2             # parking_stop will detect no more lots
                        self.generate_obs_list(angles, ranges)
                        self.pp_range = ranges[i]
                        self.pp_angle = angles[i]
                        self.__set_pp_corner(data)
                        print("corner: " + str(self.pp_corner))
                        print("car: " + str([self.car_pose.pose.pose.position.x, self.car_pose.pose.pose.position.y]))
                        self.parallell_parking_start()
                        # self.parallell_parking_start(angles[i], ranges[i])
                    elif self.parking_lot_dist > 0.03:
                        self.parking_identified = 0
                # print("Angle: "+str(angles[i]))
                # print("Range: "+str(ranges[i]))


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
    
