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


# Creates a follow then park implementation in SVEA1 from MOCAP. Keeps all, otherwise global, variables as fields in an object.
# This object can be created by the script in demo_part_2_B.
class FollowThenPark(object):
    def __init__(self):
        self.path = adjustable_path_points('parking', (1.5, 1.5), (0, 0), heading = pi/2)
        self.Estop = 0
        self.car_heading = 0
        # Subscribe to the MOCAP and lidar topics
        self.car_pose_sub = rospy.Subscriber("SVEA1/odom", Odometry, self.car_pose_cb)
        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_cb)
        self.reversed = False
        
        # States of parking
        self.has_parking_spot = False
        self.preparing_to_park = False
        self.going_backwards = False
        self.going_forwards = False
        self.doing_forward_parking = False
        self.parking_identified = -1
        self.parked = False
        self.scan = None
        self.pp_path = None
        self.found_path = False
        self.tried = False
        # Parameters of parker
        self.parking_lot_start = [0, 0]
        self.parking_lot_dist = 0
        self.Atan_start = []
        self.current_start_distance = 0.2    # Distance to first obstacle
        self.pp_range = None
        self.pp_angle = None
        self.pp_corner = None
        self.pp_heading = 0
        self.fp_corner = (0, 0)
      

        rospy.loginfo(self.car_pose_sub)
        # init Publisher to hardware
        self.car_control_pub = rospy.Publisher("lli/ctrl_request", lli_ctrl_request, queue_size=10)

        self.ld = 0.32      # Lookahead distance
        self.xs = []
        self.ys = []
        self.__follow_then_park()

    ## Goes forward along a line before turning around. Used for testing purposes.
    def __forward_then_backward(self):
        self.change_to_forward()
        self.path = path_points('linear')
        self.__pure_pursuit()
        self.change_to_reversed()
        self.path = path_points('reversed_linear')
        self.__pure_pursuit()

    ## As above, but the other way around. Used for testing purposes.
    def __backward_then_forward(self):
        self.change_to_reversed()
        self.path = path_points('reversed_linear')
        self.has_parking_spot = True
        self.__pure_pursuit()
        self.change_to_forward()
        self.path = path_points('linear')
        self.__pure_pursuit()
    
    ## Main parallel parking function. Follows a linear path (which may be changed) before parking
    ## either forward or in parallel. In this version, will then leave the parking spot.
    def __follow_then_park(self):
        self.change_to_forward()
        self.path = path_points('linear')
        self.__pure_pursuit()
        if self.has_parking_spot:
            self.parallell_parking_backwards()
            self.parallell_parking_forwards()

        if not self.tried:
            rospy.sleep(1.0)
            mode = "forward_parking" if self.parked else "backward_parking"
            self.leave_from_parking(mode)
    
    ## Top level pure pursuit algorithm. Finds a goal point, feeds it into the controller and publishes the result to the car.
    def __pure_pursuit(self):
        rate = rospy.Rate(50)
        lli_msg = lli_ctrl_request()
        while len(self.path) > 0:
            if hasattr(self, 'car_pose'):
                while not (rospy.is_shutdown() or len(self.path) == 0):
                    goal = self.choose_point()
                    lli_msg.velocity, lli_msg.steering = self.controller(goal)

                    self.car_control_pub.publish(lli_msg)
                    rate.sleep()
                    if self.going_backwards:
                        self.__check_backwards_done()
                    if self.doing_forward_parking:
                        self.__check_backwards_done(forwards=True)
        lli_msg.velocity = 0
        self.car_control_pub.publish(lli_msg)
        pose_arr = array([self.xs, self.ys])
        #print(self.xs[-1])
        #print(self.ys[-1])
        #savetxt("/home/nvidia/catkin_ws/real_path.csv", transpose(pose_arr), delimiter=",")

    ## Both following functions choose between reversed and forward depending on self.reversed
    def controller(self, goal):
        if self.reversed:
            return self.reversed_controller(goal)
        else:
            return self.forward_controller(goal)

    ## Callback function to lidar. Uses data to call a parking function.
    ## Does not currently utilize E-stop functionality.
    def lidar_cb(self,data):
        if self.reversed:
            pass
            # self.reversed_lidar_cb(data)
        else:
            pass
            # self.forward_lidar_cb(data)
        if hasattr(self, "car_pose") and not self.tried:
            self.parking_stop(data)
            self.scan = data
            
    ## Compensates for eccentricies in hardware in order to change from forward to backward velocity.
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

    ## As above, but other way around. Included for completeness sake.
    def change_to_forward(self):
        self.reversed = False

    ## Low level pure pursuit, including angle and speed compensation for travelling backwards. 
    def reversed_controller(self, goal):
        xr, yr, self.current_heading = self.__find_current_position(True)
        self.xs.append(xr)
        self.ys.append(yr)
        xg, yg = goal[0],goal[1]  # Goal coordinates
        L = 0.32    # Distance between wheel axes.
        ld = dist((xr, yr), (xg, yg))
        des_heading = arctan2((yg - yr), (xg - xr))
        head_err = des_heading - self.current_heading
        
        if head_err > pi:
            head_err = -2 * pi + head_err
        if head_err < -1 * pi:
            head_err = 2 * pi + head_err
        curv = 2 * sin(head_err) / ld
        des_phi = arctan(L * curv)
        
        if head_err > pi/2 or des_phi > pi/4:  # or 100
            phi = pi/4
        elif head_err < -pi/2 or des_phi < -pi/4:  # or -100
            phi = -pi/4
        else:
            phi = des_phi
        v = self.reversed_speed_control(phi)
        return v, int(100/(pi/4)*phi)      # MAY NEED TO ADD MINUS SIGN

    ## Works like controller in demo_part_1
    def forward_controller(self, goal):
        xr, yr, self.current_heading = self.__find_current_position()
        self.xs.append(xr)
        self.ys.append(yr)

        xg, yg = goal[0], goal[1]  # self.path
        L = 0.32
        ld = sqrt((xg - xr) ** 2 + (yg - yr) ** 2)
        des_heading = arctan2((yg - yr), (xg - xr))
        head_err = des_heading - self.current_heading
        
        if head_err > pi:
            head_err = -2 * pi + head_err
        if head_err < -1 * pi:
            head_err = 2 * pi + head_err
        curv = 2 * sin(head_err) / ld
        des_phi = arctan(L * curv)

        if head_err > pi / 2 or des_phi > pi / 4:  # or 100
            phi = pi / 4
        elif head_err < -pi / 2 or des_phi < -pi / 4:  # or -100
            phi = -pi / 4
        else:
            phi = des_phi
        v = self.speed_control(phi)
        return v, -int(100 / (pi / 4) * phi)

    ## Gives position and orientation of car from MOCAP. Compensates for offset or reverse heading
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

    ## Rudimentary low level speed control for going forwards
    def speed_control(self, phi):
        if self.Estop == 0:
            if abs(phi) < pi / 12:
                speed = 12
            else:
                speed = 12
        else:
            speed = 0
        if self.going_forwards:
            speed = 12
        return speed

    ## As above, but for going backwards.
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
        return speed

    ## Callback function to MOCAP subscriber. 
    def car_pose_cb(self, car_pose_msg):
        self.car_pose = car_pose_msg

    ## Lookahead distance part of pure pursuit algorithm. Chooses a point on the path based on current position and lookahead distance
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

    ## Finds the preferred position and start point of backwards path based on a recently measured corner, and goes there.
    def parallell_parking_start(self):
        #Car positioning
        self.pp_heading = self.current_heading
        parallell_distance = 0.5        # Distance in the car's direction between corner and starting point
        outward_distance = 0.3      # Same, but to the left
        xg = self.pp_corner[0] + (parallell_distance * cos(self.current_heading)) - (outward_distance*sin(self.current_heading))
        yg = self.pp_corner[1] + (parallell_distance * sin(self.current_heading)) + (outward_distance*cos(self.current_heading))
        xr, yr, __ = self.__find_current_position()
        start_path = adjustable_path_points("linear", (xg, yg), (xg, yg))
        self.path = start_path      # Causes the pure pursuit algorithm to readjust to the new path
        #Arctan start point
        Atan_parallell_distance = 0.29        # Distance in the car's direction between corner and starting point
        Atan_outward_distance = 0.4      # Same, but to the left
        Atan_x = self.pp_corner[0] + (Atan_parallell_distance * cos(self.current_heading)) - (Atan_outward_distance*sin(self.current_heading))
        Atan_y = self.pp_corner[1] + (Atan_parallell_distance * sin(self.current_heading)) + (Atan_outward_distance*cos(self.current_heading))
        self.Atan_start = [Atan_x, Atan_y]
        self.preparing_to_park = True

    ## Creates a parallel parking path, prepares, and goes backwards
    def parallell_parking_backwards(self):
        # Readjust states
        self.preparing_to_park = False
        self.going_backwards = True
        rospy.sleep(1.0)
        # Create and save path
        self.path = adjustable_path_points("parking", self.Atan_start, heading=self.pp_heading)
        self.pp_path = copy(self.path)  # Saves for use when going forwards
        # Prepare for going backwards
        self.ld = 0.35
        self.change_to_reversed()
        # Going backwards
        self.__pure_pursuit()
        self.parked = False

    ## Minor forwards adjustment. Icing on the cake.
    def parallell_parking_forwards(self):
        # Changing states
        self.going_backwards = False
        self.going_forwards = True
        rospy.sleep(1.0)
        # Finds goal position
        xr, yr, __ = self.__find_current_position()
        outward_distance = -0.15
        parallell_distance = -0.2
        x_goal = self.pp_corner[0] + parallell_distance * cos(self.pp_heading) - outward_distance * sin(self.pp_heading)
        y_goal = self.pp_corner[1] + parallell_distance * sin(self.pp_heading) + outward_distance * cos(self.pp_heading)
        # Finds path to goal position
        self.path = adjustable_path_points("linear", (xr, yr), goal=(x_goal, y_goal))
        # Prepares
        self.ld = 0.25
        self.change_to_forward()
        # Goes forward
        self.__pure_pursuit()
        
    ## Stops early if the car is in a good place when going backwards.
    def __check_backwards_done(self, forwards = False):
        xr, yr, heading = self.__find_current_position()
        if not forwards:
            enough_backwards = ((self.pp_corner[0] - xr) * cos(self.pp_heading) + (self.pp_corner[1] - yr) * sin(self.pp_heading) > 0.25)
            enough_inwards = (- (self.pp_corner[0] - xr) * sin(self.pp_heading) + (self.pp_corner[1] - yr) * cos(
                self.pp_heading) > 0.06)
            small_heading = (abs(self.pp_heading - heading) < pi/20)
            #print(abs(self.pp_heading - heading))
            if enough_backwards and enough_inwards and small_heading:
                print("Early stop")
                self.path = []
            if small_heading:
                print("Small heading")
        else:       # Same idea, but for forward parking
            enough_forwards = ((self.fp_corner[0] - xr) * cos(self.pp_heading) + (self.fp_corner[1] - yr) * sin(self.pp_heading) < -0.25)
            enough_inwards = (- (self.fp_corner[0] - xr) * sin(self.pp_heading) + (self.fp_corner[1] - yr) * cos(
                self.pp_heading) > 0.06)
            small_heading = (abs(self.pp_heading - heading) < pi / 20)
            #print(abs(self.pp_heading - heading))
            if enough_forwards and enough_inwards and small_heading:
                print("Early stop")
                self.path = []

    ## Changes a linear path to another parallel one, a certain outward_distance away, with a length of parallel distance
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

    ## Finds the corners of the obstacles to park between preemptively.
    def preemptive_corner_finding(self, ranges, angles):
        i = 0
        filtered_ranges = []
        filtered_angles = []
        for i in range(len(ranges)):
            if ranges[i] < 12:  # Filters away "infinity noise"
                filtered_ranges.append(ranges[i])
                filtered_angles.append(angles[i])
        tmpAngles, tmpRanges = [], []
        for i in range(len(filtered_angles)):
            if (0<= angles[i] < pi):    # Only looks in the right demicircle (left of lidar)
                tmpAngles.append(filtered_angles[i])
                tmpRanges.append(filtered_ranges[i])
        # Next, we only look at the values that are in front of the angle with the closest range.
        newAngles = tmpAngles[argmin(tmpRanges):]  # newAngles is from the angles where the shortest range starts
        newRanges = tmpRanges[argmin(tmpRanges):]  # newRanges is from the shortest range
        DeltaRanges = [j - i for i, j in zip(newRanges[:-1], newRanges[1:])]  # first derivative of ranges
        first_corner_x, first_corner_y = None, None
        second_corner_x, second_corner_y = None, None
        for i in range(len(DeltaRanges)):
            # If there is a positive jump discontinuity in distance after the closest value, we found the first corner
            if DeltaRanges[i] > 0.15:   
                first_corner_idx = i
                l1 = newRanges[first_corner_idx]
                # Local coordinates of vehicle in behind
                first_corner_x = -l1 * cos(newAngles[first_corner_idx])
                first_corner_y = -l1 * sin(newAngles[first_corner_idx])
                # If we find the first corner, we start looking for the second
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
                    #self.path = adjustable_path_points("linear", (start_point_x, start_point_y),
                    #                                   (goal_point_x, goal_point_y))
                    self.found_path = True
                DeltaRanges.reverse()       # Look for the last point, closer than l2 meters, where the distances increase
                for j in range(len(DeltaRanges)):
                    if DeltaRanges[j] < -0.01:
                        second_corner_idx = len(DeltaRanges) - 1 - j
                        l2 = newRanges[second_corner_idx]
                        if l2 < 2.0:
                            test_corner_y = -l2 * sin(newAngles[second_corner_idx])
                            if abs(test_corner_y - first_corner_y) <= 0.1:
                                #print("Angle: ",newAngles[second_corner_idx])
                                #print("Range: ",newRanges[second_corner_idx])
                                #print("MinAngle: ",newAngles[0])
                                #print("MinRange: ",newRanges[0])
                                second_corner_x = -l2 * cos(newAngles[second_corner_idx])
                                second_corner_y = -l2 * sin(newAngles[second_corner_idx])
                                break
                break

        # Calculate the distance between the 1st corner and 2nd corner
        if second_corner_x:
            self.parking_lot_dist = dist((first_corner_x, first_corner_y), (second_corner_x, second_corner_y))
        else:
            return
        if 0.7 <= self.parking_lot_dist < 1.1:      # Prepare for parallel parking using parking_stop
            self.has_parking_spot = True
            self.current_start_distance = -second_corner_y       
            self.parking_identified = 0     
            print("Preparing parallel parking, the distance <1.1")
            xr, yr, heading = self.__find_current_position()
            second_corner_x_real = xr + second_corner_x * cos(heading) - second_corner_y * sin(heading)
            second_corner_y_real = yr + second_corner_x * sin(heading) + second_corner_y * cos(heading)
            self.pp_corner = (second_corner_x_real, second_corner_y_real)
            print("Second corner :", (second_corner_x_real, second_corner_y_real))

        elif self.parking_lot_dist >= 1.1: # Prepare for forwardparking
            # self.has_parking_spot = True
            self.parking_identified = 3     # find large parking spot
            xr, yr, heading = self.__find_current_position()
            first_corner_x_real = xr + first_corner_x * cos(heading) - first_corner_y * sin(heading)
            first_corner_y_real = yr + first_corner_x * sin(heading) + first_corner_y * cos(heading)
            second_corner_x_real = xr + second_corner_x * cos(heading) - second_corner_y * sin(heading)
            second_corner_y_real = yr + second_corner_x * sin(heading) + second_corner_y * cos(heading)
            self.fp_corner = (first_corner_x_real, first_corner_y_real)
            print("Preparing forward parking,dist > 1.1")
            print("First corner local: ",(first_corner_x, first_corner_y))
            print("First corner :", self.fp_corner)
            print("Distances :", self.parking_lot_dist)
            print("Second corner :", (second_corner_x_real, second_corner_y_real))
            self.ld = 0.32
            self.doing_forward_parking = True
            self.forward_parking()
            # just drive in directly

    ## Looks behind the vehicle, goes as far back as it can, and follow an arctan path out of the parking lot
    def leave_from_parking(self, howparked):
        # Preprocess the scan data, remove infinity noise
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
        look_dist = 0.22 if howparked == "backward_parking" else 0.28       # Heuristic
        in_dist = 0.05
        goal_pos_x = xr - (backward_dist - look_dist) * cos(self.pp_heading) + in_dist * sin(self.pp_heading)
        goal_pos_y = yr - (backward_dist - look_dist) * sin(self.pp_heading) - in_dist * cos(self.pp_heading)
        self.path = adjustable_path_points("linear", (xr, yr), goal=(goal_pos_x, goal_pos_y))
        self.change_to_reversed()
        self.__pure_pursuit()
        rospy.sleep(1.0)

        if howparked == "forward_parking":
            self.path = adjustable_path_points("parking_out",(goal_pos_x, goal_pos_y), heading=self.pp_heading + pi)
            self.ld = 0.32

        if howparked == "backward_parking":
            print("i am here 5")
            print(self.pp_heading)
            self.path = adjustable_path_points("parking",(goal_pos_x, goal_pos_y), heading=self.pp_heading + pi)
            self.ld = 0.32

        self.change_to_forward()
        self.__pure_pursuit()

    ## Creates an arctan path which it lets the pure pursuit algorithm follow until it ends, based on the calculated corners.
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
    
    ## Looks to the right to determine when the lot begins and ends
    def parking_stop(self, data):
        angles = arange(data.angle_min, data.angle_max + data.angle_increment, data.angle_increment)
        ranges = data.ranges
        parking_threshold = 0.7
        pp_len_threshold = 0.7  # Length of gap, subject to change
        for i in range(len(angles)):
            if (angles[i] < pi / 2 + pi / 50) and (angles[i] > pi / 2 - pi / 50):
                if self.parking_identified == -1:       # Finding corners preemptively
                    self.preemptive_corner_finding(ranges, angles)
                elif self.parking_identified == 0:            # No lot identified yet
                    if ranges[i] < parking_threshold:
                        return
                    elif angles[i+1] > pi / 2 + pi / 50 or angles[i+1] < pi / 2 - pi / 50:    # All relevant angles passed test
                        # Identified start of lot
                        self.parking_lot_start = [self.car_pose.pose.pose.position.x, self.car_pose.pose.pose.position.y]
                        self.parking_identified = 1
                        # Change lane to be sufficiently close to parked vehicles
                        path_heading = arctan2(self.path[1][1] - self.path[0][1], self.path[1][0] - self.path[0][0])
                        xr,yr,_ = self.__find_current_position()
                        gamma = arctan2(self.pp_corner[1]-yr, self.pp_corner[0]-xr)
                        dl = dist((xr,yr),self.pp_corner)
                        self.current_start_distance = dl*sin(path_heading-gamma)
                        outward_distance_to_move = 0.3 - self.current_start_distance    # Lane adjustment
                        self.change_lane(parallell_distance=2.0, outward_distance=outward_distance_to_move)
                elif ranges[i] < parking_threshold and self.parking_identified == 1:    # Start but not end of lot identified
                    parking_lot_end = [self.car_pose.pose.pose.position.x, self.car_pose.pose.pose.position.y]
                    self.parking_lot_dist = dist(parking_lot_end, self.parking_lot_start)   # Checks that the lot is long enough
                    if self.parking_lot_dist > pp_len_threshold:
                        self.has_parking_spot = True
                    if self.has_parking_spot:
                        self.parking_identified = 2             # parking_stop will detect no more lots
                        self.generate_obs_list(angles, ranges)
                        self.pp_range = ranges[i]
                        self.pp_angle = angles[i]
                        self.__set_pp_corner(data)
                        self.parallell_parking_start()
                    elif self.parking_lot_dist > 0.03:          # Fix for slanted walls of obstacles.
                        self.parking_identified = 0
                    else:
                        self.tried = True

    ## Decides the corner of the front obstacle as the closest obstacle to the right of the vehicle
    ## Used in parking_stop(data)
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


    ## Uses MOCAP to transform obstacles from polar local coordinates to
    ## cartesian global coordinates
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
        
    ## E-stop for reverse travelling. Not currently used
    def reversed_lidar_cb(self,data):
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

    ## E-stop for forward travelling. Not currently used.
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
    
