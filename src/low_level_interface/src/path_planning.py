#!/usr/bin/env python
from numpy import *
import matplotlib.patches as plp
import matplotlib.pyplot as plt
from kinematic_model import *
from myObsList import *


class AstarNode:
    def __init__(self, position, goal, heading):
        self.p = position
        self.h = dist(position, goal)   # hcost is the Euclidean distance from the node to the goal
        self.np = None  # point to its parent
        self.heading = heading
        self.t = 0
        self.steer = 0

    def add_gcost(self, gcost):  # the cost from the start to the node
        self.G = gcost  # gcost is the Euclidean distance from the start to the node
        if self.np is None:
            self.F = self.h
        else:
            self.F = self.G + self.h #+ (self.np.F - self.np.h)
            # F cost is the sum of the total cost of the node


class Path:
    def __init__(self, start, goal, obs, current_heading):
        self.car_p = start
        self.car_size = (0.4, 0.16)   # (length, width)
        self.obs = self.inflated(obs)
        self.goal = goal  # (goal[0]-start[0], goal[1] - start[1])
        self.car_speed = 0.1
        self.car_heading = current_heading

        savetxt("/home/nvidia/catkin_ws/obs_with.csv", self.obs, delimiter=",")

    def parking_path(self):
        open_list, close_list, current_list = [], [], []
        initNode = AstarNode(self.car_p, self.goal, self.car_heading)
        print("position", self.car_p)
        print("heading",self.car_heading)
        print("goal",self.goal)

        initNode.add_gcost(0)
        open_list.append(initNode)
        #fig = plt.gcf()
        #ax = fig.gca()
        while len(open_list) > 0:
            node = open_list[0]
            for n in open_list:
                if n.F < node.F:
                    node = n
                    if reach_goal(node.p, self.goal):
                        return node
            open_list.remove(node)
            if node.t < 10:#current_list.append(node)
                open_list, close_list = self.update_neighbour(open_list, close_list, node)

        return None

    def build_path(self):
        path, controls, time = [], [], []
        print("Planning internal path...")
        route = self.parking_path()
        if not route:
            print("No path!!")
            return [0], [0]
        while route.np is not None:
            #x, y = route.p[0], route.p[1]
            path.append(route)
            route = route.np
        path.reverse()
        print("Returning...")
        for nod in path:
            controls.append(nod.p)
            time.append(nod.t)
        print(controls)
        return controls, time

    def update_neighbour(self, openL, closeL, n):
        for delta in linspace(-pi/4, pi/4, 7):    # steering range
            ns = self.find_new_node(n, delta)
            #plt.plot(ns.p[0], ns.p[1], 'g*', markersize=1)
            #plt.show()
            nn = []
            if ns not in closeL:
                nn = [n for n in openL if (dist(ns.p, n.p) <= 0.01)]
                if ns.G == 1000 or (checkcollision(ns.p, self.obs) is not True):
                    closeL.append(ns)
                elif nn:
                    if ns.G < nn[0].G:
                        openL[openL.index(nn[0])] = ns
                else:
                    openL.append(ns)

        return openL, closeL

    def find_new_node(self, node, delta):
        xl, yl, headingl = [node.p[0]], [node.p[1]], [node.heading]
        state = True
        t = 0
        collision = False
        while t < 1 and state is True:
            xn, yn, headingn = bicycle_backward(self.car_speed, xl[-1], yl[-1], headingl[-1], delta)
            #g = g + dist((xn,yn),(xl[-1], yl[-1]))
            if checkcollision((xn, yn), self.obs) and not reach_goal((xn, yn), self.goal) and self.in_bounds((xn,yn)):
                #if close_goal((xn,yn), goalpoint) and abs(headingn-self.car_heading) < 0.1:
                # if no collision
                    xl.append(xn)
                    yl.append(yn)
                    headingl.append(headingn)
                    t = t + 0.01
            elif reach_goal((xn, yn), self.goal):
                if abs(headingn-self.car_heading) < 0.2:
                    xl.append(xn)
                    yl.append(yn)
                    headingl.append(headingn)
                    t = t + 0.01
                    state = False

            else:
                state = False
                collision = True
        #plt.plot(xl, yl , 'r-', markersize=0.1)
        #plt.plot(xl[-1], yl[-1], 'g*')
        #plt.axis('equal')
        #plt.show()
        new_node = AstarNode((xl[-1], yl[-1]), self.goal, headingl[-1])
        new_node.np = node
        new_node.t = t + node.t
        new_node.steer = delta
        if state is False and collision is True:
            new_node.add_gcost(1000)
        else:
            new_node.add_gcost(dist(self.goal, (xl[-1], yl[-1])))
        return new_node

    # inflated the obstacles with circle which has cars half length as radius
    # need to some changes maybe
    def inflated(self, obs):
        inflated_obs = []
        for o in obs:
            o.append(self.car_size[0]/2)
            inflated_obs.append(o)
        return inflated_obs

    def in_bounds(self, p):
        r = 1.5
        if dist(p, self.car_p) >= r:
            return False
        else:
            return True


def dist(p1, p2):
    return sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)


def reach_goal(p, goalpoint):
    if dist(p, goalpoint) < 0.3:
        return True
    else:
        return False


def close_goal(p, goalpoint):
    d = dist(p, goalpoint)
    if d < 0.2 and d > 0.1:
        return True
    else:
        return False



# the obstacles must be inflated before checking the collision
def checkcollision(p, obs):
    for o in obs:
        if dist(p, (o[0], o[1])) < o[2] + 0.05:
            return False  # collision
    return True     # no collision



