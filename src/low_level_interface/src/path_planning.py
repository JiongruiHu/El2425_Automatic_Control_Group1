#!/usr/bin/env python
from numpy import *
import matplotlib.patches as plp
import matplotlib.pyplot as plt
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
            self.F = self.G + self.h + (self.np.F - self.np.h)
            # F cost is the sum of the total cost of the node and
            # its parents cost


class Path:
    def __init__(self, start, goal, obs, current_heading):
        self.car_p = start
        self.car_size = (4, 1.6)   # (length, width)
        self.obs = self.inflated(obs)
        self.goal = goal  # (goal[0]-start[0], goal[1] - start[1])
        self.car_speed = 1
        self.car_heading = current_heading

    def parking_path(self):
        open_list, close_list, current_list = [], [], []
        steering = linspace(-pi/4, pi/4, 5)
        initNode = AstarNode(self.car_p, self.goal, self.car_heading)
        initNode.add_gcost(0)
        open_list.append(initNode)
        while open_list is not []:
            node = open_list[0]
            for n in open_list:
                if n.F < node.F:
                    node = n
                    if reach_goal(node.p, self.goal):
                        return node
            open_list.remove(node)
            #current_list.append(node)
            open_list, close_list = self.update_neighbour(open_list, close_list, node)
        return None

    def build_path(self):
        path, controls, time = [], [], [0]
        route = self.parking_path()
        while route.np is not None:
            x, y = route.p[0], route.p[1]
            plt.plot(x, y, 'g*', markersize=1)
            plt.show()
            path.append(route)
            route = route.np
        path.reverse()
        for nod in path:
            controls.append(nod.steer)
            time.append(nod.t)
        return controls, time

    def update_neighbour(self, openL, closeL, n):
        for delta in linspace(-pi/4, pi/4, 7):    # steering range
            ns = self.find_new_node(n, delta)
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
        g = 0
        while t < 1 and state is True:
            xn, yn, headingn = self.bicycle_backward(xl[-1], yl[-1], headingl[-1], delta)
            g = g + dist((xn,yn),(xl[-1], yl[-1]))
            if checkcollision((xn, yn), self.obs):  # if no collision
                xl.append(xn)
                yl.append(yn)
                headingl.append(headingn)
                t = t + 0.01
            else:
                state = False

        new_node = AstarNode((xl[-1], yl[-1]), self.goal, headingl[-1])
        new_node.np = node
        new_node.t = t
        new_node.steer = delta
        if state is False:
            new_node.add_gcost(1000)
        else:
            new_node.add_gcost(g)
        return new_node

    # we assume the car is running in a constant speed.
    def bicycle_backward(self, x, y, heading, steering):
        dt = 0.01  # the car should reach the goal in 10 second
        lr = 3.2/2
        beta = arctan(tan(steering) * 0.5)  # 0.5=lr/(lf+lr)
        dx = - self.car_speed * cos(heading + beta)
        dy = self.car_speed * sin(heading + beta)

        xn = x + dx * dt
        yn = y + dy * dt
        headingn = heading + self.car_speed * sin(beta) * dt / lr

        return xn, yn, headingn

    def bicycle_forward(self, x, y, heading, steering):
        dt = 0.01  # the car should reach the goal in 10 second
        lr = 3.2 / 2
        beta = arctan(tan(steering) * 0.5)  # 0.5=lr/(lf+lr)
        dx = self.car_speed * cos(heading + beta)
        dy = self.car_speed * sin(heading + beta)

        xn = x + dx * dt
        yn = y + dy * dt
        headingn = heading + self.car_speed * sin(beta) * dt / lr

        return xn, yn, headingn

    # inflated the obstacles with circle which has cars half length as radius
    # need to some changes maybe
    def inflated(self, obs):
        inflated_obs = []
        for o in obs:
            o[2] = self.car_size[1]/2
            inflated_obs.append(o)
        return inflated_obs

    def my_plot(self, obs_list):
        car_length, car_width = self.car_size[0], self.car_size[1]
        cx, cy = self.car_p[0], self.car_p[1]
        fig = plt.gcf()
        ax = fig.gca()
        for obs in obs_list:
            ax.add_patch(plt.Circle((obs[0], obs[1]), obs[2], facecolor="grey", edgecolor="k"))
            ax.set_aspect('equal')
            ax.plot()

        # plt.xticks([i + 1 for i in range(0, 30)])
        # plt.yticks([i + 1 for i in range(0, 6)])
        rect_s = plp.Rectangle((5, 0.1), car_length, car_width, fill=False)
        # rect_g = plp.Rectangle((10, 2.5), car_length, car_width, fill=False)
        # ax.add_patch(plt.Circle((cx, cy), 0.1,'*'))
        plt.plot(cx,cy, marker='*', markersize=1)
        ax.add_patch(rect_s)
        # ax.add_patch(rect_g)
        plt.grid()
        plt.ylim((0, 6))
        plt.xlim((0, 30))
        plt.show()


def dist(p1, p2):
    return sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)


def reach_goal(p, goalpoint):
    if dist(p, goalpoint) < 0.01:
        return True
    else:
        return False


# the obstacles must be inflated before checking the collision
def checkcollision(p, obs):
    for o in obs:
        if dist(p, (o[0], o[1])) < o[2] + 0.05:
            return False  # collision
    return True     # no collision


if __name__ == "__main__":
    startp, goalp = (7, 0.9), (12, 3.5)

    obs_lists = myObsList()
    car = Path(startp, goalp, obs_lists, pi)
    #car.my_plot(obs_lists)
    control_list, time_list = car.build_path()

