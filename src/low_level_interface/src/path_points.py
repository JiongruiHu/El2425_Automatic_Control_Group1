from numpy import *


def path_points(type_path):
    # the area is 4 X 4 meter^2
    path = []
    nPoint = 108
    if type_path == 'circle':
        r = 1  # the radius of the circle path
        alpha = linspace(-pi,pi,nPoint)
        for a in alpha:
            x = r * cos(a)
            y = r * sin(a)
            path.append([x,y])

    elif type_path == 'reversed_circle':
        r = 1  # the radius of the circle path
        alpha = linspace(-1.5*pi,pi,nPoint)
        for a in reversed(alpha):
            x = r * cos(a)
            y = r * sin(a)
            path.append([x,y])

    elif type_path == 'linear':
        x0, y0 = -1, -1.5  # initial point
        xg, yg = 1.5, -1.5  # goal point
        xrange = linspace(x0, xg, nPoint)
        yrange = linspace(y0, yg, nPoint)
        for i in arange(nPoint):
            path.append([xrange[i], yrange[i]])

    elif type_path == 'reversed_linear':
        x0, y0 = 1.5, -1.5  # initial point
        xg, yg = -1, -1.5  # goal point
        xrange = linspace(x0, xg, nPoint)
        yrange = linspace(y0, yg, nPoint)
        for i in arange(nPoint):
            path.append([xrange[i], yrange[i]])

    elif type_path == 'ellipse':
        a, b = 0.8, 1.6
        alpha = linspace(-pi, pi, nPoint)
        for al in alpha:
            x = a * cos(al)
            y = b * sin(al)
            path.append([x,y])
    elif type_path == 'figure-8':
        a, b = 1.8, 1.8
        alpha = linspace(-pi/2, 11*pi/2, nPoint)
        for al in alpha:
            x = a * cos(al)
            y = b * sin(al) * cos(al)
            path.append([x,y])
    else:
        print("I do NOT generate this kind of path")

    return path


def adjustable_path_points(type_path, start, goal, heading = None):
    path = []
    nPoint = 108
    if type_path == 'linear':
        x0, y0 = start[0], start[1]  # initial point
        xg, yg = goal[0], goal[1]  # goal point
        xrange = linspace(x0, xg, nPoint)
        yrange = linspace(y0, yg, nPoint)
        for i in arange(nPoint):
            path.append([xrange[i], yrange[i]])
    return path



