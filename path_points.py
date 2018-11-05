from numpy import *


def path_points(type_path):
    # the area is 4 X 4 meter^2
    path = []
    nPoint = 36
    if type_path == 'circle':
        r = 3 # the radius of the circle path
        alpha = linspace(-pi,pi,nPoint)
        for a in alpha:
            x = r * cos(a)
            y = r * sin(a)
            path.append([x,y])

    elif type_path == 'linear':
        x0, y0 = 0, 0 # initial point
        xg, yg = 2, 3 # goal point
        xrange = linspace(x0, xg, nPoint)
        yrange = linspace(y0, yg, nPoint)
        for i in arange(nPoint):
            path.append([xrange[i], yrange[i]])

    elif type_path == 'ellipse':
        a, b = 3, 2
        alpha = linspace(-pi, pi, nPoint)
        for a in alpha:
            x = a * cos(a)
            y = b * sin(a)
            path.append([x,y])

    else:
        print('I do NOT generate this kind of path')
    return path


