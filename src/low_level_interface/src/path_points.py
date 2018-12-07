from numpy import *


def path_points(type_path):
    # the area is 4 X 4 meter^2
    path = []
    nPoint = 108
    if type_path == 'circle':
        r = 1  # the radius of the circle path
        alpha = linspace(-pi, 1.05 * pi, nPoint)
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
        x0, y0 = 1.3, -1.5  # initial point
        xg, yg = 1.3, 1.5  # goal point
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

    elif type_path == 'small_circle':
        r = 0.5  # the radius of the circle path
        alpha = linspace(-pi, 1.05 * pi, nPoint)
        for a in alpha:
            x = r * cos(a)
            y = r * sin(a)
            path.append([x,y])
    else:
        print("I do NOT generate this kind of path")



    return path


def adjustable_path_points(type_path, start, goal = None, heading = None):
    path = []
    nPoint = 108
    if type_path == 'linear':
        x0, y0 = start[0], start[1]  # initial point
        xg, yg = goal[0], goal[1]  # goal point
        xrange = linspace(x0, xg, nPoint)
        yrange = linspace(y0, yg, nPoint)
        for i in arange(nPoint):
            path.append([xrange[i], yrange[i]])
    elif type_path == 'parking':
        x0, y0 = 0, 0  # initial point
        xg, yg = 0.7, 0.48  # goal point
        xrange = linspace(x0, 2.2 * xg, nPoint)
        yrange = 0.48/(2*pi/2)*arctan((xrange - xg * 0.6) * 10 / 0.7)
        yrange = yrange - yrange[0]
        yrange = yrange * 0.45 / yrange[-1]
        xrange = linspace(x0, 1.25 * xg, nPoint)
        x_real = xrange * cos(pi+heading) - yrange * sin(pi+heading) + start[0]
        y_real = xrange * sin(pi+heading) + yrange * cos(pi+heading) + start[1]
        
        for i in arange(nPoint):
            path.append([x_real[i], y_real[i]])
    elif type_path == 'parking_forward':
        x0, y0 = 0, 0  # initial point
        xg, yg = 0.9, 0.48  # goal point
        xrange = linspace(x0, 2.2 * xg, nPoint)
        yrange = -0.48 / (2 * pi / 2) * arctan((xrange - xg * 0.6) * 10 / 0.9)
        yrange = yrange - yrange[0]
        yrange = -yrange * 0.45 / yrange[-1]
        xrange = linspace(x0, 1.25 * xg, nPoint)
        x_real = xrange * cos(heading) - yrange * sin(heading) + start[0]
        y_real = xrange * sin(heading) + yrange * cos(heading) + start[1]

        for i in arange(nPoint):
            path.append([x_real[i], y_real[i]])
        # savetxt("/home/lindstah/Documents/EL2425project/planned_path.csv", array(path), delimiter=",")
        print(path)[-1]
    return path

if __name__ == "__main__":
    adjustable_path_points('parking', (1.5, 1.5), (0, 0), heading=pi / 2)



