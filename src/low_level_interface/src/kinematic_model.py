from numpy import *


def bicycle_backward(car_speed,x, y, heading, steering):
    dt = 0.01  # the car should reach the goal in 10 second #Not really relevant for the choice of dt?
    lr = 0.32 / 2
    beta = -1 * arctan(tan(steering) * 0.5)  # 0.5=lr/(lf+lr)
    dx = car_speed * cos(heading + pi + beta)  # Something feels weird here
    dy = car_speed * sin(heading + pi + beta)

    xn = x + dx * dt
    yn = y + dy * dt
    headingn = heading + car_speed * sin(beta) / lr * dt

    return xn, yn, headingn


def bicycle_forward(car_speed, x, y, heading, steering):
    dt = 0.01  # the car should reach the goal in 10 second
    lr = 0.32 / 2
    beta = arctan(tan(steering) * 0.5)  # 0.5=lr/(lf+lr)
    dx = car_speed * cos(heading + beta)
    dy = car_speed * sin(heading + beta)

    xn = x + dx * dt
    yn = y + dy * dt
    headingn = heading + car_speed * sin(beta) / lr * dt

    return xn, yn, headingn
