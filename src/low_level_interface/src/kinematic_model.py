from numpy import *

def bicycle_backward(self, x, y, heading, steering):
    dt = 0.01  # the car should reach the goal in 10 second #Not really relevant for the choice of dt?
    lr = 3.2 / 2
    beta = -1 * arctan(tan(steering) * 0.5)  # 0.5=lr/(lf+lr)
    dx = self.car_speed * cos(heading + pi + beta)  # Something feels weird here
    dy = self.car_speed * sin(heading + pi + beta)

    xn = x + dx * dt
    yn = y + dy * dt
    headingn = heading + self.car_speed * sin(beta) / lr * dt

    return xn, yn, headingn


def bicycle_forward(self, x, y, heading, steering):
    dt = 0.01  # the car should reach the goal in 10 second
    lr = 3.2 / 2
    beta = arctan(tan(steering) * 0.5)  # 0.5=lr/(lf+lr)
    dx = self.car_speed * cos(heading + beta)
    dy = self.car_speed * sin(heading + beta)

    xn = x + dx * dt
    yn = y + dy * dt
    headingn = heading + self.car_speed * sin(beta) / lr *dt

    return xn, yn, headingn