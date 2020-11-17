# Christopher Iliffe Sprague
# sprague@kth.se

from random import uniform
import json
import math
import numpy as np

class Objective(object):
    '''
    Holds start and target position
    Find solution in environment (map)
    '''

    def __init__(self, xt, yt, x0, y0, env):

        # set environment
        # env = Environment(map, map_info)
        self._environment = env

        # target and start pos.
        self.x0 = x0
        self.y0 = y0
        self.xt = xt
        self.yt = yt

def step(car, x, y, theta, phi, dt=0.01):
    '''
    Returns a new state (xn, yn, thetan),
    given an initial state (x, y, theta) and control phi.
    Numerical integration is done at a time step of dt [sec].
    '''

    #_update(self, state, accel, delta, dt)
    L = 0.32
    DELTA_MAX = 40*math.pi/180
    v = 1

    phi = np.clip(phi, -DELTA_MAX, DELTA_MAX)
    xn = x + (v * np.cos(theta) * dt)
    yn = y + (v * np.sin(theta) * dt)
    thetan = theta + (v/L * np.tan(phi) * dt)

    # state rate
    #dx     = cos(theta)
    #dy     = sin(theta)
    #dtheta = tan(phi)

    # new state (forward Euler integration)
    #xn     = x     + dt*dx
    #yn     = y     + dt*dy
    #thetan = theta + dt*dtheta

    return xn, yn, thetan

class Environment(object):
    '''
    Environment described map
    Functions:  Check for collision
                Check for safe path
                Check if point is inside map
    '''

    def __init__(self, map, map_info):

        self.map = map # gridmap
        self.origin_y = map_info[4]
        self.origin_x = map_info[3]
        self.resolution = map_info[2]
        self.width = map_info[0]
        self.height = map_info[1]

    def inbounds(self, x, y):
        """ Check if point is inside bounds of gridmap """
        if x >= 0 and x < self.width:
            if y >= 0 and y < self.height:
                return True
        return False

    def obstacle_free(self, x, y):
        """ Check if point is in occupied position in gridmap """
        if self.map[y,x] != 0:
            return False
        else:
            return True

    def safe(self, x, y):
        ''' Tests whether a point is within boundaries and obstacle free. '''
        y_map = y - self.origin_y
        x_map = x - self.origin_x
        y_index = int(y_map/self.resolution)
        x_index = int(x_map/self.resolution)
        if self.inbounds(x_index, y_index) and self.obstacle_free(x_index, y_index):
            return True
        else:
            return False
