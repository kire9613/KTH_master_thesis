import bresenham
from math import sin, cos, pi,tan, atan2,log
import math
import numpy as np

class OccupancyGrid(object):
    """Occupancy Grid representation of a map. It follows the conventions
    of the ROS message OccupancyGrid in terms of parameters

    :param width: map width, in cells
    :type width: int
    :param height: map height, in cells
    :type height: int
    :param resolution: map resolution in m/cells, defaults to 1
    :type resolution: int, optional
    :param obstacles: List of obstacles (each one with obstacle edges)
    :type obstacles: list
    """

    EMPTY = 0
    FULL = 100

    def __init__(self, width, height, origin=(0, 0), resolution=1, obstacles=[]):

        self.obstacles = obstacles

        self.height = height
        self.width = width
        self.resolution = resolution
        self.origin = origin

        self.localmap = np.zeros((self.width+1, self.height+1))

        self.prior = log(0.5/0.5)
        self.logodds = np.ones((self.width+1, self.height+1))*self.prior

        self.pfree = log(0.4/0.6)
        self.pocc = log(0.95/0.05)

        self.max_logodd = 100.0
        self.max_logodd_belief = 0
        self.max_scan_range = 1.0

    def array_to_index(self, array):

        self.origin_array = np.ones([np.size(array, 0), 2])
        self.origin_array[:, 0] = self.origin_array[:, 0] * self.origin[0]
        self.origin_array[:, 1] = self.origin_array[:, 1] * self.origin[1]

        norm_array = array - self.origin_array

        index = (np.fix(norm_array*1/self.resolution)) 

        index = index.astype(int)

        return index

    def updatemap(self, scandata, obs, pose):

        obs_idx = self.array_to_index(obs)

        pose_idx = [int(math.floor((pose[0] - self.origin[0])*1/self.resolution)), int(math.floor((pose[1] - self.origin[1])*1/self.resolution))]
        
        for i in range(len(scandata)):

                l = bresenham.bresenham(pose_idx, obs_idx[i])

                for j in range(len(l.path)): 

                        index = l.path[j]

                        if(j < len(l.path)-1):
                          self.logodds[index] += self.pfree - self.prior

                        else:
                          self.logodds[index] += self.pocc - self.prior
                        
                        if self.logodds[index] > self.max_logodd:
                          self.logodds[index] = self.max_logodd

                        elif self.logodds[index] < -self.max_logodd:
                          self.logodds[index] = -self.max_logodd

                        if self.logodds[index] > self.max_logodd_belief:
                          self.localmap[index] = 100
                       
                        else:
                          self.localmap[index] = 0












