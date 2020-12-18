import numpy as np
from scipy.spatial import Delaunay
from scipy.spatial import ConvexHull

import matplotlib.pyplot as plt	

from functools import reduce
import math

import sys
import numpy
numpy.set_printoptions(threshold=sys.maxsize)

import scipy.io
import scipy.stats
from tqdm import tqdm

import bresenham
from math import sin, cos, pi,tan, atan2,log
import math
import tf
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
        ## The map resolution [m/cell]
        self.resolution = resolution
        ## Map width [cells]
        self.width = width
        ## Map height [cells]
        self.height = height
        ## The origin of the map [m, m]
        self.origin = origin
        ## Map data, occupancy probabilities in the range [0, 100]. Unknown is -1
        self.grid_data = np.zeros((self.width+1, self.height+1))

        ## Obstacles represented by edges of a polyhedron
        self.obstacles = obstacles

        #self._add_obstacles_to_map()
         
        self.point_data = []
	self.data_done = 0
        self.A = []
        self.b = []
        self.indexes = []
        #"""

        self.height = height
        self.width = width
        self.resolution = resolution
        self.origin = origin

        #self.punknown=-1.0
        #self.localmap=[self.punknown]*int(self.width/self.resolution)*int(self.height/self.resolution)
        #self.logodds=[0.0]*int(self.width/self.resolution)*int(self.height/self.resolution)
        self.localmap = np.zeros((self.width+1, self.height+1))

        self.prior = log(0.5/0.5)
        self.logodds = np.ones((self.width+1, self.height+1))*self.prior

        #self.origin_grid=int(math.ceil(origin[0]/resolution))+int(math.ceil(width/resolution)*math.ceil(origin[1]/resolution))

        self.pfree = log(0.4/0.6)
        self.pocc = log(0.95/0.05)

        self.max_logodd = 100.0
        self.max_logodd_belief = 0
        self.max_scan_range = 1.0

        #"""

    @property
    def total_width_m(self):
        """Gets the total width of the map, in m

        :return: total width of the map, in m
        :rtype: float
        """
        return self.width * self.resolution

    @property
    def total_height_m(self):
        """Gets the total height of the map, in m

        :return: total height of the map, in m
        :rtype: float
        """
        return self.height * self.resolution

    def is_node_within_map(self, node):
        """Checks if the node being passed as an argument is within the map

        :param node: Tuple representing a cell with coordinates (x, y) or (x, y, yaw)
        :type node: tuple
        :return: `True` if the node is within the map, `False` otherwise
        :rtype: bool
        """
        node = self._normalize_node_to_map_origin(node)

        check_x = node[0] >= 0 and node[0] < self.width * self.resolution
        check_y = node[1] >= 0 and node[1] < self.height * self.resolution

        return check_x and check_y

    def get_closest_cell(self, node):
        """Given an input node with continuous values, it interpolates and
        gets the closest cell available in the map (discrete cell)

        :param node: Tuple representing a cell with coordinates (x, y) or (x, y, yaw)
        :type node: tuple
        :return: closest cell (x, y) (or (x, y, yaw)) available in the map
        :rtype: tuple
        """
        node = self._normalize_node_to_map_origin(node)

        closest_x = int(node[0] / self.resolution + 0.5) * self.resolution
        closest_y = int(node[1] / self.resolution + 0.5) * self.resolution

        closest_node = (closest_x, closest_y)

        closest_node = self._restore_node_from_map_origin(closest_node)
        # Preserve other indices (e.g. orientation)
        return closest_node + node[2:]

    def _add_obstacles_to_map(self):
        """Given a list of obstacles (each one defined by its edges), it fills
        the grid cells that are contained within those obstacles.
        """
        for obstacle_edges in self.obstacles:
            self._add_obstacle_from_edges(obstacle_edges)

    def _add_obstacle_from_edges(self, obstacle_edges):
        """Given the edges of an obstacle, it fills the grid cells that are
        contained within the obstacle.

        :param obstacle_edges: List of edges with coordinates [x, y]
        :type obstacle_edges: list
        """
        normalized_obstacle_edges = [
            self._normalize_node_to_map_origin(obstacle_edge)
            for obstacle_edge in obstacle_edges
        ]

        hull = Delaunay(normalized_obstacle_edges)

        m = np.array([ [[1,1], [1,2]], [[2,1], [2,2]] ])
	r1 = np.array([[1, 1], [1, 2], [1, 3]])
	r2 = np.array([[2, 1], [2, 2], [2, 3]])
	r = np.stack((r1, r2))

        print(hull.find_simplex(r) >= 0)
        print("hej")

        """
        for x_index in range(self.width):
            for y_index in range(self.height):
                points = np.array([
                    [(x_index + 0.1 * offset_x) * self.resolution,
                     (y_index + 0.1 * offset_y) * self.resolution]
                    for offset_x in range(1, 10)
                    for offset_y in range(1, 10)
                ])

                if self.data_done == 0:
                  A, b = self._compute_convex_hull_equations(points)

		  #self.point_data.append([A, b, [x_index, y_index]])
                  self.A.append(A)
                  self.b.append(b)
                  self.indexes.append([x_index, y_index])

                if any(hull.find_simplex(points) >= 0):
                    self.grid_data[x_index, y_index] = self.FULL
        
        self.data_done = 1
        """

    def _normalize_node_to_map_origin(self, node):
        """Translates node from global coordinates to coordinates relative to map origin

        :param node: Tuple representing a cell with coordinates (x, y) or (x, y, yaw).
                      List representing a cell with coordinates [x, y]
        :type node: tuple or list
        :return: normalized tuple (x, y) (or (x, y, yaw)) with coordinates relative to the map.
        :rtype: tuple
        """
        node_x = node[0] - self.origin[0]
        node_y = node[1] - self.origin[1]

        return (node_x, node_y) + tuple(node[2:])

    def _restore_node_from_map_origin(self, node):
        """Translate node back from coordinates relative to map origin to global coordinates

        :param node: Normalized tuple (x, y) (or (x, y, yaw)) with coordinates relative to the map.
        :type node: tuple
        :return: Tuple representing a cell with coordinates (x, y) or (x, y, yaw).
        :rtype: tuple
        """
        node_x = node[0] + self.origin[0]
        node_y = node[1] + self.origin[1]

        return (node_x, node_y) + tuple(node[2:])

    def array_to_index(self, array):

        self.origin_array = np.ones([np.size(array, 0), 2])
        self.origin_array[:, 0] = self.origin_array[:, 0] * self.origin[0]
        self.origin_array[:, 1] = self.origin_array[:, 1] * self.origin[1]

        norm_array = array - self.origin_array

        index = (np.fix(norm_array*1/self.resolution)) 

        index = index.astype(int)

        return index


    def array_to_grid(self, array):

        index = self.array_to_index(array)

        for idx in index:

          self.grid_data[idx[0], idx[1]] = self.FULL

        idx_occ = np.argwhere(self.grid_data == self.FULL)

        return self.grid_data

    @staticmethod
    def _compute_convex_hull_equations(edge_points):

        hull = ConvexHull(edge_points)
        equations = hull.equations

        # Negate b to use the conventional representation Ax <= b
        A = equations[:, 0:2] #alla rader, 1:a o 2:a kolumnen
        b = -equations[:, 2] #alla rader, 3:e kolumnen

        return A, b

    def updatemap(self, scandata, angles, obs, range_min, range_max, pose):

        #robot_origin=int(pose[0])+int(math.ceil(self.width/self.resolution)*pose[1])

        #centreray = len(scandata)/2 + 1

        obs_idx = self.array_to_index(obs)

        pose_idx = [int(math.floor((pose[0] - self.origin[0])*1/self.resolution)), int(math.floor((pose[1] - self.origin[1])*1/self.resolution))]
        
        for i in range(len(scandata)):

            #if not math.isnan(scandata[i]):

                #beta = angles[i]
                #px = int(float(scandata[i])*cos(beta + pose[2])/self.resolution)
                #py = int(float(scandata[i])*sin(beta + pose[2])/self.resolution)
                #print(obs_idx[i])

                l = bresenham.bresenham(pose_idx, obs_idx[i])

                for j in range(len(l.path)): 
                   
                    #lpx = -self.origin[0]+pose[0]+l.path[j][0]*self.resolution
                    #lpy = -self.origin[1]+pose[1]+l.path[j][1]*self.resolution

                    #print(lpx)
                    #print(lpy)

                    #if (0 <= l.path[j][0] < self.width and 0 <= l.path[j][1] < self.height):

                        index = l.path[j] #self.origin_grid+int(l.path[j][0]+math.ceil(self.width/self.resolution)*l.path[j][1])

                        #if scandata[i] < self.max_scan_range*range_max:

                        if(j < len(l.path)-1):
                          self.logodds[index] += self.pfree - self.prior

                        else:
                          self.logodds[index] += self.pocc - self.prior

                        #else:
                        #  self.logodds[index]+=self.pfree
                        
                        if self.logodds[index] > self.max_logodd:
                          self.logodds[index] = self.max_logodd

                        elif self.logodds[index] < -self.max_logodd:
                          self.logodds[index] = -self.max_logodd

                        if self.logodds[index] > self.max_logodd_belief:
                          self.localmap[index] = 100
                       
                        else:
                          self.localmap[index] = 0
 
                        #self.localmap[self.origin]=100.0
   
                #print(self.logodds[index])












