"""
utils for collision check
@author: huiming zhou
"""

import math
import numpy as np
import os
import sys

from scipy.spatial import ConvexHull

class Utils:
    def __init__(self, obstacles, grid_data):

        self.delta = 0.5
	self.obstacles = obstacles
        self.grid_data = grid_data

        self.resolution = 0.05
        self.origin = np.array([-30.55, -11.4])
	
	self.A = []
	self.B = []

	for obstacle_edges in obstacles:
	  a, b = self._compute_convex_hull_equations(obstacle_edges)
	  self.A.append(a)
	  self.B.append(b)

    def is_collision(self, start, end):
        if self.is_inside_obs(start) or self.is_inside_obs(end):
            return True

        traj_vec = np.linspace([start.x, start.y], [end.x, end.y], 10)

	traj_vec[:,0][traj_vec[:,0] > 32] = 32
        traj_vec[:,1][traj_vec[:,1] > 16.5] = 16.5 

        traj_vec[:,0][traj_vec[:,0] < -30] = -30 
        traj_vec[:,1][traj_vec[:,1] < -11] = -11
    
        traj_idx = self.array_to_index(traj_vec)

        for i in traj_idx:
          if self.grid_data[i[0], i[1]] == 100: #or self.grid_data[i[0]-1, i[1]] == 100 or self.grid_data[i[0], i[1]-1] == 100 or #self.grid_data[i[0]+1, i[1]] == 100 or self.grid_data[i[0], i[1]+1] == 100 or self.grid_data[i[0]-1, i[1]-1] == 100 or self.grid_data[i[0]+1, #i[1]+1] == 100 or self.grid_data[i[0]-1, i[1]+1] == 100 or self.grid_data[i[0]+1, i[1]-1] == 100:
            return True

        return False

    def is_inside_obs(self, node):
        
      node = np.array([node.x, node.y])

      for i in range(len(self.A)):
        if all(np.dot(self.A[i], node) <= self.B[i]):
          return True
      
      return False

    @staticmethod
    def _compute_convex_hull_equations(edge_points):

        hull = ConvexHull(edge_points)
        equations = hull.equations
	#print(equations)

        # Negate b to use the conventional representation Ax <= b
        A = equations[:, 0:2] #alla rader, 1:a o 2:a kolumnen
        b = -equations[:, 2] #alla rader, 3:e kolumnen

	eps = 0.1*np.ones(len(b))
        b = b + eps

        return A, b

    def array_to_index(self, array):

        self.origin_array = np.ones([np.size(array, 0), 2])
        self.origin_array[:, 0] = self.origin_array[:, 0] * self.origin[0]
        self.origin_array[:, 1] = self.origin_array[:, 1] * self.origin[1]

        norm_array = array - self.origin_array

        index = (np.fix(norm_array*1/self.resolution)) 

        index = index.astype(int)

        return index

