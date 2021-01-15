import math
import numpy as np
import os
import sys

from scipy.spatial import ConvexHull

class Utils:
    def __init__(self, obstacles, grid_data):

	self.obstacles = obstacles # edge points of known obstacles
        self.grid_data = grid_data #occupancy grid

        self.resolution = 0.05
        self.origin = np.array([-30.55, -11.4])
	
	self.A = []
	self.B = []

        # convert obstacle edges for known convex obstacles to equations in form of Ax <= b
	for obstacle_edges in obstacles:
	  a, b = self._compute_convex_hull_equations(obstacle_edges)
	  self.A.append(a)
	  self.B.append(b)

    def is_collision(self, start, end):

        # check if start or end node is inside an known obstacle
        if self.is_inside_obs(start) or self.is_inside_obs(end):
            return True

        # create a line segment from the start and end node
        traj_vec = np.linspace([start.x, start.y], [end.x, end.y], 10)

        # make sure the line stays within the map
	traj_vec[:,0][traj_vec[:,0] > 32] = 32
        traj_vec[:,1][traj_vec[:,1] > 16.5] = 16.5 

        traj_vec[:,0][traj_vec[:,0] < -30] = -30 
        traj_vec[:,1][traj_vec[:,1] < -11] = -11
    
        traj_idx = self.array_to_index(traj_vec)

        # check if the line goes through an obstacle in the occupancy grid. Takes unknown obstacles into account. 
        for i in traj_idx:
          if self.grid_data[i[0], i[1]] == 100:
            return True

        return False

    def is_inside_obs(self, node):

      "Check if a node's coordinates is within the limits of a convex set generated from the known obstacles."
        
      node = np.array([node.x, node.y])

      for i in range(len(self.A)):
        if all(np.dot(self.A[i], node) <= self.B[i]):
          return True
      
      return False

    @staticmethod
    def _compute_convex_hull_equations(edge_points):

        "Given a set of edge points of a convex polyhedron, it returns the limits within that set in form of Ax<=b."

        hull = ConvexHull(edge_points)
        equations = hull.equations

        # Negate b to use the conventional representation Ax <= b
        A = equations[:, 0:2]
        b = -equations[:, 2]

        #Added margin
	eps = 0.1*np.ones(len(b))
        b = b + eps

        return A, b

    def array_to_index(self, array):

        "Given a set of coordinates in an array, it returns the corresponding indeces in the occupancy grid."

        self.origin_array = np.ones([np.size(array, 0), 2])
        self.origin_array[:, 0] = self.origin_array[:, 0] * self.origin[0]
        self.origin_array[:, 1] = self.origin_array[:, 1] * self.origin[1]

        norm_array = array - self.origin_array

        index = (np.fix(norm_array*1/self.resolution)) 

        index = index.astype(int)

        return index

