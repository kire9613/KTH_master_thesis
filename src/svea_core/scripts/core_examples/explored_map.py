#!/usr/bin/env python

"""
    @original author: Daniel Duberg (dduberg@kth.se)
"""

# Python standard library
import math
import rospy
import numpy as np
from math import sin, cos, atan2, fabs

# ROS
import rospy
import message_filters

# ROS messages
from geometry_msgs.msg import Point, Pose, Quaternion
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate

## LOCAL COSTMAP PARAMS #######################################################
frame_id="map" 
resolution=1 # The map resolution [m/cell]
width=10 
height=10
map_origin_x=0
map_origin_y=0
map_origin_yaw=0
default_value=-1
unknown_space = -1
free_space = 0
c_space = 1
occupied_space = 2
radius = 1
#optional = optional
###############################################################################

def quaternion_from_euler(roll, pitch, yaw):
    q = [0]*4

    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)

    q[0] = cy * sr * cp - sy * cr * sp
    q[1] = cy * cr * sp + sy * sr * cp
    q[2] = sy * cr * cp - cy * sr * sp
    q[3] = cy * cr * cp + sy * sr * sp

    return q

def get_yaw(q):
    """Returns the Euler yaw from a quaternion.
    :type q: Quaternion
    """
    return atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

def raytrace(start, end):
	"""Returns all cells in the grid map that has been traversed
	from start to end, including start and excluding end.
	start = (x, y) grid map index
	end = (x, y) grid map index
	"""
	(start_x, start_y) = start
	(end_x, end_y) = end
	x = start_x
	y = start_y
	(dx, dy) = (fabs(end_x - start_x), fabs(end_y - start_y))
	n = dx + dy
	x_inc = 1
	if end_x <= start_x:
	    x_inc = -1
	y_inc = 1
	if end_y <= start_y:
	    y_inc = -1
	error = dx - dy
	dx *= 2
	dy *= 2
	traversed = []
	for i in range(0, int(n)):
	    traversed.append((int(x), int(y)))
	    if error > 0:
	        x += x_inc
	        error -= dy
	    else:
	        if error == 0:
	            traversed.append((int(x + x_inc), int(y)))
	        y += y_inc
	        error += dx
	return traversed

class ObstacleMap:
	"""
	Store and update a local costmap
	"""
	def __init__(self):
		"""
		hej
		"""        
		self.map = OccupancyGrid()
		# Fill in the header
		self.map.header.stamp = 0
		self.map.header.frame_id = frame_id
		# Fill in the info
		self.map.info.resolution = resolution
		self.map.info.width = width
		self.map.info.height = height
		
		#world pose of map (0,0) [m, m, rad]
		q = quaternion_from_euler(0, 0, map_origin_yaw)
		self.map.info.origin = Pose(Point(map_origin_x, map_origin_y, 0),
								Quaternion(q[0], q[1], q[2], q[3]))
		# Fill in the map data
		self.map_matrix = np.full((height, width), default_value, dtype=np.int8)
		self.map.data = self.map_matrix.reshape(-1) # (self.__map.size)

	def add_to_map(self, x, y, value):
		"""
		adds value to index (x, y) in map if index is in bounds 
		and 
		returns weather (x, y) is inside map or not.
		"""
		if self.is_in_bounds(x, y):
			self.map_matrix[x, y] = value
			return True
		return False

	def is_in_bounds(self, x, y):
		"""Returns weather (x, y) is inside grid_map or not."""
		if x >= 0 and x < width:
			if y >= 0 and y < height:
				return True
		return False

	def update_map(self, state, scan):
		"""
		Updates the grid_map with the data from the laser scan and the pose.
		:type scan: LaserScan
		"""
		fpos_x = (state.x - self.map.info.origin.position.x)/resolution 
		fpos_y = (state.y - self.map.info.origin.position.y)/resolution
		R = np.array([(cos(state.yaw),-sin(state.yaw),0),(sin(state.yaw),cos(state.yaw),0),(0,0,1)])           
		P = np.transpose(np.array([(fpos_x,fpos_y,0)]))        
		start = (int(fpos_x), int(fpos_y)) #For raytracing
		min_x = 299
		max_x = 0
		min_y = 299
		max_y = 0

		obstacles = []
		i = 0    
		for k in scan.ranges:
			if k > scan.range_min and k < scan.range_max:
				x = k*cos(scan.angle_min + i*scan.angle_increment)
				y = k*sin(scan.angle_min + i*scan.angle_increment)
				Q = np.transpose(np.array([(x,y,0)]))
				U = np.dot(R,Q)/resolution
				U = U + P
				obs = (int(U[0]),int(U[1]))
				obstacles.append(obs)
				if obs[0] < min_x:
					min_x = obs[0]
				if obs[1] < min_y:
					min_y = obs[1]
				if obs[0] > max_x:
					max_x = obs[0]
				if obs[1] > max_y:
					max_y = obs[1]
			else:
				i = i + 1
				continue
			ray = raytrace(start, obs)
			for d in ray:
				self.add_to_map(d[0], d[1], free_space) 
				if d[0] < min_x:     
					min_x = d[0]
				if d[1] < min_y:     
					min_y = d[1]
				if d[0] > max_x:
					max_x = d[0]
				if d[1] > max_y:
					max_y = d[1]
			i = i + 1

		for c in obstacles:
			self.add_to_map(c[0], c[1], occupied_space)

		self.map.data = self.map_matrix.reshape(-1)

		# Only get the part that has been updated
		update = OccupancyGridUpdate()
		update.x = min_x
		update.y = min_y
		update.width = max_x - min_x + 1
		update.height = max_y - min_y + 1
		# The map data inside the rectangle, in row-major order.
		update.data = self.map_matrix[min_x:max_x + 1, min_y:max_y + 1].reshape(-1)
		# part of the map that has been updated

		return update

	def inflate_map(self, grid_map):
		"""
		Inflate the map with c_space assuming the robot
		has a radius of radius.
		"""
		for i in range(0, 299):
			for j in range(0, 299):
				if self.map_matrix[i,j] == occupied_space:
					t = 0
					for r in range(1, radius + 1):
						t = 0
						while t <= 2*np.pi:                        
							a = i + r*cos(t)
							b = j + r*sin(t)
							a = int(a)
							b = int(b)
							if self.map_matrix[a,b] != occupied_space:
								self.add_to_map(a, b, c_space)
							t = t + np.pi/32

		self.map.data = self.map_matrix.reshape(-1)                    

