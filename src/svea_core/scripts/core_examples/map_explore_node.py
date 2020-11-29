#!/usr/bin/env python

"""
    @Adapted from code by: Daniel Duberg (dduberg@kth.se)
	@by Johan Hedin Team1
"""

# Python standard library
import math
import rospy
import numpy as np
from math import sin, cos, atan2, fabs, radians, sqrt, ceil, floor
import os
import pickle

# ROS
import rospy
import tf

# ROS messages
from geometry_msgs.msg import Point, Pose, Quaternion
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path

# SVEA
from svea_msgs.msg import VehicleState

## MAP EXPLORER PARAMS ########################################################
update_rate = 5 # [Hz]
lightweight = True

frame_id="map" 
resolution=0.05 # The map resolution [m/cell]
width=879
height=171

x_org=0
y_org=0
yaw_org= radians(0)

unknown_space = np.int8(-1) #grey
free_space = np.int8(0) #white
c_space = np.int8(25) #grey
occupied_space = np.int8(100) #black
start_val = np.int8(120) #green
#path_val = -np.int8(2) #yellow
goal_val = -np.int8(120) #red
bounding_box = -np.int8(10) #yellow

dirname = os.path.dirname(__file__)
svea_core = os.path.join(dirname, '../../')
map_name = "problem_map_occ" # change this for different maps
file_path = svea_core + 'resources/maps/' + map_name + ".pickle"
map_name2 = "problem_map_np" # change this for different maps
file_path2 = svea_core + 'resources/maps/' + map_name2 + ".pickle"
###############################################################################

class Node:
	"""
	asd
	"""
	def __init__(self):

		rospy.init_node('explore_node')
		
		self.map_pub = rospy.Publisher('explored_map', OccupancyGrid, queue_size=1, latch=True)
		self.problem_pub = rospy.Publisher('problems', OccupancyGrid, queue_size=1, latch=False)

		self.state_sub = rospy.Subscriber('/state', VehicleState, self.callback_state)
		self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.callback_scan)
		self.path_sub = rospy.Subscriber('/path_plan', Path, self.callback_path)
		
		self.scan = LaserScan()
		self.state = VehicleState()

		self.mapper = MapExplore()

		self.path_lookup = np.zeros((width,height))
		self.index_lookup = np.zeros((width,height))
		self.coord_lookup = None
		
		self.rate_timeout = rospy.Rate(1)

	def callback_state(self, state):
		self.state = state
		
	def callback_scan(self, scan):
		self.scan = scan
	
	def callback_path(self, path):
		self.path_lookup = np.zeros((width,height))
		self.index_lookup = np.zeros((width,height))
		self.coord_lookup = np.zeros((len(path.poses),2))

		for i in range(len(path.poses) - 2):
			x_start = path.poses[i].pose.position.x 
			y_start = path.poses[i].pose.position.y
			x_end = path.poses[i+1].pose.position.x 
			y_end = path.poses[i+1].pose.position.y

			x_s = int(x_start/resolution)
			y_s = int(y_start/resolution)
			x_e = int(x_end/resolution)
			y_e = int(y_end/resolution)
			self.coord_lookup[i,0] = x_s
			self.coord_lookup[i,1] = y_s

			start = (x_s, y_s)
			end = (x_e,y_e)

			ray = raytrace(start, end)
			
			for cell in ray:
				#self.mapper.add_to_map(cell[0],cell[1],path_val)
				self.path_lookup[cell] = 1
				self.index_lookup[cell] = i

				for r in range(1, 1):
					t = 0
					while t <= 2*np.pi:                        
						a = cell[0] + r*cos(t)
						b = cell[1] + r*sin(t)
						a = int(a)
						b = int(b)
						if is_in_bounds(a,b):
							self.path_lookup[(a,b)] = 1
							self.index_lookup[(a,b)] = i
						t = t + np.pi/32
				
			self.path_lookup[end] = 1
			self.index_lookup[end] = len(path.poses) - 1

	def run(self):
		rate = rospy.Rate(update_rate)

		while not rospy.is_shutdown():
			self.mapper.update_map(self.state, self.scan, self.path_lookup, self.index_lookup, self.coord_lookup)
			self.map_pub.publish(self.mapper.map)

			if self.mapper.detection == True:
				"""
				pic = self.mapper.map
				
				np_pic = np.asarray(self.mapper.map.data)
				
				with open(file_path, 'wb') as f:
					pickle.dump(pic, f)
				
				with open(file_path2, 'wb') as f2:
					np.save(f2,np_pic)
				"""
				self.problem_pub.publish(self.mapper.map)
				self.rate_timeout.sleep()

			self.mapper.reset_map()
			rate.sleep()
		
		rospy.spin()

class MapExplore:
	"""
	Store and update a local costmap
	"""
	def __init__(self):
		"""
		"""
		self.map = OccupancyGrid()
		# Fill in the header
		self.map.header.stamp = rospy.Time.now()
		self.map.header.frame_id = frame_id
		# Fill in the info
		self.map.info.resolution = resolution
		self.map.info.width = width
		self.map.info.height = height
		# Fill in the map data
		self.map_matrix = np.full((height, width), unknown_space, dtype=np.int8)
		self.map.data = self.map_matrix.reshape(-1) # (self.__map.size)

		self.detection = False
		
		#self.listener = tf.TransformListener()

	def add_to_map(self, x, y, value):
		"""
		adds value to index (x, y) in map if index is in bounds 
		and 
		returns weather (x, y) is inside map or not.
		"""
		if is_in_bounds(x, y):
			self.map_matrix[y, x] = value
			return True
		else:
			return False

	def update_map(self, state, scan, path_lookup, index_lookup, coord_lookup):
		"""
		Updates the grid_map with the data from the laser scan and the pose.
		:type scan: LaserScan
		"""
		self.detection = False

		x_grid = state.x/resolution
		y_grid = state.y/resolution

		obstacles = []
		i = 0    
		for k in scan.ranges:
			if k > scan.range_min and k < scan.range_max:
				x = k*cos(scan.angle_min + i*scan.angle_increment)
				y = k*sin(scan.angle_min + i*scan.angle_increment)
				obs_x = (cos(state.yaw)*x - sin(state.yaw)*y)/resolution + x_grid
				obs_y = (sin(state.yaw)*x + cos(state.yaw)*y)/resolution + y_grid
				
				obs = (int(obs_x),int(obs_y))
				obstacles.append(obs)
			else:
				i = i + 1
				continue
					
			start = (int(x_grid), int(y_grid))
			ray = raytrace(start, obs)
			for d in ray:
				self.add_to_map(d[0], d[1], free_space) 
			i = i + 1

		for c in obstacles:
			self.add_to_map(c[0], c[1], occupied_space)

		intersected = []
		for c in obstacles:
			if is_in_bounds(c[0],c[1]):
				if path_lookup[c] == 1:
					print(c[0]*0.05, c[1]*0.05)
					print("Intersected!")
					intersected.append(c)

		if len(intersected) >= 1:
			print("Intersected >= 1")
			
			obs_ind = int(index_lookup[intersected[0]])
			print(obs_ind)
			closest_x = int(coord_lookup[obs_ind - 5,0])
			closest_y = int(coord_lookup[obs_ind - 5,1])
			furthest_x = int(coord_lookup[obs_ind + 5,0])
			furthest_y = int(coord_lookup[obs_ind + 5,1])

			print(closest_x, closest_y)
			print(furthest_x, furthest_y)
			self.add_to_map(closest_x, closest_y, start_val)
			self.add_to_map(furthest_x, furthest_y, goal_val)
			self.detection = True
		else:
			self.detection = False
			
		self.map.data = self.map_matrix.reshape(-1)

	def reset_map(self):
		self.map_matrix = np.full((height, width), unknown_space, dtype=np.int8)
		self.map.data = self.map_matrix.reshape(-1)
		self.detection = False
		
def is_in_bounds(x, y):
	"""Returns weather (x, y) is inside grid_map or not."""
	if x >= 0 and x < width:
		if y >= 0 and y < height:
			return True
	else:
		print("False: ({0},{1})".format(x,y))
		return False

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
	i = 0
	while i < int(n):
		traversed.append((int(x), int(y)))
		if error > 0:
			x += x_inc
			error -= dy
		else:
			if error == 0:
				traversed.append((int(x + x_inc), int(y)))
			y += y_inc
			error += dx
		i += 1
	return traversed

if __name__ == '__main__':
	try:
		node = Node()
		node.run()
	except rospy.ROSInterruptException:
		pass