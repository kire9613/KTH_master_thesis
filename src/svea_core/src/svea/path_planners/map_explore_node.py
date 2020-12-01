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
from map_msgs.msg import OccupancyGridUpdate
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

oob_delimiter = max(width,height) + 1

x_org=0
y_org=0
yaw_org= radians(0)

unknown_space = np.int8(50) #grey
free_space = -np.int8(25) #white
c_space = np.int8(25) #grey
occupied_space = np.int8(20) #black
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
		self.problem_pub = rospy.Publisher('problem_map', OccupancyGridUpdate, queue_size=1, latch=False)

		self.state_sub = rospy.Subscriber('/state', VehicleState, self.callback_state)
		self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.callback_scan)
		self.path_sub = rospy.Subscriber('/path_plan', Path, self.callback_path)
		
		self.scan = LaserScan()
		self.state = VehicleState()

		self.mapper = MapExplore()

		self.path_lookup = np.zeros((width,height))
		self.index_lookup = np.zeros((width,height))
		self.path = Path()

		self.rate_timeout = rospy.Rate(1)

	def callback_state(self, state):
		self.state = state
		
	def callback_scan(self, scan):
		self.scan = scan
	
	def callback_path(self, path):
		self.path = path
		self.path_lookup = np.zeros((width,height), dtype=np.uint8)
		self.index_lookup = np.zeros((width,height), dtype=np.uint16)

		for i in range(len(path.poses) - 2):
			x_start = self.path.poses[i].pose.position.x 
			y_start = self.path.poses[i].pose.position.y
			x_end = self.path.poses[i+1].pose.position.x 
			y_end = self.path.poses[i+1].pose.position.y

			int_x_start = np.int16(x_start/resolution)
			int_y_start = np.int16(y_start/resolution)
			int_x_end = np.int16(x_end/resolution)
			int_y_end = np.int16(y_end/resolution)

			ray = raytrace(int_x_start, int_y_start, int_x_end, int_y_end)
			
			l = 0
			while ray[l,0] != oob_delimiter:
				#self.mapper.add_to_map(cell[0],cell[1],path_val)
				self.path_lookup[ray[l,0], ray[l,1]] = 1
				self.index_lookup[ray[l,0], ray[l,1]] = i

				for r in range(1, 2):
					t = 0
					while t <= 2*np.pi:                        
						a = ray[l,0] + r*cos(t)
						b = ray[l,1] + r*sin(t)
						a = int(a)
						b = int(b)
						if is_in_bounds(a,b):
							self.path_lookup[(a,b)] = 1
							self.index_lookup[(a,b)] = i
						t = t + np.pi/32
				l += 1
				
			self.path_lookup[int_x_end, int_y_end] = 1
			self.index_lookup[int_x_end, int_y_end] = len(path.poses) - 1

	def run(self):
		rate = rospy.Rate(update_rate)

		while not rospy.is_shutdown():
			self.mapper.update_map(self.state, self.scan)
			self.map_pub.publish(self.mapper.map)

			map_slice = self.mapper.collision_check(self.path_lookup, self.index_lookup, self.path)

			if map_slice != None:
				self.problem_pub.publish(map_slice)
				self.rate_timeout.sleep()
				"""
				pic = self.mapper.map
				
				np_pic = np.asarray(self.mapper.map.data)
				
				with open(file_path, 'wb') as f:
					pickle.dump(pic, f)leader
				
				with open(file_path2, 'wb') as f2:
					np.save(f2,np_pic)
				"""

			#self.mapper.reset_map()
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
		q = tf.transformations.quaternion_from_euler(0, 0, yaw_org)
		self.map.info.origin = Pose(Point(x_org, y_org, 0),
								Quaternion(q[0], q[1], q[2], q[3]))
		# Fill in the map data
		self.map_matrix = np.full((height, width), unknown_space, dtype=np.int8)
		self.map.data = self.map_matrix.reshape(-1) # (self.__map.size)		
		
		#self.listener = tf.TransformListener()

	def add_to_map(self, x, y, value):
		"""
		adds value to index (x, y) in map if index is in bounds 
		and 
		returns weather (x, y) is inside map or not.
		"""
		if is_in_bounds(x, y):
			curr_val = self.map_matrix[y, x]
			if curr_val + value < 0:
				self.map_matrix[y, x] = 0
			elif curr_val + value < 100:
				self.map_matrix[y, x] = curr_val + value
			return True
		else:
			return False

	def update_map(self, state, scan):
		"""
		Updates the grid_map with the data from the laser scan and the pose.
		:type scan: LaserScan
		"""

		x_grid = state.x/resolution
		y_grid = state.y/resolution
		int_x_grid = np.int16(state.x/resolution)
		int_y_grid = np.int16(state.y/resolution)

		obstacles = np.full((136,2), np.int16(oob_delimiter))
		i = 0
		j = 0
		for k in scan.ranges:
			if k > scan.range_min and k < scan.range_max:
				x = k*cos(scan.angle_min + i*scan.angle_increment)
				y = k*sin(scan.angle_min + i*scan.angle_increment)
				obs_x = (cos(state.yaw)*x - sin(state.yaw)*y)/resolution + x_grid
				obs_y = (sin(state.yaw)*x + cos(state.yaw)*y)/resolution + y_grid
				
				int_obs_x = np.int16(obs_x)
				int_obs_y = np.int16(obs_y)

				obstacles[j,0] = np.int16(obs_x)
				obstacles[j,1] = np.int16(obs_y)
				j += 1
			else:
				i = i + 1
				continue
					
			ray = raytrace(int_x_grid, int_y_grid, int_obs_x, int_obs_y)

			l = 0
			while ray[l,0] != oob_delimiter:
				self.add_to_map(ray[l,0], ray[l,1], free_space) 
				l += 1

			i = i + 1		
		
		j = 0
		while obstacles[j,0] != oob_delimiter:
			self.add_to_map(obstacles[j,0], obstacles[j,1], occupied_space)
			j += 1

		self.map.data = self.map_matrix.reshape(-1)
	
	def collision_check(self, path_lookup, index_lookup, path):

		map_slice = None

		plan_width = 120
		plan_height = 120

		matr = np.transpose(self.map_matrix)*path_lookup

		collisions = np.where(matr >= 75)

		if collisions[0].size != 0:
			orig_x = collisions[0][0]
			orig_y = collisions[1][0] 
			
			xmin = orig_x - plan_width/2
			xmax = orig_x + plan_width/2
			ymin = orig_y - plan_height/2
			ymax = orig_y + plan_height/2

			if xmin < 0:
				xmin = 0
			if xmax >= width:
				xmax = width
			if ymin < 0:
				ymin = 0
			if ymax >= height:
				ymax = height 

			final_width = xmax - xmin - 1
			final_height = ymax - ymin - 1

			map_matr = self.map_matrix[ymin:ymax - 1, xmin:xmax - 1]
		
			print(xmin, xmax)
			print(ymin, ymax)

			print(orig_x,orig_y)
			print("Intersected path")
			obs_ind = index_lookup[orig_x, orig_y]

			start_x = np.int16(path.poses[obs_ind - 5].pose.position.x/resolution)
			start_y = np.int16(path.poses[obs_ind - 5].pose.position.y/resolution)
			goal_x = np.int16(path.poses[obs_ind + 5].pose.position.x/resolution)
			goal_y = np.int16(path.poses[obs_ind + 5].pose.position.y/resolution)

			print(start_x, start_y)
			print(goal_x, goal_y)

			map_matr[start_y - ymin, start_x - xmin] = -np.int8(1)
			map_matr[goal_y - ymin, goal_x - xmin] = -np.int8(2)

			map_slice = OccupancyGridUpdate()
			map_slice.header.stamp = rospy.Time.now()
			map_slice.header.frame_id = frame_id
			map_slice.x = xmin
			map_slice.y = ymin
			map_slice.width = final_width
			map_slice.height = final_height
			map_slice.data = map_matr.reshape(-1)

		return map_slice

	def reset_map(self):
		self.map_matrix = np.full((height, width), unknown_space, dtype=np.int8)
		self.map.data = self.map_matrix.reshape(-1)
		
def is_in_bounds(x, y):
	"""Returns weather (x, y) is inside grid_map or not."""
	if x >= 0 and x < width:
		if y >= 0 and y < height:
			return True
	else:
		print("False: ({0},{1})".format(x,y))
		return False

def raytrace(start_x, start_y, end_x, end_y):
	"""Returns all cells in the grid map that has been traversed
	from start to end, including start and excluding end.
	"""
	x = start_x
	y = start_y
	dx = np.abs(end_x - start_x)
	dy = np.abs(end_y - start_y)
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
	traversed = np.full((n + 1,2), np.int16(oob_delimiter))
	i = 0
	while i < np.int16(n):
		traversed[i,0] = np.int16(x)
		traversed[i,1] = np.int16(y)
		if error > 0:
			x += x_inc
			error -= dy
		else:
			if error == 0:
				traversed[i,0] = np.int16(x + x_inc)
				traversed[i,1] = np.int16(y)
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