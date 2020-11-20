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

frame_id="map" 
resolution=0.05 # The map resolution [m/cell]
width=721 
height=880
radius = 1
x_org=-17.581444
y_org=-22.876441
yaw_org= radians(0)

unknown_space = np.int8(-1)
free_space = np.int8(0)
c_space = np.int8(128)
intersected_obs = -np.int8(64)
occupied_space = np.int8(254)
start_spot = -np.int8(150)
goal_spot = -np.int8(170)
bounding_box = -np.int8(190)

dirname = os.path.dirname(__file__)
svea_core = os.path.join(dirname, '../../')
map_name = "problem_map_occ" # change this for different maps
file_path = svea_core + 'resources/maps/' + map_name + ".pickle"
map_name2 = "problem_map_np" # change this for different maps
file_path2 = svea_core + 'resources/maps/' + map_name2 + ".pickle"
###############################################################################

path_lookup = np.zeros((height,width))

class Node:
	"""
	asd
	"""
	def __init__(self):

		rospy.init_node('explore_node')
		
		self.map_pub = rospy.Publisher('explored_map', OccupancyGrid, queue_size=1, latch=True)
		self.problem_pub = rospy.Publisher('problems', OccupancyGrid, queue_size=1, latch=True)

		self.state_sub = rospy.Subscriber('/state', VehicleState, self.callback_state)
		self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.callback_scan)
		self.path_sub = rospy.Subscriber('/path_plan', Path, self.callback_path)
		
		self.scan = LaserScan()
		self.state = VehicleState()

		self.mapper = MapExplore()

	def callback_state(self, state):
		self.state = state
		
	def callback_scan(self, scan):
		self.scan = scan
	
	def callback_path(self, path):
		for i in range(len(path.poses) - 2):
			x_start, y_start = path_to_grid(path.poses[i].pose.position.x,
									path.poses[i].pose.position.y)
			x_end, y_end = path_to_grid(path.poses[i+1].pose.position.x,
									path.poses[i+1].pose.position.y)

			start = (x_start, y_start)
			end = (x_end, y_end)
			ray = raytrace(start, end)
			for cell in ray:
				path_lookup[cell] = 1
				for r in range(1, radius + 2):
					t = 0
					while t <= 2*np.pi:                        
						a = cell[0] + r*cos(t)
						b = cell[1] + r*sin(t)
						a = int(a)
						b = int(b)
						if is_in_bounds(a,b):
							path_lookup[(a,b)] = 1
						t = t + np.pi/32
			path_lookup[end] = 1

	def run(self):
		rate = rospy.Rate(update_rate)

		while not rospy.is_shutdown():
			self.mapper.update_map(self.state, self.scan)
			self.mapper.inflate_map()
			self.map_pub.publish(self.mapper.map)
			if self.mapper.detection == True:
				print("Hej")
				pic = self.mapper.map
				np_pic = np.asarray(self.mapper.map.data)
				with open(file_path, 'wb') as f:
					pickle.dump(pic, f)
				
				with open(file_path2, 'wb') as f2:
					np.save(f2,np_pic)

				self.problem_pub.publish(self.mapper.map)
			rate.sleep()
		
		rospy.spin()

class ExploredMap:
    def __init__(self):
        self.map = OccupancyGrid()
        self.subs = rospy.Subscriber("/explored_map", OccupancyGrid, self.callback)

    def callback(self, data):
        """
        Type :sensor_msgs.msg LaserScan:
        """
        self.map = data	

class MapExplore:
	"""
	Store and update a local costmap
	"""
	def __init__(self):
		"""
		hej
		"""
		self.map = OccupancyGrid()
		# Fill in the header
		self.map.header.stamp = rospy.Time.now()
		self.map.header.frame_id = frame_id
		# Fill in the info
		self.map.info.resolution = resolution
		self.map.info.width = width
		self.map.info.height = height
		
		#world pose of map (0,0) [m, m, rad]
		q = tf.transformations.quaternion_from_euler(0, 0, yaw_org)
		self.map.info.origin = Pose(Point(x_org, y_org, 0),
								Quaternion(q[0], q[1], q[2], q[3]))
		# Fill in the map data
		self.map_matrix = np.full((height, width), unknown_space, dtype=np.int8)
		self.map.data = self.map_matrix.reshape(-1) # (self.__map.size)

		self.detection = False
		
		self.listener = tf.TransformListener()

	def add_to_map_reflected(self, x, y, value):
		"""
		adds value to index (x, y) in map if index is in bounds 
		and 
		returns weather (x, y) is inside map or not.
		"""
		norm = sqrt(880**2 +  720**2)
		x_ref = x + 2*((x*880**2 + y*720*880)/norm**2 - x)
		y_ref = y + 2*((x*880*720 + y*720**2)/norm**2 - y)

		if is_in_bounds(x_ref, y_ref):
			self.map_matrix[int(x_ref), int(y_ref)] = value
			return True
		else:
			return False

	def add_to_map(self, x, y, value):
		"""
		adds value to index (x, y) in map if index is in bounds 
		and 
		returns weather (x, y) is inside map or not.
		"""
		if is_in_bounds(x, y):
			self.map_matrix[int(x), int(y)] = value
			return True
		else:
			return False

	def update_map(self, state, scan):
		"""
		Updates the grid_map with the data from the laser scan and the pose.
		:type scan: LaserScan
		"""
		self.detection = False
		
		(trans, rot) = self.listener.lookupTransform('/scan','map',rospy.Time(0))

		x_pose, y_pose, yaw = state_to_grid(state.x, state.y, state.yaw)

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
				obs_x = (cos(yaw)*x - sin(yaw)*y)/resolution + x_pose
				obs_y = (sin(yaw)*x + cos(yaw)*y)/resolution + y_pose
				obs = (int(obs_x),int(obs_y))
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

			start = (x_pose, y_pose)
			ray = raytrace(start, obs)
			for d in ray:
				self.add_to_map_reflected(d[0], d[1], free_space) 
				if d[0] < min_x:     
					min_x = d[0]
				if d[1] < min_y:     
					min_y = d[1]
				if d[0] > max_x:
					max_x = d[0]
				if d[1] > max_y:
					max_y = d[1]
			i = i + 1

		count = 0
		for c in obstacles:
			if is_in_bounds(c[0],c[1]):
				if path_lookup[c] == 1 and count > 4:
					lst = []
					self.add_to_map_reflected(c[0], c[1], intersected_obs)
					t = 0
					while t <= 2*np.pi:                        
						a = c[0] + 60*cos(t)
						b = c[1] + 60*sin(t)
						a = int(a)
						b = int(b)
						if is_in_bounds(a,b):
							if path_lookup[(a,b)] == 1:
								lst.append((a,b))
							self.add_to_map_reflected(a, b, bounding_box)
						t = t + np.pi/64
					
					#furthest = None
					srt = np.squeeze(np.zeros((1,len(lst))))
					for i, entry in enumerate(lst):
						srt[i] = np.sqrt((start[0] - entry[0])^2 + (start[1] - entry[1])^2)
					indices = np.argsort(srt)
					closest = lst[int(indices[0])]
					if len(lst) > 3:
						furthest = lst[int(indices[2])]
						self.add_to_map_reflected(closest[0], closest[1], start_spot)
						self.add_to_map_reflected(furthest[0], furthest[1], goal_spot)
						self.detection = True

				count = count + 1

			self.add_to_map_reflected(c[0], c[1], occupied_space)

		self.map.data = self.map_matrix.reshape(-1)

	def inflate_map(self):
		"""
		Inflate the map with c_space assuming the robot
		has a radius of radius.
		"""
		for i in range(0, height):
			for j in range(0, width):
				if self.map_matrix[i,j] == occupied_space:
					t = 0
					for r in range(1, radius + 2):
						t = 0
						while t <= 2*np.pi:                        
							a = i + r*cos(t)
							b = j + r*sin(t)
							a = int(a)
							b = int(b)
							if is_in_bounds(a,b) and self.map_matrix[a,b] != occupied_space:
								self.add_to_map(a, b, c_space)
							t = t + np.pi/32

		self.map.data = self.map_matrix.reshape(-1)

def is_in_bounds(x, y):
	"""Returns weather (x, y) is inside grid_map or not."""
	if x >= 0 and x < height:
		if y >= 0 and y < width:
			return True
	else:
		return False

def path_to_grid(x,y):
	x_org_cal = 17.581444#17.5 #23.876441
	y_org_cal = 22.876441#23.876441 #16.581444 #17.5
	yaw_cal = radians(-11.3)

	# Shift origin
	x = (x + x_org_cal)/resolution
	y = (y + y_org_cal)/resolution

	# Compensate for map twist
	x = cos(yaw_cal)*x - sin(yaw_cal)*y
	y = sin(yaw_cal)*x + cos(yaw_cal)*y

	x = int(x)
	y = int(y)
	return (x, y)

def state_to_grid(x_pose, y_pose, yaw_pose):
	x_org_cal = 22
	y_org_cal = 19
	yaw_cal = radians(-12)

	#x = x_pose
	#y = y_pose
	#yaw = yaw_pose

	yaw = yaw_pose + yaw_cal
	x = cos(yaw_cal)*x_pose - sin(yaw_cal)*y_pose
	y = sin(yaw_cal)*x_pose + cos(yaw_cal)*y_pose

	x = (x + x_org_cal)/resolution 
	y = (y + y_org_cal)/resolution

	x = int(x)
	y = int(y)
	yaw = yaw_pose + yaw_cal

	return x, y, yaw

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