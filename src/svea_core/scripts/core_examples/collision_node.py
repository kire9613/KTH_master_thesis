#!/usr/bin/env python

"""
	@by Johan Hedin Team1
"""

# Python standard library
import math
import rospy
import numpy as np
from math import sin, cos, atan2, fabs, radians, sqrt, ceil, floor
from numpy.linalg import norm

import os

# ROS
import rospy

# ROS messages
from geometry_msgs.msg import Point, Pose, Quaternion
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

# SVEA
from svea_msgs.msg import VehicleState

# PIL
try:
    from PIL import Image, ImageDraw
except ImportError:
    print("You do not have PIL/Pillow installed")

## COLLISION NODE PARAMS ######################################################
update_rate = 10 # [Hz]
detection_range = 0.2*880
goal_radius = 0.6*880

frame_id="map" 
resolution=0.05 # The map resolution [m/cell]
width=721 
height=880
radius = 1
x_org=-17.581444
y_org=-22.876441
yaw_org= radians(0)
x_org_cal = 22
y_org_cal = 19
yaw_cal = radians(-12)
default_value=np.int8(-1)
unknown_space = np.int8(-1)
free_space = np.int8(0)
c_space = np.int8(128)
occupied_space = np.int8(254)

img_file_name = os.path.dirname(os.path.abspath(__file__)) + os.sep + "problem.png"
unknown_space_rgb = (128, 128, 128)  # Grey
free_space_rgb = (255, 255, 255)     # White
c_space_rgb = (255, 0, 0)            # Red
occupied_space_rgb = (255, 255, 0)   # Yellow
wrong_rgb = (0, 0, 255)              # Blue

###############################################################################

class Node:
	"""
	asd
	"""
	def __init__(self):

		rospy.init_node('collision_node')
		
		self.problem_pub = rospy.Publisher('collision_problem', OccupancyGrid, queue_size=1, latch=True)

		self.map_exp_sub = rospy.Subscriber('/explored_map', OccupancyGrid, self.callback)
		self.map_traj_sub = rospy.Subscriber('/trajectory_on_map', OccupancyGrid, self.callback_traj)
		self.state_sub = rospy.Subscriber('/SVEA/state', VehicleState, self.callback_state)

		self.map_traj = OccupancyGrid()
		self.state = VehicleState()

		self.detector = CollisionDetector()
		self.problem = PlanningProblem()

	def callback(self, map_exp):
		arr_exp = np.asarray(map_exp.data, dtype=np.int8)
		arr_traj = np.asarray(self.map_traj.data, dtype=np.int8)
		if arr_exp.shape == arr_traj.shape:
			detection, detection_data = self.detector.detect(arr_exp, arr_traj)
			if detection:
				self.problem.generate(detection_data) 
				try:
					self.problem_pub.publish(self.problem.map)
				except rospy.ROSSerializationException:
					pass

	def callback_traj(self, map_traj):
		self.map_traj = map_traj
	
	def callback_state(self, state):
		self.state = state

	def run(self):

		rate = rospy.Rate(update_rate)

		while not rospy.is_shutdown():
			rate.sleep()

		rospy.spin()

class Problems:
	def __init__(self):
		self.problem = OccupancyGrid()
		self.solved = True
		self.subs = rospy.Subscriber("/collision_problem", OccupancyGrid, self.callback)

	def callback(self, data):
		"""
		Type :sensor_msgs.msg LaserScan:
		"""
		self.problem = data
		self.solved = False

class CollisionDetector:
	"""
	Store map of explored map with trajectories intersecting obstacles
	"""
	def __init__(self):
		"""
		hej
		"""

	def detect(self, explored_map, trajectory_map):
		"""
		asd
		"""
		detection = False
		goal_index = None
		start_index = None
		obs_locations = []

		explored = explored_map
		trajectory = trajectory_map
		combined = explored - trajectory

		combined_matrix = combined.reshape(height,width)
		
		detection_map = np.where(combined == -10,1,0)
		detection_matrix = detection_map.reshape(height,width)

#		if np.sum(detection_map) > 2:
#			position = self.get_closest_cell(state)
#			for i in range(0, height):
#				for j in range(0, width):
#					if detection_matrix[i,j] is not 0:
#						obs_locations.append((i,j))
#			for obs in obs_locations:
#				if norm(obs - position) < detection_range:
#					detection = True
		if True:
			#save_as_img(overlayed)
#			problem_map = combined.reshape(height,width) 
#			problem_map[goal_index] = np.int8(128)
#			problem_map[start_index] = np.int8(128)
#			problem_map = problem_map.reshape(-1)
			return True, combined
		else:
			return False, combined

class PlanningProblem:
	"""
	Map with obstacles, start point and goal point with
	Obstacles = 
	Freespace = 
	Start point = 
	Goal point = 
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
		q = quaternion_from_euler(0, 0, yaw_org)
		self.map.info.origin = Pose(Point(x_org, y_org, 0),
								Quaternion(q[0], q[1], q[2], q[3]))
		# Fill in the map data
		self.map_matrix = np.full((height, width), default_value, dtype=np.int8)
		self.map.data = self.map_matrix.reshape(-1) # (self.__map.size)

	def generate(self, detection_data):
		self.map.data = detection_data

def save_as_img(data):
		map_image = Image.new('RGB', (width, height))
		map_data_rgb = map_to_rgb(data)
		map_image.putdata(map_data_rgb)
		map_image.save(img_file_name)

def map_to_rgb(map_data):
	map_data_rgb = [wrong_rgb] * len(map_data)
	# Convert to RGB
	for i in range(0, len(map_data)):
		if map_data[i] == unknown_space:
			map_data_rgb[i] = unknown_space_rgb
		elif map_data[i] == free_space:
			map_data_rgb[i] = free_space_rgb
		elif map_data[i] == c_space:
			map_data_rgb[i] = c_space_rgb
		elif map_data[i] == occupied_space:
			map_data_rgb[i] = occupied_space_rgb
		else:
			# If there is something blue in the image
			# then it is wrong
			map_data_rgb[i] = wrong_rgb
			
	return map_data_rgb

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

if __name__ == '__main__':
	try:
		node = Node()
		node.run()
	except rospy.ROSInterruptException:
		pass