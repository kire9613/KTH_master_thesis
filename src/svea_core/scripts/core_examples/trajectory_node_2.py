#!/usr/bin/env python

"""
	@by Johan Hedin Team1
"""

# Python standard library
import math
import rospy
import numpy as np
from math import sin, cos, atan2, fabs, radians, sqrt, ceil, floor
import os

# ROS
import rospy

# ROS messages
from geometry_msgs.msg import Point, Pose, Quaternion 
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path

# SVEA
from svea_msgs.msg import VehicleState

# PIL
try:
    from PIL import Image, ImageDraw
except ImportError:
    print("You do not have PIL/Pillow installed")

## TRAJECTORY PUBLISHING NODE PARAMS ##########################################
update_rate = 5 # [Hz]

frame_id="map" 
resolution=0.05 # The map resolution [m/cell]
width=721 
height=880
radius = 1
x_org=-17.581444
y_org=-22.876441
yaw_org= radians(0)
x_org_cal = 17.5 #23.876441
x_org_cal_cells = x_org_cal/resolution
y_org_cal = 23.876441 #16.581444
y_org_cal_cells  = y_org_cal/resolution
yaw_cal = radians(-11.3)
default_value= np.int8(0)
free_space = np.int8(0)
occupied_space = np.int8(16)
c_space = np.int8(8)

img_file_name = os.path.dirname(os.path.abspath(__file__)) + os.sep + "trajectory.png"
free_space_rgb = (128, 128, 128)     # White
occupied_space_rgb = (255, 0, 0)     # Red
c_space_rgb = (255, 0, 0)            # Red
wrong_rgb = (0, 0, 255)              # Blue

###############################################################################

class Node:
	"""
	asd
	"""
	def __init__(self):

		rospy.init_node('trajectory_node')
		
		self.map_pub = rospy.Publisher('trajectory_on_map', OccupancyGrid, queue_size=1,latch=True)
		
		self.path_sub = rospy.Subscriber('/SVEA/path_plan', Path, self.callback)

		self.trajectory_map = TrajectoryMap()

	def callback(self, path):
		for i in range(len(path.poses) - 2):
			start = (path.poses[i].pose.position.x,path.poses[i].pose.position.y)
			end = (path.poses[i+1].pose.position.x,path.poses[i+1].pose.position.y)
			self.trajectory_map.update(start, end)
		self.trajectory_map.inflate_map()

	def run(self):

		rate = rospy.Rate(update_rate)

		while not rospy.is_shutdown():
			self.map_pub.publish(self.trajectory_map.map)
			rate.sleep()
		
		#self.trajectory_map.save()
		
		rospy.spin()

class TrajectoryMap:
	"""
	Store and update a local costmap
	"""
	def __init__(self):
		"""
		hej
		"""
		self.map_pub = rospy.Publisher('trajectory_on_map', OccupancyGrid, queue_size=1, latch=True)

		self.map = OccupancyGrid()
		self.map.header.stamp = rospy.Time.now()
		self.map.header.frame_id = frame_id
		self.map.info.resolution = resolution
		self.map.info.width = width
		self.map.info.height = height
	
		# world pose of map (0,0) [m, m, rad]
		q = quaternion_from_euler(0, 0, yaw_org)
		self.map.info.origin = Pose(Point(x_org, y_org, 0),
								Quaternion(q[0], q[1], q[2], q[3]))
		# Fill in the map data
		self.map_matrix = np.full((height, width), default_value, dtype=np.int8)
		self.map.data = self.map_matrix.reshape(-1) # (self.__map.size)

		self.map_image = Image.new('RGB', (width, height))
		self.draw_obj = ImageDraw.Draw(self.map_image)

	def update(self, start, end):
		(x, y) = start
		(xp, yp) = end

		# Shift origin
		x = x/resolution + x_org_cal_cells
		y = y/resolution + y_org_cal_cells

		# Compensate for map twist
		x = cos(yaw_cal)*x - sin(yaw_cal)*y
		y = sin(yaw_cal)*x + cos(yaw_cal)*y

		x = int(x)
		y = int(y)

		# Shift origin
		xp = xp/resolution + x_org_cal_cells
		yp = yp/resolution + y_org_cal_cells

		# Compensate for map twist
		xp = cos(yaw_cal)*xp - sin(yaw_cal)*yp
		yp = sin(yaw_cal)*xp + cos(yaw_cal)*yp

		xp = int(xp)
		yp = int(yp)

		ray = self.raytrace((x,y), (xp,yp))

		for d in ray:
			self.add_to_map_reflected(d[0], d[1], occupied_space)

	def add_to_map(self, x, y, value):
		"""
		adds value to index (x, y) in map if index is in bounds 
		and 
		returns weather (x, y) is inside map or not.
		"""
		if self.is_in_bounds(x, y):
			self.map_matrix[int(x), int(y)] = value
			draw(self.draw_obj, x, height - y, value)
			return True
		else:
			return False

	def add_to_map_reflected(self, x, y, value):
		"""
		adds value to index (x, y) in map if index is in bounds 
		and 
		returns weather (x, y) is inside map or not.
		"""
		norm = sqrt(880**2 +  720**2)
		x_ref = x + 2*((x*880**2 + y*720*880)/norm**2 - x)
		y_ref = y + 2*((x*880*720 + y*720**2)/norm**2 - y)
		if self.is_in_bounds(x_ref, y_ref):
			self.map_matrix[int(floor(x_ref)), int(floor(y_ref))] = value
			draw(self.draw_obj, x, height - y, value)
			return True
		else:
			return False

	def is_in_bounds(self, x, y):
		"""Returns weather (x, y) is inside grid_map or not."""
		if x >= 0 and x < height:
			if y >= 0 and y < width:
				return True
		else:
			return False

	def raytrace(self, start, end):
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
							if self.is_in_bounds(a,b) and self.map_matrix[a,b] != occupied_space:
								self.add_to_map(a, b, c_space)
							t = t + np.pi/32

		self.map.data = self.map_matrix.reshape(-1)

	def save(self):
		map_data_rgb = map_to_rgb(self.map.data)
		self.map_image.putdata(map_data_rgb)
		self.map_image.save(img_file_name)

def map_to_rgb(map_data):
	map_data_rgb = [wrong_rgb] * len(map_data)
	# Convert to RGB
	for i in range(0, len(map_data)):
		if map_data[i] == free_space:
			map_data_rgb[i] = free_space_rgb
		elif map_data[i] == occupied_space:
			map_data_rgb[i] = occupied_space_rgb
		else:
			# If there is something blue in the image
			# then it is wrong
			map_data_rgb[i] = wrong_rgb
			
	return map_data_rgb

def draw(draw_obj, x, y, value):
	if value == free_space:
		draw_obj.point((x,y), free_space_rgb)
	elif value == occupied_space:
		draw_obj.point((x,y), occupied_space_rgb)
	else:
		# If there is something blue in the image
		# then it is wrong
		draw_obj.point((x,y), wrong_rgb)

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

def write_yaml():
	# open file for writing 
	filename = os.path.dirname(os.path.abspath(__file__)) + os.sep + "trajectory2.yaml"
	file_out = open(filename, 'wb')

	# define PGM Header
	string = 'image: ' + filename + '\n' + 'resolution: ' \
			+ str(resolution) + '\n' \
			'origin: ' + '[' + str(-x_org) + ', ' \
			+ str(-y_org) + ', ' + str(0) + ']' + '\n' + 'negate: '\
			+ str(0) + '\n' + 'occupied_thresh: ' \
			+ str(0.65) + '\n' + 'free_thresh: ' \
			+ str(0.196) + '\n' + '\n'

	file_out.write(string)

if __name__ == '__main__':
	try:
		node = Node()
		write_yaml()
		node.run()
	except rospy.ROSInterruptException:
		pass