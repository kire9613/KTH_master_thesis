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
import tf

# ROS messages
from geometry_msgs.msg import Point, Pose, PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path

# SVEA
from svea_msgs.msg import VehicleState

# OTHER
from matrix_astar import AStarPlanner

# PIL
try:
    from PIL import Image
except ImportError:
    print("You do not have PIL/Pillow installed")

## COLLISION NODE PARAMS ######################################################
update_rate = 5
width=721 
height=880

unknown_space = np.int8(-1)
free_space = np.int8(0)
c_space = np.int8(128)
intersected_obs = -np.int8(64)
occupied_space = np.int8(254)
start_spot = -np.int8(150)
goal_spot = -np.int8(170)
bounding_box = -np.int8(190)

img_file_name = os.path.dirname(os.path.abspath(__file__)) + os.sep + "problem.png"
unknown_space_rgb = (128, 128, 128)  # Grey
free_space_rgb = (255, 255, 255)     # White
c_space_rgb = (255, 0, 0)            # Red
occupied_space_rgb = (255, 255, 0)   # Yellow
start_spot_rgb = (0, 0, 255)		 # Blue
goal_spot_rgb = (0, 255, 0) 		 # Green
other_rgb = (0, 255, 255) 		 # Green
###############################################################################

class Node:
	"""
	asd
	"""
	def __init__(self):

		rospy.init_node('collision_node')

		self.solution_pub = rospy.Publisher('trajectory_updates', Path, queue_size=1, latch=True)

		self.problem_sub = rospy.Subscriber('/pickled_map', OccupancyGrid, self.callback)

		self.problem = OccupancyGrid()

	def callback(self, map):
		self.problem = map
		problem_vector = np.asarray(self.problem.data)
		planner = AStarPlanner(problem_vector)
		x_list, y_list = planner.planning()
		self.solution_pub.publish(lists_to_path(x_list, y_list))

#		toimg = np.asarray(self.problem.data)
#		toimg = toimg.reshape(height, width)
#		for i in range(len(x_list)):
#			toimg[x_list[i],y_list[i]] = -np.int8(200)
#		toimg = np.transpose(toimg)
#		toimg = toimg.reshape(-1)
#		map_image = Image.new('RGB', (width, height))
#		map_data_rgb = map_to_rgb(toimg)
#		map_image.putdata(map_data_rgb)
#		map_image.save(img_file_name)		

	def run(self):

		rate = rospy.Rate(update_rate)

		while not rospy.is_shutdown():
			rate.sleep()

		rospy.spin()

def grid_to_irl(x, y):
	resolution = 0.05
	x_org_cal = 17.5 #23.876441
	y_org_cal = 22#16.581444 #17.5
	yaw_cal = radians(12)

	# Compensate for map twist
	x = cos(yaw_cal)*x - sin(yaw_cal)*y
	y = sin(yaw_cal)*x + cos(yaw_cal)*y

	# Shift origin
	x = x*resolution - x_org_cal
	y = y*resolution - y_org_cal

	norm = sqrt(880**2 +  720**2)
	x_ref = x + 2*((x*880**2 + y*720*880)/norm**2 - x)
	y_ref = y + 2*((x*880*720 + y*720**2)/norm**2 - y)

	return x_ref, y_ref

def lists_to_path(x_list, y_list):
	path = Path()
	path.header.stamp = rospy.Time.now()
	path.header.frame_id = 'map'
	path.poses = lists_to_pose_stampeds(x_list, y_list)
	return path

def lists_to_pose_stampeds(x_list, y_list, yaw_list=None, t_list=None):
	poses = []
	for i in range(len(x_list)):
		x, y = grid_to_irl(x_list[i], y_list[i])
		print(x)
		print(y)
		curr_pose = PoseStamped()
		curr_pose.header.frame_id = 'map'
		curr_pose.pose.position.x = x
		curr_pose.pose.position.y = y

		if not yaw_list is None:
			yaw = yaw_list[i]
			quat = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
			curr_pose.pose.orientation.x = quat[0]
			curr_pose.pose.orientation.y = quat[1]
			curr_pose.pose.orientation.z = quat[2]
			curr_pose.pose.orientation.w = quat[3]

		if not t_list is None:
			t = t_list[i]
			curr_pose.header.stamp = rospy.Time(secs = t)
		else:
			curr_pose.header.stamp = rospy.Time.now()

		poses.append(curr_pose)
	return poses

def save_as_img(data):
	map_image = Image.new('RGB', (width, height))
	map_data_rgb = map_to_rgb(data)
	map_image.putdata(map_data_rgb)
	map_image.save(img_file_name)

def map_to_rgb(map_data):
	map_data_rgb = [unknown_space_rgb] * len(map_data)
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
		elif map_data[i] == start_spot:
			map_data_rgb[i] = start_spot_rgb
		elif map_data[i] == goal_spot:
			map_data_rgb[i] = goal_spot_rgb
		else:
			map_data_rgb[i] = other_rgb

	return map_data_rgb

if __name__ == '__main__':
	try:
		node = Node()
		node.run()
	except rospy.ROSInterruptException:
		pass