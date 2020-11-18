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
from nav_msgs.msg import OccupancyGrid

# SVEA
from svea_msgs.msg import VehicleState

# PIL
try:
    from PIL import Image
except ImportError:
    print("You do not have PIL/Pillow installed")

## COLLISION NODE PARAMS ######################################################
update_rate = 5
width=721 
height=880

default_value=-1
unknown_space = np.int8(-1)
free_space = np.int8(0)
c_space = np.int8(128)
intersected_obs = -np.int8(64)
occupied_space = np.int8(254)
start_spot = -np.int8(150)
goal_spot = -np.int8(170)

img_file_name = os.path.dirname(os.path.abspath(__file__)) + os.sep + "problem.png"
unknown_space_rgb = (128, 128, 128)  # Grey
free_space_rgb = (255, 255, 255)     # White
c_space_rgb = (255, 0, 0)            # Red
occupied_space_rgb = (255, 255, 0)   # Yellow
start_spot_rgb = (0, 0, 255)		 # Blue
goal_spot_rgb = (0, 255, 0) 		 # Green
###############################################################################

class Node:
	"""
	asd
	"""
	def __init__(self):

		rospy.init_node('collision_node')

		self.problem_sub = rospy.Subscriber('/problems', OccupancyGrid, self.callback)

		self.problem = OccupancyGrid()

	def callback(self, map):
		self.problem = map
		save_as_img(self.problem.data)

	def run(self):

		rate = rospy.Rate(update_rate)

		while not rospy.is_shutdown():
			rate.sleep()

		rospy.spin()

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

	return map_data_rgb

if __name__ == '__main__':
	try:
		node = Node()
		node.run()
	except rospy.ROSInterruptException:
		pass