#!/usr/bin/env python

"""
	@by Johan Hedin Team1
"""

# Python standard library
import os
import pickle

# ROS
import rospy

# ROS messages
from nav_msgs.msg import OccupancyGrid

## TEST MAP NODE PARAMS #######################################################
update_rate = 1
dirname = os.path.dirname(__file__)
svea_core = os.path.join(dirname, '../../')
map_name = "problem_map_occ" # change this for different maps
file_path = svea_core + 'resources/maps/' + map_name + ".pickle"
###############################################################################

class Node:
	"""
	asd
	"""
	def __init__(self):

		rospy.init_node('pickle_pub')

		self.pickled_map_pub = rospy.Publisher('pickled_map', OccupancyGrid, queue_size=1, latch=True)
		
		self.pickled_map = OccupancyGrid()

		with open(file_path, 'rb') as f:
			self.pickled_map = pickle.load(f)

	def run(self):

		rate = rospy.Rate(update_rate)

		while not rospy.is_shutdown():
			self.pickled_map_pub.publish(self.pickled_map)
			rate.sleep()

		rospy.spin()

if __name__ == '__main__':
	try:
		node = Node()
		node.run()
	except rospy.ROSInterruptException:
		pass