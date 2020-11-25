#!/usr/bin/env python
"""
	@by Johan Hedin Team1
"""
# Python standard library
import numpy as np
# ROS
import rospy

# ROS messages
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path

# SVEA
from svea_msgs.msg import VehicleState

# OTHER
from matrix_astar import AStarPlanner

## COLLISION NODE PARAMS ######################################################
update_rate = 5
resolution = 0.05
splice_tol = 0.5
###############################################################################

class Node:
	"""
	asd
	"""
	def __init__(self):

		rospy.init_node('collision_node')

		self.solution_pub = rospy.Publisher('trajectory_updates', Path, queue_size=1, latch=True)

		self.problem_sub = rospy.Subscriber('/pickled_map', OccupancyGrid, self.callback_problem)
		self.path_sub = rospy.Subscriber('/global_path', Path, self.callback_path)
		self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.callback_map)

		self.global_path = Path()
		self.global_map = OccupancyGrid()
		self.problem_map = OccupancyGrid()

	def run(self):

		rate = rospy.Rate(update_rate)

		while not rospy.is_shutdown():
			rate.sleep()

		rospy.spin()

	def callback_path(self, path):
		self.global_path = path
	
	def callback_map(self, occ_map):
		self.global_map = occ_map

	def callback_problem(self, occ_map):
		self.problem_map = occ_map

		problem_vec = np.asarray(self.problem_map.data)
		values = []
		for el in problem_vec:
			if el not in values:
				values.append(el)
		print(values)

		glb_map_vec = np.asarray(self.global_map.data)

		values = []
		for el in glb_map_vec:
			if el not in values:
				values.append(el)
		print(values)

		planning_vec = glb_map_vec - problem_vec

		values = []
		for el in planning_vec:
			if el not in values:
				values.append(el)
		print(values)

		planner = AStarPlanner(planning_vec)
		y_list, x_list = planner.planning()

		for i in range(len(x_list)):
			x_list[i] = x_list[i]*resolution
			y_list[i] = y_list[i]*resolution
		
		x_new_global = []
		y_new_global = []

		n = len(self.global_path.poses)

		i = 0
		while 1:
			x = self.global_path.poses[i].pose.position.x
			y = self.global_path.poses[i].pose.position.y
			x_new_global.append(x)
			y_new_global.append(y)

			dist = np.sqrt((x_list[0] - x)**2 + (y_list[0] - y)**2)

			if dist < splice_tol:
				x_new_global.extend(x_list)
				y_new_global.extend(y_list)
				break
			i += 1
		
		i = n - 1
		while 1:
			x = self.global_path.poses[i].pose.position.x
			y = self.global_path.poses[i].pose.position.y

			dist = np.sqrt((x_list[len(x_list) - 1] - x)**2 + (y_list[len(y_list) - 1] - y)**2)

			if dist < splice_tol:
				for j in range(i, n):
					x_new_global.append(self.global_path.poses[j].pose.position.x)
					y_new_global.append(self.global_path.poses[j].pose.position.y)
				break
			i -= 1

		new_path = lists_to_path(x_new_global, y_new_global)
	
		#new_path = lists_to_path(x_list, y_list)

		self.solution_pub.publish(new_path)

def lists_to_path(x_list, y_list):
	path = Path()
	path.header.stamp = rospy.Time.now()
	path.header.frame_id = 'map'
	path.poses = []
	for i in range(len(x_list)):
		curr_pose = PoseStamped()
		curr_pose.header.frame_id = 'map'
		curr_pose.pose.position.x = float(x_list[i])
		curr_pose.pose.position.y = float(y_list[i])
		path.poses.append(curr_pose)
	return path

if __name__ == '__main__':
	try:
		node = Node()
		node.run()
	except rospy.ROSInterruptException:
		pass