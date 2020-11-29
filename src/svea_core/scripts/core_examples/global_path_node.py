#!/usr/bin/env python
"""
	@by Johan Hedin Team1
"""
# Python standard library
import numpy as np

# ROS
import rospy

# ROS messages
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

## GLOBAL PATH NODE PARAMS ####################################################
update_rate = 1
###############################################################################
class Node:
	"""
	asd
	"""
	def __init__(self):

		rospy.init_node('global_path')

		self.path_pub = rospy.Publisher('global_path', Path, queue_size=1, latch=True)
		
		# The trajectory

		xs = [14.47, 37.26]
		ys = [1.60, 1.29]
		traj_x = np.linspace(xs[0], xs[1]).tolist()
		traj_y = np.linspace(ys[0], ys[1]).tolist()
		xs = [37.26,37.35]
		ys = [1.29,6.96]
		traj_x = np.append(traj_x,np.linspace(xs[0], xs[1]).tolist()[1:])
		traj_y = np.append(traj_y,np.linspace(ys[0], ys[1]).tolist()[1:])
		xs = [37.35,13.27]
		ys = [6.96,7.11]
		traj_x = np.append(traj_x,np.linspace(xs[0], xs[1]).tolist()[1:])
		traj_y = np.append(traj_y,np.linspace(ys[0], ys[1]).tolist()[1:])
		xs = [13.27,14.47]
		ys = [7.11,1.60]
		traj_x = np.append(traj_x,np.linspace(xs[0], xs[1]).tolist()[1:-1])
		traj_y = np.append(traj_y,np.linspace(ys[0], ys[1]).tolist()[1:-1])	

		self.path = lists_to_path(traj_x, traj_y)

	def run(self):

		rate = rospy.Rate(update_rate)

		while not rospy.is_shutdown():
			self.path_pub.publish(self.path)
			rate.sleep()

		rospy.spin()

def lists_to_path(x_list, y_list):
	path = Path()
	path.header.stamp = rospy.Time.now()
	path.header.frame_id = 'map'
	path.poses = []
	for i in range(len(x_list)):
		curr_pose = PoseStamped()
		curr_pose.header.frame_id = 'map'
		curr_pose.pose.position.x = x_list[i]
		curr_pose.pose.position.y = y_list[i]
		path.poses.append(curr_pose)
	return path

if __name__ == '__main__':
	try:
		node = Node()
		node.run()
	except rospy.ROSInterruptException:
		pass