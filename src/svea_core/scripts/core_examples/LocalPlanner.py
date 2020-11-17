#!/usr/bin/env python

"""
Module containing the collision avoidance algoreithm
"""
import math
import rospy
import numpy

## PLANNER PARAMS #############################################################
a_discretization = 0 #Fineness of acceleration discretization
a_min = 0 #Minimum torque?
a_max = 0 #Maxmimum torque?
time_frame = 1 #Look-ahead window
###############################################################################

class LocalPlanner:
	"""
	Local planning/Collision avoidance classr 
	"""

	def __init__(self, global_traj_x, global_traj_y):
		"""
		:param global_traj_x/y: 
		:type global_traj_x/y: list
		"""
		self.global_map = sub_map
		self.traj_x = global_traj_x
		self.traj_y = global_traj_y

		self.lidar_subs = rospy.Subscriber("/scan", LaserScan, local_cost_map)
		self.cost_map = #Some nd array

	def integrator_fw_euler(x,y,t,v):
		return # x+1,y+1,t+1,v+1

	def local_cost_map(scan)
	    """
		create cost_map from \scan
		update cost_map 
		"""
		ranges = scan.ranges
    	min_dist = np.nanmin(ranges)
		
		#goal_x = self.traj_x[traj_ind+max_ind]
		#goal_y = self.traj_y[traj_ind+max_ind]

	def plan(state, curr_ind)
		x0 = state.x
		y0 = state.y
		t0 = state.t
		v0 = state.v

		# v_feas(v0)
		v_fe_arr = np.range(-a_min, a_max)
		"""
		cost = inf
		x_opt
		for v in v_fe_arr
			x = integrator(x,y,t,v)
			if cost(x) < cost
				x_opt = x
		"""
							
		curr_traj_x = self.traj_x[curr_ind]
		curr_traj_y = self.traj_y[curr_ind]

		self.traj_x[traj_ind+1] = 
		self.traj_y[traj_ind+1] =
		
		return self.traj_x, self.traj_y



