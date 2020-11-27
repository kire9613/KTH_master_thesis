#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np

from svea.svea_managers.path_following_sveas import SVEAPurePursuit
from svea.states import VehicleState
from svea.localizers import LocalizationInterface
from svea.controllers.pure_pursuit import PurePursuitController
from svea.data import BasicDataHandler, TrajDataHandler, RVIZPathHandler
from svea.models.bicycle import SimpleBicycleModel
from svea.simulators.sim_SVEA import SimSVEA
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from nav_msgs.msg import Path

from map_explore_node import MapExplore
"""
__team__ = "Team 1"
__maintainers__ = "Roberto Castro Sundin, Astrid Lindstedt, Johan Hedin, Aravind Sadashiv, Sarthak Manocha”
__status__ = "Development"
"""

## SIMULATION PARAMS ##########################################################
vehicle_name = ""
target_velocity = 1# [m/s]
dt = 0.01 # frequency of the model updates

xs = [14.47, 37.26]
ys = [1.60, 1.29]
traj_x_init = np.linspace(xs[0], xs[1]).tolist()
traj_y_init = np.linspace(ys[0], ys[1]).tolist()
xs = [37.26,37.35]
ys = [1.29,6.96]
traj_x_init = np.append(traj_x_init,np.linspace(xs[0], xs[1]).tolist()[1:])
traj_y_init = np.append(traj_y_init,np.linspace(ys[0], ys[1]).tolist()[1:])
xs = [37.35,14.68]
ys = [6.96,7.31]
traj_x_init = np.append(traj_x_init,np.linspace(xs[0], xs[1]).tolist()[1:])
traj_y_init = np.append(traj_y_init,np.linspace(ys[0], ys[1]).tolist()[1:])
xs = [14.69,14.47]
ys = [7.31,1.60]
traj_x_init = np.append(traj_x_init,np.linspace(xs[0], xs[1]).tolist()[1:-1])
traj_y_init = np.append(traj_y_init,np.linspace(ys[0], ys[1]).tolist()[1:-1])	
###############################################################################

## INIT #######################################################################
default_init_pt = [0.0, 0.0, 0.0, 0.0] # [x, y, yaw, v], units: [m, m, rad, m/s]
###############################################################################

class Node:
    def __init__(self):
        rospy.init_node('floor2_local_planning')

        # grab parameters from launch-file
        start_pt_param = rospy.search_param('start_pt')
        is_sim_param = rospy.search_param('is_sim')
        use_rviz_param = rospy.search_param('use_rviz')
        use_matplotlib_param = rospy.search_param('use_matplotlib')
        run_lidar_param = rospy.search_param('run_lidar')

        start_pt = rospy.get_param(start_pt_param, default_init_pt)
        if isinstance(start_pt, str):
            start_pt = start_pt.split(',')
            start_pt = [float(curr) for curr in start_pt]
            start_pt = VehicleState(*start_pt)

        self.is_sim = rospy.get_param(is_sim_param, True)
        self.use_rviz = rospy.get_param(use_rviz_param, False)
        self.use_matplotlib = rospy.get_param(use_matplotlib_param, False)
        self.run_lidar = rospy.get_param(run_lidar_param, True)

        #This subscriber and its callback function is the local planner
        traj_upd_sub = rospy.Subscriber('/trajectory_updates', Path, self.callback_traj)

        # select data handler based on the ros params
        if self.use_rviz:
            self.DataHandler = RVIZPathHandler
        elif self.use_matplotlib:
            self.DataHandler = TrajDataHandler
        else:
            # DataHandler = BasicDataHandler
            self.DataHandler = RVIZPathHandler

        if self.is_sim:
            # start the simulation
            model_for_sim = SimpleBicycleModel(start_pt)
            self.simulator = SimSVEA(vehicle_name, model_for_sim,
                                dt=dt, start_paused=True, run_lidar=self.run_lidar).start()
        self.traj_x = traj_x_init
        self.traj_y = traj_y_init

    def callback_traj(self,path):
        self.traj_x = []
        self.traj_y = []
        for i in range(len(path.poses) - 1):
            self.traj_x.append(path.poses[i].pose.position.x)
            self.traj_y.append(path.poses[i].pose.position.y)
        
    def run(self):
        # start pure pursuit SVEA manager
        svea = SVEAPurePursuit(vehicle_name,
                            LocalizationInterface,
                            PurePursuitController,
                            self.traj_x, self.traj_y,
                            data_handler = self.DataHandler)
        svea.start(wait=True)

        if self.is_sim:
            # start simulation
            self.simulator.toggle_pause_simulation()

        # simualtion loop
        
        svea.controller.target_velocity = target_velocity
        while not svea.is_finished and not rospy.is_shutdown():
            state = svea.wait_for_state()

            #This step updates the global path
            svea.update_traj(self.traj_x, self.traj_y)

            # compute control input via pure pursuit
            steering, velocity = svea.compute_control()
            svea.send_control(steering, velocity)

            # visualize data
            if self.use_matplotlib or self.use_rviz:
                svea.visualize_data()
            else:
                rospy.loginfo_throttle(1, state)

        if not rospy.is_shutdown():
            rospy.loginfo("Trajectory finished!")

        rospy.spin()

if __name__ == '__main__':
	try:
		node = Node()
		node.run()
	except rospy.ROSInterruptException:
		pass
