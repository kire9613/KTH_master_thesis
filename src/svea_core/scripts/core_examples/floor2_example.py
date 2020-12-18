#!/usr/bin/env python

import rospy
import numpy as np

from svea.svea_managers.path_following_sveas import SVEAPurePursuit
from svea.states import VehicleState
from svea.localizers import LocalizationInterface
from svea.controllers.pure_pursuit import PurePursuitController
from svea.data import BasicDataHandler, TrajDataHandler, RVIZPathHandler
from svea.models.bicycle import SimpleBicycleModel
from svea.simulators.sim_SVEA import SimSVEA
from svea.track import Track

from math import radians, cos, sin, hypot, atan2

from svea.planner.rrt import *
from svea.planner.rrt_connect import *

from svea.map.occupancy_grid import OccupancyGrid
from svea.map.ros_interface import ROSInterface as MapROSInterface
from svea.planner.ros_interface import ROSInterface

## SIMULATION PARAMS ##########################################################
vehicle_name = "SVEA"
target_velocity =  0.6 # [m/s]
dt = 0.01 # frequency of the model updates

#TODO: create a trajectory that goes around the track
xs = [0, 18.7, 19.7, -13.5, -14.3, -7.8, -3.58]
ys = [0, -0.727, 3.26, 6.04, 1.5, 1.74, 0.15]

###############################################################################

## INIT #######################################################################
default_init_pt = [0.0, 0.0, 0.0, 0.0] # [x, y, yaw, v], units: [m, m, rad, m/s]
###############################################################################


def param_init():
    """Initialization handles use with just python or in a launch file
    """
    # grab parameters from launch-file
    start_pt_param = rospy.search_param('start_pt')
    is_sim_param = rospy.search_param('is_sim')
    use_rviz_param = rospy.search_param('use_rviz')
    use_matplotlib_param = rospy.search_param('use_matplotlib')

    start_pt = rospy.get_param(start_pt_param, default_init_pt)
    if isinstance(start_pt, str):
        start_pt = start_pt.split(',')
        start_pt = [float(curr) for curr in start_pt]
        start_pt = VehicleState(*start_pt)

    is_sim = rospy.get_param(is_sim_param, True)
    use_rviz = rospy.get_param(use_rviz_param, False)
    use_matplotlib = rospy.get_param(use_matplotlib_param, False)

    return start_pt, is_sim, use_rviz, use_matplotlib


def main():
    rospy.init_node('floor2_example')
    start_pt, is_sim, use_rviz, use_matplotlib = param_init()

    #NMPC
    ros_interface = ROSInterface()
    occupancy_grid_parameters = MapROSInterface.get_occupancy_grid_parameters()
    occupancy_grid = OccupancyGrid(**occupancy_grid_parameters)

    # select data handler based on the ros params
    if use_rviz:
        DataHandler = RVIZPathHandler
    elif use_matplotlib:
        DataHandler = TrajDataHandler
    else:
        DataHandler = BasicDataHandler

    if is_sim:
	# start the simulation
        model_for_sim = SimpleBicycleModel(start_pt)
        simulator = SimSVEA(vehicle_name, model_for_sim,
                            dt=dt, start_paused=True, run_lidar=True).start()

    # start pure pursuit SVEA manager
    
    if is_sim:
        # start simulation
        simulator.toggle_pause_simulation()

    track = Track(vehicle_name, publish_track=True)
    track.start()

    # simualtion loop
    angle_lidar = [radians(angle) for angle in range(-135, 135, 2)]

    f = 0

    while not rospy.is_shutdown():

            if f == 0:

              ros_interface.sleep()

	      if not ros_interface.has_endpoint_changed:
	         continue
            
              init = ros_interface.initial_state

	      goal = [xs[1], ys[1]]

	      traj_x, traj_y = compute_path_connect(init[0], init[1], goal[0], goal[1], occupancy_grid.obstacles, occupancy_grid.localmap)

	      if traj_x[-1] == init[0]:
		 print("reversed path")
		 traj_x.reverse()
		 traj_y.reverse()
		    
	      for i in range(len(xs)-2): 
		 init = goal
		 goal = [xs[i+2], ys[i+2]]
		 t_x, t_y = compute_path_connect(init[0], init[1], goal[0], goal[1], occupancy_grid.obstacles, occupancy_grid.localmap)

		 if t_x[-1] == init[0]:
		    print("reversed path")
		    t_x.reverse()
		    t_y.reverse()

		 traj_x = np.concatenate((traj_x, t_x), axis = None)
		 traj_y = np.concatenate((traj_y, t_y), axis = None)

            elif f == 1:
              traj_x, traj_y = compute_path_connect(xs[-1], ys[-1], 0, 0, occupancy_grid.obstacles, occupancy_grid.localmap)

	      if traj_x[-1] == init[0]:
		 print("reversed path")
		 traj_x.reverse()
		 traj_y.reverse()

	    svea = SVEAPurePursuit(vehicle_name, LocalizationInterface, PurePursuitController, traj_x, traj_y, data_handler = DataHandler)
            svea.start(wait=True)
            svea.controller.target_velocity = target_velocity

	    replan = False
	    try_replan = 0

	    while not svea.is_finished and not rospy.is_shutdown():

		state = svea.wait_for_state()
		
		# compute control input via pure pursuit
		steering, velocity = svea.compute_control()
		svea.send_control(steering, velocity)
		rospy.loginfo_throttle(1, velocity)

		# visualize data
		if use_matplotlib or use_rviz:
		    svea.visualize_data()
		else:
		    rospy.loginfo_throttle(1, state)

		if svea.lidar.scan != []:
		    dist = np.array(svea.lidar.scan)
                    
                    obs_x = np.multiply(np.cos(angle_lidar+np.ones(135)*state.yaw), dist) + state.x*np.ones(135) 
                    obs_y = np.multiply(np.sin(angle_lidar+np.ones(135)*state.yaw), dist) + state.y*np.ones(135)

                    obs_x = obs_x[np.logical_not(np.isnan(obs_x))] 
                    obs_y = obs_y[np.logical_not(np.isnan(obs_y))]
                    dist = dist[np.logical_not(np.isnan(dist))]

                    obs_x[obs_x > 32.9] = 32.9 
                    obs_y[obs_y > 16.9] = 16.9 

                    obs_x[obs_x < -30.5] = -30.5 
                    obs_y[obs_y < -11.4] = -11.4 

                    obs = np.array([obs_x, obs_y])

                    occupancy_grid.updatemap(dist, obs.T, [state.x, state.y, state.yaw])

                    if dist[68] < 5 or dist[67] < 3 or dist[69] < 3:

                      target = svea.controller.target
                      idx_target = svea.controller.index

                      try:
                        future_target3 = [traj_x[idx_target+3], traj_y[idx_target+3]]
                        future_target2 = [traj_x[idx_target+2], traj_y[idx_target+2]]
		        t_v2 = np.linspace(future_target2, future_target3, 100)
		        t_i2 = occupancy_grid.array_to_index(t_v2)

                        localmap = occupancy_grid.localmap

		        for t in range(len(t_i2)):

		          if localmap[t_i2[t][0], t_i2[t][1]] == 100 or localmap[t_i2[t][0]-1, t_i2[t][1]] == 100 or localmap[t_i2[t][0]+1, t_i2[t][1]] == 100 or localmap[t_i2[t][0], t_i2[t][1]-1] == 100 or localmap[t_i2[t][0], t_i2[t][1]+1] == 100 or localmap[t_i2[t][0]-1, t_i2[t][1]-1] == 100 or localmap[t_i2[t][0]+1, t_i2[t][1]+1] == 100 or localmap[t_i2[t][0]-1, t_i2[t][1]+1] == 100 or localmap[t_i2[t][0]+1, t_i2[t][1]-1] == 100:
			    
                            print("must replan")
                            try_replan += 1

                            if try_replan == 3:
                              replan = True
                              try_replan = 0

	                      svea.send_control(steering, 0)

                              try: 
                                future_target = [traj_x[idx_target+7], traj_y[idx_target+7]]
                                s = 7

                              except IndexError:
                                future_target = [traj_x[idx_target+5], traj_y[idx_target+5]]
                                s = 5

                              traj_x1, traj_y1 = compute_path(state.x, state.y, future_target[0], future_target[1], occupancy_grid.obstacles, occupancy_grid.localmap)

                              if traj_x1 == None and s == 7:
                                print("traj is none")
                                future_target = [traj_x[idx_target+5], traj_y[idx_target+5]]
                                traj_x1, traj_y1 = compute_path(state.x, state.y, future_target[0], future_target[1], occupancy_grid.obstacles, occupancy_grid.localmap)

                              traj_x = np.concatenate((traj_x1, traj_x[idx_target+s:]), axis = None)
                              traj_y = np.concatenate((traj_y1, traj_y[idx_target+s:]), axis = None)

                              svea.update_traj(traj_x, traj_y)
			    
                            break

                      except IndexError:
                        pass

            if f == 1:
              break
            f += 1

    svea.send_control(steering, 0)

    if not rospy.is_shutdown():
        rospy.loginfo("Trajectory finished!")

    rospy.spin()

if __name__ == '__main__':
    main()
