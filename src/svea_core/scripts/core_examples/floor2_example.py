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

from math import radians, cos, sin

from svea.controllers.rrt import *

#import sys
#sys.path.insert(1, 'Home/svea_starter/src/svea_core/resources/param')
#from Home.svea_starter.src.svea_core.resources.param.read_obs import *

## SIMULATION PARAMS ##########################################################

vehicle_name = ""
target_velocity = 1.0 # [m/s]

dt = 0.01 # frequency of the model updates

#TODO: create a trajectory that goes around the track
xs = [-2.33, 10.48]
ys = [-7.09, 11.71]

traj_x1 = np.linspace(xs[0], xs[1])
traj_y1 = np.linspace(ys[0], ys[1])

traj_x2 = np.linspace(xs[1], 6)
traj_y2 = np.linspace(ys[1], 15)

traj_x = np.concatenate((traj_x1, traj_x2), axis = None)
traj_y = np.concatenate((traj_y1, traj_y2), axis = None)

traj_x3 = np.linspace(6, -7.33)
traj_y3 = np.linspace(15, -4.15)

traj_x = np.concatenate((traj_x, traj_x3), axis = None)
traj_y = np.concatenate((traj_y, traj_y3), axis = None)

traj_x4 = np.linspace(-7.33, -2.5)
traj_y4 = np.linspace(-4.15, -6.5)

traj_x = np.concatenate((traj_x, traj_x4), axis = None)
traj_y = np.concatenate((traj_y, traj_y4), axis = None)

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

    # select data handler based on the ros params
    if use_rviz:
        DataHandler = RVIZPathHandler
    else:
        DataHandler = TrajDataHandler

    if is_sim:

	# start the simulation
        model_for_sim = SimpleBicycleModel(start_pt)
        simulator = SimSVEA(vehicle_name, model_for_sim,
                            dt=dt, start_paused=True, run_lidar=True,
                 publish_pose=True,
                 publish_odometry=True).start()

    # start pure pursuit SVEA manager



    # Adding comment to test

    x_t = [-5.63, 0, 3]
    y_t = [-8.62, 0, 0]

    #cur_path = os.path.dirname(__file__)
    #print(cur_path)
    #new_path = os.path.relpath('/resources/param/obstacles.yaml', cur_path)
    #print(new_path)
    #f = open('multi.py', "r")
    #print(f.read()) 

    traj_x1, traj_y1 = solution(start_pt.x, start_pt.y, -4.23, -10, [])

    svea = SVEAPurePursuit(vehicle_name,
                           LocalizationInterface,
                           PurePursuitController,
                           traj_x1, traj_y1,
                           data_handler = DataHandler)
    svea.start(wait=True)

    # start track handler
    track = Track(vehicle_name, publish_track=True)
    track.start()

    if is_sim:
        # start simulation
        simulator.toggle_pause_simulation()

    # simualtion loop
    svea.controller.target_velocity = target_velocity

    angle_lidar = [radians(angle) for angle in range(-135, 135, 2)]

    j = 1
    while not rospy.is_shutdown():
	    while not svea.is_finished and not rospy.is_shutdown():
		state = svea.wait_for_state()

		obs = []
		
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
		    dist = svea.lidar.scan

		    for i in range(0,135):
			obs.append( (cos(angle_lidar[i])*dist[i] + state.x, sin(angle_lidar[i])*dist[i] + state.y) )

	    svea.send_control(steering, 0)
            
	    traj_x2, traj_y2 = solution(state.x, state.y, x_t[j], y_t[j], [])
	    print("hej")

	    svea = SVEAPurePursuit(vehicle_name, LocalizationInterface, PurePursuitController, traj_x2, traj_y2, data_handler = DataHandler)
    	    svea.start(wait=True)
            svea.controller.target_velocity = target_velocity

	    j = j +1

    if not rospy.is_shutdown():
        rospy.loginfo("Trajectory finished!")

    rospy.spin()


if __name__ == '__main__':
    main()
