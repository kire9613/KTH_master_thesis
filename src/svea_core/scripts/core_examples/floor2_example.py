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


## SIMULATION PARAMS ##########################################################
vehicle_name = ""
target_velocity = 1.0 # [m/s]
dt = 0.01 # frequency of the model updates

#TODO: create a trajectory that goes around the track
xs = [-2.33, 10.48, 5.8, -6.9, -2.5]
ys = [-7.09, 11.71, 14.6, -3.8, -6.7]
traj_x_1st = np.linspace(xs[0], xs[1])
traj_x_2nd =  np.linspace(xs[1], xs[2])
traj_x_3rd = np.linspace(xs[2], xs[3])
traj_x_4th = np.linspace(xs[3], xs[4])

traj_y_1st = np.linspace(ys[0], ys[1])
traj_y_2nd = np.linspace(ys[1], ys[2])
traj_y_3rd = np.linspace(ys[2], ys[3])
traj_y_4th = np.linspace(ys[3], ys[4])

traj_x = np.hstack((traj_x_1st,traj_x_2nd, traj_x_3rd, traj_x_4th)).tolist()
traj_y = np.hstack((traj_y_1st,traj_y_2nd,traj_y_3rd, traj_y_4th)).tolist()

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
                            dt=dt, run_lidar=True, start_paused=True).start()

    # start pure pursuit SVEA manager
    svea = SVEAPurePursuit(vehicle_name,
                           LocalizationInterface,
                           PurePursuitController,
                           traj_x, traj_y,
                           data_handler = DataHandler)
    svea.start(wait=True)

    if is_sim:
        # start simulation
        simulator.toggle_pause_simulation()

    # simualtion loop
    svea.controller.target_velocity = target_velocity
    while not svea.is_finished and not rospy.is_shutdown():

        state = svea.wait_for_state()

        # compute control input via pure pursuit
        steering, velocity = svea.compute_control()
        svea.send_control(steering, velocity)
        rospy.loginfo_throttle(5, velocity)

        # visualize data
        if use_matplotlib or use_rviz:
            svea.visualize_data()
        else:
            rospy.loginfo_throttle(1, state)

    if not rospy.is_shutdown():
        rospy.loginfo("Trajectory finished!")

    rospy.spin()


if __name__ == '__main__':
    main()
