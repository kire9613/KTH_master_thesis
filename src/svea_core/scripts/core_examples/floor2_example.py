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

__team__ = "Team 3"
__maintainers__ = "Albin Larsson Forsberg, Timotheos Souroulla, Filip Hestell, Roman Landin"
__status__ = "Development"

## SIMULATION PARAMS ##########################################################
vehicle_name = "SVEA"
target_velocity = 1.0 # [m/s]
dt = 0.01 # frequency of the model updates

#TODO: create a trajectory that goes around the track
xs1 = [-3.04, 10.06]
ys1 = [-8.24, 11.12]
xs2 = [9.83, 6.45]
ys2 = [12.04, 14.38]
xs12 = [xs1[1],xs2[0]]
ys12 = [ys1[1],ys2[0]]
xs3 = [5.28, -6.58]
ys3 = [14.22, -2.97]
xs23 = [xs2[1],xs3[0]]
ys23 = [ys2[1],ys3[0]]
xs4 = [-6.61, -2.34]
ys4 = [-4.44, -7.17]
xs34 = [xs3[1],xs4[0]]
ys34 = [ys3[1],ys4[0]]

traj_x1 = np.linspace(xs1[0], xs1[1]).tolist()
traj_y1 = np.linspace(ys1[0], ys1[1]).tolist()
traj_x12 = np.linspace(xs12[0], xs12[1]).tolist()
traj_y12 = np.linspace(ys12[0], ys12[1]).tolist()
traj_x2 = np.linspace(xs2[0], xs2[1]).tolist()
traj_y2 = np.linspace(ys2[0], ys2[1]).tolist()
traj_x23 = np.linspace(xs23[0], xs23[1]).tolist()
traj_y23 = np.linspace(ys23[0], ys23[1]).tolist()
traj_x3 = np.linspace(xs3[0], xs3[1]).tolist()
traj_y3 = np.linspace(ys3[0], ys3[1]).tolist()
traj_x34 = np.linspace(xs34[0], xs34[1]).tolist()
traj_y34 = np.linspace(ys34[0], ys34[1]).tolist()
traj_x4 = np.linspace(xs4[0], xs4[1]).tolist()
traj_y4 = np.linspace(ys4[0], ys4[1]).tolist()

traj_x = [traj_x1, traj_x12, traj_x2, traj_x23, traj_x3, traj_x34, traj_x4] #  ,
traj_y = [traj_y1, traj_y12, traj_y2, traj_y23, traj_y3, traj_y34, traj_y4] # , ,

traj_x = [val for sublist in traj_x for val in sublist]
traj_y = [val for sublist in traj_y for val in sublist]

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
    elif use_matplotlib:
        DataHandler = TrajDataHandler
    else:
        DataHandler = BasicDataHandler

    if is_sim:
        # start the simulation
        model_for_sim = SimpleBicycleModel(start_pt)
        simulator = SimSVEA(vehicle_name, model_for_sim,
                            dt=dt, start_paused=True).start()

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


        index =  svea.controller.last_index
        if index > 50:
            svea.controller.traj_x = traj_x
            svea.controller.traj_y = traj_y
        else:
            svea.controller.traj_x = traj_x[0:100]
            svea.controller.traj_y = traj_y[0:100]
        ##print(index)
        # compute control input via pure pursuit
        steering, velocity = svea.compute_control()
        svea.send_control(steering, velocity)

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
