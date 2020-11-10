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

import sync_lidar
import explored_map
import tf

"""
__team__ = "Team 1"
__maintainers__ = "Roberto Castro Sundin, Astrid Lindstedt, Johan Hedin, Aravind Sadashiv, Sarthak Manocha”
__status__ = "Development"
"""

## SIMULATION PARAMS ##########################################################
vehicle_name = "SVEA"
target_velocity = 1.0 # [m/s]
dt = 0.01 # frequency of the model updates

xs = [-2.33, 10.48]
ys = [-7.09, 11.71]
traj_x = np.linspace(xs[0], xs[1]).tolist()
traj_y = np.linspace(ys[0], ys[1]).tolist()
xs = [10.48,6.03]
ys = [11.71,14.8]
traj_x = np.append(traj_x,np.linspace(xs[0], xs[1]).tolist()[1:])
traj_y = np.append(traj_y,np.linspace(ys[0], ys[1]).tolist()[1:])
xs = [6.03,-6.78]
ys = [14.8,-4]
traj_x = np.append(traj_x,np.linspace(xs[0], xs[1]).tolist()[1:])
traj_y = np.append(traj_y,np.linspace(ys[0], ys[1]).tolist()[1:])
xs = [-6.78,-2.33]
ys = [-4.00,-7.09]
traj_x = np.append(traj_x,np.linspace(xs[0], xs[1]).tolist()[1:-1])
traj_y = np.append(traj_y,np.linspace(ys[0], ys[1]).tolist()[1:-1])

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
    run_lidar_param = rospy.search_param('run_lidar')

    start_pt = rospy.get_param(start_pt_param, default_init_pt)
    if isinstance(start_pt, str):
        start_pt = start_pt.split(',')
        start_pt = [float(curr) for curr in start_pt]
        start_pt = VehicleState(*start_pt)

    is_sim = rospy.get_param(is_sim_param, True)
    use_rviz = rospy.get_param(use_rviz_param, False)
    use_matplotlib = rospy.get_param(use_matplotlib_param, False)
    run_lidar = rospy.get_param(run_lidar_param, True)

    return start_pt, is_sim, use_rviz, use_matplotlib, run_lidar

def callback_scan(scan):
    """ Callback for lidarscan """
    ranges = scan.ranges
    min_dist = np.nanmin(ranges) # TODO: Make available as self.min_dist etc.?

#def publish_map_base_link_tf(br,state):
#    t = geometry_msgs.msg.TransformStamped()
#    t.header.stamp = rospy.Time.now()
#    t.header.frame_id = state.frame_id
#    t.child_frame_id = state.child_frame
#    t.transform = state._pose_msg
#    br.sendTransform(t)

def main():
    rospy.init_node('floor2_team1')
    start_pt, is_sim, use_rviz, use_matplotlib, _run_lidar = param_init()

    scanner = sync_lidar.LidarScan()
    obs_map = explored_map.ObstacleMap()

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
                            dt=dt, start_paused=True, run_lidar=_run_lidar).start()

    #lidar_sub = rospy.Subscriber("/scan", LaserScan, callback_scan)

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

    # TODO:planner = LocalPlanner(traj_x, traj_y)

    svea.controller.target_velocity = target_velocity
    while not svea.is_finished and not rospy.is_shutdown():
        state = svea.wait_for_state()
        scan = scanner.scan

        obs_map.update_map(state,scan)

        print(obs_map.map_matrix)
        # TODO: ind = svea.controller.last_index
        # TODO: upd_traj_x, upd_traj_y = planner.plan(state,ind)
        # TODO: svea.update_traj(self, upd_traj_x, upd_traj_y)

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
