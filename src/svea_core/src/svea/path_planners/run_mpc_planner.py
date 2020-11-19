#! /usr/bin/env python
"""
Path planning with non-linear MPC
"""
from mpc_map.occupancy_grid import OccupancyGrid
from mpc_map.ros_interface import ROSInterface as MapROSInterface
from mpc_planner.main_planner import Planner
from mpc_planner.ros_interface import ROSInterface

import rospy


if __name__ == '__main__':
    ros_interface = ROSInterface()


    #Fetch mpc obstacles without initializing occupancy grid class
    parameter_names = ['obstacles']
    parameters = {
        parameter_name: rospy.get_param('~{}'.format(parameter_name))
        for parameter_name in parameter_names
    }
    mpc_obs = parameters['obstacles']

    #Publish obstacles
    MapROSInterface.publish(mpc_obs)
    
    planner_name = ros_interface.planner_algorithm
    planner_parameters = ros_interface.get_planner_parameters()

    planner = Planner(mpc_obs, planner_name, planner_parameters)

    while not ros_interface.is_shutdown():
        ros_interface.sleep()

        if not ros_interface.has_endpoint_changed:
            continue

        initial_state = ros_interface.initial_state
        goal_state = ros_interface.goal_state
        path = planner.compute_path(initial_state, goal_state)
        ros_interface.publish_path(path)
