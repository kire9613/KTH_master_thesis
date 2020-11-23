#! /usr/bin/env python
"""
Path planning with non-linear MPC
"""
from mpc_map.occupancy_grid import OccupancyGrid
from mpc_map.ros_interface import ROSInterface as MapROSInterface
from mpc_planner.main_planner import Planner
from mpc_planner.ros_interface import ROSInterface
from astar import ControlState
import rospy


if __name__ == '__main__':
    ros_interface = ROSInterface()

    # old code
    occupancy_grid_parameters = MapROSInterface.get_occupancy_grid_parameters()

    occupancy_grid = OccupancyGrid(**occupancy_grid_parameters)

    MapROSInterface.publish(occupancy_grid)
    # old code
    
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
    print('Waiting for MPC')
    #planner = Planner(occupancy_grid, planner_name, planner_parameters)
    while not ros_interface.is_shutdown():
        ros_interface.sleep()
        if  ControlState.mpc_replan:
            print('Computing MPC path')
            initial_state = ros_interface.initial_state
            goal_state = ros_interface.goal_state
            path = planner.compute_path(initial_state, goal_state)
            ros_interface.publish_path(path)
            ControlState.mpc_replan = False# MPC path planner has finished
