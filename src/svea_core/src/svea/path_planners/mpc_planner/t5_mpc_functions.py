#!/usr/bin/env python
import numpy as np
import rospy
from svea.path_planners.mpc_planner.ros_interface import ROSInterface
from svea.path_planners.mpc_planner.main_planner import Planner

def run_mpc_planner(ros_interface):
    tic = rospy.get_time()
    print("entering while loop")
    if ros_interface._current_target_state != [0,0] and ros_interface.initial_state != None:
        ros_interface.compute_goal()
   
    #Fetch mpc obstacles without initializing occupancy grid class
    parameter_names = ['obstacles']
    parameters = {
        parameter_name: rospy.get_param('~{}'.format(parameter_name))
        for parameter_name in parameter_names
    }
    mpc_obs = parameters['obstacles']
    
    planner_name = ros_interface.planner_algorithm
    planner_parameters = ros_interface.get_planner_parameters()
    planner = Planner(mpc_obs, planner_name, planner_parameters)
    print('Waiting for MPC')
    #planner = Planner(occupancy_grid, planner_name, planner_parameters)
    print('Computing MPC path')
    initial_state = ros_interface.initial_state
    goal_state = ros_interface.goal_state
    print("Initial state:",initial_state)
    print("Goal state:",goal_state)
    
    path = planner.compute_path(initial_state, goal_state)
    print(type(path))
    if type(path) is None:
        print("PATH IS NONE!!!!!!!!")
    toc = rospy.get_time()
    print("Planner took ", toc-tic, "seconds to compute path")
    try:
        traj_x, traj_y = convert_mpc_path(path)
        return traj_x, traj_y, True
    except:
        print("FAILED MPC PLAN")
        return [],[], False

def convert_mpc_path(path):
    print('Getting MPC path')
    g_traj_x = []
    g_traj_y = []
    print("length:")
    print(len(path))
    for i in range(0,len(path)-1):
        g_traj_x.append(path[i,0])
        g_traj_y.append(path[i,1])
    return g_traj_x, g_traj_y

def print_every_second( message, data, state):
        # print once per second
    global print_time
    last_time = state.time_stamp.to_sec()
    if (last_time - print_time)  > 1:
        print(message, data)
        print_time = last_time   

def compute_angles(x_traj,y_traj):
    theta_traj = []
    for i in range(0,len(x_traj)-1):
        theta_traj.append(np.arctan2((y_traj[i+1] - y_traj[i]),(x_traj[i+1] - x_traj[i])))
    return theta_traj 