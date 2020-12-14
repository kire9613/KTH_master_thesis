#!/usr/bin/env python
import rospy
import numpy as np

from svea.svea_managers.path_following_sveas import SVEAPurePursuit
from svea.states import VehicleState
from svea_msgs.msg import VehicleState as VehicleStateMsg
from svea.localizers import LocalizationInterface
from svea.controllers.pure_pursuit import PurePursuitController
from svea.data import TrajDataHandler, RVIZPathHandler
from svea.models.bicycle import SimpleBicycleModel
from svea.simulators.sim_SVEA import SimSVEA
from sensor_msgs.msg import LaserScan
from svea.path_planners.astar import *
from svea.track import Track
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped
from nav_msgs.msg import Path
from svea.path_planners.mpc_planner.ros_interface import ROSInterface
from svea.path_planners.astar_ros_interface import ROSInterface as AstarROSInterface
from svea.path_planners.mpc_planner.t5_mpc_functions import *

__team__ = "Team 5"
__maintainers__ ="Bianca Otake, Holmfridur Elvarsdottir, Johanna Andersson, Marcus Norgren"
__status__ ="Development"


## SIMULATION PARAMS ##########################################################
vehicle_name = ""
target_velocity = 1.0 # [m/s]
dt = 0.01 # frequency of the model updates

## PLANNER PARAMS ##########################################################
g_traj_x, g_traj_y = [],[]
## INIT #######################################################################
default_init_pt = [0.0, 0.0, 0.0, 0.0] # [x, y, yaw, v], units: [m, m, rad, m/s]
############################################################################### 

def extract_trajectory(use_astar, use_q1):
    if use_q1: # suitable coordinates for circle around lap
        xt, yt = 15.5,4.3
        x0, y0, theta0 =  0.5,0,0
        settings = {
        "driving_distance": 0.4,
        "use_track": True,
        "safety_distance": 0.3,
        "grid_resolution": 0.1,
        "success_threshold": 0.5,
        "intermediate_point": True,
        "use_q1": True
        }
    else:
        xt, yt = 8.44, 13
        x0, y0, theta0 =  -2.73, -7.3, 0.8978652
        settings = {
        "driving_distance": 0.25,
        "use_track": True,
        "safety_distance": 0.4,
        "grid_resolution": 0.1,
        "success_threshold": 1,
        "intermediate_point": True,
        "use_q1": False
        }
    traj_x, traj_y,success = generateTrajectory(settings,x0,y0,theta0,xt,yt,False)

    return traj_x, traj_y

def param_init():
    """Initialization handles use with just python or in a launch file
    """
    # grab parameters from launch-file
    start_pt_param = rospy.search_param('start_pt')
    is_sim_param = rospy.search_param('is_sim')
    use_rviz_param = rospy.search_param('use_rviz')
    use_matplotlib_param = rospy.search_param('use_matplotlib')
    use_astar_param = rospy.search_param('use_astar')
    use_mpc_param = rospy.search_param('use_mpc')
    use_q1_param = rospy.search_param('use_q1')
    
    start_pt = rospy.get_param(start_pt_param, default_init_pt)
    if isinstance(start_pt, str):
        start_pt = start_pt.split(',')
        start_pt = [float(curr) for curr in start_pt]
        start_pt = VehicleState(*start_pt)

    is_sim = rospy.get_param(is_sim_param, True)
    use_rviz = rospy.get_param(use_rviz_param, False)
    use_matplotlib = rospy.get_param(use_matplotlib_param, False)
    use_astar = rospy.get_param(use_astar_param, True)
    use_mpc = rospy.get_param(use_mpc_param, True)
    use_q1 = rospy.get_param(use_q1_param, True)
    
    return start_pt, is_sim, use_rviz, use_matplotlib, use_astar, use_mpc, use_q1

def main():
    rospy.init_node('team_5_floor2')

    # Initialize parameters
    istate = 0
    replan_counter = 0
    backup_attempted = False

    # Get ros parameters from launch file
    start_pt, is_sim, use_rviz, use_matplotlib, use_astar, use_mpc,  use_q1 = param_init()

    # A* emergency settings for replanning
    emergency_settings = {
        "driving_distance": 0.25,
        "use_track": False,
        "safety_distance": 0.13,
        "subscribe_to_obstacles": True,
        "grid_resolution": 0.075,
        "success_threshold": 0.5,
        "maximum_expansion": 1000,
        "intermediate_point": False,
        "use_q1": use_q1
        }
    
    # Compute global path
    traj_x, traj_y = extract_trajectory(use_astar, use_q1)
    # Split global path in two
    length = len(traj_x)
    middle_index = length//2
    traj_x0 = traj_x[:middle_index]
    traj_y0 = traj_y[:middle_index]
    traj_x1 = traj_x[middle_index:-1]
    traj_y1 = traj_y[middle_index:-1]


    
      # initialize ros interface
    if use_mpc:  
        ros_interface = ROSInterface(traj_x,traj_y)
    else: 
        ros_interface = AstarROSInterface(traj_x,traj_y)
    
    # Compute vehicle angles from astar path
    traj_theta = compute_angles(traj_x,traj_y)
    ros_interface.set_goal_angles(traj_theta)

    # Set global trajectory to first path
    traj_x,traj_y = traj_x0, traj_y0

    # select data handler based on the ros params
    if use_rviz:
        DataHandler = RVIZPathHandler
    else:
        DataHandler = TrajDataHandler

    if is_sim:
        # start the simulation
        model_for_sim = SimpleBicycleModel(start_pt)
        simulator = SimSVEA(vehicle_name, model_for_sim,
                            dt=dt, start_paused=True, run_lidar=True).start()

    # start pure pursuit SVEA manager
    svea = SVEAPurePursuit(vehicle_name,
                           LocalizationInterface,
                           PurePursuitController,
                           traj_x, traj_y,
                           data_handler = DataHandler)
    svea.start(wait=True)

    # start track handler
    track = Track(vehicle_name, publish_track=True)
    track.start()

    if is_sim:
        # start simulation
        simulator.toggle_pause_simulation()
    else: 
        rospy.wait_for_message('/initialpose',PoseWithCovarianceStamped)

    # Subscribe to initial state
    rospy.Subscriber('/state', VehicleStateMsg, ros_interface.cb_initial_state)  # noqa
    # Subscribe to Astar position
    rospy.Subscriber('/target', PointStamped, ros_interface.cb_target_state)
    # simualtion loop
    svea.controller.target_velocity = target_velocity

    rospy.Subscriber('/scan', LaserScan,svea.controller.emergency_stop)
    first_traj_complete = False
    while (not svea.is_finished and not rospy.is_shutdown()) or svea.controller.emg_traj_running:
        tic = rospy.get_time() # start timer
        state = svea.wait_for_state()

        if istate == 0: # IDLE state - waits here untill emergency stop
            svea.controller.target_velocity = target_velocity
            changed_astar_settings = False
            step_ahead = 6 # Set step ahead distance for replanning to set target state
            if svea.controller.emg_stop:
                print("state 1: Mapping obstacles")
                istate = 1 
        elif istate == 1: # Emergency stop activated - mapping obstacles
            # Slow down when executing replanned trajectory
            svea.controller.target_velocity = target_velocity*0.5 
            # Perform mapping when car has stopped
            if ros_interface.current_speed < 0.01:
                svea.controller.laser_mapping(ros_interface.initial_state)
                if use_mpc:
                    print("state 2: MPC Replanning")
                    istate = 2
                elif use_astar:
                    print("state 3: AStar Replanning")
                    istate = 3
                else:
                    print("state 0: No Replanner available")
                    istate = 0
                
        elif istate == 2: # Replan with MPC
            # Wait for target state and initial state values to update
            if  ros_interface._current_target_state != [0,0] and ros_interface.initial_state != None:
                # Initialize replanning sequence
                svea.controller.set_emg_traj_running(True)
                rospy.wait_for_message('/target',PointStamped)
                ros_interface.compute_goal(step_ahead) 
                g_traj_x, g_traj_y, success = run_mpc_planner(ros_interface)
                print("MPC Replanning Trajectory:")
                if success:
                    # Reduce distance for obstacle detection
                    svea.controller.emergency_distance = 0.25
                    svea.update_traj(g_traj_x, g_traj_y)
                    svea.controller.emg_stop = False
                    print("state 4: Following replanned path")
                    istate = 4
                elif not backup_attempted:
                    # Set backup time
                    timeout = 2
                    istate = 5
                    print("state 5: Backing up!")
                else: 
                    svea.controller.set_emg_traj_running(False)  
                    print("Replanned too many times - failed!!!")
                    break  

                  
        elif istate == 3: # Replan with Astar
            # Wait for target state and initial state values to update
            if  ros_interface._current_target_state != [0,0]  and ros_interface.initial_state != None:
                # Initialize replanning sequence
                svea.controller.set_emg_traj_running(True)
                rospy.wait_for_message('/target',PointStamped)
                ros_interface.compute_goal(step_ahead)
                x0, y0, theta0 = ros_interface.initial_state
                xt,yt,thetat = ros_interface.goal_state
                g_traj_x, g_traj_y,success = generateTrajectory(emergency_settings,x0,y0,theta0,xt,yt,False)# False
                
                print("Astar Replanning Trajectory:")
                if success:
                    # Reduce distance for obstacle detection
                    svea.controller.emergency_distance = 0.25
                    svea.controller.emg_stop = False
                    svea.update_traj(g_traj_x, g_traj_y)
                    print("state 4: Following replanned path")
                    istate = 4
                elif not backup_attempted:
                    # Set backup time
                    timeout = 2
                    istate = 5
                    print("state 5: Backing up!")
                elif not changed_astar_settings:
                    # Replan again with finer grid resolution
                    changed_astar_settings = True
                    emergency_settings = {
                                        "driving_distance": 0.2,
                                        "use_track": False,
                                        "safety_distance": 0.10,
                                        "subscribe_to_obstacles": True,
                                        "grid_resolution": 0.05,
                                        "success_threshold": 0.5,
                                        "maximum_expansion": 2000,
                                        "intermediate_point": False,
                                        "use_q1": use_q1
                                        }
                    print("Planning again with finer grid in Astar")
                    istate = 1 
                else: 
                    svea.controller.set_emg_traj_running(False)  
                    print("Replanned too many times - failed!!!")
                    break  

        elif istate == 4: # Follow replanned path
            if svea.controller.emg_stop:
                timeout = 1
                svea.update_traj(traj_x, traj_y)
                backup_attempted = False
                changed_astar_settings = False
                print("state 5: Emg stop in replanned path")
                istate = 5 
             
            if  svea.is_finished:
                # Reset settings before swtiching to global path
                replan_counter = 0
                backup_attempted = False
                svea.controller.emergency_distance = 1.0
                svea.reset_isfinished() 
                svea.update_traj(traj_x, traj_y)
                svea.controller.set_emg_traj_running(False) 
                print("state 0: Switching back to global path")
                istate = 0
        elif istate == 5: # Initialize backup
            svea.controller.target_velocity = target_velocity
            backup_attempted = True
            svea.controller.backing_up = True     
            time_start = rospy.get_time()
            istate = 6
        elif istate == 6: # pulse signal 0.5 s on, 0.5 s off (This is necessary on SVEA when going from 0 speed to be able to backup)      
            if rospy.get_time() - time_start > 0.5:
                svea.controller.backing_up = False
            if rospy.get_time() - time_start > 1:
                svea.controller.backing_up = True
                istate = 7
        elif istate == 7: # Backing up for 2s
            if rospy.get_time() - time_start > timeout:
                svea.controller.backing_up = False
                istate = 1 
                print("state 1: Mapping obstacles")
                

        # compute control input via pure pursuit
        steering, velocity = svea.compute_control()
        svea.send_control(steering, velocity)

        # visualize data
        if use_matplotlib or use_rviz:
            svea.visualize_data()
        else:
            rospy.loginfo_throttle(1, state)
        toc = rospy.get_time() # stop timer

        # Uncomment to check scan time of while loop
        #rospy.loginfo("Scan time %f", toc-tic)

        # Switching between paths in global planner
        if istate == 0 and svea.is_finished and not first_traj_complete:
            first_traj_complete = True
            svea.reset_isfinished()
            traj_x,traj_y = traj_x1, traj_y1
            svea.update_traj(traj_x,traj_y)

    if not rospy.is_shutdown():
        rospy.loginfo("Trajectory finished!")

    rospy.spin()


if __name__ == '__main__':
    main()
