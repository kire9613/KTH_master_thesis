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
        xt, yt = 20,2
        x0, y0, theta0 =  0,0,0
        settings = {
        "driving_distance": 0.25,
        "use_track": True,
        "safety_distance": 0.3,
        "grid_resolution": 0.1,
        "success_threshold": 1.5,
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
    traj_x, traj_y,success = generateTrajectory(settings,x0,y0,theta0,xt,yt,True)

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

    # Get ros parameters from launch file
    start_pt, is_sim, use_rviz, use_matplotlib, use_astar, use_mpc,  use_q1 = param_init()

    # A* emergency settings 
    emergency_settings = {
        "driving_distance": 0.3,
        "use_track": False,
        "safety_distance": 0.10,
        "subscribe_to_obstacles": True,
        "grid_resolution": 0.025,
        "success_threshold": 0.5,
<<<<<<< HEAD
        "intermediate_point": False,
        "use_q1": use_q1,
        "maximum_expansion": 1000
=======
        "maximum_expansion": 1000,
        "intermediate_point": False,
        "use_q1": use_q1
>>>>>>> deb4f2b... A* adaptions for Q1 racing track
        }
    
    # extract trajectory
    traj_x, traj_y = extract_trajectory(use_astar, use_q1)
    traj_theta = compute_angles(traj_x,traj_y)
  
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

    # initialize ros interface
    ros_interface = ROSInterface(traj_x,traj_y)
    ros_interface.set_goal_angles(traj_theta)

    # Subscribe to initial state
    rospy.Subscriber('/state', VehicleStateMsg, ros_interface.cb_initial_state)  # noqa
    # Subscribe to Astar position
    rospy.Subscriber('/target', PointStamped, ros_interface.cb_target_state)
    # simualtion loop
    svea.controller.target_velocity = target_velocity

    rospy.Subscriber('/scan', LaserScan,svea.controller.emergency_stop)

    while (not svea.is_finished and not rospy.is_shutdown()) or svea.controller.emg_traj_running:
        tic = rospy.get_time() # get time
        state = svea.wait_for_state()
  
        if istate == 0: # IDLE state - waits here untill emergency stop
            svea.controller.target_velocity = target_velocity
            if svea.controller.emg_stop:
                
                print("state 1")
                istate = 1 
        elif istate == 1: # Emergency stop activated - mapping obstacles
            svea.controller.target_velocity = target_velocity*0.5
            if ros_interface.current_speed < 0.01:
                svea.controller.laser_mapping(ros_interface.initial_state)
                if use_mpc:
                    print("state 2")
                    istate = 2
                elif use_astar:
                    print("state 3")
                    istate = 3
                else:
                    print("state 0")
                    istate = 0
                
        elif istate == 2: # Replan with MPC
            if  ros_interface._current_target_state != [0,0] and ros_interface.initial_state != None:
                svea.controller.set_emg_traj_running(True)   
                g_traj_x, g_traj_y, success = run_mpc_planner(ros_interface)
                print("MPC Replanning Trajectory:")
                if success:
                    print("traj_x",g_traj_x)
                    print("traj_y",g_traj_y)
                    print("state 4")
                    svea.update_traj(g_traj_x, g_traj_y)
                    svea.controller.emergency_distance = 0.25
                    svea.controller.emg_stop = False
                    istate = 4
                elif not backup_attempted:
                    istate = 5
                    print("Backing up")
                else: # do something here if fails
                    replan_counter = replan_counter + 1
                    svea.controller.set_emg_traj_running(False)
                    if replan_counter >= 2:
                        print("Replanned twice and failed!")
                        break  
                    else:
                        print("Planning Failed - Replan with Astar")
                        istate = 3
                  
        elif istate == 3: # Replan with Astar
            if  ros_interface._current_target_state != [0,0]  and ros_interface.initial_state != None:
                svea.controller.set_emg_traj_running(True)
                rospy.wait_for_message('/target',PointStamped)
                ros_interface.compute_goal()
                x0, y0, theta0 = ros_interface.initial_state
                xt,yt,thetat = ros_interface.goal_state
                g_traj_x, g_traj_y,success = generateTrajectory(emergency_settings,x0,y0,theta0,xt,yt,True)# False
                
                print("Astar Replanning Trajectory:")
                if success:
                    svea.controller.emergency_distance = 0.25
                    svea.controller.emg_stop = False
                    svea.update_traj(g_traj_x, g_traj_y)
                    print("state 4")
                    istate = 4
                elif not backup_attempted:
                    istate = 5
                    print("Backing up")
                else: # do something here if fails
                    replan_counter = replan_counter + 1
                    svea.controller.set_emg_traj_running(False)  
                    if replan_counter >= 2:
                        print("Replanned twice and failed!")
                        break  
                    #else:
                    #    print("Planning Failed - Replan with MPC")
                    #    istate = 2

        elif istate == 4: # Follow replanned path
            if svea.controller.emg_stop:
<<<<<<< HEAD
		svea.update_traj(traj_x,traj_y)
=======
                svea.update_traj(traj_x, traj_y)
>>>>>>> 6204d06... small tuning changes to enhance performance
                print("emg stop in replan")
                istate = 5 
             
            if  svea.is_finished:
                replan_counter = 0
                backup_attempted = False
                svea.controller.emergency_distance = 1.0
                svea.reset_isfinished() # sets is_finished to false
                # extract trajectory
                print("Switching back to global path")
                svea.update_traj(traj_x, traj_y)
                svea.controller.set_emg_traj_running(False) 
                print("state 0")   
                istate = 0
        elif istate == 5: # Initialize backup
            svea.controller.target_velocity = target_velocity
            backup_attempted = True
            svea.controller.backing_up = True     
            time_start = rospy.get_time()
            print("Back up pulse")
            istate = 6
        elif istate == 6: # pulse signal for 0.5 s on, 0.5 s off
            timeout = 0.5 
            if rospy.get_time() - time_start > 0.5:
                svea.controller.backing_up = False
            if rospy.get_time() - time_start > 1:
                svea.controller.backing_up = True
                print ("Backing up")
                istate = 7
        elif istate == 7: # Backing up for 2s
            timeout = 2
            if rospy.get_time() > time_start + timeout:
                svea.controller.backing_up = False
                istate = 1 # Go to back to obstacle mapping
                print("state 1")
                print("Mapping obstacles")
        elif istate == 8: # Astar replanning for shorter periods
            if  svea.is_finished:
                svea.controller.target_velocity = 0
                svea.reset_isfinished()
                if ros_interface.current_speed < 0.01:
                    svea.controller.laser_mapping(ros_interface.initial_state)
                g_traj_x, g_traj_y,success = generateTrajectory(emergency_settings,x0,y0,theta0,xt,yt,True)# False
                print("Astar Replanning Trajectory again:")
                if success:
                    svea.update_traj(g_traj_x, g_traj_y)
                    print("state 4")
                    istate = 4
                else: # do something here if fails
                    replan_counter = replan_counter + 1
                    svea.controller.set_emg_traj_running(False)  
                    if replan_counter >= 2:
                        print("Replanned twice and failed!")
                        break  
                    else:
                        print("Planning Failed - Replan with MPC")
                        istate = 2

                

        # compute control input via pure pursuit
        steering, velocity = svea.compute_control()

        svea.send_control(steering, velocity)

        # visualize data
        if use_matplotlib or use_rviz:
            svea.visualize_data()
        else:
            rospy.loginfo_throttle(1, state)
        toc = rospy.get_time() # stop timer
        #rospy.loginfo("Scan time %f", toc-tic) # scan time of while loop

    if not rospy.is_shutdown():
        rospy.loginfo("Trajectory finished!")

    rospy.spin()


if __name__ == '__main__':
    main()
