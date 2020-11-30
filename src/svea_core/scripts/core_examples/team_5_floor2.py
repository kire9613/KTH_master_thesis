#!/usr/bin/env python
import rospy
import numpy as np
import yaml

from svea.svea_managers.path_following_sveas import SVEAPurePursuit
from svea.states import VehicleState
from svea_msgs.msg import VehicleState as VehicleStateMsg
from svea.localizers import LocalizationInterface
from svea.controllers.pure_pursuit import PurePursuitController
from svea.data import BasicDataHandler, TrajDataHandler, RVIZPathHandler
from svea.models.bicycle import SimpleBicycleModel
from svea.simulators.sim_SVEA import SimSVEA
from sensor_msgs.msg import LaserScan
from svea_msgs.msg import VehicleState as VSM
from svea.path_planners.astar import *
from geometry_msgs.msg import PoseWithCovarianceStamped
from svea.track import Track
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PointStamped
from nav_msgs.msg import Path
from svea.path_planners.mpc_map.occupancy_grid import OccupancyGrid
from svea.path_planners.mpc_map.ros_interface import ROSInterface as MapROSInterface
from svea.path_planners.mpc_planner.main_planner import Planner
from svea.path_planners.mpc_planner.ros_interface import ROSInterface
from svea.path_planners.mpc_planner.t5_mpc_functions import *
#from svea.path_planners.astar import ControlState
__team__ = "Team 5"
__maintainers__ ="Bianca Otake, Holmfridur Elvarsdottir, Johanna Andersson, Marcus Norgren"
__status__ ="Development"


## SIMULATION PARAMS ##########################################################
vehicle_name = ""
target_velocity = 1.0 # [m/s]
dt = 0.01 # frequency of the model updates

## PLANNER PARAMS ##########################################################
g_traj_x, g_traj_y = [],[]
timer1 = 0.0 # s
timer2 = 0.0 # s
## INIT #######################################################################
default_init_pt = [0.0, 0.0, 0.0, 0.0] # [x, y, yaw, v], units: [m, m, rad, m/s]
############################################################################### 

def extract_trajectory(use_astar):
    if use_astar:
        xt, yt = -3.46, -6.93
        x0, y0, theta0 =  -7.4,-15.3, 0.8978652
        traj_x, traj_y,success = generateTrajectory(x0,y0,theta0,xt,yt,0.05,False,True)
        traj_x.reverse()
        traj_y.reverse()
    else:
        xs = [-7.4 , -2.33, 10.3, 5.9, -7.2]
        ys = [-15.3,  -7.09, 11.4, 14.8, -4.2]
        traj_x = []
        traj_y = []
        for i in range(0,len(xs)-1):
            traj_x += np.linspace(xs[i], xs[i+1]).tolist()
            traj_y += np.linspace(ys[i], ys[i+1]).tolist()
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
    
    return start_pt, is_sim, use_rviz, use_matplotlib, use_astar, use_mpc

def main():
    global timer1,timer2
    rospy.init_node('team_5_floor2')
    start_pt, is_sim, use_rviz, use_matplotlib, use_astar, use_mpc = param_init()
    running_emg_traj = False
    istate = 0
    print_time = 0
    # extract trajectory
    traj_x, traj_y = extract_trajectory(use_astar)
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

    initialized = False

    # initialize ros interface
    ros_interface = ROSInterface(traj_x,traj_y)
    ros_interface.set_goal_angles(traj_theta)
    # Subscribe to initial state
    rospy.Subscriber('/state', VehicleStateMsg, ros_interface.cb_initial_state)  # noqa
    # (!) subscribe to Astar position
    rospy.Subscriber('/target', PointStamped, ros_interface.cb_target_state)
    # simualtion loop
    svea.controller.target_velocity = target_velocity
    #Can maybe be exchanged once we have Frank's emergency stop
    rospy.Subscriber('/scan', LaserScan,svea.controller.emergency_stop)

    while (not svea.is_finished and not rospy.is_shutdown()) or svea.controller.emg_traj_running:
        tic = rospy.get_time() # get time
        state = svea.wait_for_state()
  
        if istate == 0: # IDLE state - waits here untill emergency stop
            if svea.controller.emg_stop:
                print("state 1")
                istate = 1
        elif istate == 1: # Emergency stop activated - mapping obstacles
            if ros_interface.current_speed < 0.05:
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
                timer1 = rospy.get_time()
                print("trigger! Replan is true")
                g_traj_x, g_traj_y, success = run_mpc_planner(ros_interface)
                
                print('Plan ready!', len(g_traj_x))
                #rospy.Subscriber('/planner/mpc_path', Path, mpc_path_cb)
                #Update trajectory
                print("MPC Trajectory:")
                print("traj_x",g_traj_x)
                print("traj_y",g_traj_y)
                if success:
                    svea.update_traj(g_traj_x, g_traj_y)
                    print("state 4")
                    istate = 4
                else: # do something here if fails
                    svea.controller.set_emg_traj_running(False)  
                    print("state 0")
                    istate = 0   
                  
        elif istate == 3: # Replan with Astar
            if  ros_interface._current_target_state != [0,0]  and ros_interface.initial_state != None:
                svea.controller.set_emg_traj_running(True)
                ros_interface.compute_goal()
                x0, y0, theta0 = ros_interface.initial_state
                xt,yt,thetat = ros_interface.goal_state
                g_traj_x, g_traj_y,success = generateTrajectory(x0,y0,theta0,xt,yt,0.02,False,False)
                g_traj_x.reverse()
                g_traj_y.reverse()
                
                print("Astar Replanning Trajectory:")
                print("traj_x",g_traj_x)
                print("traj_y",g_traj_y)
                if success:
                    svea.update_traj(g_traj_x, g_traj_y)
                    print("state 4")
                    istate = 4
                else: # do something here if fails
                    svea.controller.set_emg_traj_running(False)  
                    print("state 0")
                    istate = 0
        elif istate == 4: # Follow replanned path
                  
            if  svea.is_finished:
                svea.controller.emg_stop = False
                svea.reset_isfinished() # sets is_finished to false
                # extract trajectory
                print("Switching back to Astar")
                svea.update_traj(traj_x, traj_y)
                svea.controller.set_emg_traj_running(False) 
                print("state 0")
                istate = 0
            
        # compute control input via pure pursuit
        steering, velocity = svea.compute_control()

        
        last_time = rospy.get_time()
        if (last_time - print_time)  > 1:
            #print("velocity",velocity)
            print_time = last_time  

        svea.send_control(steering, velocity)

        # visualize data
        if use_matplotlib or use_rviz:
            svea.visualize_data()
        else:
            rospy.loginfo_throttle(1, state)
        toc = rospy.get_time() # stop timer
        #rospy.loginfo("Scan time %f", toc-tic) # disabled emergency brake

    if not rospy.is_shutdown():
        rospy.loginfo("Trajectory finished!")

    rospy.spin()


if __name__ == '__main__':
    main()
