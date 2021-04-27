#!/usr/bin/env python

import math
import rospy
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

from svea.states import VehicleState
from svea.simulators.sim_SVEA import SimSVEA
from svea.models.bicycle import SimpleBicycleModel
from svea.svea_managers.path_following_sveas import SVEAPlatoonMember
from svea.localizers import LocalizationInterface
from svea.data import RVIZPathHandler
from svea.models.cooperative import C_OVRV
from c_ovrv_utils_MPC import *
#from visMPC import MPC_controller
from std_msgs.msg import Float64, String
from copy import deepcopy

mpl.style.use('seaborn')

## C-OVRV PARAMS ##############################################################
platoon_size = 1
num_neighbors = 2 # 0 for don't use communicated info
desired_time_headway = 0.3
k1 = 0.5
k2 = 0.5
k3 = 0.8
k4 = 0.8
k_gains = [k1, k2, k3, k4]
min_spacing = 0.3
dt = 0.01
###############################################################################

## EXPERIMENT SET UP ##########################################################
init_spacing = 0.5  # initial space between bumpers
init_velocity = 0.4  # initial target velocity
disturbance_velocity = 0.4 # experiment velocity drop

steady_state_hold_time = 12.0  # seconds

# trajectory
xs = [-2.05, 14.8]
ys = [-6.87, 18.2]
traj_x = np.linspace(xs[0], xs[1]).tolist()
traj_y = np.linspace(ys[0], ys[1]).tolist()
###############################################################################

## SVEA #######################################################################
leader_name = "SVEA0"
###############################################################################

## INIT #######################################################################
default_init_pt = [0.0, 0.0, 0.0, 0.0] # [x, y, yaw, v], units: [m, m, rad, m/s]
visualize_plot = True
###############################################################################

dMPCdata = 0.5
vrelMPCdata = 0.0
accelMPCdata = 0.0
leaderpitch = 0.0

dist_list = []
MPC_weights = []

def param_init():
    """Initialization handles use with just python or in a launch file
    """
    # grab parameters from launch-file
    start_pt_param = rospy.search_param('start_pt')
    use_rviz_param = rospy.search_param('use_rviz')
    is_sim_param = rospy.search_param('is_sim')
    use_camera_param = rospy.search_param('use_camera')

    start_pt = rospy.get_param(start_pt_param, default_init_pt)

    if isinstance(start_pt, str):
        start_pt = start_pt.split(',')
        start_pt = [float(curr) for curr in start_pt]
    use_rviz = rospy.get_param(use_rviz_param, False)
    is_sim = rospy.get_param(is_sim_param, True)
    use_camera = rospy.get_param(use_camera_param, True)


    return start_pt, use_rviz, is_sim, use_camera

def accelMPCinfo(data):
    global accelMPCdata
    accelMPCdata = data.data 

def weightsMPCinfo(data):
    global MPC_weights
    MPC_weights = data.data

def markerinfo(data):
    global marker
    marker = data

def leaderpitchinfo(data): 
    global leaderpitch
    leaderpitch = data.data


def main():
    rospy.init_node('c_ovrv_MPC')
                                            
    rospy.Subscriber('/accelMPC_publisher', Float64, accelMPCinfo, queue_size=1)
    rospy.Subscriber('/MPC_weights', String, weightsMPCinfo, queue_size=1)
    
    last_vehicle_start_pt, use_rviz, is_sim, use_camera = param_init()

    starting_distance_reached = False

    if is_sim:
        follower_prefix = "SVEA1"
    else:
        follower_prefix = ""

    # compute initial positions, these correspond with initial SVEA placement
    init_spacings = [init_spacing for _ in range(platoon_size)]
    leader_start_pt, follower_start_pts = \
        compute_positions_from_spacings(last_vehicle_start_pt, init_spacings)

    ## For testing ############################################################
    c_ovrv_model = C_OVRV(platoon_size, num_neighbors, k_gains, min_spacing,
                          desired_time_headway, init_velocity, dt=dt)
    init_eq_pt = c_ovrv_model.equilibrium_pt
    leader_start_pt, follower_start_pts = \
        compute_positions_from_spacings(follower_start_pts[-1],
                                        init_eq_pt[:platoon_size])
    ###########################################################################
    
    if not use_camera:
        # create simulated leader and followers
        leader_state = VehicleState(*leader_start_pt)
        leader_sim = SimSVEA(SimpleBicycleModel(leader_state),
                             vehicle_name=leader_name,
                             dt=dt, start_paused=True).start()
        leader = SVEAPlatoonMember(LocalizationInterface,
                               traj_x, traj_y,
                               data_handler = RVIZPathHandler,
                               vehicle_name = leader_name)
    else:
        rospy.Subscriber('/Float64', Float64, leaderpitchinfo, queue_size=1)

    follower_sims = []
    followers = []                                    

    for i in range(platoon_size):
        follower_name = follower_prefix # + str(1 + i)
        follower_state = VehicleState(*follower_start_pts[i])

        if is_sim:
            follower_sim = SimSVEA(SimpleBicycleModel(follower_state),
                                   vehicle_name = follower_name,
                                   dt=dt, start_paused=True).start()
            follower_sims.append(follower_sim)

        follower = SVEAPlatoonMember(LocalizationInterface,
                                     traj_x, traj_y,
                                     data_handler = RVIZPathHandler,
                                     vehicle_name = follower_name)
        
        followers.append(follower)

    [follower.start(wait=True) for follower in followers]

    if not use_camera:
        # spin up svea managers so they are ready before simulation unpauses
        leader.start(wait=True)
        # unpause the simulated vehicles
        toggle_pause(leader_sim, follower_sims)
        wait_for_platoon_states(leader, followers)

    if use_rviz:
        if not use_camera:
            leader.data_handler.pub_car_poly()
        [follower.data_handler.pub_car_poly() for follower in followers]

    # create cooperative model
    c_ovrv_model = C_OVRV(platoon_size, num_neighbors, k_gains, min_spacing,
                          desired_time_headway, init_velocity, dt=dt)
    init_eq_pt = c_ovrv_model.equilibrium_pt
    print("the initial eq points", init_eq_pt)
    leader_eq_pt, follower_eq_pts = \
        compute_positions_from_spacings(follower_start_pts[-1],
                                        init_eq_pt[:platoon_size])

    # get each vehicle into close-to-equilibrium positions
    rospy.loginfo("Going to initial equilibrium positions")
    #goto_eq_positions(leader, leader_eq_pt, followers, follower_eq_pts)
    

    # create unified data logs for platoon
    start_t = rospy.get_time()
    platoon_t = []
    leader_v = []
    accelerations = []
    follower_vs = [[] for follower in followers]

    leaderalphas = []
    
    

    # simualtion + animation loop
    experiment_begun = False
    reaching_speed = True
    prev_t = start_t
    timer = steady_state_hold_time
    experiment_start_time = -float('inf')
    rate = rospy.Rate(30)

    while not leader.is_finished and not rospy.is_shutdown():       
        # update all vehicle states
        leader_state = leader.wait_for_state()
        follower_states = [follower.wait_for_state() for follower in followers]
        curr_t = rospy.get_time() - start_t

         #[for i, follower_state in enumerate(follower_states)]
        
        spacings_check = compute_spacings(leader, followers)
        distance_check = spacings_check[0]
        if distance_check < 2.5 and not starting_distance_reached:
            starting_distance_reached = True
            rospy.loginfo("Starting distance achieved")
            timer = steady_state_hold_time

        if starting_distance_reached:
            platoon_t.append(curr_t)
            leader_v.append(leader_state.v)
            follower_vs[0].append(follower_state.v)
            leaderalphas.append(leaderpitch)

            if False and not experiment_begun and reaching_speed:
                # use velocity control until reached steady state
                rospy.loginfo_once("Reaching steady state speeds")

                leader.send_vel(init_velocity)

                [follower.send_vel(init_velocity) for follower in followers]
                # keep track of latest time
                experiment_start_time = max(experiment_start_time, curr_t)
                if not reached_steady_state(init_velocity, leader, followers):
                    timer = steady_state_hold_time
                else:
                    timer -= curr_t - prev_t
                timer -= curr_t - prev_t
                prev_t = curr_t
                reaching_speed = timer > 0.0
                # saving platooning distance for plotting
                spacings = compute_spacings(leader, followers)
                dist = spacings[0] 
                dist_list.append(dist)
                
                accelerations.append(0)
            else:
                rospy.loginfo_once("Beginning Experiment at t="+str(curr_t))
                if not experiment_begun:
                    v_ref = 0.
                experiment_begun = True
                # compute accelerations
                spacings = compute_spacings(leader, followers)
                #print('HEJ')
                # saving platooning distance for plotting
                dist = spacings[0] 
                dist_list.append(dist)


                leader_state = leader.wait_for_state()
                follower_states = [follower.wait_for_state() for follower in followers]
                
                speeds = [follower_state.v for follower_state in follower_states]
                #accel_ctrls = c_ovrv_model.compute_accel(spacings, speeds,
                #                                         leader_state.v)       
                if use_camera:
                    leaderalphas.append(leaderpitch)                

                for i, follower in enumerate(followers):
                    if spacings[i] > min_spacing:
                        v_ref += accelMPCdata*1.0/30
                        v_ref = max(0,v_ref)
                        v_ref = min(1.5,v_ref)
                        #print('Sending MPC v to SVEA: ' + str(v_ref))
                        accelerations.append(accelMPCdata)

                        follower.send_raw_vel(v_ref)

                    else:
                        follower.send_raw_vel(0.0)

                # creating slow down for leader
                leader.send_vel(disturbance_velocity)

                # update visualizations
        if use_rviz:
            # vizualize leader's car poly, path, and target pt
            leader.visualize_data()
            [follower.data_handler.pub_car_poly() for follower in followers]
        rate.sleep()

    if not rospy.is_shutdown():
        rospy.loginfo("Trajectory finished.")

    if True:
        if not use_camera:
            plt.plot(platoon_t, leader_v, "-", linewidth=1, label="V_L (m/s)")
        
        [plt.plot(platoon_t, follower_v, "-", linewidth=1, label="V_" + str(i) + " (m/s)")
            for i, follower_v in enumerate(follower_vs)]
        plt.plot(platoon_t, dist_list, "--", linewidth=1, label="D (m)")
        plt.plot(platoon_t, accelerations, "--", linewidth=1, label="a (m/s^2)")
        plt.axvline(experiment_start_time, 0, 1,
                    linestyle="--", color="grey", alpha=0.5)
        plt.xlabel('time (s)')
        plt.ylabel('magnitude')
        plt.ylim(-0.2, 1.8)
        plt.legend(loc="upper right")
        plt.title("Vehicle platoon with MPC weights "+str(MPC_weights))
        plt.show() 
        plt.pause(1)
        

    rospy.spin()


if __name__ == '__main__':
    main()
