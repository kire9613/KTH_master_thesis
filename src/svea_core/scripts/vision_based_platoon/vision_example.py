#!/usr/bin/env python

import rospy
import matplotlib as mpl
import matplotlib.pyplot as plt

from svea.states import VehicleState
from svea.simulators.sim_SVEA import SimSVEA
from svea.models.bicycle import SimpleBicycleModel
from svea.svea_managers.path_following_sveas import SVEAPlatoonMember
from svea.localizers import LocalizationInterface
from svea.data import RVIZPathHandler
from svea.models.cooperative import C_OVRV
from vision_based_utils import *
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

from svea.aruco.aruco_interfaces import *


## C-OVRV PARAMS ##############################################################
platoon_size = 1
num_neighbors = 0 # 0 for don't use communicated info
desired_time_headway = 0.3
k1 = 1.0 #constant time-headway term
k2 = 0.5 #follow-the-leader term (i.e. match the lead vehicle velocity)
k3 = 0.0 # =0.8
k4 = 0.0 # =0.8
k_gains = [k1, k2, k3, k4]
min_spacing = 0.3
new_space = 0
dist = []
dt = 0.01
markervis = False
#artificial_spacings = False
###############################################################################

## EXPERIMENT SET UP ##########################################################
init_spacing = 0.5  # initial space between bumpers
init_velocity = 0.8  # initial target velocity
disturbance_velocity = 0.8 # experiment velocity drop

steady_state_hold_time = 12.0  # seconds

# short traj
xs = [-2.05, 14.8]
ys = [-6.87, 18.2]
traj_x = np.linspace(xs[0], xs[1]).tolist()
traj_y = np.linspace(ys[0], ys[1]).tolist()


## SVEA #######################################################################
leader_name = "SVEA0"
follower_prefix = ""
###############################################################################

## INIT #######################################################################
default_init_pt = [0.0, 0.0, 0.0, 0.0] # [x, y, yaw, v], units: [m, m, rad, m/s]
visualize_plot = True
###############################################################################


def param_init():
    """Initialization handles use with just python or in a launch file
    """
    # grab parameters from launch-file
    start_pt_param = rospy.search_param('start_pt')
    use_rviz_param = rospy.search_param('use_rviz')
    is_sim_param = rospy.search_param('is_sim')

    start_pt = rospy.get_param(start_pt_param, default_init_pt)
    if isinstance(start_pt, str):
        start_pt = start_pt.split(',')
        start_pt = [float(curr) for curr in start_pt]
    use_rviz = rospy.get_param(use_rviz_param, False)
    is_sim = rospy.get_param(is_sim_param, True)

    return start_pt, use_rviz, is_sim

def subinfo(data):
    """Callback function from PoseStamped subscriber.
    Stores distance z in global variable new_space.
    """
    space = data.pose.position.z
    global new_space
    new_space = space

def markerinfo(data):
    """Callback function from markerinfo subscriber.
    Stores boolean value in global variable markervis.
    """
    vis = data.data
    global markervis
    markervis = vis


def main():
    rospy.init_node('vision_example')
    last_vehicle_start_pt, use_rviz, is_sim = param_init()

    # subscribe to topic published in aruco_vision, marker info
    rospy.Subscriber('/observed_pose', PoseStamped, subinfo, queue_size=1)
    rospy.Subscriber('/marker_list', Bool, markerinfo, queue_size=1)

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

    # create simulated leader and followers
    leader_state = VehicleState(*leader_start_pt)
    if is_sim:
        leader_sim = SimSVEA(SimpleBicycleModel(leader_state),          #FOR SIM?
                             vehicle_name=leader_name,                  #FOR SIM?
                             dt=dt, start_paused=True).start()          #FOR SIM?

    leader = SVEAPlatoonMember(LocalizationInterface,
                               traj_x, traj_y,
                               data_handler = RVIZPathHandler,
                               vehicle_name = leader_name)
    followers = []

    for i in range(platoon_size):
        follower_name = follower_prefix # + str(1 + i)
        follower_state = VehicleState(*follower_start_pts[i])
        if is_sim:
            follower_sims = []
            follower_sim = SimSVEA(SimpleBicycleModel(follower_state),  #FOR SIM
                                   vehicle_name = follower_name,        #FOR SIM
                                   dt=dt, start_paused=True).start()    #FOR SIM
            follower_sims.append(follower_sim)                          #FOR SIM

        follower = SVEAPlatoonMember(LocalizationInterface,
                                     traj_x, traj_y,
                                     data_handler = RVIZPathHandler,
                                     vehicle_name = follower_name)
        followers.append(follower)

    # spin up svea managers so they are ready before simulation unpauses
    leader.start(wait=True)
    [follower.start(wait=True) for follower in followers]

    # unpause the simulated vehicles
    if is_sim:
        toggle_pause(leader_sim, follower_sims)
    wait_for_platoon_states(leader, followers)

    if use_rviz:
        [follower.data_handler.pub_car_poly() for follower in followers]


    # create cooperative model
    c_ovrv_model = C_OVRV(platoon_size, num_neighbors, k_gains, min_spacing,
                          desired_time_headway, init_velocity, dt=dt)

    # create unified data logs for platoon
    start_t = rospy.get_time()
    platoon_t = []
    leader_v = []
    follower_vs = [[] for follower in followers]
    follower_as = [[] for follower in followers]

    # simualtion + animation loop
    experiment_begun = False
    reaching_speed = True
    prev_t = start_t
    timer = steady_state_hold_time
    experiment_start_time = -float('inf')

    while not rospy.is_shutdown():
        # update all vehicle states, not used for non communication
        leader_state = leader.wait_for_state()
        follower_states = [follower.wait_for_state() for follower in followers]
        curr_t = rospy.get_time() - start_t

        platoon_t.append(curr_t)
        leader_v.append(leader_state.v)

        [follower_vs[i].append(follower_state.v)
         for i, follower_state in enumerate(follower_states)]

        #get spacings from camera, se to value if visible and to 0 if not
        if markervis is True:
            spacings = [new_space]
        else:
            spacings = compute_spacings(leader, followers)

        rospy.loginfo(spacings[0])
        dist.append(spacings[0])

        if experiment_begun and not reaching_speed:
            # use velocity control until reached steady state
            rospy.loginfo_once("Reaching steady state speeds")
            if is_sim:
                leader.send_vel(init_velocity)
            [follower.send_vel(init_velocity) for follower in followers]
            # keep track of latest time
            experiment_start_time = max(experiment_start_time, curr_t)
            if not reached_steady_state(init_velocity, leader, followers):
                timer = steady_state_hold_time
            else:
                timer -= curr_t - prev_t
            prev_t = curr_t
            reaching_speed = timer > 0.0 
        else:
            #rospy.loginfo_once("Beginning Experiment at t="+str(curr_t))
            experiment_begun = True

            # compute accelerations if no visiable marker
            speeds = [follower_state.v for follower_state in follower_states]
            accel_ctrls = c_ovrv_model.compute_accel(spacings, speeds,
                                                     leader_state.v)
            if is_sim:
                leader.send_vel(init_velocity)

            [follower_as[i].append(accel_ctrls)
                for i, follower_state in enumerate(follower_states)]

            #send speed adjustments to the cars
            for i, follower in enumerate(followers):
                if spacings[i] > min_spacing:
                    follower.send_accel(accel_ctrls[i], dt)
                else:
                    follower.send_vel(0.0)

        # update visualizations
        if use_rviz:
            # vizualize leader's car poly, path, and target pt
            leader.visualize_data()
            [follower.data_handler.pub_car_poly() for follower in followers]

    if not rospy.is_shutdown():
        rospy.loginfo("Trajectory finished.")

 #plottar hastighet och distance
    if visualize_plot:
        plt.plot(platoon_t, leader_v, "-", linewidth=1, label="V_L")
        [plt.plot(platoon_t, follower_v, "-", linewidth=1, label="V" + str(i))
            for i, follower_v in enumerate(follower_vs)]
        [plt.plot(platoon_t, follower_a, "-", linewidth=1, label="A" + str(i))
            for i, follower_a in enumerate(follower_as)]

        plt.plot(platoon_t, dist, "-", linewidth=1, label="D")

        plt.axvline(experiment_start_time, 0, 1,
                    linestyle="--", color="grey", alpha=0.5)
        plt.xlabel('time (s)')
        plt.ylabel('velocity (m/s), distance (m)')
        plt.ylim(-0.2, 1.8)
        plt.legend(loc="upper right")
        plt.title(str(platoon_size) +
                  " vehicle platoon with k = " + str(num_neighbors))
        plt.show()
        plt.pause(0.001)

    rospy.spin()


if __name__ == '__main__':
    main()
