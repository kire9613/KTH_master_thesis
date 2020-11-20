#!/usr/bin/env python
import rospy
import numpy as np
import yaml

from svea.svea_managers.path_following_sveas import SVEAPurePursuit
from svea.states import VehicleState
from svea.localizers import LocalizationInterface
from svea.controllers.pure_pursuit import PurePursuitController
from svea.data import BasicDataHandler, TrajDataHandler, RVIZPathHandler
from svea.models.bicycle import SimpleBicycleModel
from svea.simulators.sim_SVEA import SimSVEA
from sensor_msgs.msg import LaserScan
from svea.path_planners.astar import *
from geometry_msgs.msg import PoseWithCovarianceStamped


__team__ = "Team 5"
__maintainers__ ="Bianca Otake, Holmfridur Elvarsdottir, Johanna Andersson, Marcus Norgren"
__status__ ="Development"


## SIMULATION PARAMS ##########################################################
vehicle_name = ""
target_velocity = 1.0 # [m/s]
dt = 0.01 # frequency of the model updates

###############################################################################

## INIT #######################################################################
default_init_pt = [0.0, 0.0, 0.0, 0.0] # [x, y, yaw, v], units: [m, m, rad, m/s]
###############################################################################

def extract_trajectory(use_astar):
    if use_astar:
        xt, yt = -3.46, -6.93
        x0, y0, theta0 =  -7.4,-15.3,  0.8978652
        traj_x, traj_y = generateTrajectory(x0,y0,theta0,xt,yt,False)
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

    start_pt = rospy.get_param(start_pt_param, default_init_pt)
    if isinstance(start_pt, str):
        start_pt = start_pt.split(',')
        start_pt = [float(curr) for curr in start_pt]
        start_pt = VehicleState(*start_pt)

    is_sim = rospy.get_param(is_sim_param, True)
    use_rviz = rospy.get_param(use_rviz_param, False)
    use_matplotlib = rospy.get_param(use_matplotlib_param, False)
    use_astar = rospy.get_param(use_astar_param, True)

    return start_pt, is_sim, use_rviz, use_matplotlib, use_astar


def main():
    rospy.init_node('team_5_floor2')
    start_pt, is_sim, use_rviz, use_matplotlib, use_astar = param_init()

    # extract trajectory
    traj_x, traj_y = extract_trajectory(use_astar)
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

    if is_sim:
        # start simulation
        simulator.toggle_pause_simulation()
    else:
        initialStatePublisher = rospy.Publisher('/initialstate', PoseWithCovarianceStamped, queue_size=1)
        initialStateMsg = PoseWithCovarianceStamped()
        initialStateMsg.header.frame_id = "map"
        initialStateMsg.pose.pose.position.x = -7.4
        initialStateMsg.pose.pose.position.x = -15.3
        initialStateMsg.pose.pose.orientation.z = 0.478559974486
        initialStateMsg.pose.pose.orientation.w = 0.878054867773
        initialStateMsg.pose.covariance =  [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
        initialStatePublisher.publish(initialStateMsg)

    # simualtion loop
    svea.controller.target_velocity = target_velocity
    while not svea.is_finished and not rospy.is_shutdown():
        state = svea.wait_for_state()

        # compute control input via pure pursuit
        steering, velocity = svea.compute_control()
        # tic = rospy.get_time() # disabled emergency brake
        svea.send_control(steering, velocity)
        #toc = rospy.get_time() # disabled emergency brake
        #rospy.loginfo_throttle(0.5, toc-tic) # disabled emergency brake

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
