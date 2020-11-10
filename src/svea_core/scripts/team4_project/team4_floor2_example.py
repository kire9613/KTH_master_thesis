#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Float32
import numpy as np

__team__ = "Team 4"
__maintainers__ = "Adam Miksits, Caroline Skoglund, Oscar Gustavsson, Ylva Modahl "
__status__ = "Development"

from svea.svea_managers.path_following_sveas import SVEAPurePursuit
from svea.states import VehicleState
from svea.localizers import LocalizationInterface
from pure_pursuit import PurePursuitController
from svea.data import BasicDataHandler, TrajDataHandler, RVIZPathHandler
from svea.models.bicycle import SimpleBicycleModel
from svea.simulators.sim_SVEA import SimSVEA
from svea.track import Track


## SIMULATION PARAMS ##########################################################
vehicle_name = "SVEA"
target_velocity = 1.0 # [m/s]
dt = 0.01 # frequency of the model updates

#TODO: create a trajectory that goes around the track
xs = [-2.33, 10.48]
ys = [-7.09, 11.71]
traj_x = np.linspace(xs[0], xs[1]).tolist()
traj_y = np.linspace(ys[0], ys[1]).tolist()
xs = [10.48, 5.88]
ys = [11.71, 14.8]
traj_x += np.linspace(xs[0], xs[1]).tolist()
traj_y += np.linspace(ys[0], ys[1]).tolist()
xs = [5.88, -6.82]
ys = [14.8, -3.72]
traj_x += np.linspace(xs[0], xs[1]).tolist()
traj_y += np.linspace(ys[0], ys[1]).tolist()
xs = [-6.82, -3.17]
ys = [-3.72, -5.97]
traj_x += np.linspace(xs[0], xs[1]).tolist()
traj_y += np.linspace(ys[0], ys[1]).tolist()


###############################################################################

## INIT #######################################################################
default_init_pt = [0.0, 0.0, 0.0, 0.0] # [x, y, yaw, v], units: [m, m, rad, m/s]
###############################################################################

# Update controller trajectory when new plan
# is pulished
def target_path_setter(svea):
    def listener(path_msg):
        traj_x = []
        traj_y = []

        for pose in path_msg.poses:
            traj_x.append(pose.pose.position.x)
            traj_y.append(pose.pose.position.y)

        svea.update_traj(traj_x, traj_y)

    return listener

# Set target velocity according ot what
# is published on /
def target_vel_setter(svea):
    def listener(vel_msg):
        svea.controller.target_velocity = vel_msg.data

    return listener

def param_init():
    """Initialization handles use with just python or in a launch file
    """
    # grab parameters from launch-file
    start_pt_param = rospy.search_param('start_pt')
    is_sim_param = rospy.search_param('is_sim')
    use_rviz_param = rospy.search_param('use_rviz')
    use_matplotlib_param = rospy.search_param('use_matplotlib')
    use_lidar = rospy.search_param('use_lidar')

    start_pt = rospy.get_param(start_pt_param, default_init_pt)
    if isinstance(start_pt, str):
        start_pt = start_pt.split(',')
        start_pt = [float(curr) for curr in start_pt]
        start_pt = VehicleState(*start_pt)

    is_sim = rospy.get_param(is_sim_param, True)
    use_rviz = rospy.get_param(use_rviz_param, False)
    use_matplotlib = rospy.get_param(use_matplotlib_param, False)
    use_lidar = rospy.get_param(use_lidar)

    return start_pt, is_sim, use_rviz, use_matplotlib, use_lidar


def main():
    rospy.init_node('floor2_example')
    start_pt, is_sim, use_rviz, use_matplotlib, use_lidar = param_init()

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
                            dt=dt, start_paused=True, run_lidar=use_lidar).start()

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
    
    # Listen for updates in the path to follow
    rospy.Subscriber('/targets', Path, target_path_setter(svea))
    # Listen for updates in the target velocity
    rospy.Subscriber('/target_vel', Float32, target_vel_setter(svea))

    if is_sim:
        # start simulation
        simulator.toggle_pause_simulation()

    # simualtion loop
    svea.controller.target_velocity = target_velocity
    while not svea.is_finished and not rospy.is_shutdown():
        state = svea.wait_for_state()

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
