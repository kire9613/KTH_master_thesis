#!/usr/bin/env python

import rospy
import numpy as np
import math
import time

from svea.svea_managers.path_following_sveas import SVEAPurePursuit
from svea.states import VehicleState
from svea.localizers import LocalizationInterface
from svea.controllers.pure_pursuit import PurePursuitController
from svea.data import BasicDataHandler, TrajDataHandler, RVIZPathHandler
from svea.models.bicycle import SimpleBicycleModel
from svea.simulators.sim_SVEA import SimSVEA
from svea_msgs.msg import emergency_break 
from svea.controllers.emergency_breaker import EmergencyBreaker
from svea.controllers.map_requester import MapRequester

from nav_msgs.msg import OccupancyGrid

#from sensor_msgs.msg import LaserScan


__team__ = "Team 3"
__maintainers__ = "Albin Larsson Forsberg, Timotheos Souroulla, Filip Hestell, Roman Landin, Filip von Reis Marlevi"
__status__ = "Development"

## SIMULATION PARAMS ##########################################################
vehicle_name = "SVEA"
target_velocity = 1.0 # [m/s]
dt = 0.01 # frequency of the model updates
emergency_breaker = EmergencyBreaker()

#TODO: create a trajectory that goes around the track


xs = [-2.44, 10.06, 9.83, 6.45, 5.28, -6.58, -6.61]
ys = [-7.24, 11.12, 12.04, 14.38, 14.22, -2.97, -4.44]

traj_x = []
traj_y = []
for i in range(len(xs)-1):
    traj_x.append(np.linspace(xs[i], xs[i+1]).tolist())
    traj_y.append(np.linspace(ys[i], ys[i+1]).tolist())
traj_x.append(np.linspace(xs[-1], xs[0]).tolist())
traj_y.append(np.linspace(ys[-1], ys[0]).tolist())

#Makes the nested lists into a one dimensional array.
traj_x = [val for sublist in traj_x for val in sublist]
traj_y = [val for sublist in traj_y for val in sublist]

###############################################################################

## INIT #######################################################################
default_init_pt = [0.0, 0.0, 0.0, 0.0] # [x, y, yaw, v], units: [m, m, rad, m/s]
###############################################################################

def param_init():
    """Initialization handles use with just python or in a launch file"""
    # grab parameters from launch-file
    start_pt_param = rospy.search_param('start_pt')
    is_sim_param = rospy.search_param('is_sim')
    use_rviz_param = rospy.search_param('use_rviz')
    use_matplotlib_param = rospy.search_param('use_matplotlib')
    obstacles_param = rospy.search_param('obstacles')

    start_pt = rospy.get_param(start_pt_param, default_init_pt)
    if isinstance(start_pt, str):
        start_pt = start_pt.split(',')
        start_pt = [float(curr) for curr in start_pt]
        start_pt = VehicleState(*start_pt)

    is_sim = rospy.get_param(is_sim_param, True)
    use_rviz = rospy.get_param(use_rviz_param, False)
    use_matplotlib = rospy.get_param(use_matplotlib_param, False)
    obstacles = rospy.get_param(obstacles_param, [])

    return start_pt, is_sim, use_rviz, use_matplotlib, obstacles

def emergency_break_callback(msg):
    if msg.emergency_break:
        emergency_breaker.enable()



def main():
    
    rospy.init_node('floor2_example')
    start_pt, is_sim, use_rviz, use_matplotlib, obstacles = param_init()
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
                            dt=dt, start_paused=True, run_lidar = True).start()

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

    
    #Test if service is correctly 
    map_requester = MapRequester()
    # simualtion loop
    svea.controller.target_velocity = target_velocity
    while not svea.is_finished and not rospy.is_shutdown():

        state = svea.wait_for_state()
        
        #Get current occupancy_grid from map_service_provider
        map_requester.update()
        occupancy_grid = map_requester.getMap()
        #a=np.array(occupancy_grid)
        #print(occupancy_grid)
        #time.sleep(1)

        #Sets the state of the emergency controller
        svea.controller.emergency_break = emergency_breaker.state()

        index =  svea.controller.last_index
        if index > 50:
            
            svea.controller.traj_x = traj_x
            svea.controller.traj_y = traj_y
        else:
            svea.controller.traj_x = traj_x[0:100]
            svea.controller.traj_y = traj_y[0:100]
        ##print(index)
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
    rospy.Subscriber('/emergency_break',
                     emergency_break,
                     emergency_break_callback)
    main()
