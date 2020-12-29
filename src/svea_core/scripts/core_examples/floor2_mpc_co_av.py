#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division

import rospy
import numpy as np

from svea.states import VehicleState
from svea.localizers import LocalizationInterface
from svea.data import BasicDataHandler, TrajDataHandler, RVIZPathHandler
from svea.models.bicycle import SimpleBicycleModel
from svea.simulators.sim_SVEA import SimSVEA
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
from std_msgs.msg import Bool

from svea.svea_managers.mpc_path_following_sveas import SVEAMPC
from svea.controllers.mpc.mpc import MPC
from svea.controllers.mpc.parameters import parameters

"""
__team__ = "Team 1"
__maintainers__ = "Roberto Castro Sundin, Astrid Lindstedt, Johan Hedin, Aravind Sadashiv, Sarthak Manocha”
__status__ = "Development"
"""

## SIMULATION PARAMS ##########################################################
# param_name = "simulation"
param_name = "ZOH-good"
params = parameters.get(param_name)

vehicle_name = ""
target_velocity = params.target_velocity# [m/s]
dt = params.dt # frequency of the model updates

traj_x_init = [15.1, 15.399999999999993, 15.699999999999987, 15.99999999999998, 16.300000000000026, 16.600000000000072, 16.90000000000012, 17.200000000000166, 17.500000000000213, 17.80000000000026, 18.100000000000307, 18.400000000000354, 18.7000000000004, 19.000000000000448, 19.300000000000495, 19.60000000000054, 19.90000000000059, 20.200000000000635, 20.500000000000682, 20.80000000000073, 21.100000000000776, 21.400000000000823, 21.70000000000087, 22.000000000000917, 22.300000000000963, 22.60000000000101, 22.900000000001057, 23.200000000001104, 23.50000000000115, 23.800000000001198, 24.100000000001245, 24.40000000000129, 24.70000000000134, 25.000000000001386, 25.300000000001432, 25.60000000000148, 25.900000000001526, 26.200000000001573, 26.50000000000162, 26.800000000001667, 27.100000000001714, 27.40000000000176, 27.700000000001808, 28.000000000001855, 28.3000000000019, 28.60000000000195, 28.900000000001995, 29.200000000002042, 29.50000000000209, 29.800000000002136, 30.100000000002183, 30.40000000000223, 30.700000000002277, 31.000000000002323, 31.30000000000237, 31.600000000002417, 31.900000000002464, 32.20000000000244, 32.50000000000238, 32.80000000000232, 33.10000000000226, 33.4000000000022, 33.70000000000214, 33.99943366737961, 34.2988077805138, 34.59880778051374, 34.89824144789121, 35.197615561025394, 35.497615561025334, 35.89704922840281, 36.19623533953543, 36.48506204754969, 36.74532344436383, 36.96061420825326, 37.1173636972051, 37.20569136101907, 37.22002955209248, 37.21123133977523, 37.21935112063529, 37.227470901495344, 37.2355906823554, 37.24371046321546, 37.251830244075514, 37.25995002493557, 37.268069805795626, 37.27618958665568, 37.28430936751574, 37.264166076723434, 37.162600013476364, 36.99413307575322, 36.76938441163795, 36.50252083260208, 36.21036382244711, 35.911329211581766, 35.613338971080594, 35.31379347436366, 35.01424797764673, 34.7147024809298, 34.41515698421287, 34.117047316864394, 33.81905707636322, 33.51951157964629, 33.221401912297814, 32.62356129495595, 32.323710419303865, 32.02386004246317, 31.72400916681109, 31.42415878997039, 31.124307914318308, 30.82445753747761, 30.524606661825526, 30.224756284984828, 29.924905409332744, 29.625055032492046, 29.325204156839963, 29.025353779999264, 28.72550290434718, 28.425652527506482, 28.1258016518544, 27.8259512750137, 27.526100399361617, 27.22625002252092, 26.926399146868835, 26.626548770028137, 26.326697894376053, 26.026847517535355, 25.72699664188327, 25.427146265042573, 25.12729538939049, 24.82744501254979, 24.5278845514912, 24.228033675839118, 23.92818329899842, 23.62862283793983, 23.328771962287746, 23.028921585447048, 22.729361124388458, 22.429510248736374, 22.129659871895676, 21.830099410837086, 21.530538949778496, 21.230688074126412, 20.930837697285714, 20.631277236227124, 20.33142636057504, 20.031575983734342, 19.732015522675752, 19.43216464702367, 19.13231427018297, 18.83275380912438, 18.532902933472297, 18.2330525566316, 17.93349209557301, 17.633641219920925, 17.333790843080227, 17.034230382021637, 16.734379506369553, 16.434529129528855, 16.134968668470265, 15.83511779281818, 15.236740012887525, 14.952650355568135, 14.700905756759507, 14.497374690182433, 14.354886536870813, 14.2824228977291, 14.263909938035656, 14.262300624506018, 14.26069131097638, 14.259081997446742, 14.257472683917104, 14.255863370387466, 14.254254056857828, 14.25264474332819, 14.235221190727412, 14.216708231033968, 14.21509891750433, 14.29767536490355, 14.299803914308374, 14.376390481236983, 14.522607506605985, 14.729238343720361, 14.983258220552536, 15.26865524346449, 15.56743969120897, 15.866556369699373, 16.165673048189745, 16.464789726680095, 16.763906405170445, 17.063023083660795, 17.362139762151145, 17.661256440641495, 17.960373119131845, 18.259489797622194, 18.558606476112544, 18.857723154602894, 19.156839833093244, 19.45660476554548, 19.756393882802122, 20.055510561292472]
traj_y_init = [1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.524182495318074, 1.5072752483923575, 1.5072752483923575, 1.4914577437104315, 1.474550496784715, 1.474550496784715, 1.458732992102789, 1.4624665427491146, 1.540603609706744, 1.6882188996446383, 1.8960076276329458, 2.1508720351292245, 2.436746994333875, 2.7356126577544804, 3.0353347057814974, 3.3352248009143373, 3.635114896047177, 3.935004991180017, 4.234895086312857, 4.534785181445697, 4.834675276578537, 5.134565371711377, 5.434455466844216, 5.734345561977056, 6.332107218820423, 6.61355100235736, 6.860826475037693, 7.058346870120075, 7.193661683349326, 7.25824147827309, 7.248015530897718, 7.214660903339672, 7.1981534687549225, 7.181646034170173, 7.165138599585424, 7.148631165000674, 7.116361351755743, 7.083006724197697, 7.066499289612947, 7.034229476368016, 7.00019211754351, 7.0006007545086675, 6.999918023242208, 7.0003266602073655, 6.999643928940906, 7.0000525659060635, 6.999369834639604, 6.9997784716047615, 6.999095740338302, 6.9995043773034595, 6.998821646037, 6.9992302830021575, 6.998547551735698, 6.998956188700856, 6.998273457434396, 6.998682094399554, 6.997999363133094, 6.998408000098252, 6.997725268831792, 6.99813390579695, 6.99745117453049, 6.997859811495648, 6.997177080229188, 6.997585717194346, 6.996902985927886, 6.997311622893044, 6.996628891626584, 7.01286250399033, 7.013271140955487, 7.012588409689028, 7.028822022052773, 7.02923065901793, 7.028547927751471, 7.044781540115216, 7.045190177080373, 7.044507445813914, 7.060741058177659, 7.0769746705414045, 7.077383307506562, 7.0767005762401025, 7.092934188603848, 7.093342825569005, 7.092660094302546, 7.108893706666291, 7.109302343631448, 7.108619612364989, 7.124853224728734, 7.125261861693891, 7.124579130427432, 7.140812742791177, 7.141221379756335, 7.140538648489875, 7.156772260853621, 7.157180897818778, 7.156498166552319, 7.172731778916064, 7.173140415881221, 7.152267139811217, 7.058355824735517, 6.896643346967788, 6.677323093837591, 6.414219702257512, 6.123917636303039, 5.824638527548555, 5.524642844063014, 5.224647160577473, 4.924651477091932, 4.624655793606391, 4.32466011012085, 4.024664426635309, 3.7246687431497554, 3.4253242352190805, 3.1260451264645956, 2.8260494429790413, 2.5267049350483664, 2.2275031005410773, 1.9382613914789386, 1.6772118751274692, 1.4608095515528163, 1.3026951288417232, 1.2128351944544293, 1.1968939811654413, 1.2198986035024852, 1.2429032258395292, 1.265907848176573, 1.288912470513617, 1.311917092850661, 1.3349217151877049, 1.3579263375247488, 1.3809309598617927, 1.4039355821988366, 1.4269402045358806, 1.4499448268729245, 1.4729494492099684, 1.4801397124451408, 1.4862388753361506, 1.5092434976731945]


###############################################################################

## INIT #######################################################################
default_init_pt = [0.0, 0.0, 0.0, 0.0] # [x, y, yaw, v], units: [m, m, rad, m/s]
###############################################################################

class Node:
    def __init__(self):
        rospy.init_node('floor2_mpc_co_av')

        # grab parameters from launch-file
        start_pt_param = rospy.search_param('start_pt')
        is_sim_param = rospy.search_param('is_sim')
        use_rviz_param = rospy.search_param('use_rviz')
        use_matplotlib_param = rospy.search_param('use_matplotlib')
        run_lidar_param = rospy.search_param('run_lidar')

        start_pt = rospy.get_param(start_pt_param, default_init_pt)
        if isinstance(start_pt, str):
            start_pt = start_pt.split(',')
            start_pt = [float(curr) for curr in start_pt]
            start_pt = VehicleState(*start_pt)

        self.is_sim = rospy.get_param(is_sim_param, True)
        self.use_rviz = rospy.get_param(use_rviz_param, False)
        self.use_matplotlib = rospy.get_param(use_matplotlib_param, False)
        self.run_lidar = rospy.get_param(run_lidar_param, True)

        #This subscriber and its callback function is the local planner
        traj_upd_sub = rospy.Subscriber('/trajectory_updates', Path, self.callback_traj)
        collision_sub = rospy.Subscriber('/collision', Bool, self.callback_collision)
        # select data handler based on the ros params
        if self.use_rviz:
            self.DataHandler = RVIZPathHandler
        elif self.use_matplotlib:
            self.DataHandler = TrajDataHandler
        else:
            # DataHandler = BasicDataHandler
            self.DataHandler = RVIZPathHandler

        if self.is_sim:
            # start the simulation
            model_for_sim = SimpleBicycleModel(start_pt)
            self.simulator = SimSVEA(vehicle_name, model_for_sim,
                                dt=dt, start_paused=True, run_lidar=self.run_lidar).start()

        self.collision = False

        self.traj_x = traj_x_init
        self.traj_y = traj_y_init

    def callback_traj(self,path):
        self.traj_x = [i.pose.position.x for i in path.poses]
        self.traj_y = [i.pose.position.y for i in path.poses]

        self.svea.update_traj(self.traj_x, self.traj_y)

    def callback_collision(self, data):
        # print(data.data)
        self.collision = data.data

    def run(self):
        self.svea = SVEAMPC(
            vehicle_name,
            LocalizationInterface,
            MPC,
            self.traj_x, self.traj_y,
            data_handler = self.DataHandler,
            target_velocity=params.target_velocity,
            dl = dt*params.low_lim,
            low_lim = params.low_lim,
        )

        ulb = [-1e2,-np.deg2rad(40)]
        uub = [ 1e2, np.deg2rad(40)]
        xlb = [-np.inf]*3+[-1]
        xub = [ np.inf]*3+[3.6]

        # self.svea.controller.build_solver(
        #     dt,
        #     Q=params.Q,
        #     R=params.R,
        #     P=params.P,
        #     ulb=ulb,
        #     uub=uub,
        #     xlb=xlb,
        #     xub=xub,
        #     max_cpu_time=0.8*dt,
        #     horizon=params.horizon,
        #     model_type=params.model_type,
        #     solver_=params.solver_,
        #     TAU = params.TAU,
        #     N_IND_SEARCH = params.N_IND_SEARCH,
        # )
        self.svea.start(wait=True)

        if self.is_sim:
            # start simulation
            self.simulator.toggle_pause_simulation()

        # simualtion loop

        self.svea.controller.target_velocity = target_velocity
        self.svea.pid.target_velocity = target_velocity
        self.svea.pid.k = 0.5#0.6  # look forward gain
        self.svea.pid.Lfc = 0.5#0.4  # look-ahead distance
        self.svea.pid.K_p = 1.0  # speed control propotional gain
        self.svea.pid.K_i = 0.2  # speed control integral gain
        self.svea.pid.K_d = 0.0  # speed control derivitive gain
        self.svea.pid.L = 0.324  # [m] wheel base of vehicle
        while not self.svea.is_finished and not rospy.is_shutdown():
            state = self.svea.wait_for_state()

            #This step updates the global path
            #self.svea.update_traj(self.traj_x, self.traj_y)

            # compute control input
            if self.collision:
                steering, _ = self.svea.compute_pid_control()
                self.svea.send_control(steering, 0)

            else:
                steering, velocity = self.svea.compute_pid_control()
                self.svea.send_control(steering, velocity)

            # visualize data
            if self.use_matplotlib or self.use_rviz:
                self.svea.visualize_data()
            else:
                self.svea.visualize_data()
                # rospy.loginfo_throttle(1, state)

        if not rospy.is_shutdown():
            rospy.loginfo("Trajectory finished!")

        rospy.spin()

if __name__ == '__main__':
	try:
		node = Node()
		node.run()
	except rospy.ROSInterruptException:
		pass
