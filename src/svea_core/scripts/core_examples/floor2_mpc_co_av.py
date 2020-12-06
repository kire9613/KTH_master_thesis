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

# xs = [14.47, 37.06]
# ys = [1.60, 1.29]
# traj_x_init = np.linspace(xs[0], xs[1]).tolist()
# traj_y_init = np.linspace(ys[0], ys[1]).tolist()
# xs = [37.06,37.05]
# ys = [1.29,6.96]
# traj_x_init = np.append(traj_x_init,np.linspace(xs[0], xs[1]).tolist()[1:])
# traj_y_init = np.append(traj_y_init,np.linspace(ys[0], ys[1]).tolist()[1:])
# xs = [37.15,14.68]
# ys = [6.96,7.31]
# traj_x_init = np.append(traj_x_init,np.linspace(xs[0], xs[1]).tolist()[1:])
# traj_y_init = np.append(traj_y_init,np.linspace(ys[0], ys[1]).tolist()[1:])
# xs = [14.69,14.47]
# ys = [7.31,1.60]
# traj_x_init = np.append(traj_x_init,np.linspace(xs[0], xs[1]).tolist()[1:-1])
# traj_y_init = np.append(traj_y_init,np.linspace(ys[0], ys[1]).tolist()[1:-1])

traj_x_init = [15.1, 15.399999999999993, 15.699999999999987, 15.99999999999998, 16.300000000000026, 16.600000000000072, 16.90000000000012, 17.200000000000166, 17.500000000000213, 17.80000000000026, 18.100000000000307, 18.400000000000354, 18.7000000000004, 19.000000000000448, 19.300000000000495, 19.60000000000054, 19.90000000000059, 20.200000000000635, 20.500000000000682, 20.80000000000073, 21.100000000000776, 21.400000000000823, 21.70000000000087, 22.000000000000917, 22.300000000000963, 22.60000000000101, 22.900000000001057, 23.200000000001104, 23.50000000000115, 23.800000000001198, 24.100000000001245, 24.40000000000129, 24.70000000000134, 25.000000000001386, 25.300000000001432, 25.60000000000148, 25.900000000001526, 26.200000000001573, 26.50000000000162, 26.800000000001667, 27.100000000001714, 27.40000000000176, 27.700000000001808, 28.000000000001855, 28.3000000000019, 28.60000000000195, 28.900000000001995, 29.200000000002042, 29.50000000000209, 29.800000000002136, 30.100000000002183, 30.40000000000223, 30.700000000002277, 31.000000000002323, 31.30000000000237, 31.600000000002417, 31.900000000002464, 32.20000000000244, 32.50000000000238, 32.80000000000232, 33.10000000000226, 33.4000000000022, 33.70000000000214, 33.99943366737961, 34.2988077805138, 34.59880778051374, 34.89824144789121, 35.197615561025394, 35.497615561025334, 35.89704922840281, 36.19623533953543, 36.48506204754969, 36.74532344436383, 36.96061420825326, 37.1173636972051, 37.20569136101907, 37.22002955209248, 37.21123133977523, 37.21935112063529, 37.227470901495344, 37.2355906823554, 37.24371046321546, 37.251830244075514, 37.25995002493557, 37.268069805795626, 37.27618958665568, 37.28430936751574, 37.292429148375795, 37.27228585758349, 37.17071979433642, 37.002252856613275, 36.77750419249801, 36.51064061346214, 36.21848360330716, 35.91881306473572, 35.618883726958735, 35.31895438918175, 35.01902505140476, 34.71909571362777, 34.419166375850786, 34.1192370380738, 33.81930770029681, 33.519378362519824, 33.219449024742836, 32.91951968696585, 32.61959034918886, 32.319661011411874, 32.01973167363489, 31.719802335857796, 31.419872998080702, 31.11994366030361, 30.820014322526514, 30.52008498474942, 30.220155646972326, 29.920226309195233, 29.62029697141814, 29.320367633641045, 29.02043829586395, 28.720508958086857, 28.420579620309763, 28.12065028253267, 27.820720944755575, 27.52079160697848, 27.220862269201387, 26.920932931424293, 26.6210035936472, 26.321074255870105, 26.02114491809301, 25.721215580315917, 25.4215091523501, 25.12183861377865, 24.821909276001556, 24.52220284803574, 24.222532309464288, 23.922602971687194, 23.622896543721378, 23.323226005149927, 23.023296667372833, 22.723590239407017, 22.423919700835565, 22.12399036305847, 21.824061025281377, 21.52435459731556, 21.22468405874411, 20.924754720967016, 20.6250482930012, 20.32537775442975, 20.025448416652655, 19.72574198868684, 19.426071450115387, 19.126142112338293, 18.826435684372477, 18.526765145801026, 18.22683580802393, 17.927129380058116, 17.627458841486664, 17.32752950370957, 17.027823075743754, 16.728152537172303, 16.42822319939521, 16.128516771429393, 15.82884623285794, 15.528916895080899, 14.930175856249761, 14.643112997893926, 14.386116617597159, 14.1753862299722, 14.024205018189717, 13.942102541563385, 13.934254049131011, 14.001154262910482, 14.07049919607521, 14.13739940985468, 14.206744343019407, 14.273644556798878, 14.342989489963605, 14.409889703743076, 14.479234636907803, 14.546134850687274, 14.615479783852, 14.682379997631472, 14.751724930796199, 14.798490628573454, 14.861913744272584, 14.925336859971715]
traj_y_init = [1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.54, 1.524182495318074, 1.5072752483923575, 1.5072752483923575, 1.4914577437104315, 1.474550496784715, 1.474550496784715, 1.458732992102789, 1.4624665427491146, 1.540603609706744, 1.6882188996446383, 1.8960076276329458, 2.1508720351292245, 2.436746994333875, 2.7356126577544804, 3.0353347057814974, 3.3352248009143373, 3.635114896047177, 3.935004991180017, 4.234895086312857, 4.534785181445697, 4.834675276578537, 5.134565371711377, 5.434455466844216, 5.734345561977056, 6.034235657109896, 6.631997313953263, 6.9134410974902, 7.160716570170533, 7.358236965252915, 7.493551778482166, 7.55813157340593, 7.568537481621252, 7.562026541550107, 7.555515601478962, 7.5490046614078175, 7.5424937213366725, 7.535982781265528, 7.529471841194383, 7.522960901123238, 7.516449961052093, 7.509939020980948, 7.503428080909803, 7.496917140838658, 7.490406200767513, 7.483895260696368, 7.477384320625223, 7.470873380554078, 7.464362440482933, 7.457851500411788, 7.451340560340643, 7.444829620269498, 7.438318680198353, 7.4318077401272085, 7.4252968000560635, 7.418785859984919, 7.412274919913774, 7.405763979842629, 7.399253039771484, 7.392742099700339, 7.386231159629194, 7.379720219558049, 7.373209279486904, 7.366698339415759, 7.360187399344614, 7.353676459273469, 7.347165519202324, 7.356480649338857, 7.366886557554179, 7.360375617483034, 7.3696907476195666, 7.380096655834889, 7.373585715763744, 7.382900845900276, 7.393306754115598, 7.386795814044453, 7.396110944180986, 7.406516852396308, 7.400005912325163, 7.393494972254018, 7.402810102390551, 7.413216010605873, 7.406705070534728, 7.4160202006712606, 7.426426108886583, 7.419915168815438, 7.42923029895197, 7.439636207167292, 7.433125267096147, 7.44244039723268, 7.452846305448002, 7.446335365376857, 7.45565049551339, 7.466056403728712, 7.459545463657567, 7.4688605937940995, 7.479266502009422, 7.472755561938277, 7.482070692074809, 7.492476600290131, 7.485965660218986, 7.485054842980147, 7.400667735862582, 7.247438727481479, 7.03502645799939, 6.77682012635922, 6.489095515269175, 6.189989063212997, 5.898354651849457, 5.607291860102596, 5.315657448739056, 5.024594656992195, 4.7329602456286555, 4.441897453881794, 4.1502630425182545, 3.8592002507713956, 3.5675658394078584, 3.276503047661, 2.984868636297463, 2.6938058445506043, 2.397624001547195, 2.1044047859507184, 1.8111855703542377]

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

        self.svea.controller.build_solver(
            dt,
            Q=params.Q,
            R=params.R,
            P=params.P,
            ulb=ulb,
            uub=uub,
            xlb=xlb,
            xub=xub,
            max_cpu_time=0.8*dt,
            horizon=params.horizon,
            model_type=params.model_type,
            solver_=params.solver_,
            TAU = params.TAU,
            N_IND_SEARCH = params.N_IND_SEARCH,
        )
        self.svea.start(wait=True)

        if self.is_sim:
            # start simulation
            self.simulator.toggle_pause_simulation()

        # simualtion loop

        self.svea.controller.target_velocity = target_velocity
        self.svea.pid.target_velocity = target_velocity/2
        self.svea.pid.k = 0.6  # look forward gain
        self.svea.pid.Lfc = 0.4  # look-ahead distance
        self.svea.pid.K_p = 1.0  # speed control propotional gain
        self.svea.pid.K_i = 0.2  # speed control integral gain
        self.svea.pid.K_d = 0.0  # speed control derivitive gain
        self.svea.pid.L = 0.324  # [m] wheel base of vehicle
        while not self.svea.is_finished and not rospy.is_shutdown():
            state = self.svea.wait_for_state()

            #This step updates the global path
            # svea.update_traj(self.traj_x, self.traj_y)

            # compute control input
            if self.collision:
                steering, velocity = self.svea.compute_pid_control()
                self.svea.send_control(steering, velocity)
            else:
                steering, velocity = self.svea.compute_control()
                self.svea.send_control(steering, velocity)

            # visualize data
            if self.use_matplotlib or self.use_rviz:
                self.svea.visualize_data()
            else:
                rospy.loginfo_throttle(1, state)

        if not rospy.is_shutdown():
            rospy.loginfo("Trajectory finished!")

        rospy.spin()

if __name__ == '__main__':
	try:
		node = Node()
		node.run()
	except rospy.ROSInterruptException:
		pass
