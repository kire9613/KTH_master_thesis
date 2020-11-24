#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np

from svea.states import VehicleState
from svea.localizers import LocalizationInterface
from svea.data import BasicDataHandler, TrajDataHandler, RVIZPathHandler
from svea.models.bicycle import SimpleBicycleModel
from svea.simulators.sim_SVEA import SimSVEA
from sensor_msgs.msg import LaserScan

from svea.svea_managers.mpc_path_following_sveas import SVEAMPC
from svea.controllers.mpc.mpc import MPC

"""
__team__ = "Team 1"
__maintainers__ = "Roberto Castro Sundin, Astrid Lindstedt, Johan Hedin, Aravind Sadashiv, Sarthak Manocha”
__status__ = "Development"
"""

## SIMULATION PARAMS ##########################################################
vehicle_name = ""
target_velocity = 1.5 # [m/s]
dt = 0.05 # frequency of the model updates

xs = [-2.33, 10.48]
ys = [-7.09, 11.71]
traj_x = np.linspace(xs[0], xs[1]).tolist()
traj_y = np.linspace(ys[0], ys[1]).tolist()
xs = [10.48,6.03]
ys = [11.71,14.8]
traj_x = np.append(traj_x,np.linspace(xs[0], xs[1]).tolist()[1:])
traj_y = np.append(traj_y,np.linspace(ys[0], ys[1]).tolist()[1:])
xs = [6.03,-6.78]
ys = [14.8,-4]
traj_x = np.append(traj_x,np.linspace(xs[0], xs[1]).tolist()[1:])
traj_y = np.append(traj_y,np.linspace(ys[0], ys[1]).tolist()[1:])
xs = [-6.78,-2.33]
ys = [-4.00,-7.09]
# traj_x = np.append(traj_x,np.linspace(xs[0], xs[1]).tolist()[1:-1])
# traj_y = np.append(traj_y,np.linspace(ys[0], ys[1]).tolist()[1:-1])

traj_x =  [0.53502483104700005, 0.75880470850270842, 0.97002287981715085, 1.1827769825984542, 1.3873426959305166, 1.6050772109688944, 1.8277535956462396, 2.0453636707167178, 2.2906895507981102, 2.5267343443202259, 2.7591674842625222, 2.9840820032121989, 3.1980016768456547, 3.4051196721759047, 3.6111423071368787, 3.8284260393523049, 4.051433846863687, 4.2908689735371723, 4.517754642744916, 4.7251538996644751, 4.9303804986079074, 5.1349925953115969, 5.343154241175319, 5.5739314485861904, 5.820995669089827, 6.0693229130231439, 6.3152405588258249, 6.5497272260477999, 6.7890446546182632, 7.0252472402907706, 7.2593409163450735, 7.4947941821366433, 7.7241778909644188, 7.9569410959073918, 8.1983955011000802, 8.4286264274833567, 8.6658386366725129, 8.8940739306884762, 9.1214671910622833, 9.3380304210278027, 9.5450458069467832, 9.7520409220153006, 9.9593172799343126, 10.100472263783605, 10.061310570316097, 9.8068153404225455, 9.4931769478416186, 9.1531472400072147, 8.8231039678230925, 8.4944093497119937, 8.1707524854278706, 7.8456549792563717, 7.5079428516283251, 7.169088944995984, 6.8292642550933156, 6.4833707296764338, 6.1056603511855272, 5.7097472179550612, 5.3743472375480126, 5.100721940172332, 4.8582529048621961, 4.6456738278272294, 4.4203025662515438, 4.1945613990024615, 3.9737382520267253, 3.7572834798431578, 3.5499376385807988, 3.3378768991181889, 3.11604696316618, 2.8780259321554498, 2.636473628285275, 2.3999346592863038, 2.1647648645735598, 1.9389352717067445, 1.713227749979646, 1.490929732113726, 1.2654650087096682, 1.0273339234975689, 0.78183895969569495, 0.53531085860905603, 0.31712382400008005, 0.095942397995614007, -0.14079177676856267, -0.38657378546973209, -0.63585866261010182, -0.87222742054208779, -1.1077114500526779, -1.33958174561376, -1.5727083587941053, -1.812532456726329, -2.0484495053317118, -2.2872451882803091, -2.5259539151345018, -2.7607643705361173, -3.0035352381653011, -3.2385014667457925, -3.4734017625789781, -3.7154619273617921, -3.9586753227189502, -4.2000003935198267, -4.43541700152484, -4.6630000659200705, -4.889956966605058, -5.1108828820222483, -5.3350483097428425, -5.5474110748373793, -5.7499625021779401, -5.9490975244395319, -6.1448882738627937, -6.3570107122543513, -6.5714694116019583, -6.7538349170229193, -6.8280466752527369, -6.8178607746263244, -6.6818733953671519, -6.4739462467128162, -6.2023183889180507, -5.8949408293483589, -5.5769693316725251, -5.2637251352628418, -4.9407592437433268, -4.6162229345651555, -4.2873471834389054, -3.9542932140258165, -3.6081914690783026, -3.2237649764044907, -2.827483832174372, -2.4624316198180192, -2.162198868635627, -1.8873724956341977, -1.6144416087095832, -1.3552424399908669, -1.1123141580444924, -0.86790742858136505, -0.6384418825060505, -0.4032885221541378, -0.17010355813884101, 0.057826253166703846, 0.27535901383809602]
traj_y =  [-2.9267062403800002, -2.5951577866933073, -2.2555137887595209, -1.9167923443005552, -1.573065306770876, -1.2375483880247167, -0.90526161507239278, -0.56991837377249566, -0.25375190014768134, 0.069129414999055602, 0.39468542478746937, 0.72544756659928167, 1.0634419380889937, 1.4056342037599125, 1.7485335269822935, 2.084302355025891, 2.4164525567438662, 2.7368129739652307, 3.0662016676266832, 3.408232495177689, 3.7515720911471306, 4.0952821149968957, 4.4368344147538084, 4.7635225202918248, 5.0781189971052285, 5.391711843962641, 5.7071083003319441, 6.031215131225526, 6.3517281206901979, 6.6745148631823543, 6.9988549314423461, 7.3222223295205824, 7.6499089312615789, 7.975207376013107, 8.2941305245715533, 8.6212147769265695, 8.9432785794551215, 9.2717730672557881, 9.6008762904009366, 9.9370927690451865, 10.279366029475124, 10.621585408929942, 10.96369806700195, 11.336987086516137, 11.729851114582898, 12.037434716113962, 12.284623281834802, 12.495391819854914, 12.721368323773021, 12.949317040473112, 13.184341893742017, 13.417320426302373, 13.631717397441809, 13.84417298054953, 14.055021356024316, 14.25578692911076, 14.386443743014716, 14.372122815928119, 14.161035485040866, 13.868755851205322, 13.551409198793747, 13.212463746041843, 12.881981144518733, 12.551842787462739, 12.218274552328818, 11.881965127947698, 11.539861269780451, 11.200740082027091, 10.867807732034889, 10.546429250457928, 10.227596968894302, 9.9050271588477674, 9.5814416439445047, 9.2513126085345423, 8.9210801211356952, 8.5885459928904933, 8.2583118967982774, 7.9367881223404364, 7.6209975759603994, 7.3062870370859976, 6.9708047879204669, 6.6375709042982187, 6.3151362726107241, 5.9995564283338432, 5.68668876169599, 5.3640702530838613, 5.0406977416315728, 4.7147818698355461, 4.3897915712157864, 4.0696359924075898, 3.746612875921401, 3.425710929387678, 3.1047592324096533, 2.7809515741484172, 2.4630430293943273, 2.139314397022742, 1.8155578978459934, 1.4971120933227171, 1.1795459346569357, 0.86054483472265209, 0.53715069714125407, 0.20821357977259397, -0.12118510079530535, -0.45460847605563193, -0.78595736627436064, -1.1247618008236664, -1.4697765908667226, -1.8166353291615334, -2.1654833705195937, -2.504617024377469, -2.842073717015003, -3.1977967158624083, -3.5915864208756663, -3.9892804453979487, -4.3671625924409643, -4.7077234360058933, -5.0019971420852904, -5.2576667823711833, -5.50035177214263, -5.7490913388986762, -5.9851087376965317, -6.2189619427410099, -6.4465528950401918, -6.6681090879680545, -6.8685530484685557, -6.9739007047398527, -6.925562550158026, -6.7623666270415805, -6.4984973445403611, -6.2080463958222305, -5.9157464216980298, -5.6109371630737677, -5.2932613856500499, -4.9766482281797755, -4.6489759720222237, -4.3254125003092048, -4.0003697047894189, -3.6717492024444853, -3.3360290634512522]
# traj_x = traj_x[::-1]
# traj_y = traj_y[::-1]
###############################################################################

## INIT #######################################################################
default_init_pt = [0.0, 0.0, 0.0, 0.0] # [x, y, yaw, v], units: [m, m, rad, m/s]
###############################################################################


def param_init():
    """Initialization handles use with just python or in a launch file
    """
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

    is_sim = rospy.get_param(is_sim_param, True)
    use_rviz = rospy.get_param(use_rviz_param, False)
    use_matplotlib = rospy.get_param(use_matplotlib_param, False)
    run_lidar = rospy.get_param(run_lidar_param, True)

    return start_pt, is_sim, use_rviz, use_matplotlib, run_lidar

def callback_scan(scan):
    """ Callback for lidarscan """
    ranges = scan.ranges
    min_dist = np.nanmin(ranges) # TODO: Make available as self.min_dist etc.?

def main():
    rospy.init_node('floor2_team1')
    start_pt, is_sim, use_rviz, use_matplotlib, _run_lidar = param_init()

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
                            dt=dt, start_paused=True, run_lidar=_run_lidar).start()

    lidar_sub = rospy.Subscriber("/scan", LaserScan, callback_scan)

    # start pure pursuit SVEA manager
    mpc = MPC
    mpc.target_velocity = target_velocity
    mpc.dl = dt*target_velocity
    svea = SVEAMPC(vehicle_name,
                   LocalizationInterface,
                   mpc,
                   traj_x, traj_y,
                   data_handler = DataHandler)
    Q = np.diag([
        100, # x
        100, # y
        0.01, # ψ
        100, # v
    ])
    R = np.diag([
        1, # a
       0.01, # δ
    ])
    # P_LQR = np.matrix(scipy.linalg.solve_discrete_are(A, B, Q, R))
    P_LQR = np.diag([
            1200,
            1200,
            1,
            100
    ])

    ulb = [-1e2,-np.deg2rad(40)]
    uub = [ 1e2, np.deg2rad(40)]
    xlb = [-np.inf]*3+[-1]
    #xlb = [-np.inf,-np.inf, -np.deg2rad(40), 3.6]
    xub = [ np.inf]*3+[3.6]
    #xub = [np.inf,np.inf, np.deg2rad(40), -3.6]
    svea.controller.build_solver(dt,
                                 Q=Q,
                                 R=R,
                                 P=P_LQR,
                                 ulb=ulb,
                                 uub=uub,
                                 xlb=xlb,
                                 xub=xub,
                                 max_cpu_time=0.8*dt,
                                 horizon=7,
                                 model_type="linear",
                                 solver_="osqp",
                                 )
    svea.start(wait=True)

    if is_sim:
        # start simulation
        simulator.toggle_pause_simulation()

    # simulation loop
    svea.controller.target_velocity = target_velocity
    while not svea.is_finished and not rospy.is_shutdown():
        state = svea.wait_for_state()

        # compute control input via pure pursuit
        steering, velocity = svea.compute_control()
        #print("velocity = ", velocity, "steering = ", steering)
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
