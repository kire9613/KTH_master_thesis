#! /usr/bin/env python

#Actuation_node

import rospy
import os
from av09_msgs.msg import *
from std_msgs.msg import *
import numpy as np
import math

from svea.svea_managers.path_following_sveas import SVEAPurePursuit
from svea.states import VehicleState
from svea.localizers import LocalizationInterface
from svea.controllers.pure_pursuit import PurePursuitController
from svea.data import BasicDataHandler, TrajDataHandler, RVIZPathHandler
from svea.models.bicycle import SimpleBicycleModel
from svea.simulators.sim_SVEA import SimSVEA
from svea.track import Track


## SIMULATION PARAMS ##########################################################
vehicle_name = ""
target_velocity = 1.0 # [m/s]
dt = 0.01 # frequency of the model updates

#TODO: create a trajectory that goes around the track
xs = [-2.33, 10.48]
ys = [-7.09, 11.71]
traj_x = np.linspace(xs[0], xs[1]).tolist()
traj_y = np.linspace(ys[0], ys[1]).tolist()
###############################################################################

## INIT #######################################################################
default_init_pt = [0.0, 0.0, 0.0, 0.0] # [x, y, yaw, v], units: [m, m, rad, m/s]
###############################################################################


def dist_calc(svea,goal_p):
    dist=math.sqrt((svea.state.x-goal_p[0])*(svea.state.x-goal_p[0])+(svea.state.y-goal_p[1])*(svea.state.y-goal_p[1]))
    if dist <=1:
        return 1
    return 0



class CTcorr:
    def __init__(self):
        self.ver=0
        self.Fcorr=0
        self.Arecc='NO ACTION'
        self.Request=0
        self.release_sd=2

    def callback(self, msg):
        self.ver=msg.ctver
        self.Fcorr=msg.fault
        self.Arecc=msg.recact
        self.Request=msg.ctreq
        self.release_sd=msg.release




class PLAN:
    def __init__(self,start_pt,vr):
        self.action='NO ACTION'
        self.RC=1
        self.goal=start_pt
        self.maingoal=self.goal
        self.workshop_pt=[-2.88,-3.2]
        self.vr=vr


class StateMachine:
    def __init__(self,state,vr):


        self.xs1 = [-2.33, 10.48]
        self.ys1 = [-7.09, 11.71]
        self.xs2= [10.48, 5.96]
        self.ys2= [11.71, 15.2]
        self.xs3= [5.96,-7.2]
        self.ys3= [15.2,-3.87]
        self.xs4= [-7.2, -2.33]
        self.ys4= [-3.87,-7.09]
        self.traj_xs=[]
        self.traj_ys=[]
        self.traj_xs.append(np.linspace(self.xs1[0], self.xs1[1]).tolist())
        self.traj_ys.append(np.linspace(self.ys1[0], self.ys1[1]).tolist())
        self.traj_xs.append(np.linspace(self.xs2[0], self.xs2[1]).tolist())  
        self.traj_ys.append(np.linspace(self.ys2[0], self.ys2[1]).tolist())
        self.traj_xs.append(np.linspace(self.xs3[0], self.xs3[1]).tolist())
        self.traj_ys.append(np.linspace(self.ys3[0], self.ys3[1]).tolist())
        self.traj_xs.append(np.linspace(self.xs4[0], self.xs4[1]).tolist())
        self.traj_ys.append(np.linspace(self.ys4[0], self.ys4[1]).tolist())

        
        start_pt=[self.xs1[1],self.ys1[1]]
        self.state=state
        self.plan=PLAN(start_pt,vr)
        self.CTcorr=CTcorr()
        self.start=0
        self.traj_num=0
        self.route='MAIN ROUTE'
        self.actuation_status='DRIVING MAIN ROUTE'
        self.operational_state='NORMAL OPERATIONS'

    def callback_start(self,msg):
        self.start=msg.data
        

    def callback(self, msg):
        if self.state==1 or self.state==3:
            self.plan.action=msg.action
            self.plan.RC=msg.rc

    def update_actuation(self,svea):
        if self.plan.action=='NO ACTION':
            self.plan.goal=self.plan.maingoal
            self.plan.vr=1
            self.route='MAIN ROUTE'
        elif self.plan.action=='EMERGENCY BREAK,CONTACT TOW':
            self.plan.vr=0
            self.actuation_status='STANDING DOWN, WAITING FOR CT'
        elif self.plan.action=='SLOW DOWN,DRIVE TO WORKSHOP':
            self.route='WORKSHOP'
            self.plan.vr=0.1
            self.plan.goal=self.plan.workshop_pt


        self.is_finished=dist_calc(svea,self.plan.goal)

        if self.route=='MAIN ROUTE':
            self.actuation_status='DRIVING MAIN ROUTE'
            if self.is_finished:
                if self.traj_num==3:
                    self.traj_num=0
                else:
                    self.traj_num=self.traj_num+1
            traj_x=self.traj_xs[self.traj_num]
            traj_y=self.traj_ys[self.traj_num]
            self.plan.maingoal=[traj_x[-1],traj_y[-1]]
            
        elif self.route=='WORKSHOP':
            traj_x= np.linspace(svea.state.x, self.plan.goal[0]).tolist()
            traj_y= np.linspace(svea.state.y, self.plan.goal[1]).tolist()
            if self.is_finished:
                self.plan.vr=0
                self.actuation_status='ARRIVED AT WORKSHOP'
            else:
                self.actuation_status='DECREASED SPEED, DRIVING TO WORKSHOP'
        return svea,traj_x,traj_y



    def Stateswitch(self):
        if self.state==1:
            return self.state_1()
        elif self.state==2:
            return self.state_2()
        elif self.state==3:
            return self.state_3()

    def state_1(self): #normal operations
        self.operational_state='NORMAL OPERATIONS'
        self.state=self.plan.RC
        

    def state_2(self): #CT release condition
        self.operational_state='CT RELEASE CONDITION'
        
        if self.CTcorr.release_sd==1:
            self.state=1
        
    def state_3(self): # AUTO
        self.operational_state='AUTO RELEASE CONDITION'
        self.state=self.plan.RC
        


def param_init():
    """Initialization handles use with just python or in a launch file
    """
    # grab parameters from launch-file
    start_pt_param = rospy.search_param('start_pt')
    is_sim_param = rospy.search_param('is_sim')
    use_rviz_param = rospy.search_param('use_rviz')
    use_matplotlib_param = rospy.search_param('use_matplotlib')

    start_pt = rospy.get_param(start_pt_param, default_init_pt)
    if isinstance(start_pt, str):
        start_pt = start_pt.split(',')
        start_pt = [float(curr) for curr in start_pt]
        start_pt = VehicleState(*start_pt)

    is_sim = rospy.get_param(is_sim_param, True)
    use_rviz = rospy.get_param(use_rviz_param, False)
    use_matplotlib = rospy.get_param(use_matplotlib_param, False)

    return start_pt, is_sim, use_rviz, use_matplotlib

def main():


    ## SIMULATION PARAMS ##########################################################
    vehicle_name = ""
    target_velocity = 1.0 # [m/s]
    dt = 0.01 # frequency of the model updates

    #TODO: create a trajectory that goes around the track


    

    ###############################################################################

    ## INIT #######################################################################
    default_init_pt = [0.0, 0.0, 0.0, 0.0] # [x, y, yaw, v], units: [m, m, rad, m/s]
    ###############################################################################





    rospy.init_node('actuation_node')
    start_pt, is_sim, use_rviz, use_matplotlib = param_init()
    SM=StateMachine(1,target_velocity)
    rospy.Subscriber('action', action, SM.callback )
    rospy.Subscriber('CTcorr', ctcorr, SM.CTcorr.callback )
    rospy.Subscriber('start_system',Int8,SM.callback_start)
    pub_actuation_status=rospy.Publisher('actuation_status',node_status,queue_size=10)

    traj_num=0
    goal_p=[SM.xs1[1],SM.ys1[1]]

    # select data handler based on the ros params
    if use_rviz:
        DataHandler = RVIZPathHandler
    else:
        DataHandler = TrajDataHandler

    if is_sim:
        # start the simulation
        model_for_sim = SimpleBicycleModel(start_pt)
        simulator = SimSVEA(vehicle_name, model_for_sim,
                            dt=dt, run_lidar=True, start_paused=True).start()

    # start pure pursuit SVEA manager
    svea = SVEAPurePursuit(vehicle_name,
                           LocalizationInterface,
                           PurePursuitController,
                           SM.traj_xs[0], SM.traj_ys[0],
                           data_handler = DataHandler)
    
    while not SM.start:
        continue
    
    
    svea.start(wait=True)
    
    if is_sim:
        # start simulation
        simulator.toggle_pause_simulation()

    # simualtion loop
    svea.controller.target_velocity = target_velocity
    while not svea.is_finished and not rospy.is_shutdown():
        SM.Stateswitch()
        
        target_velocity=SM.plan.vr
        svea.controller.target_velocity = target_velocity
        state = svea.wait_for_state()
        # compute control input via pure pursuit
        steering, velocity = svea.compute_control()
        svea.send_control(steering, velocity)

        # visualize data
        if use_matplotlib or use_rviz:
            svea.visualize_data()
        else:
            rospy.loginfo_throttle(1, state)
        svea,traj_x,traj_y=SM.update_actuation(svea)
        svea.update_traj(traj_x,traj_y)
        status_msg=node_status()
        status_msg.status1=SM.operational_state
        status_msg.status2=SM.actuation_status
        status_msg.status3=SM.is_finished
        status_msg.status4=SM.plan.vr
        status_msg.status5=velocity
        status_msg.status6=SM.route
        pub_actuation_status.publish(status_msg)
        #print(SM.CTcorr.rc)

    if not rospy.is_shutdown():
        rospy.loginfo("Trajectory finished!")


    rospy.spin()



if __name__ == '__main__':
	main()