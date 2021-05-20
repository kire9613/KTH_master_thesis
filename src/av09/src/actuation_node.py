#! /usr/bin/env python

#Erik Branzen
#Code based on floor2_example.py

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


xs = [-2.33, 10.48]
ys = [-7.09, 11.71]
traj_x = np.linspace(xs[0], xs[1]).tolist()
traj_y = np.linspace(ys[0], ys[1]).tolist()
###############################################################################

trajs_dict={'W1':'start:N1:W1:N1:N2:FIN','W2':'start:N1:N2:W2:N2:FIN','FIN':'start:N1:N2:FIN'}
trajs_adv_dict={'start:N1:W1':'start:N1:W1:N1:N2:FIN','start:N1:W2':'start:N1:N2:W2:N2:FIN','start:N1:FIN':'start:N1:N2:FIN','N1:W1:W1':'N1:W1:N1:N2:FIN','W1:N1:W1':'N1:W1:N1:N2:FIN','N1:N2:W1':'N2:N1:W1:N1:N2:FIN',
'N2:N1:W1':'N2:N1:W1:N1:N2:FIN','N2:W2:W1':'W2:N2:N1:W1:N1:N2:FIN','N2:FIN:W1':'FIN:N2:N1:W1:N1:N2:FIN','start:N1:W2':'start:N1:N2:W2:N2:FIN','N1:W1:W2':'W1:N1:N2:W2:N2:FIN','W1:N1:W2':'W1:N1:N2:W2:N2:FIN',
'N1:N2:W2':'N1:N2:W2:N2:FIN','N2:W2:W2':'N2:W2:N2:FIN','N2:FIN:W2':'FIN:N2:W2:N2:FIN','N1:W1:FIN':'W1:N1:N2:FIN',
'N1:N2:FIN':'N1:N2:FIN','N2:W2:FIN':'W2:N2:FIN','W2:N2:FIN':'W2:N2:FIN','N2:FIN:FIN':'N2:FIN','N1:W1:FIN':'W1:N1:N2:FIN','W1:N1:FIN':'W1:N1:N2:FIN','W2:N2:W1':'W2:N2:N1:W1:N1:N2:FIN','W2:N2:W2':'N2:W2:N2:FIN'}
node_dict_floor2={'start':[-7.43,-15.3],'N1':[-2.74,-7.27],'W1':[-5.167,-1.67],'N2':[9.96,10.55],'W2':[15.37,18.51],'FIN':[5.95,14.93]}
node_dict_itrl={'start':[-7.4,-1.9],'N1':[-7.12,6.13],'W1':[-8.2,8.17],'N2':[0.16,6.0],'W2':[1.65,6.3],'FIN':[0.78,11.8]}
points_dict={'floor2':node_dict_floor2,'itrl':node_dict_itrl}
#map_name_param=rospy.search_param('map_name')
#map_name=rospy.get_param(map_name_param,False)
#print(map_name)
#node_dict=points_dict[map_name]

## INIT #######################################################################
default_init_pt = [0.0, 0.0, 0.0, 0.0] # [x, y, yaw, v], units: [m, m, rad, m/s]
###############################################################################

#calculate distance to local goal. Returns True if closer than 0.4 meter.
def dist_calc(svea,goal_p):
    dist=math.sqrt((svea.state.x-goal_p[0])*(svea.state.x-goal_p[0])+(svea.state.y-goal_p[1])*(svea.state.y-goal_p[1]))
    if dist <=0.4:
        return 1
    return 0

#Calculates distance to goal given current location in node network. Iterates through the node network and calculates and adds distance between nodes to the total
def dist_to_goal(svea,curr_path,goal):
    sumdist=math.sqrt((svea.state.x-node_dict[curr_path[1]][0])**2 + (svea.state.y-node_dict[curr_path[1]][1])**2)
    node_list=trajs_adv_dict[':'.join([curr_path[0],curr_path[1],goal])].split(':')
    i=0
    started=False

    while i<=len(node_list):
        

        if node_list[i]==goal:
            return sumdist
            
        if started:
            #adds distance between nodes in the current iteration to the total distance.
            sumdist=sumdist+math.sqrt((node_dict[node_list[i]][0]-node_dict[node_list[i+1]][0])**2 + (node_dict[node_list[i]][1]-node_dict[node_list[i+1]][1])**2)
        if node_list[i]==curr_path[0] and node_list[i+1]==curr_path[1]:
            started=True
        i=i+1





#Class for representing the control tower control variables
class CTcorr:
    def __init__(self):
        self.ver=0
        self.Fcorr=0
        self.Arecc='NO ACTION'
        self.Request=0
        self.release_sd=2

    #set the control tower variables from ROS message
    def callback(self, msg):
        self.ver=msg.ctver
        self.Fcorr=msg.fault
        self.Arecc=msg.recact
        self.Request=msg.ctreq
        self.release_sd=msg.release



#Class for representing the plan.
class PLAN:
    def __init__(self,vr,vl,mgoal,wgoal,RC):
        
        self.vl=vl
        self.vr=vr
        self.mgoal=mgoal
        self.wgoal=wgoal
        self.RC=RC

#Main state machine of the actuation of the vehicle. The states correspond to operational conditions.
class StateMachine:
    def __init__(self,state,vr):


        #Setup trajectories between all nodes in the node network
        next_traj_dict_x={}
        next_traj_dict_y={}


        next_traj_dict_x['start:N1']=np.linspace(node_dict['start'][0],node_dict['N1'][0])
        next_traj_dict_y['start:N1']=np.linspace(node_dict['start'][1],node_dict['N1'][1])

        next_traj_dict_x['N1:W1']=np.linspace(node_dict['N1'][0],node_dict['W1'][0])
        next_traj_dict_y['N1:W1']=np.linspace(node_dict['N1'][1],node_dict['W1'][1])

        next_traj_dict_x['W1:N1']=np.linspace(node_dict['W1'][0],node_dict['N1'][0])
        next_traj_dict_y['W1:N1']=np.linspace(node_dict['W1'][1],node_dict['N1'][1])

        next_traj_dict_x['N1:N2']=np.linspace(node_dict['N1'][0],node_dict['N2'][0])
        next_traj_dict_y['N1:N2']=np.linspace(node_dict['N1'][1],node_dict['N2'][1])

        next_traj_dict_x['N2:W2']=np.linspace(node_dict['N2'][0],node_dict['W2'][0])
        next_traj_dict_y['N2:W2']=np.linspace(node_dict['N2'][1],node_dict['W2'][1])

        next_traj_dict_x['W2:N2']=np.linspace(node_dict['W2'][0],node_dict['N2'][0])
        next_traj_dict_y['W2:N2']=np.linspace(node_dict['W2'][1],node_dict['N2'][1])

        next_traj_dict_x['N2:FIN']=np.linspace(node_dict['N2'][0],node_dict['FIN'][0])
        next_traj_dict_y['N2:FIN']=np.linspace(node_dict['N2'][1],node_dict['FIN'][1])

        self.next_traj_dict_x=next_traj_dict_x
        self.next_traj_dict_y=next_traj_dict_y

        





        #initialization of variables and creating of Plan, CTcorr objects.
        start_pt=[node_dict['start'][0], node_dict['start'][1]]
        self.state=state
        self.plan=PLAN(vr,1,'FIN',None, 1)
        self.CTcorr=CTcorr()
        self.start=0
        self.route='MAIN ROUTE'
        self.actuation_status='DRIVING MAIN ROUTE'
        self.operational_state='NORMAL OPERATIONS'
        self.currentpath=['start','N1']
        self.wait=0
        self.waiting=False
        self.waited=False

    #set start variable from ROS message. variable determines when to start simulation
    def callback_start(self,msg):
        self.start=msg.data
        
    #Set plan variables from ROS message
    def callback_plan(self, msg):
        if self.state==1 or self.state==3:
            self.plan.vl=msg.vl
            self.plan.RC=msg.rc
            self.plan.wgoal=msg.wgoal
            self.plan.mgoal=msg.mgoal

    #Updating of trajectory and speed given current position and end goal as well workshop visit
    def update_actuation(self,svea):
        
        self.is_finished=dist_calc(svea,node_dict[self.currentpath[1]])


        #new traj update based on last and next node

        traj_x= self.next_traj_dict_x[':'.join(self.currentpath)]
        traj_y= self.next_traj_dict_y[':'.join(self.currentpath)]
        
        

        try:
            global_route=trajs_dict[self.plan.wgoal].split(':')
        except:
            global_route=trajs_dict[self.plan.mgoal].split(':')

        #When at the workshop, we wait for little while before driving again
        if self.waiting:
            if self.wait >=1:
                self.wait = self.wait - 1

            elif self.wait==0:
                self.waiting=False
                self.waited=True
                self.actuation_status='DRIVING MAIN ROUTE'
                self.plan.vr=1
        
        #driving driving
        elif not self.waiting:
            if self.is_finished:

                #if just arrived at workshop
                if (self.currentpath[1]=='W1' or self.currentpath[1]=='W2') and not self.waited:
                    self.waiting=True
                    self.waited=False
                    self.wait=150
                    self.plan.vr=0
                    self.actuation_status='AT WORKSHOP'
                
                #if at end goal
                elif self.currentpath[1]=='FIN':
                    self.plan.vr=0
                    self.actuation_status='FINISHED MISSION'
                
                
                else:
                    i=0
                    self.waited=False
                    found=False
                    while i<=len(global_route) and not found:
                        if global_route[i]==self.currentpath[0] and global_route[i+1]==self.currentpath[1]:
                            found=True
                        i=i+1
                    if not self.currentpath[1]=='FIN':
                        #update currentpath to reflect that we just passed a node on the route
                        self.currentpath[0]=self.currentpath[1]
                        self.currentpath[1]=global_route[i+1]
                    print(self.currentpath)
                    traj_x= self.next_traj_dict_x[':'.join(self.currentpath)]
                    traj_y= self.next_traj_dict_y[':'.join(self.currentpath)]
            else:
                self.plan.vr=self.plan.vl
                

 
        return svea,traj_x,traj_y

    #Depending on state variable, call different methods
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
        
    def state_3(self): # AUTOMTIC release condition
        self.operational_state='AUTO RELEASE CONDITION'
        self.state=self.plan.RC
        


def param_init():
    #Initialization handles use with just python or in a launch file
    
    # grab parameters from launch-file
    start_pt_param = rospy.search_param('start_pt')
    is_sim_param = rospy.search_param('is_sim')
    is_real_SVEA_param=rospy.search_param('is_real_SVEA')
    use_rviz_param = rospy.search_param('use_rviz')
    use_matplotlib_param = rospy.search_param('use_matplotlib')
    map_name_param = rospy.search_param('map_name')

    start_pt = rospy.get_param(start_pt_param, default_init_pt)
    if isinstance(start_pt, str):
        start_pt = start_pt.split(',')
        start_pt = [float(curr) for curr in start_pt]
        start_pt = VehicleState(*start_pt)

    is_sim = rospy.get_param(is_sim_param, True)
    is_real_SVEA=rospy.get_param(is_real_SVEA_param,True)
    use_rviz = rospy.get_param(use_rviz_param, False)
    use_matplotlib = rospy.get_param(use_matplotlib_param, False)
    map_name= rospy.get_param(map_name_param, True)

    return start_pt, is_sim, use_rviz, use_matplotlib, is_real_SVEA, map_name

def main():


    ## SIMULATION PARAMS ##########################################################
    vehicle_name = ""
    target_velocity = 1.0 # [m/s]
    dt = 0.01 # frequency of the model updates    

    ###############################################################################

    ## INIT #######################################################################
    default_init_pt = [0.0, 0.0, 0.0, 0.0] # [x, y, yaw, v], units: [m, m, rad, m/s]
    ###############################################################################





    rospy.init_node('actuation_node')
    start_pt, is_sim, use_rviz, use_matplotlib, is_real_SVEA, map_name = param_init()
    global node_dict
    node_dict=points_dict[map_name]
    SM=StateMachine(1,target_velocity)

    rospy.Subscriber('action', action, SM.callback_plan )
    rospy.Subscriber('CTcorr', ctcorr, SM.CTcorr.callback )
    rospy.Subscriber('start_system',Int8,SM.callback_start)
    pub_actuation_status=rospy.Publisher('actuation_status',node_status,queue_size=10)
    pub_map_data=rospy.Publisher('vehicle_map_data',vehicle_map_data,queue_size=10)
    

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
                           SM.next_traj_dict_x['start:N1'], SM.next_traj_dict_y['start:N1'],
                           data_handler = DataHandler)
    
    #DO not proceed until start variable is True
    while not SM.start:
        continue
    
    
    svea.start(wait=True)
    
    if is_sim:
        # start simulation
        simulator.toggle_pause_simulation()

    #Initialize distances to all different goals
    distance_W1=dist_to_goal(svea,SM.currentpath,'W1')
    distance_W2=dist_to_goal(svea,SM.currentpath,'W2')
    distance_FIN=dist_to_goal(svea,SM.currentpath,'FIN')
    
    
    #How long we have driven
    driven_dist=0

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

        #Update the trajectories and speed of the vehicle
        svea,traj_x,traj_y=SM.update_actuation(svea)

        #set the new trajectories for the simulator
        svea.update_traj(traj_x,traj_y)

        #add distance driven since last iteration
        driven_dist=driven_dist+velocity*dt

        #create and publish status messages of the node
        status_msg=node_status()
        status_msg.status1=SM.operational_state
        status_msg.status2=SM.actuation_status
        status_msg.status3=str(SM.is_finished)
        status_msg.status4=str(SM.plan.vl)
        status_msg.status5=str(velocity)
        status_msg.status6=str(driven_dist)
        pub_actuation_status.publish(status_msg)

        #update and publish distances to all goals
        distance_W1=dist_to_goal(svea,SM.currentpath,'W1')
        distance_W2=dist_to_goal(svea,SM.currentpath,'W2')
        distance_FIN=dist_to_goal(svea,SM.currentpath,'FIN')
        vhd_msg=vehicle_map_data()
        vhd_msg.dist_w1=distance_W1
        vhd_msg.dist_w2=distance_W2
        vhd_msg.dist_mgoal=distance_FIN
        vhd_msg.lastN=SM.currentpath[0]
        vhd_msg.nextN=SM.currentpath[1]
        pub_map_data.publish(vhd_msg)

    if not rospy.is_shutdown():
        rospy.loginfo("Trajectory finished!")


    rospy.spin()



if __name__ == '__main__':
	main()