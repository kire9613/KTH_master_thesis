#!/usr/bin/env python
from cvxpy import *
import numpy as np
import scipy as sp
from scipy import sparse
import rospy
from std_msgs.msg import Float64, String, Bool
import timeit
import random
from svea_msgs.msg import VehicleState
import math
from geometry_msgs.msg import PoseStamped


'''
MPC for control of speed and acceleration
model:
x = [v_follower-v_leader,d]
u = [f_engine,f_breaking]
'''
follower = VehicleState()
leader = VehicleState()
marker = PoseStamped()

LENGTH = 0.586  # [m]
BACKTOWHEEL = 0.16  # [m]
FRONTTOWHEEL = LENGTH - BACKTOWHEEL  # [m]

markerdist = 0.
leaderpitch = 0.
vrelmarker = 0.
markervisible = False

def followerinfo(data):
    global follower
    follower = data

def leaderinfo(data):
    global leader
    leader = data

def markerinfo(data):
    global marker
    marker = data

def leaderpitchinfo(data): 
    global leaderpitch
    leaderpitch = data.data

def vrelmarkerinfo(data):
    global vrelmarker
    vrelmarker = data.data

def markervisibilityinfo(data):
    global markervisible
    markervisible = data.data

def calculate_spacing_from_coms(leader,follower):
    xy0 = [leader.x, leader.y]
    xy1 = [follower.x, follower.y]
    dist = math.sqrt((xy0[0] - xy1[0])**2 + (xy0[1] - xy1[1])**2)
    d_communication = dist - BACKTOWHEEL - FRONTTOWHEEL
    return d_communication

def main():
    rospy.init_node('visMPC')
    rospy.sleep(1)

    is_sim_param = rospy.search_param('is_sim')
    is_sim = rospy.get_param(is_sim_param, True)

    use_camera_param = rospy.search_param('use_camera')
    use_camera = rospy.get_param(use_camera_param, True)

    #use_camera = False
    if is_sim:
        followername = 'SVEA1'
    else:
        followername = '' 
    
    if use_camera:
        rospy.Subscriber('/leadervehicle_pitch', Float64, leaderpitchinfo , queue_size=1)
        rospy.Subscriber('/observed_pose', PoseStamped, markerinfo, queue_size=1)
        rospy.Subscriber('/marker_velocity', Float64, vrelmarkerinfo, queue_size=1)
        rospy.Subscriber('/markervisibility', Bool, markervisibilityinfo, queue_size=1)
    else:
        rospy.Subscriber('/SVEA0/state', VehicleState, leaderinfo, queue_size=1)
       

    start_t = rospy.get_time()

    dt = 0.5 #dt should match MPC calculation time
    cr = 0.0
    dref = 1.5

    rospy.Subscriber(followername + '/state', VehicleState, followerinfo, queue_size=1)
    
    MPC_weight_publisher = rospy.Publisher('/MPC_weights', String, queue_size=1)
    accelMPC_pub = rospy.Publisher('/accelMPC_publisher', Float64, queue_size=1)
    brakeMPC_pub = rospy.Publisher('/MPC_brake', Float64, queue_size=1)   
    motorMPC_pub = rospy.Publisher('/MPC_motor', Float64, queue_size=1) 

    #rate = rospy.Rate(dt)

    MPC = MPC_controller(dt,cr,dref)
    weights = 'qe='+str(MPC.qe)+', qb='+str(MPC.qb)+', qv='+str(MPC.qv)+', qd='+str(MPC.qd)
    #print(str(weights))
    MPC_weight_publisher.publish(String(weights))
    MPC.update_pitch_traj([0]*6)
    alphalist = [0]*6

    while not rospy.is_shutdown():
        curr_t = rospy.get_time() - start_t

        if use_camera:
            d = marker.pose.position.z
            vrel = vrelmarker
        else:
            d = calculate_spacing_from_coms(leader,follower)
            vrel = follower.v-leader.v
            

        
        print('Solving MPC with: d = ',d,', vrel = ',vrel,' ...')
        x,u = MPC.solve(vrel,d)
        vrel_MPC = x[0]
        distance_MPC = x[1]
        accel_MPC = u[0]-u[1]
        if use_camera and not markervisible:
            accel_MPC = 0.
            brakeMPC_pub.publish(Float64(0))
            motorMPC_pub.publish(Float64(0))
            print('Marker not detected. Sending acceleration = 0')
        else:
            brakeMPC_pub.publish(Float64(u[1]))
            motorMPC_pub.publish(Float64(u[0]))
            print('Solved MPC. Wanted acceleration = ', accel_MPC)
        accelMPC_pub.publish(Float64(accel_MPC))
        MPC_weight_publisher.publish(String(weights))

class MPC_controller(object):
    def __init__(self,dt=0.1,cr=0.008,dref=0.5):
        #alphalist = np.array# List of pitch on the road
        self.dt = dt
        self.m = 10 # kg
        # Matrices
        self.Ad = sparse.csc_matrix([
          [1., 0.],
          [-dt, 1.]
        ])

        self.Bd = sparse.csc_matrix([
          [dt*self.m, -dt*self.m],
          [0, 0]
        ])

        self.g = 9.81 
        self.cr = cr # roll coefficient

        self.nu = 2
        self.nx = 2

        # Constraints
        self.umin = np.array([0.,0.])
        self.umax = np.array([.3,1.])

        # Objective function
        self.qe = 5.
        self.qb = 1.

        self.R = sparse.diags([self.qe,self.qb]) # Engine and braking acceleration weights

        self.qv = 1.2
        self.qd = 3. #40
        
        self.Q =sparse.diags([self.qv, self.qd]) # Relative velocity and distance weights
        self.dref = dref
        # Initial and reference states
        #self.x0 = np.array([0., 0.5]) # Initial [vrel, d]
        self.xr = np.array([0., self.dref]) # Reference [vrel, d]

    def update_pitch_traj(self,alphalist):
        # Prediction horizon
        self.alphalist = alphalist
        N = len(self.alphalist)

        # Define problem
        self.u = Variable((self.nu, N))
        self.x = Variable((self.nx, N+1))

        self.x_init = Parameter(self.nx)
        self.objective = 0
        self.constraints = [self.x[:,0] == self.x_init]
        f_preview = self.g*np.sin(np.deg2rad(self.alphalist[0]))+self.cr*self.g*np.cos(np.deg2rad(self.alphalist[0]))
        self.Cd_0 = np.array([-self.dt*f_preview, 0.])
        for k in range(N):
            alpha = self.alphalist[k] #use k:th element in list
            f_preview = self.g*np.sin(np.deg2rad(alpha))+self.cr*self.g*np.cos(np.deg2rad(alpha))
            self.Cd = np.array([-self.dt*f_preview, 0.]) #Change Cd for every alpha

            self.objective += quad_form(self.x[:,k] - self.xr, self.Q) + quad_form(self.u[:,k], self.R)
            self.constraints += [self.x[:,k+1] == self.Ad*self.x[:,k] + self.Bd*self.u[:,k] + self.Cd]
            self.constraints += [self.umin <= self.u[:,k], self.u[:,k] <= self.umax]

    def solve(self,vrel,distance): #Send v_rel as v_follower-v_leader
        self.x0 = np.array([vrel, distance])
        self.x_init.value = self.x0

        self.prob = Problem(Minimize(self.objective), self.constraints) 
        self.prob.solve(solver=OSQP, warm_start=True)
        self.x0 = self.Ad.dot(self.x0) + self.Bd.dot(self.u[:,0].value) + self.Cd_0
        #print('x = '+str(self.x0))
        #print('u = '+str(self.u[:,0].value))
        return self.x0, self.u[:,0].value
        

if __name__ == '__main__':
    main()

    


