#!/usr/bin/env python
from cvxpy import *
import numpy as np
import scipy as sp
from scipy import sparse
#import rospy
from std_msgs.msg import Float64
import timeit
import random

def main():
    #print('hej')
    MPC1 = MPC_controller()
    MPC2 = MPC_controller()
    alphalist_konst = [random.randint(1,5)]*10
    MPC2.update_pitch_traj(alphalist_konst)
    for k in range(1,100):
        alphalist = [random.randint(1,5)]*10
        mpctime1 = timeit.timeit(stmt = lambda: timefunction(MPC1,alphalist), number=1)
        print('MPC 1 tog: ', round(mpctime1, 4), ' sekunder')
        mpctime2 = timeit.timeit(stmt = lambda: timefunction2(MPC2), number=1)  
        print('MPC 2 tog: ', round(mpctime2, 4), ' sekunder')
        #MPC.update_pitch_traj(alphalist)
        #MPC.solve()
        #alphalist = [random.randint(1,5)]*10
        #MPC.update_pitch_traj(alphalist)
        #MPC.solve()
        print('Solve time: ', MPC2.prob.solver_stats.solve_time, 'Setup integers ', MPC2.prob.solver_stats.setup_time)   

def timefunction(MPC1,alphalist):
    MPC1.update_pitch_traj(alphalist)
    MPC1.solve()

def timefunction2(MPC2):
    MPC2.solve()


class MPC_controller(object):
    def __init__(self,dt=1.,cr=0.):
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
        qe = 2     
        qb = 20.    

        self.R = sparse.diags([qe,qb]) # Engine and braking acceleration weights

        qv = 0.
        qd = 20. 
        
        self.Q =sparse.diags([qv, qd]) # Relative velocity and distance weights

        # Initial and reference states
        self.x0 = np.array([0., 0.5]) # Initial [vrel, d]
        self.xr = np.array([0., 1.]) # Reference [vrel, d]

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
        #self.prob = Problem(Minimize(self.objective), self.constraints)

#    def simulate(self):
#        self.prob = Problem(Minimize(self.objective), self.constraints)
#
#        # Simulate in closed loop
#        nsim = 100
#        for i in range(nsim):
#            self.x_init.value = self.x0
#            self.prob.solve(solver=OSQP, warm_start=True)
#            self.x0 = self.Ad.dot(self.x0) + self.Bd.dot(self.u[:,0].value) + self.Cd_0
#            print('x'+str(i)+' = '+str(self.x0))
#            print('u, '+str(i)+': '+str(self.u[:,0].value))

    def solve(self):
        self.prob = Problem(Minimize(self.objective), self.constraints)
        self.x_init.value = self.x0
        self.prob.solve(solver=OSQP, warm_start=True)
        self.x0 = self.Ad.dot(self.x0) + self.Bd.dot(self.u[:,0].value) + self.Cd_0
        print('x = '+str(self.x0))
        print('u = '+str(self.u[:,0].value))
        return self.x0, self.u[:,0].value
        
    
    #def problem(self):
    #    self.prob = Problem(Minimize(self.objective), self.constraints)

if __name__ == '__main__':
    main()


