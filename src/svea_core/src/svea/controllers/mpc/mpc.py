#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Model Predictive Control - CasADi interface
Adapted from Helge-André Langåker work on GP-MPC
Customized by Pedro Roque for EL2700 Model Predictive Countrol Course at KTH
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import time
import numpy as np
import casadi as ca
import casadi.tools as ctools

from scipy.stats import norm
import scipy.linalg

from svea.controllers.mpc.model import SVEAcar

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

import math

class MPC(object):
    def __init__(self, vehicle_name=''):
        self.traj_x = []
        self.traj_y = []
        self.traj_yaw = []
        self.sp = []
        self.target = None
        # initialize with 0 velocity
        self.target_velocity = 1.5
        self.last_index = 0
        self.is_finished = False

        self.target_ind = None
        self.u0 = None

        self.pred_pub = rospy.Publisher("pred",Marker,queue_size=1)
        self.ref_pub = rospy.Publisher("ref",Marker,queue_size=1)
        self.model_type = "nonlinear"
        self.solver_ = "ipopt"

    def build_solver(self, dt, model=None, dynamics=None,
                 horizon=7, Q=None, P=None, R=None,
                 ulb=None, uub=None, xlb=None, xub=None, terminal_constraint=None,
                 solver_opts=None,
                 x_d=[0]*4,
                 max_iter=3000,
                 max_cpu_time=1e6,
                 model_type = "nonlinear",
                 solver_ = "ipopt",
                 TAU = 0.1,
                 N_IND_SEARCH = 10,  # Search index number
                ):

        """ Initialize and build the MPC solver
        # Arguments:
            horizon: Prediction horizon in seconds
            model: System model
        # Optional Argumants:
            Q: State penalty matrix, default=diag(1,...,1)
            P: Termial penalty matrix, default=diag(1,...,1)
            R: Input penalty matrix, default=diag(1,...,1)*0.01
            ulb: Lower boundry input
            uub: Upper boundry input
            xlb: Lower boundry state
            xub: Upper boundry state
            terminal_constraint: Terminal condition on the state
                    * if None: No terminal constraint is used
                    * if zero: Terminal state is equal to zero
                    * if nonzero: Terminal state is bounded within +/- the constraint
            solver_opts: Additional options to pass to the NLP solver
                    e.g.: solver_opts['print_time'] = False
                          solver_opts['ipopt.tol'] = 1e-8
        """
        self.TAU = TAU
        self.N_IND_SEARCH = N_IND_SEARCH
        self.model_type = model_type
        model = SVEAcar(dt,target_velocity=self.target_velocity,tau=self.TAU)
        if self.model_type=="linear":
            dynamics = model.discrete_time_dynamics
            self.linear = True
        elif self.model_type=="nonlinear":
            dynamics = model.continuous_time_nonlinear_dynamics
            self.linear = False

        self.horizon = horizon*dt

        build_solver_time = -time.time()
        self.dt = model.dt
        self.Nx, self.Nu = 4, 2
        Nopt = self.Nu + self.Nx
        self.Nt = int(self.horizon/self.dt)
        self.dynamics = dynamics

        # Initialize variables
        self.set_cost_functions()
        self.x_sp = None


        # Cost function weights
        if P is None:
            P = np.eye(self.Nx) * 10
        if Q is None:
            Q = np.eye(self.Nx)
        if R is None:
            R = np.eye(self.Nu) * 0.01

        self.Q = ca.MX(Q)
        self.P = ca.MX(P)
        self.R = ca.MX(R)

        if xub is None:
            xub = np.full((self.Nx), np.inf)
        if xlb is None:
            xlb = np.full((self.Nx), -np.inf)
        if uub is None:
            uub = np.full((self.Nu), np.inf)
        if ulb is None:
            ulb = np.full((self.Nu), -np.inf)


        # Starting state parameters - add slack here
        x0       = ca.MX.sym('x0', self.Nx)
        x0_ref   = ca.MX.sym('x0_ref', self.Nx*(self.Nt+1))
        # x0_ref   = ca.MX.sym('x0_ref', self.Nx)
        u0       = ca.MX.sym('u0', self.Nu)
        param_s  = ca.vertcat(x0, x0_ref, u0)


        # Create optimization variables
        opt_var = ctools.struct_symMX([(
                ctools.entry('u', shape=(self.Nu,), repeat=self.Nt),
                ctools.entry('x', shape=(self.Nx,), repeat=self.Nt+1),
        )])
        self.opt_var = opt_var
        self.num_var = opt_var.size


        # Decision variable boundries
        self.optvar_lb = opt_var(-np.inf)
        self.optvar_ub = opt_var(np.inf)


        """ Set initial values """
        obj = ca.MX(0)
        con_eq = []
        con_ineq = []
        con_ineq_lb = []
        con_ineq_ub = []
        con_eq.append(opt_var['x', 0] - x0)

        # Generate MPC Problem
        for t in range(self.Nt):

            # Get variables
            x_t = opt_var['x', t]
            u_t = opt_var['u', t]

            # Dynamics constraint
            x_t_next = self.dynamics(x_t, u_t)
            con_eq.append(x_t_next - opt_var['x', t+1])

            # Input constraints
            if uub is not None:
                con_ineq.append(u_t)
                con_ineq_ub.append(uub)
                con_ineq_lb.append(np.full((self.Nu,), -ca.inf))
            if ulb is not None:
                con_ineq.append(u_t)
                con_ineq_ub.append(np.full((self.Nu,), ca.inf))
                con_ineq_lb.append(ulb)

            # State constraints
            if xub is not None:
                con_ineq.append(x_t)
                con_ineq_ub.append(xub)
                con_ineq_lb.append(np.full((self.Nx,), -ca.inf))
            if xlb is not None:
                con_ineq.append(x_t)
                con_ineq_ub.append(np.full((self.Nx,), ca.inf))
                con_ineq_lb.append(xlb)

            # Objective Function / Cost Function
            # print((t+1)*self.Nx)
            # print(self.Nx*(self.Nt+1))
            # print("size of x_t",x_t.size())
            # print("size of xref",x0_ref[t*self.Nx:(t+1)*self.Nx].size())
            obj += self.running_cost( (x_t - x0_ref[t*self.Nx:(t+1)*self.Nx]), self.Q, u_t, self.R)

        # Terminal Cost
        obj += self.terminal_cost(opt_var['x', self.Nt] - x0_ref[self.Nt*self.Nx:], self.P)

        # Terminal contraint
        if terminal_constraint is not None:
            con_ineq.append(opt_var['x', self.Nt] - x0_ref[self.Nt*self.Nx:])
            con_ineq_lb.append(np.full((self.Nx,), - np.array(terminal_constraint)))
            con_ineq_ub.append(np.full((self.Nx,),   np.array(terminal_constraint)))
            # con_ineq_lb.append(np.full((self.Nx,), - terminal_constraint))
            # con_ineq_ub.append(np.full((self.Nx,), terminal_constraint))

        # Equality constraints bounds are 0 (they are equality constraints),
        # -> Refer to CasADi documentation
        num_eq_con = ca.vertcat(*con_eq).size1()
        num_ineq_con = ca.vertcat(*con_ineq).size1()
        con_eq_lb = np.zeros((num_eq_con,))
        con_eq_ub = np.zeros((num_eq_con,))

        # Set constraints
        con = ca.vertcat(*(con_eq+con_ineq))
        self.con_lb = ca.vertcat(con_eq_lb, *con_ineq_lb)
        self.con_ub = ca.vertcat(con_eq_ub, *con_ineq_ub)

        # Build NLP Solver (can also solve QP)
        nlp = dict(x=opt_var, f=obj, g=con, p=param_s)
        options = {
            'verbose' : False,
            'print_time' : False,
        }
        if self.linear:
            options.update({
                # 'osqp.verbose': False,
                # 'warm_start_primal': True,
                # 'error_on_fail': False,
                'expand' : True,
                'jit': True,
            })
        if self.solver_=="ipopt":
            options.update({
                'ipopt.print_level' : 0,
                'ipopt.mu_init' : 0.01,
                'ipopt.tol' : 1e-8,
                'ipopt.sb' : 'yes',
                'ipopt.warm_start_init_point' : 'yes',
                'ipopt.warm_start_bound_push' : 1e-9,
                'ipopt.warm_start_bound_frac' : 1e-9,
                'ipopt.warm_start_slack_bound_frac' : 1e-9,
                'ipopt.warm_start_slack_bound_push' : 1e-9,
                'ipopt.warm_start_mult_bound_push' : 1e-9,
                'ipopt.mu_strategy' : 'adaptive',
                'ipopt.max_iter': max_iter,
                'ipopt.max_cpu_time': max_cpu_time,
            })
        if solver_opts is not None:
            options.update(solver_opts)
        if self.solver_=="ipopt":
            self.solver = ca.nlpsol('mpc_solver', 'ipopt', nlp, options)
        elif self.solver_=="osqp":
            self.solver = ca.qpsol('mpc_solver', 'osqp', nlp, options)

        build_solver_time += time.time()
        print('\n________________________________________')
        print('# Time to build mpc solver: %f sec' % build_solver_time)
        print('# Number of variables: %d' % self.num_var)
        print('# Number of equality constraints: %d' % num_eq_con)
        print('# Number of inequality constraints: %d' % num_ineq_con)
        print('----------------------------------------\n')

    def set_cost_functions(self):

        # Create functions and function variables for calculating the cost
        Q = ca.MX.sym('Q', self.Nx, self.Nx)
        R = ca.MX.sym('R', self.Nu, self.Nu)
        P = ca.MX.sym('P', self.Nx, self.Nx)

        x = ca.MX.sym('x', self.Nx)
        u = ca.MX.sym('q', self.Nu)

        #
        self.running_cost = ca.Function('Jstage', [x, Q, u, R], \
                                      [ca.mtimes(x.T, ca.mtimes(Q, x)) + ca.mtimes(u.T, ca.mtimes(R, u))])

        self.terminal_cost = ca.Function('Jtogo', [x, P], \
                                  [ca.mtimes(x.T, ca.mtimes(P, x))] )

    def solve_mpc(self, x0, u0=None):
        """ Solve the optimal control problem
        # Arguments:
            x0: Initial state vector.
            sim_time: Simulation length.
        # Optional Arguments:
            x_sp: State set point, default is zero.
            u0: Initial input vector.
            debug: If True, print debug information at each solve iteration.
            noise: If True, add gaussian noise to the simulation.
            con_par_func: Function to calculate the parameters to pass to the
                          inequality function, inputs the current state.
        # Returns:
            mean: Simulated output using the optimal control inputs
            u: Optimal control inputs
        """

        # Initial state
        if u0 is None:
            u0 = np.zeros(self.Nu)
        if self.x_sp is None:
            self.x_sp = np.zeros(self.Nx)


        # Initialize variables
        self.optvar_x0          = np.full((1, self.Nx), x0.T)

        # Initial guess of the warm start variables
        self.optvar_init = self.opt_var(0)
        self.optvar_init['x', 0] = self.optvar_x0[0]

        # print('\nSolving MPC with %d step horizon' % self.Nt)
        solve_time = -time.time()

        param = ca.vertcat(x0, self.x_sp, u0)
        # print(x0.shape)
        # print(self.x_sp.shape)
        # print(u0.shape)
        args = dict(x0=self.optvar_init,
                    lbx=self.optvar_lb,
                    ubx=self.optvar_ub,
                    lbg=self.con_lb,
                    ubg=self.con_ub,
                    p=param)

        # Solve NLP
        sol             = self.solver(**args)
        status          = self.solver.stats()['return_status']
        optvar          = self.opt_var(sol['x'])

        print(status)


        solve_time+=time.time()
        print('MPC took %f seconds to solve.\r' %(solve_time))
        print('MPC cost: ', sol['f'])
        #print("optvar[x] = ", optvar['x'], "optvar['u'] = ", optvar['u'])
        return optvar['x'], optvar['u']

    def set_reference(self, x_sp):
        self.x_sp = x_sp

    def calc_ref_trajectory(self, state, pind):
        cx = self.traj_x
        cy = self.traj_y
        cyaw = self.traj_yaw
        sp = self.sp
        dind = None

        xref = np.zeros((self.Nx, self.Nt + 1))
        dref = np.zeros((1, self.Nt + 1))
        ncourse = len(cx)

        ind, _ = self.calc_nearest_index(state, pind)

        if pind >= ind:
            ind = pind

        #  TODO: Fake look ahead, change this later <20-11-20, rob> #
        # ind += 1

        xref[0, 0] = cx[ind]
        xref[1, 0] = cy[ind]
        xref[2, 0] = cyaw[ind]
        xref[3, 0] = sp[ind]
        # dref[0, 0] = 0.0  # steer operational point should be 0

        travel = 0.0

        for i in range(self.Nt + 1):
            # travel += abs(state[3]) * self.dt
            if dind==None:
                travel += abs(sp[ind]) * self.dt
            else:
                travel += abs(sp[ind+dind]) * self.dt
            dind = int(round(travel / self.dl))

            if (ind + dind) < ncourse:
                xref[0, i] = cx[ind + dind]
                xref[1, i] = cy[ind + dind]
                xref[2, i] = cyaw[ind + dind]
                xref[3, i] = sp[ind + dind]
                # dref[0, i] = 0.0
            else:
                xref[0, i] = cx[ncourse - 1]
                xref[1, i] = cy[ncourse - 1]
                xref[2, i] = cyaw[ncourse - 1]
                xref[3, i] = sp[ncourse - 1]
                # dref[0, i] = 0.0
        # print(xref[0,:],xref[1,:])

        marker_msg = Marker()
        marker_msg.header.stamp = rospy.Time.now()
        marker_msg.header.frame_id = "map"
        marker_msg.points = [Point(x=xref[0,i],y=xref[1,i],z=0.22) for i in range(self.Nt+1)]

        marker_msg.type = Marker.POINTS
        marker_msg.action = Marker.ADD

        marker_msg.id = 0
        marker_msg.scale.x = 0.2
        marker_msg.scale.y = 0.2
        marker_msg.color = ColorRGBA(r=1, g=0, b=0, a=1)
        marker_msg.pose.orientation.w = 1.0

        self.ref_pub.publish(marker_msg)

        # self.ref_target = (xref[0,:],xref[1,:])
        self.x_sp = xref.flatten(order='F')

        return ind, dref

    def calc_nearest_index(self,state, pind):
        if state[0]==0.0 and state[1]==0.0:
            #  TODO: Find way to wait until state properly initalized <17-11-20, rob> #
            rospy.loginfo("mpc.py: No state loaded")
            return 0, 0
        cx = self.traj_x
        cy = self.traj_y
        cyaw = self.traj_yaw

        #  TODO: Search might have to include numbers in a backward direction
        #  also <20-11-20, rob> #
        dx = [state[0] - icx for icx in cx[pind:(pind + self.N_IND_SEARCH)]]
        dy = [state[1] - icy for icy in cy[pind:(pind + self.N_IND_SEARCH)]]

        d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

        mind = min(d)

        ind = d.index(mind) + pind

        mind = math.sqrt(mind)

        dxl = cx[ind] - state[0]
        dyl = cy[ind] - state[1]

        angle = self.pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
        if angle < 0:
            mind *= -1

        return ind, mind

    def pi_2_pi(self,angle):
        while(angle > math.pi):
            angle = angle - 2.0 * math.pi

        while(angle < -math.pi):
            angle = angle + 2.0 * math.pi

        return angle

    def compute_control(self, state, u0=None):
        x0 = np.array([state.x,state.y,state.yaw,state.v])

        if self.u0 is None:
            self.u0 = np.zeros(self.Nu)

        if self.target_ind is None:
            self.target_ind, _ = self.calc_nearest_index(x0, 0)

        print(self.target_ind)
        self.target_ind, _ = self.calc_nearest_index(x0, self.target_ind)

        # self.target_ind, _ = self.calc_nearest_index(x0, 0)

        self.calc_ref_trajectory(x0, self.target_ind)

        x_pred, u_pred = self.solve_mpc(x0,self.u0)
        print("u_pred", u_pred[0][1])

        marker_msg = Marker()
        marker_msg.header.stamp = rospy.Time.now()
        marker_msg.header.frame_id = "map"
        marker_msg.points = [Point(x=i[0],y=i[1],z=0.22) for i in x_pred]

        marker_msg.type = Marker.POINTS
        marker_msg.action = Marker.ADD

        marker_msg.id = 0
        marker_msg.scale.x = 0.2
        marker_msg.scale.y = 0.2
        marker_msg.color = ColorRGBA(r=0, g=0, b=1, a=1)
        marker_msg.pose.orientation.w = 1.0

        self.pred_pub.publish(marker_msg)
        #print("u_pred[0]", u_pred[1])
        #self.u0 = u_pred[1]

        # self.target =  ([i[0] for i in x_pred], [i[1] for i in x_pred])
        # self.target =  (self.ref_target[0], self.ref_target[1])
        # t0 =  [ [i[0] for i in x_pred] , self.ref_target[0][:] ][:]
        # t1 =  [ [i[1] for i in x_pred] , self.ref_target[1][:] ][:]
        # self.target = (t0,t1)
        # self.target =  (self.ref_target[0][0], self.ref_target[1][0]) # FOR RVIZ

        vd = self.TAU*u_pred[0][0] + state.v
        #vd = 0.6
        #vd = x_pred[1][3]
        print("vd = ", vd)

        return float(u_pred[0][1]), float(vd)


def main():
    """
    Run some tests
    """
    svea = SVEAcar(h=0.1,target_velocity=1.,tau=0.1)

    horizon = 10
    x_d=[0,0,0,0]
    u_d=[0,0]
    svea.set_equilibrium_point(x_d, u_d)

    A, B, C = svea.get_discrete_system_matrices_at_eq()
    Q = np.eye(4)
    R = np.eye(2)
    P_LQR = np.matrix(scipy.linalg.solve_discrete_are(A, B, Q, R))
    P_LQR = np.eye(4)

    ulb = [-1e2,-np.deg2rad(40)]
    uub = [ 1e2, np.deg2rad(40)]
    xlb = [-np.inf]*3+[ 1.5]
    xub = [ np.inf]*3+[-1.5]

    ctl = MPC(""
        # model=svea,
        # dynamics=svea.discrete_time_dynamics,
        # Q = Q , R = R, P = P_LQR,
        # horizon=horizon,
        # ulb=ulb,
        # uub= uub,
        # xlb=xlb,
        # xub=xub,
        # x_d=x_d,
        # terminal_constraint=...,
    )

if __name__ == "__main__":
    main()
