#!/usr/bin/env python
# -*- coding: utf-8 -*-

import casadi as ca
import numpy as np

class SVEAcar(object):
    def __init__(self, x_d, h):
        """
        Quadrotor model class.

        All methods should return casadi.MX or casadi.DM variable
        types.

        :param h: sampling time, defaults to 0.1
        :type h: float, optional
        """

        # Model, gravity and sampling time parameters
        self.model = self.svea_linear_dynamics
        self.model_nl = self.svea_nonlinear_dynamics
        self.model_ag = self.svea_augmented_dynamics
        self.model_ag_nl = self.svea_augmented_nonlinear_dynamics
        self.g = 9.81
        self.dt = h

        # System reference (x_d) and disturbance (w)
        self.x_d = x_d

        self.L = 0.32 # The wheel base

        # Linearize system around
        self.x_eq = [0, # x
                     0, # y
                     0, # yaw
                     0, # velocity
                     ]
        self.u_eq = [0, # acceleration or velocity?
                     0, # δ: steering
                     ]

        self.Integrator_lin = None
        self.Integrator = None
        self.Integrator_ag = None
        self.Integrator_ag_nl = None

        self.set_svea_discretized_dynamics(self.x_eq)
        self.set_integrators() # TODO: Necessary to call this 2 times??
        self.set_discrete_time_system()
        self.set_integrators()

    def __str__(self):
        return ""

    def set_integrators(self):
        """
        Generate continuous time high-precision integrators.
        """

        # Set CasADi variables
        x = ca.MX.sym('x', 12)
        u = ca.MX.sym('u', 4)
        w = ca.MX.sym('w', 1)

        # Integration method - integrator options an be adjusted
        options = {"abstol" : 1e-5, "reltol" : 1e-9, "max_num_steps": 100,
                   "tf" : self.dt,
                   }

        # Create linear dynamics integrator
        dae = {'x': x, 'ode': self.model(x,u), 'p':ca.vertcat(u)}
        self.Integrator_lin = ca.integrator('integrator', 'cvodes', dae, options)

        # Create nonlinear dynamics integrator
        dae = {'x': x, 'ode': self.model_nl(x,u), 'p':ca.vertcat(u)}
        self.Integrator = ca.integrator('integrator', 'cvodes', dae, options)

    def set_discrete_time_system(self):
        """
        Set discrete-time system matrices from linear continuous dynamics.
        """

        # Check for integrator definition
        if self.Integrator_lin is None:
            print("Integrator_lin not defined. Set integrators first.")
            exit()

        # Set CasADi variables
        x = ca.MX.sym('x', 12)
        u = ca.MX.sym('u', 4)

        # Jacobian of exact discretization
        self.Ad = ca.Function('jac_x_Ad', [x, u], [ca.jacobian(
                            self.Integrator_lin(x0=x, p=ca.vertcat(u))['xf'], x)])
        self.Bd = ca.Function('jac_u_Bd', [x, u], [ca.jacobian(
                            self.Integrator_lin(x0=x, p=ca.vertcat(u))['xf'], u)])


        # C matrix does not depend on the state
        # TODO: put this in a better place later!
        Cd_eq = ca.DM.eye(12)


        self.Cd_eq = Cd_eq

    def svea_linear_dynamics(self, x, u):
        """
        Pendulum continuous-time linearized dynamics.

        :param x: state
        :type x: MX variable, 4x1
        :param u: control input
        :type u: MX variable, 1x1
        :param w: disturbance
        :type w: MX variable, 1x1
        :return: dot(x)
        :rtype: MX variable, 4x1
        """

        Ac = ca.MX.zeros(12,12)
        Bc = ca.MX.zeros(12,4)

        # Ac = ca.DM.zeros(12,12)
        # Bc = ca.DM.zeros(12,4)

        yaw = self.yaw

        ### Build Ac matrix
        Ac[0:3,3:6] = np.eye(3)
        Ac[6:9,9:12] = np.eye(3)
        Ac[3,6] = ca.sin(yaw)
        Ac[3,7] = ca.cos(yaw)
        Ac[4,7] = ca.sin(yaw)
        Ac[4,6] = -ca.cos(yaw)

        ### Build Bc matrix
        Bc[5, 0] = 1./(self.m)
        Bc[9, 1] = 1./self.I_x
        Bc[10,2] = 1./self.I_y
        Bc[11,3] = 1./self.I_z


        ### Store matrices as class variables
        self.Ac = Ac
        self.Bc = Bc
        # print(Ac)
        # print(Bc)

        return ca.mtimes(Ac, x) + ca.mtimes(Bc, u)

    def set_svea_discretized_dynamics(self, x):
        # TODO: DISCRETEIZED MODEL HERE (Atsushi)
        pass

    def svea_nonlinear_dynamics(self, x, u, *_):
        """
        Pendulum nonlinear dynamics.

        :param x: state
        :type x: casadi.DM or casadi.MX
        :param u: control input
        :type u: casadi.DM or casadi.MX
        :return: state time derivative
        :rtype: casadi.DM or casadi.MX, depending on inputs
        """
        x = x[0]
        y = x[1]
        yaw = x[2]
        v = x[3]

        a = u[0]
        delta = u[1]

        xdot1  = v*ca.cos(yaw)
        xdot2  = v*ca.sin(yaw)
        xdot3  = v/self.L * ca.tan(delta)
        xdot4  = a # TODO: Simulate ESC as in bicycle.py?

        dxdt = [xdot1,
                xdot2,
                xdot3,
                xdot4,
                ]

        return ca.vertcat(*dxdt)

    def set_reference(self, ref):
        """
        Simple method to set the new system reference.

        :param ref: desired reference [m]
        :type ref: float or casadi.DM 1x1
        """
        self.x_d = ref

    def set_u0(self, u0):
        """
        Simple method to set the new system reference.

        :param ref: desired reference [m]
        :type ref: float or casadi.DM 1x1
        """
        self.u0 = u0

    def set_equilibrium_point(self, x_eq, u_eq):
        """
        Set a different equilibrium poin for the system.

        :param x_eq: state equilibrium
        :type x_eq: list with 4 floats, [a,b,c,d]
        :param u_eq: control input for equilibrium point
        :type u_eq: float
        """

        self.x_eq = x_eq
        self.u_eq = u_eq

    def get_discrete_system_matrices_at_eq(self):
        """
        Evaluate the discretized matrices at the equilibrium point

        :return: A, B, C matrices for equilibrium point
        :rtype: casadi.DM
        """
        Ad_eq = self.Ad(self.x_eq, self.u_eq)
        Bd_eq = self.Bd(self.x_eq, self.u_eq)

        return Ad_eq, Bd_eq, self.Cd_eq

    def continuous_time_linear_dynamics(self, x0, u):
        """
        Perform a time step iteration in continuous dynamics.

        :param x0: initial state
        :type x0: 4x1 ( list [a, b, c, d] , ca.MX )
        :param u: control input
        :type u: scalar, 1x1
        :return: dot(x), time derivative
        :rtype: 4x1, ca.DM
        """
        out = self.Integrator_lin(x0=x0, p=ca.vertcat(u))
        return out["xf"]

    def continuous_time_nonlinear_dynamics(self, x0, u):
        out = self.Integrator(x0=x0, p=ca.vertcat(u))
        return out["xf"]

    def discrete_time_dynamics(self,x0,u):
        """
        Performs a discrete time iteration step.

        :param x0: initial state
        :type x0: 4x1 ( list [a, b, c, d] , ca.MX )
        :param u: control input
        :type u: scalar, 1x1
        :return: next discrete time state
        :rtype: 4x1, ca.DM
        """

        return ca.mtimes(self.Ad(self.x_eq, self.u_eq,  ), x0) + \
                ca.mtimes(self.Bd(self.x_eq, self.u_eq, ), u)
