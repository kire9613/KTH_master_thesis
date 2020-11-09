#!/usr/bin/env python
# -*- coding: utf-8 -*-

import casadi as ca
import numpy as np

class SVEAcar(object):
    def __init__(self, h, target_velocity, tau):
        """
        SVEA model class.

        All methods should return casadi.MX or casadi.DM variable
        types.

        :param h: sampling time, defaults to 0.1
        :type h: float, optional
        """

        # Model, gravity and sampling time parameters
        self.model_nl = self.svea_nonlinear_dynamics
        self.g = 9.81
        self.dt = h
        self.target_velocity = target_velocity
        self.TAU = tau

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

        self.Integrator = None

        self.set_integrators()
        self.set_discrete_time_system()

    def __str__(self):
        return ""

    def set_integrators(self):
        """
        Generate continuous time high-precision integrators.
        """

        # Set CasADi variables
        x = ca.MX.sym('x', 4)
        u = ca.MX.sym('u', 2)

        # Integration method - integrator options an be adjusted
        options = {"abstol" : 1e-5, "reltol" : 1e-9, "max_num_steps": 100,
                   "tf" : self.dt,
                   }

        # Create nonlinear dynamics integrator
        dae = {'x': x, 'ode': self.model_nl(x,u), 'p':ca.vertcat(u)}
        self.Integrator = ca.integrator('integrator', 'cvodes', dae, options)

    def set_discrete_time_system(self):
        """
        SVEA dynamics discretized using Euler forward step, inspired by
        PythonRobotics
        """

        # Set CasADi variables
        x = ca.MX.sym('x', 4)
        u = ca.MX.sym('u', 2)

        A = ca.MX.zeros(4,4)
        A[0, 0] = 1.0
        A[1, 1] = 1.0
        A[2, 2] = 1.0
        A[3, 3] = 1.0
        A[0, 2] = -self.dt * x[3] * ca.sin(x[2])
        A[0, 3] = self.dt * ca.cos(x[2])
        A[1, 2] = self.dt * x[3] * ca.cos(x[2])
        A[1, 3] = self.dt * ca.sin(x[2])
        A[2, 3] = self.dt * ca.tan(u[1]) / self.L

        B = ca.MX.zeros(4,2)
        B[2, 1] = self.dt * x[3] / (self.L * ca.cos(u[1]) ** 2)
        B[3, 0] = self.dt

        C = ca.MX.zeros(1,4)
        C[0,0] = self.dt * x[3] * ca.sin(x[2]) * x[2]
        C[0,1] = - self.dt * x[3] * ca.cos(x[2]) * x[2]
        C[0,2] = x[3] * u[1] / (self.L * ca.cos(u[1]) ** 2)

        self.Ad = ca.Function('Ad', [x, u], [A])
        self.Bd = ca.Function('Bd', [x, u], [B])
        self.Cd = ca.Function('Cd', [x, u], [C])

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
        yaw = x[2]
        v = x[3]

        a = u[0]
        delta = u[1]

        xdot1  = v*ca.cos(yaw)
        xdot2  = v*ca.sin(yaw)
        xdot3  = v/self.L * ca.tan(delta)
        xdot4  = a # TODO: Simulate ESC as in bicycle.py?
        # xdot4  = 1/self.TAU*(self.target_velocity-v)

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
        Cd_eq = self.Cd(self.x_eq, self.u_eq)

        return Ad_eq, Bd_eq, Cd_eq

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

        return ca.mtimes(self.Ad(x0, u), x0) + \
               ca.mtimes(self.Bd(x0, u), u )

def main():
    """
    Run some tests
    """
    svea = SVEAcar(h=0.1)

if __name__ == "__main__":
    main()
