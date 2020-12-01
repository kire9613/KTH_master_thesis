#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
For easier keeping track of tuning settings and parameters.
"""

import numpy as np

class Params(object):
    """ Holds parameters """
    def __init__(self, P=np.diag([1,1,1,1]), Q=np.diag([1,1,1,1]),
                 R=np.diag([1,1]), model_type="linear", solver_="osqp", dt=0.05,
                 horizon=5, TAU=0.1, N_IND_SEARCH=10,
                 target_velocity=1.0,dl=0.2):
        self.P = P
        self.Q = Q
        self.R = R
        self.model_type = model_type
        self.solver_ = solver_
        self.dt = dt
        self.horizon = horizon
        self.TAU = TAU
        self.N_IND_SEARCH = horizon
        self.target_velocity = target_velocity
        self.dl = dl

parameters = {
    "test": Params(
        P=np.diag([50,50,0.5,1000]),
        Q=np.diag([1,1,0.1,1000]),
        R=np.diag([1,1]),
        model_type = "linear",
        solver_ = "osqp",
        dt = 0.1,
        horizon = 7,
        TAU=0.5,
        target_velocity = 1.5,
    ),
    "ZOH-good": Params(
        P=np.diag([10, 10, 0.01, 500]),
        Q=np.diag([1, 1, 1e-12, 1000]),
        R=np.diag([1,1]),
        model_type = "linear",
        solver_ = "osqp",
        dt = 0.1,
        horizon = 7,
    ),
    "from-svea": Params(
        P=np.diag([50,50,0.5,1000]),
        Q=np.diag([1,1,0.1,1000]),
        R=np.diag([1,1]),
        model_type = "linear",
        solver_ = "osqp",
        dt = 0.1,
        horizon = 7,
        TAU=0.5,
        target_velocity = 1.5,
    ),
}

def main():
    pass

if __name__ == "__main__":
    main()
