#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
For easier keeping track of tuning settings and parameters.
"""

import numpy as np

class Params(object):
    """ Holds parameters """
    def __init__(self, P, Q, R, model_type, solver_, dt, horizon):
        self.P = P
        self.Q = Q
        self.R = R
        self.model_type = model_type
        self.solver_ = solver_
        self.dt = dt
        self.horizon = horizon

parameters = {
    "test": Params(
        P=np.diag([1200, 1200, 1, 100]),
        Q=np.diag([100, 100, 0.01, 100]),
        R=np.diag([1,0.01]),
        model_type = "linear",
        solver_ = "osqp",
        dt = 0.1,
        horizon = 10,
    ),
}

def main():
    pass

if __name__ == "__main__":
    main()
