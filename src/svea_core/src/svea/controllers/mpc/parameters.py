#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
For easier keeping track of tuning settings and parameters.
"""

import numpy as np

class Params(object):
    """ Holds parameters """
    def __init__(self, P, Q, R):
        self.P = P
        self.Q = Q
        self.R = R

parameters = {
    "test": Params(
        P=np.diag([1200, 1200, 1, 100]),
        Q=np.diag([100, 100, 0.01, 100]),
        R=np.diag([1,0.01]),
    ),
}

def main():
    pass

if __name__ == "__main__":
    main()
