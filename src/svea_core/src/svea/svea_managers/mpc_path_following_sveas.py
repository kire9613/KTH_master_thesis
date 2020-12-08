#!/usr/bin/env python

"""
Module containing path following SVEA managers based on MPC
"""
from __future__ import division
from __future__ import print_function

import math
import numpy as np

from svea.path_planners import cubic_spline_planner
from path_following_sveas import SVEAPurePursuit
from svea.data import TrajDataHandler
from svea_archetypes import SVEAManager

from svea.controllers.pure_pursuit import PurePursuitController

class SVEAMPC(SVEAPurePursuit):
    """docstring for SVEAMPC"""

    def __init__(self, vehicle_name, localizer, controller, traj_x, traj_y,
                 data_handler=TrajDataHandler,
                 target_velocity=1.0,
                 low_lim=0.5,
                 dl=0.1,
                 pid=PurePursuitController,
        ):

        SVEAManager.__init__(self, vehicle_name, localizer, controller,
                                   data_handler = data_handler)

        self.controller = controller(vehicle_name,target_velocity,low_lim,dl)
        self.pid = pid(vehicle_name)

        self.update_traj(traj_x, traj_y)

        # goto parameters
        self.goto_vel = 0.6 # m/s
        self.goto_thresh = 0.05 # m

    def update_traj(self, traj_x, traj_y):
        """Update trajectory

        :param traj_x: X coordinates of trajectory, defaults to []
        :type traj_x: list
        :param traj_y: Y coordinates of trajectory, defaults to []
        :type traj_y: list
        """

        assert len(traj_x) == len(traj_y)
        cx, cy, cyaw, ckappa, s = cubic_spline_planner.calc_spline_course(
            traj_x, traj_y, ds=self.controller.dl
        )
        self.data_handler.update_traj(traj_x, traj_y)
        self.controller.traj_x = cx
        self.controller.traj_y = cy
        self.controller.traj_yaw = cyaw
        self.controller.sp = self.calc_speed_profile(
            cx,cy,cyaw,ckappa,self.controller.target_velocity,self.controller.low_lim
        )
        self.pid.traj_x = cx
        self.pid.traj_y = cy

    def calc_speed_profile(self, cx, cy, cyaw, ckappa, target_speed, low_lim=None):

        speed_profile = [target_speed] * len(cx)
        direction = 1.0  # forward

        if low_lim == None:
            low_lim = target_speed/2

        curv = np.array(np.abs(ckappa))/abs(max(ckappa))

        # Set stop point
        for i in range(len(cx) - 1):
            dx = cx[i + 1] - cx[i]
            dy = cy[i + 1] - cy[i]

            move_direction = math.atan2(dy, dx)

            if dx != 0.0 and dy != 0.0:
                dangle = abs(self.controller.pi_2_pi(move_direction - cyaw[i]))
                if dangle >= math.pi / 4.0:
                    direction = -1.0
                else:
                    direction = 1.0

            if direction != 1.0:
                speed_profile[i] = - target_speed
            else:
                speed_profile[i] = max(target_speed*(1-curv[i]),low_lim)
        #print("speed profile=",speed_profile)
        speed_profile[-1] = 0.0

        return speed_profile

    def compute_pid_control(self, state=None):
        """Compute control for path-following using pure-pursuit

        :param state: State used to compute control; if no state is
                      given as an argument, self.state is automatically
                      used instead, defaults to None
        :type state: VehicleState, or None

        :return: Computed steering and velocity inputs from pure-pursuit
                 algorithm
        :rtype: float, float
        """
        if state is None:
            steering, velocity = self.pid.compute_control(self.state)
            self.data_handler.update_target(self.pid.target)
        else:
            steering, velocity = self.pid.compute_control(state)
            self.data_handler.update_target(self.pid.target)
        return steering, velocity