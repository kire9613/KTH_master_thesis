#!/usr/bin/env python

"""
Module containing path following SVEA managers based on MPC
"""
import math

from svea.path_planners import cubic_spline_planner
from path_following_sveas import SVEAPurePursuit
from svea.data import TrajDataHandler
from svea_archetypes import SVEAManager

class SVEAMPC(SVEAPurePursuit):
    """docstring for SVEAMPC"""

    def __init__(self, vehicle_name, localizer, controller, traj_x, traj_y,
                 data_handler=TrajDataHandler,
                 target_velocity=1.0,
                 dl=0.2,
        ):

        SVEAManager.__init__(self, vehicle_name, localizer, controller,
                                   data_handler = data_handler)

        self.controller = controller(vehicle_name,target_velocity,dl)

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
            cx,cy,cyaw,self.controller.target_velocity
        )

    def calc_speed_profile(self, cx, cy, cyaw, target_speed):

        speed_profile = [target_speed] * len(cx)
        direction = 1.0  # forward

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
                speed_profile[i] = target_speed
        #print("speed profile=",speed_profile)
        speed_profile[-1] = 0.0

        return speed_profile
