"""
Adapted from Atsushi Sakai's PythonRobotics pure pursuit example
"""
import math
import numpy as np

class PurePursuitController(object):

    k = 0.6  # look forward gain
    Lfc = 0.4  # look-ahead distance
    K_p = 1.0  #TODO speed control propotional gain
    K_i = 0.2  #TODO speed control integral gain
    K_d = 0.0  #TODO speed control derivitive gain
    L = 0.324  # [m] wheel base of vehicle
    e_sum = 0
    e_tmin1 = 0

    def __init__(self, vehicle_name=''):
        self.traj_x = []
        self.traj_y = []
        self.target = None
        # initialize with 0 velocity
        self.target_velocity = 0.0
        self.last_index = 0
        self.is_finished = False

    def compute_control(self, state, target=None):
        steering = self.compute_steering(state, target)
        velocity = self.compute_velocity(state)
        return steering, velocity

    def compute_steering(self, state, target=None):
        if target is None:
            self.find_target(state)
        else:
            # allow manual setting of target
            self.target = target

        tx, ty = self.target
        alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw
        if state.v < 0:  # back
            alpha = math.pi - alpha
        Lf = self.k * state.v + self.Lfc
        delta = math.atan2(2.0 * self.L * math.sin(alpha) / Lf, 1.0)
        return delta

    def compute_velocity(self, state):
        if self.is_finished:
            # stop moning if trajectory done
            return 0.0
        else:
            # speed control
            #TODO
            e = self.target_velocity - state.v
            self.e_sum += e
            if e*self.e_tmin1<0: # anti-windup, reset when crossing zero
                self.e_sum=0
            P = e*self.K_p
            I = self.e_sum*self.K_i
            D = (e-self.e_tmin1)*self.K_d/0.01 # dt = 0.01, first order approx
            self.e_tmin1 = e
            u = P + I
            speed_lim = 1.50
            u = np.clip(u,-speed_lim+self.target_velocity,speed_lim-self.target_velocity)
            return self.target_velocity + u

    def find_target(self, state):
        ind = self._calc_target_index(state)
        tx = self.traj_x[ind]
        ty = self.traj_y[ind]
        self.target = (tx, ty)

    def _calc_target_index(self, state):
        # search nearest point index
        dx = [state.x - icx for icx in self.traj_x]
        dy = [state.y - icy for icy in self.traj_y]
        d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
        ind = d.index(min(d))
        dist = 0.0
        Lf = self.k * state.v + self.Lfc

        # search look ahead target point index
        while Lf > dist and (ind + 1) < len(self.traj_x):
            dx = self.traj_x[ind + 1] - self.traj_x[ind]
            dy = self.traj_y[ind + 1] - self.traj_y[ind]
            dist += math.sqrt(dx ** 2 + dy ** 2)
            ind += 1

        # terminating condition
        thresh = 0.1
        target_dist = math.hypot(self.traj_x[-1]-state.x,self.traj_y[-1]-state.y)
        is_close = (target_dist<thresh)
        if ind+1==len(self.traj_x) and is_close:
            self.is_finished = True

        return ind
