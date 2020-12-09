"""
Adapted from Atsushi Sakai's PythonRobotics pure pursuit example
"""
import math
import numpy as np

class PurePursuitController(object):

    k = 0.6  # look forward gain
    Lfc = 0.2  # look-ahead distance
    K_p = 1.0  # speed control propotional gain
    K_i = 0.2  # speed control integral gain
    K_d = 0.0  # speed control derivitive gain
    L = 0.324  # [m] wheel base of vehicle
    e_sum = 0
    e_tmin1 = 0
    K_str = 0.5
    st_cor = 0

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

        st_err,st_dir = self.short_dist(state)

        if st_dir > 0:
            self.st_cor = self.K_str*st_err
        elif st_dir < 0:
            self.st_cor = -self.K_str*st_err
        
        print("st_cor",self.st_cor)

        delta = math.atan2(2.0 * self.L * math.sin(alpha) / Lf, 1.0) + self.st_cor
        return delta

    def compute_velocity(self, state):
        if self.is_finished:
            # stop moning if trajectory done
            return 0.0
        else:
            # speed control
            e = self.target_velocity - state.v
            self.e_sum += e*0.1
            # if e*self.e_tmin1<0: # anti-windup, reset when crossing zero
            #     self.e_sum=0
            P = e*self.K_p
            I = self.e_sum*self.K_i
            D = (e-self.e_tmin1)*self.K_d/0.01 # dt = 0.01, first order approx
            self.e_tmin1 = e
            u = P
            speed_lim = 2.50
            u = np.clip(u,-speed_lim+self.target_velocity,speed_lim-self.target_velocity)
            return self.target_velocity + u

    def find_target(self, state):
        _,tar = self._calc_target_index(state)
        tx = self.traj_x[tar]
        ty = self.traj_y[tar]
        self.target = (tx, ty)

    def short_dist(self, state):
        ind,tar = self._calc_target_index(state)
        cur_x = self.traj_x[ind]
        cur_y = self.traj_y[ind]
        tx = self.traj_x[tar]
        ty = self.traj_y[tar]

        dist = math.sqrt((cur_y-state.y)**2 + (cur_x-state.x)**2)
        st_dir = state.x*(ty-cur_y) -  state.y*(tx - cur_x)

        return dist,st_dir

    def _calc_target_index(self, state):
        # search nearest point index
        dx = [state.x - icx for icx in self.traj_x]
        dy = [state.y - icy for icy in self.traj_y]
        d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
        ind = d.index(min(d))
        tar = ind
        dist = 0.0
        Lf = self.k * state.v + self.Lfc

        # search look ahead target point index
        while Lf > dist and (tar + 1) < len(self.traj_x):
            dx = self.traj_x[tar + 1] - self.traj_x[tar]
            dy = self.traj_y[tar + 1] - self.traj_y[tar]
            dist += math.sqrt(dx ** 2 + dy ** 2)
            tar += 1

        # terminating condition
        thresh = 0.3
        target_dist = math.hypot(self.traj_x[-1]-state.x,self.traj_y[-1]-state.y)
        is_close = (target_dist<thresh)
        if tar+1==len(self.traj_x) and is_close:
            pass
            # self.is_finished = True

        return ind,tar
