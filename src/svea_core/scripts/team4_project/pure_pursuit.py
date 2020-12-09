"""
Adapted from Atsushi Sakai's PythonRobotics pure pursuit example
"""
import math
import numpy as np
from threading import Lock

traj_lock = Lock()

class PurePursuitController(object):

    k = 0.6       # look forward gain
    Lfc = 0.22    # look-ahead distance
    K_p = 2.0     # Coefficient for P-part of PI
    K_i = 0.0025    # Coefficient for I-part of PI
    T = 0.25         # Anti-windup Coefficient
    L = 0.324     # [m] wheel base of vehicle
    MAX_U = 1.7   # Maximum control signal
    MIN_U = -0.45 # Minimum control signal
    TURN_VEL = 0.4 # Speed when doing a pi/2 turn
    TURN_LIMIT = math.pi/8 # Angle when speed is reduced

    def __init__(self, traj_lock, vehicle_name=''):
        self.traj_x = []
        self.traj_y = []
        self.traj_lock = traj_lock
        self.target = None
        # initialize with 0 velocity
        self.target_velocity = 0.0
        self.last_index = 0
        self.is_finished = False
        self.prev_u = 0
        self.prev_e = 0

        self.current_index = 0

    def compute_control(self, state, target=None):
        steering = self.compute_steering(state, target)
        # target_vel = self.target_velocity
        # if steering > 0.1:
        #     target_vel = self.target_velocity * 0.75
        # elif steering > 0.3:
        #     target_vel = self.target_velocity * 0.5

        target_vel = self.target_velocity
        p = np.array([state.x,state.y])
        t = np.array([self.traj_x[min(self.current_index+10,len(self.traj_x)-1)],self.traj_y[min(self.current_index+10,len(self.traj_y)-1)]])
        tp = t-p
        tp /= np.linalg.norm(tp)
        h = np.array([np.cos(state.yaw),np.sin(state.yaw)])
        delta = np.arccos(np.dot(h,tp))
        if delta > self.TURN_LIMIT and self.target_velocity != 0:
            #print("restrict velocity")
            #target_vel = 0.3
            k = (target_vel - self.TURN_VEL)/(self.TURN_LIMIT - math.pi/2)
            m = target_vel - k*self.TURN_LIMIT
            target_vel = k*delta + m
            target_vel = max(target_vel, self.TURN_VEL)
        velocity = self.compute_velocity(state, target_vel)
        return steering, velocity

    def compute_steering(self, state, target=None):
        if target is None:
            self.find_target(state)


        else:
            # allow manual setting of target
            self.target = target

        # Don't apply any steering if car is standing still
        if self.target_velocity == 0:
            return 0

        tx, ty = self.target
        alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw
        k = self.k
        if state.v < 0:  # back
            alpha = math.pi - alpha
            # Decrease gain when going backwards to increase stability
            k *= 0.5
        Lf = k * state.v + self.Lfc
        delta = math.atan2(2.0 * self.L * math.sin(alpha) / Lf, 1.0)
        return delta

    def compute_velocity(self, state, vel_t):
        # speed control

        #e = self.target_velocity - state.v
        e = vel_t - state.v

        saturated_prev_u = self.prev_u
        if not (self.MIN_U <= self.prev_u <= self.MAX_U):
            saturated_prev_u = min([self.MAX_U, max([self.MIN_U, self.prev_u])])

        u = self.K_p*e + (self.K_i-self.K_p)*self.prev_e + self.T*saturated_prev_u + (1.0-self.T)*self.prev_u
        self.prev_e = e
        self.prev_u = u

        # Manually saturate control signal to make sure that
        # the control limits are the ones used for the anti
        # reset windup
        if not (self.MIN_U <= u <= self.MAX_U):
            u = min([self.MAX_U, max([self.MIN_U, u])])

        #if self.target_velocity == 0:
        if vel_t == 0:
            return 0
        else:
            return u

    def find_target(self, state):
        with traj_lock:
            ind = self._calc_target_index(state)
            self.current_index = ind
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
        Lf = self.k * state.v + math.copysign(self.Lfc, state.v)

        # search look ahead target point index
        while abs(Lf) > dist and (ind + 1) < len(self.traj_x):
            dx = self.traj_x[ind + 1] - self.traj_x[ind]
            dy = self.traj_y[ind + 1] - self.traj_y[ind]
            dist += math.sqrt(dx ** 2 + dy ** 2)
            ind += 1

        return ind
