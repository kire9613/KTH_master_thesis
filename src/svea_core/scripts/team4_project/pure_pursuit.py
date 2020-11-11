"""
Adapted from Atsushi Sakai's PythonRobotics pure pursuit example
"""
import math

class PurePursuitController(object):

    k = 0.6       # look forward gain
    Lfc = 0.4     # look-ahead distance
    K_p = 0.6     # Coefficient for P-part of PI
    K_i = 0.03    # Coefficient for I-part of PI
    T = 5         # Anti-windup Coefficient
    L = 0.324     # [m] wheel base of vehicle
    MAX_U = 1.7   # Maximum control signal
    MIN_U = -0.45 # Minimum control signal

    def __init__(self, vehicle_name=''):
        self.traj_x = []
        self.traj_y = []
        self.target = None
        # initialize with 0 velocity
        self.target_velocity = 0.0
        self.last_index = 0
        self.is_finished = False
        self.prev_u = 0
        self.prev_e = 0

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
        k = self.k
        if state.v < 0:  # back
            alpha = math.pi - alpha
            # Decrease gain when going backwards to increase stability
            k *= 0.5
        Lf = k * state.v + self.Lfc
        delta = math.atan2(2.0 * self.L * math.sin(alpha) / Lf, 1.0)
        return delta

    def compute_velocity(self, state):
        if self.is_finished or self.target_velocity == 0:
            # stop moning if trajectory done
            return 0.0
        else:
            # speed control
            #TODO

            e = self.target_velocity - state.v

            saturated_prev_u = self.prev_u
            if not (self.MIN_U <= self.prev_u <= self.MAX_U):
                saturated_prev_u = min([self.MAX_U, max([self.MIN_U, self.prev_u])])
            
            u = self.K_p*e + (self.K_i-self.K_p)*self.prev_e + self.T*saturated_prev_u + (1-self.T)*self.prev_u
            self.prev_e = e
            self.prev_u = u

            # Manually saturate control signal to make sure that
            # the control limits are the ones used for the anti
            # reset windup
            if not (self.MIN_U <= u <= self.MAX_U):
                u = min([self.MAX_U, max([self.MIN_U, u])])

            return u #self.target_velocity

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
        Lf = self.k * state.v + math.copysign(self.Lfc, state.v)

        # search look ahead target point index
        while abs(Lf) > dist and (ind + 1) < len(self.traj_x):
            dx = self.traj_x[ind + 1] - self.traj_x[ind]
            dy = self.traj_y[ind + 1] - self.traj_y[ind]
            dist += math.sqrt(dx ** 2 + dy ** 2)
            ind += 1

        # terminating condition
        #TODO
        if math.sqrt((state.x-self.traj_x[-1]) ** 2 + (state.y-self.traj_y[-1]) ** 2) < 0.05:
            self.is_finished = True
        else:
            self.is_finished = False

        return ind
