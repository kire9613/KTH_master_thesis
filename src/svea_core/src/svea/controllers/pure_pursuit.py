"""
Adapted from Atsushi Sakai's PythonRobotics pure pursuit example
"""
import math

class PurePursuitController(object):

    k = 0.6  # look forward gain
    Lfc = 0.09  # look-ahead distance
    K_p = 1.5  # speed control propotional gain
    K_i = 0.1  # speed control integral gain
    K_d = 0.0  # speed control derivitive gain
    L = 0.324  # [m] wheel base of vehicle

    def __init__(self, vehicle_name=''):
        self.traj_x = []
        self.traj_y = []
        self.target = None
        # initialize with 0 velocity
        self.target_velocity = 0.0
        self.last_index = 0
        self.is_finished = False

	self.error = []
	self.dt = 0.01
        self.index = 0

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
            u = 0
            return u

        else:
            # speed control

            if self.error == []:
                e_prev = 0
            else:
                e_prev = self.error[self.last_index - 1]

            try:
              # swith velocities depending on path is curvy or straight by calculating the angle between car position and future target 2 
              # steps ahead

              tx, ty = self.traj_x[self.index+2], self.traj_y[self.index+2]
              dx = tx - state.x	
              dy = ty - state.y	

              teta = math.atan2(dy, dx)
              delta_teta = abs(state.yaw - teta)

              # condition for angle to consider if path is curvy or straight
              if delta_teta >= 0.5:
                self.target_velocity = 0.5
              else:
                self.target_velocity = 0.6

            except IndexError:
              pass 
             
            # compute the PID errors
            e = self.target_velocity - state.v
            e_sum = sum(self.error) + e*self.dt
            dedt = (e - e_prev) / self.dt

            self.error.append(e)
            self.last_index = self.last_index + 1 

            # compute the control signal according to the PID formula
            u = self.K_p*e + self.K_i*e_sum + self.K_d*dedt

            # saturation
            if u > 0.55:
              u = 0.55
            elif u < -0.55:
              u = -0.55
            
            return u

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

        # terminating condition to stop car
	if self.target != None:
	  tx, ty = self.traj_x[-1], self.traj_y[-1] #condition for distance between state and target position
	  dx = tx - state.x
	  dy = ty - state.y 
	  if math.sqrt(dx**2 + dy**2) < 0.1:
	    self.is_finished = True

        self.index = ind

        return ind
