"""
Adapted from Atsushi Sakai's PythonRobotics pure pursuit example
"""
import math
import rospy
import numpy as np

class PurePursuitController(object):

    k = 0.6  # look forward gain
    Lfc = 0.4  # look-ahead distance
    K_p = 1  #TODO speed control propotional gain
    K_i = 2  #TODO speed control integral gain
    K_d = -0.1  #TODO speed control derivitive gain
    L = 0.324  # [m] wheel base of vehicle
    threshold = 0.2
    checkpointsTouched = False

    def __init__(self, vehicle_name=''):
        self.traj_x = []
        self.traj_y = []
        self.target = None
        # initialize with 0 velocity
        self.target_velocity = 0.0
        self.last_index = 0
        self.is_finished = False
        
        self.hist_v = []
        self.errors = []
        self.flag_halfway = False

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
            self.errors.append(self.target_velocity-state.v)

            P = self.K_p*(self.errors[-1])

            #N=300
            #if len(self.errors)>N:
            #    I = self.K_i*(sum(self.errors[-N:-1]))
            #else:
            I = self.K_i*(sum(self.errors))

            if len(self.errors)>1:
                D = self.K_d*(self.errors[-1]-self.errors[-2])
            else:
                D = 0

            return P+I+D

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
        #TODO
        
        x_dist = state.x-self.traj_x[-1]
        y_dist = state.y-self.traj_y[-1]
        target_distance = math.sqrt(x_dist**2+y_dist**2)


        self.hist_v.append(state.v)
        rospy.loginfo_throttle(2,"Speed is: {}".format(np.mean(self.hist_v[-30:-1])))
        if ind > len(self.traj_x)/2 and ind < len(self.traj_x)/2+30:
            self.flag_halfway = True
            rospy.loginfo_throttle(1000,"Passed the Halfway point!")

        if target_distance < self.threshold and self.flag_halfway:
            self.is_finished = True
            rospy.loginfo_throttle(1000,"The Average speed was: {}".format(np.mean(self.hist_v)))

        return ind
