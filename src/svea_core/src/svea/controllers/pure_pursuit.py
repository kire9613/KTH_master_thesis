"""
Adapted from Atsushi Sakai's PythonRobotics pure pursuit example
"""
import math, rospy
from pprint import pprint

class PurePursuitController(object):

    k = 0.6  # look forward gain
    Lfc = 0.4  # look-ahead distance
    K_p = 0.1 #TODO speed control propotional gain
    K_i = 1.0  #TODO speed control integral gain
    K_d = 0.0  #TODO speed control derivitive gain
    P = 0 # initilize P value (PID)
    I = 0 # intialize I value (PID)
    L = 0.324  # [m] wheel base of vehicle
    max_velocity = 1
    emergency_distance = 0.1 # [m] minimum distance until emergency_stop activated
    emg_angle_range = 0.785 # angle range 

    def __init__(self, vehicle_name=''):
        self.traj_x = []
        self.traj_y = []
        self.target = None
        # initialize with 0 velocity
        self.target_velocity = 0.0
        self.last_index = 0
        self.is_finished = False
        self.emg_stop = False
        self.last_time = 0.0
        self.print_time = 0.0

    def compute_control(self, state, target=None):
        steering = self.compute_steering(state, target)
        velocity = self.compute_velocity(state)
        return steering, velocity

    def compute_steering(self, state, target=None):
        if target is None:
            dist = self.find_target(state)
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
        if self.is_finished or self.emg_stop:
            # stop moving if trajectory done and reset controller.
            self.I = 0.0
            self.P = 0.0
            return 0.0
        else:
            # speed control
            e = self.target_velocity - state.v
            dt =  state.time_stamp.to_sec() - self.last_time
            self.last_time = state.time_stamp.to_sec()
            self.P = self.K_p*e
            self.I = self.I + self.K_i*e*dt
            actuated_velocity = self.target_velocity + self.P + self.I
            if actuated_velocity > self.max_velocity:
                self.I = 0
                return self.max_velocity
        return  actuated_velocity


    def find_target(self, state):
        ind, dist = self._calc_target_index(state)
        tx = self.traj_x[ind]
        ty = self.traj_y[ind]
        self.target = (tx, ty)
        return dist

    def emergency_stop(self, laserScan):
        # Compute index ranges for emergency stop scan
        min_index =  int(round((-self.emg_angle_range - laserScan.angle_min)/laserScan.angle_increment))
        max_index =  int(round((self.emg_angle_range - laserScan.angle_min)/laserScan.angle_increment))
        closestDistance = min(laserScan.ranges[min_index:max_index])
        # Compute index of closest object
        index = laserScan.ranges.index(closestDistance)
        # Compute angle of closest object
        angle = laserScan.angle_min + index*laserScan.angle_increment

        #self.print_every_second("dist", closestDistance)

        # Check if object is too close - Emergency stop
        if closestDistance < self.emergency_distance and not self.emg_stop:
            self.emg_stop = True
            rospy.loginfo("Controller: Too close, stopping! Object at angle: %d degrees", angle*180/math.pi)
            if angle > 0:
                rospy.loginfo("Object is slightly to the left of the vehicle:")
            else:
                rospy.loginfo("Object is slightly to the right of the vehicle:")
        return

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
        if dist < 0.1:
            self.is_finished = True

        return ind, dist
    
    def print_every_second(self, message, data):
                # print once per second
        if (self.last_time - self.print_time)  > 1:
            print(message, data)
            self.print_time = self.last_time

