#!/usr/bin/env python

import rospy
import numpy as np
import math
from svea_msgs.msg import VehicleState
from svea_msgs.msg import CoordinateArray
from svea_msgs.srv import ControlMessage
from svea_msgs.srv import ControlMessageResponse
from svea_msgs.msg import next_traj
from std_msgs.msg import Bool

'''
Node with PID controller with pure pursuit. 
To change the control in this node. Add other functions that are called in getControl()
'''

class Controller:
    def __init__(self):
        self.state = VehicleState()
        self.trajectory = next_traj()
        # self.trajectory_planner =
        xs = [-2.00, -6.58]
        ys = [3.60, -2.97]
        traj_x = []
        traj_y = []
        for i in range(len(xs)-1):
            traj_x.append(np.linspace(xs[i], xs[i+1]).tolist())
            traj_y.append(np.linspace(ys[i], ys[i+1]).tolist())
        traj_x.append(np.linspace(xs[-1], xs[0]).tolist())
        traj_y.append(np.linspace(ys[-1], ys[0]).tolist())

        #Makes the nested lists into a one dimensional array.
        self.traj_x = [val for sublist in traj_x for val in sublist]
        self.traj_y = [val for sublist in traj_y for val in sublist]
        self.publisher = rospy.Publisher('/ControlMessage', ControlMessageResponse, queue_size=2)
        self.target_publisher = rospy.Publisher('/TargetPoint', next_traj, queue_size=2)
        #Controller vars for PID
        self.errors = []
        self.errors_for_I = []
        self.target_velocity = 1.0 #1.0
        self.K_p = 1.0
        self.K_i = 0.1
        self.K_d = 0
        self.last_time = rospy.get_time()
        self.Lfc = 0.4
        self.L = 0.324
        self.k = 0.3

    
    def set_state(self, state_msg):
        self.state = state_msg
        message = self.getResponse()
        self.publisher.publish(message)

    def update_trajectory(self, trajectory_msg):
        # self.trajectory = trajectory_msg
        self.traj_x = trajectory_msg.x_coordinates
        self.traj_y = trajectory_msg.y_coordinates
        
    '''def update_lookahead(self, msg):
        if msg.data == True:
            self.k = 0.1
            self.errors_for_I = []
        else:
            self.k = 0.6
    '''
    
    def getPIDSpeed(self):
        test = False
        if test:
            return 1
        
        self.errors.append(self.target_velocity-self.state.v)

        P = self.K_p*(self.errors[-1])

        if abs(self.errors[-1]) < self.target_velocity*0.4:
            self.errors_for_I.append(self.errors[-1])
            I = self.K_i*(sum(self.errors_for_I))
        else:
            self.errors_for_I = []
            I = 0.4

        newtime = rospy.get_time()
        dt = newtime - self.last_time
        self.last_time = newtime
        if len(self.errors)>1:
            D = self.K_d*(self.errors[-1]-self.errors[-2])/dt
        else:
            D = 0
        
        PID = P+I+D
        maxinput = 4
        # print(max(0, min(maxinput, PID)))
        return max(0, min(maxinput, PID))


    def getSteering(self):
        test = False
        if test:
            return 0


        ind = self._calc_target_index(self.state)
        tx = self.traj_x[ind]
        ty = self.traj_y[ind]
        target = next_traj()
        target.x_coordinates = [tx]
        target.y_coordinates = [ty]
        self.target_publisher.publish(target)

        alpha = math.atan2(ty - self.state.y, tx - self.state.x) - self.state.yaw
        if self.state.v < 0:  # back
            alpha = math.pi - alpha
        Lf = self.k * self.state.v + self.Lfc
        delta = math.atan2(2.0 * self.L * math.sin(alpha) / Lf, 1.0)
        return delta


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
        return ind

    def getControl(self):
        speed = self.getPIDSpeed()
        steering = self.getSteering()
        return speed, steering

    def getResponse(self):
        speed, steering = self.getControl()
        response = ControlMessageResponse()
        response.speed = speed
        response.steering = steering
        return response




if __name__ == '__main__':
    rospy.init_node("speed_steering")
    
    controller = Controller()
    rospy.Subscriber('/TrajMessage',
                     next_traj,
                     controller.update_trajectory)

    rospy.Subscriber('/state',
                     VehicleState,
                     controller.set_state)

    rospy.Subscriber('/SVEA/state',
                     VehicleState,
                     controller.set_state)
                     
    #rospy.Subscriber('/using_astar',
    #                 Bool,
    #                 controller.update_lookahead)

    rospy.loginfo('Speed and Steering Node Initialized')

    rospy.spin()
