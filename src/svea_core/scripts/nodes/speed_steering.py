#!/usr/bin/env python

import rospy
import numpy as np
import math
from svea_msgs.msg import VehicleState
from svea_msgs.msg import CoordinateArray
from svea_msgs.srv import ControlMessage
from svea_msgs.srv import ControlMessageResponse
from svea_msgs.msg import next_traj
from std_msgs.msg import Bool, Float32

'''
Node with PID controller with pure pursuit. 
To change the control in this node. Add other functions that are called in getControl()
'''

class Controller:
    def __init__(self):
        self.state = VehicleState()
        self.trajectory = next_traj()

        #create publishers
        self.publisher = rospy.Publisher('/ControlMessage', ControlMessageResponse, queue_size=1)
        self.target_publisher = rospy.Publisher('/TargetPoint', next_traj, queue_size=1)
        self.control_publisher = rospy.Publisher('/Debugging/ControlSignal', Float32, queue_size=1)
        self.error_publisher = rospy.Publisher('/Debugging/ErrorSignal', Float32, queue_size=1)
        self.error_sum_publisher = rospy.Publisher('/Debugging/ErrorSumSignal', Float32, queue_size=1)

        #Controller vars for PID
        self.errors = []

        #Set target_velocity
        self.target_velocity = 1.0 #1.0

        #Parameters for PID controller
        self.K_p = 1
        self.K_i = 0.1
        self.K_d = 0.01
        
        #Parameters for steering
        self.Lfc = 0.4
        self.L = 0.324
        self.k = 0.8

        self.last_time = rospy.get_time()

    
    def set_state(self, state_msg):
        #Update state to new location and update control signal
        self.state = state_msg
        message = self.getResponse()
        self.publisher.publish(message)

    def update_trajectory(self, trajectory_msg):
        #Update trajectory to follow
        self.traj_x = trajectory_msg.x_coordinates
        self.traj_y = trajectory_msg.y_coordinates
        
    def using_astar(self, msg):
        #Set different target speeds depending on if using the standard path or calculated path
        if msg.data == True:
            self.target_velocity = 0.6
            self.k = 0.005
        else:
            self.target_velocity = 1.0
            self.k = 0.8
    
    
    def getPIDSpeed(self):
        #Calculate the speed for the PID controller

        #Only use speeds over a certain threshold to avoid windup
        if not self.state.v < 0.1:
            self.errors.append(self.target_velocity-self.state.v)
        else:
            self.errors.append(0)

        #Get P part
        P = self.K_p*(self.errors[-1])

        #Get I part
        if len(self.errors)>1:
            I = self.K_i*(sum(self.errors))
        else:
            I = 0

        #Get dt since last time
        newtime = rospy.get_time()
        dt = newtime - self.last_time
        self.last_time = newtime

        #Get D part
        if len(self.errors)>1:
            D = self.K_d*(self.errors[-1]-self.errors[-2])/dt
        else:
            D = 0
        
        #Only use the latest 100 samples
        if len(self.errors)>100:
            self.errors = self.errors[-100:-1]
        
        #If speed is lower than threshold, give constant input
        if np.abs(self.state.v) < 0.1:
            PID = 0.4
        else:
            PID = P+I+D

        #Publish message for plotting
        self.control_publisher.publish(PID)
        self.error_publisher.publish(self.errors[-1])
        self.error_sum_publisher.publish(sum(self.errors)/len(self.errors))

        #Set thresholds for output signal
        maxinput = 4
        return max(0, min(maxinput, PID))

    def getSteering(self):

        #Get steering to target point
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
        # Search nearest point index
        
        dx = [state.x - icx for icx in self.traj_x]
        dy = [state.y - icy for icy in self.traj_y]
        d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
        ind = d.index(min(d))
        dist = 0.0
        Lf = self.k * state.v + self.Lfc
        # Search look ahead target point index
        while Lf > dist and (ind + 1) < len(self.traj_x):
            dx = self.traj_x[ind + 1] - self.traj_x[ind]
            dy = self.traj_y[ind + 1] - self.traj_y[ind]
            dist += math.sqrt(dx ** 2 + dy ** 2)
            ind += 1
        return ind

    def getControl(self):
        #function to modularize the steering and speed part to make them easily interchangeable
        speed = self.getPIDSpeed()
        steering = self.getSteering()
        return speed, steering

    def getResponse(self):
        #Calulcate speed and steering
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
                     
    rospy.Subscriber('/using_astar',
                     Bool,
                     controller.using_astar)

    rospy.loginfo('Speed and Steering Node Initialized')

    rospy.spin()
