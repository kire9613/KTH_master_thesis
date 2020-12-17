#!/usr/bin/env python

import rospy
import numpy as np
import math
from svea_msgs.srv import ControlMessage, ControlMessageResponse
from svea_msgs.msg import emergency_break, slow_down, VehicleState

'''
Control filter that only lets through the control signal if emergency break is not enabled.
If emergency break is enabled the car will reverse
It will also go at a slower pace if the slow_down flag is raised. In the current implementation that means stopping.
Potential upgrades would be to add steering around obstacles instead of just stopping
Control is gotten from the /GetSpeedSteering service.
'''
class ControlFilter:

    def __init__(self):
        #Init variables
        self.control = ControlMessageResponse()
        self.emergency_break = emergency_break()
        self.slow_down = slow_down()
        self.emergency_break_triggered = False
        self.emergency_break_start_time = rospy.get_time()

    def update_control(self, control_msg):
        #Update control to current value
        self.control = control_msg

    def set_flag(self, flag):
        #Update emergency_break to current value
        self.emergency_break = flag
    
    def slow_down_flag(self, flag):
        #Update slow_down to current value
        self.slow_down = flag


    def get_control(self, request):
        '''
        Return control as a service in the GetSpeedSteering service
        The priority is as follows
        1. If reversing from emergency break, do so
        2. Stop if emergency break is on
        3. Slow down if described to do so
        4. Drive as normal along the path
        '''
        emergency_break_backing_time = 1.8
        current_time = rospy.get_time()
        time_difference = current_time-self.emergency_break_start_time
        if  time_difference > emergency_break_backing_time:
            self.emergency_break_triggered = False
        
        if self.emergency_break_triggered:
            #print("GO BACK")
            response = ControlMessageResponse()
            response.speed = -0.8
            response.steering = 0
        elif self.emergency_break.emergency_break:
            #print("STOP")
            self.emergency_break_triggered = True
            self.emergency_break_start_time = rospy.get_time()  
            response = ControlMessageResponse()
            response.speed = 0
            response.steering = 0
        elif self.slow_down.slow_down:
            #print("SLOW DOWN")
            response = ControlMessageResponse()
            response.speed = 0 #0.2
            response.steering = self.control.steering
        else:
            #print("CONTINUE")
            #response = ControlMessageResponse()
            response = self.control
            if time_difference < emergency_break_backing_time+ 0.5:
                response.speed /= 2
            
        return response




if __name__ == '__main__':
    rospy.init_node("filter")
    
    controlFilter = ControlFilter()
    rospy.Subscriber('/ControlMessage',
                     ControlMessageResponse,
                     controlFilter.update_control)

    rospy.Subscriber('/emergency_break',
                     emergency_break,
                     controlFilter.set_flag)

    rospy.Subscriber('/slow_down',
                     slow_down,
                     controlFilter.slow_down_flag)

    s = rospy.Service('/GetSpeedSteering', ControlMessage, controlFilter.get_control)
    rospy.loginfo('Filter Node Initialized')

    rospy.spin()
