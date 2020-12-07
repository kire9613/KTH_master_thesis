#!/usr/bin/env python

import rospy
import numpy as np
import math
from svea_msgs.srv import ControlMessage
from svea_msgs.srv import ControlMessageResponse
from svea_msgs.msg import emergency_break
from svea_msgs.msg import slow_down

'''
Control filter that only lets through the control signal if emergency break is not enabled.
Potential upgrades would be to add steering around obstacles instead of just stopping
Control is gotten from the /GetSpeedSteering service.
'''
class ControlFilter:



    def __init__(self):
        self.control = ControlMessageResponse()
        self.emergency_break = emergency_break()
        self.slow_down = slow_down()

    def update_control(self, control_msg):
        self.control = control_msg

    def set_flag(self, flag):
        self.emergency_break = flag
    
    def slow_down_flag(self, flag):
        self.slow_down = flag


    def get_control(self, request):
        if self.emergency_break.emergency_break:
            #print("STOP")
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
