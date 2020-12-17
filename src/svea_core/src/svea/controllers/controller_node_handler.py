"""
Adapted from Atsushi Sakai's PythonRobotics pure pursuit example
"""
import math
import rospy
import numpy as np
from datetime import datetime
#import matplotlib.pyplot as plt

from nav_msgs.msg import OccupancyGrid
from svea_msgs.srv import ControlMessage


'''
Controller interface for path_following_sveas.
The vehicle class requires a controller with a compute_control function to work.
'''

#TODO: Albin Comment

class NodeController(object):

    k = 0.6  # look forward gain
    Lfc = 0.4  # look-ahead distance
    K_p = 1  #TODO speed control propotional gain
    K_i = 0.1  #TODO speed control integral gain
    K_d = 0  #TODO speed control derivitive gain
    L = 0.324  # [m] wheel base of vehicle
    threshold = 0.2
    checkpointsTouched = False

    def __init__(self, vehicle_name=''): 
        rospy.wait_for_service('/GetSpeedSteering')
        self.control_client = rospy.ServiceProxy('/GetSpeedSteering', ControlMessage, persistent=True)

        self.target = None
        self.is_finished = False

    def compute_control(self, state, target=None):
        try:
            control = self.control_client()
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            control.speed = 0
            control.steering = 0

        steering = control.steering
        velocity = control.speed
        return steering, velocity
