#!/usr/bin/env python

"""
Class for capturing and holding the current lidar scan
"""
import rospy
from svea_msgs.msg import VehicleState

class SyncState:
    def __init__(self):
        self.state = VehicleState()
        self.subs = rospy.Subscriber("/SVEA/state", VehicleState, self.callback)

    def callback(self, data):
        """
        Type :sensor_msgs.msg LaserScan:
        """
        self.state = data