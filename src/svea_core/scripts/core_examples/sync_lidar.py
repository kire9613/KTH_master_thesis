#!/usr/bin/env python

"""
Class for capturing and holding the current lidar scan
"""
import rospy
from sensor_msgs.msg import LaserScan

class SyncScan:
    def __init__(self):
        self.scan = LaserScan()
        self.subs = rospy.Subscriber("/scan", LaserScan, self.callback)

    def callback(self, data):
        """
        Type :sensor_msgs.msg LaserScan:
        """
        self.scan = data
