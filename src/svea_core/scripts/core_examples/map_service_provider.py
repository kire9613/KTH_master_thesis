#!/usr/bin/env python  
import math
import roslib
import rospy
import svea_msgs.msg

from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from sensor_msgs.msg import LaserScan
from svea_msgs.msg import VehicleState 

import pickle

class Map():
    def __init__(self):
        self.current_map = OccupancyGrid()
        self.current_scan = LaserScan()
        self.current_state = VehicleState()

    def map_callback(self, msg):
        self.current_map = msg

    def scan_callback(self, msg):
        self.current_scan = msg

    def state_callback(self, msg):
        self.current_state = msg

    def get_map(self, request):
        return self.current_map
    


if __name__ == '__main__':
    mapper = Map()
    rospy.init_node("map_service_provider")
    rospy.Subscriber('/map',
                     OccupancyGrid,
                     mapper.map_callback)

    rospy.Subscriber('/scan',
                     LaserScan,
                     mapper.scan_callback)

    rospy.Subscriber('/SVEA/state',
                     VehicleState,
                     mapper.state_callback)

    s = rospy.Service('get_map', GetMap, mapper.get_map)
    print("Map_service_provider node is working")
    rospy.spin()
 
