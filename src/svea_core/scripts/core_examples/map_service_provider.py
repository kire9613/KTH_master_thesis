#!/usr/bin/env python  
import math
import roslib
import rospy
import svea_msgs.msg

from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
import pickle


class Map():
    def __init__(self):
        self.current_map = OccupancyGrid()

    def callback(self, msg):
        self.current_map = msg

    def get_map(self, request):
        return self.current_map


if __name__ == '__main__':
    mapper = Map()
    rospy.init_node("map_service_provider")
    rospy.Subscriber('/map',
                     OccupancyGrid,
                     mapper.callback)
    s = rospy.Service('get_map', GetMap, mapper.get_map)
    print("Map_service_provider node is working")
    rospy.spin()
 
