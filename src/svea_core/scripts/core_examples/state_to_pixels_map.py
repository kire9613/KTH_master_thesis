#!/usr/bin/env python

import rospy
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from svea_msgs.msg import map_pixel_coordinates
from nav_msgs.msg import OccupancyGrid
from svea_msgs.msg import VehicleState


scans_per_rotation = 135#135 #for scan 
###################################
# A node that gets as inputs from subscriptions the Lidar scans and the car's state.
# Publishes the coordinates (in map frame) in pixels of the detected obstacles.
##################################

class Map_to_Pixels():
    def __init__(self):
        self.flag = False
    def set_state(self,state_msg):
        self.state = state_msg
        
    def get_map(self,msg):
        static_map_in = msg
        self.grid_resolution = static_map_in.info.resolution
        self.grid_origin_x = static_map_in.info.origin.position.x
        self.grid_origin_y = static_map_in.info.origin.position.y
        self.flag = True

    def fromStateToPixelCoordinates(self,msg):
        #print("Length of scans: ", len(msg.ranges))
        map_pixel_points = map_pixel_coordinates()
        pub = rospy.Publisher('pixel_coordinates', map_pixel_coordinates, queue_size=10)
        if (self.flag):
            for ang in np.arange(msg.angle_min, msg.angle_max - msg.angle_increment, msg.angle_increment):  
                ang_index=int(np.ceil(scans_per_rotation/2)+round((ang/(msg.angle_increment)))) #int(scans_per_rotation/2)
                if not(np.isnan(msg.ranges[ang_index])):
                    x_scanned = msg.ranges[ang_index] * math.cos(ang)  # Coordinates according to Robot frame
                    y_scanned = msg.ranges[ang_index] * math.sin(ang)

                    x_robot = (x_scanned * math.cos(self.state.yaw) - y_scanned * math.sin(self.state.yaw))
                    y_robot = (y_scanned * math.cos(self.state.yaw) + x_scanned * math.sin(self.state.yaw))
                    

                    map_pixel_points.map_pixel_coordinates_x = int((x_robot + self.state.x - self.grid_origin_x)/self.grid_resolution)
                    map_pixel_points.map_pixel_coordinates_y = int((y_robot + self.state.y - self.grid_origin_y)/self.grid_resolution)

                    pub.publish(map_pixel_points)

if __name__ == '__main__':
    rospy.init_node("state_to_pixels_map")

    map_to_pixels = Map_to_Pixels()
    rospy.Subscriber('/scan',
                     LaserScan,
                     map_to_pixels.fromStateToPixelCoordinates)

    rospy.Subscriber('/map',
                     OccupancyGrid,
                     map_to_pixels.get_map)

    rospy.Subscriber('/state',
                     VehicleState,
                     map_to_pixels.set_state)

    rospy.Subscriber('/SVEA/state',
                     VehicleState,
                     map_to_pixels.set_state)                 

    rospy.spin()


