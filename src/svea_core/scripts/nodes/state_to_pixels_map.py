#!/usr/bin/env python

import rospy
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from svea_msgs.msg import map_pixel_coordinates
from nav_msgs.msg import OccupancyGrid
from svea_msgs.msg import VehicleState


scans_per_rotation = 135 #135 for scan in sim, 1081 in real
#scans_per_rotation = 1081 #135 for scan in sim, 1081 in real

###################################
# A node that gets as inputs subscriptions from the Lidar scans, the map and the car's state.
# Publishes the coordinates (in map frame) in list of pixels of the detected obstacles.
##################################

class Map_to_Pixels():
    def __init__(self):
        self.flag = False
        self.min_angle = np.deg2rad(-45)
        self.max_angle = np.deg2rad(45)
        self.xs = []
        self.ys = []
        
    def set_state(self,state_msg):
        # Gets the state of the SVEA car
        self.state = state_msg
        
    def get_map(self,msg):
        # Gets the map from the map server
        static_map_in = msg
        self.grid_resolution = static_map_in.info.resolution
        self.grid_origin_x = static_map_in.info.origin.position.x
        self.grid_origin_y = static_map_in.info.origin.position.y
        self.flag = True

    def fromStateToPixelCoordinates(self,msg):
        # Gets Lidar's scans and transform them into map pixels
        map_pixel_points = map_pixel_coordinates()
        pub = rospy.Publisher('pixel_coordinates', map_pixel_coordinates, queue_size=1)
        zero_angle_ind = np.ceil(scans_per_rotation/2)
        coefficient = msg.angle_increment
        angle_increment = msg.angle_increment * 2
        
        if (self.flag):
            self.xs = []
            self.ys = []
            for ang in np.arange(self.min_angle, self.max_angle, angle_increment):  
                ang_index=int(zero_angle_ind+round((ang/coefficient)))
                if not(np.isnan(msg.ranges[ang_index])) and msg.ranges[ang_index]<10:

                    # Transform Lidar Scans into coordinates according to Robot frame
                    x_scanned = msg.ranges[ang_index] * math.cos(ang)  
                    y_scanned = msg.ranges[ang_index] * math.sin(ang)

                    # Transform Robot Frame x and y into coordinates according to World frame
                    x_robot = (x_scanned * math.cos(self.state.yaw) - y_scanned * math.sin(self.state.yaw))
                    y_robot = (y_scanned * math.cos(self.state.yaw) + x_scanned * math.sin(self.state.yaw))

                    # Transform World Frame x and y into pixel coordinates according to Map frame
                    map_pixel_coordinates_x = int((x_robot + self.state.x - self.grid_origin_x)/self.grid_resolution)
                    map_pixel_coordinates_y = int((y_robot + self.state.y - self.grid_origin_y)/self.grid_resolution)
                    
                    # Append values to the lists that are going to be published
                    self.xs.append(map_pixel_coordinates_x)
                    self.ys.append(map_pixel_coordinates_y)

            # Add the lists into the same message and publish the message
            map_pixel_points.map_pixel_coordinates_x = self.xs
            map_pixel_points.map_pixel_coordinates_y = self.ys
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


