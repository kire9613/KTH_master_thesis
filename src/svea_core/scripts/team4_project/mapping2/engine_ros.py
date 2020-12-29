#!/usr/bin/env python3

# Python standard library
import copy
import numpy as np
import os.path
import re
from copy import deepcopy
from math import sqrt

# ROS
import rospy
import message_filters

# ROS messages
from geometry_msgs.msg import PoseStamped, PolygonStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan, PointCloud
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from map_msgs.msg import OccupancyGridUpdate
from svea_msgs.msg import VehicleState
from std_msgs.msg import Bool

from grid_map import GridMap, quaternion_from_euler
from mapping import Mapping
import matplotlib.pyplot as plt


class EngineROS:
    """
    ROS node handeling mapping
    Manages map and adding updates from lidar scans.
    Publishes latest map on topis /custom_map
    """

    def __init__(self, map_frame_id, map_resolution, map_width, map_height,
                 map_origin_x, map_origin_y, map_origin_yaw, inflate_radius,
                 unknown_space, free_space, c_space, occupied_space, optional=None):

        rospy.loginfo("Init EngineROS")
        rospy.init_node('Mapper')

        # Map properties
        self.width = map_width
        self.height = map_height
        self.res = map_resolution
        self.xo = map_origin_x
        self.yo = map_origin_y

        self.polygon_space = 120
        self.radius = inflate_radius

        # Class handeling updating of map
        self.__mapping = Mapping(self.polygon_space, unknown_space, free_space, c_space,
                                 occupied_space, inflate_radius, optional)

        self.default_map = rospy.wait_for_message('/map', OccupancyGrid)
        self.__map = rospy.wait_for_message('/map', OccupancyGrid)
        m = rospy.wait_for_message('/map', OccupancyGrid)

        # Init map publishers
        # Normal map
        self.__map_pub = rospy.Publisher('/custom_map', OccupancyGrid, queue_size=1,
                                         latch=True)
        # Inflated map
        self.__infl_pub = rospy.Publisher('/infl_map', OccupancyGrid, queue_size=1,
                                         latch=True)

        # Map updates publishers publishing only updated parts of the map
        self.__map_updates_pub = rospy.Publisher("custom_map_updates",
                                                 OccupancyGridUpdate,
                                                 queue_size=10)
        self.__map_inflated_pub  = rospy.Publisher("infl_map_updates",
                                                 OccupancyGridUpdate,
                                                 queue_size=10)

        # Publishing node setup status
        self.started_pub = rospy.Publisher('/node_started/mapping', Bool, latch=True, queue_size=5)

        # Subscribe to polygons typic
        self.pi_ok = False # Indicators for receiving polygons
        self.po_ok = False
        self.added_polygons = False
        self.stay_inside_polygon = None # Polygon variables
        self.keep_outside_polygon = None
        self.polygon_index = []
        self.__stay_in_sub = rospy.Subscriber('/track/stay_in', PolygonStamped, self.track_callback_in)
        self.__keep_out_sub = rospy.Subscriber('/track/keep_out', PolygonStamped, self.track_callback_out)

        #Init subscribers and variables for poses and scans
        self.pose = None
        self.scan = None
        self.received_scan = False
        self.received_pose = False

        self.setup_ok = False

        self.__map_pub.publish(self.__map) # publish map on topic /custom_map
        self.__infl_pub.publish(self.__map)

        if (os.path.isfile(os.path.dirname(os.path.realpath(__file__)) + "/default_map.txt")):
            # read saved default map if exists
            self.set_map_from_file()
        else:
            # create new default map and save
            self.add_polygons()
            while not self.added_polygons:
                rospy.sleep(1)
            self.__map_pub.publish(self.__map)
            self.__infl_pub.publish(self.__map)
            rospy.sleep(5)
            self.write_map_to_file()

        self.__map = np.array(self.__map.data).reshape(self.height,self.width)
        self.__imap = deepcopy(self.__map)

        # wait for initial position before start updating maps
        rospy.loginfo('Mapping is waiting for initial position...')
        rospy.wait_for_message('/initialpose', PoseWithCovarianceStamped)
        rospy.sleep(1)

        # set mapping redy
        self.started_pub.publish(True)
        rospy.loginfo("Start mapping main loop")

        # Loop adding updates to map from lidar
        while not rospy.is_shutdown():
            scan = rospy.wait_for_message("/scan", LaserScan)
            pose = rospy.wait_for_message("/state", VehicleState)
            pose = self.to_pose_stamped(pose)
            self.check_for_updates(pose, scan)

    def write_map_to_file(self):
        """ Saves map at initialization to file """

        current_map = self.__map
        f = open(os.path.dirname(os.path.realpath(__file__)) + "/default_map.txt", "a")
        content = str(current_map)
        f.write(content)
        f.close()
        rospy.loginfo("Map written to file.")
        self.setup_ok = True

    def set_map_from_file(self):
        """ Set map at initialization from file """

        rospy.loginfo("Read map from file...")
        map = OccupancyGrid()
        f = open(os.path.dirname(os.path.realpath(__file__)) + "/default_map.txt", "r")
        Lines = f.readlines()
        map.header.seq = 0
        map.header.frame_id = "map"
        map.info.resolution = float(re.sub("[A-Za-z\:]", "", Lines[10]).strip())
        map.info.width = float(re.sub("[A-Za-z\:]", "", Lines[11]).strip())
        map.info.height = float(re.sub("[A-Za-z\:]", "", Lines[12]).strip())
        map.info.origin.position.x = float(re.sub("[A-Za-z\:]", "", Lines[15]).strip())
        map.info.origin.position.y = float(re.sub("[A-Za-z\:]", "", Lines[16]).strip())
        map.info.origin.position.z = float(re.sub("[A-Za-z\:]", "", Lines[17]).strip())
        map.info.origin.orientation.x = float(re.sub("[A-Za-z\:]", "", Lines[19]).strip())
        map.info.origin.orientation.y = float(re.sub("[A-Za-z\:]", "", Lines[20]).strip())
        map.info.origin.orientation.z = float(re.sub("[A-Za-z\:]", "", Lines[21]).strip())
        map.info.origin.orientation.w = float(re.sub("[A-Za-z\:]", "", Lines[22]).strip())
        d = re.sub("[A-Za-z]", "", Lines[23]).strip()
        d = d.replace(":", "").replace("[", "").replace("]", "").replace(" ", "")
        d = d.split(",")
        for i in range(0,len(d)):
            d[i] = int(d[i])
        map.data = d
        self.__map = map

        self.__map_pub.publish(self.__map)
        self.__infl_pub.publish(self.__map)
        rospy.sleep(5)
        self.setup_ok = True
        rospy.loginfo("Read map from file done.")

    def track_callback_out(self, polygon):
        """ Receiving from inner polygon topic """

        self.keep_outside_polygon = polygon
        rospy.sleep(3)
        self.pi_ok = True

    def track_callback_in(self, polygon):
        """ Receiving from outer polygon topic """

        self.stay_inside_polygon = polygon
        rospy.sleep(3)
        self.po_ok = True

    def add_polygons(self):
        """
        Adding polygons (virtual walls) to map as obstacles.
        """

        while self.pi_ok != True and self.po_ok != True:
            rospy.sleep(0.1)

        # Create lines of obstacle dots along polygon edges
        obs_l_x = []
        obs_l_y = []

        for i in range(1,len(self.keep_outside_polygon.polygon.points)):
            p1 = self.keep_outside_polygon.polygon.points[i-1]
            p2 = self.keep_outside_polygon.polygon.points[i]
            dx = abs(p1.x-p2.x)
            dy = abs(p1.y-p2.y)
            if dx > dy:
                lx = np.linspace(float(p1.x), float(p2.x), num=int(dx/0.05)).tolist()
                ly = np.linspace(float(p1.y), float(p2.y), num=len(lx)).tolist()
                obs_l_x = obs_l_x + lx
                obs_l_y = obs_l_y + ly
            else:
                ly = np.linspace(float(p1.y), float(p2.y), num=int(dy/0.05)).tolist()
                lx = np.linspace(float(p1.x), float(p2.x), num=len(ly)).tolist()
                obs_l_y = obs_l_y + ly
                obs_l_x = obs_l_x + lx

        ps = self.keep_outside_polygon.polygon.points[0]
        pe = self.keep_outside_polygon.polygon.points[-1]
        dx = abs(ps.x-pe.x)
        dy = abs(ps.y-pe.y)
        if dx > dy:
            lx = np.linspace(float(ps.x), float(pe.x), num=int(dx/0.05)).tolist()
            ly = np.linspace(float(ps.y), float(pe.y), num=len(lx)).tolist()
            obs_l_x = obs_l_x + lx
            obs_l_y = obs_l_y + ly
        else:
            ly = np.linspace(float(ps.y), float(pe.y), num=int(dy/0.05)).tolist()
            lx = np.linspace(float(ps.x), float(pe.x), num=len(ly)).tolist()
            obs_l_y = obs_l_y + ly
            obs_l_x = obs_l_x + lx


        for i in range(1,len(self.stay_inside_polygon.polygon.points)):
            p1 = self.stay_inside_polygon.polygon.points[i-1]
            p2 = self.stay_inside_polygon.polygon.points[i]
            dx = abs(p1.x-p2.x)
            dy = abs(p1.y-p2.y)
            if dx > dy:
                lx = np.linspace(float(p1.x), float(p2.x), num=int(dx/0.05)).tolist()
                ly = np.linspace(float(p1.y), float(p2.y), num=len(lx)).tolist()
                obs_l_x = obs_l_x + lx
                obs_l_y = obs_l_y + ly
            else:
                ly = np.linspace(float(p1.y), float(p2.y), num=int(dy/0.05)).tolist()
                lx = np.linspace(float(p1.x), float(p2.x), num=len(ly)).tolist()
                obs_l_y = obs_l_y + ly
                obs_l_x = obs_l_x + lx

        ps = self.stay_inside_polygon.polygon.points[0]
        pe = self.stay_inside_polygon.polygon.points[-1]
        dx = abs(ps.x-pe.x)
        dy = abs(ps.y-pe.y)
        if dx > dy:
            lx = np.linspace(float(ps.x), float(pe.x), num=int(dx/0.05)).tolist()
            ly = np.linspace(float(ps.y), float(pe.y), num=len(lx)).tolist()
            obs_l_x = obs_l_x + lx
            obs_l_y = obs_l_y + ly
        else:
            ly = np.linspace(float(ps.y), float(pe.y), num=int(dy/0.05)).tolist()
            lx = np.linspace(float(ps.x), float(pe.x), num=len(ly)).tolist()
            obs_l_y = obs_l_y + ly
            obs_l_x = obs_l_x + lx


        obs_l = []
        for i in range(len(obs_l_x)):
            obs_l.append((int((obs_l_x[i] - self.xo)/self.res),int((obs_l_y[i] - self.yo)/self.res)))
        self.polygon_index = obs_l

        min_x = int((min(obs_l_x) - self.xo)/self.res)
        max_x = int((max(obs_l_x) - self.xo)/self.res)
        min_y = int((min(obs_l_y) - self.yo)/self.res)
        max_y = int((max(obs_l_y) - self.yo)/self.res)

        d = np.array(self.__map.data).reshape(self.height, self.width)

        radius = 1 #self.radius
        # Add polygons to map
        x = min_x
        y = min_y
        while y <= max_y:
            while x <= max_x:
                if (x,y) in obs_l:
                    d[y][x] = self.polygon_space
                    for xp in range(-radius,radius+1):
                        for yp in range(-radius,radius+1):
                            if not d[y+yp,x+xp] == 100:
                                if sqrt(xp**2+yp**2) <= radius:
                                    d[y+yp,x+xp] = self.polygon_space
                else:
                    pass
                x += 1
            x = min_x
            y += 1

        rospy.loginfo("Set polygons to map.")
        self.added_polygons = True
        self.__map.data = d.reshape(self.width*self.height).tolist()

    def to_pose_stamped(self, vehicle_state):
        """
        Transform vehicle pose on format svea_msgs.msg VehicleState
        to pose on format geometry_msgs.msg PoseStamped
        """

        pose = PoseStamped()
        pose.header.seq = vehicle_state.header.seq
        pose.header.stamp = vehicle_state.header.stamp
        pose.header.frame_id = vehicle_state.header.frame_id

        pose.pose.position.x = vehicle_state.x
        pose.pose.position.y = vehicle_state.y
        pose.pose.position.z = 0

        q = quaternion_from_euler(0, 0, vehicle_state.yaw)

        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        return pose

    def check_for_updates(self, pose, scan):
        """
        If lidar scans and vehicle pose available:
            - Update map with detected Obstacles
            - Publish map updates
        """

        map_info = [self.width, self.height, self.xo, self.yo, self.res]
        _, _, update, iupdate = self.__mapping.update_map(self.__map, self.__imap, pose, scan, map_info)
        self.__map_updates_pub.publish(update)
        self.__map_inflated_pub.publish(iupdate)
