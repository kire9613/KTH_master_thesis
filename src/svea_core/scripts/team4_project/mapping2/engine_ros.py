#!/usr/bin/env python3

# Python standard library
import copy
import numpy as np
import os.path
import re
from copy import deepcopy

# ROS
import rospy
import message_filters

# ROS messages
from geometry_msgs.msg import PoseStamped, PolygonStamped
from sensor_msgs.msg import LaserScan, PointCloud
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from map_msgs.msg import OccupancyGridUpdate
from svea_msgs.msg import VehicleState

from grid_map import GridMap, quaternion_from_euler
from mapping import Mapping
import matplotlib.pyplot as plt


class EngineROS:
    def __init__(self, map_frame_id, map_resolution, map_width, map_height,
                 map_origin_x, map_origin_y, map_origin_yaw, inflate_radius,
                 unknown_space, free_space, c_space, occupied_space, optional=None):

        print("Init EngineROS")
        rospy.init_node('Mapper')

        # Map properties
        self.width = map_width
        self.height = map_height
        self.res = map_resolution
        self.xo = map_origin_x
        self.yo = map_origin_y

        self.__mapping = Mapping(unknown_space, free_space, c_space,
                                 occupied_space, inflate_radius, optional)

        self.default_map = rospy.wait_for_message('/map', OccupancyGrid)
        self.__map = None
        #self.__infl_map = None
        #self.__map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        # Init map publishers
        self.__map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1,
                                         latch=True)

        self.__d_map_pub = rospy.Publisher('/dmap', OccupancyGrid, queue_size=1,
                                         latch=True)

        self.__map_updates_pub = rospy.Publisher("map_updates",
                                                 OccupancyGridUpdate,
                                                 queue_size=10)


        #self.__map_inflated_pub = rospy.Publisher('inflated_map', OccupancyGrid, queue_size=1, latch=True)

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
        #self.__state_sub = rospy.Subscriber('/state', VehicleState, self.pose_callback)
        self.__state_sub = rospy.Subscriber('/vis_pose', PoseStamped, self.pose_callback_sim)
        #self.__scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.__scan_sub = rospy.Subscriber('/viz_lidar_points', PointCloud, self.scan_callback_sim)

        """
        Freq. difference for state and scan gives bad performance
        Trigger map update on slowest message
        Tune mapping based on performance on car
        Solution? -> Interpolate position between messages?
        """

        self.setup_ok = False

        if os.path.isfile("default_map.txt"):
            self.set_map_from_file()
        else:
            pass
            self.set_default_map()
            self.add_polygons()
            while not self.added_polygons:
                rospy.sleep(1)
            self.__map_pub.publish(self.__map)
            rospy.sleep(5)
            self.write_map_to_file()

        rospy.spin()

    def map_callback(self, map):
        pass
        #self.__map = map
        #print(map.header)
        #print(map.info)

    def write_map_to_file(self):
        """ Saves map at initialization to file """

        current_map = self.__map
        f = open("default_map.txt", "a")
        content = str(current_map)
        f.write(content)
        f.close()
        print("Map written to file.")
        self.setup_ok = True

    def set_map_from_file(self):
        """ Set map at initialization from file """

        print("Read map from file...")
        map = OccupancyGrid()
        f = open("default_map.txt", "r")
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
        self.default_map = deepcopy(self.__map)
        #print(type(self.__map))
        #self.__infl_map = map
        #self.__infl_map.data = self.__mapping.inflate_map(self.__infl_map).reshape(self.width*self.heigh)
        #self.__map_inflated_pub(self.__infl_map)

        self.__map_pub.publish(self.__map)
        self.__d_map_pub.publish(self.default_map)
        rospy.sleep(5)
        self.setup_ok = True
        print("Read map from file done.")

    # def set_default_map(self):
    #     """
    #     Publish floor2 map as map
    #     """
    #     while self.default_map == None:
    #         rospy.sleep(0.1)
    #
    #     self.__map = self.default_map
    #     #self.__infl_map = self.default_map
    #     #self.__infl_map.data = self.__mapping.inflate_map(self.__infl_map).reshape(self.width*self.heigh)
    #     #self.__map_inflated_pub(self.__infl_map)
    #     self.__map_pub.publish(self.__map)
    #     print("Default map published.")

    def track_callback_out(self, polygon):
        self.keep_outside_polygon = polygon
        rospy.sleep(3)
        self.pi_ok = True
        print("Inner polygon ok!")

    def track_callback_in(self, polygon):
        self.stay_inside_polygon = polygon
        rospy.sleep(3)
        self.po_ok = True
        print("Outer polygon ok!")

    # def add_polygons(self):
    #     """
    #     Adding polygons (virtual walls) to map as obstacles.
    #     """
    #
    #     while self.pi_ok != True and self.po_ok != True:
    #         rospy.sleep(0.1)
    #
    #     # Create lines of obstacle dots along polygon edges
    #     obs_l_x = []
    #     obs_l_y = []
    #
    #     for i in range(1,len(self.keep_outside_polygon.polygon.points)):
    #         p1 = self.keep_outside_polygon.polygon.points[i-1]
    #         p2 = self.keep_outside_polygon.polygon.points[i]
    #         dx = abs(p1.x-p2.x)
    #         dy = abs(p1.y-p2.y)
    #         if dx > dy:
    #             lx = np.linspace(float(p1.x), float(p2.x), num=int(dx/0.05)).tolist()
    #             ly = np.linspace(float(p1.y), float(p2.y), num=len(lx)).tolist()
    #             obs_l_x = obs_l_x + lx
    #             obs_l_y = obs_l_y + ly
    #         else:
    #             ly = np.linspace(float(p1.y), float(p2.y), num=int(dy/0.05)).tolist()
    #             lx = np.linspace(float(p1.x), float(p2.x), num=len(ly)).tolist()
    #             obs_l_y = obs_l_y + ly
    #             obs_l_x = obs_l_x + lx
    #
    #     ps = self.keep_outside_polygon.polygon.points[0]
    #     pe = self.keep_outside_polygon.polygon.points[-1]
    #     dx = abs(ps.x-pe.x)
    #     dy = abs(ps.y-pe.y)
    #     if dx > dy:
    #         lx = np.linspace(float(ps.x), float(pe.x), num=int(dx/0.05)).tolist()
    #         ly = np.linspace(float(ps.y), float(pe.y), num=len(lx)).tolist()
    #         obs_l_x = obs_l_x + lx
    #         obs_l_y = obs_l_y + ly
    #     else:
    #         ly = np.linspace(float(ps.y), float(pe.y), num=int(dy/0.05)).tolist()
    #         lx = np.linspace(float(ps.x), float(pe.x), num=len(ly)).tolist()
    #         obs_l_y = obs_l_y + ly
    #         obs_l_x = obs_l_x + lx
    #
    #
    #     for i in range(1,len(self.stay_inside_polygon.polygon.points)):
    #         p1 = self.stay_inside_polygon.polygon.points[i-1]
    #         p2 = self.stay_inside_polygon.polygon.points[i]
    #         dx = abs(p1.x-p2.x)
    #         dy = abs(p1.y-p2.y)
    #         if dx > dy:
    #             lx = np.linspace(float(p1.x), float(p2.x), num=int(dx/0.05)).tolist()
    #             ly = np.linspace(float(p1.y), float(p2.y), num=len(lx)).tolist()
    #             obs_l_x = obs_l_x + lx
    #             obs_l_y = obs_l_y + ly
    #         else:
    #             ly = np.linspace(float(p1.y), float(p2.y), num=int(dy/0.05)).tolist()
    #             lx = np.linspace(float(p1.x), float(p2.x), num=len(ly)).tolist()
    #             obs_l_y = obs_l_y + ly
    #             obs_l_x = obs_l_x + lx
    #
    #     ps = self.stay_inside_polygon.polygon.points[0]
    #     pe = self.stay_inside_polygon.polygon.points[-1]
    #     dx = abs(ps.x-pe.x)
    #     dy = abs(ps.y-pe.y)
    #     if dx > dy:
    #         lx = np.linspace(float(ps.x), float(pe.x), num=int(dx/0.05)).tolist()
    #         ly = np.linspace(float(ps.y), float(pe.y), num=len(lx)).tolist()
    #         obs_l_x = obs_l_x + lx
    #         obs_l_y = obs_l_y + ly
    #     else:
    #         ly = np.linspace(float(ps.y), float(pe.y), num=int(dy/0.05)).tolist()
    #         lx = np.linspace(float(ps.x), float(pe.x), num=len(ly)).tolist()
    #         obs_l_y = obs_l_y + ly
    #         obs_l_x = obs_l_x + lx
    #
    #
    #     obs_l = []
    #     for i in range(len(obs_l_x)):
    #         obs_l.append((int((obs_l_x[i] - self.xo)/self.res),int((obs_l_y[i] - self.yo)/self.res)))
    #     self.polygon_index = obs_l
    #
    #     min_x = int((min(obs_l_x) - self.xo)/self.res)
    #     max_x = int((max(obs_l_x) - self.xo)/self.res)
    #     min_y = int((min(obs_l_y) - self.yo)/self.res)
    #     max_y = int((max(obs_l_y) - self.yo)/self.res)
    #
    #     d = np.array(self.__map.data).reshape(self.height, self.width)
    #
    #     # Add polygons to map
    #     x = min_x
    #     y = min_y
    #     while y <= max_y:
    #         while x <= max_x:
    #             if (x,y) in obs_l:
    #                 d[y][x] = 100
    #             else:
    #                 pass
    #             x += 1
    #         x = min_x
    #         y += 1
    #
    #     print("Set polygons to map.")
    #     self.added_polygons = True
    #     self.__map.data = d.reshape(self.width*self.height).tolist()

    def pose_callback(self, pose):
        #print("pose_callback")
        self.pose = self.to_pose_stamped(pose)
        self.received_pose = True
        #if self.setup_ok and self.received_scan:
            #print("trigger map update")
            #self.check_for_updates()

    def pose_callback_sim(self, pose):
        #print("pose_callback")
        self.pose = pose
        self.received_pose = True
        #if self.setup_ok and self.received_scan:
            #print("trigger map update")
            #self.check_for_updates()

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

    def scan_callback_sim(self, scan):
        #print("scan_callback sim")
        self.scan = scan
        self.received_scan = True
        if self.setup_ok and self.received_pose:
            #print("trigger map update")
            self.check_for_updates_sim()

    def scan_callback(self, scan):
        #print("scan_callback")
        self.scan = scan
        self.received_scan = True
        if self.setup_ok and self.received_pose:
            #print("trigger map update")
            self.check_for_updates()

    def check_for_updates_sim(self):
        """
        If lidar scans and vehicle pose available
        update map with detected Obstacles
        publish updated map
        """
        print("check for updates_sim")

        if self.pose != None and self.scan != None:
            map_info = [self.width, self.height, self.xo, self.yo, self.res]
            new_map = rospy.wait_for_message('/dmap', OccupancyGrid)
            #print(self.__map.header)
            #self.__map.header.seq += 1
            #new_map = deepcopy(self.__map)
            tic = rospy.Time.now()
            d, update = self.__mapping.update_map_sim(self.__map, self.pose, self.scan, map_info)
            new_map.data = d.reshape(self.width*self.height)
            #print(new_map.header)
            toc = rospy.Time.now()
            #print(str((toc-tic).to_sec()))
            self.__map_updates_pub.publish(update)
            #self.__map_pub.publish(new_map)

    def check_for_updates(self):
        """
        If lidar scans and vehicle pose available
        update map with detected Obstacles
        publish updated map
        """
        print("check for updates")

        #tic = rospy.Time.now()
        #if use_matplotlib or use_rviz:
        #    svea.visualize_data()
        #else:
        #    rospy.loginfo_throttle(1, state)
        #toc = rospy.Time.now()
        #rospy.loginfo_throttle(0.5, (tic-toc).to_sec())
        #rospy.loginfo_throttle(0.5, 'len(svea.data_handler.x): ' + str(len(svea.data_handler.x)))

        if self.pose != None and self.scan != None:
            map_info = [self.width, self.height, self.xo, self.yo, self.res]
            new_map = rospy.wait_for_message('/dmap', OccupancyGrid)
            tic = rospy.Time.now()
            d, update = self.__mapping.update_map(self.__map, self.pose, self.scan, map_info)
            new_map.data = d.reshape(self.width*self.height)
            toc = rospy.Time.now()
            self.__map_updates_pub.publish(update)
            #self.__map_pub.publish(new_map)

        # if self.pose != None and self.scan != None:
        #     map_info = [self.width, self.height, self.xo, self.yo, self.res]
        #     new_map = deepcopy(self.__map)
        #     tic = rospy.Time.now()
        #     new_map.data = self.__mapping.update_map(self.__map, self.pose, self.scan, map_info).reshape(self.width*self.height)
        #     self.__map_pub.publish(new_map)
        #     toc = rospy.Time.now()
        #     #print(str((toc-tic).to_sec()))

        # if self.pose != None and self.scan != None:
        #     map_info = [self.width, self.height, self.xo, self.yo, self.res]
        #     new_map = self.__map
        #     map = self.__mapping.update_map(new_map, self.pose, self.scan, map_info)
        #     new_map.data = map.reshape(self.width*self.height).tolist()
        #     #new_map.header. =rospy.time()
        #     self.__map_pub.publish(new_map)
        #     #self.__infl_map = self.__map
        #     #self.__infl_map.data = self.__mapping.inflate_map(self.__infl_map).reshape(self.width*self.heigh)
        #     #self.__map_inflated_pub(self.__infl_map)
