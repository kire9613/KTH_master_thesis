#!/usr/bin/env python3

"""
    @author: Daniel Duberg (dduberg@kth.se)
"""

# Python standard library
import copy
import numpy as np

# ROS
import rospy
import message_filters

# ROS messages
from geometry_msgs.msg import PoseStamped, PolygonStamped
from sensor_msgs.msg import LaserScan
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

        # Init maps
        self.__map = GridMap(map_frame_id, map_resolution, map_width, map_height,
                         map_origin_x, map_origin_y, map_origin_yaw)
        self.__inflated_map = self.__map

        # Map properties
        self.width = map_width
        self.height = map_height
        self.res = map_resolution
        self.xo = map_origin_x
        self.yo = map_origin_y

        self.__mapping = Mapping(unknown_space, free_space, c_space,
                                 occupied_space, inflate_radius, optional)

        print("Get default map...")
        #self.__default_map_sub = rospy.Subscriber('/map', OccupancyGrid, self.default_map_callback)
        #self.default_map = None
        self.default_map = rospy.wait_for_message('/map', OccupancyGrid)

        # Init map publishers
        self.__map_pub = rospy.Publisher('map', OccupancyGrid, queue_size=1,
                                         latch=True)
        self.__map_updates_pub = rospy.Publisher("map_updates",
                                                 OccupancyGridUpdate,
                                                 queue_size=10)

        self.__map_inflated_pub = rospy.Publisher('inflated_map', OccupancyGrid, queue_size=1, latch=True)

        # Subscribe to polygons typic
        self.pi_ok = False # Indicators for receiving polygons
        self.po_ok = False
        self.stay_inside_polygon = None # Polygon variables
        self.keep_outside_polygon = None
        self.polygon_index = []
        self.__stay_in_sub = rospy.Subscriber('track/stay_in', PolygonStamped, self.track_callback_in)
        self.__keep_out_sub = rospy.Subscriber('track/keep_out', PolygonStamped, self.track_callback_out)

        #self.__odom_sub = message_filters.Subscriber('odom', OdometryROS)
        #self.__scan_sub = message_filters.Subscriber('scan', LaserScanROS)

        #self.__odom_sub = message_filters.Subscriber('SVEA/vis_pose', PoseStamped)
        #self.__scan_sub = message_filters.Subscriber('scan', LaserScan)

        #Init subscribers and variables for poses and scans
        self.pose = None
        self.scan = None
        self.has_new_pose = False # Indicators for receiving poses resp. scans
        self.has_new_scan = False

        self.__state_sub = rospy.Subscriber('SVEA/state', VehicleState, self.pose_callback_2)
        self.__state_pub = rospy.Publisher('odom', Odometry, queue_size=10)
        #self.__state_sub = rospy.Subscriber('SVEA/state', VehicleState, self.pose_callback)
        #self.__scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        self.__odom_sub = message_filters.Subscriber('odom', Odometry)
        self.__scan_sub = message_filters.Subscriber('scan', LaserScan)

        dt = float(1/15)

        self.__ts = message_filters.ApproximateTimeSynchronizer([self.__odom_sub,
                                                     self.__scan_sub], 10, 1) #0.01
        self.__ts.registerCallback(self.callback)

        self.setup_ok = False

        # Setup map at initialization
        self.set_default_map() # Set default map as floor 2 map
        #self.publish_map()

        self.add_polygons() # Add polygons (virtual walls) to map

        #self.publish_map()

        # self.test_map = OccupancyGrid()
        # # Fill in the header
        # self.test_map.header.stamp = rospy.Time(0)
        # self.test_map.header.frame_id = "map"
        # # Fill in the info
        # self.test_map.info.resolution = 0.05
        # self.test_map.info.width = 721
        # self.test_map.info.height = 880
        # self.test_map.info.origin = self.default_map.info.origin
        # self.test_map.data = []
        # for i in range(880*721):
        #     self.test_map.data.append(-1)

        # while not self.setup_ok:
        #     rospy.sleep(0.1)

        #rospy.sleep(10)
        # i = 0
        # while i < 50:
        #     self.lidar_test()
        #     rospy.sleep(1)
        #    i += 1

        rospy.spin()

    def lidar_test(self):
        test_pose = rospy.wait_for_message('SVEA/state', VehicleState)
        test_pose = self.to_pose_stamped(test_pose)
        test_lidar_scan = rospy.wait_for_message('scan', LaserScan)
        map_info = [self.width, self.height, self.xo, self.yo, self.res]
        current_map = rospy.wait_for_message('map', OccupancyGrid)
        map , update = self.__mapping.update_map(current_map, test_pose, test_lidar_scan, map_info, self.polygon_index)
        #map , update = self.__mapping.update_map(self.test_map, test_pose, test_lidar_scan, map_info, self.polygon_index)
        #self.test_map.data = map.reshape(1,880*721).tolist()
        self.publish_map_update(update)

    def set_default_map(self):
        """
        Publish floor2map as map
        """
        while self.default_map == None:
            rospy.sleep(0.1)

        # map = OccupancyGrid()
        #
        # # Fill in the header
        # map.header.stamp = rospy.Time(0)
        # map.header.frame_id = "map"
        #
        # # Fill in the info
        # map.info.resolution = 0.05
        # map.info.width = 721
        # map.info.height = 880
        # map.info.origin = self.default_map.info.origin
        # map.data = []
        # for i in range(len(self.default_map.data)):
        #     map.data.append(-1)
        #
        # self.__map_pub.publish(map)

        self.__map_pub.publish(self.default_map)

    def default_map_callback(self, map):
        self.default_map = map

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

    def add_polygons(self):
        """
        Adding polygons (virtual walls) to map as obstacles
        by puplishing OccupancyGridUpdate
        """
        while self.pi_ok != True and self.po_ok != True:
            rospy.sleep(0.1)

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

        map_def = np.array(self.default_map.data).reshape(self.height, self.width)

        update = OccupancyGridUpdate()
        update.x = min_x
        update.y = min_y
        update.width = max_x - min_x + 1
        update.height = max_y - min_y + 1
        update.data = []

        x = min_x
        y = min_y
        while y <= max_y:
            while x <= max_x:
                if (x,y) in obs_l:
                    update.data.append(100)
                else:
                    update.data.append(map_def[y][x])
                x += 1
            x = min_x
            y += 1

        if isinstance(update, OccupancyGridUpdate) and len(update.data) != 0:
            print("publish polygons")
            self.setup_ok = True
            self.publish_map_update(update)

    def pose_callback_2(self, pose):
        #print("publish odom")
        odom = Odometry()
        odom.header = pose.header
        odom.child_frame_id = pose.child_frame_id
        odom.pose.pose.position.x = pose.x
        odom.pose.pose.position.y = pose.y
        q = quaternion_from_euler(0, 0, pose.yaw)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        self.__state_pub.publish(odom)

    def pose_callback(self, pose):
        #print("pose_callback")
        self.pose = self.to_pose_stamped(pose)
        self.has_new_pose = True

        if self.setup_ok:
            if self.has_new_pose and self.has_new_scan:
                self.has_new_pose = False
                self.has_new_scan = False
                #self.check_for_updates()

    def to_pose_stamped(self, vehicle_state):
        """
        Transform vehicle pose on format svea_msgs.msg VehicleState
        to pose on format geometry_msgs.msg PoseStamped
        """

        #print(vehicle_state)

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

    def to_pose_stamped_2(self, odom):
        """
        Transform vehicle pose on format svea_msgs.msg VehicleState
        to pose on format geometry_msgs.msg PoseStamped
        """

        pose = PoseStamped()
        pose.header = odom.header
        pose.pose.position.x = odom.pose.pose.position.x
        pose.pose.position.y = odom.pose.pose.position.yy
        pose.pose.position.z = 0

        pose.pose.orientation.x = odom.pose.pose.orientation.x
        pose.pose.orientation.y = odom.pose.pose.orientation.y
        pose.pose.orientation.z = odom.pose.pose.orientation.z
        pose.pose.orientation.w = odom.pose.pose.orientation.w

        return pose

    def scan_callback(self, scan):
        #print("scan_callback")
        self.scan = scan
        self.has_new_scan = True

        if self.setup_ok:
            if self.has_new_pose and self.has_new_scan:
                self.has_new_pose = False
                self.has_new_scan = False
                #self.check_for_updates()

    def check_for_updates(self):

        if self.pose != None and self.scan != None:
            #print(abs(self.pose.header.stamp-self.scan.header.stamp))

            map_info = [self.width, self.height, self.xo, self.yo, self.res]
            current_map = rospy.wait_for_message('map', OccupancyGrid)
            _ , update = self.__mapping.update_map(current_map, self.pose, self.scan, map_info, self.polygon_index)
            #map , update = self.__mapping.update_map(self.test_map, test_pose, test_lidar_scan, map_info, self.polygon_index)
            #self.test_map.data = map.reshape(1,880*721).tolist()

            #self.__map, update = self.__mapping.update_map(self.__map, self.pose, self.scan)

            if isinstance(update, OccupancyGridUpdate) and len(update.data) != 0:
                self.publish_map_update(update)
                #map = copy.deepcopy(self.__map)
                #self._publish_inflated_map_inflated_map = self.__mapping.inflate_map(map)
                #self.publish_inflated_map()
            else:
                pass
                #self.publish_map()

    def callback(self, odom_ros, scan_ros):
        print("callback")
        scan = scan_ros #self.from_ros_scan(scan_ros)
        pose = self.to_pose_stamped_2(odom_ros) #self.from_ros_odom(odom_ros)

        if self.setup_ok:
            map_info = [self.width, self.height, self.xo, self.yo, self.res]
            current_map = rospy.wait_for_message('map', OccupancyGrid)
            _ , update = self.__mapping.update_map(current_map, pose, scan, map_info, self.polygon_index)

            if isinstance(update, OccupancyGridUpdate) and len(update.data) != 0:
                self.publish_map_update(update)

    def publish_map(self):
        """
        Publishing complete map to topic /map
        """
        print("publish_map")
        map = self.__map.to_message()
        self.__map_pub.publish(map)

    def publish_map_update(self, update):
        """
        Publishing map update of format OccupancyGridUpdate
        """
        print("publish_map_update")
        update_ros = self.map_update_to_message(update)
        self.__map_updates_pub.publish(update_ros)

    def publish_inflated_map(self):
        """
        Publishing inflated complete map to topic 'inflated_map'
        """
        print("publish_inflated_map")
        map = self.__inflated_map.to_message()
        self.__map_inflated_pub.publish(map)

    def map_to_message(self, map):
        return map

    def map_update_to_message(self, update):
        return update
