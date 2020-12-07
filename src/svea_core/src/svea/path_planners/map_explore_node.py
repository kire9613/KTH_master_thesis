#!/usr/bin/env python

"""
    @Team1
"""

from __future__ import print_function

# Python standard library
import rospy
import numpy as np
from math import floor

# ROS
import rospy
import tf

# ROS messages
from geometry_msgs.msg import Point, Pose, Quaternion
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import Path
from std_msgs.msg import Bool

# SVEA

import matplotlib.pyplot as plt

## MAP EXPLORER PARAMS ########################################################
update_rate = 5 # [Hz]

###############################################################################

class Node:
    """
    asd
    """
    def __init__(self):

        rospy.init_node('explore_node')

        self.window = 3 # m, the window size for collision check

        self.collision_pub = rospy.Publisher('collision', Bool, queue_size=1, latch=False)
        self.problem_pub = rospy.Publisher('problem_map', OccupancyGridUpdate, queue_size=1, latch=False)

        self.map_sub = rospy.Subscriber('costmap_node/costmap/costmap_updates', OccupancyGridUpdate, self.callback_map_updates)
        # self.map_sub = rospy.Subscriber('costmap_node/costmap/costmap', OccupancyGrid, self.callback_map)
        # self.rolling_map_sub = rospy.Subscriber('costmap_node/costmap/costmap', OccupancyGrid, self.callback_rolling_map)
        self.glb_map_sub = rospy.Subscriber('map', OccupancyGrid, self.callback_glb_map)
        self.path_sub = rospy.Subscriber('path_plan', Path, self.callback_path)

        self.scan = LaserScan()
        self.path = Path()
        self.map_matrix = None
        self.path_lookup = None
        self.index_lookup = None

        self.rate_timeout = rospy.Rate(1)

    def callback_glb_map(self, map):
        self.global_height = map.info.height
        self.global_width = map.info.width
        self.resolution = map.info.resolution
        self.oob_delimiter = max(self.global_width,self.global_height) + 1
        self.map_frame_id = 'map'

        self.map_matrix = np.full((self.global_width, self.global_height), np.int8(50), dtype=np.int8)
        self.path_lookup = np.zeros((self.global_width,self.global_height), dtype=np.int16)
        self.index_lookup = np.zeros((self.global_width,self.global_height), dtype=np.int16)

    def callback_map_updates(self, map):
        # print("got new map update!")
        self.map = map
        self.map_matrix[map.x:map.x+map.width,map.y:map.y+map.height] = np.reshape(map.data, (map.width, map.height), order='F')

        map_slice = self.collision_check()

        if map_slice != None:
            self.problem_pub.publish(map_slice)
            #self.rate_timeout.sleep()


    def callback_map(self, map):
        # print("got new map")
        self.map = map
        self.map_matrix = np.reshape(map.data, (map.info.width, map.info.height), order='F')
        # plt.imshow(self.map_matrix.T)
        # plt.show()

    def callback_rolling_map(self, map):
        # print("got new map")
        self.map = map
        self.map_matrix = np.zeros((width, height), dtype=np.int8)
        # self.map_matrix = np.reshape(map.data, (map.info.width, map.info.height), order='F')
        # for big matrix
        x_ind = int(floor(map.info.origin.position.x/resolution))
        y_ind = int(floor(map.info.origin.position.y/resolution))
        xmin,xmax = np.clip([x_ind,x_ind+map.info.width],0,width-1)
        ymin,ymax = np.clip([y_ind,y_ind+map.info.height],0,height-1)
        if x_ind>0 and y_ind>0:
            self.map_matrix[xmin:xmax,ymin:ymax] = np.reshape(
                map.data, (map.info.width,map.info.height), order='F'
            )[:xmax-xmin,:ymax-ymin]
        elif y_ind<0:
            self.map_matrix[xmin:xmax,ymin:ymax] = np.reshape(
                map.data, (map.info.width,map.info.height), order='F'
            )[:,-y_ind:]
        elif x_ind<0:
            self.map_matrix[xmin:xmax,ymin:ymax] = np.reshape(
                map.data, (map.info.width,map.info.height), order='F'
            )[-x_ind:,:]
        else:
            self.map_matrix[xmin:xmax,ymin:ymax] = np.reshape(
                map.data, (map.info.width,map.info.height), order='F'
            )[-x_ind:,-y_ind:]

        # plt.imshow(self.map_matrix.T)
        # plt.show()

    def callback_state(self, state):
        self.state = state

    def callback_scan(self, scan):
        self.scan = scan

    def callback_path(self, path):
        # Only run each time a new global path comes in.
        self.path = path
        self.path_lookup[:] = np.int16(0)
        self.index_lookup[:] = np.int16(0)

        for i in range(len(path.poses) - 2):
            x_start = self.path.poses[i].pose.position.x
            y_start = self.path.poses[i].pose.position.y
            x_end = self.path.poses[i+1].pose.position.x
            y_end = self.path.poses[i+1].pose.position.y

            int_x_start = np.int16(x_start/self.resolution)
            int_y_start = np.int16(y_start/self.resolution)
            int_x_end = np.int16(x_end/self.resolution)
            int_y_end = np.int16(y_end/self.resolution)

            ray = self.raytrace(int_x_start, int_y_start, int_x_end, int_y_end)

            l = 0
            while ray[l,0] != self.oob_delimiter:
                #self.mapper.add_to_map(cell[0],cell[1],path_val)
                self.path_lookup[ray[l,0], ray[l,1]] = 1
                self.index_lookup[ray[l,0], ray[l,1]] = i
                """
                for r in range(1, 2):
                    t = 0
                    while t <= 2*np.pi:
                        a = ray[l,0] + r*np.cos(t)
                        b = ray[l,1] + r*np.sin(t)
                        a = int(a)
                        b = int(b)
                        if self.is_in_bounds(a,b):
                            self.path_lookup[(a,b)] = 1
                            self.index_lookup[(a,b)] = i
                        t = t + np.pi/32
                """
                l += 1

            self.path_lookup[int_x_end, int_y_end] = 1
            self.index_lookup[int_x_end, int_y_end] = len(path.poses) - 1

    def collision_check(self):

        map_slice = None

        plan_width = 120
        plan_height = 120

        # If rolling window is not set to true
        # window = int(floor(self.window/resolution))

        # x_ind = int(floor(self.state.x/resolution))
        # y_ind = int(floor(self.state.y/resolution))
        # xmin = np.clip(x_ind-window,0,width-1)
        # xmax = np.clip(x_ind+window,0,width-1)
        # ymin = np.clip(y_ind-window,0,height-1)
        # ymax = np.clip(y_ind+window,0,height-1)

        # close_path = np.zeros((width,height))
        # close_path[xmin:xmax,ymin:ymax] = path_lookup[xmin:xmax,ymin:ymax]

        matr = self.map_matrix*self.path_lookup

        collisions = np.where(matr >= 75)

        if collisions[0].size != 0:
            self.collision_pub.publish(Bool(True))
            orig_x = collisions[0][0]
            orig_y = collisions[1][0]

            xmin = np.clip(orig_x - plan_width/2,0,self.global_width)
            xmax = np.clip(orig_x + plan_width/2,0,self.global_width)
            ymin = np.clip(orig_y - plan_height/2,0,self.global_height)
            ymax = np.clip(orig_y + plan_height/2,0,self.global_height)

            final_width = xmax - xmin - 1
            final_height = ymax - ymin - 1

            map_matr = self.map_matrix[xmin:xmax - 1, ymin:ymax - 1]

            # print(xmin, xmax)
            # print(ymin, ymax)

            # print(orig_x,orig_y)
            rospy.loginfo("Intersected path found!")

            obs_ind = self.index_lookup[orig_x, orig_y]

            print(obs_ind)

            start_x = np.int16(self.path.poses[obs_ind - 5].pose.position.x/self.resolution)
            start_y = np.int16(self.path.poses[obs_ind - 5].pose.position.y/self.resolution)
            goal_x = np.int16(self.path.poses[obs_ind + 5].pose.position.x/self.resolution)
            goal_y = np.int16(self.path.poses[obs_ind + 5].pose.position.y/self.resolution)

            # print(start_x, start_y)
            # print(goal_x, goal_y)

            map_matr[start_x - xmin, start_y - ymin] = -np.int8(1)
            map_matr[goal_x - xmin, goal_y - ymin] = -np.int8(2)

            map_slice = OccupancyGridUpdate()
            map_slice.header.stamp = rospy.Time.now()
            map_slice.header.frame_id = self.map_frame_id
            map_slice.x = xmin
            map_slice.y = ymin
            map_slice.width = final_width
            map_slice.height = final_height
            map_slice.data = map_matr.reshape(-1)

        return map_slice

    def is_in_bounds(self, x, y):
        """Returns weather (x, y) is inside grid_map or not."""
        if x >= 0 and x < self.global_width:
            if y >= 0 and y < self.global_height:
                return True
        else:
            print("False: ({0},{1})".format(x,y))
            return False

    def raytrace(self, start_x, start_y, end_x, end_y):
        """Returns all cells in the grid map that has been traversed
        from start to end, including start and excluding end.
        """
        x = start_x
        y = start_y
        dx = np.abs(end_x - start_x)
        dy = np.abs(end_y - start_y)
        n = dx + dy
        x_inc = 1
        if end_x <= start_x:
            x_inc = -1
        y_inc = 1
        if end_y <= start_y:
            y_inc = -1
        error = dx - dy
        dx *= 2
        dy *= 2
        traversed = np.full((n + 1,2), np.int16(self.oob_delimiter))
        i = 0
        while i < np.int16(n):
            traversed[i,0] = np.int16(x)
            traversed[i,1] = np.int16(y)
            if error > 0:
                x += x_inc
                error -= dy
            else:
                if error == 0:
                    traversed[i,0] = np.int16(x + x_inc)
                    traversed[i,1] = np.int16(y)
                y += y_inc
                error += dx
            i += 1
        return traversed

    def run(self):
        rate = rospy.Rate(update_rate)

        while not rospy.is_shutdown():

            rate.sleep()

        rospy.spin()

if __name__ == '__main__':
    try:
        node = Node()
        node.run()
    except rospy.ROSInterruptException:
        pass
