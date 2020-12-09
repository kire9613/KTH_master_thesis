#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function


import rospy
import numpy as np
from math import floor

import rospy
import tf

from geometry_msgs.msg import Point, Pose, Quaternion
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Int16MultiArray

import svea.bresenham.bresenham as bresenham

import matplotlib.pyplot as plt

"""
map_explore_node: Collision check
"""

__author__ = "Team 1"

## MAP EXPLORER PARAMS ########################################################

update_rate = 5 # [Hz]
width = 635#1269
height = 284#567
resolution = 0.1
shift_x = np.int16(30.549770/resolution)
shift_y = np.int16(11.414917/resolution)
collision_distance = 5
timeout_rate = 1.0

###############################################################################
#                                   FLOOR 2                                   #
###############################################################################
update_rate = 2 # [Hz]
width = 879
height = 171
resolution = 0.05
shift_x = 0
shift_y = 0
collision_distance = 5
timeout_rate = 1.0
###############################################################################

class Node:
    """
    asd
    """
    def __init__(self):

        rospy.init_node('explore_node')
        self.collision = False

        self.window = 3 # m, the window size for collision check

        self.collision_pub = rospy.Publisher('collision', Bool, queue_size=1, latch=False)
        self.index_pub = rospy.Publisher('problem_index', Int16MultiArray, queue_size=1, latch=False)
        self.problem_pub = rospy.Publisher('problem_map', OccupancyGridUpdate, queue_size=1, latch=False)

        self.collision_sub = rospy.Subscriber('collision', Bool, self.callback_collision)
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

        self.rate_timeout = rospy.Rate(timeout_rate)

    def callback_collision(self, msg):
        self.collision = msg.data

    def callback_glb_map(self, map):
        self.global_height = map.info.height
        self.global_width = map.info.width
        self.resolution = map.info.resolution
        self.oob_delimiter = max(self.global_width,self.global_height) + 1
        self.map_frame_id = 'map'

        self.map_matrix = np.full((self.global_width, self.global_height), np.int8(50), dtype=np.int8)
        self.path_lookup = np.zeros((self.global_width,self.global_height), dtype=np.int32)
        self.index_lookup = np.zeros((self.global_width,self.global_height), dtype=np.int32)

    def callback_map_updates(self, map):
        # print("got new map update!")
        self.map = map
        self.map_matrix[map.x:map.x+map.width,map.y:map.y+map.height] = np.reshape(map.data, (map.width, map.height), order='F')
        self.map_update_matrix= np.reshape(map.data, (map.width, map.height), order='F')

        map_slice = self.collision_check()

        if map_slice != None:
            self.problem_pub.publish(map_slice)

    def callback_map(self, map):
        # print("got new map")
        self.map = map
        self.map_matrix = np.reshape(map.data, (map.info.width, map.info.height), order='F')
        # plt.imshow(self.map_matrix.T,origin="lower")
        # plt.show()

    def callback_rolling_map(self, map):
        # print("got new map")
        self.map = map
        self.map_matrix = np.zeros((width, height), dtype=np.int8)
        # self.map_matrix = np.reshape(map.data, (map.info.width, map.info.height), order='F')
        # for big matrix
        x_ind = np.int16((map.info.origin.position.x+30.549770)/resolution)
        y_ind = np.int16((map.info.origin.position.y+11.414917)/resolution)
        xmin,xmax = np.clip([x_ind,x_ind+map.info.width],0,width-1)
        ymin,ymax = np.clip([y_ind,y_ind+map.info.height],0,height-1)
        # self.xmin = xmin
        # self.xmax = xmax
        # self.ymin = ymin
        # self.ymax = ymax
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

        # plt.imshow(self.map_matrix.T,origin="lower")
        # plt.show()

    def callback_state(self, state):
        self.state = state

    def callback_scan(self, scan):
        self.scan = scan

    def callback_path(self, path):
        # Only run each time a new global path comes in.
        self.path = path
        # generator = (((np.int16(poses.pose.position.x/resolution)+shift_x,np.int16(poses.pose.position.y/resolution)+shift_y), i) for i, poses in enumerate(self.path.poses))
        # self.index_lookup = dict(generator)
        # for key in self.index_lookup:
        #     # print(key)
        #     self.path_lookup[key[0],key[1]] = 1
        # generator =
        gen = np.array([[
            np.int16(self.path.poses[i].pose.position.x/resolution)+shift_x,
            np.int16(self.path.poses[i].pose.position.y/resolution)+shift_y,
            np.int16(self.path.poses[i+1].pose.position.x/resolution)+shift_x,
            np.int16(self.path.poses[i+1].pose.position.y/resolution)+shift_y,
            i,
                 ]
        for i in range(len(self.path.poses)-1)],dtype=np.int32)
        self.path_lookup = np.zeros((self.global_width,self.global_height),dtype=np.int32)
        self.index_lookup = np.zeros((self.global_width,self.global_height),dtype=np.int32)
        bresenham.bres_segments_count(gen,self.path_lookup,self.index_lookup)

        # plt.imshow(self.path_lookup.T,origin="lower")
        # plt.show()

    def collision_check(self):

        map_slice = None

        plan_width = 2*self.map.width
        plan_height = 2*self.map.height

        # matr = self.map_matrix*self.path_lookup

        matr = self.map_update_matrix*self.path_lookup[self.map.x:self.map.x+self.map.width,self.map.y:self.map.y+self.map.height]

        collisions = np.where(matr >= 75)
        # print(collisions[0])

        if collisions[0].size != 0 and not self.collision:
            self.collision_pub.publish(Bool(True))

            fig, (ax1,ax2,ax3,ax4) = plt.subplots(4)
            ax1.imshow(self.map_update_matrix.T,origin="lower")
            ax2.imshow(self.path_lookup[self.map.x:self.map.x+self.map.width,self.map.y:self.map.y+self.map.height].T,origin="lower")
            ax3.imshow(matr.T,origin="lower")
            ax4.imshow(self.index_lookup[self.map.x:self.map.x+self.map.width,self.map.y:self.map.y+self.map.height].T,origin="lower")
            plt.show()

            orig_x = collisions[0][0]+self.map.x
            orig_y = collisions[1][0]+self.map.y

            obs_ind = self.index_lookup[orig_x, orig_y] # gives index in global path of the collision point.
            # obs_ind = self.index_lookup.get((orig_x,orig_y)) # gives index in global path of the collision point.
            print(obs_ind)

            # TODO: collision_distance can actually be IMPORTED in collision_node instead
            self.index_pub.publish(Int16MultiArray(
                data=[obs_ind,collision_distance]
            ))

            # print(obs_ind)
            print(obs_ind + collision_distance)
            print(len(self.path.poses))

            start_x = np.int16(self.path.poses[obs_ind - collision_distance].pose.position.x/self.resolution)+shift_x
            goal_x = np.int16(self.path.poses[obs_ind + collision_distance].pose.position.x/self.resolution)+shift_x

            start_y = np.int16(self.path.poses[obs_ind - collision_distance].pose.position.y/self.resolution)+shift_y
            goal_y = np.int16(self.path.poses[obs_ind + collision_distance].pose.position.y/self.resolution)+shift_y

            xmin = np.clip(orig_x - plan_width/2,0,self.global_width)
            xmax = np.clip(orig_x + plan_width/2,0,self.global_width)
            ymin = np.clip(orig_y - plan_height/2,0,self.global_height)
            ymax = np.clip(orig_y + plan_height/2,0,self.global_height)

            final_width = xmax - xmin - 1
            final_height = ymax - ymin - 1

            map_matr = self.map_matrix[xmin:xmax - 1, ymin:ymax - 1]

            rospy.loginfo("Intersected path found!")
            # print(orig_x,orig_y)


            # print(start_x,goal_x,xmin,xmax)
            # print(start_y,goal_y,ymin,ymax)

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