#!/usr/bin/env python
# -*- coding: utf-8 -*-

# TODO:
# map_matr[start_x - xmin, start_y - ymin] = -np.int8(1)
# IndexError: index 23 is out of bounds for axis 1 with size 23

from __future__ import print_function
from __future__ import division

import rospy
import numpy as np

import rospy

from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Int16MultiArray, Float32MultiArray

import svea.bresenham.bresenham as bresenham
import svea.pyastar.pyastar as pyastar

import matplotlib.pyplot as plt

"""
map_collision_node: Collision check. Takes care of collision using A* and then
updates path
"""

__author__ = "Team 1"

## MAP EXPLORER PARAMS ########################################################

update_rate = 2 # [Hz]
width = 635#1269
height = 284#567
resolution = 0.1
shift_x = np.int16(30.549770/resolution)
shift_y = np.int16(11.414917/resolution)
collision_distance = 5
# timeout_rate = 1.0

###############################################################################
#                                   FLOOR 2                                   #
###############################################################################
# update_rate = 2 # [Hz]
# width = 879
# height = 171
# resolution = 0.05
# shift_x = 0
# shift_y = 0
# collision_distance = 5
# timeout_rate = 1.0
###############################################################################

class Node:
    """
    asd
    """
    def __init__(self):

        rospy.init_node('explore_node')
        self.collision = False

        self.traj_x = None
        self.traj_y = None

        self.collision_pub = rospy.Publisher('collision', Bool, queue_size=1, latch=False)
        self.problem_pub = rospy.Publisher('problem_map', OccupancyGridUpdate, queue_size=1, latch=False)

        self.initial_traj_sub = rospy.Subscriber('init_traj', Float32MultiArray, self.callback_init_traj)

        self.collision_sub = rospy.Subscriber('collision', Bool, self.callback_collision)
        self.map_sub = rospy.Subscriber('costmap_node/costmap/costmap_updates', OccupancyGridUpdate, self.callback_map_updates)

        self.glb_map_sub = rospy.Subscriber('map', OccupancyGrid, self.callback_glb_map)
        # self.path_sub = rospy.Subscriber('path_plan', Path, self.callback_path)

        self.scan = LaserScan()
        self.path = Path()
        self.map_matrix = None
        self.path_lookup = None
        self.index_lookup = None

        self.solution_pub = rospy.Publisher('trajectory_updates', Path, queue_size=1, latch=False)
        self.problem_sub = rospy.Subscriber('problem_map', OccupancyGridUpdate, self.callback_problem)

        # self.map_sub = rospy.Subscriber('map', OccupancyGrid, self.callback_map)

        self.rate = rospy.Rate(update_rate)

        self.planner = None

    def callback_init_traj(self, msg):
        rospy.loginfo("Map collision trying to read initial trajectory data!")
        n = len(msg.data)
        self.traj_x = np.array(msg.data[0:int(n/2)])
        self.traj_y = np.array(msg.data[int(n/2):])
        assert len(self.traj_x)==len(self.traj_y)
        rospy.loginfo("Map collision node received initial trajectory!")
        self.traj_to_occupgrid()
        rospy.loginfo("Map collision node converted initial trajectory to OccupGrid!")

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

        plan_width = 1.0*self.map.width
        plan_height = 1.0*self.map.height

        # matr = self.map_matrix*self.path_lookup

        matr = self.map_update_matrix*self.path_lookup[self.map.x:self.map.x+self.map.width,self.map.y:self.map.y+self.map.height]

        collisions = np.where(matr >= 90)
        # print(collisions[0])

        if collisions[0].size != 0 and not self.collision:
            self.collision_pub.publish(Bool(True))

            # fig, (ax1,ax2,ax3,ax4) = plt.subplots(4)
            # ax1.imshow(self.map_update_matrix.T,origin="lower")
            # ax2.imshow(self.path_lookup[self.map.x:self.map.x+self.map.width,self.map.y:self.map.y+self.map.height].T,origin="lower")
            # ax3.imshow(matr.T,origin="lower")
            # ax4.imshow(self.index_lookup[self.map.x:self.map.x+self.map.width,self.map.y:self.map.y+self.map.height].T,origin="lower")
            # plt.show()

            orig_x = collisions[0][0]+self.map.x
            orig_y = collisions[1][0]+self.map.y

            # print("Map center at ",orig_x,orig_y)

            self.obs_ind = self.index_lookup[orig_x, orig_y] # gives index in global path of the collision point.
            # self.obs_ind = self.index_lookup.get((orig_x,orig_y)) # gives index in global path of the collision point.
            # print("obs_ind =",self.obs_ind)

            # print("obs_ind + collision_distance =", self.obs_ind + collision_distance)
            # print("len(self.path.poses) =",len(self.path.poses))

            start_x = np.int16(self.traj_x[self.obs_ind - collision_distance]/self.resolution)+shift_x
            start_y = np.int16(self.traj_y[self.obs_ind - collision_distance]/self.resolution)+shift_y

            try:
                goal_x = np.int16(self.traj_x[self.obs_ind + collision_distance]/self.resolution)+shift_x
                goal_y = np.int16(self.traj_y[self.obs_ind + collision_distance]/self.resolution)+shift_y
            except:
                rospy.logwarn("obs_ind + collision_distance is out of bounds for self.traj_x/y. Returning last element instead")
                goal_x = np.int16(self.traj_x[-1]/self.resolution)+shift_x
                goal_y = np.int16(self.traj_y[-1]/self.resolution)+shift_y

            xmin = np.int16(np.clip(orig_x - plan_width/2,0,self.global_width))
            xmax = np.int16(np.clip(orig_x + plan_width/2,0,self.global_width))
            ymin = np.int16(np.clip(orig_y - plan_height/2,0,self.global_height))
            ymax = np.int16(np.clip(orig_y + plan_height/2,0,self.global_height))

            final_width = xmax - xmin - 1
            final_height = ymax - ymin - 1

            map_matr = self.map_matrix[xmin:xmax - 1, ymin:ymax - 1]

            rospy.loginfo("Intersected path found!")

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

    def traj_to_occupgrid(self):
        gen = np.array([[
            np.int16(self.traj_x[i]/resolution)+shift_x,
            np.int16(self.traj_y[i]/resolution)+shift_y,
            np.int16(self.traj_x[i+1]/resolution)+shift_x,
            np.int16(self.traj_y[i+1]/resolution)+shift_y,
            i,]
        for i in range(len(self.traj_x)-1)],dtype=np.int32)
        self.path_lookup = np.zeros((self.global_width,self.global_height),dtype=np.int32)
        self.index_lookup = np.zeros((self.global_width,self.global_height),dtype=np.int32)
        bresenham.bres_segments_count(gen,self.path_lookup,self.index_lookup)

        # fig, (ax1,ax2) = plt.subplots(2)
        # ax1.imshow(self.path_lookup.T,origin="lower")
        # ax2.imshow(self.index_lookup.T,origin="lower")
        # plt.show()

    def callback_problem(self, problem_map):

        problem_matr = np.array(problem_map.data,dtype=np.float32).reshape(problem_map.width, problem_map.height)
        # plt.imshow(problem_matr.T,origin="lower")
        # plt.show()
        sx,sy = np.where(problem_matr==-np.int8(1))
        gx,gy = np.where(problem_matr==-np.int8(2))
        sx,sy = sx[0], sy[0]
        gx,gy = gx[0], gy[0]
        problem_matr[sx,sy] += 1
        problem_matr[gx,gy] += 2
        problem_matr += 1
        path = pyastar.astar_path(problem_matr, (sx,sy), (gx,gy), allow_diagonal=True)

        x_list = np.array((path[:,0]+problem_map.x-shift_x)*resolution,dtype=np.float32)
        y_list = np.array((path[:,1]+problem_map.y-shift_y)*resolution,dtype=np.float32)
        # print(x_list)
        # print(y_list)

        idx_1 = self.obs_ind-collision_distance
        idx_2 = self.obs_ind+collision_distance

        # print("idx_1 =", idx_1)
        # print("idx_2 =", idx_2)

        # print("traj_x =", self.traj_x)
        # print("traj_y =", self.traj_y)
        # print("Deleting elements")
        self.traj_x = np.delete(self.traj_x,slice(idx_1,idx_2))
        self.traj_y = np.delete(self.traj_y,slice(idx_1,idx_2))
        # print("traj_x =", self.traj_x)
        # print("traj_y =", self.traj_y)

        # print("Inserting elements")
        self.traj_x = np.insert(self.traj_x,idx_1,x_list)
        self.traj_y = np.insert(self.traj_y,idx_1,y_list)
        # print("traj_x =", self.traj_x)
        # print("traj_y =", self.traj_y)
        self.traj_to_occupgrid()

        new_path = lists_to_path(self.traj_x, self.traj_y)

        #new_path = lists_to_path(x_list, y_list)

        self.solution_pub.publish(new_path)
        rospy.loginfo("New global path published! Setting collision to False.")
        self.collision_pub.publish(Bool(False))
        self.rate.sleep()

    def run(self):
        rate = rospy.Rate(update_rate)

        while not rospy.is_shutdown():
            rate.sleep()

        rospy.spin()

def lists_to_path(x_list, y_list):
    # TODO: A bit unnecessary to go from path to list to path to list. Try
    # fixing this in data.py or something.
    path = Path()
    path.header.stamp = rospy.Time.now()
    path.header.frame_id = 'map'
    path.poses = []
    for i in range(len(x_list)):
        curr_pose = PoseStamped()
        curr_pose.header.frame_id = 'map'
        curr_pose.pose.position.x = float(x_list[i])
        curr_pose.pose.position.y = float(y_list[i])
        path.poses.append(curr_pose)
    return path

if __name__ == '__main__':
    try:
        node = Node()
        node.run()
    except rospy.ROSInterruptException:
        pass
