#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = "Team1"

# Python standard library
import numpy as np
# ROS
import rospy
import message_filters

# ROS messages
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import Path
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Bool, Int16MultiArray

import svea.pyastar.pyastar as pyastar

# SVEA
from svea_msgs.msg import VehicleState

# OTHER
from matrix_astar3 import AStarPlanner

import matplotlib.pyplot as plt
from math import floor

## COLLISION NODE PARAMS ######################################################
update_rate = 2
timeout_rate = 0.1
splice_tol = 0.5
width = 635 #1269
height = 284 #567
resolution = 0.1

shift_x = int(floor(30.549770/resolution))
shift_y = int(floor(11.414917/resolution))

oob_delimiter = max(width,height) + 1
###############################################################################
#                                   FLOOR 2                                   #
###############################################################################
update_rate = 2 # [Hz]
width = 879
height = 171
resolution = 0.05
shift_x = 0
shift_y = 0
oob_delimiter = max(width,height) + 1
###############################################################################

class Node:
    """
    asd
    """
    def __init__(self):

        rospy.init_node('collision_node')

        self.solution_pub = rospy.Publisher('trajectory_updates', Path, queue_size=1, latch=True)
        self.collision_pub = rospy.Publisher('collision', Bool, queue_size=1, latch=False)
        self.index_sub = rospy.Subscriber('problem_index', Int16MultiArray, self.callback_index)
        self.problem_sub = rospy.Subscriber('problem_map', OccupancyGridUpdate, self.callback_problem)

        self.path_sub = rospy.Subscriber('path_plan', Path, self.callback_path)
        # self.map_sub = rospy.Subscriber('map', OccupancyGrid, self.callback_map)

        self.global_path = Path()

        self.rate_timeout = rospy.Rate(timeout_rate)

        self.planner = None

    def run(self):

        rate = rospy.Rate(update_rate)

        while not rospy.is_shutdown():
            rate.sleep()

        rospy.spin()

    def callback_index(self, msg):
        self.obs_ind = msg.data[0]
        self.collision_distance = msg.data[1]

    def callback_path(self, path):
        self.global_path = path

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

        x_list = (path[:,0]+problem_map.x-shift_x)*resolution
        y_list = (path[:,1]+problem_map.y-shift_y)*resolution
        # print(x_list)
        # print(y_list)

        n = len(self.global_path.poses)
        x_new_global = [i.pose.position.x for i in self.global_path.poses]
        y_new_global = [i.pose.position.y for i in self.global_path.poses]
        idx_1 = self.obs_ind-self.collision_distance
        idx_2 = self.obs_ind+self.collision_distance
        x_new_global[idx_1:idx_2] = x_list # insert list in list at point idx_x
        y_new_global[idx_1:idx_2] = y_list # insert list in list at point idx_y

        new_path = lists_to_path(x_new_global, y_new_global)

        #new_path = lists_to_path(x_list, y_list)

        self.solution_pub.publish(new_path)
        rospy.loginfo("New global path published! Setting collision to False.")
        self.collision_pub.publish(Bool(False))
        # self.rate_timeout.sleep()

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
