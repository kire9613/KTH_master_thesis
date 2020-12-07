#!/usr/bin/env python
"""
    @by Johan Hedin Team1
"""
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

import svea.pyastar.pyastar as pyastar

# SVEA
from svea_msgs.msg import VehicleState

# OTHER
from matrix_astar3 import AStarPlanner

## COLLISION NODE PARAMS ######################################################
update_rate = 1
timeout_rate = 0.1
resolution = 0.05
splice_tol = 0.5
width=879
height=171

oob_delimiter = max(width,height) + 1
###############################################################################

class Node:
    """
    asd
    """
    def __init__(self):

        rospy.init_node('collision_node')

        self.solution_pub = rospy.Publisher('trajectory_updates', Path, queue_size=1, latch=True)

        self.problem_sub = rospy.Subscriber('/problem_map', OccupancyGridUpdate, self.callback_problem)

        self.path_sub = rospy.Subscriber('/path_plan', Path, self.callback_path)
        # self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.callback_map)

        self.global_path = Path()

        self.rate_timeout = rospy.Rate(timeout_rate)

        self.planner = None

    def run(self):

        rate = rospy.Rate(update_rate)

        while not rospy.is_shutdown():
            rate.sleep()

        rospy.spin()

    def callback_path(self, path):
        self.global_path = path

    # def callback_map(self, occ_map):
    #     self.planner = AStarPlanner(np.asarray(occ_map.data))

    def callback_problem(self, problem_map):

        problem_matr = np.array(problem_map.data,dtype=np.float32).reshape(problem_map.width, problem_map.height)
        sx,sy = np.where(problem_matr==-np.int8(1))
        gx,gy = np.where(problem_matr==-np.int8(2))
        sx,sy = sx[0], sy[0]
        gx,gy = gx[0], gy[0]
        problem_matr[sx,sy] += 1
        problem_matr[gx,gy] += 2
        problem_matr += 1
        path = pyastar.astar_path(problem_matr, (sx,sy), (gx,gy), allow_diagonal=True)

        x_list = (path[:,0]+problem_map.x)*resolution
        y_list = (path[:,1]+problem_map.y)*resolution
        # print(x_list)
        # print(y_list)

        # TODO: OLD CODE speed this up later...

        x_new_global = []
        y_new_global = []

        n = len(self.global_path.poses)
        # print("Path length: {0}".format(n))
        i = 0
        while 1:
            x = self.global_path.poses[i].pose.position.x
            y = self.global_path.poses[i].pose.position.y
            x_new_global.append(x)
            y_new_global.append(y)

            dist = np.sqrt((x_list[0] - x)**2 + (y_list[0] - y)**2)

            if dist < splice_tol:
                x_new_global.extend(x_list)
                y_new_global.extend(y_list)
                break
            i += 1

        i = n - 1
        while 1:
            x = self.global_path.poses[i].pose.position.x
            y = self.global_path.poses[i].pose.position.y

            dist = np.sqrt((x_list[len(x_list) - 1] - x)**2 + (y_list[len(y_list) - 1] - y)**2)

            if dist < splice_tol:
                for j in range(i, n):
                    x_new_global.append(self.global_path.poses[j].pose.position.x)
                    y_new_global.append(self.global_path.poses[j].pose.position.y)
                break
            i -= 1

        new_path = lists_to_path(x_new_global, y_new_global)

        #new_path = lists_to_path(x_list, y_list)

        self.solution_pub.publish(new_path)
        self.rate_timeout.sleep()


    def callback_problemOLD(self, problem_map):

        while self.planner == None:
            self.rate_timeout.sleep()

        obstacles = []

        problem_matr = np.asarray(problem_map.data).reshape(problem_map.width, problem_map.height)

        for i in range(0, problem_map.width):
            for j in range(0, problem_map.height):
                if problem_matr[i,j] >= 75:
                    obstacles.append((problem_map.x + i, problem_map.y + j))
                if problem_matr[i,j] == -np.int8(1):
                    sx = problem_map.x + i
                    sy = problem_map.y + j
                if problem_matr[i,j] == -np.int8(2):
                    gx = problem_map.x + i
                    gy = problem_map.y + j

        print(sx,sy)
        print(gx,gy)

        y_list, x_list = self.planner.planning(obstacles, sx, sy, gx, gy)

        for i in range(len(x_list)):
            x_list[i] = x_list[i]*resolution
            y_list[i] = y_list[i]*resolution

        x_new_global = []
        y_new_global = []

        n = len(self.global_path.poses)
        print("Path length: {0}".format(n))
        i = 0
        while 1:
            x = self.global_path.poses[i].pose.position.x
            y = self.global_path.poses[i].pose.position.y
            x_new_global.append(x)
            y_new_global.append(y)

            dist = np.sqrt((x_list[0] - x)**2 + (y_list[0] - y)**2)

            if dist < splice_tol:
                x_new_global.extend(x_list)
                y_new_global.extend(y_list)
                break
            i += 1

        i = n - 1
        while 1:
            x = self.global_path.poses[i].pose.position.x
            y = self.global_path.poses[i].pose.position.y

            dist = np.sqrt((x_list[len(x_list) - 1] - x)**2 + (y_list[len(y_list) - 1] - y)**2)

            if dist < splice_tol:
                for j in range(i, n):
                    x_new_global.append(self.global_path.poses[j].pose.position.x)
                    y_new_global.append(self.global_path.poses[j].pose.position.y)
                break
            i -= 1

        new_path = lists_to_path(x_new_global, y_new_global)

        #new_path = lists_to_path(x_list, y_list)

        self.solution_pub.publish(new_path)
        self.rate_timeout.sleep()

def lists_to_path(x_list, y_list):
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