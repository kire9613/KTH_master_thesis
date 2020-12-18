"""
RRT_2D
@author: huiming zhou
"""

import os
import sys
import math
import numpy as np

import utils, smoothing


class Node:
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None

class Rrt:
    def __init__(self, s_start, s_goal, step_len, goal_sample_rate, iter_max, obstacles, grid_data):
        self.s_start = Node(s_start)
        self.s_goal = Node(s_goal)
        self.step_len = step_len
        self.goal_sample_rate = goal_sample_rate
        self.iter_max = iter_max
        self.vertex = [self.s_start]

        self.utils = utils.Utils(obstacles, grid_data)
        self.obstacles = obstacles

        self.x_range = (-15, 21)

        self.k = -0.095
        l1_s = self.k*self.s_start.x + 9
        l2_s = self.k*self.s_start.x + 4.2
        l3_s = self.k*self.s_start.x + 4.2
        l4_s = self.k*self.s_start.x - 1.3

        l1_g = self.k*self.s_goal.x + 9
        l2_g = self.k*self.s_goal.x + 4.2
        l3_g = self.k*self.s_goal.x + 4.2
        l4_g = self.k*self.s_goal.x - 1.3

        if l4_s <= self.s_start.y <= l3_s and l4_g <= self.s_goal.y <= l3_g:
           self.y_range = 43

        elif l2_s <= self.s_start.y <= l1_s and l2_g <= self.s_goal.y <= l1_g:
           self.y_range = 21

        else:
           self.y_range = 41

    def planning(self):
        for i in range(self.iter_max):
            node_rand = self.generate_random_node(self.goal_sample_rate)
            node_near = self.nearest_neighbor(self.vertex, node_rand)
            node_new = self.new_state(node_near, node_rand)

            if node_new and not self.utils.is_collision(node_near, node_new):
                self.vertex.append(node_new)
                dist, _ = self.get_distance_and_angle(node_new, self.s_goal)

                if dist <= self.step_len:
                    self.new_state(node_new, self.s_goal)
                    print(i)
                    return self.extract_path(node_new)

        return None

    def generate_random_node(self, goal_sample_rate):
        delta = self.utils.delta

        if np.random.random() > goal_sample_rate:

            rand_x = np.random.uniform(self.x_range[0], self.x_range[1])

	    if self.y_range == 43:
	      y_max = self.k*rand_x + 4.2
              y_min = self.k*rand_x - 1.3

	    elif self.y_range == 21:
	      y_max = self.k*rand_x + 9
              y_min = self.k*rand_x + 4.2

	    else:
	      y_max = self.k*rand_x + 9
              y_min = self.k*rand_x - 1.3

            rand_y = np.random.uniform(y_min, y_max)
            return Node((rand_x, rand_y))

        return self.s_goal

    @staticmethod
    def nearest_neighbor(node_list, n):
        return node_list[int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y)
                                        for nd in node_list]))]

    def new_state(self, node_start, node_end):
        dist, theta = self.get_distance_and_angle(node_start, node_end)

        dist = min(self.step_len, dist)
        node_new = Node((node_start.x + dist * math.cos(theta),
                         node_start.y + dist * math.sin(theta)))
        node_new.parent = node_start

        return node_new

    def extract_path(self, node_end):
        path = [(self.s_goal.x, self.s_goal.y)]
        node_now = node_end

        while node_now.parent is not None:
            node_now = node_now.parent
            path.append((node_now.x, node_now.y))
	
        path.reverse()

        return path

    @staticmethod
    def get_distance_and_angle(node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx, dy), math.atan2(dy, dx)


def compute_path(x0, y0, xt, yt, obstacles, grid_data):
    
    x_start = (x0, y0)
    x_goal = (xt, yt)
    
    rrt = Rrt(x_start, x_goal, 0.4, 0.05, 20000, obstacles, grid_data)
    path = rrt.planning()

    if path:
        path = smoothing.smooth(path)

        traj_x = []
        traj_y = []

        print("Path found")

	for p in path:
          traj_x.append(p[0])
	  traj_y.append(p[1])

    else:
        print("No Path Found!")
        return None, None

    return traj_x, traj_y





