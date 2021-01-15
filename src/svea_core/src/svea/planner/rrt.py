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
        self.step_len = step_len # how far away a new node can be
        self.goal_sample_rate = goal_sample_rate # how often we should sample a random point
        self.iter_max = iter_max # maximum iterations
        self.vertex = [self.s_start] # tree

        self.utils = utils.Utils(obstacles, grid_data) # for obstacle avoidance
        self.obstacles = obstacles

        self.x_range = (-15, 21)

        # divide the map in four different lines for faster sampling and direct path
        self.k = -0.095
        l1_s = self.k*self.s_start.x + 9
        l2_s = self.k*self.s_start.x + 4.2
        l3_s = self.k*self.s_start.x + 4.2
        l4_s = self.k*self.s_start.x - 1.3

        l1_g = self.k*self.s_goal.x + 9
        l2_g = self.k*self.s_goal.x + 4.2
        l3_g = self.k*self.s_goal.x + 4.2
        l4_g = self.k*self.s_goal.x - 1.3

        # assign range for y-axis depending on the start and goal position
        if l4_s <= self.s_start.y <= l3_s and l4_g <= self.s_goal.y <= l3_g:
           self.y_range = 43

        elif l2_s <= self.s_start.y <= l1_s and l2_g <= self.s_goal.y <= l1_g:
           self.y_range = 21

        else:
           self.y_range = 41

    def planning(self):
        for i in range(self.iter_max):
            # generate a random point
            node_rand = self.generate_random_node(self.goal_sample_rate)
            # find the nearest node in tree to the random point
            node_near = self.nearest_neighbor(self.vertex, node_rand)
            # move towards the random point from the nearest neighbor, create a new node where you end up
            node_new = self.new_state(node_near, node_rand)

            # collision avoidance
            if node_new and not self.utils.is_collision(node_near, node_new):
                self.vertex.append(node_new)
                dist, _ = self.get_distance_and_angle(node_new, self.s_goal)

                # check if we are close enough goal
                if dist <= self.step_len:
                    self.new_state(node_new, self.s_goal)
                    # execute path
                    return self.extract_path(node_new)

        return None

    def generate_random_node(self, goal_sample_rate):
        "Generate a random point in the map"

        if np.random.random() > goal_sample_rate:

            rand_x = np.random.uniform(self.x_range[0], self.x_range[1])

            # sample in the assigned range for y-axis

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
        "Given a tree of nodes and a point, it return the node that is closest to the point."

        return node_list[int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y)
                                        for nd in node_list]))]

    def new_state(self, node_start, node_end):
        "Given a start node, it moves towards the end node and return the position that it ends up in."
        
        dist, theta = self.get_distance_and_angle(node_start, node_end)

        dist = min(self.step_len, dist)
        node_new = Node((node_start.x + dist * math.cos(theta),
                         node_start.y + dist * math.sin(theta)))
        node_new.parent = node_start

        return node_new

    def extract_path(self, node_end):
        "Extracts the path from the start and goal position in the tree."

        path = [(self.s_goal.x, self.s_goal.y)]
        node_now = node_end

        while node_now.parent is not None:
            node_now = node_now.parent
            path.append((node_now.x, node_now.y))
	
        path.reverse()

        return path

    @staticmethod
    def get_distance_and_angle(node_start, node_end):
        "Calculates distance and angle between two nodes."

        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx, dy), math.atan2(dy, dx)


def compute_path(x0, y0, xt, yt, obstacles, grid_data):
    
    x_start = (x0, y0)
    x_goal = (xt, yt)
    
    # create rrt object and start planning algorithm
    rrt = Rrt(x_start, x_goal, 0.4, 0.05, 20000, obstacles, grid_data)
    path = rrt.planning()

    if path:
        # smooth path
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





