#!/usr/bin/env python

"""
A* grid planning
author: Atsushi Sakai(@Atsushi_twi)
        Nikos Kanargias (nkana@tee.gr)
See Wikipedia article (https://en.wikipedia.org/wiki/A*_search_algorithm)
"""

import math
import os
import pickle
import numpy as np
import matplotlib.pyplot as plt

from PIL import Image, ImageDraw

img_file_name = os.path.dirname(os.path.abspath(__file__)) + os.sep + "astar.png"
space_RGB = (255, 255, 255)# White
node_RGB = (0, 0, 255)     # Blue
obstacle_RGB = (0, 0, 0)   # Black
path_RGB = (255, 0, 0)     # Red
start_RGB = (0, 255, 0)    # Green
goal_RGB = (255, 255, 0)   # Yellow

show_animation = True

dirname = os.path.dirname(__file__)
svea_core = os.path.join(dirname, '../../')
map_name = "problem_map_np" # change this for different maps
file_path = svea_core + 'resources/maps/' + map_name + ".pickle"

width = 879 #xmax
height = 171 #ymax

occupied_space = np.int8(100) #black

class AStarPlanner:

    def __init__(self, init_map):
        """
        Initialize grid map for a star planning
        """
        self.motion = self.get_motion_model()

        init_map = init_map.reshape(height, width)

        self.x_width = width
        self.y_width = height

        self.obstacle_map = np.full((self.x_width, self.y_width), 0, dtype=np.uint8)

        for x in range(self.x_width):
            for y in range(self.y_width):
                if init_map[y,x] == occupied_space:
                    self.obstacle_map[x,y] = np.uint8(1)
                    for r in range(1, 4):
                        t = 0
                        while t <= 2*np.pi:
                            a = x + r*np.cos(t)
                            b = y + r*np.sin(t)
                            a = int(a)
                            b = int(b)
                            if 0 <= a < self.x_width and 0 <= b < self.y_width:
                                self.obstacle_map[a,b] = np.uint8(1)
                            t = t + np.pi/32
        """
        self.grid_indices = np.zero((width,height))

        for i in range(self.x_width):
            for j in range(self.y_width):
                self.grid_indices[i,j] = j*self.x_width + i
        """

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index

    def planning(self, obstacles, sx, sy, gx, gy):
        if show_animation:
            #Draw images of the planning
            image = Image.new('RGB', (width, height))
            draw_obj = ImageDraw.Draw(image)
            image.putdata([space_RGB]*(height*width))

        obstacle_map = self.obstacle_map

        for entry in obstacles:
            obstacle_map[entry] = np.uint8(1)
            for r in range(1, 10):
                    t = 0
                    while t <= 2*np.pi:
                        a = entry[0] + r*np.cos(t)
                        b = entry[1] + r*np.sin(t)
                        a = int(a)
                        b = int(b)
                        if self.verify(a,b, obstacle_map):
                            obstacle_map[a,b] = np.uint8(1)
                        t = t + np.pi/32

        if show_animation:
            for x in range(self.x_width):
                for y in range(self.y_width):
                    if obstacle_map[x,y] == 1:
                        draw_obj.point((x, y), obstacle_RGB)
            draw_obj.point((sx, sy), start_RGB)
            draw_obj.point((gx, gy), goal_RGB)

        start_node = self.Node(sx, sy, 0.0, -1)
        goal_node = self.Node(gx, gy, 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        while 1:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            c_id = min(open_set, key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node, open_set[o]))
            current = open_set[c_id]

            # show graph
            if show_animation:  # pragma: no cover
                draw_obj.point((current.x, current.y), node_RGB)

            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                    current.y + self.motion[i][1],
                                    current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify(node.x, node.y, obstacle_map):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)

        if show_animation:
            for i in range(len(rx)):
                draw_obj.point((rx[i], ry[i]), path_RGB)
            image.save(img_file_name)
            print("Tried to save image")

        rx.reverse()
        ry.reverse()

        return ry, rx

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [goal_node.x], [goal_node.y]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(n.x)
            ry.append(n.y)
            parent_index = n.parent_index

        return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_index(self, node):
        return (node.y) * self.x_width + (node.x)

    def verify(self, x, y, obstacle_map):
        if x < 0:
            return False
        elif y < 0:
            return False
        elif x >= self.x_width:
            return False
        elif y >= self.y_width:
            return False

        # collision check
        if obstacle_map[x, y] == np.uint8(1):
            return False

        return True

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                    [0, 1, 1],
                    [-1, 0, 1],
                    [0, -1, 1],
                    [-1, -1, math.sqrt(2)],
                    [-1, 1, math.sqrt(2)],
                    [1, -1, math.sqrt(2)],
                    [1, 1, math.sqrt(2)]]

        return motion
"""
def main():
    with open(file_path, 'rb') as f:
        pickled_problem = np.load(f)

    a_star = AStarPlanner(pickled_problem)
    rx, ry = a_star.planning()

    for i in range(len(rx)):
        print(rx[i],ry[i])

if __name__ == '__main__':
    main()
"""
