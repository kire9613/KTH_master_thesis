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

        self.grid_indices = np.zeros((width,height))

        for i in range(self.x_width):
            for j in range(self.y_width):
                self.grid_indices[i,j] = j*self.x_width + i

        self.open_set = np.zeros((4, self.x_width*self.y_width))
        self.closed_set = np.zeros((4, self.x_width*self.y_width))

        self.motions = np.array([[1, 0, 1],
                                [0, 1, 1],
                                [-1, 0, 1],
                                [0, -1, 1],
                                [-1, -1, np.sqrt(2)],
                                [-1, 1, np.sqrt(2)],
                                [1, -1, np.sqrt(2)],
                                [1, 1, np.sqrt(2)]])

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

        start_node = np.array([sx, sy, 0.0, -1])
        goal_node = np.array([gx, gy, 0.0, -1])

        self.open_set[2,:] = np.inf
        self.open_set[3,:] = 0
        self.open_set[:, self.grid_indices[sx, sy]] = start_node

        self.closed_set[2,:] = np.inf
        self.closed_set[3,:] = 0
        self.closed_set[:, self.grid_indices[gx, gy]] = goal_node

        while 1:

            w = 1.0 #Heuristic weight
            result = self.open_set[2,:] + w*np.hypot(gx - self.open_set[0,:], gy - self.open_set[1,:])
            c_id = np.argmin(result)
            current = self.open_set[:,c_id]

            if current[2] == np.inf:
                print("Open set i empty")
                break

            # show graph
            if show_animation:  # pragma: no cover
                draw_obj.point((current[0], current[1]), node_RGB)

            if current[0] == gx and current[1] == gy:
                print("Found goal")
                self.closed_set[2,self.grid_indices[gx, gy]] = current[2] #cost
                self.closed_set[3,self.grid_indices[gx, gy]] = current[3] #parent index
                break

            # Remove the item from the open set
            self.open_set[:, c_id] = np.inf

            self.closed_set[:, c_id] = current

            # expand_grid search grid based on motion model
            for i in range(self.motions.shape[0]):
                node = np.array([current[0] + self.motions[i,0],
                                current[1] + self.motions[i,1],
                                current[2] + self.motions[i,2],
                                c_id])

                next_id = self.grid_indices[node[0], node[1]]

                # If the node is not safe, do nothing

                if self.verify(node[0], node[1], obstacle_map) == False:
                    continue

                if self.closed_set[2,next_id] != np.inf:
                    continue

                if self.open_set[2,next_id] == np.inf:  # discovered a new node
                    self.open_set[:,next_id] = node

                else:
                    if self.open_set[2,next_id] > node[2]: # This path is the best until now
                        self.open_set[:,next_id] = node

        rx, ry = [goal_node[0]], [goal_node[1]]
        parent_index = goal_node[3]
        while parent_index != -1:
            rx.append(self.closed_set[0,parent_index])
            ry.append(self.closed_set[1,parent_index])
            parent_index = self.closed_set[3,parent_index]

        if show_animation:
            for i in range(len(rx)):
                draw_obj.point((rx[i], ry[i]), path_RGB)
            image.save(img_file_name)
            print("Tried to save image")

        rx.reverse()
        ry.reverse()

        return ry, rx

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
