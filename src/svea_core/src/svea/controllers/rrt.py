#!/usr/bin/env python

from math import *
import random

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y

def crash(x, y, obstacles):

    margin = 0.2

    #if x >= car.xub or x <= car.xlb or y >= car.yub or y <= car.ylb:
    #    return True

    #else:
    for obj in obstacles:

      if hypot(obj[0]-x, obj[1]-y) <= margin:
        return True

    return False

def nearmest_node(nodes, random_point):

    distances = []

    for node in nodes:
        distance = hypot(node.x-random_point[0], node.y-random_point[1])
        distances.append(distance)

    i = distances.index(min(distances))
    node_nearmest = nodes[i]

    return node_nearmest

def solution(x0, y0, xt, yt, simulator):

    obstacles = simulator.simulated_lidar.obstacles()

    nodes = []
    phis = [-pi/4, 0, pi/4]
    
    x, y = x0, y0

    start_node = Node(x, y)
    nodes.append(start_node)

    traj_x = []
    traj_y = []

    while True:

        random_x = random.uniform(car.xlb, car.xub)
        random_y = random.uniform(car.ylb, car.yub)

        node_nearmest = nearmest_node(nodes, [random_x,random_y])

        states = []
        distances = []

        for phi in phis:

            x, y = node_nearmest.x, node_nearmest.y

            for steps in range(0, 25):

                #x, y, theta = step(car, x, y, theta, phi)

		x = x + cos(phi)
                y = y + sin(phi)

                if crash(x, y, obstacles) == True:
                    break

            if crash(x, y, obstacles) == False:
                distance = hypot(x-random_x, y-random_y)
                distances.append(distance)
                states.append([x, y])


        if len(distances) == 0:
            continue

        i = distances.index(min(distances))
        state = states[i]

        new_node = Node(state[0], state[1])
        nodes.append(new_node)

        if hypot(new_node.x-xt, new_node.y-yt) < 1.25:
            break

    for node in nodes:
	traj_x.append(node.x)
	traj_y.append(node.y)

    return traj_x, traj_y




