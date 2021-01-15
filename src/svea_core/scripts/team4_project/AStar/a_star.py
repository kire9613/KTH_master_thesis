#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# {Oscar Gustavsson}
# {ogust@kth.se}

from dublins import *
from Queue import PriorityQueue
from math import hypot, pi, ceil
import numpy

DEBUG = False

if DEBUG:
    import ass3_debug as dbg

N_GRID_X = 40       # For debug plot
N_GRID_Y = N_GRID_X//2  # For debug plot
GOAL_RADIUS = 1.5   # radius of goal point (if inside == at goal)
GOAL_ANGLE = 0      # heading at goal point
SAMPLE_TIME = 0.01  
ANGLES = [-pi/4,-pi/8,0,pi/8,pi/4]  # discretized directions
N_HEADINGS = 6    
N_STEPS = None      # number of steps in one direction, initialized later
GRID_SIZE_X = 0     # grid dimension, initialized later 
GRID_SIZE_Y = 0     # grid dimension, initialized later
OBJECTIVE = []      # definition of task, initialized later

plotter = None
counter = 0

class Node:
    """
    Nodes in A* search-treee
    """

    def __init__(self, theta, x, y, cost, parent, control):
        '''control is control signal applied to get to this node'''

        self.xd = int(x/GRID_SIZE_X) # discretized cord.
        self.yd = int(y/GRID_SIZE_Y)
        self.theta = theta # heading
        self.x = x
        self.y = y
        self.cost = cost # cost to go from root
        self.parent = parent
        self.control = control

        # Discretize heading

        # Make sure theta is in [0, 2π)
        while theta <= 0:
            theta += 2*pi
        while theta > 2*pi:
            theta -= 2*pi

        # discretizise heading
        self.thetad = None
        for i in range(N_HEADINGS):
            if theta < (i+1)*2*pi/N_HEADINGS:
                self.thetad = i
                break
        if self.thetad is None:
            self.thetad = N_HEADINGS-1

    def __lt__(self, other):
        return False

def heur(posx, posy, goalx, goaly):
    return hypot(posx-goalx, posy-goaly)

def isClosed(closedSet, n):
    """
    True if node n is dead
    """
    return (n.xd, n.yd, n.thetad) in closedSet

def closeNode(closedSet, n):
    """ Set node dead if all paths from node is explored """
    closedSet[(n.xd, n.yd, n.thetad)] = True

def searchQueue(openSet, node):
    '''Converts queue to list and returns the index of node
    if it exists and -1 otherwise.'''

    index = -1
    i = 0
    nodes = []
    while not openSet.empty():
        estCost, n = openSet.get()
        nodes.append((estCost, n))
        if n.xd == node.xd and n.yd == node.yd and n.thetad == node.thetad:
            index = i
        i += 1

    return index, nodes

def isObstacle(obj, n):
    """ Check for collisions """

    status = obj._environment.safe(n.x,n.y)
    print("safe status: ")
    print(status)
    if status:
        return False
    else:
        return True

def isObstacle_2(obj, x,y):
    """ Check for collisions """

    status = obj._environment.safe(x,y)
    print("safe status: ")
    print(status)
    if status:
        return False
    else:
        return True

def updateNeighbors(obs, openSet, closedSet, n):
    """
    Create and add new nodes in A* search tree
    Do collision check
    """

    print("updateNeighbors()")
    for angle in ANGLES:
        xn, yn, thetan = n.x, n.y, n.theta
        safe = True
        for i in range(N_STEPS):
            xnew, ynew, thetan = step(obs, xn, yn, thetan, angle, DELTA_T)
            """ check all nodes inbetween """
            if isObstacle_2(obs,xnew,ynew):
                safe = False
                break
            xn = xnew
            yn = ynew
        cost = 1 if angle == 0 else 1.5#1.1
        nn = Node(thetan, xn, yn, n.cost+cost, n, angle)

        if not isClosed(closedSet, nn):
            if isObstacle(obs, nn) or not safe:
                # Node in unusable path
                closeNode(closedSet, nn)
                if DEBUG:
                    plotter.addNode(nn)
                    plotter.closeNode(nn)
                continue

            i, nodes = searchQueue(openSet, nn)
            nnPriority = (nn.cost + 3*heur(nn.x, nn.y, obs.xt, obs.yt), nn)
            # Save node
            if i != -1:
                if nn.cost < nodes[i][1].cost:
                    if DEBUG:
                        plotter.updateNode(nodes[i][1], nn)
                    nodes[i] = nnPriority
            else:
                nodes.append(nnPriority)
                if DEBUG:
                    plotter.addNode(nn)

            # Rebuild queue
            for node in nodes:
                openSet.put(node)

def run_astar(objective, smooth=False):
    global GRID_SIZE_X, GRID_SIZE_Y, N_STEPS, OBJECTIVE, DELTA_T, plotter, counter

    openSet = PriorityQueue() # Set holding active nodes to search more path from
    closedSet = {} # Set holding dead nodes

    controls = []
    times = []

    GRID_SIZE_X = objective._environment.resolution
    GRID_SIZE_Y = objective._environment.resolution

    OBJECTIVE = objective # defines task to solve

    if DEBUG:
        dbg.plotMap(objective, N_GRID_X, N_GRID_Y, OBJECTIVE)
        dbg.show(objective)

    N_STEPS = 10
    DELTA_T = 0.040 #dt_max = 0.05 given max_vel = 1 m/s
    GOAL_RADIUS = 0.1   # Accepted deviation from goal position
    GOAL_ANGLE = pi/6   # Accepted deviation from goal theta

    if DEBUG:
        plotter = dbg.TreePlot(objective._environment, N_STEPS)

    openSet.put((heur(objective.x0, objective.y0, objective.xt, objective.yt), Node(objective.theta0, objective.x0, objective.y0, 0, None, 0)))

    print("Searching for path...")
    while not openSet.empty():
        est_cost, n = openSet.get()
        closeNode(closedSet, n)
        if n.parent is not None:
            if DEBUG:
                plotter.closeNode(n)

        # We have reached the goal
        if hypot(n.x-objective.xt, n.y-objective.yt) < GOAL_RADIUS and numpy.fabs(n.theta - objective.thetat) < GOAL_ANGLE:
            if DEBUG:
                plotter.markBestPath(n)
            # Reconstruct control signals
            nodes_in_path = []
            while n.parent is not None:
                controls.append(n.control)
                nodes_in_path.append((n.x,n.y))
                n = n.parent
            controls.reverse()
            nodes_in_path.reverse()

            for i in range(len(controls)+1):
                times.append(i*N_STEPS*SAMPLE_TIME)

            print("Path found!")
            break
        else:
            # Seach paths from node
            updateNeighbors(objective, openSet, closedSet, n)

    if not controls:
        controls = [0]
        times = [0, SAMPLE_TIME]
        nodes_in_path = []
        print("Path not found!")

    if DEBUG:
        dbg.wait()

    if smooth:
        nodes_in_path = smooth_path(objective._environment,nodes_in_path)

    return nodes_in_path #controls, times

def smooth_path(environment, path):
    new_path = []

    ok_i = [1]
    new_path_exists = False

    for i in range(1,len(path)):
        path_ok = True
        dx = abs(path[0][0]-path[i][0])
        dy = abs(path[0][1]-path[i][1])
        if dx > dy:
            xv = numpy.linspace(path[0][0], path[i][0], num=ceil(dx/0.05))
            yv = numpy.linspace(path[0][1], path[i][1], num=len(xv))
        else:
            yv = numpy.linspace(path[0][1], path[i][1], num=ceil(dy/0.05))
            xv = numpy.linspace(path[0][0], path[i][0], num=len(yv))
        if 1 < len(xv):
            for k in range(len(xv)):
                if not environment.safe(xv[k],yv[k]):
                    path_ok = False

        if path_ok:
            new_path_exists = True
            ok_i.append(i)


    if ok_i[-1] == len(path)-1:
        # insert first node
        p = (path[0][0],path[0][1])
        new_path.append(p)

        # insert next node
        p = (path[ok_i[-1]][0],path[ok_i[-1]][1])
        new_path.append(p)

        return new_path
    elif ok_i[-1]+1 == len(path)-1:
        # insert first node
        p = (path[0][0],path[0][1])
        new_path.append(p)

        # insert next node
        p = (path[ok_i[-1]][0],path[ok_i[-1]][1])
        new_path.append(p)

        # add last node
        p = (path[-1][0],path[-1][1])
        new_path.append(p)

        return new_path
    else:
        # insert first node
        p = (path[0][0],path[0][1])
        new_path.append(p)

        new_path += smooth_path(environment,path[ok_i[-1]:-1])

    return new_path
