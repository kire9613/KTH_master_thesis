#!/usr/bin/env python2
# -*- coding: utf-8 -*-
# {Astrid Lindstedt, November}
# {930701-1982}
# {astridli@kth.se}

import sys
import os
import random
import rospy
import numpy as np
import pickle
import matplotlib.pyplot as plt

from math import hypot, sqrt
#from map.occupancy_grid import *
from sync_occupsancygrid import SyncOcc
#from map.ros_interface import ROSInterface as MapROSInterface
from nav_msgs.msg import OccupancyGrid


dirname = os.path.dirname(__file__)
svea_core = os.path.join(dirname, '../../')
map_name = "floor2_rot" # change this for different maps
file_path = svea_core + 'scripts/core_examples/' + map_name + ".pickle"

import svea.models.bicycle

MODEL = svea.models.bicycle.SimpleBicycleModel()

update_rate = 5 # [Hz]

frame_id = "map" 
resolution = 0.05 # The map resolution [m/cell]
width = 879 
height = 171
#origin = [-17.581444, -22.876441]
x = 13.5
y = 1.54
targetx = 36.3
targety = 1.44
fig, ax = plt.subplots(1)
plt.ion()
plt.show()

class Node():

    def __init__(self, x, y, phi=None, theta=None, time=None, parent=None):
        self.x = x
        self.y = y
        self.phi = phi # control
        self.theta = theta 
        self.parent = parent
        self.time = time
        self.cost = 0.0


class MAPP():

    def __init__(self):
       	self.map = OccupancyGrid()
        self.frame_id = frame_id
        self.resolution = resolution
        self.width = width
        self.height = height
        self.map_matrix = np.full((width, height), np.inf)
        self.xt = targetx # xtarget
        self.yt = targety # ytarget

    

    def run(self): # behover jag denna??? 

		rate = rospy.Rate(update_rate)

		while not rospy.is_shutdown():
			rate.sleep()
		
		rospy.spin()


def solution(car):

    global step_length, times, NodeList, countis, TTR_set
    done = False
    
    pkl_file = open(file_path, 'rb')
    obsticle = pickle.load(pkl_file)
    ob_array =  np.array(obsticle.data)

    MAPP.map_matrix = np.reshape(ob_array,(width, height), order='F')

    obslist = get_obs(MAPP.map_matrix)

    car.obs = obslist

    i = 0
    x = 13.5
    y = 1.54
    timestart = 0
    phistart = 0
    thetastart = 0
    step_length = 20
    countis = 0

    start = Node(x, y, phistart, thetastart, timestart)
    NodeList = [start]
    
    count = 2000
    first_step = 10

    # one big step in the beginning.

    for i in range(first_step):
    #    x, y, theta__ = step(car, x, y, thetastart, phistart)
        x = x + np.cos(phistart)*0.1
        y = y + np.sin(phistart)*0.1
        theta__ = thetastart 
    curr_node = Node(x, y, phistart, theta__, (start.time + first_step * 0.01), start)
    NodeList.append(curr_node)

 #   print('here is first in nodelist')
 #   print((start.x, start.y))
 #   print('here is first curr_node')
 #   print((curr_node.x, curr_node.y))


    #

    for j in range(count):

        # THETA = BILENSVINKEL GENTE SYSTEMET
      #  print('here is random p')
        random_point = get_random_position(car)
     #   print(random_point.x, random_point.y)
        nearest_node = GetNearestListNode(NodeList, random_point) # brings the best parent to the random point
    #    print('here is nearest node')
    #    print(nearest_node.x, nearest_node.y)
        # hämtar ut bästa barnet
        New_nod = get_best_child(car, nearest_node, random_point)

        if New_nod: # Om den inte är nonetype

            xn = New_nod.x
            yn = New_nod.y
            NodeList.append(New_nod)

            ax.plot(New_nod.x, New_nod.y, 'o')
            plt.draw()
            plt.plot(x, y, "xr")
            plt.plot(targetx, targety, "xr")
            plt.axis([10, 40, -2, 10])
            plt.grid(True)
            plt.pause(0.0001)
            done = True if ((targetx - xn) ** 2 + (targety - yn) ** 2) ** 0.5 < 0.5 else False
            closedone = True if ((targetx - xn) ** 2 + (targety - yn) ** 2) ** 0.5 < 5 else False
            if closedone:
                print('JÄVLIGT NÄRA')
                pass

        if done:
            break
# Perhaps dont need this one! 
    X, Y = gets_path(NodeList[-1])
#    print(type(NodeList))
#    print(NodeList[0].x)
    #return [i.x for i in NodeList], [i.y for i in NodeList]
    return X, Y

def gets_path(barn): 

    X = []
    Y = []
    tid = []
    while barn.parent != None:

        for i in range(step_length):
            X.append(barn.x)
            Y.append(barn.y)
            tid.append(round(barn.time, 3))
            barn.time -= 0.01
        barn = barn.parent
    tid.append(0)
    tid.reverse()
    X.reverse()
    Y.reverse()

    return X, Y

# Denna tar fram  noden som är närmast randompunkten jag tagit fram
def GetNearestListNode(nodeList, rnd):
   
    mindist = 10000
#    print('here is nodelist 0')
#    print((nodeList[1].x, nodeList[1].y))
    for i in range(len(nodeList)):
        x_list = nodeList[i].x
        y_list = nodeList[i].y

        dist = ((x_list - rnd.x) ** 2 + (y_list - rnd.y) ** 2) ** 0.5

        if dist <= mindist:
            mindist = dist
            j = i
    return nodeList[j]


# functions that lists all nodes with obsticles from the matrix that says a value == 100 if obstacle
def get_obs(obs_matrix): 
    obslist = []
    for n in range(200, 878):
        for m in range(0, 170):
            value = obs_matrix[n, m]

            if value > 10:
                obslist.append([n*0.05, m*0.05])
                ax.plot(n*0.05, m*0.05, 'x')
                #plt.draw()
                #plt.pause(0.0001)
                #hold(True)
    return obslist


    

def safe_or_not(car, node):
    answer_is = True

    for n in range(len(car.obs)):
        x_ = car.obs[n][0] # Mittpunkt
        y_ = car.obs[n][1]
        radius = 0.5 # ökar radien!

        band_low_x = x_ - radius # nedre xdelen av hindret
        band_upp_x = x_ + radius # övre xdelen av hindret
        band_low_y = y_ - radius # nedre ydelen av hindretssssssss
        band_upp_y = y_ + radius # övre ydelen av hindret

        # Kollar om nära ett hinder
        if (node.x <= band_upp_x) and (node.x >= band_low_x) and (node.y >= band_low_y) and (node.y <= band_upp_y):
            answer_is = False
            #print('close to hinder!')
            break

        # Kollar om nära en vägg
        if (node.x <= 0) or (node.x >= width ) or (node.y <= 0) or (node.y >= height):
            answer_is = False
          #  print('close to wall!')
            break

    return answer_is




def get_best_child(car, nearest_node, rnd):

    dist_shortest = 10000
    curr_node = None
    phi_val = [-MODEL.DELTA_MAX, -MODEL.DELTA_MAX/2, 0, MODEL.DELTA_MAX/2, MODEL.DELTA_MAX]
    for k in range(len(phi_val)):
        phi = phi_val[k]
        x = nearest_node.x
        y = nearest_node.y
        theta = nearest_node.theta

        for i in range(step_length):

            #print('before step =',(x, y))
            x, y, theta = step(car, x, y, theta, phi)
            #print('after step =',(xny, yny))
            check = Node(x,y)

            safe = safe_or_not(car, check)

            if not safe:
            #    print('not safe!!!!')
                break
            #else:
            #    print('SAFEEEEE')
        dist = sqrt((x - rnd.x) ** 2 + (y - rnd.y) ** 2)
        if dist < dist_shortest and safe:

            dist_shortest = dist

            curr_node = Node(x, y, phi, theta, (nearest_node.time + step_length * 0.01), nearest_node)
            #print('Safe and shortest distance =', curr_node.x, curr_node.y)


    # This is if none of the phi values are ok, then we need to choose another parent?
    if curr_node == None: 
        NodeList.remove(nearest_node)
        return

    return curr_node


def step(car, x, y, theta, phi, dt=0.02): # get from another function! 

    dx     = np.cos(theta)
    dy     = np.sin(theta)
    dtheta = np.tan(phi)

    # new state (forward Euler integration)
    xn     = x     + dt*dx
    yn     = y     + dt*dy
    thetan = theta + dt*dtheta

    return xn, yn, thetan


def get_random_position(car):
    global countis
    countis += 1

    if countis < 3:

        rnd = [random.uniform(0, width * resolution -0.5), random.uniform(1, height * resolution -0.5)]
        random_p = Node(rnd[0], rnd[1])

    else:
        random_p = Node(targetx, targety)
        countis = 0

    return random_p
