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
#from sync_occupsancygrid import SyncOcc

from nav_msgs.msg import OccupancyGrid


import svea.models.bicycle

MODEL = svea.models.bicycle.SimpleBicycleModel()

update_rate = 5 # [Hz]

frame_id = "map" 
resolution = 0.05 # The map resolution [m/cell]
width = 879 
height = 171

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

    

    def run(self): # behover jag denna??? 

		rate = rospy.Rate(update_rate)

		while not rospy.is_shutdown():
			rate.sleep()
		
		rospy.spin()


def solution(car, start, target, ax):

    global step_length, NodeList, countis, Blacklist, tick
    done = False


    i = 0
#   x = 13.5
#    y = 1.54
   # timestart = 0
   # phistart = 0
   # thetastart = 0
    step_length = 30
    countis = 0
    dist2 = -1

    NodeList = [start]
    Blacklist = []
    tick = 1
    
    count = 7000
    first_step = 10

    # one big step in the beginning.

    for i in range(first_step):
        x = start.x + np.cos(start.phi)*0.1
        y = start.y + np.sin(start.phi)*0.1
        theta__ = start.theta


    curr_node = Node(x, y, start.phi, theta__, (start.time + first_step * 0.01), start)
    NodeList.append(curr_node)



    for j in range(count):

        # THETA = BILENSVINKEL GENTE SYSTEMET
        
        closest_to_target1, dist1 = GetNearestListNode(NodeList, target)
        
        random_point = get_random_position(NodeList, dist1, dist2, target)
        dist2 = dist1
     
        nearest_node, dist = GetNearestListNode(NodeList, random_point) # brings the best parent to the random point
    
        # hämtar ut bästa barnet
        New_nod = get_best_child(car, nearest_node, random_point, ax)
        
        if New_nod: # Om den inte är nonetype

            xn = New_nod.x
            yn = New_nod.y
            NodeList.append(New_nod)

            ax.plot(New_nod.x, New_nod.y, 'o')
            plt.draw()
            plt.pause(0.0001)
            done = True if ((target.x - xn) ** 2 + (target.y - yn) ** 2) ** 0.5 < 0.55 else False
            closedone = True if ((target.x - xn) ** 2 + (target.y - yn) ** 2) ** 0.5 < 5 else False
            if closedone:
                #print('JÄVLIGT NÄRA')
                pass

        if done:
            break

    X, Y, THETA = gets_path(NodeList[-1])

    return X, Y, THETA

def gets_path(barn): 

    X = []
    Y = []
    THETA = []

    while barn.parent != None:

        X.append(barn.x)
        Y.append(barn.y)
        THETA.append(barn.theta)

        barn = barn.parent

    X.reverse()
    Y.reverse()
    THETA.reverse()

    return X, Y, THETA

# Denna tar fram  noden som är närmast randompunkten jag tagit fram
def GetNearestListNode(nodeList, rnd):
   
    mindist = 10000

    for i in range(len(nodeList)):
        x_list = nodeList[i].x
        y_list = nodeList[i].y

        dist = ((x_list - rnd.x) ** 2 + (y_list - rnd.y) ** 2) ** 0.5

        if dist <= mindist:
            mindist = dist
            j = i
    return nodeList[j], dist

def safe_or_not(car, node):
    answer_is = True

    for n in range(len(car.obs)):
        x_ = car.obs[n][0] # Mittpunkt
        y_ = car.obs[n][1]
        radius =  0.2 # ökar radien!
        # this is okkkk
        band_low_x = x_ - radius # nedre xdelen av hindret
        band_upp_x = x_ + radius # övre xdelen av hindret
        band_low_y = y_ - radius # nedre ydelen av hindret
        band_upp_y = y_ + radius # övre ydelen av hindret

        # Kollar om nära ett hinder
        if (node.x <= band_upp_x) and (node.x >= band_low_x) and (node.y >= band_low_y) and (node.y <= band_upp_y):
            answer_is = False
            break

        # Kollar om nära en vägg
        #if (node.x <= car.xlb) or (node.x >= car.xub ) or (node.y <= car.ylb) or (node.y >= car.yub):
        #    answer_is = False
        #    break
    return answer_is

def safe_disc(node, mapp):

    answer_is = True 

    xdisc = np.floor(node.x/(resolution))
    ydisc = np.floor(node.y/(resolution))

    ans = mapp[int(xdisc), int(ydisc)] 

    if ans > 10:
        answer_is = False
        print('not safe!!')

    return answer_is


def get_best_child(car, nearest_node, rnd, ax):
    global tick

    dist_shortest = 10000
    curr_node = None
    phi_val = [-MODEL.DELTA_MAX, -MODEL.DELTA_MAX/2, 0, MODEL.DELTA_MAX/2, MODEL.DELTA_MAX]
    xkand = []
    ykand = []
    thetakand = []
    phikand = []
    distkand = []
    ddist = []

    for k in range(len(phi_val)):
        phi = phi_val[k]
        x = nearest_node.x
        y = nearest_node.y
        theta = nearest_node.theta

        for i in range(step_length):


            x, y, theta = step(car, x, y, theta, phi)           
            check = Node(x,y)

            #safe = safe_disc(check, car.matrix)
            safe = safe_or_not(car, check)

            if not safe:

                break

        dist = sqrt((x - rnd.x) ** 2 + (y - rnd.y) ** 2)
        if dist < dist_shortest and safe:

            distkand.append(dist)
            dist_shortest = dist
            xkand.append(x)
            ykand.append(y)
            thetakand.append(theta)
            phikand.append(phi)

    for i in range(len(Blacklist)):
        x_list = Blacklist[i].x
        y_list = Blacklist[i].y
        tick += 1

        for j in range(len(xkand)):
            print('xkand =', xkand)
            print('j =', j)
           # if xkand: # because if empty I get a error that it cannot enter the next if 
            if x_list == xkand[j] and y_list == ykand[j]:
                distkand[j] = 1000
               # xkand.pop(j)
               # ykand.pop(j)
               # j-=1
                    

    # This is if none of the phi values are ok, then we need to choose another parent?
    if distkand:
        ddist = np.min(distkand)

    if not xkand or ddist > 100:
        NodeList.remove(nearest_node)
        Blacklist.append(nearest_node)
        tick += 1 
       # print('Blacklist =', Blacklist)
        ax.plot(nearest_node.x, nearest_node.y, 'k*')
        plt.draw()
        plt.pause(0.0001)
        return
    else:
        index_min = np.argmin(distkand)
        x = xkand[index_min]
        y = ykand[index_min]
        theta = thetakand[index_min]
        phi = phikand[index_min]

        del xkand[:]
        del ykand[:]
        del thetakand[:]
        del phikand[:]

        curr_node = Node(x, y, phi, theta, (nearest_node.time + step_length * 0.01), nearest_node)


    return curr_node


def step(car, x, y, theta, phi, dt=0.01): # get from another function! 

    dx     = np.cos(theta)
    dy     = np.sin(theta)
    dtheta = np.tan(phi)

    # new state (forward Euler integration)
    xn     = x     + dt*dx
    yn     = y     + dt*dy
    thetan = theta + dt*dtheta

    return xn, yn, thetan


def get_random_position(NodeList, dist1, dist2, target):
    global countis
    global tick
    print('tick =', tick)
   
    if dist1 == dist2 or tick > 4:
        print('I am stuck! searching randomly!')
        if countis == 2:
            rnd = [target.x, random.uniform(1, height * resolution)]
            random_p = Node(rnd[0], rnd[1])
            countis = 0
            tick = 0
        elif countis == 1:
            rnd = [random.uniform(0, width * resolution), target.y]
            random_p = Node(rnd[0], rnd[1])
            countis += 1
            tick = 0
        else:
            rnd = [random.uniform(0, width * resolution), random.uniform(1, height * resolution)]
            random_p = Node(rnd[0], rnd[1])
            countis += 1
            tick = 0
    else:
        random_p = Node(target.x, target.y)
        countis = 0

    return random_p

