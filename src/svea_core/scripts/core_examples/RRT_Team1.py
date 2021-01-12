#!/usr/bin/env python2
# -*- coding: utf-8 -*-
# {Astrid Lindstedt, November}
# {930701-1982}
# {astridli@kth.se}

import sys
import os
import random
#import rospy
import numpy as np
import pickle
import matplotlib.pyplot as plt
import svea.models.bicycle
from math import hypot, sqrt
from nav_msgs.msg import OccupancyGrid  

MODEL = svea.models.bicycle.SimpleBicycleModel()

class Node():

    def __init__(self, x, y, phi=None, theta=None, parent=None):
        self.x = x
        self.y = y
        self.phi = phi # control angle
        self.theta = theta # car angle
        self.parent = parent 


def solution(car, start, target, ax, args):

    global NodeList, countis, Blacklist, tick
    done = False

    countis = 0
    dist2 = -1

    NodeList = [start]
    Blacklist = []
    tick = 1
    
    count = 7000 # how many node attempts before shutting down 
    first_step = 5 # First stepsize

    # one step in the beginning.

    for i in range(first_step):
        x = start.x + np.cos(start.phi)*0.01
        y = start.y + np.sin(start.phi)*0.01
        theta__ = start.theta


    curr_node = Node(x, y, start.phi, theta__, start)
    NodeList.append(curr_node)

    for j in range(count):
        
        _ , dist1 = GetNearestListNode(NodeList, target) # here I never use close_to_target1 
        
        random_point = get_random_position(NodeList, dist1, dist2, target)
        dist2 = dist1
     
        nearest_node, _ = GetNearestListNode(NodeList, random_point) # brings the best parent to the random point
    
        # Get the best child
        New_nod = get_best_child(car, nearest_node, random_point, ax, args)
        
        if New_nod: # if not nonetype

            xn = New_nod.x
            yn = New_nod.y
            NodeList.append(New_nod)
            # Plotting nodes to map
            ax.plot(New_nod.x, New_nod.y, 'o')
            plt.draw()
            plt.pause(0.00001)

            done = True if ((target.x - xn) ** 2 + (target.y - yn) ** 2) ** 0.5 < args.reached_goal else False

        if done:
            print('Hey! I think I found my target :) ')
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
# This searches my nodelist and extract the closest node from my list to the target/randompoint. 
def GetNearestListNode(nodeList, rnd):
   
    mindist = 10000 # Big distance 

    for i in range(len(nodeList)): # checking every node 
        x_list = nodeList[i].x
        y_list = nodeList[i].y

        dist = ((x_list - rnd.x) ** 2 + (y_list - rnd.y) ** 2) ** 0.5

        if dist <= mindist: # updates the smallest distance 
            mindist = dist 
            j = i
    return nodeList[j], dist

def safe_or_not(car, node, args):
    answer_is = True

    for n in range(len(car.obs)): # This for loop checks distance to every obstacle 
        x_ = car.obs[n][0] # Center point of obstacle
        y_ = car.obs[n][1]
        

        band_low_x = x_ - args.obs_radius # lower x bounds of obstacle
        band_upp_x = x_ + args.obs_radius # upper x bounds of obstacle
        band_low_y = y_ - args.obs_radius # lower y bounds of obstacle
        band_upp_y = y_ + args.obs_radius # upper y bounds of obstacle


        # Checks of it is close to any of the obstacles. 
        if (node.x <= band_upp_x) and (node.x >= band_low_x) and (node.y >= band_low_y) and (node.y <= band_upp_y):
            answer_is = False
            break

    return answer_is

def safe_disc(node, mapp):

    answer_is = True 

    xdisc = np.floor(node.x/(args.resolution))
    ydisc = np.floor(node.y/(args.resolution))

    ans = mapp[int(xdisc), int(ydisc)] 

    if ans > 10:
        answer_is = False
        print('not safe!!')

    return answer_is


def get_best_child(car, nearest_node, rnd, ax, args):
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

        for i in range(args.step_length):


            x, y, theta = step(car, x, y, theta, phi)           
            check = Node(x,y)

            #safe = safe_disc(check, car.matrix) # fast!!!
            safe = safe_or_not(car, check, args) # take long time! 

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

           # if xkand: # because if empty I get a error that it cannot enter the next if 
            if x_list == xkand[j] and y_list == ykand[j]:
                distkand[j] = 1000

    # This is if none of the phi values are ok, then we need to choose another parent?
    if distkand:
        ddist = np.min(distkand)
    # if none of the next steps are safe or if all next states are blacklisted add allso this node to the blacklist
    # and remove it from the nodelist. 
    if not xkand or ddist > 100: 
        NodeList.remove(nearest_node)
        Blacklist.append(nearest_node)
        tick += 1 
       # This plots a black * on top of the node. To visulize the black listed nodes.  
        ax.plot(nearest_node.x, nearest_node.y, 'k*')
        plt.draw()
        plt.pause(0.00001)
        return
    else: # if everything is fine, grad the next step that is closest to the target this is the ''best'' step!
        index_min = np.argmin(distkand)
        x = xkand[index_min]
        y = ykand[index_min]
        theta = thetakand[index_min]
        phi = phikand[index_min]

        del xkand[:]
        del ykand[:]
        del thetakand[:]
        del phikand[:]

        curr_node = Node(x, y, phi, theta, nearest_node)

    return curr_node


def step(car, x, y, theta, phi, dt=0.01):

    dx     = np.cos(theta)
    dy     = np.sin(theta)
    dtheta = np.tan(phi)

    # new state (forward Euler integration)
    xn     = x     + dt*dx
    yn     = y     + dt*dy
    thetan = theta + dt*dtheta

    return xn, yn, thetan


def get_random_position(NodeList, dist1, dist2, target):
    global countis, tick 
   
    if dist1 == dist2 or tick > 4:
        print('I am stuck! searching randomly!')
        if countis == 2:
            rnd = [target.x, random.uniform(1, args.height * args.resolution)]
            random_p = Node(rnd[0], rnd[1])
            countis = 0
            tick = 0
        elif countis == 1:
            rnd = [random.uniform(0, args.width * args.resolution), target.y]
            random_p = Node(rnd[0], rnd[1])
            countis += 1
            tick = 0
        else:
            rnd = [random.uniform(0, args.width * args.resolution), random.uniform(1, args.height * args.resolution)]
            random_p = Node(rnd[0], rnd[1])
            countis += 1
            tick = 0
    else: # Sends the target point if the car is not stuck. 
        random_p = Node(target.x, target.y)
        tick = 0

    return random_p

if __name__ == '__main__':

    print('Please play me by running the GlobalPathfinder.py file!')