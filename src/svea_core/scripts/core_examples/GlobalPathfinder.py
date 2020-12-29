#!/usr/bin/env python2
from svea.states import VehicleState # Prova radera bara!
import pickle
import os
#from RRT_Team1 import solution
import matplotlib.pyplot as plt
import numpy as np
import argparse
from Track import *
#from FILENAME import raytrace


dirname = os.path.dirname(__file__)
svea_core = os.path.join(dirname, '../../')
map_name = "q1" # change this for different maps
file_path = svea_core + 'scripts/core_examples/' + map_name + ".pickle"


# Plott settings 
fig, ax = plt.subplots(1)
ax.set_xlim([-15, 22])
ax.set_ylim([-2.5, 8])
plt.grid(True) 

# loads obsticle from a pickle file 
pkl_file = open(file_path, 'rb')
obsticle = pickle.load(pkl_file)
ob_array =  np.array(obsticle.data)

V = VehicleState() # TODO delete
V.matrix = []


class Node(): # TODO kanske inte behover

    def __init__(self, x, y, phi=None, theta=None, time=None, parent=None):
        self.x = x
        self.y = y
        self.phi = phi # control
        self.theta = theta 
        self.parent = parent


def main(args):

    phistart = 0
    thetastart = 0
    start = Node(0, 0, phistart, thetastart)
    
    # Here is different kind of targets 

    target1 = Node(2, 0)
    target2 = Node(4, 2)
    target3 = Node(6, 2)
    target4 = Node(9, -0.1)
    target5 = Node(15, -1)
    target6 = Node(18.7, -1)
    target7 = Node(18.6, 3.5)
    target8 = Node(6, 5)
    target9 = Node(4.2, 7)
    target10 = Node(0.5, 5.5)
    target11 = Node(-13, 6)
    target12 = Node(-14, 1.6)
    target13 = Node(-13.4, 1.1)
    target14 = Node(-9, 0.4)
    target15 = Node(-7.82, 2.9)
    target16 = Node(-5.8, 3.2)
    target17 = Node(-5.6, 1)
    target18 = Node(0, 0)

    targetlist = [target1, target2, target3, target4, target5, target6, target7, target8, target9, target10, target11, target12, target13, target14, target15, target16, target17, target18]

    map_matrix = np.reshape(ob_array,(args.width, args.height), order='F')

    global oob_delimiter
    oob_delimiter = max(args.width,args.height) + 1


    ######## Here user can choose between having a track or let the Pathfinder respect obsticle from a 
    ## tickle file. or both!! 

    # Gets track info (this is the fastest one)
    obslist = get_track(keep_out, stay_in)

   # Gets obstacle info (this is the fastest one)
    #obslist = get_obs(map_matrix)

    V.obs = obslist #TODO maybe no need 
    V.matrix = map_matrix
    traj_x = []
    traj_y = []
    
    for i in range(len(targetlist)):
        target = targetlist[i]

        ax.plot(start.x, start.y, "*r")
        ax.plot(target.x, target.y, "*r")
        print('start point:', (start.x, start.y))
        print('target point:', (target.x, target.y))

        [Nodelistx, Nodelisty, Nodelisttheta] = solution(V, start, target, ax, args)
        if i < (len(targetlist)-1):

            # Here is the last node closes to the target poped out to be the start point for the next path finder
            start.x = Nodelistx.pop()
            start.y = Nodelisty.pop()
            start.theta = Nodelisttheta.pop()

        traj_x += Nodelistx
        traj_y += Nodelisty


    for i in range(len(traj_x)):
        ax.plot(traj_x[i],traj_y[i], "*r")
        plt.draw()
        plt.pause(0.0001)

    print('x cord =', traj_x)
    print('y cord =', traj_y)
    plt.show()


# functions that lists all nodes with obsticles from the matrix that says a value == 100 if obstacle
def get_obs(obs_matrix): 
    obslist = []

    diffx = 23.7024 - (-6.76)
    diffy = 15.5143 - 4.02
    for n in range(140, 1100):
        for m in range(140, 400):
            value = obs_matrix[n, m]

            if value > 10:
               
                obslist.append([n*args.resolution - diffx, m*args.resolution - diffy])
                ax.plot(n*args.resolution - diffx, m*args.resolution - diffy, '.k')

    return obslist


# This function gets the track and plots it. 
def get_track(keep_out, stay_in):
    obslist = []

    for i in range(0, len(keep_out) -1):

        track = raytrace(keep_out[i, 0], keep_out[i, 1], keep_out[i+1, 0], keep_out[i+1, 1])
        for j in range(0, len(track)):
            ax.plot(track[j, 0]*args.resolution, track[j, 1]*args.resolution, '.b')
            obslist.append(track[j]*args.resolution)

    for i in range(0, len(stay_in) -1):
        track = raytrace(stay_in[i, 0], stay_in[i, 1], stay_in[i+1, 0], stay_in[i+1, 1])
        for j in range(0, len(track)):
            ax.plot(track[j, 0]*args.resolution, track[j, 1]*args.resolution, '.b')
            obslist.append(track[j]*args.resolution)


    track = raytrace(keep_out[len(keep_out)-1, 0], keep_out[len(keep_out)-1, 1], keep_out[0, 0], keep_out[0, 1])
    for j in range(0, len(track)):
            ax.plot(track[j, 0]*args.resolution, track[j, 1]*args.resolution, '.b')
            obslist.append(track[j]*args.resolution)

    track = raytrace(stay_in[len(stay_in)-1, 0], stay_in[len(stay_in)-1, 1], stay_in[0, 0], stay_in[0, 1])
    for j in range(0, len(track)):
            ax.plot(track[j, 0]*args.resolution, track[j, 1]*args.resolution, '.b')
            obslist.append(track[j]* args.resolution)

    return obslist



def raytrace(start_x, start_y, end_x, end_y):
    """Returns all cells in the grid map that has been traversed
    from start to end, including start and excluding end.
    """
    start_x = np.floor(start_x/(args.resolution))
    start_y = np.floor(start_y/(args.resolution))

    end_x = np.floor(end_x/(args.resolution))
    end_y = np.floor(end_y/(args.resolution))

    x = start_x
    y = start_y
    dx = np.abs(end_x - start_x)
    dy = np.abs(end_y - start_y)
    n = int(dx + dy)
    x_inc = 1
    if end_x <= start_x:
        x_inc = -1
    y_inc = 1
    if end_y <= start_y:
        y_inc = -1
    error = dx - dy
    dx *= 2
    dy *= 2
    traversed = np.full((n + 1,2), np.int16(oob_delimiter))
    i = 0
    while i < np.int16(n):
        traversed[i,0] = np.int16(x)
        traversed[i,1] = np.int16(y)
        if error > 0:
            x += x_inc
            error -= dy
        else:
            if error == 0:
                traversed[i,0] = np.int16(x + x_inc)
                traversed[i,1] = np.int16(y)
            y += y_inc
            error += dx
        i += 1
    return traversed


def parser():

    parser = argparse.ArgumentParser(
        description = "These are all choosable constants")

    parser.add_argument("--step_length", type=int, default = 30, help="Check safeness for every step this long.")

    parser.add_argument("--obs_radius", type=float,  default = 0.2, help="Safe distance [m] to an obstacle as radius.")

    parser.add_argument("--reached_goal", type=float, default = 0.45, help="Distance [m] the tolerance to hit the target.")
    
    parser.add_argument("--resolution", type=float, default = 0.05, help="<<<<DO NOT CHANGE>>>>,  The map resolution [m/cell].")

    parser.add_argument("--width", type=int, default = 1269, help="<<<<DO NOT CHANGE>>>>, Width of the map.")

    parser.add_argument("--height", type=int, default = 567, help="<<<<DO NOT CHANGE>>>>, Height of the map.")

    return parser.parse_args()

if __name__ == '__main__':

    args = parser()
    from RRT_Team1 import solution
    print(args.obs_radius)
    main(args)