#!/usr/bin/env python2

from svea.states import VehicleState
import pickle
import os
from planning import solution
import matplotlib.pyplot as plt
import numpy as np

dirname = os.path.dirname(__file__)
svea_core = os.path.join(dirname, '../../')
map_name = "floor2_rot" # change this for different maps
file_path = svea_core + 'scripts/core_examples/' + map_name + ".pickle"

fig, ax = plt.subplots(1)
ax.set_xlim([10, 40])
ax.set_ylim([-2, 10])
plt.grid(True)
resolution = 0.05

pkl_file = open(file_path, 'rb')
obsticle = pickle.load(pkl_file)
ob_array =  np.array(obsticle.data)

V = VehicleState()
V.matrix = []

class Node():

    def __init__(self, x, y, phi=None, theta=None, time=None, parent=None):
        self.x = x
        self.y = y
        self.phi = phi # control
        self.theta = theta 
        self.parent = parent
        self.time = time
        self.cost = 0.0

def main():

    timestart = 0
    phistart = 0
    thetastart = 0
    start = Node(15, 1.54, phistart, thetastart, timestart)
    #start = Node(37.29002809179802, 6.561762500969493, phistart, thetastart, timestart)
    
    target2 = Node(22, 4)
    target1 = Node(20, 1.45)
    target3 = Node(27, 4.5)
    target4 = Node(28.4, 7)
   
   # target1 = Node(36.3, 1.44)
   # target2 = Node(37.3, 6.6)
   # target3 = Node(14.8, 7.5)
   # target4 = Node(15, 1.54)

    targetlist = [target1, target2, target3, target4]

    width = 879 
    height = 171

    map_matrix = np.reshape(ob_array,(width, height), order='F')
    obslist = get_obs(map_matrix)

    V.obs = obslist
    V.matrix = map_matrix
    traj_x = []
    traj_y = []
    for i in range(4):
        target = targetlist[i]

        ax.plot(start.x, start.y, "*r")
        ax.plot(target.x, target.y, "*r")
        print('start point:', (start.x, start.y))
        print('target point:', (target.x, target.y))

        [Nodelistx, Nodelisty, Nodelisttheta] = solution(V, start, target, ax)
        if i < 4:
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
    for n in range(200, 878):
        for m in range(0, 170):
            value = obs_matrix[n, m]

            if value > 10:
                obslist.append([n*resolution, m*resolution])
                ax.plot(n*resolution, m*resolution, '.k')

    return obslist



if __name__ == '__main__':
    main()