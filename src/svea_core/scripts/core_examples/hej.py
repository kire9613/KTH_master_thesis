#!/usr/bin/env python2

from svea.states import VehicleState
import pickle
import os
from planning import solution
import matplotlib.pyplot as plt
import numpy as np

dirname = os.path.dirname(__file__)
svea_core = os.path.join(dirname, '../../')
map_name = "q1" # change this for different maps
file_path = svea_core + 'resources/maps/' + map_name + ".pickle"

fig, ax = plt.subplots(1)
ax.set_xlim([7, 55])
ax.set_ylim([7, 20])
plt.grid(True)
resolution = 0.05

pkl_file = open(file_path, 'rb')
obsticle = pickle.load(pkl_file)
ob_array =  np.array(obsticle.data)
#print(ob_array)

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
    start = Node(10.95, 12.03, phistart, thetastart, timestart)
    #start = Node(37.29002809179802, 6.561762500969493, phistart, thetastart, timestart)
    
    #target2 = Node(22, 4)
    #target1 = Node(20, 1.45)
    #target3 = Node(27, 4.5)
    #target4 = Node(28.4, 7)
    ''' floor 2'''
    #target1 = Node(36.3, 1.44)
    #target2 = Node(37.3, 6.6)
    #target3 = Node(14.8, 7.5)
    #target4 = Node(15, 1.54)
    '''Fine-tuned floor2'''
    '''
    target2 = Node(37.3,6.6)
    target1 = Node(36.3, 1.44)
    target3 = Node(32.50, 6.97)
    target4 = Node(14.958, 7.19)
    target5 = Node(14.18,2.12)
    target6 = Node(20.57,1.53)
    '''
    '''q1'''
    #target1 = Node(40.00, 10.88)
    #target2 = Node(49.23, 10.10)
    #print("I am here")
    #target3 = Node(49.52,15.21)
    #target4 = Node(49.23, 15.29)
    #target4 = Node(24.89, 17.06)
    #target5 = Node(24.89, 13.09)

    '''fine tuned q1'''
    target1 = Node(40.00, 10.88)
    target2 = Node(48.7262, 10.356)
    target3 = Node(49.3279, 14.4277)
    target4 = Node(18.1763, 17.2387)
    target5 = Node(16.3045, 13.2011)
    target6 = Node(18.6442, 11.8382)
    target7 = Node(21.51, 11.80)
    '''
    target6 = Node(37, 2)
    target7 = Node(26.4, 2.1)
    target8 = Node(22, 7.35)
    target9 = Node(14.5, 7)
    target10 = Node(15, 1.5)
    '''



   
   # target1 = Node(36.3, 1.44)
   # target2 = Node(37.3, 6.6)
   # target3 = Node(14.8, 7.5)
   # target4 = Node(15, 1.54)

    targetlist = [target1, target2, target3, target4,target5,target6, target7]# target5]#, target6]#, target6, target7, target8, target9, target10]

    width = 1269 #879 
    height = 567  #171
    map_matrix = np.reshape(ob_array,(width, height), order='F')
    #big_matrix = get_bigger_matrix(map_matrix)
    #matrix_plot(map_matrix)
    
    #plt.imshow(map_matrix)
    #plt.show()
    obslist = get_obs(map_matrix)
    #print(obslist)
    V.obs = obslist
    V.matrix = map_matrix
    #print(V.obs)
    traj_x = []
    traj_y = []
    for i in range(len(targetlist)):
        target = targetlist[i]

        ax.plot(start.x, start.y, "*r")
        ax.plot(target.x, target.y, "*r")
        print('start point:', (start.x, start.y))
        print('target point:', (target.x, target.y))

        [Nodelistx, Nodelisty, Nodelisttheta] = solution(V, start, target, ax)
        if i < (len(targetlist)-1):
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
    for n in range(140,1100):
        for m in range(140,400):
            value = obs_matrix[n, m]

            if value > 10:
               
                obslist.append([n*resolution, m*resolution])
                ax.plot(n*resolution, m*resolution, '.k')

    return obslist

# functions that lists all nodes with obsticles from the matrix that says a value == 100 if obstacle
def get_bigger_matrix(obs_matrix): 
    obslist = []
    for n in range(140,1100):
        for m in range(140,400):
            value = obs_matrix[n, m]

            if value > 10:
                obs_matrix = np.pad(obs_matrix, pad_width=4, mode='constant', constant_values=100)
              #  obs_matrix[n+i, m+1] = 100
              #  obs_matrix[n+1, m-1] = 100
              #  obs_matrix[n-1, m-1] = 100
              #  obs_matrix[n-1, m+1] = 100
              #  obs_matrix[n, m+1] = 100
              #  obs_matrix[n, m-1] = 100
              #  obs_matrix[n+1, m] = 100
              #  obs_matrix[n-1, m] = 100               

    return obslist


def matrix_plot(obs_matrix): 

    for n in range(140,1100):
        for m in range(140,400):
            value = obs_matrix[n, m]

            if value > 10:
                ax.plot(n*resolution, m*resolution, '.k')              
    return





if __name__ == '__main__':
    main()