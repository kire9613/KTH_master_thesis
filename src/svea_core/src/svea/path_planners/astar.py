#!/usr/bin/env python2
import yaml
import random
import math
import numpy
import matplotlib.pyplot as plt
import os


def generateTrajectory(x0,y0,theta0, xt,yt,plotBool,file = None):
    ###########################################################################
    # define parameters and initialize variables: #
    obstacleResolution = 0.05# [m]
    # create lists that contain x,y coordinates of obstacles
    expanded_obstacle_list_x = []
    expanded_obstacle_list_y = []
    # define target and initial coordinates of trajectory, and initial angle


    ###########################################################################

    # find path to obstacles
    astarPath = os.path.dirname(os.path.abspath(__file__))
    file = open(astarPath + '/aStarPlannerObstacles.yaml') # track with the corridor as obstacles
    #file = open(astarPath + '/aStarTrack.yaml') # the geofenced track provided by the TAs
    obstacles = yaml.safe_load(file)


    ###########################################################################
    # range over the different obstacles and add
    track = obstacles.get('track')
    for i in range(len(track)): # ranges over all obstacles in the map_file
        obstacle = track[i]
        for indexPt in range(len(track[i])-1):
            dist = numpy.linalg.norm(numpy.array(obstacle[indexPt])-numpy.array(obstacle[indexPt+1]))
            expanded_obstacle_list_x += numpy.linspace(obstacle[indexPt][0],obstacle[indexPt+1][0],int(dist/obstacleResolution)).tolist()
            expanded_obstacle_list_y += numpy.linspace(obstacle[indexPt][1],obstacle[indexPt+1][1],int(dist/obstacleResolution)).tolist()
    assert(len(expanded_obstacle_list_x) == len(expanded_obstacle_list_y))

    # call A_star planner to generate trajectory based on the extracted and
    # modified obstacles
    xtraj,ytraj = A_star(xt,yt,x0,y0,theta0,expanded_obstacle_list_x,expanded_obstacle_list_y)
    if plotBool:
        fig1, (ax1,ax2) = plt.subplots(nrows=1, ncols=2)
        ax1.set_title('Generated A* Path')
        ax1.plot(expanded_obstacle_list_x,expanded_obstacle_list_y,'.')
        ax1.plot(xtraj,ytraj)
        ax1.legend(["obstacles","generated A* path"])
        ax1.axis('equal')
        ax1.grid()
        ax2.set_title('Explored Points on map, 1 = unexplored,  0.5 = occupied, 0 = explored')
        ax2.set_xlabel("grid index")
        ax2.imshow(numpy.rot90(grid))
        plt.show()


    return xtraj,ytraj

def calculateHeuristic(elem):
    return elem[2] + elem[1]

class gridpt(object):
    # a linked node object representing a coordinate in the map providing ability
    # to run A* with regard to approximated car dynamics.
    global grid, dl, xlb, ylb
    def __init__(self,x,y,theta,xt,yt):
        # coordinates of node
        self.x  = x
        self.y  = y
        # coordinate of target
        self.xt = xt
        self.yt = yt
        # pointer to parent node
        self.parent = None
        # heading at given position
        self.theta =  theta
        # list of nodes providing path to initial coordinates
        self.howtofindme = [] # [x,y], steering angle
        # list of children to current node
        self.children = []
        # control signal required to end up at current node from parent
        self.optimal_control = None
        # heuristic values set as the euclidean distance to target node
        self.heuristic = math.sqrt((x-xt)**2+(y-yt)**2)
        # time interval of one step
        self.dt = 0.2    # should not be put too larer number than 2times safety distance
        # estimated speed of car when planning
        self.v = 1 # keep as 1


    def generate_children(self,list_obs_x,list_obs_y):
        safety_dist = 0.25
        new_pose = []
        for dTheta in ([-math.pi/8, -math.pi/12, 0, math.pi/12, math.pi/8 ]):
            new_pose.append([self.x + self.v*self.dt*math.cos(self.theta +dTheta), self.y + self.v*self.dt*math.sin(self.theta +dTheta), self.theta +dTheta, dTheta])
###### for first angle #####
        for child in new_pose:
          status = True
          gridx = int(numpy.floor((child[0]-xlb)/dl))
          gridy = int(numpy.floor((child[1]-ylb)/dl))
          if (grid[gridx,gridy]) == 0 or (grid[gridx,gridy]) == 0.5:
            status = False
          else:
            for i in range(len(list_obs_x)):
              if math.sqrt((child[0]-list_obs_x[i])**2+(child[1]-list_obs_y[i])**2) <safety_dist:
                grid[gridx,gridy] = 0.5
                status = False
                break
          if status != False:
            grid[gridx,gridy] = 0
            self.children.append([gridpt(child[0],child[1],child[2],self.xt,self.yt),child[3]]) # attach and create new node to list w the corresponding steeering angle

def A_star(xt,yt,x0,y0,theta0,list_obs_x,list_obs_y):
    print('calling A*')

    global grid, dl, xlb, ylb
    dl = 0.1
    xub, xlb = max(list_obs_x),min(list_obs_x)
    yub, ylb = max(list_obs_y),min(list_obs_y)
    # initialize grid to keep track of explored coordinates
    grid = numpy.ones([int((xub-xlb + 1)/dl),int((yub-ylb + 1)/dl)]) # 1 = not visited
    xtraj =[]
    ytraj = []
    #### PARAMS #####

    #### A* algorithm: ####

    S = [] # explored list
    S0 = gridpt(x0,y0,theta0,xt,yt)
    Q = [[S0,S0.heuristic,0,None,None]] # expansion queue
    Vr = [0] # cost list
    Vr_pathpter = [None] # pa

    while len(Q) > 0:
        Q.sort(key=calculateHeuristic)
        currnode = Q[0] # pick the current node as the one w the best heur
        S.append(Q.pop(0))  ## adding the best node to the solved list
        currnode[0].generate_children(list_obs_x,list_obs_y)
        children_nodes  = currnode[0].children
        print "\r number of expanded nodes: ", len(S),
        if len(children_nodes) > 0:
            for index in range(len(children_nodes)): ###### bygg om!!!!!1 children_nodes[index]
                Q.append([children_nodes[index][0], children_nodes[index][0].heuristic,numpy.inf,None,[]])  # 0:node object, 1:heursitic, 2:Vr, 3:Pointer
                Qchild = Q[-1]
                children_nodes[index][0].parent = currnode[0]
                steering_angle = children_nodes[index][1]
                children_nodes[index][0].howtofindme.append(steering_angle) # append the angle # append x,y
                if currnode[2] + 0.2 + abs(steering_angle)*0.1 < Qchild[2]: # cost to reach
                    Q[Q.index(Qchild)][2] = currnode[2] + 0.2 + abs(steering_angle)*0.1 # cost to reach from start
                    Q[Q.index(Qchild)][3] = currnode[0]    # update pointer to mother-node
                    Q[Q.index(Qchild)][4] = children_nodes[index][1] # heuristic cost from child to goal
        if S[-1][0].heuristic <= 0.3: # if we're close to the target --> stop
            print('\n target found!')
            endnode = S[-1][0]
            xtraj.append(endnode.x)
            ytraj.append(endnode.y)
            while endnode.parent != None: # trace back in history and find total trajectory
                xtraj.append(endnode.x)
                ytraj.append(endnode.y)
                endnode = endnode.parent
            break
    if len(xtraj) == 0:
        print('\n could not find trajectory... try other settings?')
    return xtraj,ytraj

def __main__():
    xt, yt = -3.46, -6.93
    x0, y0, theta0 =  -2.84,-8.11,  0.8978652
    return generateTrajectory(x0,y0,theta0,xt,yt,plotBool=True)


if __name__ == "__main__":
    __main__()
