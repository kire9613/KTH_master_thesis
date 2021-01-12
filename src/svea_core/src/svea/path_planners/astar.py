#!/usr/bin/env python2
import yaml
import random
import math
import numpy
import matplotlib.pyplot as plt
import os
import rospy


def generateTrajectory( astar_settings,x0,y0,theta0, xt,yt,plotBool, file = None):
    # Generate path trajectory from initial position to goal position
    ###########################################################################
    # define parameters and initialize variables: #

    obstacleResolution = 0.05 # [m]

    # create lists that contain x,y coordinates of obstacles
    expanded_obstacle_list_x = []
    expanded_obstacle_list_y = []

    # find path to obstacles file
    astarPath = os.path.dirname(os.path.abspath(__file__))

    #file = open(astarPath + '/aStarPlannerObstacles.yaml') # track with the corridor as obstacles
    if "subscribe_to_obstacles" in astar_settings:
        track = rospy.get_param('/team_5_floor2/lidar_obstacles')
    if astar_settings["use_q1"]:
        file = open(astarPath + '/race_track.yaml') # the geofenced track provided by the TAs
    else:
        file = open(astarPath + '/aStarTrack.yaml') # the geofenced track provided by the TAs

    obstacles = yaml.safe_load(file)
    keep_out = obstacles.get('keep_out')
    stay_in = obstacles.get('stay_in')
           
    ###########################################################################
    # range over the different obstacles and add
    if "subscribe_to_obstacles" in astar_settings:
        for i in range(len(track)): # ranges over all obstacles in the map_file
            obstacle = track[i]
            for indexPt in range(len(track[i])-1):           
                dist = numpy.linalg.norm(numpy.array(obstacle[indexPt])-numpy.array(obstacle[indexPt+1]))
                expanded_obstacle_list_x += numpy.linspace(obstacle[indexPt][0],obstacle[indexPt+1][0],int(dist/obstacleResolution)).tolist()
                expanded_obstacle_list_y += numpy.linspace(obstacle[indexPt][1],obstacle[indexPt+1][1],int(dist/obstacleResolution)).tolist()

    for indexPt in range(len(keep_out)-1): # ranges over all obstacles in the map_file
        dist = numpy.linalg.norm(numpy.array(keep_out[indexPt])-numpy.array(keep_out[indexPt+1]))
        expanded_obstacle_list_x += numpy.linspace(keep_out[indexPt][0],keep_out[indexPt+1][0],int(dist/obstacleResolution)).tolist()
        expanded_obstacle_list_y += numpy.linspace(keep_out[indexPt][1],keep_out[indexPt+1][1],int(dist/obstacleResolution)).tolist()

    for indexPt in range(len(stay_in)-1): # ranges over all obstacles in the map_file
        dist = numpy.linalg.norm(numpy.array(stay_in[indexPt])-numpy.array(stay_in[indexPt+1]))
        expanded_obstacle_list_x += numpy.linspace(stay_in[indexPt][0],stay_in[indexPt+1][0],int(dist/obstacleResolution)).tolist()
        expanded_obstacle_list_y += numpy.linspace(stay_in[indexPt][1],stay_in[indexPt+1][1],int(dist/obstacleResolution)).tolist()

    assert(len(expanded_obstacle_list_x) == len(expanded_obstacle_list_y))

    # call A_star planner to generate trajectory based on the extracted and modified obstacles
    xtraj,ytraj,success, lastTheta = A_star(xt,yt,x0,y0,theta0,expanded_obstacle_list_x,expanded_obstacle_list_y, astar_settings)
    if plotBool:
        fig1, (ax1,ax2) = plt.subplots(nrows=1, ncols=2)
        ax1.plot(expanded_obstacle_list_x,expanded_obstacle_list_y,'.')
        ax1.plot(xtraj,ytraj)
        ax2.imshow(numpy.rot90(grid))

    if success and astar_settings["intermediate_point"] == True:
        print("attempting second lap with:", x0, y0, xtraj[-1], ytraj[-1])
        x,y,success, blabla = A_star(x0, y0, xtraj[-1], ytraj[-1], lastTheta ,expanded_obstacle_list_x,expanded_obstacle_list_y, astar_settings)
        xtraj += x 
        ytraj += y 
            
    if plotBool:
        ax1.plot(x0,y0,'o')
        ax1.plot(xt,yt,'o')
        ax1.plot(xtraj,ytraj)
        ax1.legend(["obstacles","generated A* path"])
        ax1.axis('equal')
        ax1.grid()
        ax2.set_title('Explored Points on map, 1 = unexplored,  0.5 = occupied, 0 = explored')
        ax2.set_xlabel("grid index")
        ax2.imshow(numpy.rot90(grid))
        plt.show()
       
    return xtraj, ytraj, success

def calculateHeuristic(elem):
    return elem[2] + elem[1]

class gridpt(object):
    # a linked node object representing a coordinate in the map providing ability
    # to run A* with regard to approximated car dynamics.
    global grid, dl, xlb, ylb
    def __init__(self, settings, x, y, theta, xt, yt):
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
        self.dt = settings["driving_distance"]   # should not be put too larer number than 2times safety distance
        # estimated speed of car when planning
        self.v = 1 # keep as 1


    def generate_children(self,settings,list_obs_x,list_obs_y):
        # Expand nodes for Astar
        safety_dist = settings["safety_distance"]
        new_pose = []
        for dTheta in ([-math.pi/8, -math.pi/12, 0, math.pi/12, math.pi/8 ]):
            new_pose.append([self.x + self.v*self.dt*math.cos(self.theta +dTheta), self.y + self.v*self.dt*math.sin(self.theta +dTheta), self.theta +dTheta, dTheta])
        for child in new_pose:
            closest_distance_to_obstacle = 1000 # big value 
            status = True
            gridx = int(numpy.floor((child[0]-xlb)/dl))
            gridy = int(numpy.floor((child[1]-ylb)/dl))
            if (grid[gridx,gridy]) == 0 or (grid[gridx,gridy]) == 0.5:
                status = False
            else:
                for i in range(len(list_obs_x)):
                    distance_to_obstacle =  math.sqrt((child[0]-list_obs_x[i])**2+(child[1]-list_obs_y[i])**2) 
                    if distance_to_obstacle < safety_dist:
                        grid[gridx,gridy] = 0.5
                        status = False
                        break
                    elif distance_to_obstacle < closest_distance_to_obstacle:
                        closest_distance_to_obstacle = distance_to_obstacle

            if status != False:
                grid[gridx,gridy] = 0
                self.children.append([gridpt(settings,child[0],child[1],child[2],self.xt,self.yt),child[3],closest_distance_to_obstacle]) # attach and create new node to list w the corresponding steeering angle and cost

def A_star(xt,yt,x0,y0,theta0,list_obs_x,list_obs_y, settings):
    # Main Astar algorithm
    
    print('calling A*')

    global grid, dl, xlb, ylb
    if "grid_resolution" in settings:
        dl = settings["grid_resolution"]
    else:
        dl = 0.075

    if "success_threshold" not in settings:
        settings["success_threshold"] = 0.4

    if "maximum_expansion" not in settings:
        settings["maximum_expansion"] = 100000

    #retrieve max/min coordinates of obstacles and 1 meter additional space in all directions.    
    xub, xlb = max(list_obs_x) + 1, min(list_obs_x) -1
    yub, ylb = max(list_obs_y) + 1, min(list_obs_y) -1
    # initialize grid to keep track of explored coordinates
    grid = numpy.ones([int((xub-xlb + 1)/dl),int((yub-ylb + 1)/dl)]) # 1 = not visited

    #initialize list for A* solution 
    xtraj =[]
    ytraj = []
    # initialize success boolean
    success = False

    #### A* algorithm: ####

    S = [] # explored list
    S0 = gridpt(settings,x0,y0,theta0,xt,yt) # initial node
    Q = [[S0,S0.heuristic,0,None,None]] # expansion queue
    Vr = [0] # cost list
    Vr_pathpter = [None] # path pointer 

    while len(Q) > 0 and not rospy.is_shutdown():
        Q.sort(key=calculateHeuristic) # sort the list of nodes according to heuristic value
        currnode = Q[0] # pick the current node as the one w the best heuristic
        S.append(Q.pop(0))  # adding the best node to the solved list
        currnode[0].generate_children(settings,list_obs_x,list_obs_y)
        children_nodes  = currnode[0].children
        print "\r number of expanded nodes: ", len(S),
        if len(S) > settings["maximum_expansion"]:
            print("\r  A* failiure: too many nodes expanded")
            break
        if len(children_nodes) > 0: # append children to queue
            for index in range(len(children_nodes)): 
                Q.append([children_nodes[index][0], children_nodes[index][0].heuristic,numpy.inf,None,[]])  # 0:node object, 1:heursitic, 2:Vr, 3:Pointer
                Qchild = Q[-1]
                children_nodes[index][0].parent = currnode[0]
                steering_angle = children_nodes[index][1]
                closest_distance_to_obstacle = children_nodes[index][2]
                children_nodes[index][0].howtofindme.append(steering_angle) # append the angle # append x,y
                if currnode[2] + 0.1 + abs(steering_angle)*0.3 + 0.5/closest_distance_to_obstacle < Qchild[2]: # cost to reach
                    Q[Q.index(Qchild)][2] = currnode[2] + 0.1 + abs(steering_angle)*0.3  + 0.5/closest_distance_to_obstacle# cost to reach from start
                    Q[Q.index(Qchild)][3] = currnode[0]    # update pointer to mother-node
                    Q[Q.index(Qchild)][4] = children_nodes[index][1] # heuristic cost from child to goal
        if S[-1][0].heuristic <= settings["success_threshold"]: # if we're close to the target --> stop
            success = True
            print('\n target found!')
            endnode = S[-1][0]
            endnodeTheta = endnode.theta
            xtraj.insert(0,endnode.x)
            ytraj.insert(0,endnode.y)
            print("we found our way to:",  xtraj, ytraj)
            while endnode.parent != None: # trace back in history and find total trajectory
                xtraj.insert(0,endnode.x)
                ytraj.insert(0,endnode.y)
                endnode = endnode.parent

            break
            
    if len(xtraj) == 0:
        endnodeTheta = 0 
        print('\n Astar Failed - could not find trajectory')      
    return xtraj,ytraj,success, endnodeTheta

def __main__(): 
    xt, yt = 15.5,4.3
    x0, y0, theta0 = 1, 0, 0
    settings = {
        "driving_distance": 0.4,
        "use_track": True,
        "safety_distance": 0.3,
        "grid_resolution": 0.1,
        "success_threshold": 0.5,
        "use_q1": True,
        "intermediate_point": True
        }

    return generateTrajectory(settings,x0,y0,theta0,xt,yt,True)


if __name__ == "__main__":
    __main__()
