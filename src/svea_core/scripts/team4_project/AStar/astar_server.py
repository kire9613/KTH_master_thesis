#!/usr/bin/env python

import rospy
from dublins import *
from a_star import *
from Queue import PriorityQueue
from nav_msgs.msg import OccupancyGrid
from math import hypot, pi, ceil
import numpy
import team4_msgs.msg
import actionlib
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from team4_project.mapping2.updatemap import UpdateMap

N_GRID_X = 40
N_GRID_Y = N_GRID_X//2
DELTA_T = 0.040     # time step, dt_max = 0.05 given max_vel = 1 m/s
GOAL_RADIUS = 0.1   # Accepted deviation from goal position
GOAL_ANGLE = pi/6   # Accepted deviation from goal theta
SAMPLE_TIME = 0.01
ANGLES = [-pi/4,-pi/8,0,pi/8,pi/4]#[-pi/4, 0, pi/4]
N_HEADINGS = 6
N_STEPS = 10        # number of steps with same control signal
GRID_SIZE_X = 0     # initialized in solution
GRID_SIZE_Y = 0     # initialized in solution
OBJECTIVE = []      # initialized in solution

DEBUG = False

if DEBUG:
    import ass3_debug as dbg

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

        # Make sure theta is in [0, 2pi]
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
            self.thetad = N_HEADINGS-1;

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
    if status:
        return False
    else:
        return True

def isObstacle_2(obj, x,y):
    """ Check for collisions """

    status = obj._environment.safe(x,y)
    if status:
        return False
    else:
        return True

def updateNeighbors(obs, openSet, closedSet, n):
    """
    Create and add new nodes in A* search tree
    Do collision check
    """

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
            nnPriority = (nn.cost + 3*heur(nn.x, nn.y, obs.xt, obs.yt), nn)#(nn.cost + heur(nn.x, nn.y, obs.xt, obs.yt), nn)

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

def create_path_obj(path):
    new_path = Path()
    new_path.header.frame_id = "map"
    new_path.poses = []
    for p in path:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x =p[0]
        pose.pose.position.y = p[1]
        new_path.poses.append(pose)

    return new_path

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

        return create_path_obj(new_path)
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

        return create_path_obj(new_path)
    else:
        # insert first node
        p = (path[0][0],path[0][1])
        new_path.append(p)

        new_path += smooth_path(environment,path[ok_i[-1]:-1])

    return create_path_obj(new_path)

class AStarAction(object):
    # create messages that are used to publish feedback/result
    _feedback = team4_msgs.msg.AStarFeedback()
    _result = team4_msgs.msg.AStarResult()

    def __init__(self, name):
        self._action_name = name

        self._map_srv = UpdateMap()
        self._map = self._map_srv.get_inflated_map()
        self._mapinfo = self._map_srv.get_map_info()

        oc_map = rospy.wait_for_message('/map', OccupancyGrid)
        self._map = np.array(oc_map.data).reshape(oc_map.info.height,oc_map.info.width)
        self._mapinfo = [oc_map.info.width, oc_map.info.height, oc_map.info.resolution, oc_map.info.origin.position.x, oc_map.info.origin.position.y]

        self._as = actionlib.SimpleActionServer(self._action_name, team4_msgs.msg.AStarAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(1)
        self.success = True

        # append the seeds for the fibonacci sequence
        self._feedback.distance_to_goal = float(0)

        # publish info to the console for the user
        rospy.loginfo('%s: Executing, searching for path from (%f,%f) to (%f,%f)' % (self._action_name, goal.x0, goal.y0, goal.xt, goal.yt))

        self._map = self._map_srv.get_inflated_map()
        env = Environment(self._map, self._mapinfo)
        car = Objective(goal.xt, goal.yt, goal.thetat, goal.x0, goal.y0, goal.theta0, env)

        path = self.run_astar(car,goal.smooth)

        if self.success:
            self._result.path = path
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

    def run_astar(self, objective, smooth=False):
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

        if DEBUG:
            plotter = dbg.TreePlot(objective._environment, N_STEPS)

        openSet.put((heur(objective.x0, objective.y0, objective.xt, objective.yt), Node(objective.theta0, objective.x0, objective.y0, 0, None, 0)))

        while not openSet.empty():

            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                self.success = False
                break

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
                path = Path()
                path.header.frame_id = "map"
                path.poses = []
                while n.parent is not None:
                    controls.append(n.control)
                    nodes_in_path.append((n.x,n.y))
                    pose = PoseStamped()
                    pose.header.frame_id = "map"
                    pose.pose.position.x = n.x
                    pose.pose.position.y = n.y
                    path.poses.append(pose)
                    n = n.parent
                controls.reverse()
                nodes_in_path.reverse()
                path.poses.reverse()

                for i in range(len(controls)+1):
                    times.append(i*N_STEPS*SAMPLE_TIME)

                break
            else:
                # Seach paths from node
                updateNeighbors(objective, openSet, closedSet, n)

        if not controls:
            controls = [0]
            times = [0, SAMPLE_TIME]
            nodes_in_path = []
            path = Path()
            path.header.frame_id = "map"
            path.poses = []

        if DEBUG:
            dbg.wait()

        if smooth:
            path = smooth_path(objective._environment,nodes_in_path)

        return path

if __name__ == '__main__':
    rospy.init_node('AStar_server')
    server = AStarAction(rospy.get_name())
    rospy.spin()
