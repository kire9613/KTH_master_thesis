#!/usr/bin/env python

"""
Hybrid A* algorithm performing path planning.
Using content in dublins.py
To be used as a ROS action service
Input: ROS msg of type AStarAction: string frame_id
                                    float32 x0 (plan from position (x0,y0) and heading theta0)
                                    float32 y0
                                    float32 theta0
                                    float32 xt (plan to position (xt,yt) and heading thetat)
                                    float32 yt
                                    float32 thetat
                                    bool smooth (smooth path True/False)
Output: ROS msg of type AStarResult containing a Path pobject with the planned path.
(Defined in folder /team4_msgs)
"""

import rospy
from dublins import *
from Queue import PriorityQueue
from nav_msgs.msg import OccupancyGrid
from math import hypot, pi, ceil
import numpy
import team4_msgs.msg
import actionlib
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from team4_project.mapping2.updatemap import UpdateMap

# Used only for debug plotting
N_GRID_X = 40
N_GRID_Y = N_GRID_X//2

# AStar params
DELTA_T = 0.040     # time step, dt_max = 0.05 given max_vel = 1 m/s
GOAL_RADIUS = 0.1   # Accepted deviation from goal position
GOAL_ANGLE = pi   # Accepted deviation from goal theta
ANGLES = [-pi/4,-pi/8,0,pi/8,pi/4] # Search path in directions
N_HEADINGS = 6 # len(angles)
N_STEPS = 10        # number of steps with same control signal
OBJECTIVE = []      # planning objective, initialized in solution
GRID_SIZE_X = 0     # map resolution, initialized in solution
GRID_SIZE_Y = 0     # map resolution, initialized in solution

# Dynamic illustration of planning procedure for debugging.
DEBUG = False
if DEBUG:
    import ass3_debug as dbg

# For debugging
plotter = None
counter = 0

class Node:
    """
    Nodes in A* search-treee
    """

    def __init__(self, theta, x, y, cost, parent, control):

        self.xd = int(x/GRID_SIZE_X) # discretized cord.
        self.yd = int(y/GRID_SIZE_Y)
        self.theta = theta # heading
        self.x = x
        self.y = y
        self.cost = cost # cost to go from root
        self.parent = parent # parent node
        self.control = control # steering angle

        # Make sure theta is in [0, 2pi]
        while theta <= 0:
            theta += 2*pi
        while theta > 2*pi:
            theta -= 2*pi

        # Discretizise heading
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
    """ Check for collisions for node n,
    obj is the action request msg defining planning objective
    """

    status = obj._environment.safe(n.x,n.y)
    if status:
        return False
    else:
        return True

def isObstacle_2(obj, x,y):
    """ Check for collisions for node n,
    obj is the action request msg defining planning objective
    """

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

    # Iterate over angles used in path search
    for angle in ANGLES:
        xn, yn, thetan = n.x, n.y, n.theta
        safe = True
        for i in range(N_STEPS):
            xnew, ynew, thetan = step(obs, xn, yn, thetan, angle, DELTA_T)
            """ check all points transversed when going from (xn,yn) to (xnew,ynew) """
            if isObstacle_2(obs,xnew,ynew):
                """ If collision - discard """
                safe = False
                break
            xn = xnew
            yn = ynew
        cost = 1 if angle == 0 else 1.5

        # Convert position to grid indices
        y_index = int((yn - obs._environment.origin_y)/obs._environment.resolution)
        x_index = int((xn - obs._environment.origin_x)/obs._environment.resolution)

        # Create a slice of the map centered around the
        # node to search for non-free space in
        map = np.array(obs._environment.map)
        # Search in a square of size 2*obstacle_square_size
        obstacle_square_size = int(round(0.5/obs._environment.resolution))
        miny = max(0, y_index-obstacle_square_size)
        maxy = min(map.shape[0], y_index+obstacle_square_size+1)
        minx = max(0, x_index-obstacle_square_size)
        maxx = min(map.shape[1], x_index+obstacle_square_size+1)
        map_slice = map[miny:maxy, minx:maxx]

        node_pos_in_slice = np.array([y_index-miny, x_index-minx]).reshape((2,1))

        # Create a matrix of all indices that correspond to
        # any kind of non-free space
        occupied_pos = np.concatenate(np.where(map_slice >= 100)).reshape((2, -1))
        if np.prod(occupied_pos.shape) > 0:
            # Add a cost that is a function of the distance to the closest non-free cell
            distance = np.min(np.linalg.norm(occupied_pos - node_pos_in_slice, axis=0))
            # Avoid division by zero
            if distance == 0.0:
                distance = 0.001
            cost += 1.5/distance**2

        # Create new node
        nn = Node(thetan, xn, yn, n.cost+cost, n, angle)

        # Check node status
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
    """ Creates Path object from path (list) """

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
    """
    Smoothing path:
    Checks if it exists is a straight line without collisions between nodes.
    If no collison - replace old path between nodes with a straight line.
    """

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
    # create messages that are used to publish feedback/result from action server
    _feedback = team4_msgs.msg.AStarFeedback()
    _result = team4_msgs.msg.AStarResult()

    def __init__(self, name):
        self._action_name = name

        # Functions to get map
        self._map_srv = UpdateMap()
        self._map = self._map_srv.get_inflated_map()
        self._mapinfo = self._map_srv.get_map_info()

        # Init map
        oc_map = rospy.wait_for_message('/map', OccupancyGrid)
        self._map = np.array(oc_map.data).reshape(oc_map.info.height,oc_map.info.width)
        self._mapinfo = [oc_map.info.width, oc_map.info.height, oc_map.info.resolution, oc_map.info.origin.position.x, oc_map.info.origin.position.y]

        # Define action server
        self._as = actionlib.SimpleActionServer(self._action_name, team4_msgs.msg.AStarAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def execute_cb(self, goal):
        """
        Envoked when requesting path from action server.
        """

        # helper variables
        r = rospy.Rate(1)
        self.success = True

        # publish info to the console for the user
        rospy.loginfo('%s: Executing, searching for path from (%f,%f) to (%f,%f)' % (self._action_name, goal.x0, goal.y0, goal.xt, goal.yt))

        self._map = self._map_srv.get_inflated_map() # get latest map
        # Init anvironment (map of environment) and planning objective
        env = Environment(self._map, self._mapinfo)
        objective = Objective(goal.xt, goal.yt, goal.thetat, goal.x0, goal.y0, goal.theta0, env)

        # Search for path
        path = self.run_astar(objective,goal.smooth)

        if self.success:
            self._result.path = path
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

    def run_astar(self, objective, smooth=False):
        global GRID_SIZE_X, GRID_SIZE_Y, N_STEPS, OBJECTIVE, DELTA_T, plotter, counter

        openSet = PriorityQueue() # Set holding active nodes to search more path from
        closedSet = {} # Set holding dead nodes

        GRID_SIZE_X = objective._environment.resolution
        GRID_SIZE_Y = objective._environment.resolution
        nodes_in_path = []
        OBJECTIVE = objective # defines task to solve

        if DEBUG:
            dbg.plotMap(objective, N_GRID_X, N_GRID_Y, OBJECTIVE)
            dbg.show(objective)
            plotter = dbg.TreePlot(objective._environment, N_STEPS)

        # Add start position node to set
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

            # If we have reached the goal
            if hypot(n.x-objective.xt, n.y-objective.yt) < GOAL_RADIUS and numpy.fabs(n.theta - objective.thetat) < GOAL_ANGLE:
                if DEBUG:
                    plotter.markBestPath(n)
                # Reconstruct control signals
                nodes_in_path = []
                path = Path()
                path.header.frame_id = "map"
                path.poses = []
                while n.parent is not None:
                    nodes_in_path.append((n.x,n.y))
                    pose = PoseStamped()
                    pose.header.frame_id = "map"
                    pose.pose.position.x = n.x
                    pose.pose.position.y = n.y
                    path.poses.append(pose)
                    n = n.parent
                nodes_in_path.reverse()
                path.poses.reverse()
                break
            else:
                # Seach paths from node
                updateNeighbors(objective, openSet, closedSet, n)

        if nodes_in_path == []:
            # Return empty path
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
