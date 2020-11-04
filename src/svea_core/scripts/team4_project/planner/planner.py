#!/usr/bin/env python2
import rospy
import actionlib
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import Marker, MarkerArray
import irob_assignment_1.msg
# from sensor import Sensor
from rrt import RRT
import numpy as np
import tf2_ros
from math import pi, atan2, sqrt
from tf.transformations import quaternion_from_euler
import re
import matplotlib.pyplot as plt

max_nodes = 1000
extension_range = 1.0
radius = 0.105

def get_next_goal(start, goal):
    global grid_map

    bbx_min = [grid_map.info.origin.position.x,
               grid_map.info.origin.position.y]
    bbx_max = [bbx_min[0] + (grid_map.info.width * grid_map.info.resolution),
               bbx_min[1] + (grid_map.info.height * grid_map.info.resolution)]

    root_position = np.array([[start[0], start[1]]])
    target_position = np.array([[goal[0], goal[1]]])

    tree = RRT(root_position, target_position, bbx_min, bbx_max, radius, extension_range)

    best_dist = [None,float("inf")]
    success = False
    i = 0
    while i < max_nodes and not rospy.is_shutdown() and success == False:

        node = tree.expand_tree(grid_map)
        if node is None:
            continue
        i += 1

        dist = sqrt((goal[0]-node._position[0][0]) ** 2 + (goal[1]-node._position[0][1]) ** 2)
        if dist < best_dist[1]:
            best_dist = [node,dist]
            if dist < 0.05:
                return node.get_path(grid_map.header.frame_id, grid_map)

    return best_dist[0].get_path(grid_map.header.frame_id, grid_map)

def inflate_map(map, radius):
    """For C only!
    Inflate the map with self.c_space assuming the robot
    has a radius of self.radius.

    Returns the inflated grid_map.

    Inflating the grid_map means that for each self.occupied_space
    you calculate and fill in self.c_space. Make sure to not overwrite
    something that you do not want to.


    You should use:
        self.c_space  # For C space (inflated space).
        self.radius   # To know how much to inflate.

        You can use the function add_to_map to be sure that you add
        values correctly to the map.

        You can use the function is_in_bounds to check if a coordinate
        is inside the map.

    :type grid_map: GridMap
    """

    #for (t_x, t_y) in t[1:]:
    #    if 0 != grid_map.data[t_y * grid_map.info.width + t_x]:

    grid_map = map

    x = -23
    y = -17
    while y < 17:
        while x < 23:
            if is_in_bounds(grid_map, x, y):
                if grid_map.data[y * grid_map.info.width + x] == 100:
                    for xp in [-0.05, 0.05]:
                        for yp in [-0.05, 0.05]:
                            if is_in_bounds(grid_map, x+xp, y+yp):
                                if not grid_map.data[(y+yp) * grid_map.info.width + x+xp] == 100:
                                    grid_map.data[y+yp * grid_map.info.width + x+xp] = 50
                x += 0.05
        x = -23
        y += 0.05

    """
    Fill in your solution here
    """

    # Return the inflated map
    return grid_map

def is_in_bounds(grid_map, x, y):
    """Returns weather (x, y) is inside grid_map or not."""
    if grid_map.data[y * grid_map.info.width + x]:
            return True
    return False

def map_callback(msg):
    global grid_map
    grid_map = msg

    r = 0.05
    #grid_map = inflate_map(grid_map, r)

    start = [-2.33,-7.09]
    goal = [10.48,11.71]

    path = get_next_goal(start, goal)

    print(str(path))
    obstacleList = []

    f = open("/home/caroline/EL2425/svea_starter/src/svea_core/scripts/team4_project/obstacles.txt", "r")
    #f = open("/home/caroline/Documents/test.txt", "r")

    for line in f:
        #print("new obs")
        lp = []
        reg = re.compile("\[[+-]?[0-9]+.[0-9]+,[ ][+-]?[0-9]+.[0-9]+\]")
        obs_coord = re.findall(reg, line)
        for p in obs_coord:
            o = p.strip("[").strip("]").replace(" ", "")
            o = o.split(",")
            lp.append((o))
        #print("nodes")
        #print(len(lp))
        #print(lp)
        for i in range(0,len(lp)):
            #print("i:")
            #print(i)
            if i == len(lp)-1:
                xc = np.linspace(float(lp[0][0]), float(lp[i][0]))
                yc = np.linspace(float(lp[0][1]), float(lp[i][1]))
                #print(lp[i-1][0] + " , " + lp[i-1][1] + " -> " + lp[i][0] + " , " + lp[i][1])
                for k in range(0,len(xc)):
                    x = xc[k]
                    y = yc[k]
                    obstacleList.append((x,y,0.1))
            else:
                xc = np.linspace(float(lp[i+1][0]), float(lp[i][0]))
                yc = np.linspace(float(lp[i+1][1]), float(lp[i][1]))
                #print(lp[i-1][0] + " , " + lp[i-1][1] + " -> " + lp[i][0] + " , " + lp[i][1])
                for k in range(0,len(xc)):
                    x = xc[k]
                    y = yc[k]
                    obstacleList.append((x,y,0.05))
    f.close()

    print("Obstacles loaded, start plannig...")

    #"""
    plt.figure()
    plt.clf()
    for (ox, oy, size) in obstacleList:
        #print("plot")
        plt.plot(ox, oy, "ok", ms=30 * size)
    plt.axis([-23, 23, -18, 18])
    print("show plot")
    #plt.show()
    #"""

    plt.plot([start[0],goal[0]], [start[1],goal[1]], "rx")
    #plt.show()

    print("plot start goal")
    xv = []
    yv = []
    for i in range(0,len(path.poses)):
        xv.append(path.poses[i].pose.position.x)
        yv.append(path.poses[i].pose.position.y)
        if i == 0:
            x1 = path.poses[i].pose.position.x
            y1 = path.poses[i].pose.position.y
        else:
            x2 = path.poses[i].pose.position.x
            y2 = path.poses[i].pose.position.y
            #plt.plot([x1,x2], [y1,y2], "r-")
            #plt.show()
            x1 = x2
            y1 = y2

    print("plot done")
    plt.plot(xv, yv, "r-")
    plt.show()

def main():
    print("potential_field_planning start")

    rospy.init_node("planner")

    print("init map subscriber...")
    mapSubscriber = rospy.Subscriber('/map', OccupancyGrid, map_callback)

    rospy.spin()

if __name__ == '__main__':
    print(__file__ + " start!!")
    main()
    print(__file__ + " Done!!")
