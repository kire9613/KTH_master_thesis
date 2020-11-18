#!/usr/bin/env python2
import rospy
import actionlib
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import Marker, MarkerArray
from team4_project.planner.rrt import RRT
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
    # To be completed...

    grid_map = map

    # x = -23
    # y = -17
    # while y < 17:
    #     while x < 23:
    #         if is_in_bounds(grid_map, x, y):
    #             if grid_map.data[y * grid_map.info.width + x] == 100:
    #                 for xp in [-0.05, 0.05]:
    #                     for yp in [-0.05, 0.05]:
    #                         if is_in_bounds(grid_map, x+xp, y+yp):
    #                             if not grid_map.data[(y+yp) * grid_map.info.width + x+xp] == 100:
    #                                 grid_map.data[y+yp * grid_map.info.width + x+xp] = 50
    #             x += 0.05
    #     x = -23
    #     y += 0.05

    return grid_map

def is_in_bounds(grid_map, x, y):
    """Returns weather (x, y) is inside grid_map or not."""
    # To be completed...
    if grid_map.data[y * grid_map.info.width + x]:
            return True
    return False

def map_to_print_format(grid_map):

    map_list = [] # map_list[y][x]
    x = 1
    y = 1
    gm_i = 0
    while y <= 880:
        row = []
        while x <= 721:
            row.append(grid_map.data[gm_i])
            x += 1
            gm_i += 1
        map_list.append(row)
        x = 1
        y += 1

    xv_n = []
    yv_n = []
    xv_0 = []
    yv_0 = []
    xv_100 = []
    yv_100 = []

    j = 0
    i = 0
    x = -18
    y = -22
    while j < 880:
        row = map_list[j]
        while i < 721:
            val = row[i]
            if val == 0:
                xv_0.append(x)
                yv_0.append(y)
            if val == 100:
                xv_100.append(x)
                yv_100.append(y)
            else:
                xv_n.append(x)
                yv_n.append(y)
            i += 1
            x += 0.05
        j += 1
        y += 0.05
        i = 0
        x = -18

    #print(len(xv_n))
    #print(len(xv_100))
    #print(len(xv_0))

    #map = np.array(grid_map.data).reshape(880, 721)
    #print(map)
    #plt.imshow(map[:, :])
    #plt.show()

    plt.figure()

    plt.plot(xv_100,yv_100,"k,")
    plt.plot(xv_n,yv_n,"b,")
    plt.plot(xv_0,yv_0,"r,")
    plt.axis([-19, 19, -23, 23])
    plt.show()

    plt.figure()
    plt.plot(xv_0,yv_0,"r,")
    plt.axis([-19, 19, -23, 23])
    plt.show()

    plt.figure()
    plt.plot(xv_n,yv_n,"b,")
    plt.axis([-19, 19, -23, 23])
    plt.show()

    #return xv_n,yv_n,xv_0,yv_0,xv_100,yv_100
    return map_list

def get_path(grid, start, goal):
    global grid_map
    grid_map = grid
    return get_next_goal(start, goal)

def map_callback(msg):
    global grid_map
    grid_map = msg

    #map_list = map_to_print_format(grid_map)

    obstacleList = []
    f = open("/home/caroline/Documents/svea_starter/src/svea_core/scripts/team4_project/planner/obstacles.txt", "r")
    #f = open("/home/caroline/Documents/test.txt", "r")

    for line in f:
        lp = []
        reg = re.compile("\[[+-]?[0-9]+.[0-9]+,[ ][+-]?[0-9]+.[0-9]+\]")
        obs_coord = re.findall(reg, line)
        for p in obs_coord:
            o = p.strip("[").strip("]").replace(" ", "")
            o = o.split(",")
            lp.append((o))
        for i in range(0,len(lp)):
            if i == len(lp)-1:
                xc = np.linspace(float(lp[0][0]), float(lp[i][0]))
                yc = np.linspace(float(lp[0][1]), float(lp[i][1]))
                for k in range(0,len(xc)):
                    x = xc[k]
                    y = yc[k]
                    obstacleList.append((x,y,0.1))
            else:
                xc = np.linspace(float(lp[i+1][0]), float(lp[i][0]))
                yc = np.linspace(float(lp[i+1][1]), float(lp[i][1]))
                for k in range(0,len(xc)):
                    x = xc[k]
                    y = yc[k]
                    obstacleList.append((x,y,0.05))
    f.close()

    print("Obstacles loaded, start plannig...")

    r = 0.05
    #grid_map = inflate_map(grid_map, r)

    start = [-2.33,-7.09]
    goal = [10.48,11.71]
    goal = [8.25,13.2]

    path = get_next_goal(start, goal)

    #print(str(path))

    plt.figure()
    plt.clf()
    for (ox, oy, size) in obstacleList:
        plt.plot(ox, oy, "ok", ms=30 * size)
    plt.axis([-23, 23, -18, 18])
    #plt.show()

    plt.plot([start[0],goal[0]], [start[1],goal[1]], "rx")
    #plt.show()

    print(str(path))

    xv = []
    yv = []
    for i in range(0,len(path)):
        xv.append(path[i].pose.position.x)
        yv.append(path[i].pose.position.y)
        if i == 0:
            x1 = path[i].pose.position.x
            y1 = path[i].pose.position.y
        else:
            x2 = path[i].pose.position.x
            y2 = path[i].pose.position.y
            #plt.plot([x1,x2], [y1,y2], "r-")
            #plt.show()
            x1 = x2
            y1 = y2

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
