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
from team4_project.mapping2.updatemap import UpdateMap

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

def get_path(grid, start, goal):
    global grid_map
    grid_map = grid
    return get_next_goal(start, goal)

def map_callback(msg):
    global grid_map
    grid_map = msg

    print("Obstacles loaded, start plannig...")

    r = 0.05

    #start = [-2.33,-7.09]
    start =[-1.76, -6]
    #goal = [10.48,11.71]
    #goal = [8.25,13.2]
    goal= [1.81,6.09]

    path = get_next_goal(start, goal)

    plot_map = np.array(grid_map.data).reshape(880,721)
    plt.imshow(plot_map)

    xv = []
    yv = []
    for i in range(0,len(path)):
        xv.append((path[i].pose.position.x+17.581444)/0.05)
        yv.append((path[i].pose.position.y+22.876441)/0.05)
        if i == 0:
            x1 = (path[i].pose.position.x+17.581444)/0.05
            y1 = (path[i].pose.position.y+22.876441)/0.05
        else:
            x2 = (path[i].pose.position.x+17.581444)/0.05
            y2 = (path[i].pose.position.y+22.876441)/0.05
            plt.plot([x1,x2], [y1,y2], "r-")
            x1 = x2
            y1 = y2

    plt.show()

def main():
    rospy.init_node("planner")

    map_service = UpdateMap()
    map = map_service.get_map()
    map_info = map_service.get_map_info()
    og = OccupancyGrid()
    og.info = map_info
    og.data = map.reshape(880*721).tolist()
    map_callback(og)

    rospy.spin()

if __name__ == '__main__':
    print(__file__ + " start!!")
    main()
    print(__file__ + " Done!!")
