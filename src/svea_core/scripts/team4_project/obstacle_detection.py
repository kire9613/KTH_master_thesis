#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
from svea_msgs.msg import VehicleState
from team4_msgs.msg import Collision
from team4_project.mapping2.updatemap import UpdateMap
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point

import numpy as np
import math

def world_to_grid(info, pos):
    """Convert world coordinate to grid coordinate"""
    x_grid = int((pos[0] - info.origin.position.x)/info.resolution)
    y_grid = int((pos[1] - info.origin.position.y)/info.resolution)
    return np.array([x_grid, y_grid])

def grid_to_world(info, pos):
    """Convert grid coordinate to world coordinate"""
    x_world = pos[0]*info.resolution + info.origin.position.x
    y_world = pos[1]*info.resolution + info.origin.position.y
    return np.array([x_world, y_world])

# Function from code provided by course DD2410
def raytrace(start, end):
    """Returns all cells in the grid map that has been traversed
    from start to end, including start and excluding end.
    start = (x, y) grid map index
    end = (x, y) grid map index
    """
    (start_x, start_y) = start
    (end_x, end_y) = end
    x = start_x
    y = start_y
    (dx, dy) = (math.fabs(end_x - start_x), math.fabs(end_y - start_y))
    n = dx + dy
    x_inc = 1
    if end_x <= start_x:
        x_inc = -1
    y_inc = 1
    if end_y <= start_y:
        y_inc = -1
    error = dx - dy
    dx *= 2
    dy *= 2

    for i in range(0, int(n)):
        yield (int(x), int(y))

        if error > 0:
            x += x_inc
            error -= dy
        else:
            if error == 0:
                yield (int(x + x_inc), int(y))
            y += y_inc
            error += dx

def main():
    rospy.init_node('obstacle_detection')

    map_update = UpdateMap()
    collision_pub = rospy.Publisher('/collision', Collision, queue_size=1)
    vis_pub = rospy.Publisher('/vis_collision', PointStamped, queue_size=1)

    # Run check continuously
    while not rospy.is_shutdown():
        state = rospy.wait_for_message('/state', VehicleState)
        grid = map_update.get_inflated_map()
        path_msg = rospy.wait_for_message('/targets', Path)

	# Check closest point in path
        dx = []
        dy = []
        for poseStamped in path_msg.poses:
            dx.append(state.x-poseStamped.pose.position.x)
            dy.append(state.y-poseStamped.pose.position.y)
        d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
        start_index = d.index(min(d))

        collision_msg = Collision()
        collision_msg.collision = False

        collision = False
        for i in range(start_index, len(path_msg.poses)-1):
            # Raytrace between each pair of targets
            tgt1 = world_to_grid(map_update.get_map_info(), [path_msg.poses[i].pose.position.x, path_msg.poses[i].pose.position.y])
            tgt2 = world_to_grid(map_update.get_map_info(), [path_msg.poses[i+1].pose.position.x, path_msg.poses[i+1].pose.position.y])

            vis = PointStamped()
            vis.header.frame_id = 'map'
            vis.point.x = -999
            vis.point.y = -999
            for point in raytrace(tgt1, tgt2):
                # Grid is stored as (row, col) whereas point
                # is stored as (x, y)
                value = grid[point[1], point[0]]
                # If vlue is not free space or unknown
                if not (value == 0 or value == -1):
                    rospy.loginfo("GridMap value at collision: %d" % value)
                    collision_point = grid_to_world(map_update.get_map_info(), point)

                    collision_msg.collision = True
                    collision_msg.collision_point.x = collision_point[0]
                    collision_msg.collision_point.y = collision_point[1]
                    collision = True

                    vis.point.x = collision_point[0]
                    vis.point.y = collision_point[1]

                    break
            vis_pub.publish(vis)

            if collision:
                break
        collision_pub.publish(collision_msg)

if __name__ == '__main__':
    main()
