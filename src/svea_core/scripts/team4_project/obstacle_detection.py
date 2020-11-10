#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
from svea_msgs.msg import VehicleState
from team4_msgs.msg import Collision

import numpy as np
import math

def world_to_grid(grid, pos):
    """Convert world coordinate to grid coordinate"""
    x_grid = int((pos[0] - grid.info.origin.position.x)/grid.info.resolution)
    y_grid = int((pos[1] - grid.info.origin.position.y)/grid.info.resolution)
    return x_grid, y_grid

def grid_to_world(grid, pos):
    """Convert grid coordinate to world coordinate"""
    x_world = pos[0]*grid.info.resolution + grid.info.origin.position.x
    y_world = pos[1]*grid.info.resolution + grid.info.origin.position.y
    return x_world, y_world

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

    collision_pub = rospy.Publisher('/collision', Collision, queue_size=1)

    # Run check continuously
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        state = rospy.wait_for_message('/SVEA/state', VehicleState)
        grid_msg = rospy.wait_for_message('/map', OccupancyGrid)
        path_msg = rospy.wait_for_message('/targets', Path)

        collision_msg = Collision()
        collision_msg.collision = False
        collision_msg.distance = -1

        grid = np.reshape(grid_msg.data, (grid_msg.info.height, grid_msg.info.width))

        collision = False
        for i in range(len(path_msg.poses)-1):
            # Raytrace between each pair of targets
            tgt1 = world_to_grid(grid_msg, ([path_msg.poses[i].pose.position.x, path_msg.poses[i].pose.position.y]))
            tgt2 = world_to_grid(grid_msg, ([path_msg.poses[i+1].pose.position.x, path_msg.poses[i+1].pose.position.y]))

            for point in raytrace(tgt1, tgt2):
                # Grid is stored as (row, col) whereas point
                # is stored as (x, y)
                value = grid[point[1], point[0]]
                if not (value == 0 or value == -1):
                    collision_point = grid_to_world(grid_msg, point)
                    collision_msg.collision = True
                    collision_msg.distance = math.hypot(collision_point[0]-state.x, collision_point[1]-state.y)
                    collision = True
                    break

            if collision:
                break
        collision_pub.publish(collision_msg)

        if rate.remaining().to_sec() < 0:
            rospy.logwarn('Obstacle detection too slow for sample rate!')
        rate.sleep()

if __name__ == '__main__':
    main()
