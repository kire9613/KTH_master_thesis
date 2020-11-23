# Python standard library
from math import cos, sin, atan2, fabs, sqrt, pi

# Numpy
import numpy as np

from geometry_msgs.msg import PoseStamped, Quaternion
from sensor_msgs.msg import LaserScan
from map_msgs.msg import OccupancyGridUpdate

from grid_map import GridMap
import matplotlib.pyplot as plt
from copy import deepcopy

class Mapping:
    """
    Map handeling class
    Functions:  Provide map updates from lidar scans
                Inflate map
    """

    def __init__(self, polygon_space, unknown_space, free_space, c_space, occupied_space,
                 radius, optional=None):
        # Map values
        self.polygon_space = polygon_space
        self.unknown_space = unknown_space
        self.free_space = free_space
        self.c_space = c_space
        self.occupied_space = occupied_space
        self.allowed_values_in_map = {"self.unknown_space": self.unknown_space,
                                      "self.free_space": self.free_space,
                                      "self.c_space": self.c_space,
                                      "self.occupied_space": self.occupied_space}
        self.radius = radius # Inflate radius
        self.__optional = optional

    def get_yaw(self, q):
        """Returns the Euler yaw from a quaternion.
        :type q: Quaternion
        """
        return atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))

    def raytrace(self, start, end):
        """Returns all cells in the grid map that has been traversed
        from start to end, including start and excluding end.
        start = (x, y) grid map index
        end = (x, y) grid map index
        """
        (start_x, start_y) = start
        (end_x, end_y) = end
        x = start_x
        y = start_y
        (dx, dy) = (fabs(end_x - start_x), fabs(end_y - start_y))
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

        traversed = []
        for i in range(0, int(n)):
            traversed.append((int(x), int(y)))

            if error > 0:
                x += x_inc
                error -= dy
            else:
                if error == 0:
                    traversed.append((int(x + x_inc), int(y)))
                y += y_inc
                error += dx

        return traversed

    def add_to_map(self, grid_map, x, y, value, map_info, inflate=False):
        """Adds value to index (x, y) in grid_map if index is in bounds.
        Returns weather (x, y) is inside grid_map or not.
        """
        if value not in self.allowed_values_in_map.values():
            raise Exception("{0} is not an allowed value to be added to the map. "
                            .format(value) + "Allowed values are: {0}. "
                            .format(self.allowed_values_in_map.keys()) +
                            "Which can be found in the '__init__' function.")

        if self.is_in_bounds(grid_map, x, y, map_info):
            if not grid_map[y,x] == self.polygon_space or grid_map[y,x] == self.c_space:
                grid_map[y,x] = value
                if inflate:
                    for xp in range(-self.radius,self.radius+1):
                        for yp in range(-self.radius,self.radius+1):
                            if self.is_in_bounds(grid_map, x+xp, y+yp, map_info):
                                if not grid_map[y+yp,x+xp] == self.occupied_space:
                                    if sqrt(xp**2+yp**2) <= self.radius:
                                        grid_map[y+yp,x+xp] = self.c_space

                return True
        return False

    def is_in_bounds(self, grid_map, x, y, map_info):
        """Returns weather (x, y) is inside grid_map or not."""
        width = map_info[0]
        height = map_info[1]

        if x >= 0 and x < width:
            if y >= 0 and y < height:
                return True
        return False

    def update_map(self, grid_map, i_grid_map, pose, scan, map_info):
        """
        Create OccupancyGridUpdate from lidar scan info
        Pose in map coordinates, type geometry_msgs.msg PoseStamped
        Scan of type sensor_msgs.msg LaserScan
        Map of type OccupancyGrid
        map_inf = [width, height, x_origin, y_origin, resolution]
        """

        #print("update_map")

        origin_x = map_info[2]
        origin_y = map_info[3]
        resolution = map_info[4]

        # Adjust lidar pos. in relation to car
        #pose.pose.position.y += 0.2#0.39
        #pose.pose.position.x += 0.4#0.598

        # Current yaw of the robot
        robot_yaw = self.get_yaw(pose.pose.orientation)
        robot_x = pose.pose.position.x
        robot_y = pose.pose.position.y

        x_robot_map = robot_x - origin_x
        y_robot_map = robot_y - origin_y
        x_index_s = int(x_robot_map/resolution)
        y_index_s = int(y_robot_map/resolution)

        x_index_list = []
        y_index_list = []

        angle = scan.angle_min
        scan_index = 0
        num_measures = len(scan.ranges)
        obs_ind_list = []
        while scan_index < num_measures:
            # angle
            range = scan.ranges[scan_index]
            if range > scan.range_min and range < scan.range_max/2 and angle < pi/4 and angle > -pi/4:
                # coordinates in scan frame
                x_scan_p = range * cos(angle)
                y_scan_p = range * sin(angle)

                # rotational transformation of scan coordinates
                x_scan = x_scan_p * cos(robot_yaw) - y_scan_p * sin(robot_yaw)
                y_scan = x_scan_p * sin(robot_yaw) + y_scan_p * cos(robot_yaw)

                # translational transformation of scan coordinates
                x_scan_world = robot_x + x_scan
                y_scan_world = robot_y + y_scan

                x_scan_map = x_scan_world - origin_x
                y_scan_map = y_scan_world - origin_y
                x_index_e = int(x_scan_map/resolution)
                y_index_e = int(y_scan_map/resolution)

                # Update free cells in map
                free_cells = self.raytrace((x_index_s,y_index_s), (x_index_e,y_index_e))
                for cell in free_cells:
                    if self.is_in_bounds(grid_map, cell[0],cell[1], map_info):
                         self.add_to_map(grid_map, cell[0], cell[1], self.free_space, map_info)
                         self.add_to_map(i_grid_map, cell[0], cell[1], self.free_space, map_info)
                         x_index_list.append(cell[0])
                         y_index_list.append(cell[1])

                obs_ind_list.append((x_index_e,y_index_e))

            angle += scan.angle_increment
            scan_index += 1

        # Update occupied cells in map
        for obs in obs_ind_list:
            if self.is_in_bounds(grid_map, obs[0],obs[1], map_info):
                self.add_to_map(grid_map, obs[0], obs[1], self.occupied_space, map_info)
                self.add_to_map(i_grid_map, obs[0], obs[1], self.occupied_space, map_info,inflate=True)
                x_index_list.append(obs[0])
                y_index_list.append(obs[1])

        x_index_list.sort()
        y_index_list.sort()
        min_x = x_index_list[0]
        max_x = x_index_list[-1]
        min_y = y_index_list[0]
        max_y = y_index_list[-1]

        # Only get the part that has been updated
        update = OccupancyGridUpdate()
        # The minimum x index in 'grid_map' that has been updated
        update.x = min_x
        #print("x_min:" + str(update.x))
        # The minimum y index in 'grid_map' that has been updated
        update.y = min_y
        #print("y_min: " + str(update.y))
        # Maximum x index - minimum x index + 1
        update.width = max_x - min_x + 1
        # Maximum y index - minimum y index + 1
        update.height = max_y - min_y + 1
        # The map data inside the rectangle, in row-major order.
        update.data = []

        slice = grid_map[min_y:(max_y+1), min_x:(max_x+1)]
        update.data = slice.reshape((update.width*update.height,)).tolist()

        i_update = deepcopy(update)
        slice = i_grid_map[min_y:(max_y+1), min_x:(max_x+1)]
        i_update.data = slice.reshape((i_update.width*i_update.height,)).tolist()

        return grid_map, i_grid_map, update, i_update
