# Python standard library
from math import cos, sin, atan2, fabs, sqrt

# Numpy
import numpy as np

from geometry_msgs.msg import PoseStamped, Quaternion
from sensor_msgs.msg import LaserScan
from map_msgs.msg import OccupancyGridUpdate

from grid_map import GridMap

class Mapping:
    """
    Map handeling class
    Functions:  Provide map updates from lidar scans
                Inflate map
    """

    def __init__(self, unknown_space, free_space, c_space, occupied_space,
                 radius, optional=None):
        # Map values
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

    def add_to_map(self, grid_map, x, y, value):
        """Adds value to index (x, y) in grid_map if index is in bounds.
        Returns weather (x, y) is inside grid_map or not.
        """
        if value not in self.allowed_values_in_map.values():
            raise Exception("{0} is not an allowed value to be added to the map. "
                            .format(value) + "Allowed values are: {0}. "
                            .format(self.allowed_values_in_map.keys()) +
                            "Which can be found in the '__init__' function.")

        if self.is_in_bounds(grid_map, x, y):
            grid_map[x, y] = value
            return True
        return False

    def is_in_bounds(self, grid_map, x, y):
        """Returns weather (x, y) is inside grid_map or not."""
        if x >= 0 and x < grid_map.get_width():
            if y >= 0 and y < grid_map.get_height():
                return True
        return False

    def update_map(self, grid_map, pose, scan):
        """
        Create OccupancyGridUpdate from lidar scan info
        Pose in map coordinates, type geometry_msgs.msg PoseStamped
        Scan of type sensor_msgs.msg LaserScan
        """

        print("update_map")

        # # Current yaw of the robot
        # robot_yaw = self.get_yaw(pose.pose.orientation)
        # robot_x = pose.pose.position.x
        # robot_y = pose.pose.position.y
        # # The origin of the map [m, m, rad]. This is the real-world pose of the
        # # cell (0,0) in the map.
        # origin = grid_map.get_origin()
        # origin_x = origin.position.x
        # origin_y = origin.position.y
        # # The map resolution [m/cell]
        # resolution = grid_map.get_resolution()
        #
        # x_robot_map = robot_x - origin_x
        # y_robot_map = robot_y - origin_y
        # x_index_s = int(x_robot_map/resolution)
        # y_index_s = int(y_robot_map/resolution)
        #
        # x_index_list = []
        # y_index_list = []
        #
        # """
        # Fill in your solution here
        # """
        #
        # angle = scan.angle_min
        # scan_index = 0
        # num_measures = len(scan.ranges)
        # obs_ind_list = []
        # while scan_index < num_measures:
        #     # angle
        #     range = scan.ranges[scan_index]
        #     if range > scan.range_min and range < scan.range_max:
        #         # coordinates in scan frame
        #         x_scan_p = range * cos(angle)
        #         y_scan_p = range * sin(angle)
        #
        #         # rotational transformation of scan coordinates
        #         x_scan = x_scan_p * cos(robot_yaw) - y_scan_p * sin(robot_yaw)
        #         y_scan = x_scan_p * sin(robot_yaw) + y_scan_p * cos(robot_yaw)
        #
        #         # translational transformation of scan coordinates
        #         x_scan_world = robot_x + x_scan
        #         y_scan_world = robot_y + y_scan
        #
        #         x_scan_map = x_scan_world - origin_x
        #         y_scan_map = y_scan_world - origin_y
        #         x_index_e = int(x_scan_map/resolution)
        #         y_index_e = int(y_scan_map/resolution)
        #
        #         free_cells = self.raytrace((x_index_s,y_index_s), (x_index_e,y_index_e))
        #         for cell in free_cells:
        #             if self.is_in_bounds(grid_map, cell[0],cell[1]):
        #               self.add_to_map(grid_map, cell[0], cell[1], self.free_space)
        #               x_index_list.append(cell[0])
        #               y_index_list.append(cell[1])
        #
        #         obs_ind_list.append((x_index_e,y_index_e))
        #
        #     angle += scan.angle_increment
        #     scan_index += 1
        #
        # for obs in obs_ind_list:
        #     if self.is_in_bounds(grid_map, obs[0],obs[1]):
        #         self.add_to_map(grid_map, obs[0], obs[1], self.occupied_space)
        #         x_index_list.append(obs[0])
        #         y_index_list.append(obs[1])
        #
        # x_index_list.sort()
        # y_index_list.sort()
        min_x = 365 #int((-8.24+17.581444)/0.05)
        max_x = 365+60 #min_x + 60
        min_y = 440 #int((-15.77+22.876441)/0.05)
        max_y = 500 #min_y + 60

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

        x = min_x
        y = min_y
        while y <= max_y:
            while x <= max_x:
                update.data.append(100) #grid_map[x,y]
                # if not polygon...
                x += 1
            x = min_x
            y += 1

        # Return the updated map together with only the
        # part of the map that has been updated
        return grid_map, update

    # def inflate_map(self, grid_map):
    #     """
    #     needs to read map from /map topic to get map with all updates!!!!!!!!!!!!!!!!
    #     """
    #
    #     x,y = 0,0
    #     while y < len(grid_map[:,0]):
    #         while x < len(grid_map[0,:]):
    #             if grid_map[x,y] == self.occupied_space:
    #                 for xp in range(-self.radius,self.radius+1):
    #                     for yp in range(-self.radius,self.radius+1):
    #                         if self.is_in_bounds(grid_map, x+xp, y+yp):
    #                             if not grid_map[x+xp,y+yp] == self.occupied_space:
    #                                 if sqrt(xp**2+yp**2) <= self.radius:
    #                                     self.add_to_map(grid_map, x+xp, y+yp, self.c_space)
    #             x += 1
    #         x = 1
    #         y +=1
    #
    #     """
    #     Fill in your solution here
    #     """
    #
    #     # Return the inflated map
    #     return grid_map
