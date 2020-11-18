#!/usr/bin/env python2
from math import exp, fabs, atan2, fmod, pi, hypot
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
from sets import Set


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
    for _ in range(0, int(n)):
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


class RRTNode:
    def __init__(self, node_id, position):
        self._node_id = node_id

        self._position = position

        self._parent = None
        self._children = []

        self._cost = None
        self.__cost_parent = None

        self._score = None
        self.__score_parent = None

        self._gain = None

    def __del__(self):
        self.set_parent(None)

        for child in self._children:
            child.set_parent(None)

        del self._children[:]

    def get_id(self):
        return self._node_id

    def set_parent(self, new_parent):
        if self.has_parent():
            self._parent.erase_child(self)

        if new_parent is not None:
            new_parent.add_child(self)

        self._parent = new_parent

    def get_parent(self):
        return self._parent

    def has_parent(self):
        return self._parent is not None

    def add_child(self, new_child):
        self._children.append(new_child)

    def erase_child(self, child):
        self._children.remove(child)

    def get_children(self):
        return self._children

    def get_num_children(self):
        return len(self._children)

    def has_children(self):
        return 0 < len(self._children)

    def cost(self):
        if not self.has_parent():
            self._cost = 0
            return self._cost

        if self.__cost_parent is not None and self.__cost_parent == self._parent:
            return self._cost

        self._cost = self.cost_with_parent(self.get_parent())

        self.__cost_parent = self._parent

        return self._cost

    def cost_with_parent(self, parent):
        return parent.cost() + self.distance(parent)

    def constrain_angle(self, angle):
        angle = fmod(angle + pi, 2 * pi)
        if angle < 0:
            angle += 2 * pi
        return angle - pi

    def rad_bounded(self, value):
        value = fmod(value + pi, 2 * pi)
        if 0 > value:
            value += 2 * pi
        return value - pi

    def distance(self, node):
        if not node.has_parent():
            return np.linalg.norm(node.get_position() - self._position)
        else:
            node_position = node.get_position()
            node_parent_position = node.get_parent().get_position()
            node_yaw = atan2(node_position[0][1] - node_parent_position[0]
                             [1], node_position[0][0] - node_parent_position[0][0])
            yaw = atan2(self._position[0][1] - node_position[0]
                        [1], self._position[0][0] - node_position[0][0])
            yaw_diff = self.rad_bounded(yaw - node_yaw)
            return hypot(np.linalg.norm(node.get_position() - self._position), yaw_diff)

    def set_position(self, position):
        self._position = position

    def get_position(self):
        return self._position

    def get_path(self, frame_id, grid_map):
        path = []

        current_node = self
        while current_node is not None:
            if not current_node.has_parent():
                break

            position = current_node.get_position()

            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.pose.orientation.w = 1
            pose.pose.position.x = position[0][0]
            pose.pose.position.y = position[0][1]

            parent_position = current_node.get_parent().get_position()

            q = quaternion_from_euler(0, 0, atan2(
                position[0][1] - parent_position[0][1], position[0][0] - parent_position[0][0]))

            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]

            path.append(pose)
            current_node = current_node.get_parent()

        position = current_node.get_position()
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.pose.orientation.w = 1
        pose.pose.position.x = position[0][0]
        pose.pose.position.y = position[0][1]
        path.append(pose)

        last = path[-1]
        path = self.smooth_path(path, grid_map)
        path.append(last)
        return path

    def smooth_path(self, path, grid_map):
        new_path = []

        ok_i = [1]
        new_path_exists = False

        for i in range(1,len(path)):
            start_node_map_coord = [(path[0].pose.position.x - grid_map.info.origin.position.x) / grid_map.info.resolution,
                                (path[0].pose.position.y - grid_map.info.origin.position.y) / grid_map.info.resolution]
            end_node_map_coord = [(path[i].pose.position.x - grid_map.info.origin.position.x) / grid_map.info.resolution,
                                (path[i].pose.position.y - grid_map.info.origin.position.y) / grid_map.info.resolution]
            start = np.array([start_node_map_coord], dtype=np.float)
            end = np.array([end_node_map_coord], dtype=np.float)
            t = raytrace((start[0][0], start[0][1]), (end[0][0], end[0][1]))
            path_ok = True
            if 1 < len(t):
                for (t_x, t_y) in t[1:]:
                    if 80 < grid_map.data[t_y * grid_map.info.width + t_x]: # C-space = 90 , occupied = 100, polygons = 120
                        path_ok = False

            # Check last node
            if 0 != grid_map.data[int(end[0][1]) * grid_map.info.width + int(end[0][0])]:
                path_ok = False

            if path_ok:
                new_path_exists = True
                ok_i.append(i)

        if ok_i[-1] == len(path)-1:
            # insert first node
            pose = PoseStamped()
            pose.pose.orientation.w = 1
            pose.pose.position.x = path[0].pose.position.x
            pose.pose.position.y = path[0].pose.position.y
            new_path.append(pose)

            # insert next node
            pose = PoseStamped()
            pose.pose.orientation.w = 1
            pose.pose.position.x = path[ok_i[-1]].pose.position.x
            pose.pose.position.y = path[ok_i[-1]].pose.position.y
            new_path.append(pose)

            return new_path
        elif ok_i[-1]+1 == len(path)-1:
            # insert first node
            pose = PoseStamped()
            pose.pose.orientation.w = 1
            pose.pose.position.x = path[0].pose.position.x
            pose.pose.position.y = path[0].pose.position.y
            new_path.append(pose)

            # insert next node
            pose = PoseStamped()
            pose.pose.orientation.w = 1
            pose.pose.position.x = path[ok_i[-1]].pose.position.x
            pose.pose.position.y = path[ok_i[-1]].pose.position.y
            new_path.append(pose)

            # add last node
            pose = PoseStamped()
            pose.pose.orientation.w = 1
            pose.pose.position.x = path[-1].pose.position.x
            pose.pose.position.y = path[-1].pose.position.y
            new_path.append(pose)

            return new_path
        else:
            # insert first node
            pose = PoseStamped()
            pose.pose.orientation.w = 1
            pose.pose.position.x = path[0].pose.position.x
            pose.pose.position.y = path[0].pose.position.y
            new_path.append(pose)

            new_path += self.smooth_path(path[ok_i[-1]:-1], grid_map)

        return new_path
