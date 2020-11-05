#!/usr/bin/env python2
from rrt_node import RRTNode, raytrace
from rtree import index
import random
import numpy as np
from sklearn import preprocessing
from math import log
import matplotlib.pyplot as plt


class RRT:
    def __init__(self, root_position, target_position, bbx_min, bbx_max, radius, extension_range, ):
        self._bbx_min = bbx_min
        self._bbx_max = bbx_max
        self._radius = radius
        self._extension_range = extension_range
        self._sample_nr = 0
        self._target = target_position

        # Create r-tree
        p = index.Property()
        p.dimension = 2
        self._tree = index.Index(properties=p)

        self._id = 0
        self._root = self.create_new_node(root_position)
        self.insert(self._root, None)

    def sample(self):
        position = np.array([[0, 0]], dtype=np.float)
        for i in range(0, len(position[0])):
            position[0][i] = random.uniform(self._bbx_min[i], self._bbx_max[i])
        if self._sample_nr % 10 == 0:
            position[0][0] = self._target[0][0]
            position[0][1] = self._target[0][1]
        self._sample_nr += 1
        return self.create_new_node(position)

    def create_new_node(self, position):
        node = RRTNode(self._id, position)
        self._id += 1
        return node

    def expand_tree(self, grid_map):
        new_node = self.sample()

        return self.extend(grid_map, new_node)

    def extend(self, grid_map, new_node):
        closest_node = self.get_closest(new_node)
        new_node = self.restrict_distance(new_node, closest_node)

        if self.valid_node(grid_map, new_node, closest_node):
            parent = closest_node
            self.insert(new_node, parent)
            return new_node

        return None

    def valid_node(self, grid_map, node, parent):
        node_position = node.get_position()
        node_map_coord = [(node_position[0][0] - grid_map.info.origin.position.x) / grid_map.info.resolution,
                          (node_position[0][1] - grid_map.info.origin.position.y) / grid_map.info.resolution]

        parent_position = parent.get_position()
        parent_map_coord = [(parent_position[0][0] - grid_map.info.origin.position.x) / grid_map.info.resolution,
                            (parent_position[0][1] - grid_map.info.origin.position.y) / grid_map.info.resolution]

        start = np.array([parent_map_coord], dtype=np.float)
        end = np.array([node_map_coord], dtype=np.float)

        t = raytrace((start[0][0], start[0][1]), (end[0][0], end[0][1]))
        if 1 < len(t):
            for (t_x, t_y) in t[1:]:
                if 0 != grid_map.data[t_y * grid_map.info.width + t_x]:
                    return False

        # Check last node
        if 0 != grid_map.data[int(end[0][1]) * grid_map.info.width + int(end[0][0])]:
            return False

        return True

    def distance_to_line_segment(self, start, end, point):
        length = np.linalg.norm(end - start)
        np.linalg.norm

        if 0 == length:
            return np.linalg.norm(start - point)

        t = max(0, min(1, np.dot(np.reshape(point - start, 2),
                                 np.reshape(end - start, 2)) / (length ** 2)))
        projection = start + (t * (end - start))

        return np.linalg.norm(point - projection)

    def insert(self, node, parent):
        self.update(node, parent)
        position = node.get_position()
        self._tree.insert(
            node.get_id(), (position[0][0], position[0][1]), obj=node)

    def update(self, node, new_parent):
        node.set_parent(new_parent)

    def erase(self, node):
        if node.has_children():
            pass  # TODO: Throw error

        node.set_parent(None)
        # Remove from tree
        position = node.get_position()
        self._tree.delete(
            node.node_id, (position[0][0], position[0][1]))

    def get_closest(self, node):
        # Get the closest node
        position = node.get_position()
        hits = list(self._tree.nearest(
            (position[0][0], position[0][1]), 1, objects=True))
        assert(1 <= len(hits))
        return hits[0].object

    def get_near(self, node):
        # Get nodes that are nearby
        position = node.get_position()
        bbx = (position[0][0] - self._extension_range, position[0][1] - self._extension_range,
               position[0][0] + self._extension_range, position[0][1] + self._extension_range)
        hits = self._tree.intersection(bbx, objects=True)
        nodes = []
        for hit in hits:
            nodes.append(hit.object)
        return nodes

    def restrict_distance(self, node, closest_node):
        origin = closest_node.get_position()
        direction = node.get_position() - origin

        if np.linalg.norm(direction) > self._extension_range:
            direction = self._extension_range * \
                preprocessing.normalize(direction, norm="l2")
            node.set_position(origin + direction)

        return node

    def get_nodes(self, grid_map):
        bbx = (grid_map.info.origin.position.x, grid_map.info.origin.position.y, grid_map.info.origin.position.x + (grid_map.info.width *
                                                                                                                    grid_map.info.resolution), grid_map.info.origin.position.y + (grid_map.info.height * grid_map.info.resolution))
        hits = self._tree.intersection(bbx, objects=True)
        nodes = []
        for hit in hits:
            nodes.append(hit.object)
        return nodes
