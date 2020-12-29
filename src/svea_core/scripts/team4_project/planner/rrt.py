#!/usr/bin/env python2

from team4_project.planner.rrt_node import RRTNode, raytrace
from rtree import index
import random
import numpy as np
from math import log


class RRT:
    """
    RRT algorithm class searching for a peth from root position to target position.
    Managing search-tree, samples points and adds to tree.
    """

    def __init__(self, root_position, target_position, bbx_min, bbx_max, extension_range, ):
        self._extension_range = extension_range # edge len in tree
        self._sample_nr = 0
        self._target = target_position

        # map coord. ranges
        self._bbx_min = bbx_min
        self._bbx_max = bbx_max

        # Create r-tree
        p = index.Property()
        p.dimension = 2
        self._tree = index.Index(properties=p)

        # Insert root in tree
        self._id = 0
        self._root = self.create_new_node(root_position)
        self.insert(self._root, None)

    def sample(self):
        """ Samples random point (biased towards target point) in environment and creates new node"""

        position = np.array([[0, 0]], dtype=np.float)
        for i in range(0, len(position[0])):
            position[0][i] = random.uniform(self._bbx_min[i], self._bbx_max[i]) # get random point in environment

        # Introduce bias towards the target position
        if self._sample_nr % 5 == 0:
            dist = 1.0
            position[0][0] = random.uniform(self._target[0][0] - dist, self._target[0][0] + dist)
            position[0][1] = random.uniform(self._target[0][1] - dist, self._target[0][1] + dist)
        if self._sample_nr % 10 == 0:
            position[0][0] = self._target[0][0]
            position[0][1] = self._target[0][1]

        self._sample_nr += 1
        return self.create_new_node(position)

    def create_new_node(self, position):
        """ Creates new RRT node """
        node = RRTNode(self._id, position)
        self._id += 1
        return node

    def expand_tree(self, grid_map):
        """ Samples new node adn expands search tree """

        new_node = self.sample()
        return self.extend(grid_map, new_node)

    def extend(self, grid_map, new_node):
        closest_node = self.get_closest(new_node)
        new_node = self.restrict_distance(new_node, closest_node) # extend tree towards sampled point new_node

        if self.valid_node(grid_map, new_node, closest_node):
            parent = closest_node
            self.insert(new_node, parent)
            return new_node

        return None

    def get_closest(self, node):
        """ Get tree-node in tree closest to node """
        position = node.get_position()
        hits = list(self._tree.nearest(
            (position[0][0], position[0][1]), 1, objects=True))
        assert(1 <= len(hits))
        return hits[0].object

    def restrict_distance(self, node, closest_node):
        """
        Restrict distance between parent node and children to extention_range
        Modifies node position to be in direction of sampled point with a maximum distance of extention_range
        from parent node.
        """
        origin = closest_node.get_position()
        direction = node.get_position() - origin # vector from closest_node to node

        # if distance from closest_node to sample gt extension_range
        # add edge to tree in direction towards sampled node, with distance extension_range from parent node
        if np.linalg.norm(direction) > self._extension_range:
            direction = self._extension_range * direction / np.linalg.norm(direction)
            node.set_position(origin + direction) # modified node position

        return node

    def valid_node(self, grid_map, node, parent):
        """ Returns True if all points between node and parent not causes collision with obstacle (value 0) in map. Else False. """

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

    def insert(self, node, parent):
        """ Insert node in tree """
        self.update(node, parent) # Set node parent
        position = node.get_position()
        self._tree.insert(
            node.get_id(), (position[0][0], position[0][1]), obj=node)

    def update(self, node, new_parent):
        node.set_parent(new_parent)
