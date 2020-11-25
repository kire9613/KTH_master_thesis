#!/usr/bin/env python

"""
A* grid planning
author: Atsushi Sakai(@Atsushi_twi)
        Nikos Kanargias (nkana@tee.gr)
See Wikipedia article (https://en.wikipedia.org/wiki/A*_search_algorithm)
"""

import math
import os
import pickle
import numpy as np
import matplotlib.pyplot as plt

show_animation = False

dirname = os.path.dirname(__file__)
svea_core = os.path.join(dirname, '../../')
map_name = "problem_map_np" # change this for different maps
file_path = svea_core + 'resources/maps/' + map_name + ".pickle"

width = 879
height = 171

unknown_space = np.int8(-1) #grey
free_space = np.int8(0) #white
c_space = np.int8(25) #grey
occupied_space = np.int8(75) #black
start_spot = np.int8(127) #green
goal_spot = -np.int8(127) #red
bounding_box = -np.int8(10) #yellow

unknown_space_glb = np.int8(-1) #grey
free_space_glb = np.int8(0) #white
occupied_space_glb = np.int8(100) #black

obstacles1 = occupied_space_glb #100
obstacles2 = occupied_space_glb - unknown_space#101
obstacles3 = occupied_space_glb - occupied_space #25
obstacles4 = occupied_space_glb - c_space #75
obstacles5 = unknown_space_glb - occupied_space #-76
obstacles6 = unknown_space_glb - c_space #-26
obstacles7 = -c_space #-25
obstacles8 = -occupied_space #-75

start = -start_spot
goal = -goal_spot

bounds1 = -bounding_box
bounds2 = unknown_space_glb - bounding_box

class AStarPlanner:

	def __init__(self, problem_vector):
		"""
		Initialize grid map for a star planning
		"""
		self.motion = self.get_motion_model()

		self.sx = None
		self.sy = None
		self.gx = None
		self.gy = None

		print(start)
		print(goal)

		problem_map = problem_vector.reshape(height, width)

		minx = height
		maxx = 0
		miny = width
		maxy = 0
		for i in range(height):
			for j in range(width):
				if problem_map[i,j] == bounds1 or problem_map[i,j] == bounds2:
					if i < minx:
						minx = i
					if j < miny:
						miny = j
					if i > maxx:
						maxx = i
					if j > maxy:
						maxy = j		

		self.min_x = minx
		self.min_y = miny
		self.max_x = maxx
		self.max_y = maxy

		if self.min_x > 5:
			self.min_x = self.min_x - 5
		if self.max_x < height - 5:
			self.max_x = self.max_x + 5
		if self.min_y > 5:
			self.min_y = self.min_y - 5
		if self.max_y < width - 5:
			self.max_y = self.max_y + 5

		self.x_width = self.max_x - self.min_x
		self.y_width = self.max_y - self.min_y

		self.obstacle_map = np.full((self.x_width, self.y_width), 0, dtype=np.uint8)
		
		for ix in range(self.x_width):
			x = ix + self.min_x
			for iy in range(self.y_width):
				y = iy + self.min_y
				if problem_map[x,y] == obstacles1:
					self.obstacle_map[ix,iy] = np.uint8(1)
				if problem_map[x,y] == obstacles2:
					self.obstacle_map[ix,iy] = np.uint8(1)
				if problem_map[x,y] == obstacles3:
					self.obstacle_map[ix,iy] = np.uint8(1)
				if problem_map[x,y] == obstacles4:
					self.obstacle_map[ix,iy] = np.uint8(1)
				if problem_map[x,y] == obstacles5:
					self.obstacle_map[ix,iy] = np.uint8(1)
				if problem_map[x,y] == obstacles6:
					self.obstacle_map[ix,iy] = np.uint8(1)
				if problem_map[x,y] == obstacles7:
					self.obstacle_map[ix,iy] = np.uint8(1)
				if problem_map[x,y] == obstacles8:
					self.obstacle_map[ix,iy] = np.uint8(1)					
				if problem_map[x,y] == start:
					print("Hej")
					self.sx = x
					self.sy = y
				if problem_map[x,y] == goal:
					print("Hej2")
					self.gx = x
					self.gy = y

		if show_animation:
			plt.grid(True)
			plt.axis("equal")
			for ix in range(self.x_width):
				x = ix + self.min_x
				for iy in range(self.y_width):
					y = iy + self.min_y
					if problem_map[x,y] == obstacles1:
						plt.plot(x, y, ".k")
					if problem_map[x,y] == obstacles2:
						plt.plot(x, y, ".k")
					if problem_map[x,y] == obstacles3:
						plt.plot(x, y, ".k")
					if problem_map[x,y] == obstacles4:
						plt.plot(x, y, ".k")
					if problem_map[x,y] == obstacles5:
						plt.plot(x, y, ".k")
					if problem_map[x,y] == obstacles6:
						plt.plot(x, y, ".k")
					if problem_map[x,y] == obstacles7:
						plt.plot(x, y, ".k")
					if problem_map[x,y] == obstacles8:
						plt.plot(x, y, ".k")
					if problem_map[x,y] == start_spot:
						plt.plot(x, y, "og")
					if problem_map[x,y] == goal_spot:
						plt.plot(x, y, "xb")

		"""
		print("min_x:", self.min_x)
		print("min_y:", self.min_y)
		print("max_x:", self.max_x)
		print("max_y:", self.max_y)

		print("x_width:", self.x_width)
		print("y_width:", self.y_width)
		"""
	class Node:
		def __init__(self, x, y, cost, parent_index):
			self.x = x  # index of grid
			self.y = y  # index of grid
			self.cost = cost
			self.parent_index = parent_index

	def planning(self):
		"""
		A star path search
		output:
			rx: x position list of the final path
			ry: y position list of the final path
		"""

		start_node = self.Node(self.sx - self.min_x, self.sy - self.min_y, 0.0, -1)
		goal_node = self.Node(self.gx - self.min_x, self.gy - self.min_y, 0.0, -1)

		open_set, closed_set = dict(), dict()
		open_set[self.calc_grid_index(start_node)] = start_node

		while 1:
			if len(open_set) == 0:
				print("Open set is empty..")
				break

			c_id = min(open_set, key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node, open_set[o]))
			current = open_set[c_id]

			# show graph
			if show_animation:  # pragma: no cover
				plt.plot(current.x + self.min_x, current.y + self.min_y, "xc")
				# for stopping simulation with the esc key.
				plt.gcf().canvas.mpl_connect('key_release_event',
												lambda event: [exit(
													0) if event.key == 'escape' else None])
				
			if current.x == goal_node.x and current.y == goal_node.y:
				print("Find goal")
				goal_node.parent_index = current.parent_index
				goal_node.cost = current.cost
				break

			# Remove the item from the open set
			del open_set[c_id]

			# Add it to the closed set
			closed_set[c_id] = current

			# expand_grid search grid based on motion model
			for i, _ in enumerate(self.motion):
				node = self.Node(current.x + self.motion[i][0],
									current.y + self.motion[i][1],
									current.cost + self.motion[i][2], c_id)
				n_id = self.calc_grid_index(node)

				# If the node is not safe, do nothing
				if not self.verify_node(node):
					continue

				if n_id in closed_set:
					continue

				if n_id not in open_set:
					open_set[n_id] = node  # discovered a new node
				else:
					if open_set[n_id].cost > node.cost:
						# This path is the best until now. record it
						open_set[n_id] = node

		rx, ry = self.calc_final_path(goal_node, closed_set)
		
		if show_animation:  # pragma: no cover
			plt.plot(rx, ry, "-r")
			plt.show()

		rx.reverse()
		ry.reverse()
		
		return rx, ry

	def calc_final_path(self, goal_node, closed_set):
		# generate final course
		rx, ry = [goal_node.x + self.min_x], [goal_node.y + self.min_y]
		parent_index = goal_node.parent_index
		while parent_index != -1:
			n = closed_set[parent_index]
			rx.append(n.x + self.min_x)
			ry.append(n.y + self.min_y)
			parent_index = n.parent_index

		return rx, ry

	@staticmethod
	def calc_heuristic(n1, n2):
		w = 1.0  # weight of heuristic
		d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
		return d

	def calc_grid_index(self, node):
		return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

	def verify_node(self, node):
		px = node.x + self.min_x
		py = node.y + self.min_y

		if px < self.min_x:
			return False
		elif py < self.min_y:
			return False
		elif px >= self.max_x:
			return False
		elif py >= self.max_y:
			return False

		# collision check
		if self.obstacle_map[node.x, node.y] == np.uint8(1):
			return False

		return True

	@staticmethod
	def get_motion_model():
		# dx, dy, cost
		motion = [[1, 0, 1],
					[0, 1, 1],
					[-1, 0, 1],
					[0, -1, 1],
					[-1, -1, math.sqrt(2)],
					[-1, 1, math.sqrt(2)],
					[1, -1, math.sqrt(2)],
					[1, 1, math.sqrt(2)]]

		return motion

def main():
	with open(file_path, 'rb') as f:
		pickled_problem = np.load(f)

	a_star = AStarPlanner(pickled_problem)
	rx, ry = a_star.planning()

	for i in range(len(rx)):
		print(rx[i],ry[i])

if __name__ == '__main__':
    main() 
