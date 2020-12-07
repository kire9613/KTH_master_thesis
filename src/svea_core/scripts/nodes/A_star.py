#!/usr/bin/env python
import math
import numpy as np
import roslib
import rospy
from svea_msgs.srv import PathEstimator, PathEstimatorResponse
from svea_msgs.msg import VehicleState
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from svea.controllers.grid_type import Grid


class A_star():

    def __init__(self):
        self.current_state = VehicleState()
        

    def state_callback(self, msg):
        self.current_state = msg
    
    
    

    def calculate_path(self, req):
        start = [req.current_state[0],req.current_state[1]]
        end = [req.target_state[0], req.target_state[1]]
        grid_array = req.grid
        width = req.width
        hight = req.height
        self.occupancy_grid_t = np.reshape(grid_array, (hight,width))

        response = PathEstimatorResponse()
        path = astar(self.occupancy_grid_t, [start[1], start[0]], [end[1], end[0]])
        path_x = []
        path_y = []
        try:
            for element in path:
                path_x.append(element[0])
                path_y.append(element[1])
            response.estimated_path_x = path_x
            response.estimated_path_y = path_y
            return response
        except:
            pass
    


"""
A* algorithm for path finding:
start = list with two coordinates in pixel formar
end = list with two coordinates in pixel formar
ocupancy_grid =  a matrix that represents a map:
    -1 =  unknown
    100 = obstacle
     0 =  free
Returns a list of lists as a path from the given start to the given end in the given map
"""

class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        
        self.penalty = False
        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position


def astar(ocupancy_grid, start, end):
    
    #Timeout Threshold parameter    
    timeout_time = 5
    # Create start and end node
    start_node = Node(None, np.array(start))
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, np.array(end))
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    open_list.append(start_node)
    counter = 0
    # Loop until you find the end
    start_time = rospy.get_rostime()
    while len(open_list) > 0:
    
        end_time = rospy.get_rostime()
        if (end_time.secs-start_time.secs >= timeout_time):
            print("Timeout happened")
            return None
        #print(remote_overwrite)
        # Interupter
        #counter+=1
        #if counter > 
        # Get the current node
        current_node = open_list[0]
        
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Found the goal
        if (current_node == end_node).all():
            #print("Found the goal")
            path = []            

            # Make smooth path 
            checkPoint = current_node
            current = current_node.parent
            path.append(checkPoint.position)
            while current.parent is not None:
                if path_is_clear(ocupancy_grid,checkPoint.position,current.position):
                    current = current.parent
                else:
                    checkPoint = current
                    current = current.parent
                    path.append(checkPoint.position)
            path.append(current.position)
            
            #path = relevant_pix(path, ocupancy_grid)

            return np.array(path[::-1]) # Return reversed path

        # Generate children
        children = []
        for new_position in [[0, -1], [0, 1], [-1, 0], [1, 0], [-1, -1], [-1, 1], [1, -1], [1, 1]]: # Adjacent squares

            # Get node position
            node_position = [current_node.position[0] + new_position[0], current_node.position[1] + new_position[1]]


            # Make sure walkable terrain
            pose = ocupancy_grid[node_position[0],node_position[1]]
            if pose != -1 and pose != 0:
                continue

            # Create new node
            new_node = Node(current_node, np.array(node_position))
            children.append(new_node)

        # Loop through children
        for child in children:
            # Child is on the closed list
            for closed_child in closed_list:
                if (child == closed_child).all():
                    continue


            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h

            # Child is already in the open list
            for open_node in open_list:
                if (child == open_node).all() and child.g > open_node.g:
                    continue

            # Add the child to the open list
            open_list.append(child)
            
def relevant_pix(path, grid):
    new_path = []
    new_path.append(path[0])
    for element in range(1,len(path)-1):
        coord=path[element]
        x = coord[0]
        y = coord[1]
        if grid[x + 1,y] or grid[x,y + 1] or grid[x - 1,y] or grid[x,y - 1] > 100:
            new_path.append(path[element])
        elif grid[x + 1,y +1] or grid[x - 1,y + 1] or grid[x - 1,y - 1] or grid[x + 1,y - 1] > 100:
            new_path.append(path[element])
    new_path.append(path[-1])
    return new_path

"""def iterate_path(path_points):
    connected_path = []
    for i in range(0,len(path_points)-1):
        point_A = path_points[i]
        point_B = path_points[i+1]
        while point_A[0] != point_B[0] and point_A[1] != point_B[1]:
            connected_path.append(point_A)
            if point_A[0] > point_B[0]:
                point_A[0]-=1
            if point_A[0] < point_B[0]:
                point_A[0]+=1
            if point_A[1] > point_B[1]:
                point_A[1]-=1
            if point_A[1] < point_B[1]:
                point_A[1]+=1
    connected_path.append(path_points[-1])
    return connected_path"""

def path_is_clear(grid ,start, end):
    x1 = start[0]
    y1 = start[1]
    x2 = end[0]
    y2 = end[1]
    detector = True
    while x1 != x2 and y1 != y2:
        if x1 > x2:
            x1-= 1
        if x1 < x2:
            x1+= 1
        if y1 > y2:
            y1-= 1
        if y1 < y2:
            y1+= 1
        if grid[x1,y1] == 0: # check if pixel along a line is not walkable  
            detector = False
            break
    return detector

def main():
    rospy.init_node("A_star")
    a_star = A_star()
    

    rospy.Subscriber('/SVEA/state',
                     VehicleState,
                     a_star.state_callback)
                     
    rospy.Subscriber('/state',
                     VehicleState,
                     a_star.state_callback)
     

    rospy.Service('calculate_path', PathEstimator, a_star.calculate_path)
    rospy.spin()


if __name__ == '__main__':
    print("A_star node is working")
    main()
    
    
