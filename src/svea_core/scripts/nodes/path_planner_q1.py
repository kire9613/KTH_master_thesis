#!/usr/bin/env python

import rospy
import numpy as np
import time
from nav_msgs.msg import OccupancyGrid
from svea_msgs.msg import next_traj
from svea.path_planners.path_requester import PathRequester
from svea_msgs.msg import VehicleState
from svea_msgs.msg import lli_ctrl
from std_msgs.msg import Bool
from svea_msgs.msg import slow_down

'''
Path planner node. 
Executes an A* path finding algorithm if obstacles are detected
In case path is free from osbtacels pubslihs main trajectory 
'''

# Main trajectory to follow
xs = [0.0, 1.4, 4.1, 6.48,  8.9, 18.1, 19.5, 18.6, 6.9, 4.8, 2.7, 1.0, -13.1, -13.7, -14.1, -13.5, -8.2, -7.4, -6.22, -4.6, 0.0]
ys = [0.0, 0.0, 2.0, 1.98, -0.01, -1.2, 1.1,   3.5, 4.8, 6.8, 7.2, 5.2,  5.65,   4.9,  1.9,  1.1,   0.7,  3.14,  3.26,  0.8, 0.0]

class Path_logic():
    def __init__(self):

        # Enumerate path  
        traj_x = []
        traj_y = []
        for i in range(len(xs)-1):
            traj_x.append(np.linspace(xs[i], xs[i+1]).tolist())
            traj_y.append(np.linspace(ys[i], ys[i+1]).tolist())
        traj_x.append(np.linspace(xs[-1], xs[0]).tolist())
        traj_y.append(np.linspace(ys[-1], ys[0]).tolist())
        self.traj_x = [val for sublist in traj_x for val in sublist]
        self.traj_y = [val for sublist in traj_y for val in sublist]
         

        self.publisher_next_traj = rospy.Publisher('/TrajMessage', next_traj, queue_size = 1)
        self.pub = rospy.Publisher('/slow_down', slow_down, queue_size=1)
        self.look_ahead = 25 # how many pixels forward the path should be estimated (in pix)
        self.threshold_distance = 10 # trigger A* when distance to obstacle is less than a threshold
        self.threshold_wait = 1 # defines how fast A* should be triggered when new obstacles are detected (1 means each pixel = 10 cm)
        self.current_path = next_traj()
        self.current_path.x_coordinates = self.traj_x #Initialize current path as a main trajectory
        self.current_path.y_coordinates = self.traj_y #Initialize current path as a main trajectory
        self.path_publisher(self.current_path)
        self.A_star_activated = False #Flag to check if A* was activated 
        self.sent = False #Flag to check if A* trajectory is publushed
        self.A_star_path_done = True #Flag to check if car completed trajectory calculated by A*
        self.A_star_path = next_traj()
        self.current_x=[] #Current path for car to follow
        self.current_y=[] #Current path for car to follow
        self.remote_overwrite = False #Flag to check if remote overwrite is trigered
        self.using_astar_publisher = rospy.Publisher('/using_astar', Bool, queue_size = 1)
    
    #With localisation issues the remote can be used to reset tracjectory
    def remote_control(self,msg):
        control = msg.ctrl
        #Key 4 stands for overwrite function of a remote        
        if control >= 4:
            self.remote_overwrite = True
            self.current_path.x_coordinates = self.traj_x
            self.current_path.y_coordinates = self.traj_y
            self.path_publisher(self.current_path)
        else:
            self.remote_overwrite = False
    
    def path_publisher(self, path):
        self.publisher_next_traj.publish(path)
        

    # Gets inflated map continiousluy and checks if obstacles are detected
    def get_inflated_map(self,msg):

        #Define current map
        path_requester = PathRequester()
        map = msg
        height = map.info.height
        width = map.info.width
        resolution = map.info.resolution
        origin_x = map.info.origin.position.x
        origin_y = map.info.origin.position.y
        inflated_map = np.reshape(map.data, (height, width))

        #Define current position in pix
        self.current_x = int((self.state.x - origin_x)/resolution)
        self.current_y = int((self.state.y - origin_y)/resolution)

        #Define slow fown message
        slow_down_msg = slow_down()
              

        #Check if any point of current trajectory correlates with detected obstacles
        for i in range(0,len(self.current_path.x_coordinates)):    
            x_coordinate_pixel = int((self.current_path.x_coordinates[i] - origin_x)/resolution)            
            y_coordinate_pixel = int((self.current_path.y_coordinates[i] - origin_y)/resolution)             
            
            #Returns True if obstacle is on the path
            check = self.obstacle_path_check(inflated_map, x_coordinate_pixel, y_coordinate_pixel)

            if check and not self.sent and not self.remote_overwrite:
                #Slow down car if obstacle is on the path
                slow_down_msg.slow_down = True
                self.pub.publish(slow_down_msg)

                #Define target point
                target_x = 0
                target_y = 0
                if not self.A_star_path_done:
                    #If an obstacle was detected while following regular trajectory
                    x_path = self.traj_x
                    y_path = self.traj_y
                    target_x, target_y = self.get_target_ind(x_path, y_path, origin_x, origin_y, resolution)
                    print("Target point estimated.")
                else:
                    #If a hidden obstacle was detected while car was following trajectory calculated by A*
                    x_path = self.current_path.x_coordinates
                    y_path = self.current_path.y_coordinates
                    target_x, target_y = self.get_target_ind(x_path, y_path, origin_x, origin_y, resolution)  
                
                # Use path_requester service to calculate trajectory around obstacle         
                print("New obstacle observed. Calculating new path...")
                new_path = path_requester.estimate_path([self.current_x, self.current_y],[target_x,target_y], map.data, width, height)
                           
                if new_path == None:
                    slow_down_msg.slow_down = False
                    self.pub.publish(slow_down_msg)
                    break
                print("New path has been calculated")   
                self.A_star_activated = True
                
                traj_x = []
                traj_y = []

                #Enumerate path to make it smooth
                for i in range(len(new_path.estimated_path_x)-1):
                    traj_x.append(np.linspace(new_path.estimated_path_x[i], new_path.estimated_path_x[i+1]).tolist())
                    traj_y.append(np.linspace(new_path.estimated_path_y[i], new_path.estimated_path_y[i+1]).tolist())
                # #Makes the nested lists into a one dimensional array.
                new_path.estimated_path_x = [val for sublist in traj_x for val in sublist]
                new_path.estimated_path_y = [val for sublist in traj_y for val in sublist]
                
                xs = new_path.estimated_path_y
                ys = new_path.estimated_path_x                     

                A_traj_x = []
                A_traj_y = []

                #Convert path from pixel to state
                for i in range(0,len(xs)):
                    A_traj_x.append(xs[i] * resolution + origin_x)
                    A_traj_y.append(ys[i] * resolution + origin_y)
                
                self.current_path.x_coordinates = A_traj_x
                self.current_path.y_coordinates = A_traj_y

                slow_down_msg.slow_down = False
                self.pub.publish(slow_down_msg)

                self.update_to_next_path()
                
                break
    

    def update_inflated_map(self, msg):
        self.inflated_not_reshaped = msg.data

    #Calculate target point coordinates
    def get_target_ind(self, x_path, y_path, orx, ory, res):
        current_ind = self.find_position(x_path, y_path)
        target_ind =  current_ind + self.look_ahead
        #Check if target point index is higher than number of path points
        if target_ind > len(x_path):
            target_ind = len(x_path)-1
        target_distance = np.sqrt((x_path[current_ind] - x_path[target_ind])**2 + (y_path[current_ind] - y_path[target_ind])**2)
        #While loop keeps target coordinates within reasonable boundaries if trajectory is unevenly sampled
        while target_distance > 4:
            target_ind = target_ind - 2
            target_distance = np.sqrt((x_path[current_ind] - x_path[target_ind])**2 + (y_path[current_ind] - y_path[target_ind])**2)
        target_x = int((x_path[target_ind] - orx)/res)  
        target_y = int((y_path[target_ind] - ory)/res)

        return target_x, target_y

    #Check if obstacle is on the given way
    def obstacle_path_check(self, inflated_map, x, y):
        if (inflated_map[y,x] > 0):
            if np.sqrt((self.current_x - x)**2 + (self.current_y -  y)**2) < self.threshold_distance: #  check distance to obstacle
                return True
            else:
                return False
        else:
            return False

    def current_position_callback(self,msg):
        self.state = msg
        self.check_trajectory_index()
    
    #Find where the car is in given trajectory
    def find_position(self, path_x, path_y):
        errors=[]
        for i in range(0,len(path_x)):
            #Compare current state with all trajectory points and choose point with minimal error
            errors.append(np.sqrt((self.state.x - path_x[i])**2 + (self.state.y - path_y[i])**2))
        result = np.where(errors == np.amin(errors))
        result = result[0]
        return result[0]

    #Check if car is almost done with trajectory calculated by A*
    def check_trajectory_index(self):
        pose = self.find_position(self.current_path.x_coordinates, self.current_path.y_coordinates)
        if not self.A_star_path_done:
            threshold = np.sqrt((self.state.x - self.current_path.x_coordinates[-1])**2 + (self.state.y - self.current_path.y_coordinates[-1])**2)
            #Car is aproaching final trajectory point calculated by A*
            if threshold < 0.3:
                self.A_star_path_done = True
                self.sent = False
            #Release A* when car goes around a vissible obstacle
            #Makes it possible to detect hidden obstacles
            elif pose > self.threshold_wait:
                self.sent = False

        self.update_to_next_path()
    
    #Publish trajectory to follow
    def update_to_next_path(self):
        
        # Publish A* trajectory if obstacles are on the way
        if self.A_star_activated:
            self.using_astar_publisher.publish(True)
            print("A* path is published")
            self.path_publisher(self.current_path)
            self.sent = True
            self.A_star_activated = False
            self.A_star_path_done = False
        
        # Publish regular trajectory if obstacles are not on the way
        if not self.A_star_activated and self.A_star_path_done:
            self.using_astar_publisher.publish(False)
            self.current_path.x_coordinates = self.traj_x
            self.current_path.y_coordinates = self.traj_y
            self.path_publisher(self.current_path)
        

if __name__ == '__main__':
    rospy.init_node("path_planner")
    path_logic = Path_logic()
    
    rospy.Subscriber('/lli/remote',
                     lli_ctrl,
                     path_logic.remote_control, queue_size = 1)

    rospy.Subscriber('/inflated_map',
                     OccupancyGrid,
                     path_logic.get_inflated_map, queue_size = 1, buff_size = 2**28)

    rospy.Subscriber('/state',
                     VehicleState,
                     path_logic.current_position_callback, queue_size = 1)

    rospy.Subscriber('/SVEA/state',
                     VehicleState,
                     path_logic.current_position_callback, queue_size = 1)

    rospy.spin()