#!/usr/bin/env python

import rospy
import numpy as np
import time
from nav_msgs.msg import OccupancyGrid
from svea_msgs.msg import next_traj
from svea.path_planners.path_requester import PathRequester
from svea_msgs.msg import VehicleState
from svea_msgs.msg import lli_ctrl
#import matplotlib.pyplot as plt

from svea_msgs.msg import slow_down

number_of_laps = 2

#xs = [-2.40, -6.58, -6.43, -3.0, -1.87, 1.18, 1.24, 1.82 , 4.5, 10.06, 9.83, 6.45, 5.28, -1.9]
#ys = [2.9, -2.97, -4.46, -6.8, -6.12, -2.05, 1.8, 2.66, 3.35, 11.12, 12.04, 14.38, 14.22, 3.75]

xs = [-2.4, -6.58, -6.43, -3.0, -1.87, 1.18, 3, 2.3, 3.35, 5.6, 10.06, 9.83, 6.45, 5.28, 1.5, 1.9, -0.1, -2.4]
ys = [2.9, -2.97, -4.46, -6.8, -6.12, -2.05, 1, 2.9, 4.5, 4.8, 11.12, 12.047, 14.38, 14.22, 8.6, 5.0, 3.0, 2.9]

class Path_logic():
    def __init__(self):

        traj_x = []
        traj_y = []
        for i in range(len(xs)-1):
            traj_x.append(np.linspace(xs[i], xs[i+1]).tolist())
            traj_y.append(np.linspace(ys[i], ys[i+1]).tolist())
        traj_x.append(np.linspace(xs[-1], xs[0]).tolist())
        traj_y.append(np.linspace(ys[-1], ys[0]).tolist())
        self.traj_x = [val for sublist in traj_x for val in sublist]
        self.traj_y = [val for sublist in traj_y for val in sublist]
         

        self.publisher_next_traj = rospy.Publisher('/TrajMessage', next_traj, queue_size = 10)
        self.pub = rospy.Publisher('/slow_down', slow_down, queue_size=10)
        self.look_ahead = 60 #60 # how many pixels forward the path should be estimated 
        self.threshold_distance = 15 #15 trigger A* when distance to obstacle is less than a threshold
        self.threshold_wait = 10 #7 wait until car turns around obstacle and check if new obstacles are hidden
        self.count_laps = 0
        self.obs_N = 0
        self.current_path = next_traj()
        self.current_path.x_coordinates = self.traj_x
        self.current_path.y_coordinates = self.traj_y
        self.path_publisher(self.current_path)
        self.A_star_activated = False
        self.sent = False
        self.A_star_path_done = True
        self.A_star_path = next_traj()
        self.current_x=[]
        self.current_y=[]
        self.remote_overwrite = False
        
    def remote_control(self,msg):
        control = msg.ctrl        
        if control >= 4:
            self.remote_overwrite = True
            self.current_path.x_coordinates = self.traj_x
            self.current_path.y_coordinates = self.traj_y
            self.path_publisher(self.current_path)
        else:
            self.remote_overwrite = False
            #plt.imshow(self.to_print_map)
            #plt.colorbar()
            #plt.show()
    
    def path_publisher(self, path):
        self.publisher_next_traj.publish(path)
        

    def get_inflated_map(self,msg):
        path_requester = PathRequester()
        map = msg
        height = map.info.height
        width = map.info.width
        resolution = map.info.resolution
        origin_x = map.info.origin.position.x
        origin_y = map.info.origin.position.y
        inflated_map = np.reshape(map.data, (height, width))
        self.to_print_map = inflated_map 
     
        self.current_x = int((self.state.x - origin_x)/resolution)
        self.current_y = int((self.state.y - origin_y)/resolution)
        slow_down_msg = slow_down()
        
        """if not self.remote_overwrite:
            print("PRINTING MAP")
            plt.imshow(inflated_map)
            plt.colorbar()
            plt.show()"""        

        for i in range(0,len(self.current_path.x_coordinates)):    
            x_coordinate_pixel = int((self.current_path.x_coordinates[i] - origin_x)/resolution)            
            y_coordinate_pixel = int((self.current_path.y_coordinates[i] - origin_y)/resolution)             
            
            #Returns True if obstacle is on the path
            check = self.obstacle_path_check(inflated_map, x_coordinate_pixel, y_coordinate_pixel)

            if check and not self.sent and not self.remote_overwrite:
                slow_down_msg.slow_down = True
                self.pub.publish(slow_down_msg)

                #Define target point
                target_x = 0
                target_y = 0
                if not self.A_star_path_done:
                    x_path = self.traj_x
                    y_path = self.traj_y
                    target_x, target_y = self.get_target_ind(x_path, y_path, origin_x, origin_y, resolution)
                    print("Estimated path is not completed.")
                else:
                    x_path = self.current_path.x_coordinates
                    y_path = self.current_path.y_coordinates
                    target_x, target_y = self.get_target_ind(x_path, y_path, origin_x, origin_y, resolution)  
                
                # Find a new path and publish it                     
                print("New obstacle observed. Calculating new path...")      
                new_path = path_requester.estimate_path([self.current_x, self.current_y],[target_x,target_y], map.data, width, height)
                print("New path has been calculated")
    
                

                if new_path == None:
                    slow_down_msg.slow_down = False
                    self.pub.publish(slow_down_msg)
                    break
                
                traj_x = []
                traj_y = []
                
                for i in range(len(new_path.estimated_path_x)-1):
                    traj_x.append(np.linspace(new_path.estimated_path_x[i], new_path.estimated_path_x[i+1]).tolist())
                    traj_y.append(np.linspace(new_path.estimated_path_y[i], new_path.estimated_path_y[i+1]).tolist())
                # #Makes the nested lists into a one dimensional array.
                new_path.estimated_path_x = [val for sublist in traj_x for val in sublist]
                new_path.estimated_path_y = [val for sublist in traj_y for val in sublist]

                self.A_star_activated = True                      

                """self.obs_N+=1
                if self.obs_N > 0:
                    for j in range(0, len(new_path.estimated_path_x) -1 ):
                        inflated_map[int(new_path.estimated_path_x[j]),int(new_path.estimated_path_y[j])] += 50
                    plt.imshow(inflated_map)
                    plt.colorbar()
                    plt.show()"""
                
                xs = new_path.estimated_path_y
                ys = new_path.estimated_path_x                     

                A_traj_x = []
                A_traj_y = []
            
                for i in range(0,len(xs)):
                    A_traj_x.append(xs[i] * resolution + origin_x)
                    A_traj_y.append(ys[i] * resolution + origin_y)
                
                self.current_path.x_coordinates = A_traj_x
                self.current_path.y_coordinates = A_traj_y

                slow_down_msg.slow_down = False
                self.pub.publish(slow_down_msg)

                self.update_to_next_path()
                
                break
    
    def check_laps(self):
        #TODO: make sure cars stops when it should stop
        err = np.sqrt((self.state.x - self.traj_x[-50])**2 + (self.state.y - self.traj_y[-50])**2)
        if err < 0.1:
            self.count_laps+=1
        #print(self.count_laps)

    def get_target_ind(self, x_path, y_path, orx, ory, res):
        current_ind = self.find_position(x_path, y_path)
        target_ind =  current_ind + self.look_ahead
        if target_ind > len(x_path):
            target_ind = len(x_path)-1
        target_distance = np.sqrt((x_path[current_ind] - x_path[target_ind])**2 + (y_path[current_ind] - y_path[target_ind])**2)
        while target_distance > 4:
            target_ind = target_ind - 2
            target_distance = np.sqrt((x_path[current_ind] - x_path[target_ind])**2 + (y_path[current_ind] - y_path[target_ind])**2)
        target_x = int((x_path[target_ind] - orx)/res)  
        target_y = int((y_path[target_ind] - ory)/res)

        #print(target_distance)
        return target_x, target_y

    def obstacle_path_check(self, inflated_map, x, y):
        if (inflated_map[y,x] > 0): # check if obstacle is on the way
            if np.sqrt((self.current_x - x)**2 + (self.current_y -  y)**2) < self.threshold_distance: #  check distance to obstacle
                return True
            else:
                return False
        else:
            return False

    def current_position_callback(self,msg):
        self.state = msg
        self.check_trajectory_index()
        self.check_laps()
    
    def find_position(self, path_x, path_y):
        errors=[]
        for i in range(0,len(path_x)):
            errors.append(np.sqrt((self.state.x - path_x[i])**2 + (self.state.y - path_y[i])**2))
        result = np.where(errors == np.amin(errors))
        result = result[0]
        return result[0]

    def check_trajectory_index(self):
        pose = self.find_position(self.current_path.x_coordinates, self.current_path.y_coordinates)
        if not self.A_star_path_done:
            threshold = np.sqrt((self.state.x - self.current_path.x_coordinates[-1])**2 + (self.state.y - self.current_path.y_coordinates[-1])**2)
            if threshold < 0.3:
                self.A_star_path_done = True
                self.sent = False
            elif pose > self.threshold_wait:
                self.sent = False

        self.update_to_next_path()
    

    def update_to_next_path(self):
        if self.A_star_activated:
            print("A* path is published")
            self.path_publisher(self.current_path)
            self.sent = True
            self.A_star_activated = False
            self.A_star_path_done = False
            
        if not self.A_star_activated and self.A_star_path_done:
            #print("Regular path published")
            self.current_path.x_coordinates = self.traj_x
            self.current_path.y_coordinates = self.traj_y
            self.path_publisher(self.current_path)

        """elif self.count_laps == number_of_laps:
            self.current_path.x_coordinates = []
            self.current_path.y_coordinates = []
            self.path_publisher(self.current_path)
            # Publish an empty path so that the speed_steering will take care of this and
            # set velocity to 0 to stop the car. This will be done in speed_steering
            self.current_path.x_coordinates = [0]
            self.current_path.y_coordinates = [0]"""   


if __name__ == '__main__':
    rospy.init_node("path_planner")
    path_logic = Path_logic()
    
    rospy.Subscriber('/lli/remote',
                     lli_ctrl,
                     path_logic.remote_control)

    rospy.Subscriber('/inflated_map',
                     OccupancyGrid,
                     path_logic.get_inflated_map)                                                         

    rospy.Subscriber('/state',
                     VehicleState,
                     path_logic.current_position_callback)

    rospy.Subscriber('/SVEA/state',
                     VehicleState,
                     path_logic.current_position_callback)

    rospy.spin()
