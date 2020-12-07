#!/usr/bin/env python

import rospy
import numpy as np
import math
from svea_msgs.msg import map_pixel_coordinates
from nav_msgs.msg import OccupancyGrid
from svea.track import Track
#import matplotlib.pyplot as plt
# import skimage.measure #install with: python -m pip install -U scikit-image


"""
#Filips notes 20/11-2020 at 01:30

#dynamical order:
#get map
#rescale
#get cordinates
#add cordinates to current map
#save map
#forget if to long ago
#inflate
#publish

#DONE: make it have a memory (handle dynamical)
#DONE: rescale the static map in the beguining and then "rescale" the pixel points so everything is rescaled
#this is not nessesary since the rescaling is done in advance now#TODO: think if the max_pooling should perhaps insted add all the values in the patch as many values might now get lost...
#TODO: figure out best value for radius_in_meters (maybe just a bit more then the width of the car since that is what has to pass throug the coridor and not the lengt, but a bit more due to turning)
#TODO: self.static_map_in_info_origin_position_x might have to be calculated, see lower down! migh maybe never used so well well...


#so basicaly everything is working
#remember that length_of_memory_list, count_of_scans_until_publish, occupied_space_threshold and rescaling_factor have a close relationship to determine what gets inflated
"""

#for dynamical memory
length_of_memory_list = 4 #10 #how many maps to remember, to high makes it not able to forget stuff that might have been caused by noise in the reading, to small might make it update to often

#for the lidar
count_of_scans_until_publish = 140 #5 #preferably small so it sends the values faster and can update faster

#for the inflation
car_radius_in_meters = 0.4 #meters
occupied_space_threshold = 15#35
inflation_space_value = 125 #int8 max is 127...

#For rescaling
rescaling_factor = 2#2#4   

#for publishing
pub = rospy.Publisher('inflated_map', OccupancyGrid, queue_size=10)




def block_reduce(image, block_size, func=np.sum, cval=0, func_kwargs=None):
    #taken from source: https://github.com/scikit-image/scikit-image/tree/ea7a828c67765d4e24c2d561efa4e8e047b6772e
    if len(block_size) != image.ndim:
        raise ValueError("block_size must have the same length "
                         "as image.shape.")
    if func_kwargs is None:
        func_kwargs = {}
    pad_width = []
    for i in range(len(block_size)):
        if block_size[i] < 1:
            raise ValueError("Down-sampling factors must be >= 1. Use "
                             "skimage.transform.resize to up-sample an "
                             "image.")
        if image.shape[i] % block_size[i] != 0:
            after_width = block_size[i] - (image.shape[i] % block_size[i])
        else:
            after_width = 0
        pad_width.append((0, after_width))
    image = np.pad(image, pad_width=pad_width, mode='constant',constant_values=cval)
    blocked = view_as_blocks(image, block_size)
    return func(blocked, axis=tuple(range(image.ndim, blocked.ndim)),**func_kwargs)

def view_as_blocks(arr_in, block_shape):
    #taken from source: https://github.com/scikit-image/scikit-image/tree/ea7a828c67765d4e24c2d561efa4e8e047b6772e
    if not isinstance(block_shape, tuple):
        raise TypeError('block needs to be a tuple')
    block_shape = np.array(block_shape)
    if (block_shape <= 0).any():
        raise ValueError("block_shape elements must be strictly positive")
    if block_shape.size != arr_in.ndim:
        raise ValueError("block_shape must have the same length "
                         "as arr_in.shape")
    arr_shape = np.array(arr_in.shape)
    if (arr_shape % block_shape).sum() != 0:
        raise ValueError("block_shape is not compatible with arr_in")
    # -- restride the array to build the block view
    new_shape = tuple(arr_shape // block_shape) + tuple(block_shape)
    new_strides = tuple(arr_in.strides * block_shape) + arr_in.strides
    arr_out = np.lib.stride_tricks.as_strided(arr_in, shape=new_shape, strides=new_strides)
    return arr_out








def add_maps(map_list):
    total_map = sum(map_list)
    total_map = np.clip(total_map, a_min = None, a_max = 100, out = total_map)
    return total_map


def rescale_map(map_in, resolution_in, rescaling_factor):
    if rescaling_factor == 1:
        return map_in, resolution_in
    else:
        rescaled_map = block_reduce(map_in, (rescaling_factor,rescaling_factor), np.max)
        new_resolution = resolution_in * rescaling_factor
        return rescaled_map, new_resolution


def inflate_map(map, car_radius_in_meters, resolution, occupied_space_threshold, inflation_space_value):

    radius = int(np.ceil((float(car_radius_in_meters)/2)/resolution))
    #print(radius)

    [height, width] = np.shape(map)

    inflated_map = np.zeros((height, width))
    #inflated_map = map#.copy() #TODO: could maybe be faster somehow

    for x in range(radius, height - radius):
        for y in range(radius, width - radius):
            if map[x,y] > occupied_space_threshold:
                
                inflated_map[x, y] = map[x,y]
                
                for dx in range(-radius, radius + 1):
                    for dy in range(-radius, radius + 1):
                        if math.sqrt(dx**2 + dy**2) <= radius and map[x + dx, y + dy] < occupied_space_threshold:
                            inflated_map[x + dx, y + dy] = inflation_space_value

    return inflated_map

def publisher(map):
    #change the data into its original format
    map.data = list(np.array(np.ndarray.flatten(map.data), dtype = np.int8))

    #Publish
    pub.publish(map)



class Map_logic():
    def __init__(self):
        self.lidar_step_counter = 0
        self.dynamical_uppdate_counter = 0 # not realy nessesary but good to have to save image series      
        #for dynamic memory
        self.map_list = []
        self.flag = False
        #To add track limitations to the map
        track = Track()
        self.keep_out = track.keep_out
        self.stay_in = track.stay_in
        self.origin, self.resolution = self.param_init()
        self.generate_track()
        self.inflated_map = OccupancyGrid()
        
        
    def generate_track(self):
        points_stay_in = []
        points_keep_out = []
        for point in self.stay_in:
            points_stay_in.append([int((point[0] - self.origin[0])/self.resolution),
                                int((point[1] - self.origin[1])/self.resolution)])
        for point in self.keep_out:
            points_keep_out.append([int((point[0] - self.origin[0])/self.resolution),
                                int((point[1] - self.origin[1])/self.resolution)])
        self.stay_in = points_stay_in
        self.keep_out = points_keep_out
        
        
    def param_init(self):
        """Initialization handles use with just python or in a launch file"""
        # grab parameters from launch-file
        origin_param = rospy.search_param('origin')
        resolution_param = rospy.search_param('resolution')
        origin = rospy.get_param(origin_param, [0, 0, 0])
        resolution = rospy.get_param(resolution_param, 1)
        if isinstance(origin, str):
            origin = origin.split(',')
            origin = [float(curr) for curr in origin]
        return origin, resolution
        
        
    def get_track_map(self, size):
        track_map = np.zeros(size)
        start = []
        end = []
        n = len(self.stay_in)
        for i in range(n):
            start.append(self.stay_in[i])
            end.append(self.stay_in[(i+1) % n])
        track_boundaries = np.linspace(start, end)
        x_coord = np.rint(track_boundaries[:,:,0].flatten())
        y_coord = np.rint(track_boundaries[:,:,1].flatten())
        x_coord = x_coord.tolist()
        y_coord = y_coord.tolist()
        for i in range(len(x_coord)):
            x = int(x_coord[i])
            y = int(y_coord[i])
            track_map[y][x] = 100
        start = []
        end = []
        n = len(self.keep_out)
        for i in range(n):
            start.append(self.keep_out[i])
            end.append(self.keep_out[(i+1) % n])
        track_boundaries = np.linspace(start, end)
        x_coord = np.rint(track_boundaries[:,:,0].flatten())
        y_coord = np.rint(track_boundaries[:,:,1].flatten())
        x_coord = x_coord.tolist()
        y_coord = y_coord.tolist()
        for i in range(len(x_coord)):
            x = int(x_coord[i])
            y = int(y_coord[i])
            track_map[y][x] = 100
        return track_map
        
        

    def get_map(self, msg):
        #extract info from the given static map
        static_map_in = msg
        resolution = static_map_in.info.resolution
        height = static_map_in.info.height
        width = static_map_in.info.width
        static_map_in_info_origin_position_x = static_map_in.info.origin.position.x
        static_map_in_info_origin_position_y = static_map_in.info.origin.position.y
        
        if len(self.inflated_map.data) == 0:
            print("Static map processed")
            #Add the track to the static map
            track_map = self.get_track_map((height, width))
            
            #shape, rescale and inflate the static map and update infos
            static_map = np.reshape(static_map_in.data, (height, width))
            #static_map = static_map+track_map
            
            [static_map, self.resolution] = rescale_map(static_map, resolution, rescaling_factor)
            [track_map, dummy] = rescale_map(track_map, resolution, rescaling_factor)
            
            self.static_map = inflate_map(static_map, car_radius_in_meters, resolution, occupied_space_threshold, inflation_space_value)
            #self.static_map = self.static_map + track_map
            [self.height, self.width] = np.shape(self.static_map)
            self.flag = True

            #prepare the inflated_map
            self.inflated_map.info.resolution = self.resolution
            self.inflated_map.info.height = self.height
            self.inflated_map.info.width = self.width
            self.inflated_map.info.origin.position.x = static_map_in_info_origin_position_x #-self.inflated_map.info.width*self.resolution/2
            self.inflated_map.info.origin.position.y = static_map_in_info_origin_position_y #-self.inflated_map.info.height*self.resolution/2
            self.inflated_map.info.origin.orientation.w = 1.0


    def update_obstical_map(self, msg):
        if self.flag:
            if self.lidar_step_counter == 0:
                self.obstacle_map = np.zeros((self.height, self.width))

            self.lidar_step_counter += 1
            xs = msg.map_pixel_coordinates_x
            ys = msg.map_pixel_coordinates_y
            
            n = len(xs)
            
            for i in range(n):
                x = int(xs[i]/rescaling_factor) #convert the lidar readingst to resolution of intrest
                y = int(ys[i]/rescaling_factor)
                
                if (y + 1 > self.height) or (x + 1 > self.width) or (y + 1 < 0) or (x + 1 < 0) :
                    print("Out of bounds")
                    d = 0
                elif (self.obstacle_map[y, x] < 100): # # 0 = unknown space, -1 = free space, 100 = obstacle
                    self.obstacle_map[y, x] += 10 #can be changed to decierd value

            

            if self.lidar_step_counter == count_of_scans_until_publish: #135 = full scan, count_of_scans_until_publish times

                #dynamical memory
                self.dynamical_uppdate_counter += 1
                self.map_list.append(self.obstacle_map.copy()) #remember
                if len(self.map_list) > length_of_memory_list: #forget
                    self.map_list.pop(0)

                #what to send/publish to the pathplaner
                total_obstacle_map = add_maps(self.map_list)

                """
                im = plt.imshow(np.flip(total_obstacle_map))
                #plt.colorbar()
                #plt.show()
                plt.pause(0.05)
                plt.clf()
                """

                inflated_obstacle_map = inflate_map(total_obstacle_map, car_radius_in_meters, self.resolution, occupied_space_threshold, inflation_space_value)

                """
                plt.imshow(np.flip(inflated_obstacle_map))
                plt.colorbar()
                plt.show()
                """

                self.inflated_map.data = add_maps([self.static_map, inflated_obstacle_map])
                
                """
                plt.imshow(np.flip(self.inflated_map.data))
                plt.colorbar()
                plt.show()
                """
                """
                im = plt.imshow(np.flip(self.inflated_map.data))
                #plt.colorbar()
                #plt.show()
                plt.pause(0.05)
                plt.clf()
                """
                publisher(self.inflated_map)

                #restart the counter for the lidars steps
                self.lidar_step_counter = 0
                
                #print(len(self.map_list))
                #print(self.dynamical_uppdate_counter)



if __name__ == '__main__':
    rospy.init_node("map_logic")
    
    map_logic = Map_logic()
    rospy.Subscriber('/pixel_coordinates',
                     map_pixel_coordinates,
                     map_logic.update_obstical_map)

    rospy.Subscriber('/map',
                     OccupancyGrid,
                     map_logic.get_map)


    rospy.spin()
