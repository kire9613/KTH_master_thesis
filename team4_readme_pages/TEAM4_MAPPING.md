# Mapping the Environment
The SVEA car perceives the surrounding environment using lidar scans. The information from the lidar is used to build a map, which in turn is used by the car to navigate, i.e. for path planning and obstacle avoidance.

The structure for handling the map is divided into two parts:
* The map updater: Reading lidar scans and translating them to a map representation.
* The map provider: A help class used by nodes to manage the map.

A more detailed description of the different parts is provided below.

![alt text][map_example]

[map_example]: figures/mapping_img.png "Map example"


## Map Updater
The ROS node *Mapper* is the node that reads the lidar scans, and translates the information into map updates. The environment is represented as a grid map where each tile in map is assigned a value, representing the following:
* occupied space: 100
* polygon space: 120
* c-space: 110
* free space: 0
* unknown space: -1

The node uses the following topics:
* */node_started/mapping*: Notify behaviour tree mapping is started.
* */track/stay_in*: Reading polygon
* */track/keep_out*: Reading polygon
* */scan*: Getting lidar scans
* */state*: Getting car position
* */custom_map_updates*: Publishing map updates
* */infl_map_updates*: Publishing inflated map updates

The Mapper node has two versions of the map, one normal and one inflated version. At initialization both maps are initialized with the map of floor 2, and the polygons the car needs to stay inside respectively keep outside (read from topics */track/stay_in* and */track/keep_out*) is added to the maps. When initialization is done a message is published to */node_started/mapping* informing the task switching that the mapping node is ready.

When receiving lidar scans and car position (from topics */scan* and */state*) from the car, the node starts to build an update to the map. The procedure is as follows:
```
1. Receive lidar scan and car position on topics */scan* and */state*.
2. Looping over the lidar scans, do:
  If scan is in lidar range limits and scan angle within angle limits:
  - Calculate the map coordinate of the detected obstacle:  
      x_scan = (range * cos(angle) + 0.282) * cos(robot_yaw) - range * sin(angle) * sin(robot_yaw)  
      y_scan = range * cos(angle) * sin(robot_yaw) + range * sin(angle) * cos(robot_yaw)  
      where range is the measured distance to obstacle and angle is the scan direction, given by the lidar scan. Further:  
      x_scan_map = x_scan - origin_x  
      y_scan_map = y_scan - origin_y  
      where origin_x, origin_y is origin of the map.
  - Translate the map coordinate to index in the grid map:  
      x_index = int(x_scan_map/resolution)  
      y_index = int(y_scan_map/resolution)  
      where resolution is the resolution of the map.
  - Add map indexes of obstacle to list of obstacles.
  - Add map indexes of tiles in map between obstacle and car position to list holding free space.
3. Loop over list with free space, do:
  - Set tile with indeces x_index, y_index to value 0 in both versions of the map.
4. Loop over list with obstacles, do:
  - Set tile with indeces x_index, y_index to value 1 in the normal map.
  - Set tile with indeces x_index, y_index to value 1, and tiles within a specified radius around the obstacle to value c-space, in the inflated map.
5. From both versions of the map, clip out only the parts of the map that have been recently updated. Pack those into a message of type *OccupancyGridUpdate*.
6. Publish the two messages to *custom_map_updates* respectively *infl_map_updates*.
```
A note to the procedure above is that every time the map needs to be updated, only the updated part of the map is published. This speeds up the process as less data needs to be sent every time.

## Map Provider
The map provider is a class within *updatemap.py* that is imported by different nodes in the system that needs the map. The class manages the map, adding the latest updates and provides functions to read the map.

### Reading the Map
A node can get the map by calling the following functions:
* *get_map()*: Returns the latest version of the map.
* *get_inflated_map()*: Returns a inflated version of the map.

### Updating the Map
*map example*
The class is initialized with two versions of the map, one normal and one inflated version. The default map is the map of floor 2, and the maps are represented as 2d Numpy arrays.

Updates to the map are received by two subscribers listening to the topics *infl_map_updates* and *custom_map_updates*, receiving map updates that are inflated and not inflated respectively. The received updates are continuously added to the maps.  

![alt text][infl_map_example]

[infl_map_example]: figures/infl1.png "Inflated map example"
