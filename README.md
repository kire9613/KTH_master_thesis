# Purpose
The purpose of this document is to give insight into how the different nodes of our SVEA implementation are supposed to interact with each other. It also gives explanations for what the purpose of the nodes are.

# Nodes
The different nodes to be implemented are the following:
- Svea
- Emergency
- Speed and Steering (Controller)
- Control Filter
- Path planner:
  - Trajectory
  - Obstacle avoidance
- Obstacle detection
- Map server
- Map logic


# Vocab
State representation is the coordinate system that the car sees and uses.

Pixel representation is the coordinate system that the map is given in. Each pixel is one step.

Scaled pixel representation is the coordinate system that is a scaled down version of the pixel representation. The origin is still in the same place as the pixel representation as is different compared to the state representation.

## 1. SVEA

#### Inputs
|Name|Type|Data|Description|
|---|---|---|---|
|Control|control_msg|Not Specified|Look up how it should be specified|


#### Outputs

|Name|Type|Data|Description|
|---|---|---|---|
|State|Vehicle_State|x, y, v, yaw|Coordinates in Vehicle_State System|
|Scan|Laser_Scan|Many|Distances in Vehicle_State coordinates|

#### Parameters

Add parameters if it has any.

#### Purpose
This is the given node in the beginning that signifies the car.



## 2. Emergency

#### Inputs
|Name|Type|Data|Description|
|---|---|---|---|
|State|Vehicle_State|x, y, v, yaw|Coordinates in Vehicle_State System|
|Scan|Laser_Scan|Many|Distances in Vehicle_State coordinates|

#### Outputs
|Name|Type|Data|Description|
|---|---|---|---|
|STOP|boolean|boolean|Describes if the emergency break is on or off|

#### Parameters
|Name|Type|Data|Description|
|---|---|---|---|
|Stop_distance|Float|d|Distance at which the emergency break should enable|

#### Purpose
The purpose of this node is to stop the car if it gets too close to an object to protect it in case of accidents. Perhaps it could be added that instead of stopping, it should try to avoid the obstacle through steering.

## 3. Speed and Steering

#### Inputs
|Name|Type|Data|Description|
|---|---|---|---|
|State|Vehicle_State|x, y, v, yaw|Coordinates in Vehicle_State System|
|Trajectory|Trajectory_Path|int8[x,y]|Sequence of coordinates for the trajectory. Given in scaled pixel representation|

#### Outputs
|Name|Type|Data|Description|
|---|---|---|---|
|Control|control_msg|Not Specified|Look up how it should be specified|
|Next_traj|boolean|boolean|Output if close to end of trajectory to generate next trajectory|

#### Parameters
|Name|Type|Data|Description|
|---|---|---|---|
|New_traj_distance|Float|percentage_to_next_traj|Percentage of trajectory left when the next trajectory should be calculated|

#### Purpose
The purpose of this node is to calculate the appropriate speed and steering so that the car follows the trajectory fast and precise. It should also notify the path_planner node when it gets close to the end of the trajectory.



## 4. Control Filter

#### Inputs
|Name|Type|Data|Description|
|---|---|---|---|
|Control|control_msg|Not Specified|Look up how it should be specified|
|STOP|boolean|boolean|Describes if the emergency break is on or off|

#### Outputs
|Name|Type|Data|Description|
|---|---|---|---|
|Control|control_msg|Not Specified|Look up how it should be specified|

#### Parameters
N/A

#### Purpose
Controller to filter the speed to give depending on if the stop signal is on or not.




## 5. Path planner

#### Inputs
|Name|Type|Data|Description|
|---|---|---|---|
|Occupancy_grid|Occupancy_grid|int[], dimensions|Occupancy grid describing where the obstacles are. Given in scaled pixel representation|
|Next_traj|boolean|boolean|Output if close to end of trajectory to generate next trajectory|

#### Outputs
|Name|Type|Data|Description|
|---|---|---|---|
|Trajectory|Trajectory_Path|int8[x,y]|Sequence of coordinates for the trajectory. Given in scaled pixel representation|

#### Parameters
Parameters for the path planner. Was thinking about parameters that defines how much the planner is allowed to deviate from the most optimal route and such.

The node should be given the trajectory coordinates in the beginning and number of laps. Need to define in which coordinate system the nodes need to be provided in.

#### Purpose
Purpose is to calculate the path for which the car is supposed to drive along.



## 6. Obstacle detection

#### Subscribers
|Name|Type|Data|Description|
|---|---|---|---|
|State|Vehicle_State|x, y, v, yaw|Coordinates in Vehicle_State System|
|Scan|Laser_Scan|Many|Distances in Vehicle_State coordinates|
|Map|map|Many|Occupancy grid of original map. Given in pixel representation|

#### Publishers
|Name|Type|Data|Description|
|---|---|---|---|
|State|list of coordinate_messages|int8[x,y]|Coordinates where the car has seen obstacles. Given in pixel representation|

#### Parameters
|Name|Type|Data|Description|
|---|---|---|---|
|Scans_per_Rotation|Integer|number_of_rotations_per_scan|How many scans does the Lidar detect, simulation is 135 and real car is 1081|

#### Purpose
Transforms the Lidar readings into coordinates of the real world in order to place them correctly in the occupancy grid.


## 7. Map server

#### Inputs
N/A
#### Outputs
|Name|Type|Data|Description|
|---|---|---|---|
|Map|map|many|Occupancy grid of original map. Given in pixel representation|

#### Parameters
N/A
#### Purpose
Provides the original static map. Given from the beginning.



## 8. Map logic

#### Inputs
|Name|Type|Data|Description|
|---|---|---|---|
|Map|map|many|Occupancy grid of original map. Given in pixel representation|
|State|coordinate_message|int8[x,y]|Coordinates where the car has seen obstacles. Given in pixel representation?|

#### Outputs
|Occupancy_grid|Occupancy_grid|int[], dimensions|Occupancy grid describing where the obstacles are. Given in scaled pixel representation|

#### Parameters
Scale
obstacle inflation size

#### Purpose
Purpose is to provide a proper map used for pathfinding
