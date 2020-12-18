# Map Representation

The system uses two sorts of maps for obstacle avoidance. For known obstacles, it uses a map that represents obstacles as convex sets, while for unknown obstacles it uses an occupancy grid. 

## Convex Set Representation

As mentioned above, this map is used to represent known obstacles as convex sets and need to be defined before running the program. To create the known obstacles, a GUI was provided on a given map. To run the GUI, one need to install some dependencies first: 

```bash
sudo apt install python-imaging-tk python3-tk
pip3 install numpy pillow
```

Then, the GUI can be excuted with the following commands:

```bash
roscd svea_core/scripts/util
python3 obstacles_builder_gui.py
```

Once you have run the GUI, a blank window with some buttons should appear. You should press the button "Load map" and choose a pickle file that represents your map. Once the map has been loaded, you can start adding obstacles with the button "Add new obstacle". 

For our specific map, we form our known obstacles as convex sets, see the figure below as an example.

![key-teleop example](./media/convex_map.png) 

When the map is done, you should export and save it. A yaml file will be genereted, which consists of lists of vertices for each obstacle in the map. These lists are stored in the variable occupancy_grid.obstacles in the main file floor2_example.py. This variable is then used in the obstacle avoidance for the path planners, more specifically in the code utils.py, which can be found in /svea_starter/src/svea_core/src/svea/planner. There, with the help of the function _compute_convex_hull_equations, it converts every list of vertices to a condition

<img src="https://render.githubusercontent.com/render/math?math=Ax \leq b">

that describes the limits for x and y within that obstacle that is formed by the vertices. These limits are then used for the path planners to check if they collide with a known obstacle or not, see the section Path Planning for further description. 

## Occupancy Grid Representation

For the unknown obstacles in the map, the system uses an occupancy grid representation. The dimensions of the occupancy grid is 1270x568 with a resolution of 0.05 m. The origin is (-30.55, -11.4), which is being mapped as grid (0, 0) in the occupancy grid. The grids are initialized with 0. If they become occupied, then their values updates to 100.

### Update Map 

The algorithm that updates the occupancy grid, takes a list of obstacle coordinates from which the LIDAR has detected as input. How the LIDAR data is converted to obstacle coordinates, can be found in the section Main Node. Apart from this list, the algortihms also take the distances from the car to the detected obstacles for each LIDAR beam and their corresponding angles togheter with the car's state. Note that the update of the occupancy grid takes place every time step.

Two grids represented as matrices are used, logodds, which is grid with logarithmic probabilities in each node, and localmap, which is a usual occupancy grid with values of either 0 (free) or 100 (occupied). The probabilites in the logodds map, state the probability of a node being occupied. If a node in the logodds exceeds a certain probability or threshold, then the corresponding node in the localmap is set to 100 (occupied). Both these matrcies are initialized with 0 values.

Once the function has everything it needs, it starts by converting the obstacles' coordinates and car's position to indeces in the occupancy grid. Then for each LIDAR beam that has detected an obstacle, a list of indeces that the beam has traversed is being returned from the Bresenham's algorithm. For all the indeces that the beam has traversed, except the last one, are treated as free nodes, who's values are updated with pfree (logarithmic probability of being free) in the logodds map. The last node, is treated as an occupied grid, which is being updated with pocc (logarithmic probability of being occupied). As mentioned above, if a node's value exceeds a certain probability or threshold, the coressponding node in the occupancy grid is set to 100.

The alghorithm can be summarized in the following procedure:

```bash
1. Convert the obstacle coordinates and car's position to indeces
2. For every LIDAR beam:
  3. Calculate the beam's traversed nodes
  4. For every traversed node:
    5. If not last node:
      6. logodds(node) = logodds(node) + pfree
    7. Else:
      8. logodds(node) = logodds(node) + pocc
    9. If logodds(node) > threshold:
      10. localmap(node) = 100
``` 

You can find the code for occupancy grid in /svea_starter/src/svea_core/src/svea/map/occupancy_grid.py. Note that you need to install tqdm. 
  

 

 
