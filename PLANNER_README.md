# Path Planning

The system uses two seperate path planners, a local planner and a global planner. As a global path planner, it uses RRT Connect and as a local path planner it uses the usual RRT. You can find the codes for those tasks under svea_starter/src/svea_core/src/svea/planner. Other scripts that they depend on are utils.py and smoothing.py, which you can find in the same dictionary. The script utils.py, provide the path planners the position of the obstacles and if a collision will occur or not when following the genereated path. The script smoothing.py, smoothes the path that is genereted from the path planner. 

## Global Planner - RRT Connect

## Local Planner - RRT

## Modifications

### Sampling Domain

### Obstacle Avoidance

### Smoothing




