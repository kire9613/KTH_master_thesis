# Path planning algorithms
The car uses two separate path planning systems, one for global planning which is only run during initialization and another for local re-planning which is called continuously. 

## Global path planner - RRT
The global planner is based on RRT. 

The sampling is biased so that the sampled point is the goal point every tenth sample, and a point close to the goal every tenth sample. This increases the likelihood that the RRT finds a short path between the start and goal position. 

Also, path smoothing is used in order to reduce unnecessary turns. 

The global planner returns a set of way-points.

## Local path planner - Hybrid A*
A local planner is used to re-plan between way-points from the global planner, this is done continuously between the car position and the second next waypoint. Calling the local planner as often as possible works as obstacle avoidance, since the local planner is fed the updated map. 

The local planning is based on hybrid A* which includes a model of the SVEA. We included costs for being far away from the goal point, for turning, and for being close to obstacles in the map. This helps provide a reasonably straight path from start to goal which keeps some additional distance to obstacles if possible.   