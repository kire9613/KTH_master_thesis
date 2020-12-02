# Path planning algorithms
The car uses two separate path planning systems, one for global planning which is only run during initialization and another for local re-planning which is called continuously. 

## Global path planner - RRT
The ROS node *planner* is the node which handles the global path planning. It is called with the map provider described in [Mapping](https://github.com/KTH-SML/svea_starter/blob/team4_master/TEAM4_MAPPING.md "TEAM4_MAPPING") to get the updated map, a start position, and a goal position. When called, the node runs a RRT algorithm and returns a set of way-points from the start position to the goal position. The way-points are so that the lines connecting each way-point avoids obstacles in the map.  

### RRT
The RRT algorithm used takes a start position (x0,y0) and a goal position (xT,yT) as input. The algorithm works as follows:
```
1. Initialize a tree of nodes with the start position (x0,y0)
2. Sample a point in the map 
3. Test taking a step from (x0,y0) in the direction towards the sampled point.
  - If the new point is safe (map value = 0 or -1) and line from (x0,y0) to new point is safe, add new node to tree.  
  Set the starting node as parent node to the new node.
  - If the new node is in an obstacle, c-space or polygon space, discard node and start over from 2.
4. Sample a new point
5. Find the closest node in the tree
6. Test taking a step from the closest node towards the sampled point.
  - If the new point is safe (map value = 0 or -1), add new node to tree.  
  Set the closest node as parent node to the new node.
  - If the new node is in an obstacle, c-space or polygon space, discard node and start over from 4.
7. When a node in the tree is close enough to the goal position, retrieve the path taken (a list of positions from (x0,y0) to (xT,yT))
9. Smooth path
8. Return smoothed path
```

The path smoothing is used to reduce the number of unnecessary way-points, this especially removes unnecessary turns, making the path straighter. The smoothing removes points if a straight line segment between the prior and next point doesn't go through either occupied space, c-space, or polygon space.

The sampling is biased so that the sampled point is the goal point every tenth sample, and a point close to the goal every tenth sample. This increases the likelihood that the RRT finds a short path between the start and goal position.

## Local path planner - Hybrid A*
A local planner is used to re-plan between way-points from the global planner, this is done continuously between the car position and the second next way-point. Calling the local planner as often as possible works as obstacle avoidance, since the local planner is fed the updated map. 

The local planning is based on hybrid A* which includes a model of the SVEA. We included costs for being far away from the goal point, for turning, and for being close to obstacles in the map. This helps provide a reasonably straight path from start to goal which keeps some additional distance to obstacles if possible.

### Action server
The local planning is contained within an action server called *AStar_server*. For this action server we have defined some new messages AStarGoal, AStarResult and AStarFeedback.

The message AStarGoal is defined as follows:  
string *frame_id*  
float32 *x0*  
float32 *y0*  
float32 *theta0*  
float32 *xt*  
float32 *yt*  
float32 *thetat*  
bool *smooth*

The message AStarResult is defined as follows:  
nav_msgs/Path *path*

The message AStarFeedback is defined as follows:  
float32 *distance_to_goal*

When called with a goal message, the server starts running a Hybrid A* algorithm, see next section. If the server finds a path from start to goal it goes into a *succeeded* state and publishes a AStarResult message containing a path which is a list of poses along a trajectory from start to goal.

The server can be preempted, this aborts the process running Hybrid A*

### Hybrid A*

**This whole section has to be re-written!**  
The algorithm takes an initial pose (x0,y0,theta0) and a goal pose (xt,yt,thetat) as input.

We build a priority queue by testing certain control inputs and use a model of the SVEA to predict what the next pose will be.   
The control signals we test during planning is velocity = 1 m/s and five different steering angles ANGLES = [-pi/4,-pi/8,0,pi/8,pi/4].   
Test stepping forward 10 steps, if any position on that trajectory == not ok, then disregard branch. (If a branch will go through a position in the map which is not allowed (occupied space, polygon space or c-space), the branch is disregarded and removed from the priority queue.)  
Set a cost for each branch, length of path, number of turns, length to goal, close to obstacle... <- write what costs?

Pick branch with lowest cost from priority queue, test all actions and add new branches to priority queue with updated costs.
repeat. 

if close enough to goal pose, return path. 
Send feedback of how close the closest path is to reaching goal position. 

If preempted,return nothing (?) 
