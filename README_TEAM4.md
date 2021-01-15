# EL2425 TEAM4

## Short description
The goal of this project was to create a solution making it possible for the SVEA car to autonomously drive along a given track in a safe manner avoiding unknown (and known) obstacles. The code for the project is contained in the folder `svea_starter/src/svea_core/scripts/team4_project`, see rest of readme for a description of what the different parts do.

## Requirements
First, make sure you have followed the basic setup process for running the code in the main-branch of svea_starter given in the README. 

Then for running the code of team4 a few more things needs to be installed.

Before running the code for the first time run the commands:
```
pip install py-trees==0.6.5
sudo apt install python-rtree python-scipy
echo 'export PYTHONPATH="$PYTHONPATH:$(rospack find svea_core)/scripts"' >> ~/.bashrc
source ~/.bashrc
```
Also remember to build the code: `catkin build`

## Running the code

### In simulation

First, make sure the system is set up to run in simulation. Edit `svea_starter/src/svea_core/scripts/team4_project/team4.launch` and make sure `is_sim` is set to `true`. By running the launch script all nodes needed will be started. 

In two different terminals, run:
```
roslaunch svea_core team4.launch
```
and 
```
rosrun svea_core start_pause.py
```

From the first command you will receive a whole bunch of prints from ROS, and after a few seconds you shall end up with something like:
![alt text][run_sim]

[run_sim]: team4_readme_pages/figures/run_sim.png "Run Sim"

Then, in rviz, use `2D pose estimate` to give the car an initial pose. Then press *enter* in the terminal you ran `start_pause.py`. The car should start moving after finishing planning.

When system is running correctly you will se the car moving, and you get some feedback in the terminal:
![alt text][run_sim3]

[run_sim3]: team4_readme_pages/figures/sim3.png "Run Sim 3"

![alt text][run_sim2]

[run_sim2]: team4_readme_pages/figures/sim_2.png "Run Sim 2"

Note: Sometims setting up the map and gobal planning might take a few minutes if previously saved maps and plans are not used. If for example the terminal gets stuck on *Planning path...* you just need to wait for the RRT to finish. When the purple diamonds appear in rviz the planner is done. On the other hand, if the terminal shows *Track sucessfully initialized* for a while, you probably also miss the fat green polygons defining the track in the map. When those to appear, the map has been successfully constructed with the inflated polygons.

### On the car

First make sure the system is set up to run on hardware. Edit `svea_starter/src/svea_core/scripts/team4_project/team4.launch` and make sure `is_sim` is set to `false`. The system is configured to distribute the ROS nodes over multiple machines. Either add your computer(s) to the list of machines and select what nodes are launched on them, or remove the `machine=` part on all `node` tags to run everything on the SVEA.

*** Some extra stuff ***

If you want to be able to run nodes on different machines than the SVEA itself, some setup of ssh-keys is needed. Otherwise the following steps regarding ssh keys can be skipped, then go directly to `roslaunch svea_core team4.launch`.

Setting up ssh:
* On the car, run: `cat ~/.ssh/id_rsa.pub` copy the output.

* On the machine to be used first run `sudo apt install openssh-server`. Edit the file `~/.ssh/authorized_keys` (create the file if it does not exist) and paste what you got from the command that was ran on the car. Finally run `chmod 600 ~/.ssh/authorized_keys`.

* Edit `team4.launch` and define a machine tag for your machine: `<machine name="define a name" address="hostname.local" user="your computer user name" env-loader="path to remote-env.sh"/>`
(output from running "hostname" in the terminal and add the string .local in the end). Then this machine tag can be used in the launchfile as described above.

* Using this for the first time, manually connect to your computer from the SVEA car in order to make sure your computer is known by the car.

Whit this setup done and with machines defined in the launch file, the code can then be ran following the instructons below. Make sure that all machines are connected to the SVEA network.

*** End of extra stuff ***

Now back to the normal procedure of launching the code.

To run the code, you need to start three different programs. First, launch our code on the vehicle with:

```
roslaunch svea_core team4.launch
```
Then, on either the car or your own computer, having done `source svea_starter/utilities/export_ros_ip.sh` first, run:
```
rosrun svea_core start_pause.py
```
If running on hardware, you also need to run rviz manually on your computer with:
```
rviz -d svea_starter/src/svea_core/scripts/team4_project/rviz.rviz
```
In rviz, use `2D pose estimate` to give the car an initial pose. Then press *enter* in the terminal you ran `start_pause.py`. The car should start moving after finishing planning.

## System description

#### Behaviour Trees

Our system is based on a behaviour tree. A behaviour tree is hierarchial and reacting allowing the system to prioritize different task and react to specific events. The behaviour tree also makes the system modular and easy to maintain.

The behaviour tree is built up by nodes in form of: 
* Fallbacks **?** (read as *or*)
* Sequences **->** (read as *and*) 
* Tree leafs (function calls or conditions) 

The tree is continously ticked from the top, checking the status of the nodes in the tree from left to right, i.e. checking what the different nodes returns. 

* The leafs returns either *true*, *false* or *running* telling wether a condition is *true* or *false*, or if a function associated with the leaf returned *true*, *false* or is still *running*. 
* Fallbacks and sequences handle the flow in the tree, interpreted as *and*/*or* conditions, prioritizing the different leafs and subtrees. 
* A sequence ticks all it's children sequentially from left to right and returns *true* if all children return *true*. As soon as one child returns *false* also the sequence node returns *false*.
* A fallback also ticks all it's children sequentially from left to right, but instead returns *true* as soon as one child returns *true*. If all children of the fallback node return *false*, also the fallback node returns *false*.

#### The system

![alt text][behaviour_tree]

[behaviour_tree]: team4_readme_pages/figures/behaviour_tree.svg "Behaviour Tree"

The system consists of several ROS nodes performing different tasks. One node managing the mapping, one the planning and so on. Further, the behavior of the system is defined in a behavior tree. Using a behavior tree provides a reactive and modular structure. It is then easy to prioritize some behaviors over others, and to insert additional functionality. In order to give an overview of how the system works, let's have a look at the different parts of the behavior tree. A more detailed description of all subsystems mentioned is provided in separate readme files linked in the bottom of this page.

#### Before Launching Behaviour Tree
A global plan is calculated using a [RRT planner](team4_readme_pages/TEAM4_PATHPLANNING.md "TEAM4_PATHPLANNING"), planning a path from the cars initial position to a defined goal.
This plan is done only with knowledge of the initial map of floor 2 and the polygons that defines the "virtual walls" defining the race track. The planner generates a list of waypoints defining a path (seen as purple diamonds in the gif below), which is later used to generate a local plan (the yellow line in front of the car).

#### Part 1 - Initialization
When launching the behavior tree, the systems waits until all initialization processes are finished. This includes:
* Waiting for [global planner](team4_readme_pages/TEAM4_PATHPLANNING.md "TEAM4_PATHPLANNING") to be ready by ckecking if there exists any waypoints - *Next waypoint exists?*
* Calculation of a [local plan](team4_readme_pages/TEAM4_PATHPLANNING.md "TEAM4_PATHPLANNING") is done in *Replan path*

When initialization is done the condition *Has initialized* is set to *True* by *Set initialized*.

Next, the main behavior of the car consists of the following:

#### Part 2 - Check if Paused
One can at any time pause the system. Then the car is stopped and velocity is set to zero. When starting the system pause will always be activated. The system can be paused/started using *start_pause.py*. By running the file, pressing enter will switch the status to the opposite one, so if system is in pause enter will start the system, and if system is started enter will pause the system.

#### Part 3 - Check if Done
This part of the tree checks if the car have reached it's goal, that is if the car is at the last waypoint. This procedure consists of:
* First, check if the car is near a waypoint - *Is at waypoint*
* If we are near a waypoint, check if this is the last waypoint, e.g. the goal - *Is last waypoint*
* If the car is at the goal we are done, and speed is set to 0 - *Set speed 0.0*

#### Part 4 - Planning Timeout
Sometimes the [local planner](team4_readme_pages/TEAM4_PATHPLANNING.md "TEAM4_PATHPLANNING") might run into problems, if for example a waypoint is blocked by an obstacle, or if planning just happens to be extra difficult for the current target. Therefore a timeout is used in order to abort the planning process. If the timeout is activated, the following happens:
* Move the current target waypoint along the line between the current and the next waypoint.
When this is done, a new planning process will be started hoping that the new waypoint is not blocked.

#### Part 5 - Obstacle Detection
Along execution we are continiously searching for collisions in the planned path. For every point in the path we check if the point is located at a forbidden area where we are not allowed to drive. The status of the collision detection is then published to a topic */collision* that is read by other nodes that might be interested. The message published to the topic contains two fields: 
```
bool collision 
geometry_msgs/Point collision_point
```
telling if there is a collision (True/False) and the coordinates of the collision point.

#### Part 6 - Waypoint update
![](team4_readme_pages/figures/waypoint_timeout.gif )
The local planner plans from the car position to a target waypoint. This target waypoint needs to be continuously updated when driving along the path. This is done by the following procedure:
* First, check if the car is near a waypoint, otherwise we can keep the current target waypoint - *Is at waypoint*
* If we are near a waypoint, we can update the target waypoint to be the next waypoint in the list. - *Update waypoint*
By doing this we are iteration over the list of waypoint defining our global plan until we have reached out goal.

#### Part 7 - Plan and Drive
The car tries to calculate a new path as often as possible, that is, as long as no planning process is already running we call the local planner through *Replan path*. *Replan path* starts a [Hybrid A* planning](team4_readme_pages/TEAM4_PATHPLANNING.md "TEAM4_PATHPLANNING") process with start point as the cars position and a target waypoint. Every time a new planning process is initialized, the planner gets the latest [map](team4_readme_pages/TEAM4_MAPPING.md "TEAM4_MAPPING") which ensures that the latest plan consider the latest obstacles that have been detected. As soon as a new path is returned, the old path is replaced with the new path. As long there is a path available, the car drives along that path.

### Descriptions of subsystems
* [Mapping](team4_readme_pages/TEAM4_MAPPING.md "TEAM4_MAPPING")
* [Planning](team4_readme_pages/TEAM4_PATHPLANNING.md "TEAM4_PATHPLANNING")
* [Control](team4_readme_pages/TEAM4_CONTROLLER.md "TEAM4_CONTROLLER")
