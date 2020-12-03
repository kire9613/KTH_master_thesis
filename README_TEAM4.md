# EL2425 TEAM4

## Short description

## Requirements
Before running the code for the first time run the commands
```
pip install py-trees==0.6.5
sudo apt install python-rtree python-scipy
echo 'export PYTHONPATH="$PYTHONPATH:$(rospack find svea_core)/scripts"' >> ~/.bashrc
source ~/.bashrc
```

## Running the code

To run the code on the car, you need to start three different programs. First, launch our code on the vehicle with:

```
roslaunch svea_core team4.launch
```
Then, on either the car or your own computer, having done `source svea_starter/utilities/export_ros_ip.sh` first, run:
```
rosrun svea_core start_pause.py
```
You also need to run rviz on your computer with:
```
rviz -d svea_starter/src/svea_core/scripts/team4_project/rviz.rviz
```
In rviz, use `2D pose estimate` to give the car an initial pose. Then press *enter* in the terminal you ran `start_pause.py`. The car should start moving after finishing planning.

## System description

![alt text][behaviour_tree]

[behaviour_tree]: team4_readme_pages/figures/behaviour_tree.svg "Behaviour Tree"

The behavior of the system is defined in a behavior tree. Using a behavior tree provides a reactive and modular structure. It is then easy to prioritize some behaviors over others, and to insert additional functionality. In order to give an overview of how the system works, let's have a look at the different parts of the behavior tree. A more detailed description of all subsystems is provided in separate readme files linked in the bottom of this page.

#### Before Launching Behaviour Tree
A global plan is calculated using a [RRT planner](team4_readme_pages/TEAM4_PATHPLANNING.md "TEAM4_PATHPLANNING"), planning a path from the cars initial position to a defined goal.
This plan is done only with knowledge of the initial map of floor 2 and the polygons that defines the "virtual walls" defining the race track. The planner generates a list of waypoints defining a path, which is later used to generate a local plan.

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
When this is done, a new planning process will be started which hopefully not suffer from the same problem.

#### Part 5 - Obstacle Detection
Along execution we search for collisions in the planned path...

#### Part 6 - Waypoint update
![](team4_readme_pages/figures/waypoint_timeout.gif )
The local planner plans from the car position to a target waypoint. This target waypoint needs to be continuously updated when driving along the path. This is done by the following procedure:
* First, check if the car is near a waypoint, otherwise we can keep the current target waypoint - *Is at waypoint*
* If we are near a waypoint, we can update the target waypoint to be the next waypoint in the list. - *Update waypoint*

#### Part 7 - Plan and Drive
The car tries to calculate a new path as often as possible, that is, as long as no planning process is already running we call the local planner through *Replan path*. *Replan path* starts a [Hybrid A* planning](team4_readme_pages/TEAM4_PATHPLANNING.md "TEAM4_PATHPLANNING") process with start point as the cars position and a target waypoint. Every time a new planning process is initialized, the planner gets the latest [map](team4_readme_pages/TEAM4_MAPPING.md "TEAM4_MAPPING") which ensures that the latest plan also consider the latest obstacles that have been detected. As soon as a new path is returned, the old path is replaced with the new path.

### Descriptions of subsystems
* [Mapping](team4_readme_pages/TEAM4_MAPPING.md "TEAM4_MAPPING")
* [Planning](team4_readme_pages/TEAM4_PATHPLANNING.md "TEAM4_PATHPLANNING")
* [Control](team4_readme_pages/TEAM4_CONTROLLER.md "TEAM4_CONTROLLER")
