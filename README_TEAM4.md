# EL2425 TEAM4

## Short description

## Requirements

## Running the code

To run the code on the car, you need to start three different programs. First, launch our code on the vehicle with:

```
roslaunch svea_core team4.launch
```
Then, on either the car or your own computer, having done ´source svea_starter/utilities/export_ros_ip.sh´ first, run:
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

[behaviour_tree]: https://github.com/KTH-SML/svea_starter/blob/team4_master/behaviour_tree.svg "Behaviour Tree"

The behavior of the system is defined in a behavior tree. Using a behavior tree provides a reactive and modular structure. It is then easy to prioritize some behaviors over others, and to insert additional functionality. In order to give an overview of how the system works, let's have a look at the different parts of the behavior tree.

#### Before Launching Behaviour Tree
Waypoints and RRT...

#### Part 1 - Initialization
When launching the behavior tree, the system waits until all initialization processes are finished. This includes:
* Waiting for global planner to be ready by checking if there exists any waypoints - *Next waypoint exists?*
* Calculation of a local plan is done in *Replan path*
When initialization is done the condition *Has initialized* is set to *True* by *Set initialized*.

#### Part 2 - Check if Paused
One can at any time pause the system. Then the car is stopped and velocity is set to zero. When starting the system pause will always be activated. The system can be paused/started using *start_pause.py*. By running the file, pressing enter will switch the status to the opposite one, so if system is in pause *enter* will start the system, and if system is started *enter* will pause the system.

#### Part 3 - Check if Done

#### Part 4 - Planning Timeout

#### Part 5 - Obstacle Detection

#### Part 6 - Plan and Drive
