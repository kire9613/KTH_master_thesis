#EL2425 TEAM2

## System Requirements

The system reuiqres the packages scipy, tqdm, matplotlib and numpy. To install them, run the following commands on the terminal:

```bash
pip install scipy
pip install tqdm
pip install matplotlib
pip install numpy
```

## Run System in Simulation

To run the system in simulation, open a terminal and type the following command:

```bash
roslaunch svea_core floor2.launch
```

Rviz should open and you should see the map. Use 2D pose estimate to initialize the car's position (approximately (0, 0)). The global planning should start and yield a path. If the path has been genereted and is shown on rviz, the car should start following the path. While the system runs, keep an eye on the terminal to check if the car replans and do not find a path. If the car does not find a path, restart the program. You terminate the program with ctrl + c.

## Run System in Reality

To run the system in reality, you have to modify the launch file /svea_starter/src/svea_core/launch/floor2.launch in the car. Set is_sim to False and comment the line <node pkg="map_server" type="map_server" name="map_server" output="screen" args="$(arg map_file)" />. Add the following lines to your launch file:

```bash
<!--open serial connection for controlling SVEA-->
<node pkg="rosserial_python" type="serial_node.py" name="serial_node">
<param name="port" value="/dev/ttyACM0"/>
<param name="baud" value="250000"/>
</node>

<include file="$(find svea_sensors)/launch/localize.launch">
    <arg name="use_rs" value="true"/>
    <arg name="file_name" value="$(arg map_file)"/>
</include>
```

When the launch file has been modified, the program is ready to run. However, when running the program in the car for the first time you need to go to /svea_starter/src/svea_sensors in the terminal and run the following command:

```bash
sudo bash grant_hardware_permisions.sh
```

When the step above is done, you can proceed as usual. Connect to the car by open a terminal and type:

```bash
ssh nvidia@svea2.local
```

where svea2.local is the IP-adress of the car. A password will be required, type nvidia. Then run the following command:

```bash
roslaunch svea_core floor2.launch
```

You also need to open another terminal and go to /svea_starter/src/svea_core/resources/rviz. Run the command:

```bash
rviz -d SVEA_floor2.rviz
```

As in the simulation, rviz should open and you should see the map. Use 2D pose estimate to initialize the car's position (approximately (0, 0)). The global planning should start and yield a path. If the path has been genereted and is shown on rviz, the car should start following the path. While the system runs, keep an eye on the terminal to check if the car replans and do not find a path. If the car does not find a path, restart the program. You terminate the program with ctrl + c.

## System Description

When the program has been launched from the terminal, it starts to initialize some processes and then waits for the user to initilize the car's position with 2d pose estimate. Thereafter, the global planner starts to find a path from the start to the goal poisiton. When the path has been genereted, the system starts to go into a loop where it first always checks if it has reached the goal or not. If it has not reached the goal, then it computes the desired control signals (genereted from path tracker) to track the path. While doing so, it simultaneously scans the environment with a LIDAR sensor and updates the map with the LIDAR data. The LIDAR data consists of distances from the car to the obstacles that the LIDAR has detected for each beam (135 beams in total). After updating the map, the system runs the replanning algorithm. It executes all the mentioned steps above in one time step, in order to repeat them, where it starts by checking if it has reached the goal or not. When the car has reached the goal, it sets the speed to zero and waits for the user to terminate the program.

The code for main system can be found in /svea_starter/src/svea_core/scripts/core_examples/floor2_examples.py. For a further description of the subsystems such as path planning or path tracking, see the sections below.


 

 
