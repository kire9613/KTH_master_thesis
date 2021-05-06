Documentation for  package av09

By Erik Branzén (ebranzen@kth.se)

last update 2021-05-06

#########################
Project description
#########################
The code base developed in the package av09 as well as av09_msgs was a part of the the Master thesis of Erik Branzén. The code was written during the spring and summer of 2021. The goal of the thesis was to develop fault diagnostics and prognostics algorithms for an autonomous vehicle. Based on a knowledge base on active and potential faults, the vehicle could counteract faults and plan to minimize the chance of severe faults. The vehicle could also consider potential downtime for different actions and choose actions that optimized time lost during driving. These functionalities were implemented on the SVEA-platform, first simulated but later on a real physical vehicle. Some scenarios were developed to test the system functionalities.



#########################
Software architecture description
#########################
Five distinct modules make up the system. In ROS, these are declared as nodes. Communication between them is done via ROS-topics. One node can publish on a topic and others can subscribe to it in order to access the data. The content of the topics are defined by a ROS message. 

The five nodes are : FaultSimulation, FaultDetection, Diagnosis/prognosis/planning, Actuation and Vehicle_to_firebase. A brief black box description of these nodes follow

Faultsimulation:
This node generated fault events based on user input. As there were no components with sensors available to generate low-level diagnostic warnings, these had to be simulated in the software. The user chooses a fault and based on a fault-to-symptom probabilistic model, symptoms are generated. These symptoms are subsequently published to \rawsignals. 

FaultDetection:
Based on the simulated sensor signals, This node first checks if any new signals are present and then publishes the signals as symptoms on \symptoms. Also, A handshake variable is published on a seperate topic \detection_status. In turn, the node is subscribed to \diagnose_decision_status to access the corresponding handshake variable sent from the diagnosis/prognosis/planning node. When a new symptom is detected, the detection node publishes a message on \detection_status. When the diagnosis/prognosis/planning node has noticed this, a confirmation is published on diagnose_decision_status.

Diagnose/prognosis/planning:
This is the core module of the system. First, a diagnosis is made based on the current symptoms if these are new symptoms. If the diagnosis confirms a new fault, then a severity analysis is made. If the fault is severe, an immediate reaction is necessary. If it is not severe, a prognosis can be made along with a long-term plan. The prognosis uses the active fault as a basis to find possible new faults that might propagate as a result. These are then combined to calculate a time window in which the vehicle can operate relaitvely safely (based on a user specified risk limit). This time window is subsequently used as a hard condition in the route planning that follows. Here, the system chooses which workshop to visit (if it needs to visit one) based on the the time window and the downtimes for driving to and spending time at them. 

The node subscribes to many topics. The most important are \CTcorr, \symptoms and \detection_status. On \CTcorr, the control tower publishes its control variables on the system. For example, the control tower might need to verify the diagnosis in some cases where reliability is low. The control tower can also override a plan.

The node publishes to several topics aswell. These are \diag, \plan, \diagnose_decision_status. The diag consists of the active fault and its severity and the plan consists of speed limit, route and release condition. The release condition determines what will allow the system to go back to normal operation after a fault has been dealt with. For example, if the fault is severe, the control tower is deemed the only actor that can "release" the vehicle back to normal operation even though the vehicle itself might conclude that the fault is healed.

Actuation:
This is the node that implements that plan on the vehicle in simulation or on the physical system. This includes updating route based on plan and current location along with switching between operational states based on release condition and active faults. The most important inputs to the node are \CTcorr and \plan. From \CTcorr, the control tower can release the vehicle from a degraded operational state. The \plan topic supplies the speed limit, route and release condition.

This node also simulates the vehicle in rviz. The vehicle follows a declared trajectory with the PurePursuitController. 

Vehicle_to_firestore:
This node subscribes to all topics that are of interest to the control tower and writes data from these to the firebase firestore online database. The node also reads from the database to collect the variables from the control tower that controls functions in the vehicle. This data is then published to the \CTcorr topic.

########################
File Organization:
########################

The ROS package for the project is located in svea_starter/src with the name av09. In av09/src, the python code for the project is located along with other datafiles and other documentation.

All python files that operate as seperate ROS nodes end with '_node' in their name. Other python files are subscripts consisting of classes with methods employed in the different nodes. Examples are faultclass.py which is not a ROS node but instead consists of a class for describing individual component or subsystem faults.

Faults.txt and symps.txt are datafiles for setting up the simulated vehicle case explored in the project. These are used in the diagnosis/prognosis/planning node.

In av09/launch, launchfiles for different case studies can be located.

#########################
Design Decisions and Programming practices:
#########################
Across many of the nodes, the same data is used and represented. Some examples are the diagnosis, symptoms, plan. The representations of these metrics are shared between nodes to make the code easy to interpret. For example, the diagnosis is represented as a DIAG class whose methods and parameters are the same between nodes for the most part (with exception to callback functions).

The core mechanics of the nodes are that of a state machine. The parameters that determine the function of a state machine is: state, inputs, outputs and state transition conditions. In all nodes, these paramters are defined locally in the python files.

The modular nature of the system was chosen as a key design feature for various reasons. Firstly, a modular architecture is easier to grasp and explain to another person. Secondly, in a modular architecture, changing features locally will not affect the system as a whole as long as long as good documentation is available where it is clear what each module inputs and outputs. Handing over a code base to another developer or working in parallell with someone else is much less pain-inducing. 

The problem of communicating with the control tower was solved through the use of an online database, called firebase. A seperate node for writing to and reading from this database was developed to again make the design more intuitive and easy to approach. The more nodes added the more reliance on ROS was required. 

########################
Installation procedure
########################
The code is written in python 2.7 and uses ROS melodic on a Linux Ubuntu OS. To run the code as it was designed to be run, these need to be installed. 
to
The project uses the SVEA-platform developed by ITRL at KTH (Royal institute of technology). The code base is available at https://github.com/KTH-SML/svea_starter. The code can be cloned from git to your local machine. Permission is necessary in order to access the code base.

A more thorough installation process is available in the README in /svea_starter, the base directory of the above mentioned code base.

When the base code for SVEA is installed, the av09 and av09_msgs packages need to be added to svea_starter/src. catkin_make will then install the packages.


########################
Detailed Node descriptions
########################
Simulation node


ROS topic subscribers: \diag, \symptoms, \plan, \actuation_status

ROS topic publishers: \rawsignals, \start_system

Thread_function():

Inputs: None

Outputs: None

Declaration of ROS topic subscribers \diag, \symptoms, \plan, Actuation_status

Creation of GUI with buttons for fault generation

Class GUI:

__init__(): 
inputs: GUI root, Pub_start

Creation of faultlist and symplist with object representations.

GUI.click_start():

When button on GUI is clicked, a variable to start the simulation of the vehicle is published to the \start topic.

Gui.repaired():

When the vehicle has repaired at a workshop, all symptoms will be gone, and this method will be called. 

GUI.click_fault():

Given the fault being clicked, this method is called and symptoms signals corresponding to the fault is generated.

GUI.diagnose_decision_status_callback():

Handshake variables from diagnosis/prognosis/planning node is set from ROS message

GUI.actuation_callback():

handshake variables and status variables from actuation node is set from ROS message


Main():

A paralell thread is set up for the GUI. In this thread, the signal variable will be set. Communication between threads is done via a global variable 'scenario'. In the main loop of main(), the signals are published every iteration.



Diagnose_plan_node

ROS topic subscribers: \symptoms, \detection_status, \CTcorr, \vehicle_map_data, \actuation_status

ROS topic publishers: \diag, \diagnose_decision_status, \action

Class STATEMACHINE:

__init__():
inputs: initial state, pub_diag, slist, flist

Creation of Symp, CTcorr, Diag, Prognosis, Plan objects.

Initialization of handshake variables.

Statemachine.Stateswitch():

Calls state specific method depending on state variable

Statemachine.state_1():

System is idle. Waits for new symptoms, then go to state 2

Statemachine.state_2():
Diagnosing. If new diagnosis is different from last, then go to state 3

Statemachine.state_3():
Waiting for verification of diagnosis from control tower. If fault is severe, go to state 4 without verification.

Statemachine.state 4():
Planning state. publish diagnosis and find optimal plan by first making a prognosis with Prognosis.prognose() and then plan through Plan.replan(). If the diagnosis is still not verified, we go back to state 3. Otherwise, go back to state 1.

Statemachine.state_5():
Control tower has full control over vehicle. Overrides all autonomous functions of the vehicle.


Class DIAG:

__init__():
set initial timestamp for reference
creation of P(S|F) matrix 

Diag.Diagnose():
inputs: symptoms

Creation of diagnosis given set of symptoms. Uses matrix calculations to find the likeliest fault. Uses time under operation, asumptions of relations between faults and symptoms as well as which faults are possible given the symptoms.


Class Symp:

__init__():
Inputs: None
Initialize symp and deltasymp

Symp.callback_symp():
set object variables from ROS message.

Symp.callback_detectstatus():
set object handshake variables from ROS message

Class CTcorr:

__init__():
Inputs: None

Initialize object variables

CTcorr.callback():

set object variables from ROS message



Class plan:

__init__():
inputs: speed limit, RC, mgoal, wgoal

Initialization of object variables

Plan.vhd_callback():
set vehicle distances to workshops and main goal, as well as vehicle location, from ROS message

Plan.actuation_callback():

set vehicle operational state and actuation_status from ROS message

Plan.replan():
Inputs: Diag object, Prognosis object

From the diagnosis, a prognosis is made, through the Prognosis.Prognose() function. This prognosis is used to determine the feasible action space in terms of speeds and routes. From this feasible set, a plan that minimizes downtime is chosen. Downtime analysis is done through plan.downtime(). The plan is then set.

Plan.downtime():
inputs: plan, dist_w1,dist_w2, dist_mgoal

Using estimates of time spent at workshop and assumptions on downtime for driving at different speeds, the function returns the total downtime from taking a certain route and driving with a certain speed.



Class Prognosis:

__init__():
Inputs: flist

Initialization of object variables.


Prognosis.prognose():

Inputs: diag, dist_w1, dist_w2, dist_mgoal

Given diagnosis, create a list of potential new faults that might occur. Then calculate the shared distribution between the faults and find the time when any new fault occurs with probability alpha. Alpha is set by the user. This is translated into a risk to drive to the different goals and workshops.


Main():

First, all parameters from the launchfile are translated into python variables. Then, the data from faults.txt and symps.txt are translated into python objects. These are later sent into the Diag class upon initialization within the statemachine. After this, all ROS subscribers and publishers are declared. Following the main loop of the node is initiated. Here, the plan along with status messages and handshake variables are published continuasly. At the end of the loop, the statemachine switches based on the internal variables of the node. The node opeartes at 5Hz.


Detection_node

ROS topic subscribers: \rawsignals, \diagnose_decision_status

ROS topic publishers: \symptoms, \detection_status


class STATEMACHINE:

__init__():

inputs: initial state

Creation of symptom object and signal object

Statemachine.Stateswitch():

Calls state specific method depending on state variable

Statemachine.state_1():

Looking for new symptoms. The node reads the signals and translate them into symptoms via Symp.create_symptoms(). If the symptoms are new, then go to state 2

Statemachine.state_2():

New symptoms found. Handshake variable for diagnostic/prognosis/planning node is set and when the diagnostic/prognostic/planning node returns that the new symptom is confirmed, go back to state 1.


Class Symptom:

__init__():

initialize object variables

Symptom.create_symptoms():

sets an "oldsymp" variable and a "newsymp" variable and compare them through the function delta().


Function delta():

inputs: newsymp, oldsymp

If any event signal boolean in the new signal is different from the old signal, then return True. Otherwise, return False.


Class Signals:

__init__():

initialize object variables

signal.callback_signal():

set signal variable from ROS message

signal.callback_diagstatus():

set diag handshake variables from ROS message


Main():

The node operates continusaly through a while loop where symptom data and handshake variables are published continuasely. At the end of each loop, the statemachine switches states depending on the internal variables of the node. The node operates at a frequency of 10Hz.

 

Vehicle_to_firestore

ROS topic subscribers: \diag, \diagnose_decision_status, \action, \symptoms, \path_path, \vehicle_map_data, \actuation_status

ROS topic publishers: \CTcorr

Class Topics_to_firestore:

__init__():

Setup the credentials for the firestore database by reading from ServiceAccountKey.json. After, all variables that relate to the whole system are initialized. 

callback functions:

The class has a callback function for all relevant ROS topics. In these, variables are set from the ROS messages.


Topics_to_firestore.update_firestore_write():

Here, All relevant variable are translated into datasets and written to firestore. The specific structure of the database can be seen at ...........  

Topics_to_firestore.CT_update_read():

The control variables set by the control tower are collected from the database and set as python variables within the Topics_to_firebase object.


Main():

First, all subscribers and publishers are set up. Then, the main loop is initialized. In this the update_firestore_write() and CT_update_read methods are called and control tower variables published to the ROS topic \CTcorr. The loop operates at 1Hz. 


Actuation_node

ROS topic subscribers: \action, \CTcorr, \start_system

ROS topic publishers: \actuation_status, \vehicle_map_data

Class StateMachine:

__init__():

Given a map consisting of nodes, all trajectories corresponding to the route between nodes are generated. These are put into a dictionary where the key is which node the vehicle is driving from and the the node the vehicle is driving to. After this, all object variables are initialized and the plan and CTcorr objects are created.


Statemachine.update_actuation():

inputs: svea object

This method updates route and speed  given the position of the vehicle and the plan.  The route consists of a series of nodes and once a node is reached, the trajectory is updated. If the vehicle has reached a workshop, it stops and waits a certain amount of time and if it has reached the end goal, it stops indefinetly.

The method returns the new  trajectories.

Statemachine.Stateswitch():

Calls state specific method depending on state variable. The states correspond to operational conditions: Normal operations, CT release only, automatic release

Statemachine.state_1():

The vehicle is in normal operation. No fault is present


Statemachine.state_2():

The vehicle has suffered a severe fault. Only the control tower can release the vehicle back to normal operations.

Statemachine.state_3(): 

The vehicle has suffered a fault but can autonomously go back to normal operations if the fault is gone.

class CTcorr:

representation of control variables from control tower

class Plan:

Representation of variables corresponding to the plan to be actuated.


Main():

First, all ROS subscribers and publishers are set up and statemachine initialized. Then, The model for the svea vehicle is created and the controller chosen. In the main loop of this node, the simulation is updated, statemachine updated, target velocity and vehicle trajectory updated in statemachine.update_actuation(). Along with this, all status messages and distances to the goals are publishes on to their respective topics.


Function dist_to_goal():

inputs: svea object, curr_path, goal

calculates distance to goal by iterating through node network with the route from the current position. Returns distance float number.

Function dist_calc():

inputs: svea object, goal_p

calculates distance to local goal point and returns true if within one meter of it. Returns False otherwise.


############################
Description of other python files
############################

RUL_model.py:

Creates python objects of class Propfault from a .csv file. Some faults can be directly connected to other faults via propagation. The RUL_model class also has a method (RUL_model.possible_new()) for finding all possible new faults that might occur and their average time until occurance, given a diagnosis. 


faultclass.py:

Class for representing a fault and its potential symptoms


symptomsclass.py:

class for representing a symptom and all its potential faults that might have created it.


















