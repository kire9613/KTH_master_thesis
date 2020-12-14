from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PointStamped
from svea_msgs.msg import VehicleState as VehicleStateMsg
from nav_msgs.msg import Path
import rospy
import numpy as np
from tf.transformations import euler_from_quaternion

from svea.path_planners.mpc_planner.utils import are_nodes_close_enough


class ROSInterface(object):
    """ROSInterface used to handle all ROS related functions for the planner
    """

    def __init__(self, x_traj,y_traj):
        self.distance_tolerance = rospy.get_param('~distance_tolerance')
        self.angle_tolerance = rospy.get_param('~angle_tolerance')
        self.initial_state = None
        self.goal_state = None
        self._has_endpoint_changed = False
        self._current_target_state = [0,0] #needs to be not None
        self.goal_angles = []
        self.x_traj = x_traj
        self.y_traj = y_traj
        self.current_speed = 0
        self.path_index = 0


    def set_goal_angles(self, angles):
        self.goal_angles = angles

    def cb_target_state(self,msg):
        """Callback to get the initial state from a ROS message
        :param msg: ROS message
        :type msg: ROS message
        """
        x, y = [getattr(msg.point, coord) for coord in ('x', 'y')] 

        self._current_target_state = [x, y]

    def compute_goal(self, step_ahead):
        old_goal_state = self.goal_state
        traj_length = len(self.x_traj)
        #print("searching for value", self._current_target_state[0], self._current_target_state[1])

        for i in range(0,traj_length-1):
            #if (abs(self.x_traj[i] - self._current_target_state[0]) <1e-2) and (abs(self.y_traj[i] - self._current_target_state[1])) :
            if self.x_traj[i] == self._current_target_state[0]:
                print("index found")
                path_index = i
                self.path_index = i
                break
            else: #index not found, might need to fix this further (!)
                path_index = self.path_index

        if path_index == 0:
            print("index not found")
            
        if step_ahead + path_index > traj_length:
            goal_index = traj_length - 1 
            #print("index is last index:",goal_index)
        else:
            goal_index = step_ahead + path_index
            #print("index:",goal_index)

        x, y = self.x_traj[goal_index], self.y_traj[goal_index]
        
         # check if index is valid otherwise get final value
        if len(self.goal_angles) > goal_index:
            yaw = self.goal_angles[goal_index]
        else:
            yaw = self.goal_angles[len(self.goal_angles)-1]
        self.goal_state = [x, y, yaw]
        print("The goal state is:",self.goal_state)

        kwargs = {
            'first_node': old_goal_state,
            'second_node': self.goal_state,
            'distance_tolerance': self.distance_tolerance,
            'angle_tolerance': self.angle_tolerance
        }
        if not are_nodes_close_enough(**kwargs):
            self._has_endpoint_changed = True
        #else:
            #rospy.loginfo('[planner] Nodes are close enough. Won\'t replan')


    def cb_initial_state(self, msg):
        """Callback to get the initial state from a ROS message
        :param msg: ROS message
        :type msg: ROS message
        """
        
        old_initial_state = self.initial_state
        #At the moment the inital state is hardcoded
        # (!) uncommented this to get position from ROS
        #x, y = [getattr(msg.pose.pose.position, coord) for coord in ('x', 'y')]
        x, y, yaw = [getattr(msg, coord) for coord in ('x', 'y','yaw')]

        self.initial_state = [x, y, yaw]
        self.current_speed = msg.v
        
        # self.initial_state = [self.x0,self.y0,self.theta0] #original 

        #self._has_endpoint_changed = True

        kwargs = {
            'first_node': old_initial_state,
            'second_node': self.initial_state,
            'distance_tolerance': self.distance_tolerance,
            'angle_tolerance': self.angle_tolerance
        }

        if not are_nodes_close_enough(**kwargs):
            self._has_endpoint_changed = True
        else:
            self._has_endpoint_changed = False
            #rospy.loginfo('[planner] Nodes are close enough. Won\'t replan')


    @staticmethod
    def is_shutdown():
        """Wrapper of the rospy.is_shutdown() method in ROS
        :return: `True` if the ros node is stopped/being stopped, `False` otherwise
        :rtype: bool
        """
        return rospy.is_shutdown()



