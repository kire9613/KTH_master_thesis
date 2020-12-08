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
        #rospy.init_node('planner')

        self.path_frame = rospy.get_param('~path_frame')
        self.rate = rospy.Rate(rospy.get_param('~publish_frequency'))
        self.planner_algorithm = rospy.get_param('~planner_algorithm')
        self.distance_tolerance = rospy.get_param('~distance_tolerance')
        self.angle_tolerance = rospy.get_param('~angle_tolerance')
        self.initial_state = None
        self.goal_state = None
        self._has_endpoint_changed = False
        self._obstacles = None
        self._current_target_state = [0,0] #needs to be not None
        self.goal_angles = []
        self.x_traj = x_traj
        self.y_traj = y_traj
        self.current_speed = 0
        self.path_index = 0
        #self.path_publisher = rospy.Publisher('~path', Path, queue_size=1)
        
    def set_goal_angles(self, angles):
        self.goal_angles = angles
    @property
    def has_endpoint_changed(self):
        """Checks if either the initial state or goal state are valid and have
        changed. Additionally, after reading the flag once, it resets it to
        False (to avoid more than one computation when the endpoints change)
        :return: `True` if any endpoint has changed, `False` otherwise
        :rtype: bool
        """
        if self.initial_state is None or self.goal_state is None:
            return False

        has_changed = self._has_endpoint_changed
        self._has_endpoint_changed = False

        return has_changed

    @property
    def obstacles(self):
        if self._obstacles is not None:
            return self._obstacles

        self._obstacles = rospy.get_param('~obstacles')

        return self._obstacles
    
    def publish_path(self, path, publisher=None):
        """Given the arrays corresponding to the path, it publishes a Path
        ROS message.
        :param path: array of poses of the format [x, y, yaw]
        :type path: numpy array
        :param publisher: publisher to use, defaults to None (self.path_publisher)
        :type publisher: rospy Publisher, optional
        """
        if path is None:
            return

        if publisher is None:
            publisher = self.path_publisher

        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = self.path_frame

        poses = []
        #added
        #poses = Path.poses
        for ind in range(len(path)):
            pose = PoseStamped()
            pose.header.frame_id = self.path_frame
            pose.pose.position.x = path[ind][0]
            pose.pose.position.y = path[ind][1]

            poses.append(pose)


        #print("Poses",poses) # (!) added
        path_msg.poses = poses
        publisher.publish(path_msg)

    def sleep(self):
        """Wrapper of the rate.sleep() method in ROS
        """
        try:
            self.rate.sleep()
        except rospy.exceptions.ROSInterruptException as e:
            rospy.loginfo(
                '[planner] Program terminated. Exception: {}'.format(e))

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

    def get_planner_parameters(self, planner_algorithm=None):
        """Creates a dictionary representation of the parameters that will be
        used for the path planner.
        :return: dictionary of ROS parameters
        :rtype: dictionary
        """
        planner_algorithm = planner_algorithm or self.planner_algorithm

        parameter_names = ['distance_tolerance', 'angle_tolerance']

        if planner_algorithm == 'mpc':
            parameter_names += [
                'N', 'wheelbase', 'width', 'length',
                'R_speed', 'R_steering_angle', 'kappa',
                'speed_min', 'speed_max',
                'steering_angle_min', 'steering_angle_max'
            ]

        planner_parameters = {
            parameter_name: rospy.get_param('~{}'.format(parameter_name))
            for parameter_name in parameter_names
        }

        return planner_parameters

    @staticmethod
    def is_shutdown():
        """Wrapper of the rospy.is_shutdown() method in ROS
        :return: `True` if the ros node is stopped/being stopped, `False` otherwise
        :rtype: bool
        """
        return rospy.is_shutdown()


class Logger(object):
    DEBUG = rospy.DEBUG
    INFO = rospy.INFO
    WARN = rospy.WARN
    ERROR = rospy.ERROR

    @staticmethod
    def debug(text):
        rospy.logdebug('[planner] {}'.format(text))

    @staticmethod
    def info(text):
        rospy.loginfo('[planner] {}'.format(text))

    @staticmethod
    def warning(text):
        rospy.logwarn('[planner] {}'.format(text))

    @staticmethod
    def error(text):
        rospy.logerr('[planner] {}'.format(text))
