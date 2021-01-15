import py_trees as pt
import numpy as np
from svea_msgs.msg import VehicleState
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped, Point32, PoseWithCovarianceStamped, Quaternion
from std_msgs.msg import Float32, Bool
from team4_msgs.msg import Collision, AStarAction, AStarGoal
from sensor_msgs.msg import PointCloud
from tf.transformations import quaternion_from_euler
import rospy
import actionlib
from copy import deepcopy

TARGET_DISTANCE = 2e-1 # 2dm between targets (path between waypoints)
PLANNING_TIMEOUT = 20 # timeout in seconds, move waypoint and replan after this time
TARGET_VELOCITY = 1.0 # m/s, default speed for car when not turning/braking

waypoints = [] # global path
# Used to check collisions in current path
current_path = Path() 
# Save current "destination", can send to planner
current_waypoint = 0 
# Initialize tree once, then set this to True
initialized = False 
 # Where in [x,y] is the collision?
collision_point = np.zeros((2,))
# Used to determine when planning timeout occurs
planning_start_time = None 
# used to reset position if localization is lost
last_pose = None 
# set to True if |last_pose-current_pose| is too large
localization_is_lost = False 

def interpolate(start, end, distance):
    """
    Creates n_steps points between start and end that are evenly distributed
    along a line with distance TARGET_DISTANCE between points.
    """
    n_steps = int(round(np.linalg.norm(start-end)/distance))
    ts = np.linspace(0, 1, n_steps+1)
    for t in ts:
        target = start + t*(end-start)
        yield target

class status_pause(pt.behaviour.Behaviour):
    """
    Listen to topic /pause if car is paused or not, sets system to the corresponding states runing/paused.
    Messaged posted to topic by using start_pause.py
    """
    def __init__(self):
        self.status_pause_sub = rospy.Subscriber('/pause', Bool, self.cache_pause)
        self.pause = True

        super(status_pause, self).__init__('Status pause')

    def cache_pause(self, msg):
        self.pause = msg.data

    def update(self):
        if self.pause:
            return  pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE

class has_initialized(pt.behaviour.Behaviour):
    """ Checks if system is initialized """
    def __init__(self):
        super(has_initialized, self).__init__('Has initialized?')

    def update(self):
        if initialized:
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE

class set_initialized(pt.behaviour.Behaviour):
    """ Sets system initialization status """
    def __init__(self):
        super(set_initialized, self).__init__('Set initialized')

    def update(self):
        global initialized
        initialized = True
        return pt.common.Status.FAILURE

class is_at_waypoint(pt.behaviour.Behaviour):
    """
    Returns True if vehicle is located at the current waypoint, else False.
    Accepted distances to waypoints: 0.65 for final waypoint, else 2.
    """

    def __init__(self):
        self.position = np.array([0, 0])
        self.state_sub = rospy.Subscriber('/state', VehicleState, self.cache_state)
        super(is_at_waypoint, self).__init__("Is at waypoint?")

    def update(self):
        if np.all(waypoints[current_waypoint] == waypoints[-1]):
            margin = 0.65
        else:
            margin = 2

        if np.linalg.norm(self.position - waypoints[current_waypoint]) < margin:
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE

    def cache_state(self, msg):
        self.position = np.array([msg.x, msg.y])

class is_last_waypoint(pt.behaviour.Behaviour):
    def __init__(self):
        super(is_last_waypoint, self).__init__("Is last waypoint?")

    def update(self):
        # The car has reached the waypoint if it is within 5dm
        if current_waypoint == len(waypoints)-1:
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE

    def cache_state(self, msg):
        self.position = np.array([msg.x, msg.y])

class next_waypoint_exists(pt.behaviour.Behaviour):
    def __init__(self):
        super(next_waypoint_exists, self).__init__("Next waypoint exists?")

    def update(self):
        if (current_waypoint + 1) < len(waypoints):
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE

class update_waypoint(pt.behaviour.Behaviour):
    """ Increments the index of the current waypoint """
    def __init__(self):
        super(update_waypoint, self).__init__('Update waypoint')

    def update(self):
        rospy.loginfo("Updating waypoint!")
        global current_waypoint
        current_waypoint += 1

        if current_waypoint >= len(waypoints):
            return pt.common.Status.FAILURE

        return pt.common.Status.SUCCESS

class interpolate_to_next_waypoint(pt.behaviour.Behaviour):
    """
    Extend the path with a line segment by interpolating from current waypoint
    (end of already obtained path) to the following waypoint.
    """
    def __init__(self):
        self.targets_pub = rospy.Publisher('/targets', Path, queue_size=1, latch=True)
        super(interpolate_to_next_waypoint, self).__init__('Interpolate to next waypoint')

    def update(self):
        global current_waypoint, current_path

        if current_waypoint+1 >= len(waypoints):
            return pt.common.Status.SUCCESS

        start = waypoints[current_waypoint]
        end = waypoints[current_waypoint+1]

        for tgt in interpolate(start, end, TARGET_DISTANCE):
            p = PoseStamped()
            p.pose.position.x = tgt[0]
            p.pose.position.y = tgt[1]
            current_path.poses.append(p)

        self.targets_pub.publish(current_path)

        return pt.common.Status.SUCCESS

class obstacle_detected(pt.behaviour.Behaviour):
    def __init__(self):
        self.collision = False
        self.collision_sub = rospy.Subscriber('/collision', Collision, self.cache_collision)

        super(obstacle_detected, self).__init__("Obstacle detected?")

    def update(self):
        if self.collision:
            return pt.Status.SUCCESS
        else:
            return pt.Status.FAILURE

    def cache_collision(self, msg):
        global collision_point
        self.collision = msg.collision
        collision_point[0] = msg.collision_point.x
        collision_point[1] = msg.collision_point.y

class set_speed(pt.behaviour.Behaviour):
    def __init__(self, speed, return_val=pt.common.Status.SUCCESS):
        self.speed_pub = rospy.Publisher('/target_vel', Float32, queue_size=1)
        self.speed = speed
        self.return_val = return_val

        super(set_speed, self).__init__("Set speed %.1f" % self.speed)

    def update(self):
        self.speed_pub.publish(self.speed)
        return self.return_val

class planning_timed_out(pt.behaviour.Behaviour):
    def __init__(self):
        super(planning_timed_out, self).__init__("Planning timed out?")

    def update(self):
        if planning_start_time is None:
            return pt.common.Status.FAILURE

        if (rospy.Time.now() - planning_start_time).to_sec() > PLANNING_TIMEOUT:
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE

class move_waypoint(pt.behaviour.Behaviour):
    """ If waypoint unreachable in planning, move waypoint towards next waypoint alang a straight line """
    def __init__(self):
        self.waypoint_vis = rospy.Publisher("/vis_waypoints", PointCloud, queue_size=1, latch=True)

        self.WAYPOINT_CONVERGED_DISTANCE = 0.3

        super(move_waypoint, self).__init__("Remove waypoint")

    def update(self):
        global waypoints

        # If waypoint we are planning to is not the last
        # waypoint, move it clser to the next one
        if current_waypoint+1 < len(waypoints)-1:
            p1 = waypoints[current_waypoint+1]
            p2 = waypoints[current_waypoint+2]

            # Move half the distance to p2
            p1 = p1 + 0.5*(p2-p1)

            # If we move the waypoint on top of the next
            # one remove it
            if np.linalg.norm(p1-p2) <= self.WAYPOINT_CONVERGED_DISTANCE:
                del waypoints[current_waypoint+1]
            else:
                waypoints[current_waypoint+1] = p1

            vis_waypoint_msg = PointCloud()
            vis_waypoint_msg.header.frame_id = 'map'

            for p in waypoints:
                vis_point = Point32()
                vis_point.x = p[0]
                vis_point.y = p[1]
                vis_waypoint_msg.points.append(vis_point)

            self.waypoint_vis.publish(vis_waypoint_msg)

            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE

class reset_planning_timeout(pt.behaviour.Behaviour):
    def __init__(self):
        super(reset_planning_timeout, self).__init__("Reset planning timeout")

    def update(self):
        global planning_start_time
        planning_start_time = rospy.Time.now()
        return pt.common.Status.SUCCESS

class replan_path(pt.behaviour.Behaviour):
    """ Calls A* planner to get path from vehicle position to current waypoint. """

    def __init__(self):
        self.targets_pub = rospy.Publisher('/targets', Path, queue_size=1, latch=True)
        self.waypoint_vis = rospy.Publisher("/vis_waypoints", PointCloud, queue_size=1, latch=True)
        self.planning_client = actionlib.SimpleActionClient('/astar_planning', AStarAction)
        self.planning_client.wait_for_server()

        self.is_planning = False
        self.has_planned = False

        super(replan_path, self).__init__("Replan path")

    def update(self):
        global waypoints, planning_start_time

        if not self.is_planning and not self.has_planned:
            rospy.loginfo("Replanning path...")
            planning_start_time = rospy.Time.now()
            self.is_planning = True

            if current_waypoint+1 < len(waypoints):
                target = waypoints[current_waypoint+1]
            else:
                target = waypoints[current_waypoint]

            state_msg = rospy.wait_for_message('/state', VehicleState)

            action_msg = AStarGoal()
            action_msg.x0 = state_msg.x
            action_msg.y0 = state_msg.y
            action_msg.theta0 = state_msg.yaw
            action_msg.xt = target[0]
            action_msg.yt = target[1]
            action_msg.smooth = False

            self.planning_client.send_goal(action_msg, done_cb=self.done_planning)

        if self.is_planning:
            return pt.common.Status.RUNNING
        else:
            self.has_planned = False
            return pt.common.Status.SUCCESS

    def terminate(self, new_status):
        if new_status == pt.common.Status.INVALID:
            # Obstacle no longer detected. Cancel replanning.
            self.planning_client.cancel_all_goals()
            self.is_planning = False

    def done_planning(self, state, msg):
        global current_path
        self.is_planning = False
        self.has_planned = True
        if state == actionlib.TerminalState.PREEMPTED:
            rospy.loginfo("Planning stopped!")
            return
        if state != actionlib.TerminalState.SUCCEEDED:
            rospy.logerr("Replanning failed!")
            return
        if len(msg.path.poses) == 0:
            rospy.logerr("Replanning failed! Empty path")
            return

        rospy.loginfo("Planning done!")
        current_path = deepcopy(msg.path)
        self.targets_pub.publish(msg.path)

class init_path(pt.behaviour.Behaviour):
    def __init__ (self):
        self.targets_pub = rospy.Publisher('/targets', Path, queue_size=1, latch=True)
        super(init_path, self).__init__("Init path")

    def update(self):
        veh_state = rospy.wait_for_message('/state', VehicleState)
        path_msg = Path()
        p = PoseStamped()
        p.pose.position.x = veh_state.x
        p.pose.position.y = veh_state.y
        path_msg.poses.append(p)
        self.targets_pub.publish(path_msg)

        return pt.common.Status.SUCCESS

class adjust_replan_speed(pt.behaviour.Behaviour):
    """ Adjust vehicle speed when replanning due to detected obstacle"""
    def __init__(self):
        self.position = np.zeros((2,))
        self.speed_pub = rospy.Publisher('/target_vel', Float32, queue_size=1)
        self.state_sub = rospy.Subscriber('/state', VehicleState, self.cache_state)

        # Maximum speed when approaching a collision
        self.MAX_SPEED = TARGET_VELOCITY
        # Minimum distance to collision
        self.MIN_DISTANCE = 2#1.5
        # Gain for speed regulation
        self.K = 0.7

        super(adjust_replan_speed, self).__init__("Adjust speed while replanning")

    def update(self):
        distance = np.linalg.norm(self.position - collision_point)
        speed = max(0, min(self.MAX_SPEED, self.K*(distance-self.MIN_DISTANCE)))
        if speed <= 0.25:
            speed = 0
        self.speed_pub.publish(speed)
        return pt.common.Status.SUCCESS

    def cache_state(self, msg):
        self.position = np.array([msg.x, msg.y])

class is_lost(pt.behaviour.Behaviour):
    def __init__(self):
        self.state_sub = rospy.Subscriber('/state', VehicleState, self.state_callback)

        self.LOST_THRESHOLD = 2

        super(is_lost, self).__init__('Localization is lost?')

    def update(self):
        if localization_is_lost:
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE

    def state_callback(self, state):
        global last_pose, localization_is_lost

        if last_pose is None:
            pose = Pose()
            pose.position.x = state.x
            pose.position.y = state.y
            pose.orientation = quaternion_from_euler(0, 0, state.yaw)
            last_pose = pose
            return

        if not localization_is_lost:
            pos = np.array([state.x, state.y])
            last_pos = np.array([last_pose.position.x, last_pose.position.y])

            if  np.linalg.norm(pos - last_pos) > self.LOST_THRESHOLD:
                localization_is_lost = True
                rospy.logwarn('Lost localization! Trying to reset.')
            else:
                pose = Pose()
                pose.position.x = state.x
                pose.position.y = state.y
                q = quaternion_from_euler(0, 0, state.yaw)
                pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
                last_pose = pose

class reset_localization(pt.behaviour.Behaviour):
    def __init__(self):
        self.loc_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)

        super(reset_localization, self).__init__('Reset localization')

    def update(self):
        global localization_is_lost

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose = last_pose
        msg.pose.covariance = np.zeros((36,)).tolist()

        self.loc_pub.publish(msg)

        localization_is_lost = False

        return pt.common.Status.SUCCESS
