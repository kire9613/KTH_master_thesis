import py_trees as pt
import numpy as np
from svea_msgs.msg import VehicleState
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from team4_msgs.msg import Collision, AStarAction, AStarGoal
import rospy
import actionlib
from copy import deepcopy

waypoints = []
current_waypoint = 0
initialized = False
replan_point = np.zeros((2,))

class has_initialized(pt.behaviour.Behaviour):
    def __init__(self):
        super(has_initialized, self).__init__('Has initialized?')

    def update(self):
        if initialized:
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE

class set_initialized(pt.behaviour.Behaviour):
    def __init__(self):
        super(set_initialized, self).__init__('Set initialized')

    def update(self):
        global initialized
        initialized = True
        return pt.common.Status.FAILURE

class is_at_waypoint(pt.behaviour.Behaviour):
    def __init__(self):
        self.position = np.array([0, 0])
        self.state_sub = rospy.Subscriber('/state', VehicleState, self.cache_state)
        super(is_at_waypoint, self).__init__("Is at waypoint?")

    def update(self):
        # The car has reached the waypoint if it is within 5dm
        if np.linalg.norm(self.position - waypoints[current_waypoint]) < 0.5:
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

class interpolate_to_next_waypoint(pt.behaviour.Behaviour):
    def __init__(self):
        self.TARGET_DISTANCE = 2e-1 # 2dm between targets
        self.targets_pub = rospy.Publisher('/targets', Path, queue_size=1, latch=True)
        super(interpolate_to_next_waypoint, self).__init__('Interpolate to next waypoint')

    def update(self):
        global current_waypoint

        current_waypoint += 1

        if current_waypoint >= len(waypoints):
            return pt.common.Status.FAILURE

        start = waypoints[current_waypoint-1]
        end = waypoints[current_waypoint]

        path_msg = Path()
        # Create n_steps points that are evenly distributed
        # along a line with distance TARGET_DISTANCE between them
        n_steps = int(round(np.linalg.norm(start-end)/self.TARGET_DISTANCE))
        ts = np.linspace(0, 1, n_steps+1)
        for t in ts:
            target = start + t*(end-start)
            p = PoseStamped()
            p.pose.position.x = target[0]
            p.pose.position.y = target[1]
            path_msg.poses.append(p)

        self.targets_pub.publish(path_msg)

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
        global replan_point
        self.collision = msg.collision
        replan_point[0] = msg.replan_point.x
        replan_point[1] = msg.replan_point.y

class set_speed(pt.behaviour.Behaviour):
    def __init__(self, speed, return_val=pt.common.Status.SUCCESS):
        self.speed_pub = rospy.Publisher('/target_vel', Float32, queue_size=1)
        self.speed = speed
        self.return_val = return_val

        super(set_speed, self).__init__("Set speed %.1f" % self.speed)

    def update(self):
        self.speed_pub.publish(self.speed)
        return self.return_val

class replan_path(pt.behaviour.Behaviour):
    def __init__(self):
        self.targets_pub = rospy.Publisher('/targets', Path, queue_size=1, latch=True)
        self.planning_client = actionlib.SimpleActionClient('/astar_planning', AStarAction)
        self.planning_client.wait_for_server()
        self.last_planning_pont = np.zeros((2,))

        self.is_planning = False

        super(replan_path, self).__init__("Replan path")

    def update(self):
        if not self.is_planning and (self.last_planning_pont != replan_point).any():
            rospy.loginfo("Replanning path...")
            self.is_planning = True
            self.last_planning_pont = deepcopy(replan_point)

            action_msg = AStarGoal()
            action_msg.x0 = replan_point[0]
            action_msg.y0 = replan_point[1]
            action_msg.xt = waypoints[current_waypoint][0]
            action_msg.yt = waypoints[current_waypoint][1]
            action_msg.smooth = False

            self.planning_client.send_goal(action_msg, done_cb=self.done_planning)

        if self.is_planning:
            return pt.common.Status.RUNNING
        else:
            return pt.common.Status.SUCCESS

    def terminate(self, new_status):
        if new_status == pt.common.Status.INVALID:
            # Obstacle no longer detected. Cancel replanning.
            self.planning_client.cancel_all_goals()
            self.is_planning = False

    def done_planning(self, state, msg):
        if state != actionlib.TerminalState.SUCCEEDED:
            rospy.logerr("Replanning failed!")
            return

        self.targets_pub.publish(msg.path)
        self.is_planning = False

class adjust_replan_speed(pt.behaviour.Behaviour):
    def __init__(self):
        self.position = np.zeros((2,))
        self.speed_pub = rospy.Publisher('/target_vel', Float32, queue_size=1)
        self.state_sub = rospy.Subscriber('/state', VehicleState, self.cache_state)

        # Maximum speed when approaching a collision
        self.MAX_SPEED = 0.5
        # Gain for speed regulation
        self.K = 0.7

        super(adjust_replan_speed, self).__init__("Adjust speed while replanning")

    def update(self):
        distance = np.linalg.norm(self.position - replan_point)
        speed = min(self.MAX_SPEED, self.K*distance)
        self.speed_pub.publish(0)
        return pt.common.Status.SUCCESS

    def cache_state(self, msg):
        self.position = np.array([msg.x, msg.y])
