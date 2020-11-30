import py_trees as pt
import numpy as np
from svea_msgs.msg import VehicleState
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point32
from std_msgs.msg import Float32, Bool
from team4_msgs.msg import Collision, AStarAction, AStarGoal
from sensor_msgs.msg import PointCloud
import rospy
import actionlib
from copy import deepcopy

TARGET_DISTANCE = 2e-1 # 2dm between targets
PLANNING_TIMEOUT = 20 # seconds

waypoints = []
current_path = Path()
current_waypoint = 0
initialized = False
collision_point = np.zeros((2,))
planning_start_time = None

def interpolate(start, end, distance):
    # Create n_steps points that are evenly distributed
    # along a line with distance TARGET_DISTANCE between them
    n_steps = int(round(np.linalg.norm(start-end)/distance))
    ts = np.linspace(0, 1, n_steps+1)
    for t in ts:
        target = start + t*(end-start)
        yield target

class status_pause(pt.behaviour.Behaviour):
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
        if np.all(waypoints[-1] == waypoints[current_waypoint]):
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
    def __init__(self):
        self.waypoint_vis = rospy.Publisher("/vis_waypoints", PointCloud, queue_size=1, latch=True)

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

class astar_plan_path(pt.behaviour.Behaviour):
    def __init__(self):
        self.targets_pub = rospy.Publisher('/targets', Path, queue_size=1, latch=True)
        self.waypoint_vis = rospy.Publisher("/vis_waypoints", PointCloud, queue_size=1, latch=True)
        self.planning_client = actionlib.SimpleActionClient('/astar_planning', AStarAction)
        self.planning_client.wait_for_server()

        self.is_planning = False

        super(astar_plan_path, self).__init__("Plan path")

    def update(self):
        global waypoints
        if not self.is_planning:
            rospy.loginfo("Replanning path...")
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
            return pt.common.Status.SUCCESS

    def terminate(self, new_status):
        if new_status == pt.common.Status.INVALID:
            # Obstacle no longer detected. Cancel replanning.
            self.planning_client.cancel_all_goals()
            self.is_planning = False

    def done_planning(self, state, msg):
        self.is_planning = False
        if state == actionlib.TerminalState.PREEMPTED:
            rospy.logerr("Planning stopped!")
            return
        if state != actionlib.TerminalState.SUCCEEDED:
            rospy.logerr("Replanning failed!")
            return
        if len(msg.path.poses) == 0:
            rospy.logerr("Replanning failed! Empty path")
            return

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
    def __init__(self):
        self.position = np.zeros((2,))
        self.speed_pub = rospy.Publisher('/target_vel', Float32, queue_size=1)
        self.state_sub = rospy.Subscriber('/state', VehicleState, self.cache_state)

        # Maximum speed when approaching a collision
        self.MAX_SPEED = 0.5
        # Minimum distance to collision
        self.MIN_DISTANCE = 2#1.5
        # Gain for speed regulation
        self.K = 0.7

        super(adjust_replan_speed, self).__init__("Adjust speed while replanning")

    def update(self):
        distance = np.linalg.norm(self.position - collision_point)
        speed = max(0, min(self.MAX_SPEED, self.K*(distance-self.MIN_DISTANCE)))
        self.speed_pub.publish(speed)
        return pt.common.Status.SUCCESS

    def cache_state(self, msg):
        self.position = np.array([msg.x, msg.y])
