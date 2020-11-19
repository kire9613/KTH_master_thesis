import py_trees as pt
import numpy as np
from svea_msgs.msg import VehicleState
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from team4_msgs.msg import Collision
import rospy

waypoints = []
current_waypoint = 0
initialized = False

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
        self.collision = msg.collision

class set_speed(pt.behaviour.Behaviour):
    def __init__(self, speed, return_val=pt.common.Status.SUCCESS):
        self.speed_pub = rospy.Publisher('/target_vel', Float32, queue_size=1)
        self.speed = speed
        self.return_val = return_val

        super(set_speed, self).__init__("Set speed %.1f" % self.speed)

    def update(self):
        self.speed_pub.publish(self.speed)
        return self.return_val
