#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from team4_project.planner.planner import get_path
from nav_msgs.msg import OccupancyGrid
from svea_msgs.msg import VehicleState
from nav_msgs.msg import Path
from std_msgs.msg import Float32
import math
import numpy as np
import py_trees as pt
#import py_trees_ros as ptr
from team4_project.task_switching.reactive_sequence import RSequence
import team4_project.task_switching.path_follow as pf

TARGET_DISTANCE = 2e-1 # 2dm between targets

def print_tree(t):
    print("\033[2J\033[H")
    pt.display.print_ascii_tree(t, show_status=True)

class BehaviourTree(pt.trees.BehaviourTree):

    def __init__(self):
        initialization = pt.composites.Selector('Initialization', children=[
            pf.has_initialized(),
            RSequence('Initialize', children=[
                pf.next_waypoint_exists(),
                pf.interpolate_to_next_waypoint(),
                pf.set_speed(1.0),
                pf.set_initialized()
            ])
        ])

        collision = RSequence("Collision", children=[
            pf.obstacle_detected()
        ])

        following = RSequence("Waypoint", children=[
            pf.is_at_waypoint(),
            pf.next_waypoint_exists(),
            pf.interpolate_to_next_waypoint()
        ])

        self.tree = RSequence("Behaviour Tree", children=[
            initialization,
            pt.composites.Selector('Behaviour', children=[
                collision,
                following
            ])
        ])

        super(BehaviourTree, self).__init__(self.tree)

def main():
    rospy.init_node('task_switching')

    rospy.loginfo('Waiting for initial position...')
    rospy.wait_for_message('/initialpose', PoseWithCovarianceStamped)

    # Nodes to wait for before startup
    nodes_wait = ['mapping']
    for node in nodes_wait:
        rospy.loginfo('Waitig for node %s...' % node)
        while not rospy.wait_for_message('/node_started/' + node, Bool).data:
            rospy.sleep(0.1)

    # Give positioning time to estimate vehicle state
    rospy.sleep(1)
    start_state = rospy.wait_for_message('/state', VehicleState)
    rospy.loginfo('Planning path...')
    path = get_path(rospy.wait_for_message('/map', OccupancyGrid), [start_state.x, start_state.y], [5.88, 14.8])
    path = list(reversed(path))
    for p in path:
        pf.waypoints.append(np.array([p.pose.position.x, p.pose.position.y]))

    behaviour_tree = BehaviourTree()
    behaviour_tree.setup(timeout=10000)
    rospy.sleep(1)
    while not rospy.is_shutdown():
        behaviour_tree.tick_tock(1, post_tick_handler=lambda t: print_tree(behaviour_tree.tree))

    rospy.spin()

if __name__ == '__main__':
    main()
