#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud
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
import team4_project.task_switching.tree_nodes as tn
from team4_project.mapping2.updatemap import UpdateMap

TARGET_DISTANCE = 2e-1 # 2dm between targets

def print_tree(t):
    print("\033[2J\033[H")
    pt.display.print_ascii_tree(t, show_status=True)

class BehaviourTree(pt.trees.BehaviourTree):

    def __init__(self):
        initialization = pt.composites.Selector('Initialization', children=[
            tn.has_initialized(),
            RSequence('Initialize', children=[
                tn.next_waypoint_exists(),
                tn.replan_path(),
                tn.set_initialized()
            ])
        ])

        timeout = RSequence('Plannin timeout', children=[
            tn.planning_timed_out(),
            tn.move_waypoint(),
            tn.reset_planning_timeout()
        ])

        collision = RSequence("Collision", children=[
            tn.obstacle_detected(),
            tn.adjust_replan_speed(),
            tn.replan_path()
        ])

        following = pt.composites.Selector("Path planning", children=[
            RSequence("Reached waypoint?", children=[
                tn.is_at_waypoint(),
                tn.update_waypoint(),
                tn.interpolate_to_next_waypoint()
            ]),
            RSequence("Drive", children=[
                tn.set_speed(1.0),
                tn.replan_path()
            ])
        ])

        check_at_goal = RSequence("Is at goal?", children=[
            tn.is_at_waypoint(),
            tn.is_last_waypoint(),
            tn.set_speed(0)
        ])

        self.tree = RSequence("Behaviour Tree", children=[
            initialization,
            pt.composites.Selector('Behaviour', children=[
                RSequence("Pause?", children=[
                    tn.status_pause(),
                    tn.set_speed(0)
                ]),
                check_at_goal,
                timeout,
                collision,
                following
            ])
        ])

        super(BehaviourTree, self).__init__(self.tree)

def main():

    rospy.init_node('task_switching')

    show_tree_param = rospy.search_param('show_tree')
    show_tree = rospy.get_param(show_tree_param, True)

    rospy.loginfo('Waiting for initial position...')
    rospy.wait_for_message('/initialpose', PoseWithCovarianceStamped)

    map_updater = UpdateMap()

    waypoint_vis = rospy.Publisher("/vis_waypoints", PointCloud, queue_size=1, latch=True)

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
    path = get_path(map_updater, [start_state.x, start_state.y], [-3.89, -6.19]) #[-5.36, -1.66] [5.94, 14.5]
    path = list(reversed(path))

    vis_waypoint_msg = PointCloud()
    vis_waypoint_msg.header.frame_id = 'map'

    for p in path:
        tn.waypoints.append(np.array([p.pose.position.x, p.pose.position.y]))
        vis_point = Point32()
        vis_point.x = p.pose.position.x
        vis_point.y = p.pose.position.y
        vis_waypoint_msg.points.append(vis_point)

    waypoint_vis.publish(vis_waypoint_msg)
    # map_updater no longer needed. Delete to save computational resources
    del map_updater

    behaviour_tree = BehaviourTree()
    behaviour_tree.setup(timeout=10000)
    rospy.loginfo('Launching Behaviour Tree')
    rospy.sleep(1)
    while not rospy.is_shutdown():
        if show_tree:
            behaviour_tree.tick_tock(100, post_tick_handler=lambda t: print_tree(behaviour_tree.tree))
        else:
            behaviour_tree.tick_tock(100)

    rospy.spin()

if __name__ == '__main__':
    main()
