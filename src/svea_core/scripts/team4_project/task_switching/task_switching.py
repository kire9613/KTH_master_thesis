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
#q2
GOAL_LIST = [[2.6,0.2],[10.4, 11.8],[5.92, 14.7],[-6.77,-3.51],[-4.17, -6.03]]
#GOAL_LIST = [[5.92,14.7],[1.23,-1.97],[6.61,6.02],[4.73,13.1],[-0.77,5.01],[-3.33,-6.73]] # List of "goals" or waypoints for car to plan between initially
#q1 start pos: '17.9, 3.54, 3.1, 0'
#GOAL_LIST = [[11.9,3.64],[1.86,4],[-7.22,4.86],[-13.6,2.66],[-4.61,0.558],[3.32,-0.942],[11.9,-2.6],[12.3,2.7],[1.86,4.05]] # List of "goals" or waypoints for car to plan between initially

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
                tn.set_speed(tn.TARGET_VELOCITY),
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

    vis_waypoint_msg = PointCloud()
    vis_waypoint_msg.header.frame_id = 'map'

    for i in range(len(GOAL_LIST)):
        if i == 0: # If first waypoint, plan from start to waypoint
            path = get_path([start_state.x, start_state.y], GOAL_LIST[i]) #[-5.36, -1.66] [5.94, 14.5]
            path = list(reversed(path))
        else: # Else, plan from previous waypoint to next
            path = get_path(GOAL_LIST[i-1], GOAL_LIST[i])
            path = list(reversed(path))
            del path[0]

        for p in path:
            tn.waypoints.append(np.array([p.pose.position.x, p.pose.position.y]))
            vis_point = Point32()
            vis_point.x = p.pose.position.x
            vis_point.y = p.pose.position.y
            vis_waypoint_msg.points.append(vis_point)
    
    waypoint_vis.publish(vis_waypoint_msg)


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
