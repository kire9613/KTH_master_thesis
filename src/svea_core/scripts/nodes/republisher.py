#!/usr/bin/env python

import rospy
import tf 

from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
from svea_msgs.msg import next_traj

chassis_height = 0.06 # [m] approx.

path_publisher = rospy.Publisher('path_plan', Path, queue_size=2)
target_publisher = rospy.Publisher('target', PointStamped, queue_size=2)

'''
The purpose of this class is to subscribe to different topics and then publish them under another topic as to
make them visible in rviz
'''


def publish_target(msg):
    """
    Publish target point for visualization in rviz

    :param target_publisher: ROS publisher to broadcast target with
    :type target_publisher: rospy.topics.Publisher
    """
    target_pt = PointStamped()
    target_pt.header.stamp = rospy.Time.now()
    target_pt.header.frame_id = 'map'
    target_pt.point.x = msg.x_coordinates[0]
    target_pt.point.y = msg.y_coordinates[0]
    target_pt.point.z = chassis_height
    target_publisher.publish(target_pt)

def lists_to_pose_stampeds(x_list, y_list, yaw_list=None, t_list=None):
    poses = []
    for i in range(len(x_list)):
        x = x_list[i]
        y = y_list[i]

        curr_pose = PoseStamped()
        curr_pose.header.frame_id = 'map'
        curr_pose.pose.position.x = x
        curr_pose.pose.position.y = y

        if not yaw_list is None:
            yaw = yaw_list[i]
            quat = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
            curr_pose.pose.orientation.x = quat[0]
            curr_pose.pose.orientation.y = quat[1]
            curr_pose.pose.orientation.z = quat[2]
            curr_pose.pose.orientation.w = quat[3]

        if not t_list is None:
            t = t_list[i]
            curr_pose.header.stamp = rospy.Time(secs = t)
        else:
            curr_pose.header.stamp = rospy.Time.now()

        poses.append(curr_pose)
    return poses

def publish_path(msg):
    """Publish trajectory visualization to rviz
    *t_list untested

    :param path_publisher: ROS publisher to broadcast trajectory with
    :type path_publisher: rospy.topics.Publisher
    :param x_list: x trajectory in [m]
    :type x_list: list
    :param y_list: y trajecotory in [m]
    :type y_list: list
    :param yaw_list: yaw trajectory in [rad], defaults to None
    :type yaw_list: list
    :param t_list: time trajectory in [s], defaults to None
    :type t_list: list
    """
    path = Path()
    path.header.stamp = rospy.Time.now()
    path.header.frame_id = 'map'

    x_list, y_list, yaw_list, t_list = msg.x_coordinates, msg.y_coordinates, None, None

    path.poses = lists_to_pose_stampeds(x_list, y_list, yaw_list, t_list)
    path_publisher.publish(path)


if __name__ == '__main__':
    rospy.init_node("visualization_node")

    rospy.Subscriber('/TrajMessage',
                     next_traj,
                     publish_path)

    rospy.Subscriber('/TargetPoint',
                     next_traj,
                     publish_target)


    rospy.spin()
