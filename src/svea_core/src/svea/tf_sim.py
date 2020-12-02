#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

import tf2_ros
import tf_conversions
from geometry_msgs.msg import PoseWithCovarianceStamped
from svea_msgs.msg import VehicleState
import geometry_msgs.msg

def broadcast_pose(msg):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map"
    t.child_frame_id = "base_link"
    t.transform.translation.x = msg.x
    t.transform.translation.y = msg.y
    t.transform.translation.z = 0.22
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.yaw)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('tf_sim')
    rospy.Subscriber('/state',
                     VehicleState,
                     broadcast_pose,
                     )
    rospy.spin()
