#!/usr/bin/env python
"""
Some examples of how to interact with the ArucoInterface() class
Developed for: KTH Smart Mobility Lab
"""

__license__ = "MIT"
__maintainer__ = "Kyle Coble"
__email__ = "coble@kth.se"
__status__ = "Development"

import sys
import os
import time
import math
import rospy

from geometry_msgs.msg import Pose, PoseStamped
from fiducial_msgs.msg import FiducialTransformArray
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

from svea.aruco.aruco_interfaces import *

"""
Some necessary initializations
"""
rospy.init_node('aruco_pitch')
aruco_detect = ArucoInterface(vehicle_frame='base_link', camera_frame='camera_link').start()
vehicle_state = State() # Defaults to origin
posepub = rospy.Publisher('/observed_pose', PoseStamped, queue_size=1)
offset_pub = rospy.Publisher('/offset_pose', PoseStamped, queue_size=1)
pitch_pub = rospy.Publisher('/observed_pitch', Float64, queue_size=1)
leaderpitch_pub = rospy.Publisher('/leadervehicle_pitch', Float64, queue_size=1)
markervelocity_pub = rospy.Publisher('/marker_velocity', Float64, queue_size=1)
markervisibility_pub = rospy.Publisher('/markervisibility', Bool, queue_size=1)
marker_dict = {}
rospy.sleep(1)


imudata = Imu()

def imuinfo(data):
    global imudata
    imudata = data



def get_imu_pitch():

    quat = (imudata.orientation.x,
        imudata.orientation.y,
        imudata.orientation.z,
        imudata.orientation.w)

    _, imu_pitch, _ = euler_from_quaternion(quat)

    pitch = imu_pitch*180/math.pi

    return pitch

def main():
    theta = 0.
    markervelocity = 0.
    d = 0.
    d_previous = 0.
    """
    Select which of the example functions
    to run or define your own actions
    """
    add_marker_to_dictionary(marker_dict, 2)
    #add_marker_to_dictionary(marker_dict, 14)
    #print(marker_dict)

    rospy.Subscriber('/imu/data', Imu, imuinfo, queue_size=1)

    rate = rospy.Rate(10) # [hz]
    dt = 1./10.
    while not rospy.is_shutdown():
        """
        Check if a specific Marker() object is currently detected
        """
        marker_2_seen = aruco_detect.check_for_marker(marker_dict['added marker 2'])
        if marker_2_seen:
            #print("Marker 2 is detected")
            offset = [0, 0, 0]
            theta = leader_pitch(marker_dict['added marker 2'], offset) 
            d = publish_detected_markers()
            markervelocity = (d_previous-d)/dt #relative velocity of marker as seen by camera
            d_previous = d
            
            
        markervisibility_pub.publish(Bool(marker_2_seen))
        #theta observed marker pitch
        
        #alpha from follower IMU
        alpha = get_imu_pitch() 

        leaderalpha = theta+alpha

        leaderpitch_pub.publish(Float64(leaderalpha))
        markervelocity_pub.publish(Float64(markervelocity))
        rate.sleep()


def add_marker_to_dictionary(marker_dict, fid):
    """
    Add a marker to the marker dictionary with any arbitrary pose
    """
    float_pose = Pose()
    float_pose.position.y = 1.3
    float_pose.orientation = Quaternion(*quaternion_from_euler(math.pi/2, 0, -math.pi/2))
    marker_dict['added marker '+str(fid)] = Marker(fid, pose=float_pose, dim=0.14)


def publish_detected_markers():
    """
    Get the fiducial id of all visible markers
    """
    vis_markers = aruco_detect.visible_markers()
    #print(vis_markers)

    """
    Get the detected Marker() objects included in the dictionary
    NOTE: If a detected marker is not in the marker
    dictionary, it will not be returned in marker_objects
    """
    marker_objects = aruco_detect.get_markers_from_dict(vis_markers, marker_dict)
    for marker in marker_objects:
        #print("Marker id", marker.fid)
        """
        Get marker position as PoseStamped msg
        """
        stamped_pose = aruco_detect.marker_pose_stamped(vehicle_state, marker)
        """
        Publish marker pose (to visualize in RVIZ)
        """
        posepub.publish(stamped_pose)
        """
        Update pose in Marker() object
        """
        marker.pose = stamped_pose.pose

    return marker.pose.position.z


def publish_an_offset_pose(marker, offset_vec):
    """
    Publish a pose offset from the aruco marker
    """
    s = vehicle_state
    m = marker
    vec = offset_vec #[1.5, 1.0, math.pi/4]
    off_pose = aruco_detect.relative_pose_from_marker(s, m, vec, facing_marker=True)
    if off_pose:
        offset_pub.publish(off_pose)


def move_state(state):
    """
    Drives the base_link (state) in a circle
    Marker pose is based on the transform from base_link
    to marker and the base_link (state) pose in the map frame
    """
    state.v = 0#0.1
    state.x += 0#state.v*math.cos(state.yaw)
    state.y += 0#state.v*math.sin(state.yaw)
    state.yaw += 0#math.pi/45

def leader_pitch(marker, offset_vec):
    """
    Returns and publishes the pitch off an ArUco marker in degrees
    """
    m = marker
    s = vehicle_state
    vec = offset_vec
    angle = aruco_detect.detected_pitch_change(s, m, vec, facing_marker=True)
    if angle:
        pitch_pub.publish(Float64(angle))
        return angle

if __name__ == '__main__':
    main()
