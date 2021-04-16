#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

def imuinfo(data):
    global imudata
    imudata = data 

def start_imu_collect():
    rospy.init_node('imu_interfaces')
    imudata = Imu()
    rospy.Subscriber('/imu/data', Imu, imuinfo, queue_size=10)

def get_imu_pitch():

    quat = (imudata.orientation.x,
	    imudata.orientation.y,
	    imudata.orientation.z,
	    imudata.orientation.w)
    _, imu_pitch, _ = euler_from_quaternion(quat)

    pitch = imu_pitch*180/math.pi

    return pitch
