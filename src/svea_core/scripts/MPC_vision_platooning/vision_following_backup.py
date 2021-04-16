#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
#from standard_msgs.msg import FLoat64
#<node pkg="svea_core" type="vision_following2.py" name="vision_following" output="screen"/>

rospy.init_node('vision_following')
imudata = Imu()

def imuinfo(data):
    global imudata
    imudata = data    

def main():
    rospy.sleep(1)
    rate = rospy.Rate(10)

    rospy.Subscriber('/imu/data', Imu, imuinfo, queue_size=10)
    while not rospy.is_shutdown():

		quat = (imudata.orientation.x,
                imudata.orientation.y,
                imudata.orientation.z,
                imudata.orientation.w)
		imu_roll, imu_pitch, imu_yaw = euler_from_quaternion(quat)

		roll = imu_roll*180/math.pi
		pitch = imu_pitch*180/math.pi
		yaw = imu_yaw*180/math.pi
	
		print(imudata)


		rate.sleep()

class Leader_vehicle(object):
    def __init__(self,)

if __name__ == '__main__':
    main()
