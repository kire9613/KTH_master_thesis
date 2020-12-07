#!/usr/bin/env python  
import math
import roslib
import rospy
import numpy as np
import svea_msgs.msg
from sensor_msgs.msg import LaserScan 
from svea_msgs.msg import emergency_break 
def callback(msg):
    emergency_break_msg = emergency_break()
    angles=[-15,-10,-5,0,5,10,15] #Angles to check Lidar range
    emergency_break_msg.emergency_break = False
    coefficient = msg.angle_increment*(180/math.pi)
    
    for ang in angles:
        ang_index=int(540+round(ang/coefficient))
        #print(msg.ranges[ang_index])
        if msg.ranges[ang_index] < 0.3:
            emergency_break_msg.emergency_break = True
            print("Emergency break")
    pub = rospy.Publisher('emergency_break', emergency_break, queue_size=10)
    pub.publish(emergency_break_msg)

if __name__ == '__main__':
    rospy.init_node("emergency_break")
    rospy.Subscriber('/scan',
                     LaserScan,
                     callback)
    rospy.spin()



 
