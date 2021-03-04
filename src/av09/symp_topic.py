#! /usr/bin/env python

#Detection Node

import rospy
import math
from std_msgs.msg import *

class Signals:
    def __init__(self):
        self.signals=None

    def callback(msg):


rospy.init_node('symp_topic')
pub=rospy.publisher('/symptoms', UInt8MultiArray,queue_size=10)
signals=Signals()
sub=rospy.subscriber('/Signals',UInt8Multiarray,queue_size=10)
symp_msg=UInt64Multiarray()

rate=rospy.Rate(1)

while not rospy.is_shutdown():
    pub.publish(symp_msg)
    rate.sleep()