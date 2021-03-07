#! /usr/bin/env python

from std_msgs.msg import *
import math
from av09_msgs.msg import signals
import rospy




def main():
    while not rospy.is_shutdown():

        rospy.init_node('simulation_node')
        pub_signals=rospy.Publisher('rawsignals',signals,queue_size=5)
        signal_msg=signals()
        signal_msg.data=int(input('Ange symptom: '))
        pub_signals.publish(signal_msg)
        #rospy.loginfo(signal_msg)
    
if __name__ == '__main__':
	main()