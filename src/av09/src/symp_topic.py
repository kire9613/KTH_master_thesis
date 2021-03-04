#! /usr/bin/env python

#Detection Node

import rospy
import math
from std_msgs.msg import *

class Signals:
    def __init__(self):
        self.signals=None

    def callback(msg):







def delta(symp):
	%calculate if new symptoms


Class Symptom:

	def __init__(self):
		self.symp=NONE
		self.deltasymp=0

	def callback(self,msg):
		self.symp=msg.Symp.symp
		self.deltasymp=delta(symp)
		




def main():
	rospy.init_node(Detection_node)
	pub=rospy.publisher('/symptoms', UInt8MultiArray,queue_size=10)
    
    symp_msg=UInt64Multiarray()
	
	Symp=Symp()
	rospy.subscriber('\Symptom', msgClass, Symp.callback )
	if Symp.deltasymp==1:
		publish(symp)

	rospy.spin()



if __name__ == '__main__':
	main()