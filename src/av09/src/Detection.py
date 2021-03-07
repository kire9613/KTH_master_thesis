#! /usr/bin/env python

#Detection Node

import rospy
import math
from std_msgs.msg import *
from av09_msgs.msg import *

class Signals:
    def __init__(self):
        self.signals=0

    def callback(self,msg):
		self.signals=msg.data


def delta(newsymp,oldsymp):
	#calculate if new symptoms
	deltasymp=abs(newsymp-oldsymp)
	if deltasymp >0:
		return 1
	else:
		return 0

class Symptom:

	def __init__(self):
		self.newsymp=0
		self.oldsymp=0
		self.deltasymp=0

	def create_symptoms(self,signals):
		self.oldsymp=self.newsymp
		self.newsymp=signals.signals
		
		self.deltasymp=delta(self.newsymp,self.oldsymp)
		




def main():
	signal=Signals()
	Symp=Symptom()
	rospy.init_node('Detection_node')
	while not rospy.is_shutdown():

		
		pub_symp=rospy.Publisher('symptoms',symp,queue_size=10)
		symp_msg=symp()

		rospy.Subscriber('rawsignals', signals, signal.callback )
		Symp.create_symptoms(signal)
		if Symp.deltasymp==1:
			symp_msg.symp=Symp.newsymp
			symp_msg.deltasymp=Symp.deltasymp
			pub_symp.publish(symp_msg)
			rospy.loginfo(symp_msg)

	



if __name__ == '__main__':
	main()