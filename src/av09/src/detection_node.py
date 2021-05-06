#! /usr/bin/env python

#Detection Node

import rospy
import math
from std_msgs.msg import *
from av09_msgs.msg import *
from av09_msgs.msg import node_status

class Signals:
	def __init__(self):
		self.signals='0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0'
		self.diag_status=0
	def callback_signal(self,msg):
		self.signals=msg.data
	def callback_diagstatus(self,msg):
		self.diag_state=msg.status1
		self.diag_status=msg.status2


def delta(newsymp,oldsymp):
	newsymp=newsymp.split(',')
	
	oldsymp=oldsymp.split(',')
	i=0
	while i<len(newsymp):
		if not (newsymp[i]==oldsymp[i]):
			deltasymp=1
			return 1 
		
		i=i+1
	return 0
	
	if deltasymp >0:
		return 1
	elif deltasymp ==0:
		return 0
class Symptom:

	def __init__(self):
		self.newsymp='0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0'
		self.oldsymp='0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0'
		self.deltasymp=0

	def create_symptoms(self,signals,state):
		self.oldsymp=self.newsymp
		self.newsymp=str(signals.signals)
		self.deltasymp=delta(self.newsymp,self.oldsymp)

class Statemachine:

	def __init__(self,state):
		self.state=state
		self.Symp=Symptom()
		self.signals=Signals()
		self.diag_handshake='NO NEW SYMPTOMS'		

	def Stateswitch(self):
		if self.state=='LOOKING':
			return self.state_1()
		elif self.state=='W4C':
			return self.state_2()
				
	
	#IDLE
	def state_1(self):
		self.Symp.create_symptoms(self.signals,self.state)
		self.diag_handshake='NO NEW SYMPTOMS'
		if self.Symp.deltasymp==1:
			self.state='W4C'

			


	def state_2(self):

		self.Symp.create_symptoms(self.signals,self.state)
		self.diag_handshake='NEW SYMPTOMS'
		if self.signals.diag_status == 'CONFIRMED NEW SYMP':
			self.state='LOOKING'


def main():

	rospy.init_node('detection_node')
	pub_symp=rospy.Publisher('symptoms',symp,queue_size=10)
	pub_det_status=rospy.Publisher('detection_status',node_status, queue_size=10)
	SM=Statemachine('LOOKING')

	r=rospy.Rate(10)
	while not rospy.is_shutdown():

		rospy.Subscriber('rawsignals', signals, SM.signals.callback_signal )
		rospy.Subscriber('diagnose_decision_status',node_status, SM.signals.callback_diagstatus )
		symp_msg=symp()	
		symp_msg.symp=SM.Symp.newsymp
		symp_msg.deltasymp=SM.Symp.deltasymp
		pub_symp.publish(symp_msg)
		det_status=node_status()
		det_status.status1=SM.state
		det_status.status2=SM.diag_handshake
		pub_det_status.publish(det_status)
		SM.Stateswitch()
		r.sleep()

	



if __name__ == '__main__':
	main()