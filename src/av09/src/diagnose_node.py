#! /usr/bin/env python

#diagnosis node


import numpy
import math
import rospy
from av09_msgs.msg import *


class Diag:
	def __init__(self):
		self.diag=None
		self.deltadiag=None
	def Diagnose(self,ctcorr):

		self.diag='Something'
		rospy.loginfo('diagnosing')
		self.deltadiag=1

class Symp:
	def __init__(self):
		self.symp=None
		self.deltasymp=None
	def callback(self, msg):
		self.symp=msg.symp
		self.deltasymp=msg.deltasymp

class CTcorr:
	def __init__(self):
		self.ver=None
		self.Fcorr=None
		self.Arecc=None
		self.Request=None

	def callback(self, msg):
		self.ver=msg.ctver
		self.Fcorr=msg.fault
		self.Arecc=msg.recact
		self.Request=msg.ctreq
		

class StateMachine:
	def __init__(self,state,pub_ct,pub_diag):
		self.state=state
		self.symp=Symp()
		self.CTcorr=CTcorr()
		self.Diag=Diag()
		self.pub_ct=pub_ct
		self.pub_diag=pub_diag
	
	def Stateswitch(self):
		#getattr(self, 'state_' + str(self.state))
		if self.state==1:
			return self.state_1()
		elif self.state==2:
			return self.state_2()
		elif self.state==3:
			return self.state_3()
		else:
			return self.state_4()
				
	
	#IDLE
	def state_1(self):
		if self.symp.deltasymp==1:
			self.state=2
			print('state 1')


	def state_2(self):
		self.Diag.Diagnose(self.CTcorr)
		self.state=3
		print('state 2')
		#self.symp.deltasymp=0
	def state_3(self):
		print('state 3')
		if self.symp.deltasymp==1:
			self.state=2
		elif Severity_check(self.Diag,self.CTcorr) or self.CTcorr.ver==1:
			self.state=4
		

	def state_4(self):
		diag_msg=symp()
		diag_msg.diag=self.Diag.diag
		diag_msg.deltadiag=self.Diag.deltadiag
		self.pub_diag.publish(diag_msg)
		rospy.loginfo(diag_msg)
		if self.CTcorr.ver==0:
			self.state=3
		elif self.CTcorr.ver==1:
			self.state=1
		

def main():
	rospy.init_node('Diagnosis_node')
	
	

	pub_ct=rospy.Publisher('ctcorr',ctcorr,queue_size=10)
	pub_diag=rospy.Publisher('diag',diag,queue_size=10)
	SM=StateMachine(1,pub_ct,pub_diag)
	while not rospy.is_shutdown():
		rospy.Subscriber('symptoms', symp, SM.symp.callback )
		rospy.Subscriber('ctcorr', ctcorr, SM.CTcorr.callback )

		SM.Stateswitch()
		print('switching')


	



if __name__ == '__main__':
	main()