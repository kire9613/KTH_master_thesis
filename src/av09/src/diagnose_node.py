

#diagnosis node


import numpy
import math
import rospy


Class Diag:
	def __init__(self);
		self.diag=NONE
		self.deltadiag=NONE
	def Diagnose(self):

		self.diag='Something'

Class Symp:
	def __init__(self);
		self.symp=NONE
		self.deltasymp=NONE
	def callback(self, msg):
		self.symp=msg.Symp.symp
		self.deltasymp=msg.Symp.deltasymp

Class CTcorr:
	def __init__(self):
		self.ver=NONE
		self.Fcorr=NONE
		self.Arecc=NONE
		self.Request=NONE

	def callback(self, msg):
		self.ver=msg.CT.ver
		self.Fcorr=msg.CT.Fcorr
		self.Arecc=msg.CT.Arecc
		self.Request=msg.CT.Request
		

Class StateMachine:
	def __init__(self,state=1):
		self.state=state
		self.symp=Symp()
		self.CTcorr=CTcorr()
		self.Diag=Diag()
	
	def Stateswitch(self):
		return getattr(self, 'state_' + str(self.state), lambda: default)
				
	

	def state_1(self):
		if self.symp.deltasymp==1:
			self.state=2


	def state_2(self):
		self.Diag.Diagnose(self.Symp,self.CTcorr)
		self.state=3
		%self.symp.deltasymp=0
	def state_3(self):

		if self.symp.deltasymp==1:
			self.state=2
		elif Severity_check(self.Diag,self.CTcorr) or self.CTcorr.ver==1:
			self.state=4
		

	def state_4(self):
		publish(self.Diag.diag)
		if self.CTcorr.ver==0:
			self.state=3
		elif self.CTcorr.ver==1:
			self.state=1
		

def main():
	rospy.init_node(Diagnosis_node)
	
	
	SM=StateMachine(1)
	rospy.subscriber('\Symptom', msgClass, SM.symp.callback1 )
	rospy.subscriber('\CTcorr', msgClass, SM.CTcorr.callback2 )
	SM.stateswitch()


	rospy.spin()



if __name__ == '__main__':
	main()