#! /usr/bin/env python

#diagnosis node


import numpy as np
import math
import rospy
from av09_msgs.msg import *
from std_msgs.msg import *
from faultclass import Fault_obj
from symptomclass import Symptom_obj
import os

#hejhej


def Severity_check(a,b):
	
	if str(a.diag).split(',')[1]=='SEVERE':
		
		return True
	return False


class Diag:
	def __init__(self,slist,flist):
		self.diag='nofault,SAFE'
		self.deltadiag=0
		self.slist=slist
		self.flist=flist
		mat=[]
		amat=[]
		self.zero_time=rospy.Time.now()
		
		for ele in self.flist:
			mat.append(ele.P_symps)
			avec=[]
			for i in ele.P_symps:
				avec.append(1-i)
			amat.append(avec)
		self.npmat=np.array(mat) #P(S|F) matrix where rows symbolize each symptoms and the elements in rows is P(s|F)
		self.npamat=np.array(amat) #P(S_bar|F)

		
		

	def Diagnose(self,ctcorr,symptoms):

		timestamp=rospy.Time.now()-self.zero_time
		p_vec=[]
		for ele in self.flist:
			if ele.probmodel=='exp':
				p_vec.append(float(1-np.exp(-(timestamp.secs)/(float(ele.modelparam)*3600))))
			elif ele.probmodel=='const':
				p_vec.append(float(ele.modelparam))
		s_bar=[]
		for i in symptoms.newsymp:
			if i==1:
				s_bar.append(0)
			else:
				s_bar.append(1)

		npp_vec=np.array(p_vec)
		npsymp=np.array(symptoms.newsymp)
		nps_bar=np.array(s_bar)
		B=np.dot(np.transpose(self.npmat),np.transpose(npp_vec))
		C=np.dot(np.transpose(self.npamat),np.transpose(npp_vec))
		
		Psm=np.dot(np.transpose(npsymp),B)
		Psn=np.dot(np.transpose(nps_bar),C)
		A=[]
		sum=0
		for i,ele in enumerate(npp_vec):
			
			PsfPf=np.dot(self.npmat[i],np.transpose(npsymp))
			PsbfPs=np.dot(self.npamat[i],np.transpose(npsymp))
			A.append((np.dot(PsfPf,ele)/(Psm+0.001) + np.dot(PsbfPs,ele))/(Psn))
			sum=sum+A[i]
		
		A[-1]=A[-1]+0.01
		F=np.argmax(A)
		newdiag=str(self.flist[F].name + ',' + self.flist[F].severity)
		
		if newdiag == self.diag:
			self.deltadiag=0
		else:
			
			self.deltadiag=1
		self.diag=newdiag
		rospy.loginfo(newdiag)

class Symp:
	def __init__(self):
		self.oldsymp=None
		self.newsymp=self.oldsymp
		self.deltasymp=None
		self.detect_status=None
	def callback_symp(self, msg):
		self.oldsymp=self.newsymp
		a=msg.symp
		b=a.split(',')
		self.newsymp=[]
		for ele in b:
			self.newsymp.append(int(ele))
			
		self.deltasymp=msg.deltasymp
	def callback_detectstatus(self, msg):
		self.detect_state=msg.status1
		self.detect_status=msg.status2

class CTcorr:
	def __init__(self):
		self.ver=0
		self.Fcorr=0
		self.Arecc=0
		self.Request=0
		self.release_sd=0

	def callback(self, msg):
		self.ver=msg.ctver
		#self.Fcorr=msg.fault
		#self.Arecc=msg.recact
		#self.Request=msg.Request
		self.release_sd=msg.release
		#print(self.ver)




class Plan:
	def __init__(self,RC,action):
		self.RC=RC
		self.action=action

	def replan(self,Diag):
		diag=Diag.split(',')
		old_action=self.action
		old_rc=self.RC	
		if diag[1]=='SEVERE':
			self.action='EMERGENCY BREAK,CONTACT TOW'
			self.RC=2
		elif diag[1]=='NOTSEVERE':
			self.action='SLOW DOWN,DRIVE TO WORKSHOP'
			self.RC=3
		elif diag[1]=='SAFE':
			self.action='NO ACTION'
			self.RC=1
		print(self.action,str(self.RC))


class StateMachine:

	def __init__(self,state,pub_diag,slist,flist):
		self.state=state
		self.symp=Symp()
		self.CTcorr=CTcorr()
		self.Diag=Diag(slist,flist)
		self.plan=Plan(1,'NO ACTION')
		self.pub_diag=pub_diag
		self.ct_handshake='NO NEW DIAGNSOSIS'
		self.detect_handshake=''
		self.actuate_handshake='NO NEW PLAN'
	
	
	def Stateswitch(self):
		if self.state=='IDLE':
			return self.state_1()
		elif self.state=='DIAGNOSING':
			return self.state_2()
		elif self.state=='W4V': #W2V=wating for veification
			return self.state_3()
		elif self.state=='PLANNING':
			return self.state_4()
		elif self.state=='CTCONTROL':
			return self.state_5()
				
	
	#IDLE
	def state_1(self):
		self.detect_handshake='WAITING FOR SYMP'
		self.ct_handshake='NO NEW DIAGNOSIS'
		if self.symp.detect_status=='NEW SYMPTOMS':
			self.state='DIAGNOSING'
			


	def state_2(self):
		self.Diag.Diagnose(self.CTcorr,self.symp)
		self.detect_handshake='CONFIRMED NEW SYMP'
		if self.Diag.deltadiag==0:
			self.state='IDLE'
		else:
			self.state='W4V'



	def state_3(self):
		self.detect_handshake='DIAGNOSING SYMPTOMS'
		self.ct_handshake='WAITING FOR CT CONFIRMATION'
		if self.CTcorr.ver==1:
			self.ct_handshake='NEW DIAGNOSIS, CT CONFIRMED'
			self.state='PLANNING'
		
		
		elif Severity_check(self.Diag,self.CTcorr):
			self.ct_handshake='NEW DIAGNOSIS, SEVERE FAULT'
			self.state='PLANNING'


		if self.CTcorr.Request==1:
			self.state='CTCONTROL'
		diag_msg=diag()
		diag_msg.diag=self.Diag.diag
		diag_msg.deltadiag=self.Diag.deltadiag
		self.pub_diag.publish(diag_msg)

	def state_4(self): #PLAN
		self.plan.replan(self.Diag.diag)
		self.actuate_handshake='NEW PLAN'
		rospy.loginfo(self.actuate_handshake)

		if self.CTcorr.Request==1:
			self.state='CTCONTROL'
		elif self.CTcorr.ver==0:
			self.state='W4V'
		else:
			self.state='IDLE'

	def state_5(self): #CTCONTROL
		if not(self.CTcorr.Arecc =='None'):
			
			self.plan.action=self.CTcorr.Arecc
			self.plan.rc=2

			self.actuate_handshake='NEW PLAN'
		if self.CTcorr.Request==0:
			self.state='IDLE'



def main():
	rospy.init_node('diagnose_plan_node')
	r=rospy.Rate(5)
	THIS_FOLDER = os.path.dirname(os.path.abspath(__file__))
	my_file1 = os.path.join(THIS_FOLDER, 'symps.txt')
	my_file2 = os.path.join(THIS_FOLDER, 'faults.txt')
	
	sympdoc=open(my_file1,mode='r')
	list2=sympdoc.readlines()
	faultdoc=open(my_file2,mode='r')
	list1=faultdoc.readlines()
	sympssize=len(list2)
	faultssize=len(list1)

	
	symps_list=[]
	for i in list2:
		i="".join(i.split())
		
		
		objlist2=i.split(',')
		
		
		symps_list.append(Symptom_obj(objlist2,faultssize))


	faults_list=[]
	for i in list1:
		i="".join(i.split())
		objlist1=i.split(',')
	 	faults_list.append(Fault_obj(objlist1,sympssize))


	pub_diag=rospy.Publisher('diag',diag,queue_size=10)
	pub_diag_dec_status=rospy.Publisher('diagnose_decision_status',node_status,queue_size=10)
	pub_act=rospy.Publisher('action',action,queue_size=10)
	SM=StateMachine('IDLE',pub_diag,symps_list,faults_list)
	rospy.Subscriber('symptoms', symp, SM.symp.callback_symp )
	rospy.Subscriber('detection_status', node_status, SM.symp.callback_detectstatus )
	rospy.Subscriber('CTcorr', ctcorr, SM.CTcorr.callback )
	while not rospy.is_shutdown():
		act_msg=action()
		act_msg.action=SM.plan.action
		act_msg.rc=int(SM.plan.RC)
		pub_act.publish(act_msg)
		status_msg=node_status()
		status_msg.status1=SM.state
		status_msg.status2=SM.detect_handshake
		status_msg.status3=SM.actuate_handshake
		status_msg.status4=SM.ct_handshake
		pub_diag_dec_status.publish(status_msg)
		#print(SM.CTcorr.ver)
		SM.CTcorr.ver=1
		SM.Stateswitch()
		r.sleep()
		


if __name__ == '__main__':
	main()