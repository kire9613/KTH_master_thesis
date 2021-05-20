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
from RUL_model import *
from svea.states import VehicleState


#
v_dict={'Vnormal':1,'Vred':0.3}
#Hard-coded distances from workshops to end goal
dist_dict={'W1:FIN':36.86,'W2:FIN':15.55}
default_init_pt = [0.0, 0.0, 0.0, 0.0]


#Checks if fault is severe
def Severity_check(a,b):
	
	if str(a.diag).split(',')[1]=='SEVERE':
		
		return True
	return False

#Class for diagnosis. Methods include Diagnose() 
class Diag:
	def __init__(self,slist,flist):
		self.diag='nofault,SAFE'
		self.deltadiag=0
		self.slist=slist
		self.flist=flist
		self.zero_time=rospy.Time.now()
		self.gsh=100

		#Create the matrix P(S|F)
		mat=[]
		for ele in self.flist:
			mat.append(ele.P_symps)
			avec=[]
			for i in ele.P_symps:
				avec.append(1-i)
			
		self.npmat=np.array(mat) #P(S|F) matrix where rows symbolize each symptoms and the elements in rows is P(s|F)
		

		
		
	#The method calculates the likeliest diagnosis given a set of symptoms and inherent knowledge of the faults.
	def Diagnose(self,ctcorr,symptoms):

		#Calculation of time since system start
		timestamp=rospy.Time.now()-self.zero_time
		
		#P_vec is probability of all faults. failure of some components are exponentially distributed and some have static probs.
		p_vec=[]
		for ele in self.flist:
			if ele.probmodel=='exp':
				p_vec.append(float(1-np.exp(-(timestamp.secs)/(float(ele.modelparam)*3600))))
			elif ele.probmodel=='const':
				p_vec.append(float(ele.modelparam))

		npp_vec=np.array(p_vec)
		npsymp=np.array(symptoms.newsymp)
		#P(S|F)*P(F), for all faults
		B=np.dot(np.transpose(self.npmat),np.transpose(npp_vec))
		
		#S*P(S|F)*P(F), gives us P(S|F)*P(F) for all possible faults given symptoms
		Psm=np.dot(np.transpose(npsymp),B)
		A=[]
		sum=0
		for i,ele in enumerate(npp_vec):
			
			PsfPf=np.dot(self.npmat[i],np.transpose(npsymp))
			A.append(np.dot(PsfPf,ele)/(Psm))
			sum=sum+A[i]

		F=np.argmax(A)
		self.prob_fault=np.max(A)
		newdiag=str(self.flist[F].name + ',' + self.flist[F].severity)
		
		if newdiag == self.diag:
			self.deltadiag=0
		else:
			
			self.deltadiag=1
		self.diag=newdiag
		

		#When symp vector is all zeros, the calculation returns NaN as a division with zero occurs. The following code diagnoses the no-symptoms case
		for ele in symptoms.newsymp:
			if not ele==0:
				return
		self.prob_fault=1
		newdiag=str(self.flist[9].name + ',' + self.flist[9].severity)
		self.diag=newdiag
		self.deltadiag=1

#Class for representing the symptoms. Includes callback functions for the ROS communication with detection node 
class Symp:
	def __init__(self):
		self.deltasymp=None
		self.detect_status=None
	def callback_symp(self, msg):
		a=msg.symp
		b=a.split(',')
		self.newsymp=[]
		for ele in b:
			self.newsymp.append(int(ele))
			
		self.deltasymp=msg.deltasymp
	def callback_detectstatus(self, msg):
		self.detect_state=msg.status1
		self.detect_status=msg.status2

#Class for representing control tower communication. Includes callback function for ROS comm with control tower.
class CTcorr:
	def __init__(self):
		self.ver=0
		self.Fcorr=0
		self.Arecc=0
		self.Request=0
		self.release_sd=0

	def callback(self, msg):
		self.ver=msg.ctver
		self.Fcorr=msg.fault
		self.Arecc=msg.recact
		self.Request=msg.Request
		self.release_sd=msg.release



#Class for representing the active plan. Includes callback functions for ROS comm
#and a method for replanning given diagnosis and prognosis
class Plan:
	def __init__(self,vl,RC,mgoal,wgoal):
		self.vl=vl
		self.mgoal=mgoal
		self.wgoal=wgoal
		self.RC=RC
		self.approxDT

	def vhd_callback(self,msg):
		self.dist_W1=msg.dist_w1
		self.dist_W2=msg.dist_w2
		self.dist_mgoal=msg.dist_mgoal
		self.lastnode=msg.lastN
		self.nextnode=msg.nextN

	def actuation_callback(self,msg):
		self.operational_state=msg.status1
		self.actuation_status=msg.status2

	#Method for finding new plan given a prognosis of faults and active fault in vehicle.
	#The planning consists of a prognosis, action feasibility analysis and downtime analysis for the action space.
	def replan(self,Diag,prognosis):
		diag=Diag.split(',')
		
		if diag[1]=='SEVERE':
			self.vl=0
			self.mgoal='FIN'
			self.wgoal='None'
			self.RC=2
			return
		prognosis.prognose(diag,self.dist_W1,self.dist_W2,self.dist_mgoal)
		
		feasible_plans={}
		downtime_feasible={}

		#Feasability analysis
		for i, risk in enumerate(prognosis.risk_to_goals):
			#0.1 is set as the risk limit. In future, this number will be set by user
			if risk <= 0.1:
				feasible_plans[prognosis.plan_list[i]]=True
				downtime_feasible[prognosis.plan_list[i]]=self.downtime(prognosis.plan_list[i],self.dist_W1,self.dist_W2,self.dist_mgoal)
			else:
				feasible_plans[prognosis.plan_list[i]]=False

		for ele in prognosis.plan_list:
			try:
				print(ele, downtime_feasible[ele])
			except:
				print('')

		#Find the optimal plan (speed, route)
		opt_plan = min(downtime_feasible, key=downtime_feasible.get)
		#approximate total downtime
		self.approxDT=downtime_feasible[opt_plan]-self.downtime('Vnormal:mgoal')
		if opt_plan=='stop':
			self.vl=0
			self.mgoal='FIN'
			self.wgoal='None'
			self.RC=2
		else:
			if self.actuation_status=='AT WORKSHOP':
				self.vl=1
				
			else:
				self.vl=v_dict[opt_plan.split(':')[0]]
				self.mgoal='FIN'
				self.wgoal=opt_plan.split(':')[1]
				self.RC=3
		#rospy.loginfo('New plan',str(self.vl),str(self.mgoal),str(self.wgoal),str(self.RC))
		print(self.vl)
		print(self.mgoal)
		print(self.wgoal)
		print(self.RC)



	#Method calculates downtime for a specific route with a specific speed. Hard-coded for a specific map.		
	def downtime(self,plan,dist_W1,dist_W2,dist_mgoal):
		plan_list=plan.split(':')
		if plan=='stop':
			return 100000
		if plan_list[1]=='W1':
			time=dist_W1/v_dict[plan_list[0]] + dist_dict['W1:FIN']/v_dict['Vnormal']
		elif plan_list[1]=='W2':
			time=dist_W2/v_dict[plan_list[0]] + dist_dict['W2:FIN']/v_dict['Vnormal']
		elif plan_list[1]=='mgoal':
			time=dist_mgoal/v_dict[plan_list[0]]

		return time
			







#Class for representing prognosis. Includes prognosis method.
class Prognosis:
	def __init__(self,flist):
		self.flist=flist
		# order is [[Vnormal,W1],[Vnormal,W2],[Vnormal,mgoal],[Vred,W1],[Vred,W2],[Vred,mgoal],stop]
		self.plan_list=['Vnormal:W1','Vnormal:W2','Vnormal:mgoal','Vred:W1','Vred:W2','Vred:mgoal','stop']
		self.risk_to_goals=[]
		self.pot_faults=[]
		self.RUL_model=RUL_model()

	#Prognosis method. Assumes all potential new faults are exponentially distributed and calculates time until any faults given risk.
	def prognose(self,diag,dist_W1,dist_W2,dist_mgoal):
		self.risk_to_goals=[]
		self.pot_faults=self.RUL_model.possible_new(diag)
		gammasum1=0
		gammasum2=0
		gammaprod1=1
		gammaprod2=1

		for ele in self.pot_faults:
			if not ele.name=='NoFault':
				gammasum1=gammasum1+0.2*int(ele.gamma)
				gammasum2=gammasum2+int(ele.gamma)
				gammaprod1=gammaprod1*0.2*int(ele.gamma)
				gammaprod2=gammaprod2*int(ele.gamma)
		
		#It can be proven that the union of a set of exp distributed variables is equivalent to a exp distribution
		#with the average life time of the sum of all average lifetimes divided by the product of all average lifetimes.
		combined_gamma1= gammasum1/gammaprod1

		combined_gamma2=gammasum2/gammaprod2
		print('dist_w1',dist_W1,'  ','dist_w2',dist_W2,'  ','dist_FIN',dist_mgoal)
		self.risk_to_goals.append(1-np.exp((-dist_W1/v_dict['Vnormal'])*combined_gamma1))
		self.risk_to_goals.append(1-np.exp((-dist_W2/v_dict['Vnormal'])*combined_gamma1))
		self.risk_to_goals.append(1-np.exp((-dist_mgoal/v_dict['Vnormal'])*combined_gamma1))
		self.risk_to_goals.append(1-np.exp((-dist_W1/v_dict['Vred'])*combined_gamma2))
		self.risk_to_goals.append(1-np.exp((-dist_W2/v_dict['Vred'])*combined_gamma2))
		self.risk_to_goals.append(1-np.exp((-dist_mgoal/v_dict['Vred'])*combined_gamma2))
		self.risk_to_goals.append(0)

		#return risk_W1_normal,risk_W1_reduced,risk_W2_normal,risk_W2_reduced,risk_mgoal_normal,risk_mgoal_reduced
		print('risk_to_goals',self.risk_to_goals[0],self.risk_to_goals[1],self.risk_to_goals[2],self.risk_to_goals[3],self.risk_to_goals[4],self.risk_to_goals[5])
#Main statemachine of the node. One method for state switching followed by methods on how to operate in each state.
class StateMachine:

	def __init__(self,state,pub_diag,slist,flist):
		self.state=state

		#Initializing objects for representing symptoms, diagnosis, control tower, plan and prognosis.
		self.symp=Symp()
		self.CTcorr=CTcorr()
		self.Diag=Diag(slist,flist)
		self.prognosis=Prognosis(flist)
		self.plan=Plan(1,3,'None','FIN')

		self.pub_diag=pub_diag
		self.ct_handshake='NO NEW DIAGNSOSIS'
		self.detect_handshake=''
		self.actuate_handshake='NO NEW PLAN'
	


	#Switching method for different node states
	def Stateswitch(self):
		if self.state=='IDLE':
			return self.state_1()
		elif self.state=='DIAGNOSING':
			return self.state_2()
		elif self.state=='W4V': #W2V=wating for verification
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
			

	#Diagnosing
	def state_2(self):
		self.Diag.Diagnose(self.CTcorr,self.symp)
		self.detect_handshake='CONFIRMED NEW SYMP'
		if self.Diag.deltadiag==0:
			self.state='IDLE'
		else:
			self.state='W4V'


	#W4V
	def state_3(self):
		self.detect_handshake='DIAGNOSING SYMPTOMS'
		self.ct_handshake='WAITING FOR CT CONFIRMATION'
		if self.CTcorr.ver==1:
			self.ct_handshake='NEW DIAGNOSIS, CT CONFIRMED'
			self.state='PLANNING'
		
		#If fault is severe, we need to react and cannot wait for CT
		elif Severity_check(self.Diag,self.CTcorr):
			self.ct_handshake='NEW DIAGNOSIS, SEVERE FAULT'
			self.state='PLANNING'


		if self.CTcorr.Request==1:
			self.state='CTCONTROL'

	#PLAN
	def state_4(self):
		self.plan.replan(self.Diag.diag,self.prognosis)
		self.actuate_handshake='NEW PLAN'
		rospy.loginfo(self.actuate_handshake)

		#create and publish diag message
		diag_msg=diag()
		diag_msg.diag=self.Diag.diag
		diag_msg.prob_fault=self.Diag.prob_fault
		diag_msg.deltadiag=self.Diag.deltadiag
		pot_list=[]
		for ele in self.prognosis.pot_faults:
			pot_list.append(str(ele.name) + ':' + str(ele.gamma))
		diag_msg.potfaults=','.join(pot_list)
		self.pub_diag.publish(diag_msg)
		if self.CTcorr.Request==1:
			self.state='CTCONTROL'
		elif self.CTcorr.ver==0:
			self.state='W4V'
		else:
			self.state='IDLE'

	def state_5(self): #CTCONTROL
		if not(self.CTcorr.Arecc =='None'):
			
			self.plan.vl=self.CTcorr.vl
			self.plan.wgoal=self.CTcorr.wgoal
			self.plan.mgoal=self.CTcorr.mgoal
			self.plan.rc=2

			self.actuate_handshake='NEW PLAN'
		if self.CTcorr.Request==0:
			self.state='IDLE'



#Initialization handles use with just python or in a launch file
def param_init():
    # grab parameters from launch-file
    start_pt_param = rospy.search_param('start_pt')
    
    start_pt = rospy.get_param(start_pt_param, default_init_pt)
    if isinstance(start_pt, str):
        start_pt = start_pt.split(',')
        start_pt = [float(curr) for curr in start_pt]
        start_pt = VehicleState(*start_pt)


    return start_pt



def main():
	rospy.init_node('diagnose_plan_node')
	r=rospy.Rate(5)
	start_pt=param_init()


	#Creates fault objects and symptom objects for use in diagnosis
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


	#Declare all publishers
	pub_diag=rospy.Publisher('diag',diag,queue_size=10)
	pub_diag_dec_status=rospy.Publisher('diagnose_decision_status',node_status,queue_size=10)
	pub_act=rospy.Publisher('action',action,queue_size=10)

	SM=StateMachine('IDLE',pub_diag,symps_list,faults_list)

	#Declare all subscribers
	rospy.Subscriber('symptoms', symp, SM.symp.callback_symp )
	rospy.Subscriber('detection_status', node_status, SM.symp.callback_detectstatus )
	rospy.Subscriber('CTcorr', ctcorr, SM.CTcorr.callback )
	rospy.Subscriber('vehicle_map_data',vehicle_map_data,SM.plan.vhd_callback)
	rospy.Subscriber('actuation_status',node_status,SM.plan.actuation_callback)
	
	#main loop of the node
	while not rospy.is_shutdown():

		#create and publish the plan message
		act_msg=action()
		act_msg.vl=SM.plan.vl
		act_msg.wgoal=SM.plan.wgoal
		act_msg.mgoal=SM.plan.mgoal
		act_msg.rc=int(SM.plan.RC)
		pub_act.publish(act_msg)


		#Update general state of health (GSH)
		severity=SM.Diag.diag.split(',')[1]
		if severity=='SAFE':
			SM.Diag.gsh=100
		elif severity=='NOT SEVERE':
			SM.Diag.gsh=50
		elif severity=='SEVERE':
			SM.Diag.gsh=20


		#create and publish the status message
		status_msg=node_status()
		status_msg.status1=SM.state
		status_msg.status2=SM.detect_handshake
		status_msg.status3=SM.actuate_handshake
		status_msg.status4=SM.ct_handshake
		status_msg.status5=SM.Diag.gsh
		status_msg.status6=SM.plan.approxDT
		pub_diag_dec_status.publish(status_msg)
		
		#Verification always set to True until control tower fully integrated with SVEA
		SM.CTcorr.ver=1

		SM.Stateswitch()
		r.sleep()
		


if __name__ == '__main__':
	main()