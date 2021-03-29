#! /usr/bin/env python

from std_msgs.msg import *
import math
from av09_msgs.msg import *
import rospy
import threading
import time
from faultclass import *
from symptomclass import *
from Tkinter import *
import os
import random
import Tkinter
import tkMessageBox

global scenario

scenario='0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0'


def decision(probability):
    if random.random() < probability:
        return 1
    return 0


class CTcorr:
	def __init__(self):
		self.ver=0
		self.Fcorr=0
		self.Arecc=0
		self.Request=0
		self.rc=2



class GUI:
    def __init__(self,root,pub_start,pub_ct):
        self.root=root
        self.pub_start=pub_start
        self.pub_ct=pub_ct
        self.symp_list,self.fault_list=read_create()
        self.scenario=scenario
        self.ctcorr=CTcorr()



    def release_click(self):
        self.ctcorr.rc=0
        ct_msg=ctcorr()
        ct_msg.ctver=self.ctcorr.ver
        self.pub_ct.publish(ct_msg)
        

    def ver_click(self):
        self.ctcorr.ver=1
        ct_msg=ctcorr()
        ct_msg.ctver=self.ctcorr.ver
        self.pub_ct.publish(ct_msg)
        #print(self.ctcorr.ver)

    def click_start(self):
        self.pub_start.publish(1)

    def click_fault(self,fault_num):
        scenario_local=self.scenario.split(',')
        
        fault=self.fault_list[fault_num]
        # Initialize size and simulate outcome
        i=0
        for prob in fault.P_symps:
            scenario_local[i]=str(decision(prob))
            i=i+1
        global scenario
        self.scenario=','.join(scenario_local)
        scenario=','.join(scenario_local)

    def diag_callback(self,msg):
        self.diag=msg.diag

    def diagnose_decision_status_callback(self,msg):
        self.diag_dec_state=msg.status1
        self.diag_detect_handshake=msg.status2
        self.dec_act_handshake=msg.status3
        self.diag_ct_handshake=status4

        if self.diag_dec_state=='IDLE':
            self.ctcorr.ver=0
            self.ctcorr.rc=2

    def symp_callback(self,msg):
        self.symptoms=msg.symp

    def action_callback(self,msg):
        self.action=msg.action
        self.rc=msg.rc

    def actuation_callback(self,msg):
        self.operational_state=msg.status1
        self.actuation_status=msg.status2





def thread_function():
    
        global scenario
        
        pub_start=rospy.Publisher('start_system',Int8,queue_size=10) 
        pub_ct=rospy.Publisher('CTcorr',ctcorr,queue_size=10)

       
        root=Tk()
        gui=GUI(root,pub_start,pub_ct)
        rospy.Subscriber('diag',diag,gui.diag_callback)
        rospy.Subscriber('symptoms',symp,gui.symp_callback)
        rospy.Subscriber('action',action,gui.action_callback)

        start_button=Button(root,text='START SIMULATION',padx=50,pady=50, command=gui.click_start)
        start_button.pack()

        faulti_button=Button(root,text=gui.fault_list[0].name,padx=50,pady=25, command=lambda: gui.click_fault(0))
        faulti_button.pack()
        faulti_button=Button(root,text=gui.fault_list[1].name,padx=50,pady=25, command=lambda: gui.click_fault(1))
        faulti_button.pack()
        faulti_button=Button(root,text=gui.fault_list[2].name,padx=50,pady=25, command=lambda: gui.click_fault(2))
        faulti_button.pack()
        faulti_button=Button(root,text=gui.fault_list[3].name,padx=50,pady=25, command=lambda: gui.click_fault(3))
        faulti_button.pack()
        faulti_button=Button(root,text=gui.fault_list[4].name,padx=50,pady=25, command=lambda: gui.click_fault(4))
        faulti_button.pack()
        faulti_button=Button(root,text=gui.fault_list[5].name,padx=50,pady=25, command=lambda: gui.click_fault(5))
        faulti_button.pack()
        faulti_button=Button(root,text=gui.fault_list[6].name,padx=50,pady=25, command=lambda: gui.click_fault(6))
        faulti_button.pack()
        faulti_button=Button(root,text=gui.fault_list[7].name,padx=50,pady=25, command=lambda: gui.click_fault(7))
        faulti_button.pack()
        faulti_button=Button(root,text=gui.fault_list[8].name,padx=50,pady=25, command=lambda: gui.click_fault(8))
        faulti_button.pack()
        faulti_button=Button(root,text=gui.fault_list[9].name,padx=50,pady=25, command=lambda: gui.click_fault(9))
        faulti_button.pack()
        root.mainloop()

"""         faulti_button=Button(root,text='Verify Diag',padx=50,pady=25, command=gui.ver_click)
        faulti_button.pack()
        faulti_button=Button(root,text='Release from stand down',padx=50,pady=25, command= gui.release_click)
        faulti_button.pack() """

 
        

def read_create():
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

    return symps_list,faults_list



def main():
    rospy.init_node('simulation_node')
    global scenario
    newsignal =3
    r=rospy.Rate(4)
    x = threading.Thread(target=thread_function)
    x.start()
    while not rospy.is_shutdown():
        
        
        pub_signals=rospy.Publisher('rawsignals',signals,queue_size=5)
        signal_msg=signals()
        
        signal_msg.data=scenario
        pub_signals.publish(signal_msg)
        r.sleep()
    
if __name__ == '__main__':
	main()