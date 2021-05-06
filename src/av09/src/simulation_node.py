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

#Returns True with probability set by input. Returns False by probability 1-input.
def decision(probability):
    if random.random() < probability:
        return 1
    return 0

#Class for representing GUI and all functionalities within GUI, including communication with ROS.
class GUI:
    def __init__(self,root,pub_start):
        self.root=root
        self.pub_start=pub_start
        self.symp_list,self.fault_list=read_create()
        self.scenario=scenario

    #when clicked, publish True to \start
    def click_start(self):
        self.pub_start.publish(1)

    #When repaired at workshop, set all symptoms to zero and publish to \rawsignals
    def repaired(self):
        global scenario
        scenario_local=scenario.split(',')
        
        i=0
        while i<16:
            scenario_local[i]=str(0)
            i=i+1

        scenario=','.join(scenario_local)
        
    #when clicked, generate symptoms corresponding to fault chosen
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
        
    #set handshake variables and status variables from ROS message
    def diagnose_decision_status_callback(self,msg):
        self.diag_dec_state=msg.status1
        self.diag_detect_handshake=msg.status2
        self.dec_act_handshake=msg.status3
        self.diag_ct_handshake=status4


    #set actuation status variables from ROS message
    def actuation_callback(self,msg):
        self.operational_state=msg.status1
        self.actuation_status=msg.status2
        if self.actuation_status=='AT WORKSHOP':
            self.repaired()




#Thread for generating and maintaining GUI. Communication with Main is done with global variable 'Scenario'.
def thread_function():
    
        global scenario
        
        pub_start=rospy.Publisher('start_system',Int8,queue_size=10)

       
        root=Tk()
        gui=GUI(root,pub_start)
        
        rospy.Subscriber('actuation_status',node_status,gui.actuation_callback)


        #Create all buttons for GUI
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
        faulti_button=Button(root,text=gui.fault_list[10].name,padx=50,pady=25, command=lambda: gui.click_fault(10))
        faulti_button.pack()
        faulti_button=Button(root,text=gui.fault_list[11].name,padx=50,pady=25, command=lambda: gui.click_fault(11))
        faulti_button.pack()
        root.mainloop()

 
        
#Read from file and create fault and symptoms objects. Returns list of faults and symptoms
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
    r=rospy.Rate(4)

    #create and initialize thread for GUI
    x = threading.Thread(target=thread_function)
    
    x.start()
    
    pub_signals=rospy.Publisher('rawsignals',signals,queue_size=5)

    #main loop of node.
    while not rospy.is_shutdown():
        
        
        #create and publish signal message
        signal_msg=signals()
        signal_msg.data=scenario
        pub_signals.publish(signal_msg)
        r.sleep()
    
if __name__ == '__main__':
	main()