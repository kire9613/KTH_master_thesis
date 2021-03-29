#! /usr/bin/env python

#decision node


import numpy
import math
import rospy
from av09_msgs.msg import *
from std_msgs.msg import *





class Diag:
    def __init__(self):
        self.diag='nofault,SAFE'
        self.deltadiag=0
        self.diag_status=None
    def callback_diag(self,msg):
        self.diag=msg.diag
        self.deltadiag=msg.deltadiag

    def callback_diagstatus(self,msg):
        self.diag_status=msg.status3




class CTcorr:
    def __init__(self):
        self.ver=0
        self.Fcorr=0
        self.Arecc='NO ACTION'
        self.Request=0
        self.rc=0

    def callback(self, msg):
        self.ver=msg.ctver
        self.Fcorr=msg.fault
        self.Arecc=msg.recact
        self.Request=msg.ctreq
        self.rc=msg.rc


class Plan:
    def __init__(self,RC,action):
        self.RC=RC
        self.action=action
        

    def replan(self,Diag,ctrc,ctarec):
        diag=Diag.split(',')
        old_action=self.action
        old_rc=self.RC
        if diag[1]=='SEVERE':
            self.action='EMERGENCY BREAK,CONTACT TOW'
            self.RC=2
        elif diag[1]=='NOT SEVERE':
            self.action='SLOW DOWN,DRIVE TO WORKSHOP'
            self.RC=3
        elif diag[1]=='SAFE':
            self.action='NO ACTION'
            self.RC=1
        

        

class StateMachine:
    def __init__(self,state,pub_act,pub_dec_status):
        self.state=state
        self.diag=Diag()
        self.CTcorr=CTcorr()
        self.plan=Plan(1,self.CTcorr.Arecc)
        self.pub_act=pub_act
        self.diag_handshake='WAITING FOR DIAGNOSIS'
        
    def Stateswitch(self):
        if self.state=='IDLE':
            return self.state_1()
        elif self.state=='PLANNING':
            return self.state_2()
        elif self.state=='PUBLISHING': 
            return self.state_3()
        elif self.state=='CTCONTROL':
            return self.state_4()		
    #IDLE
    def state_1(self):
        #print('state IDLE')
        if self.diag.diag_status=='NEW DIAGNOSIS, SEVERE FAULT' or self.diag.diag_status=='NEW DIAGNOSIS, CT CONFIRMED':
            self.diag_handshake='DIAGNOSIS RECEIVED AND PLANNING'
            self.state='PLANNING' 
        else:
            self.diag_handshake='WAITING FOR GOAHEAD' 

    def state_2(self):
        self.plan.replan(self.diag.diag,self.CTcorr.rc,self.CTcorr.Arecc)
        #print('state PLANNING')
        #if self.diag.diag_status=='NEW DIAGNOSIS, SEVERE FAULT' or self.diag.diag_status=='NEW DIAGNOSIS, CT CONFIRMED':
         #   self.state='PLANNING'
        if self.CTcorr.Request==1:
            self.state='CTCONTROL'

        else:
            self.state='PUBLISHING'


    def state_3(self):
        #print('state PUBLISHING')
        act_msg=action()
        act_msg.action=self.plan.action
        act_msg.rc=int(self.plan.RC)
        #rospy.loginfo(act_msg)
        self.pub_act.publish(act_msg)
        self.state='IDLE'


    def state_4(self):
        print('state CTCONTROL')
        if not(self.CTcorr.Arecc =='None'):
            act_msg=action()
            act_msg.action=self.CTcorr.Arecc
            act_msg.rc=self.CTcorr.rc
            #rospy.loginfo(act_msg)
            self.pub_act.publish(act_msg)
        if self.CTcorr.ctreq==0:
            self.state='IDLE'
		

def main():
    rospy.init_node('decision_node')
    r=rospy.Rate(4) 

    pub_act=rospy.Publisher('action',action,queue_size=10)
    pub_dec_status=rospy.Publisher('decision_status',node_status,queue_size=10)
    SM=StateMachine('IDLE',pub_act,pub_dec_status)  
    rospy.Subscriber('diag', diag, SM.diag.callback_diag )
    rospy.Subscriber('diagnose_status', node_status, SM.diag.callback_diagstatus )
    rospy.Subscriber('ctcorr', ctcorr, SM.CTcorr.callback )
    while not rospy.is_shutdown():

        status_msg=node_status()
        status_msg.status1=SM.state
        status_msg.status2=SM.diag_handshake
        pub_dec_status.publish(status_msg)
        SM.Stateswitch()
        r.sleep()

if __name__ == '__main__':
    main()