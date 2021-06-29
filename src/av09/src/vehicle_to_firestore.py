#! /usr/bin/env python

import firebase_admin
from firebase_admin import credentials
from firebase_admin import firestore
import rospy
from faultclass import Fault_obj
from symptomclass import Symptom_obj
from nav_msgs.msg import *
from std_msgs.msg import *
from av09_msgs.msg import *
import os
import datetime as dt




nodes_to_towns={'start':'Sodertalje','N1':'Nykoping','N2':'Linkoping','FIN':'Jonkoping','W1':'Skavsta workshop','W2':'Tannefors'}

class Topics_to_firestore:

    #Set up firestore database and system variables
    def __init__(self, sympslist):
        THIS_FOLDER = os.path.dirname(os.path.abspath(__file__))
        cred=credentials.Certificate(os.path.join(THIS_FOLDER,"serviceAccountKey.json"))

        firebase_admin.initialize_app(cred)
        
        self.database=firestore.client()

        self.sympslist=sympslist
        #variable to signal a diagnosis has been made
        self.diagevent=False

        #plan, position and time variables
        self.oc=1
        self.pos_x=0
        self.pos_y=0
        self.pos_y_history=[]
        self.pos_x_history=[]
        self.zero_time=rospy.Time.now()
        self.is_finished=0
        self.rc=3
        self.vr=1
        self.vl=1
        
        

        #diagnosis, symptom, prognosis variables
        self.symp=[]
        self.P_faults=[]
        self.diag='nofault,SAFE'
        self.prob_fault=1
        self.potfaults=[]
        self.diag_det_status='WAITING FOR NEW SYMPS'
        self.ct_diag_status='NOTHING TO VERIFY'
        self.gsh=100
        self.approxDT=0
        
        #plan and actuation variables
        self.action_history=[['NO ACTION',0.000]]
        self.plan_status='WAITING FOR NEW DIAG'
        self.actuation_status='NORMAL OPERATION'
        self.distance_FIN=100
        self.distance_W1=100
        self.distance_W2=100
        self.driven_dist=0
        self.last_node='start'
        self.next_node='N1'
        self.mission_finished=False

        #control tower variables
        self.ctver=0
        self.CT_actuation_handshake='WAITING FOR CT RELEASE'
        self.CT_release_from_stand_down=0
        self.correct_fault=['NO FAULT','SAFE']
        self.request_control=0

    #set diagnosis and prognosis variables from ROS message
    def diag_callback(self,msg):
        old_diag=self.diag
        self.diag=msg.diag
        self.potfaults=msg.potfaults.split(',')
        self.prob_fault=msg.prob_fault
        if not old_diag == self.diag:
            self.diagevent=True



    #set diagnosis/prognosis/plan handshake variables from ROS message
    def diag_dec_status_callback(self,msg):
        old_status2=self.diag_det_status
        old_status4=self.ct_diag_status
        self.diag_det_status=msg.status2
        self.ct_diag_status=msg.status4
        self.gsh=msg.status5
        self.approxDT=msg.status6


    #set plan variables from ROS messages
    def action_callback(self,msg):
        self.rc=msg.rc
        self.vl=msg.vl
        self.mgoal=msg.mgoal
        self.wgoal=msg.wgoal
        
    #set symptom variable from ROS message
    def symptoms_callback(self,msg):
        old_symp=self.symp
        self.symp=msg.symp

    #set distance to goals, next node in route and last node in route.
    def vmd_callback(self,msg):
        self.distance_W1=msg.dist_w1
        self.distance_W2=msg.dist_w2
        self.distance_FIN=msg.dist_mgoal
        self.last_node=msg.lastN
        self.next_node=msg.nextN

    def actuation_status_callback(self,msg):
        self.actuation_status=msg.status2
        self.oc=msg.status1
        self.is_finished=msg.status3
        self.vl=msg.status4
        self.vr=msg.status5
        self.driven_dist=msg.status6

    #set current position from ROS message
    def path_callback(self,msg):
        try:

            self.pos_x=msg.poses[-1].pose.position.x
            self.pos_y=msg.poses[-1].pose.position.y
        except:
            print('could not find position')

    #Write to firestore. Structure of database can be found on ....
    def update_firestore_write(self):

        self.pos_x_history.append(self.pos_x)
        self.pos_y_history.append(self.pos_y)
        
        
        #diag and prog update
        prognoses=[]
        for progfault in self.potfaults:
            prog={'AvgLife':unicode(progfault.split(':')[1]),'prognosis':unicode(progfault.split(':')[0])}
            prognoses.append(prog)
        dia=[]
        dia.append({'diagnose':unicode(self.diag),'likelyhood':self.prob_fault,'prognoses':prognoses})
        
        diag_prog_data={'dia':dia}
        self.database.collection(u'VehiclesTest').document(u'Vehicle1').collection(u'diagnosis').document(u'diagnoses').set(diag_prog_data,merge=True)


        #update general state of health
        active_symps={}
        for i,ele in enumerate(self.sympslist):
            
            #As there are 4 wheels, symptoms for each wheel need to be written. We simply say that wheel 2 has either vibrations or hot nave.
            if int(self.symp.split(',')[i])==1:
                if ele.name=='hotnave' or ele.name=='vibration':
                    active_symps[ele.name]=[False,True,False,False]
                else:
                    active_symps[ele.name]=True
            else:
                if ele.name=='hotnave' or ele.name=='vibration':
                    active_symps[ele.name]=[False,False,False,False]
                else:
                    active_symps[ele.name]=False
        #print(active_symps)
        self.database.collection(u'VehiclesTest').document(u'Vehicle1').collection(u'vehicleinfo').document(u'vehicleHealth').set({'gsh':int(self.gsh),'symptoms':active_symps},merge=True)
        


        #create log event
        if self.diagevent:
            datetime = dt.datetime.now()
            content=unicode('NEW DIAGNOSIS: ' + str(self.diag))
            log_ref=self.database.collection(u'VehiclesTest').document(u'Vehicle1').collection(u'vehicleinfo').document(u'VehicleLogs')
            type_=unicode('VEHICLE DIAGNOSIS EVENT')
            log_ref.update({u'Logs': firestore.ArrayUnion([{'content':content,'occ_time':unicode(str(datetime)),'type':type_}])})
            
            self.diagevent=False


        #route info and downtime
        info_ref=self.database.collection('VehiclesTest').document('Vehicle1').collection('vehicleinfo').document('vehicleInfoCurrent')
        route=[unicode('Sodertelje'),unicode('Jonkoping')]
        if self.next_node=='FIN' and self.is_finished:
            self.mission_finished=True
        else:
            self.mission_finished=False
        info_ref.set({'distance':self.distance_FIN,'distanceDriven':float(self.driven_dist),'route':route,'totalDowntimeApprox': float(self.approxDT),'speed':float(self.vr),'localroute':[unicode(nodes_to_towns[self.last_node]),unicode(nodes_to_towns[self.next_node])],'mission_finished':self.mission_finished},merge=True)

        
        
    #Read from firestore. Control tower variables are collected
    def CT_update_read(self):
        CT_data=self.database.collection('VehiclesTest').document('Vehicle1').collection('commands').document('command').get().to_dict()
        #self.CT_release_from_stand_down=CT_data['CT_release_from_stand_down']
        self.request_control=CT_data['takeControl']
        
        self.CTactSpeed=CT_data['action']['speed']
        self.CTactRoute=CT_data['action']['route']
        self.ctver=CT_data['verifyDiagnose']
def main():

    rospy.init_node('vehicle_to_firestore')



    THIS_FOLDER = os.path.dirname(os.path.abspath(__file__))
    my_file1 = os.path.join(THIS_FOLDER, 'symps.txt')
    my_file2 = os.path.join(THIS_FOLDER, 'faults.txt')

    sympdoc=open(my_file1,mode='r')
    list2=sympdoc.readlines()
    faultdoc=open(my_file2,mode='r')
    list1=faultdoc.readlines()
    faultssize=len(list1)

    symps_list=[]
    for i in list2:
        i="".join(i.split())
        objlist2=i.split(',')
        symps_list.append(Symptom_obj(objlist2,faultssize))

    channel=Topics_to_firestore(symps_list)
    #Frequency of main loop
    r=rospy.Rate(1)

    #Declaration of subscribers
    rospy.Subscriber('diag',diag,channel.diag_callback)
    rospy.Subscriber('diagnose_decision_status',node_status,channel.diag_dec_status_callback)
    rospy.Subscriber('action',action,channel.action_callback)
    rospy.Subscriber('symptoms',symp,channel.symptoms_callback)
    rospy.Subscriber('past_path',Path,channel.path_callback)
    rospy.Subscriber('vehicle_map_data',vehicle_map_data, channel.vmd_callback)
    rospy.Subscriber('actuation_status',node_status,channel.actuation_status_callback)
    ct_pub=rospy.Publisher('ctcorr',ctcorr,queue_size=10)

    #Main loop
    while not rospy.is_shutdown():
        channel.update_firestore_write()
        channel.CT_update_read()
        ct_msg=ctcorr()
        ct_msg.ctreq=channel.request_control
        ct_msg.ctver=channel.ctver
        
        ct_msg.release=channel.CT_release_from_stand_down
        ct_pub.publish(ct_msg)
        r.sleep()






if __name__ == '__main__':
	main()