#! /usr/bin/env python

import firebase_admin
from firebase_admin import credentials
from firebase_admin import firestore
import rospy
from nav_msgs.msg import *
from std_msgs.msg import *
from av09_msgs.msg import *
import os






class Topics_to_firestore:
    def __init__(self):
        THIS_FOLDER = os.path.dirname(os.path.abspath(__file__))
        cred=credentials.Certificate(os.path.join(THIS_FOLDER,"serviceAccountKey.json"))

        firebase_admin.initialize_app(cred)
        
        self.database=firestore.client()




        self.oc=1
        self.route='MAIN ROUTE'
        self.pos_x=0
        self.pos_y=0
        self.pos_y_history=[]
        self.pos_x_history=[]
        self.zero_time=rospy.Time.now()
        self.is_finished=0
        self.rc=3
        self.vr=1
        self.v=0
        self.actuation_status='NORMAL OPERATION'


        self.symp=[]
        self.P_faults=[]
        self.diag=['NO fAULT','SAFE']
        self.diag_det_status='WAITING FOR NEW SYMPS'
        self.ct_diag_status='NOTHING TO VERIFY'
        

        self.plan_current='NO ACTION'
        self.action_history=[['NO ACTION',0.000]]
        self.plan_status='WAITING FOR NEW DIAG'
        

        self.ctver=0
        self.CT_actuation_handshake='WAITING FOR CT RELEASE'
        self.CT_release_from_stand_down=0
        self.correct_fault=['NO FAULT','SAFE']
        self.request_control=0


    def diag_callback(self,msg):
        old_diag=self.diag
        self.diag=msg.diag




    def diag_dec_status_callback(self,msg):
        old_status2=self.diag_det_status
        old_status4=self.ct_diag_status
        self.diag_det_status=msg.status2
        self.ct_diag_status=msg.status4


    def action_callback(self,msg):
        self.rc=msg.rc
        old_action=self.action_history[-1][0]
        if not old_action==msg.action:
            self.action_history.append([msg.action,rospy.Time.now()-self.zero_time])
        

    def symptoms_callback(self,msg):
        old_symp=self.symp
        self.symp=msg.symp
        

    def actuation_status_callback(self,msg):
        old_status=self.actuation_status
        self.actuation_status=msg.status2
        self.oc=msg.status1
        self.is_finished=msg.status3
        self.vr=msg.status4
        self.v=msg.status5
        self.route=msg.status6


    def path_callback(self,msg):
        try:

            self.pos_x=msg.poses[-1].pose.position.x
            self.pos_y=msg.poses[-1].pose.position.y
            #self.pos_x_history.append(self.pos_x)
            #self.pos_y_history.append(self.pos_y)
            #print(self.pos_x,self.pos_y)
        except:
            print('could not find position')

    def update_firestore_write(self):

        self.pos_x_history.append(self.pos_x)
        self.pos_y_history.append(self.pos_y)
        print(self.pos_x_history)
        diag_data={u'DIAG':self.diag,u'Diag_status':self.diag_det_status,u'P_faults':0,u'warnings_symptoms':self.symp}
        act_data={u'is_finished':self.is_finished,u'oc':self.oc,u'pos_x_history':self.pos_x_history,u'pos_y_history':self.pos_y_history,u'rc':self.rc,u'route':self.route,u'v':self.v,u'vr':self.vr}
        plan_data={u'plan_current':str(self.plan_current),u'plan_status':self.plan_status}
        self.database.collection(u'Vehicle').document(u'Act_data').set(act_data,merge=True)
        self.database.collection(u'Vehicle').document(u'Diagnostic_data').set(diag_data,merge=True)
        self.database.collection(u'Vehicle').document(u'plan_data').set(plan_data,merge=True)

    def CT_update_read(self):
        CT_data=self.database.collection('Vehicle').document('CT_data').get().to_dict()
        self.CT_release_from_stand_down=CT_data['CT_release_from_stand_down']
        self.request_control=CT_data['CT_request_control']
        self.correct_fault=CT_data['corr_fault']
        self.ctver=CT_data['ctver']
def main():

    rospy.init_node('vehicle_to_firestore')

    channel=Topics_to_firestore()

    r=rospy.Rate(1)
    rospy.Subscriber('diag',diag,channel.diag_callback)
    rospy.Subscriber('diagnose_decision_status',node_status,channel.diag_dec_status_callback)
    rospy.Subscriber('action',action,channel.action_callback)
    rospy.Subscriber('symptoms',symp,channel.symptoms_callback)
    rospy.Subscriber('past_path',Path,channel.path_callback)
    ct_pub=rospy.Publisher('ctcorr',ctcorr,queue_size=10)
    while not rospy.is_shutdown():
        channel.update_firestore_write()
        ct_msg=ctcorr()
        ct_msg.ctreq=channel.request_control
        ct_msg.ctver=channel.ctver
        ct_msg.fault=channel.correct_fault
        ct_msg.release=channel.CT_release_from_stand_down
        ct_pub.publish(ct_msg)
        r.sleep()

    #rospy.spin()





if __name__ == '__main__':
	main()