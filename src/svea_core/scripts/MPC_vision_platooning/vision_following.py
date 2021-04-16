#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float64
from visMPC import MPC_controller
from svea.actuation import ActuationInterface

#from svea.states import *
#from svea.svea_managers.path_following_sveas import SVEAPlatoonMember

#from svea.sensors import IMU
#from standard_msgs.msg import FLoat64
#<node pkg="svea_core" type="vision_following2.py" name="vision_following" output="screen"/>

rospy.init_node('vision_following')
imudata = Imu()
markerpitchdata = 0.0
markerdist = 0.0
accel_pub = rospy.Publisher('/acceleration_publisher', Float64, queue_size=1)

def imuinfo(data):
    global imudata
    imudata = data

def markerpitchinfo(data):
    global markerpitchdata
    markerpitchdata = data.data  

def markerdistinfo(data):
    global markerdist
    markerdist = data.pose.position.z

def main():
    rospy.sleep(1)
    dt = 0.1
    rate = rospy.Rate(10) #Hz

    follower_v = 0.0  

    rospy.Subscriber('/imu/data', Imu, imuinfo, queue_size=1)
    rospy.Subscriber('/observed_pitch', Float64, markerpitchinfo, queue_size=1)
    rospy.Subscriber('/observed_pose', PoseStamped, markerdistinfo, queue_size=1)
    #self.controller.target_velocity += accel * dt
    pitch_trajectory = Pitch_traj()
    leader = Leader_vehicle()
    MPC = MPC_controller()
    target_velocity = 0.
    #MPC.update_pitch_traj([5]*20)
    #MPC.simulate()
    while not rospy.is_shutdown():
        tic = rospy.get_time()
        alpha = get_imu_pitch()
        leader.update(alpha, markerpitchdata)
        pitch_trajectory.add_new_pitch(leader.alpha)
        #print(pitch_trajectory)
        
        #MPC.update_pitch_traj(pitch_trajectory.alphalist)
        MPC.update_pitch_traj([0]*6)
        x,u = MPC.solve()
        toc = rospy.get_time()
        #rospy.loginfo_throttle(0.5, toc-tic) 
        accel = u[0]-u[1]
        #print(accel)
        #print(str(x),str(f))
        vrel = x[0]

        accel_pub.publish(Float64(accel))
        rospy.loginfo_throttle(0.1, accel)

        #print('alpha ='+ str(alpha))
        #print('theta =' + str(markerpitchdata))
        #print('leader alpha =' + str(leader.alpha))
        #print('leader distance =' + str(markerdist))
        #print('leader speed =' + str(leader.v))

        rate.sleep()

def get_imu_pitch():

    quat = (imudata.orientation.x,
        imudata.orientation.y,
        imudata.orientation.z,
        imudata.orientation.w)

    _, imu_pitch, _ = euler_from_quaternion(quat)

    #roll = imu_roll*180/math.pi
    pitch = imu_pitch*180/math.pi
    #yaw = imu_yaw*180/math.pi
    return pitch

class Leader_vehicle(object):
    def __init__(self,alpha=0.0,theta=0.0):
        self.alpha = alpha+theta
        #self.v = 0.0
        #self.d = d

    def update(self,alpha,theta):
        self.alpha = alpha+theta
        #self.v = (d - self.d)*10 + follower_v
        #self.d = d

class Pitch_traj(object):	
    def __init__(self):
        self.alphalist = []
        #self.timelist = [] #change time to position

    def __str__(self):
        return str(self.alphalist)

    def add_new_pitch(self,pitch):
        self.alphalist.append(pitch)
        if len(self.alphalist) >= 6:
            self.alphalist.pop(0)

#self.timelist.append(time)

#def remove_first_pitch(self,pitch,time):
#if s[0] < s
	#remove s[0] (pop?)




if __name__ == '__main__':
    main()
