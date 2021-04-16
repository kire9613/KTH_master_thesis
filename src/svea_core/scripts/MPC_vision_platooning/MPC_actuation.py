#!/usr/bin/env python
import rospy
from svea.actuation import ActuationInterface
from std_msgs.msg import Float64


rospy.init_node('MPC_actuation')
accel = 0.

def accelinfo(data):
    global accel
    accel = data.data  

def main():
    rospy.sleep(1)
    dt = 0.1
    rate = rospy.Rate(10) #Hz
    ctrl_interface = ActuationInterface().start(wait = True)
    rospy.Subscriber('/acceleration_publisher', Float64, accelinfo, queue_size=1)
    
    
    target_velocity = 0.

    while not rospy.is_shutdown():
        ctrl_interface.send_control(
                steering=0,
                velocity=target_velocity,
                transmission=0)
        #rospy.loginfo_throttle(dt, ctrl_interface)
        target_velocity += accel*dt
        print(accel)

        rate.sleep()



if __name__ == '__main__':
    main()


