#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool

rospy.init_node('start_pause')

set_pause = False
pause_pub = rospy.Publisher('/pause', Bool, queue_size=1, latch=True)
rospy.sleep(1)

while not rospy.is_shutdown():
    if set_pause:
        raw_input("Press enter to pause")
    else:
        raw_input("Press enter to unpause")

    pause_pub.publish(set_pause)
    set_pause = not set_pause
