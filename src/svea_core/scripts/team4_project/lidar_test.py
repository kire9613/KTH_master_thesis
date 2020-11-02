import rospy
from sensor_msgs.msg import LaserScan

def lidar(scan):
    print(min(scan.ranges))

rospy.init_node('lidar_test')

rospy.Subscriber('/scan', LaserScan, lidar)

rospy.spin()
