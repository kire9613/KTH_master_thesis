import roslib
import rospy


if __name__ == '__main__':
    s = rospy.Service('get_map', GetMap, mapper.get_map)
    print("Map_service_provider node is working")
    rospy.spin()
 
