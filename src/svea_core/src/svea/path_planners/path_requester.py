import rospy
from svea_msgs.srv import PathEstimator

class PathRequester():
    def __init__(self):    
        rospy.wait_for_service('calculate_path')

    def estimate_path(self, start, target, grid, width, height):
        try:
            path_client = rospy.ServiceProxy('calculate_path', PathEstimator)
            path = path_client(start, target, grid, width, height)
            #print("Path estimated")
            return path
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            return None
    
   
