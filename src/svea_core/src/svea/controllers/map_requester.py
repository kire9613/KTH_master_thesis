    
import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
    
    
class MapRequester():
    def __init__(self):    
        rospy.wait_for_service('get_map')
        self.latest_map = OccupancyGrid()


    def update(self):
        #print("Attempting request")
        try:
            map_client = rospy.ServiceProxy('get_map', GetMap)
            self.latest_map = map_client()
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        #print("Attempt done")
    
    def getMap(self):
        return self.latest_map

    def test(self):
        self.update()
        #print("The test was succesful")
        return self.latest_map