import rospy
import numpy as np
from copy import deepcopy
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
import matplotlib.pyplot as plt

class UpdateMap:
    """
    Help class for adding local gridmap updates to global map
    """

    def __init__(self):

        rospy.loginfo("Init UpdateMap")
        rospy.init_node('update_map')

        map = rospy.wait_for_message('/map', OccupancyGrid)
        self.height = map.info.height
        self.width = map.info.width
        self.gridmap = np.array(map.data).reshape(self.height,self.width)
        self.__update_map_sub = rospy.Subscriber("map_updates", OccupancyGridUpdate, self.update_map)
        rospy.loginfo("Map updater running!")

    def update_map(self, update):
        min_x = update.x
        min_y = update.y
        max_x = min_x + update.width - 1
        max_y = min_y + update.height - 1
        self.gridmap[min_y:(max_y+1), min_x:(max_x+1)] = np.array(update.data).reshape(update.height,update.width)

    def get_map(self):
        return deepcopy(self.gridmap)

def main():
    map_updater = UpdateMap()
    while True:
        print("plot map")
        gm = map_updater.get_map()
        plt.imshow(gm)
        plt.show()

if __name__ == '__main__':
    print(__file__ + " start!!")
    main()
    print(__file__ + " Done!!")
