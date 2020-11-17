import rospy
import numpy as np
from copy import deepcopy
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
import matplotlib.pyplot as plt
import thread

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
        self.resolution = map.info.resolution
        self.origin_x = map.info.origin.position.x
        self.origin_y = map.info.origin.position.y
        self.map_info = [self.width, self.height, self.resolution, self.origin_x, self.origin_y]
        self.gridmap = np.array(map.data).reshape(self.height,self.width)
        self.infl_gridmap = np.array(map.data).reshape(self.height,self.width)

        thread.start_new_thread( self.subscriber, ("map_updates", OccupancyGridUpdate, self.update_map))
        thread.start_new_thread( self.subscriber, ("infl_map_updates", OccupancyGridUpdate, self.update_infl_map))

        #self.__update_map_sub = rospy.Subscriber("map_updates", OccupancyGridUpdate, self.update_map)
        #self.__update_infl_map_sub = rospy.Subscriber("infl_map_updates", OccupancyGridUpdate, self.update_infl_map)
        rospy.loginfo("Map updater running!")

        self.occupied_space = 1
        self.polygon_space = 120
        self.c_space = 90
        self.radius = 2

    def get_map_info(self):
        return self.map_info

    def subscriber(self, topic, type, callback):
        rospy.Subscriber(topic, type, callback)

    def update_infl_map(self, iupdate):
        print("upd infl map")
        min_x = iupdate.x
        min_y = iupdate.y
        max_x = min_x + iupdate.width - 1
        max_y = min_y + iupdate.height - 1
        self.infl_gridmap[min_y:(max_y+1), min_x:(max_x+1)] = np.array(iupdate.data).reshape(iupdate.height,iupdate.width)

    def update_map(self, update):
        print("upd map")
        min_x = update.x
        min_y = update.y
        max_x = min_x + update.width - 1
        max_y = min_y + update.height - 1
        self.gridmap[min_y:(max_y+1), min_x:(max_x+1)] = np.array(update.data).reshape(update.height,update.width)

    def get_map(self):
        return deepcopy(self.gridmap)

    def get_map_update(self):
        upd = rospy.wait_for_message("map_updates", OccupancyGridUpdate)
        return upd

    def get_infl_map_update(self):
        iupd = rospy.wait_for_message("infl_map_updates", OccupancyGridUpdate)
        return iupd

    def add_to_map(self, imap, x, y, value):
        """Adds value to index (x, y) in grid_map if index is in bounds.
        Returns weather (x, y) is inside grid_map or not.
        """

        if self.is_in_bounds(imap, x, y):
            if not imap[y,x] == self.polygon_space:
                imap[y,x] = value
                return True
        return False

    def is_in_bounds(self, imap, x, y):
        """Returns weather (x, y) is inside grid_map or not."""

        if x >= 0 and x < self.width:
            if y >= 0 and y < self.height:
                return True
        return False

    def get_inflated_map(self):
        """ Inflate only update and add to inflated global map """
        return deepcopy(self.infl_gridmap)

def main():
    map_updater = UpdateMap()
    #while True:
    while True:
        print("plot map")
        gm = map_updater.get_map()
        gmi = map_updater.get_inflated_map()
        #upd = map_updater.get_map_update()
        #plt.imshow(np.array(upd.data).reshape(upd.height,upd.width))
        plt.imshow(gm)
        plt.show()
        plt.imshow(gmi)
        plt.show()

if __name__ == '__main__':
    print(__file__ + " start!!")
    main()
    print(__file__ + " Done!!")
