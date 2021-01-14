from nav_msgs.msg import OccupancyGrid
import numpy as np

class Grid:    
    def __init__(self, occupancyGrid = OccupancyGrid()):        
        self.height = occupancyGrid.info.height
        self.width = occupancyGrid.info.width
        self.grid = np.flip(np.reshape(occupancyGrid.data, (self.height, self.width)), axis = 0)
        self.origin = np.array([occupancyGrid.info.origin.position.x, occupancyGrid.info.origin.position.y, occupancyGrid.info.origin.position.z])
        self.resolution = occupancyGrid.info.resolution
        self.updateCounter = 0

    def stateToPixel(self, state):
        pixel = []
        for i in range(len(state)):
            pixel.append(int((state[i]-self.origin[i])/self.resolution))
        return np.array(pixel)

    def pixelToState(self, pixel):
        state = []
        for i in range(len(pixel)):
            state.append(pixel[i]*self.resolution+self.origin[i])
        return np.array(state)

    def update(self, occupancyGrid):
        self.height = occupancyGrid.info.height
        self.width = occupancyGrid.info.width
        self.grid = np.flip(np.reshape(occupancyGrid.data, (self.height, self.width)), axis = 0)
        self.origin = np.array([occupancyGrid.info.origin.position.x, occupancyGrid.info.origin.position.y, occupancyGrid.info.origin.position.z])
        self.resolution = occupancyGrid.info.resolution
        self.updateCounter += 1
    
    def __str__(self):
        return("Map with height: {} \t width: {} \t Iteration: {}".format(self.height, self.width, self.updateCounter))