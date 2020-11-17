from team4_project.mapping2.updatemap import UpdateMap
from dublins import *
from nav_msgs.msg import OccupancyGrid
from a_star import *
from test import *
import matplotlib.pyplot as plt
import rospy
#echo 'export PYTHONPATH=$PYTHONPATH:"$(rospack find svea_core)/scripts"' >> ~/.bashrc

def main():

    #map_service = UpdateMap()
    #map = map_service.get_map()
    #map_info = map_service.get_map_info()
    #map_info = None
    #map = None
    rospy.init_node('AStar')
    oc_map = rospy.wait_for_message('/map', OccupancyGrid)
    map = np.array(oc_map.data).reshape(oc_map.info.height,oc_map.info.width)
    map_info = [oc_map.info.width, oc_map.info.height, oc_map.info.resolution, oc_map.info.origin.position.x, oc_map.info.origin.position.y]

    # (x0,y0) = (-4,-9.2)
    # (xt,yt) = (-4.3,-6.85)
    #
    # (x0,y0) = (-2,-6.7)
    # (xt,yt) = (-4,-2.47)

    (x0,y0) = (-6.64,-14.1)
    (xt,yt) = (10.5,11.8)


    env = Environment(map, map_info)
    car = Objective(xt, yt, x0, y0, env)

    print("Setup done, start path-planning...")

    path = run_astar(car)
    #path = solution(car)

    print("Plot path...")

    plot_path(path,env)

def plot_path(path,env):

    # Plot map
    plt.imshow(env.map)
    origin_x = env.origin_x
    origin_y = env.origin_y
    resolution = env.resolution

    # Plot path
    xv = []
    yv = []
    for i in range(0,len(path)):

        x = path[i][0] - origin_x
        x = int(x/resolution)
        y = path[i][1] - origin_y
        y = int(y/resolution)

        xv.append(x)
        yv.append(y)
        if i == 0:
            x1 = x
            y1 = y
        else:
            x2 = x
            y2 = y
            #plt.plot([x1,x2], [y1,y2], "r-")
            #plt.show()
            x1 = x2
            y1 = y2

    plt.plot(xv, yv, "r-")
    plt.plot(xv, yv, "b*", markersize=1)
    plt.show()

if __name__ == '__main__':
    print(__file__ + " start!!")
    main()
    print(__file__ + " Done!!")
