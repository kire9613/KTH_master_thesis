from team4_project.mapping2.updatemap import UpdateMap
from dublins import *
from nav_msgs.msg import OccupancyGrid
from a_star import *
from test import *
import actionlib
import matplotlib.pyplot as plt
from math import pi
import rospy
import team4_msgs.msg
#echo 'export PYTHONPATH=$PYTHONPATH:"$(rospack find svea_core)/scripts"' >> ~/.bashrc

def main2():

    #map_service = UpdateMap()
    #map = map_service.get_map()
    #map_info = map_service.get_map_info()

    rospy.init_node('AStar')
    oc_map = rospy.wait_for_message('/map', OccupancyGrid)
    map = np.array(oc_map.data).reshape(oc_map.info.height,oc_map.info.width)
    map_info = [oc_map.info.width, oc_map.info.height, oc_map.info.resolution, oc_map.info.origin.position.x, oc_map.info.origin.position.y]

    # (x0,y0) = (-4,-9.2)
    # (xt,yt) = (-4.3,-6.85)

    (x0,y0) = (-2,-6.7)
    theta0 = pi/4
    (xt,yt) = (-4,-2.47)
    thetat = pi/4

    # (x0,y0) = (-6.64,-14.1)
    # (xt,yt) = (10.5,11.8)


    env = Environment(map, map_info)
    car = Objective(xt, yt, thetat, x0, y0, theta0, env)

    print("Setup done, start path-planning...")

    path = run_astar(car,smooth=False)

    print("Plot path...")

    plot_path(path)

def plot_path(path):#env

    #rospy.init_node('AStar')
    oc_map = rospy.wait_for_message('/map', OccupancyGrid)
    map = np.array(oc_map.data).reshape(oc_map.info.height,oc_map.info.width)
    map_info = [oc_map.info.width, oc_map.info.height, oc_map.info.resolution, oc_map.info.origin.position.x, oc_map.info.origin.position.y]

    env = Environment(map, map_info)

    path = path.poses

    # Plot map
    plt.imshow(env.map)
    origin_x = env.origin_x
    origin_y = env.origin_y
    resolution = env.resolution

    xv = []
    yv = []
    for i in range(0,len(path)):

        x = path[i].pose.position.x - origin_x
        x = int(x/resolution)
        y = path[i].pose.position.y - origin_y
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

    # Plot path
    # xv = []
    # yv = []
    # for i in range(0,len(path)):
    #
    #     x = path[i][0] - origin_x
    #     x = int(x/resolution)
    #     y = path[i][1] - origin_y
    #     y = int(y/resolution)
    #
    #     xv.append(x)
    #     yv.append(y)
    #     if i == 0:
    #         x1 = x
    #         y1 = y
    #     else:
    #         x2 = x
    #         y2 = y
    #         #plt.plot([x1,x2], [y1,y2], "r-")
    #         #plt.show()
    #         x1 = x2
    #         y1 = y2

    plt.plot(xv, yv, "r-")
    plt.plot(xv, yv, "b*", markersize=1)
    plt.show()

def goal_result(state, result):
    if actionlib.TerminalState.SUCCEEDED == state:
        rospy.loginfo("Action returned succeeded")
        plot_path(result.path)

    elif actionlib.TerminalState.RECALLED == state:
        rospy.loginfo("Action returned recalled")
    elif actionlib.TerminalState.REJECTED == state:
        rospy.loginfo("Action returned rejected")
    elif actionlib.TerminalState.PREEMPTED == state:
        rospy.loginfo("Action returned preempted")
    elif actionlib.TerminalState.ABORTED == state:
        rospy.loginfo("Action returned aborted")
    elif actionlib.TerminalState.LOST == state:
        rospy.loginfo("Action returned lost")

def get_path():
    global astar_client
    astar_client.wait_for_server()
    objective = team4_msgs.msg.AStarGoal()
    # initial position and heading
    objective.x0 = -6.64
    objective.y0 = -14.1
    objective.theta0 = pi/4
    # goal position and heading
    objective.xt = -3.28
    objective.yt = -6.55
    objective.thetat = pi
    objective.smooth = False
    astar_client.send_goal(objective, done_cb=goal_result)

if __name__ == "__main__":

    rospy.init_node("controller")

    """ Init exploration action service client name: get_next_goal (node: /explore), msg type: irob_assignment_1/GetNextGoalAction """
    astar_client = actionlib.SimpleActionClient('AStar_server', team4_msgs.msg.AStarAction) # client = actionlib.SimpleActionClient('ActionName', Topic)
    astar_client.wait_for_server()
    
    get_path()

    rospy.spin()

# def main():
#
#     rospy.init_node('AStar_client')
#     rospy.spin()
#
#     plot_path(path,env)

# if __name__ == '__main__':
#     print(__file__ + " start!!")
#     main()
#     print(__file__ + " Done!!")
