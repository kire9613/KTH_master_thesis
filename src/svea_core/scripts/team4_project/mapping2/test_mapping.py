#!/usr/bin/env python

"""
Code used by the mapping node is partly based on code from lab in DD2410
Introduction to Robotics written by: @author: Daniel Duberg (dduberg@kth.se).
Further, code produced by team4 members in DD2410 submisions, together with modifications
to make the code usable for this project, is also part of the mapping node code.
"""

from engine_ros import EngineROS
import rospy

def main():
    """
    Calling ros engine to init mapping node "Mapper".
    Defines parameters to be used for specific environments:
    - q1, outside q1
    - q1, outside q2

    Switching between running q1/q2
    - Comment/uncomment params below
    - Change map, obstacles and track files in team4.launch
    - Change start_pt in team4.launch
    - Delete default_map.txt
    """

    map_frame_id = "map"
    map_resolution = 0.05

    # Running outside q2
    #map_width = 721
    #map_height = 880
    #map_origin_x = -17.581444
    #map_origin_y = -22.876441

    # Runing outside q1
    map_width = 1269
    map_height = 567
    map_origin_x = -30.55
    map_origin_y = -11.4

    map_origin_yaw = 0
    inflate_radius = 5
    unknown_space = -1
    free_space = 0
    c_space = 110
    occupied_space = 100

    mapping_engine = EngineROS(map_frame_id, map_resolution, map_width, map_height,
                 map_origin_x, map_origin_y, map_origin_yaw, inflate_radius,
                 unknown_space, free_space, c_space, occupied_space)

    rospy.spin()

if __name__ == '__main__':
    print(__file__ + " start!!")
    main()
    print(__file__ + " Done!!")
