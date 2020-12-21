#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division

import numpy as np

import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion

import svea.bresenham.bresenham as bresenham
import matplotlib.pyplot as plt

update_rate = 2 # [Hz]
width = 635#1269
height = 284#567
resolution = 0.1# 0.050000
origo_x = -30.549770
origo_y = -11.414917
shift_x = np.int16(-origo_x/resolution)
shift_y = np.int16(-origo_y/resolution)

keep_out = [ -4.334904193878174    , 1.1745818853378296  , -2.0807793140411377 ,
            1.0223605632781982     , -0.7974302768707275 , 0.433208703994751   ,
            1.0080901384353638     , 0.3776954114437103  , 2.304389238357544   ,
            2.077775478363037      , 3.400698184967041   , 2.6027960777282715  ,
            5.336561679840088      , 2.9498884677886963  , 6.986306190490723   ,
            2.4867916107177734     , 7.869754791259766   , 1.757921576499939   ,
            9.999625205993652      , 0.22264790534973145 , 10.763415336608887  ,
            0.00029087066650390625 , 18.133031845092773  , -0.5012322664260864 ,
            19.05765724182129      , 0.9421164989471436  , 18.479339599609375  ,
            2.661898612976074      , 18.24079132080078   , 3.2987730503082275  ,
            10.238487243652344     , 3.6555912494659424  , 5.328366756439209   ,
            4.174242973327637      , 4.020337104797363   , 6.569239616394043   ,
            3.472522735595703      , 6.7261881828308105  , 2.8425261974334717  ,
            6.379829406738281      , 1.503035306930542   , 4.609742641448975   ,
            0.9624210596084595     , 4.542537689208984   , -1.8879657983779907 ,
            4.617576599121094      , -3.8524134159088135 , 4.670773983001709   ,
            -9.22355842590332      , 4.917575359344482   , -13.259159088134766 ,
            5.201533794403076      , -13.490726470947266 , 4.969977378845215   ,
            -13.685510635375977    , 1.985185980796814   , -13.411819458007812 ,
            1.508816123008728      , -8.568108558654785  , 1.242281436920166   ,
            -8.303826332092285     , 1.5394721031188965  , -8.146903991699219  ,
            4.079605579376221      , -3.9821314811706543 , 3.896056652069092   , ]

stay_in = [ -5.353427886962891 , -0.38703346252441406 , -0.44403958320617676 ,
           -0.804934024810791  , 0.7682813405990601   , -0.42697232961654663 ,
           2.3166537284851074  , -0.4010797142982483  , 3.8560824394226074   ,
           0.9267876148223877  , 4.498049259185791    , 1.5299468040466309   ,
           5.938640594482422   , 1.4919934272766113   , 6.941065788269043    ,
           0.7893449068069458  , 7.931108474731445    , -0.6752626895904541  ,
           10.626561164855957  , -0.9736595153808594  , 13.798611640930176   ,
           -2.083423376083374  , 17.296281814575195   , -2.097644090652466   ,
           20.340763092041016  , -1.0839290618896484  , 20.691726684570312   ,
           3.4208574295043945  , 18.08633041381836    , 4.772830486297607    ,
           6.789177894592285   , 5.406223297119141    , 5.2891845703125      ,
           7.81485652923584    , 2.277754783630371    , 7.969513893127441    ,
           0.39436399936676025 , 5.879500865936279    , -7.070324897766113   ,
           6.414761543273926   , -14.484965324401855  , 6.861196994781494    ,
           -14.800274848937988 , 0.9635370969772339   , -13.652288436889648  ,
           0.30832695960998535 , -8.261316299438477   , 0.10501611232757568  ,
           -7.194767951965332  , -0.08113396167755127 , -7.0688395500183105  ,
           3.063446044921875   , -6.759504318237305   , 3.0577328205108643   ,
           -6.789384365081787  , -0.16967664659023285 , ]

out_x = keep_out[0::2]+[keep_out[0]]
out_y = keep_out[1::2]+[keep_out[1]]

in_x = stay_in[0::2]+[stay_in[0]]
in_y = stay_in[1::2]+[stay_in[1]]

gen_in = np.array([[
    np.int16(in_x[i]/resolution)+shift_x,
    np.int16(in_y[i]/resolution)+shift_y,
    np.int16(in_x[i+1]/resolution)+shift_x,
    np.int16(in_y[i+1]/resolution)+shift_y,
    i,
         ]
    for i in range(len(in_x)-1)],dtype=np.int32)

gen_out = np.array([[
    np.int16(out_x[i]/resolution)+shift_x,
    np.int16(out_y[i]/resolution)+shift_y,
    np.int16(out_x[i+1]/resolution)+shift_x,
    np.int16(out_y[i+1]/resolution)+shift_y,
    i,
         ]
    for i in range(len(out_x)-1)],dtype=np.int32)

in_matrix = np.zeros((width,height),dtype=np.int32)
out_matrix = np.zeros((width,height),dtype=np.int32)
_ = np.zeros((width,height),dtype=np.int32)
bresenham.bres_segments_count(gen_in,in_matrix,_)
bresenham.bres_segments_count(gen_out,out_matrix,_)
track_matrix = (in_matrix + out_matrix)*100

np.savetxt('track_matrix.txt', track_matrix, fmt='%d')


# plt.imshow(track_matrix.T,origin="lower")
# plt.show()

# plt.plot(in_x,in_y,'ob')
# plt.plot(out_x,out_y,'or')
# plt.axis('equal')
# plt.show()

class Node:
    def __init__(self):
        rospy.init_node("map_from_track")
        map_pub = rospy.Publisher("map",OccupancyGrid,queue_size=1,latch=True)
        map = OccupancyGrid()
        map.header.stamp = rospy.Time.now()
        map.header.frame_id = "map"
        map.info.width = width
        map.info.height = height
        map.info.resolution = resolution
        map.info.origin.position.x = origo_x
        map.info.origin.position.y = origo_y
        map.info.origin.position.z = 0.0
        map.info.origin.orientation.x = 0.0
        map.info.origin.orientation.y = 0.0
        map.info.origin.orientation.z = 0.0
        map.info.origin.orientation.w = 1.0
        map.data = track_matrix.reshape(-1,order='F')
        map_pub.publish(map)
    def run(self):
        while not rospy.is_shutdown():
            pass
        rospy.spin()

if __name__ == "__main__":
	try:
		node = Node()
		node.run()
	except rospy.ROSInterruptException:
		pass
