#!/bin/bash
source /opt/ros/melodic/setup.bash
source ~/svea_starter/devel/setup.bash
export ROS_HOSTNAME="svea1.local"
exec "$@"
