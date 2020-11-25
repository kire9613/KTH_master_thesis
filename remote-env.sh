#!/bin/bash
source /opt/ros/melodic/setup.bash
source /home/oscar/svea_starter/devel/setup.bash
export PYTHONPATH="$PYTHONPATH:$(rospack find svea_core)/scripts"
export ROS_HOSTNAME=$(hostname).local
exec "$@"
