#!/bin/bash

source /opt/ros/melodic/setup.bash
source /home/ros/catkin_ws/devel/setup.bash
source /home/ros/autoware.ai/install/setup.bash

export ROS_HOSTNAME=xavier
export ROS_MASTER_URI=http://xavier:11311
export ROSLAUNCH_SSH_UNKNOWN=1

exec "$@"
