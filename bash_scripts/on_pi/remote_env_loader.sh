#!/bin/bash

export ROS_IP=192.168.43.101
export ROS_MASTER_URI=http://192.168.43.2:11311
#export ROSLAUNCH_SSH_UNKNOWN=1

source /home/pi/ros_catkin_ws/devel/setup.bash

exec "$@"
