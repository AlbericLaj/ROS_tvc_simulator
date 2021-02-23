#!/bin/bash
set -e
cd ../../
catkin_make
source devel/setup.bash
rosnode kill -a
#uncomment to launch time_keeper automatically
#(sleep 5;nohup rosrun tvc_simulator time_keeper 2>/dev/null &)
roslaunch tvc_simulator rocket_launch.launch


