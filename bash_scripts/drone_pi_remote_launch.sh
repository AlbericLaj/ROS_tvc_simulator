#!/bin/bash
./copy_on_pi.sh
#export ROS_MASTER_URI=http://155.207.33.185:11311
#export ROS_IP=155.207.33.185
#roslaunch tvc_simulator drone_pi_remote_launch.launch

ssh pi@raspberrypi.local "source /home/pi/ros_catkin_ws/devel/setup.bash; roslaunch tvc_simulator drone_pi_launch.launch"
