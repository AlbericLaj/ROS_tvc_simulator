#!/bin/bash
ssh pi@raspberrypi "~/pi/.../start.sh"
catkin_make
roslaunch tvc_simulator drone_pi_launch.launch