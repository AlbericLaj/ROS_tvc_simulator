#!/bin/bash
ssh pi@raspberrypi.local "source /home/pi/ros_catkin_ws/devel/setup.bash; rosnode kill -a"
