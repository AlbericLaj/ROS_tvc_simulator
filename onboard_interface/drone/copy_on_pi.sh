#!/bin/bash
#copy avionic bridge
scp -i ~/.ssh/id_rsa -r on_pi/avionic_bridge.py pi@raspberrypi.local:~/ros_catkin_ws/src/drone_tvc_simulator/scripts/avionic_bridge.py

#copy actuators_tests
scp -i ~/.ssh/id_rsa -r on_pi/actuators_tests.py pi@raspberrypi.local:~/ros_catkin_ws/src/drone_tvc_simulator/scripts/actuators_tests.py

#copy launch file
scp -i ~/.ssh/id_rsa -r on_pi/drone_pi_launch.launch pi@raspberrypi.local:~/ros_catkin_ws/src/drone_tvc_simulator/drone_pi_launch.launch

#copy drone control c++ files
scp -i ~/.ssh/id_rsa -r ../src/prototypes/drone/* pi@raspberrypi.local:~/ros_catkin_ws/src/drone_tvc_simulator/src

#scp -i ~/.ssh/id_rsa -r ../msg/* pi@raspberrypi.local:~/ros_catkin_ws/src/drone_tvc_simulator/msg

#copy CMakeLists
scp -i ~/.ssh/id_rsa -r on_pi/CMakeLists.txt  pi@raspberrypi.local:~/ros_catkin_ws/src/drone_tvc_simulator


scp -i ~/.ssh/id_rsa -r on_pi/remote_env_loader.sh  pi@raspberrypi.local:~/ros_catkin_ws/devel

#copy PolyMPC
#scp -i ~/.ssh/id_rsa -r ../submodule  pi@raspberrypi.local:~/ros_catkin_ws/src/drone_tvc_simulator/submodule
