#!/bin/bash
cd ../../
source devel/setup.bash
(nohup rosrun tvc_simulator time_keeper 2>/dev/null &)

