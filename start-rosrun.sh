#!/bin/bash
sleep 10
. /ros_entrypoint.sh
cd /catkin_ws
source devel/setup.bash
rosrun my_package listener.py