#!/bin/bash
. /ros_entrypoint.sh
sleep 10
rosrun rosserial_python serial_node.py tcp