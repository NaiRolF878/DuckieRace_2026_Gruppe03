#!/bin/bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash

rosrun intersection_handling detect_lane_node.py &
rosrun intersection_handling detect_sign_node.py &
#rosrun intersection_handling configuration_node.py &
rosrun intersection_handling switch_control_node.py &
rosrun intersection_handling cross_intersection_node.py &
#rosrun intersection_handling camera_dashboard_node.py &
sleep 5

rosrun intersection_handling control_lane_node.py

wait
