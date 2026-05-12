#!/bin/bash

source /opt/ros/noetic/setup.bash
source devel/setup.bash

trap "kill 0" EXIT

# --- perception + system ---
rosrun follow_lane detect_lane_node.py &
rosrun follow_lane switch_control_node.py &
rosrun intersection_handling detect_apriltag_node.py &

sleep 5

# --- control ---
rosrun follow_lane control_lane_node.py &
rosrun intersection_handling control_intersection_node.py &

wait
