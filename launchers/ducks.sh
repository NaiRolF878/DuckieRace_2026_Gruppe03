#!/bin/bash

source /opt/ros/noetic/setup.bash
source devel/setup.bash

trap "kill 0" EXIT

# --- core lane system (immer aktiv, damit der Duckiebot fährt) ---
rosrun follow_lane detect_lane_node.py &
rosrun follow_lane switch_control_node.py &
rosrun follow_lane camera_dashboard_node.py &
# Nur für die Entwicklungszeit - gezieltes Debugging
rosrun rostopic echo /detect/duck &

sleep 3

rosrun follow_lane control_lane_node.py &

# --- duck detection (perception) ---
rosrun ducks detect_duck_node.py &

sleep 2

# --- obstacle / reaction layer ---
rosrun ducks control_obstacle_node.py &

wait
