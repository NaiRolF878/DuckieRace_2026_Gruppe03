#!/bin/bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash

# Wahrnehmung
rosrun ducks detect_lane_node.py &
# Steuerung
rosrun ducks control_lane_node.py &
rosrun ducks control_obstacle_node.py &
rosrun ducks switch_control_node.py &
# Visualisierung (optional, braucht Display)
#rosrun ducks camera_dashboard_node.py &

wait
