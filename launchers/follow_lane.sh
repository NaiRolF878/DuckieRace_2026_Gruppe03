#!/bin/bash

source /opt/ros/noetic/setup.bash
source devel/setup.bash

# ── Nodes starten ────────────────────────────────────────────────────────────
rosrun follow_lane detect_lane_node.py &
rosrun follow_lane switch_control_node.py &
# rosrun follow_lane camera_dashboard_node.py &

# control_lane_node erst starten wenn detect_lane_node Werte liefert
sleep 5
rosrun follow_lane control_lane_node.py &

wait
