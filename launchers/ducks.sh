#!/bin/bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash

# ── Wahrnehmung ──────────────────────────────────────────────────────────────
rosrun ducks detect_lane_node.py &
# ── FSM + Visualisierung ─────────────────────────────────────────────────────
rosrun ducks switch_control_node.py &
# rosrun ducks camera_dashboard_node.py &
rosrun ducks configuration_node.py &
# ── Steuerung (erst starten wenn detect_lane Werte liefert) ──────────────────
sleep 5
rosrun ducks control_lane_node.py &
rosrun ducks control_obstacle_node.py &

wait
