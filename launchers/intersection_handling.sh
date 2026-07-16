#!/bin/bash

source /opt/ros/noetic/setup.bash
source devel/setup.bash

# ── Nodes starten ─────────────────────────────────────────
# ── Wahrnehmung ──────────────────────────────────────────────────────────────
rosrun intersection_handling detect_lane_node.py &
rosrun intersection_handling detect_apriltag_node.py &

# ── FSM + Visualisierung ─────────────────────────────────────────────────────
rosrun intersection_handling switch_control_node.py &
rosrun intersection_handling camera_dashboard_node.py &

# ── Steuerung (erst starten wenn detect_lane Werte liefert) ──────────────────
sleep 5
rosrun intersection_handling control_lane_node.py &
rosrun intersection_handling control_intersection_node.py &

# Auf alle Nodes warten (blockiert bis Strg+C)
wait
