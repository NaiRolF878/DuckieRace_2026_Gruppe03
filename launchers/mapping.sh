#!/bin/bash

source /opt/ros/noetic/setup.bash
source devel/setup.bash

# ── Wahrnehmung ──────────────────────────────────────────────────────────────
rosrun mapping detect_lane_node.py &
rosrun mapping detect_apriltag_node.py &
# ── Graph-Zustand + Phasen-Logik + Dashboard ────────────────────────────────
rosrun mapping graph_state_node.py &
rosrun mapping switch_control_node.py &
rosrun mapping explore_control_node.py &
rosrun mapping path_planner_node.py &
rosrun mapping debug_graph_node.py &
# rosrun mapping camera_dashboard_node.py &
# ── Steuerung (erst starten wenn detect_lane Werte liefert) ──────────────────
sleep 5
rosrun mapping control_lane_node.py &
rosrun mapping control_intersection_node.py &
# Auf alle Nodes warten (blockiert bis Strg+C)
wait
