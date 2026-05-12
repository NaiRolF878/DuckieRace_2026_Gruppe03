#!/bin/bash

source /opt/ros/noetic/setup.bash
source devel/setup.bash

trap "kill 0" EXIT

# --- core lane system (Basis für Bewegung) ---
rosrun follow_lane detect_lane_node.py &
rosrun follow_lane switch_control_node.py &

sleep 3

rosrun follow_lane control_lane_node.py &

# --- gate detection (mapping perception) ---
rosrun mapping detect_gate_node.py &

sleep 2

# --- mapping logic (Graph-Aufbau) ---
rosrun mapping mapping_node.py &

sleep 2

# --- path planning (entscheidet nächste Ziele) ---
rosrun mapping pathfinding_node.py &

# --- optional: waypoint execution (falls getrennt von lane control) ---
rosrun mapping waypoint_navigation_node.py &

wait
