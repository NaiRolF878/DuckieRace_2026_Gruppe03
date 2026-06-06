#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# intersection_handling.sh – Launcher für Challenge 2 (Intersection Handling)
#
# Startet alle Nodes des intersection_handling-Pakets. roscore wird NICHT
# gestartet (läuft bereits separat auf dem Bot).
#
# Reihenfolge:
#   1. detect_lane_node        – Spur + rote Haltelinie (/detect/lane, /detect/stop_line)
#   2. detect_apriltag_node    – AprilTag-Richtung      (/detect/apriltag/direction, /id)
#   3. detect_red_lane_node    – Gegenspur beim Abbiegen (/intersection/turn_complete)
#   4. switch_control_node     – FSM (/enable/lane, /enable/intersection, /intersection/phase)
#   5. camera_dashboard_node   – Visualisierung
#   6. control_lane_node       – PID-Spurfolge (startet nach kurzer Wartezeit)
#   7. control_intersection_node – Kreuzungssteuerung
#
# Strg+C beendet alle Nodes sauber (cleanup-trap → cmd_vel = 0).
# Voraussetzung: VEHICLE_NAME ist gesetzt (z.B. export VEHICLE_NAME=dorette).
# ─────────────────────────────────────────────────────────────────────────────

source /opt/ros/noetic/setup.bash
source devel/setup.bash

pids=()
cleanup() {
    echo ""
    echo "[intersection_handling.sh] Beende alle Nodes ..."
    for pid in "${pids[@]}"; do
        kill -INT "$pid" 2>/dev/null
    done
    wait
    echo "[intersection_handling.sh] Alle Nodes beendet."
}
trap cleanup INT TERM EXIT

if ! rostopic list &>/dev/null; then
    echo "[intersection_handling.sh] FEHLER: ROS-Master nicht erreichbar."
    echo "                            Läuft roscore? Ist ROS_MASTER_URI korrekt?"
    exit 1
fi

# ── Wahrnehmung ──────────────────────────────────────────────────────────────
rosrun intersection_handling detect_lane_node.py &
pids+=($!)

rosrun intersection_handling detect_apriltag_node.py &
pids+=($!)

rosrun intersection_handling detect_red_lane_node.py &
pids+=($!)

# ── FSM + Visualisierung ─────────────────────────────────────────────────────
rosrun intersection_handling switch_control_node.py &
pids+=($!)

rosrun intersection_handling camera_dashboard_node.py &
pids+=($!)

# ── Steuerung (erst starten wenn detect_lane Werte liefert) ──────────────────
sleep 5

rosrun intersection_handling control_lane_node.py &
pids+=($!)

rosrun intersection_handling control_intersection_node.py &
pids+=($!)

# Auf alle Nodes warten (blockiert bis Strg+C)
wait
