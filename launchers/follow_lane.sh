#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# Strg+C beendet alle Nodes sauber (siehe cleanup-trap).
# ─────────────────────────────────────────────────────────────────────────────

source /opt/ros/noetic/setup.bash
source devel/setup.bash

# ── Sauberes Herunterfahren bei Strg+C oder Skript-Ende ──────────────────────
# Sendet SIGINT an alle gestarteten Nodes, damit rospy.on_shutdown() greift
# (wichtig für control_lane_node: setzt cmd_vel auf 0 → Bot stoppt sofort).
pids=()
cleanup() {
    echo ""
    echo "[follow_lane.sh] Beende alle Nodes ..."
    for pid in "${pids[@]}"; do
        kill -INT "$pid" 2>/dev/null
    done
    wait
    echo "[follow_lane.sh] Alle Nodes beendet."
}
trap cleanup INT TERM EXIT

# ── Prüfen ob der ROS-Master erreichbar ist ──────────────────────────────────
if ! rostopic list &>/dev/null; then
    echo "[follow_lane.sh] FEHLER: ROS-Master nicht erreichbar."
    echo "                  Läuft roscore? Ist ROS_MASTER_URI korrekt gesetzt?"
    exit 1
fi

# ── Nodes starten ────────────────────────────────────────────────────────────
rosrun follow_lane detect_lane_node.py &
rosrun follow_lane switch_control_node.py &
# rosrun follow_lane camera_dashboard_node.py &

# control_lane_node erst starten wenn detect_lane_node Werte liefert
sleep 5
rosrun follow_lane control_lane_node.py &

wait
