#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# ducks.sh – Launcher für Challenge 3 "Watch out for Ducks"
# Startet alle Nodes aus dem SELBEN Ordner. Voraussetzung: VEHICLE_NAME gesetzt.
# ─────────────────────────────────────────────────────────────────────────────
set -e
if [ -z "$VEHICLE_NAME" ]; then
    echo "[ducks.sh] FEHLER: VEHICLE_NAME ist nicht gesetzt."
    exit 1
fi
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
echo "[ducks.sh] Starte Challenge 3 fuer VEHICLE_NAME=$VEHICLE_NAME"

pids=()
cleanup() {
    echo ""; echo "[ducks.sh] Beende alle Nodes ..."
    for pid in "${pids[@]}"; do kill "$pid" 2>/dev/null || true; done
    wait 2>/dev/null || true
    echo "[ducks.sh] Fertig."
}
trap cleanup SIGINT SIGTERM EXIT

# Wahrnehmung
python3 "$DIR/detect_lane_node.py" &
pids+=($!)
pids+=($!)
# Steuerung
python3 "$DIR/control_lane_node.py" &
pids+=($!)
python3 "$DIR/control_obstacle_node.py" &
pids+=($!)
python3 "$DIR/switch_control_node.py" &
pids+=($!)
# Visualisierung (optional, braucht Display)
python3 "$DIR/camera_dashboard_node.py" &
pids+=($!)

echo "[ducks.sh] Alle Nodes gestartet. Ctrl-C zum Beenden."
wait
