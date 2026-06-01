#!/bin/bash

# ─────────────────────────────────────────────────────────────────────────────
# ducks.sh – Launcher für Challenge 3 "Watch out for Ducks"
#
# Startet alle benötigten Nodes als Hintergrundprozesse und beendet sie
# gemeinsam, wenn das Skript per Ctrl-C oder Container-Stopp terminiert wird.
#
# Annahme: Alle Python-Nodes liegen im SELBEN Ordner wie dieses Skript.
# Voraussetzung: VEHICLE_NAME ist gesetzt (z.B. export VEHICLE_NAME=dorette).
# ─────────────────────────────────────────────────────────────────────────────

set -e

if [ -z "$VEHICLE_NAME" ]; then
    echo "[ducks.sh] FEHLER: VEHICLE_NAME ist nicht gesetzt."
    exit 1
fi

# Verzeichnis dieses Skripts – die Nodes liegen direkt daneben
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "[ducks.sh] Starte Challenge 3 für VEHICLE_NAME=$VEHICLE_NAME"

# Alle Kindprozesse beim Beenden gemeinsam stoppen
pids=()
cleanup() {
    echo ""
    echo "[ducks.sh] Beende alle Nodes ..."
    for pid in "${pids[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
    echo "[ducks.sh] Fertig."
}
trap cleanup SIGINT SIGTERM EXIT

# Wahrnehmung
python3 "$SCRIPT_DIR/detect_lane_node.py" &
pids+=($!)
python3 "$SCRIPT_DIR/detect_duck_node.py" &
pids+=($!)

# Steuerung
python3 "$SCRIPT_DIR/control_lane_node.py" &
pids+=($!)
python3 "$SCRIPT_DIR/control_obstacle_node.py" &
pids+=($!)
python3 "$SCRIPT_DIR/switch_control_node.py" &
pids+=($!)

# Visualisierung (optional – braucht Display)
python3 "$SCRIPT_DIR/camera_dashboard_node.py" &
pids+=($!)

echo "[ducks.sh] Alle Nodes gestartet. Ctrl-C zum Beenden."
wait
