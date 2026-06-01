#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# intersection.sh – Launcher für Challenge 2 "Intersection Handling"
#
# Startet alle benötigten Nodes als Hintergrundprozesse und beendet sie
# gemeinsam, wenn das Skript per Ctrl-C oder Container-Stopp terminiert wird.
#
# Annahme: Alle Python-Nodes liegen im SELBEN Ordner wie dieses Skript (src/).
# Voraussetzung: VEHICLE_NAME ist gesetzt (z.B. export VEHICLE_NAME=dorette).
# ─────────────────────────────────────────────────────────────────────────────
set -e

if [ -z "$VEHICLE_NAME" ]; then
    echo "[intersection.sh] FEHLER: VEHICLE_NAME ist nicht gesetzt."
    exit 1
fi

# Verzeichnis dieses Skripts – die Nodes liegen direkt daneben
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "[intersection.sh] Starte Challenge 2 für VEHICLE_NAME=$VEHICLE_NAME"

# Alle Kindprozesse beim Beenden gemeinsam stoppen
pids=()
cleanup() {
    echo ""
    echo "[intersection.sh] Beende alle Nodes ..."
    for pid in "${pids[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
    echo "[intersection.sh] Fertig."
}
trap cleanup SIGINT SIGTERM EXIT

# ── Wahrnehmung ──────────────────────────────────────────────────────────────
# Spurerkennung (Spurversatz -> /detect/lane)
python3 "$SCRIPT_DIR/detect_lane_node.py" &
pids+=($!)

# AprilTag + rote Haltelinie (-> /detect/intersection, /detect/apriltag/direction,
#                                /detect/red_line_side)
python3 "$SCRIPT_DIR/detect_apriltag_node.py" &
pids+=($!)

# ── Steuerung ────────────────────────────────────────────────────────────────
# Normales Spurfolgen (aktiv bei ControlType.Lane)
python3 "$SCRIPT_DIR/control_lane_node.py" &
pids+=($!)

# Kreuzungssteuerung (aktiv bei Approaching / Turning / LaneHandover)
python3 "$SCRIPT_DIR/control_intersection_node.py" &
pids+=($!)

# Zentraler Zustandsautomat (steuert die Phasen, publiziert /switch/control)
python3 "$SCRIPT_DIR/switch_control_node.py" &
pids+=($!)

echo "[intersection.sh] Alle Nodes gestartet. Ctrl-C zum Beenden."
wait
