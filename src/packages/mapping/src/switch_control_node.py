#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────────────────────
# switch_control_node.py  (Challenge 4 – Mapping & Path Finding)
#
# Zentraler Zustandsautomat (FSM).
#
# Phasen:
#   Lane        – normales Spurfolgen (control_lane aktiv)
#   Stopping    – an der Haltelinie warten
#   Turning     – abbiegen (Sequenz wird von control_intersection gesteuert)
#
# Die Abbiegerichtung kommt deterministisch von explore_control_node/
# path_planner_node ueber /navigation/next_direction. Ist next_direction nicht
# in allowed_dirs (den an der aktuellen Kreuzung laut Tag erlaubten Richtungen)
# enthalten, bleibt der Bot in STOPPING stehen und wartet weiter – kein
# Zufalls-Fallback. Die Richtungspruefung liegt deshalb in _update_state (nicht
# in cbStopLine): sie muss bei jedem STOPPING-Tick neu versucht werden, weil
# next_direction erst NACH dem Anhalten eintreffen kann.
#
# Graph-Fallback: allowed_dirs stammt normalerweise aus der LIVE Tag-Erkennung
# (/detect/apriltag/direction). Kann die Kamera den Tag an dieser Kreuzung gar
# nicht (mehr) lesen, bliebe der Bot sonst fuer immer in STOPPING haengen -
# graph_state_node kennt die erlaubten Richtungen an dieser Kreuzung aber
# bereits deterministisch aus der eigenen Kartenverfolgung (/graph/
# allowed_directions), unabhaengig von der Kamera. Diese Quelle wird daher als
# Fallback verwendet: sofort, falls beim Eintritt in STOPPING gar keine Live-
# Richtung vorliegt (cbStopLine), und nach stopping_fallback_timeout Sekunden,
# falls die eingefrorene Live-Richtung zwar vorliegt aber nicht zu
# next_direction passt (_update_state) - z.B. weil sie noch ein Rest der
# vorherigen Kreuzung war ("Speicher hat noch die letzte ID").
# ─────────────────────────────────────────────────────────────────────────────

import os
import rospy
from std_msgs.msg import Bool, String
import util


class SwitchControlNode:
    LANE        = "Lane"
    STOPPING    = "Stopping"
    TURNING     = "Turning"

    def __init__(self, node_name):
        rospy.init_node(node_name)
        self._vehicle_name = os.environ['VEHICLE_NAME']

        self.phase            = self.LANE
        self.phase_start_time = rospy.Time.now()
        self.allowed_dirs     = []
        self.graph_allowed_dirs = []   # Fallback aus /graph/allowed_directions
        self.used_graph_fallback = False   # nur fuers Debug-Dashboard
        self.direction        = "straight"
        self.next_direction   = ""     # von explore_control_node/path_planner_node
        self.stop_line        = False
        self.turn_done        = False

        # Timing-Defaults
        self.stop_duration            = 2.0
        self.turning_timeout          = 8.0
        # Deutlich laenger als detect_apriltag_node's tag_memory.seconds (3.0s
        # Default), damit dem Live-Pfad genug Zeit bleibt, bevor auf den
        # Graph-Fallback zurueckgegriffen wird.
        self.stopping_fallback_timeout = 6.0

        util.init_parameters(node_name, self.cbUpdateParameters)

        # ── Publisher ─────────────────────────────────────────────────────────
        self.pub_enable_lane = rospy.Publisher(
            f'/{self._vehicle_name}/enable/lane', Bool, queue_size=1)
        self.pub_enable_inter = rospy.Publisher(
            f'/{self._vehicle_name}/enable/intersection', Bool, queue_size=1)
        self.pub_phase = rospy.Publisher(
            f'/{self._vehicle_name}/intersection/phase', String, queue_size=1)
        self.pub_direction = rospy.Publisher(
            f'/{self._vehicle_name}/intersection/direction', String, queue_size=1)
        # Bündelt Start-Signal + Richtung in EINER Nachricht (einmalig beim
        # Uebergang nach TURNING, nicht wie /intersection/phase & /intersection/
        # direction jeden Tick auf getrennten Topics) - verhindert, dass
        # control_intersection_node die Abbiege-Sequenz mit einer noch alten
        # Richtung startet, falls die "phase"-Nachricht dort vor der
        # "direction"-Nachricht verarbeitet wird (zwei unabhaengige Topics,
        # keine Verarbeitungsreihenfolge garantiert).
        self.pub_turn_start = rospy.Publisher(
            f'/{self._vehicle_name}/intersection/turn_start', String, queue_size=1)

        # ── Subscriber ────────────────────────────────────────────────────────
        rospy.Subscriber(f'/{self._vehicle_name}/graph/allowed_directions',
                         String, self.cbGraphAllowedDirections, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/detect/stop_line',
                         Bool, self.cbStopLine, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/detect/apriltag/direction',
                         String, self.cbApriltagDirection, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/intersection/turn_done',
                         Bool, self.cbTurnDone, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/navigation/next_direction',
                         String, self.cbNextDirection, queue_size=1)

        rospy.loginfo(f"[{node_name}] Bereit – Zustand: Lane")

    def cbUpdateParameters(self, parameters):
        # Safely get parameters to avoid KeyErrors
        timing = parameters.get("timing", {})

        if "stop_duration" in timing:
            self.stop_duration = timing["stop_duration"].get("default", 2.0)

        if "turning_timeout" in timing:
            self.turning_timeout = timing["turning_timeout"].get("default", 8.0)

        if "stopping_fallback_timeout" in timing:
            self.stopping_fallback_timeout = timing["stopping_fallback_timeout"].get("default", 6.0)

    def cbGraphAllowedDirections(self, msg):
        self.graph_allowed_dirs = msg.data.split(",") if msg.data else []

    def cbStopLine(self, msg):
        self.stop_line = msg.data

        if not (msg.data and self.phase == self.LANE):
            return

        dirs = self.allowed_dirs
        self.used_graph_fallback = False
        if not dirs or dirs == ["unknown"]:
            # Live-Erkennung liefert gerade nichts Brauchbares - sofort auf
            # den Graph-Fallback ausweichen statt erst noch stopping_fallback_
            # timeout in STOPPING zu verlieren (hier gibt es ja gar keinen
            # Live-Wert, den es abzuwarten lohnt).
            dirs = self.graph_allowed_dirs
            self.used_graph_fallback = bool(dirs)

        if not dirs or dirs == ["unknown"]:
            rospy.loginfo_throttle(2.0,
                "[switch] Rote Linie ohne Tag-Richtung (auch kein Graph-Fallback) "
                "-> keine Kreuzung, fahre weiter")
            return

        self.allowed_dirs = dirs
        source = "Graph-Fallback" if self.used_graph_fallback else "Live-Tag"
        rospy.loginfo(f"[switch] Kreuzung (Linie+Tag) -> STOPPING | "
                      f"erlaubte Richtungen ({source}): {self.allowed_dirs}")
        self._transition_to(self.STOPPING)

    def cbApriltagDirection(self, msg):
        if self.phase == self.LANE and msg.data and msg.data != "unknown":
            self.allowed_dirs = msg.data.split(",")

    def cbTurnDone(self, msg):
        if msg.data:
            self.turn_done = True

    def cbNextDirection(self, msg):
        self.next_direction = msg.data

    def _transition_to(self, new_phase):
        self.phase            = new_phase
        self.phase_start_time = rospy.Time.now()
        if new_phase == self.TURNING:
            self.turn_done = False
            # self.direction ist an beiden Aufrufstellen (_update_state) bereits
            # auf die bestaetigte Richtung gesetzt, BEVOR _transition_to
            # aufgerufen wird - hier also garantiert aktuell.
            self.pub_turn_start.publish(String(data=self.direction))
        rospy.loginfo(f"[switch] -> {new_phase}")

    def _update_state(self):
        elapsed = (rospy.Time.now() - self.phase_start_time).to_sec()

        if self.phase == self.STOPPING:
            if elapsed < self.stop_duration:
                return
            if self.next_direction and self.next_direction in self.allowed_dirs:
                self.direction = self.next_direction
                rospy.loginfo(f"[switch] Richtung bestaetigt: {self.direction} "
                              f"(aus {self.allowed_dirs}) -> TURNING")
                self._transition_to(self.TURNING)
            elif (self.next_direction and not self.used_graph_fallback
                    and elapsed >= self.stopping_fallback_timeout
                    and self.next_direction in self.graph_allowed_dirs):
                # Die beim STOPPING-Eintritt eingefrorene Live-Richtung passt
                # seit stopping_fallback_timeout Sekunden nicht zu next_direction
                # (z.B. weil sie noch ein Rest der vorherigen Kreuzung war) -
                # auf den unabhaengigen Graph-Fallback ausweichen, statt fuer
                # immer haengen zu bleiben (siehe Kopfkommentar).
                self.used_graph_fallback = True
                self.direction = self.next_direction
                rospy.logwarn(f"[switch] Live-Richtung {self.allowed_dirs} passt seit "
                              f"{elapsed:.1f}s nicht zu next_direction "
                              f"('{self.next_direction}') -> Graph-Fallback "
                              f"{self.graph_allowed_dirs} -> TURNING")
                self._transition_to(self.TURNING)
            else:
                rospy.logwarn_throttle(1.0,
                    f"[switch] Keine gueltige next_direction ('{self.next_direction}') "
                    f"in erlaubten Richtungen {self.allowed_dirs} - bleibe in STOPPING")

        elif self.phase == self.TURNING:
            if self.turn_done or elapsed >= self.turning_timeout:
                rospy.loginfo("[switch] Turning fertig -> LANE")
                self.allowed_dirs = []
                self._transition_to(self.LANE)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self._update_state()

            lane_active = (self.phase == self.LANE)
            self.pub_enable_lane.publish(Bool(data=lane_active))
            self.pub_enable_inter.publish(Bool(data=not lane_active))
            # direction VOR phase publizieren: graph_state_node loest seinen
            # Graph-Uebergang am Wechsel zu "Turning" aus und liest dabei die
            # zuletzt empfangene direction - die muss also garantiert schon
            # die neue (gerade bestaetigte) Richtung sein.
            self.pub_direction.publish(String(data=self.direction))
            self.pub_phase.publish(String(data=self.phase))

            rate.sleep()


if __name__ == '__main__':
    node = SwitchControlNode('switch_control_node')
    node.run()
    rospy.spin()
