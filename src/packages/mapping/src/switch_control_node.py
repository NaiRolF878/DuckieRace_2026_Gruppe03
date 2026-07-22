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
# path_planner_node ueber /navigation/next_direction - diese hat IMMER
# Vorrang, sobald sie vorliegt (_update_state wartet in STOPPING nur, bis
# next_direction nicht mehr leer ist, kein Zufalls-Fallback). Sie basiert auf
# der Graph-Topologie aus mapping_node.json (von Hand gegen die echte Strecke
# geprueft).
#
# Kreuzungs-Erkennung (cbStopLine) und die geloggten "erlaubten Richtungen"
# kommen bewusst NUR NOCH aus /graph/allowed_directions (graph_state_node,
# rein aus der Kartenverfolgung + predicted_entry_tag). Eine fruehere, davon
# unabhaengige LIVE-Tag-Erkennung (/detect/apriltag/direction) wurde entfernt:
# sie sollte urspruenglich als zweite, kamera-only Quelle absichern, hat aber
# in der Praxis selbst Fehler eingebracht (z.B. denselben Tag ueber laengere
# Zeit hinweg konsistent falsch gelesen, oder tag_directions unvollstaendig
# konfiguriert - siehe fehler.md-Analyse vom 2026-07-22). Ist die Graph-
# Topologie korrekt und current_node/predicted_entry_tag sauber mitgefuehrt
# (turn_start-Atomaritaet, "Bot versetzt"-Reset, siehe graph_state_node),
# liefert die Karte die zuverlaessigere Antwort als die Kamera - daher genau
# EINE Quelle statt zwei potenziell widerspruechlichen. Voraussetzung:
# mapping_start_entry_tag/delivery_start_entry_tag muessen in
# mapping_node.json gesetzt sein, sonst bleibt predicted_entry_tag an der
# allerersten Kreuzung jeder Phase leer und die Kreuzung wird nicht erkannt.
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
        self.allowed_dirs     = []   # aus /graph/allowed_directions, nur Info/Dashboard
        self.direction        = "straight"
        self.next_direction   = ""     # von explore_control_node/path_planner_node
        self.stop_line        = False
        self.turn_done        = False

        # Timing-Defaults
        self.stop_duration            = 2.0
        self.turning_timeout          = 8.0

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

    def cbGraphAllowedDirections(self, msg):
        self.allowed_dirs = msg.data.split(",") if msg.data else []

    def cbStopLine(self, msg):
        self.stop_line = msg.data

        if not (msg.data and self.phase == self.LANE):
            return

        if not self.allowed_dirs:
            # predicted_entry_tag (graph_state_node) noch nicht bekannt - kann
            # an der allerersten Kreuzung jeder Phase passieren, wenn
            # mapping_start_entry_tag/delivery_start_entry_tag nicht gesetzt
            # sind (siehe Kopfkommentar). Ohne erlaubte Richtungen keine
            # sinnvolle Kreuzungs-Erkennung - weiterfahren statt falsch stehen
            # zu bleiben.
            rospy.loginfo_throttle(2.0,
                "[switch] Rote Linie, aber noch keine erlaubten Richtungen vom "
                "Graph (predicted_entry_tag unbekannt) -> keine Kreuzung, fahre weiter")
            return

        rospy.loginfo(f"[switch] Kreuzung (Linie+Graph) -> STOPPING | "
                      f"erlaubte Richtungen: {self.allowed_dirs}")
        self._transition_to(self.STOPPING)

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
            if self.next_direction:
                # next_direction (von explore_control_node/path_planner_node)
                # hat Vorrang - kein Abgleich gegen allowed_dirs mehr noetig
                # (siehe Kopfkommentar: beide stammen jetzt ohnehin aus
                # derselben Quelle, graph_state_node).
                self.direction = self.next_direction
                rospy.loginfo(f"[switch] Richtung: {self.direction} "
                              f"(aus Planung; Graph erlaubt: {self.allowed_dirs}) -> TURNING")
                self._transition_to(self.TURNING)
            else:
                rospy.logwarn_throttle(1.0,
                    "[switch] Noch keine next_direction von der Planung - bleibe in STOPPING")

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
