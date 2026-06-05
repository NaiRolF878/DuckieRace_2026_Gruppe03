#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────────────────────
# switch_control_node.py  (Challenge 2 – Intersection Handling)
#
# Zentraler Zustandsautomat (FSM). Einzige Instanz mit Zustandslogik – alle
# Erkennungs-Nodes liefern nur Signale, hier wird entschieden.
#
# Phasen:
#   Lane        – normales Spurfolgen (control_lane aktiv)
#   Approaching – ueber die Haltelinie fahren (control_intersection aktiv)
#   Turning     – abbiegen
#   Handover    – sanft zurueck in die Spur, bis stabil -> Lane
#
# Steuerung der Control-Nodes ueber das enable/<node>-Schema:
#   /enable/lane          (Bool)   – control_lane aktiv?
#   /enable/intersection  (Bool)   – control_intersection aktiv?
#   /intersection/phase   (String) – Approaching/Turning/Handover (fuer control_intersection + Node B)
#   /intersection/direction (String) – left/right/straight (einmal gewuerfelt)
#
# Kreuzung = rote Haltelinie (detect_lane) UND Tag-Richtung bekannt (detect_apriltag).
# Rote Linie ohne Tag -> ignoriert, Bot faehrt weiter (sicheres Default).
#
# Bei erkannter Kreuzung wird control_lane via enable/lane=False komplett
# stillgelegt; die FSM (control_intersection) uebernimmt Anhalten + Abbiegen.
# ─────────────────────────────────────────────────────────────────────────────

import os
import random
import rospy
from std_msgs.msg import Bool, String, Float64, Int32
import util


class SwitchControlNode:
    # Phasen als einfache String-Konstanten (kein Enum -> passt zum String-Topic)
    LANE        = "Lane"
    APPROACHING = "Approaching"
    TURNING     = "Turning"
    HANDOVER    = "Handover"

    def __init__(self, node_name):
        rospy.init_node(node_name)
        self._vehicle_name = os.environ['VEHICLE_NAME']

        # ── self.*-Defaults VOR init_parameters ───────────────────────────────
        self.phase            = self.LANE
        self.phase_start_time = rospy.Time.now()
        self.allowed_dirs     = []        # erlaubte Richtungen aus letztem Tag
        self.direction        = "straight"
        self.stop_line        = False     # rote Haltelinie (detect_lane)
        self.turn_complete    = False     # Abbiegen fertig (Node B)
        self.lane_error       = 0.0
        self._approach_cleared = False    # Haltelinie beim Approaching ueberfahren?
        self.lane_stable_count = 0

        # Timing-Defaults (aus JSON ueberschrieben)
        self.approaching_min_time  = 0.5
        self.approaching_timeout   = 4.0
        self.approaching_duration  = 1.5     # nur Variante B (zeitgesteuert)
        self.turning_timeout       = 6.0
        self.handover_timeout      = 8.0
        self.lane_stable_threshold = 0.15
        self.lane_stable_required  = 15
        self.turn_time_left        = 3.0     # nur Variante B
        self.turn_time_right       = 2.0
        self.turn_time_straight    = 2.0

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

        # ── Subscriber ────────────────────────────────────────────────────────
        rospy.Subscriber(f'/{self._vehicle_name}/detect/stop_line',
                         Bool, self.cbStopLine, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/detect/apriltag/direction',
                         String, self.cbApriltagDirection, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/intersection/turn_complete',
                         Bool, self.cbTurnComplete, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/detect/lane',
                         Float64, self.cbLane, queue_size=1)

        rospy.loginfo(f"[{node_name}] Bereit – Zustand: Lane")

    # ── Parameter ─────────────────────────────────────────────────────────────

    def cbUpdateParameters(self, parameters):
        t = parameters["timing"]
        self.approaching_min_time = t["approaching_min_time"]["default"]
        self.approaching_timeout  = t["approaching_timeout"]["default"]
        self.approaching_duration = t["approaching_duration"]["default"]
        self.turning_timeout      = t["turning_timeout"]["default"]
        self.handover_timeout     = t["handover_timeout"]["default"]
        tt = parameters["turn_time"]
        self.turn_time_left     = tt["left"]["default"]
        self.turn_time_right    = tt["right"]["default"]
        self.turn_time_straight = tt["straight"]["default"]
        h = parameters["handover"]
        self.lane_stable_threshold = h["lane_stable_threshold"]["default"]
        self.lane_stable_required  = int(h["lane_stable_required"]["default"])

    # ── Sensor-Callbacks ──────────────────────────────────────────────────────

    def cbStopLine(self, msg):
        # 1) Flag fuers distanzbasierte Approaching-Ende
        self.stop_line = msg.data
        # 2) Im Lane-Zustand + Tag-Richtung bekannt -> Kreuzung
        if not (msg.data and self.phase == self.LANE):
            return
        if not self.allowed_dirs or self.allowed_dirs == ["unknown"]:
            rospy.loginfo_throttle(2.0,
                "[switch] Rote Linie ohne Tag-Richtung -> keine Kreuzung, fahre weiter")
            return
        self.direction = random.choice(self.allowed_dirs)
        rospy.loginfo(f"[switch] Kreuzung (Linie+Tag) -> APPROACHING | "
                      f"Richtung: {self.direction} (aus {self.allowed_dirs})")
        self._transition_to(self.APPROACHING)

    def cbApriltagDirection(self, msg):
        # Nur im Lane-Zustand merken (waehrend der Kreuzung nicht ueberschreiben)
        if self.phase == self.LANE and msg.data and msg.data != "unknown":
            self.allowed_dirs = msg.data.split(",")

    def cbTurnComplete(self, msg):
        self.turn_complete = msg.data

    def cbLane(self, msg):
        self.lane_error = msg.data

    # ── Zustandsuebergaenge ─────────────────────────────────────────────────────

    def _transition_to(self, new_phase):
        self.phase            = new_phase
        self.phase_start_time = rospy.Time.now()
        self.lane_stable_count = 0
        if new_phase == self.APPROACHING:
            self._approach_cleared = False
        if new_phase == self.TURNING:
            self.turn_complete = False
        rospy.loginfo(f"[switch] -> {new_phase}")

    def _update_state(self):
        elapsed = (rospy.Time.now() - self.phase_start_time).to_sec()

        if self.phase == self.APPROACHING:
            # ── APPROACHING-ENDE ──────────────────────────────────────────────
            # VARIANTE A (aktiv): distanzbasiert – bis Haltelinie weg + Puffer
            if not self.stop_line:
                self._approach_cleared = True
            cleared = self._approach_cleared and elapsed >= self.approaching_min_time
            if cleared or elapsed >= self.approaching_timeout:
                self._transition_to(self.TURNING)
            # ── VARIANTE B (zeitgesteuert) – stattdessen: ─────────────────────
            # if elapsed >= self.approaching_duration:
            #     self._transition_to(self.TURNING)

        elif self.phase == self.TURNING:
            done = False
            # ── TURNING-ENDE ──────────────────────────────────────────────────
            # VARIANTE A (aktiv): regionsbasiert via detect_red_lane_node
            if self.turn_complete:
                done = True
            elif elapsed >= self.turning_timeout:
                done = True
            # ── VARIANTE B (zeitgesteuert) – stattdessen: ─────────────────────
            # turn_time = self.turn_time_straight
            # if self.direction == "left":  turn_time = self.turn_time_left
            # elif self.direction == "right": turn_time = self.turn_time_right
            # if elapsed >= turn_time:
            #     done = True
            if done:
                self._transition_to(self.HANDOVER)

        elif self.phase == self.HANDOVER:
            # Warten bis Spur stabil zentriert (control_intersection lenkt sanft ein)
            if abs(self.lane_error) < self.lane_stable_threshold:
                self.lane_stable_count += 1
            else:
                self.lane_stable_count = 0
            if self.lane_stable_count >= self.lane_stable_required or elapsed >= self.handover_timeout:
                rospy.loginfo("[switch] Handover fertig -> LANE")
                self.allowed_dirs = []     # fuer naechste Kreuzung zuruecksetzen
                self._transition_to(self.LANE)

    # ── Hauptschleife ──────────────────────────────────────────────────────────

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self._update_state()

            lane_active = (self.phase == self.LANE)
            self.pub_enable_lane.publish(Bool(data=lane_active))
            self.pub_enable_inter.publish(Bool(data=not lane_active))
            self.pub_phase.publish(String(data=self.phase))
            self.pub_direction.publish(String(data=self.direction))

            rate.sleep()


if __name__ == '__main__':
    node = SwitchControlNode('switch_control_node')
    node.run()
    rospy.spin()
