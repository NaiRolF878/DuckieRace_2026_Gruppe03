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
from std_msgs.msg import Bool, String
import util


class SwitchControlNode:
    # Phasen als einfache String-Konstanten (kein Enum -> passt zum String-Topic)
    LANE        = "Lane"
    STOPPING    = "Stopping"
    APPROACHING = "Approaching"
    PRETURN     = "PreTurnPause"
    TURNING     = "Turning"

    def __init__(self, node_name):
        rospy.init_node(node_name)
        self._vehicle_name = os.environ['VEHICLE_NAME']

        # ── self.*-Defaults VOR init_parameters ───────────────────────────────
        self.phase            = self.LANE
        self.phase_start_time = rospy.Time.now()
        self.allowed_dirs     = []        # erlaubte Richtungen aus letztem Tag
        self.direction        = "straight"
        self.stop_line        = False     # rote Haltelinie (detect_lane)
        self._approach_cleared = False    # Haltelinie beim Approaching ueberfahren?

        # Timing-Defaults (aus JSON ueberschrieben)
        self.stop_duration         = 2.0
        # Debug-Pause zwischen Approaching und Turning (0 = aus)
        self.pre_turn_pause        = 0.0
        # approaching_min_time richtungsabhaengig: wie weit faehrt der Bot in die
        # Kreuzung, bevor er dreht. Rechts kuerzer (engere Kurve), links laenger.
        self.approach_min_left     = 1.2
        self.approach_min_right    = 0.5
        self.approach_min_straight = 0.5
        self.approaching_timeout   = 4.0
        self.turn_time_left        = 3.0     # zeitgesteuertes Abbiegen
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

        rospy.loginfo(f"[{node_name}] Bereit – Zustand: Lane")

    # ── Parameter ─────────────────────────────────────────────────────────────

    def cbUpdateParameters(self, parameters):
        t = parameters["timing"]
        self.stop_duration        = t["stop_duration"]["default"]
        self.pre_turn_pause       = t["pre_turn_pause"]["default"]
        self.approaching_timeout  = t["approaching_timeout"]["default"]
        am = parameters["approach_min_time"]
        self.approach_min_left     = am["left"]["default"]
        self.approach_min_right    = am["right"]["default"]
        self.approach_min_straight = am["straight"]["default"]
        tt = parameters["turn_time"]
        self.turn_time_left     = tt["left"]["default"]
        self.turn_time_right    = tt["right"]["default"]
        self.turn_time_straight = tt["straight"]["default"]

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
        rospy.loginfo(f"[switch] Kreuzung (Linie+Tag) -> STOPPING | "
                      f"Richtung: {self.direction} (aus {self.allowed_dirs})")
        self._transition_to(self.STOPPING)

    def cbApriltagDirection(self, msg):
        # Nur im Lane-Zustand merken (waehrend der Kreuzung nicht ueberschreiben)
        if self.phase == self.LANE and msg.data and msg.data != "unknown":
            self.allowed_dirs = msg.data.split(",")

    # ── Zustandsuebergaenge ─────────────────────────────────────────────────────

    def _transition_to(self, new_phase):
        self.phase            = new_phase
        self.phase_start_time = rospy.Time.now()
        if new_phase == self.APPROACHING:
            self._approach_cleared = False
        rospy.loginfo(f"[switch] -> {new_phase}")

    def _update_state(self):
        elapsed = (rospy.Time.now() - self.phase_start_time).to_sec()

        if self.phase == self.STOPPING:
            # An der Haltelinie stehen bleiben (control_intersection haelt v=0),
            # bis die Stopp-Dauer abgelaufen ist -> dann ueber die Linie fahren.
            if elapsed >= self.stop_duration:
                rospy.loginfo(f"[switch] Stopp beendet ({self.stop_duration:.1f}s) -> APPROACHING")
                self._transition_to(self.APPROACHING)

        elif self.phase == self.APPROACHING:
            # ── APPROACHING-ENDE ──────────────────────────────────────────────
            # VARIANTE A (aktiv): distanzbasiert – bis Haltelinie weg + Puffer
            if not self.stop_line:
                self._approach_cleared = True
            # Richtungsabhaengige Mindest-Fahrzeit in die Kreuzung
            if self.direction == "left":
                min_time = self.approach_min_left
            elif self.direction == "right":
                min_time = self.approach_min_right
            else:
                min_time = self.approach_min_straight
            cleared = self._approach_cleared and elapsed >= min_time
            if cleared or elapsed >= self.approaching_timeout:
                # Wenn Debug-Pause aktiv (>0), erst in PreTurnPause, sonst direkt drehen
                if self.pre_turn_pause > 0:
                    self._transition_to(self.PRETURN)
                else:
                    self._transition_to(self.TURNING)
            # ── VARIANTE B (zeitgesteuert) – stattdessen: ─────────────────────
            # if elapsed >= self.approaching_duration:
            #     self._transition_to(self.TURNING)

        elif self.phase == self.PRETURN:
            # Debug-Pause: Bot steht still, damit man die Position vor dem Drehen
            # sieht. pre_turn_pause auf 0 setzen, um diese Phase zu ueberspringen.
            if elapsed >= self.pre_turn_pause:
                rospy.loginfo(f"[switch] Pre-Turn-Pause beendet -> TURNING ({self.direction})")
                self._transition_to(self.TURNING)

        elif self.phase == self.TURNING:
            # ── TURNING-ENDE ──────────────────────────────────────────────────
            # Zeitgesteuert (Hardcode): feste Drehzeit pro Richtung.
            turn_time = self.turn_time_straight
            if self.direction == "left":
                turn_time = self.turn_time_left
            elif self.direction == "right":
                turn_time = self.turn_time_right
            if elapsed >= turn_time:
                # Direkt zurueck zu Lane: der control_lane-PID faengt die Spur.
                rospy.loginfo("[switch] Turning fertig -> LANE")
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
