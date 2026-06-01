#!/usr/bin/env python3
"""
SwitchControlNode – Zentraler Zustandsautomat für Kreuzungshandling.

Zustände (ControlType):
  Lane        (1) – normales Spurfolgen (control_lane_node aktiv)
  Approaching (3) – Kreuzung erkannt, rote Linie überfahren
  Turning     (4) – in die gewählte Richtung abbiegen
  LaneHandover(5) – auf neue Spur einscheren
  Done            – intern; sofortiger Übergang zu Lane

Übergänge:
  Lane → Approaching   : /detect/intersection == True
  Approaching → Turning: APPROACHING_DURATION abgelaufen
  Turning → LaneHandover: Zielkriterium (rote Linie auf Gegenseite)
                          ODER Timeout
  LaneHandover → Lane  : Spur stabil erkannt ODER Timeout

Publiziert:
  /{vehicle}/switch/control        (Int32)  – aktueller Zustand
  /{vehicle}/intersection/direction(String) – gewählte Richtung (left/right/straight)
"""

import os
import random
import rospy
from std_msgs.msg import Float64, Int32, String, Bool
from enum import Enum
import util


class ControlType(Enum):
    Lane         = 1
    Approaching  = 3
    Turning      = 4
    LaneHandover = 5


class SwitchControlNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        self._vehicle_name = os.environ['VEHICLE_NAME']
        util.init_parameters(node_name, self.cbUpdateParameters)

        # ── Subscriber ────────────────────────────────────────────────────────
        rospy.Subscriber(f'/{self._vehicle_name}/detect/intersection',
                         Bool, self.cbIntersection, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/detect/apriltag/direction',
                         String, self.cbApriltagDirection, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/detect/red_line_side',
                         String, self.cbRedLineSide, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/detect/lane',
                         Float64, self.cbLane, queue_size=1)

        # ── Publisher ─────────────────────────────────────────────────────────
        self.pub_control   = rospy.Publisher(f'/{self._vehicle_name}/switch/control',
                                             Int32, queue_size=1)
        self.pub_direction = rospy.Publisher(f'/{self._vehicle_name}/intersection/direction',
                                             String, queue_size=1)

        # ── Zustandsvariablen ─────────────────────────────────────────────────
        self.phase            = ControlType.Lane
        self.phase_start_time = rospy.Time.now()

        # Gewählte Abbiegerichtung (wird in Lane-Phase zufällig gezogen)
        self.direction        = "straight"
        # Letzte bekannte erlaubte Richtungen vom AprilTag
        self.allowed_dirs     = ["straight"]

        # Sensor-Inputs (thread-safe durch GIL bei einfachen Typen)
        self.red_line_side    = "none"   # "none" | "left" | "center" | "right"
        self.lane_error       = 0.0      # [-1, +1]

        # Zähler für stabile Spur-Erkennung in LaneHandover
        self.lane_stable_count = 0

        # Parameter-Defaults (werden durch cbUpdateParameters überschrieben)
        self.approaching_duration  = 1.5
        self.turning_timeout       = 6.0
        self.handover_timeout      = 8.0
        self.lane_stable_threshold = 0.15
        self.lane_stable_required  = 15    # Frames bei 10 Hz = 1,5 s

        rospy.loginfo(f"[{node_name}] Bereit – Zustand: Lane")

    # ── Parameter-Callback ────────────────────────────────────────────────────

    def cbUpdateParameters(self, parameters):
        self.approaching_duration  = parameters["timing"]["approaching_duration"]["default"]
        self.turning_timeout       = parameters["timing"]["turning_timeout"]["default"]
        self.handover_timeout      = parameters["timing"]["handover_timeout"]["default"]
        self.lane_stable_threshold = parameters["handover"]["lane_stable_threshold"]["default"]
        self.lane_stable_required  = int(parameters["handover"]["lane_stable_required"]["default"])

    # ── Sensor-Callbacks ──────────────────────────────────────────────────────

    def cbIntersection(self, msg):
        """Kreuzung erkannt (rote Linie + AprilTag gleichzeitig sichtbar)."""
        if msg.data and self.phase == ControlType.Lane:
            # Richtung zufällig aus erlaubten Optionen wählen
            self.direction = random.choice(self.allowed_dirs)
            rospy.loginfo(f"[switch] Kreuzung erkannt → APPROACHING | Richtung: {self.direction}")
            self._transition_to(ControlType.Approaching)

    def cbApriltagDirection(self, msg):
        """Erlaubte Richtungen vom AprilTag empfangen (nur in Lane-Phase speichern)."""
        if self.phase == ControlType.Lane and msg.data and msg.data != "unknown":
            self.allowed_dirs = msg.data.split(",")

    def cbRedLineSide(self, msg):
        self.red_line_side = msg.data

    def cbLane(self, msg):
        self.lane_error = msg.data

    # ── Zustandsübergänge ─────────────────────────────────────────────────────

    def _transition_to(self, new_phase):
        self.phase            = new_phase
        self.phase_start_time = rospy.Time.now()
        self.lane_stable_count = 0
        rospy.loginfo(f"[switch] → {new_phase.name}")

    def _update_state(self):
        elapsed = (rospy.Time.now() - self.phase_start_time).to_sec()

        if self.phase == ControlType.Approaching:
            # Einfach Zeit abwarten – control_intersection_node fährt gerade drüber
            if elapsed >= self.approaching_duration:
                rospy.loginfo(f"[switch] Approaching abgeschlossen → TURNING ({self.direction})")
                self._transition_to(ControlType.Turning)

        elif self.phase == ControlType.Turning:
            done = False
            reason = ""

            if self.direction == "left"     and self.red_line_side == "right":
                done, reason = True, "rote Linie rechts sichtbar"
            elif self.direction == "right"  and self.red_line_side == "left":
                done, reason = True, "rote Linie links sichtbar"
            elif self.direction == "straight" and self.red_line_side == "none":
                done, reason = True, "rote Linie verschwunden"
            elif elapsed >= self.turning_timeout:
                done, reason = True, f"Timeout ({self.turning_timeout:.1f}s)"

            if done:
                rospy.loginfo(f"[switch] Turning abgeschlossen ({reason}) → LANE_HANDOVER")
                self._transition_to(ControlType.LaneHandover)

        elif self.phase == ControlType.LaneHandover:
            # Spur stabil, wenn Fehler lange genug unter Schwelle
            if abs(self.lane_error) < self.lane_stable_threshold:
                self.lane_stable_count += 1
            else:
                self.lane_stable_count = 0

            lane_ok  = self.lane_stable_count >= self.lane_stable_required
            timed_out = elapsed >= self.handover_timeout

            if lane_ok or timed_out:
                reason = "Spur stabil" if lane_ok else f"Timeout ({self.handover_timeout:.1f}s)"
                rospy.loginfo(f"[switch] LaneHandover abgeschlossen ({reason}) → LANE")
                self._transition_to(ControlType.Lane)

    # ── Hauptschleife ─────────────────────────────────────────────────────────

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self._update_state()

            self.pub_control.publish(Int32(data=self.phase.value))
            self.pub_direction.publish(String(data=self.direction))

            rospy.logdebug(f"[switch] Phase={self.phase.name} | dir={self.direction} "
                           f"| red_side={self.red_line_side} | lane_err={self.lane_error:.3f}")
            rate.sleep()


if __name__ == '__main__':
    node = SwitchControlNode('switch_control_node')
    node.run()
    rospy.spin()
