#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────────────────────
# control_intersection_node.py  (Challenge 4 – Mapping & Path Finding)
#
# Faehrt die Kreuzung. Aktiv nur wenn /enable/intersection == True.
# Die Phase kommt von switch_control_node ueber /intersection/phase:
#
#   Stopping    – stehen bleiben (v=0)
#   Turning     – Abbiege-SEQUENZ abfahren
#
# Segment-Ende wird encoder-basiert bestimmt, PRO RAD einzeln:
#   Segment-JSON: {"v":..., "ticks_left":..., "ticks_right":..., "timeout":...}
#   fertig wenn |Δlinks| >= |ticks_left| UND |Δrechts| >= |ticks_right|
#   timeout ist ein Sicherheitsnetz falls das Encoder-Ziel nie erreicht wird.
#
# Nur EINE Geschwindigkeit v pro Segment wird vorgegeben. v_left/v_right
# werden daraus intern abgeleitet, proportional zum Verhaeltnis von
# ticks_left zu ticks_right (das Rad mit dem groesseren Ticks-Ziel muss sich
# schneller drehen, damit beide Raeder etwa gleichzeitig fertig werden):
#   v_left  = sign(ticks_left)  * v * 2*|ticks_left|  / (|ticks_left|+|ticks_right|)
#   v_right = sign(ticks_right) * v * 2*|ticks_right| / (|ticks_left|+|ticks_right|)
# Das Vorzeichen von ticks_left/ticks_right legt dabei auch die Fahrtrichtung
# des jeweiligen Rads fest (negativ = rueckwaerts, z.B. bei sehr scharfen
# Kurven). Fuer die Stop-Bedingung zaehlt nur der Betrag, da die Ticks lt.
# Hardware ohnehin immer aufwaerts zaehlen.
#
# car_cmd_switch_node nimmt nur Twist2DStamped (v/omega) entgegen, deshalb
# werden die abgeleiteten v_left/v_right beim Publish ueber Standard-
# Differentialantrieb-Kinematik zurueckgerechnet:
#   v_cmd     = (v_left + v_right) / 2
#   omega_cmd = (v_right - v_left) / WHEEL_BASELINE
# ─────────────────────────────────────────────────────────────────────────────

import os
import rospy
from std_msgs.msg import Bool, String
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped
import util

WHEEL_BASELINE = 0.1  # m, Achsabstand der Raeder (siehe README/Konzept)


class ControlIntersectionNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        self._vehicle_name = os.environ['VEHICLE_NAME']

        self.enable     = False
        self.phase      = "Lane"
        self.direction  = "straight"
        self.turn_segments = {"left": [], "right": [], "straight": []}

        self._last_phase      = "Lane"
        self._turn_done_sent  = False

        # Encoder-Zustand (data zaehlt lt. Hardware IMMER aufwaerts)
        self.resolution     = None
        self._left_ticks    = 0
        self._right_ticks   = 0

        # Segment-Fortschritt
        self._seg_index      = 0
        self._seg_start_time = None
        self._seg_left_ref   = 0
        self._seg_right_ref  = 0

        util.init_parameters(node_name, self.cbUpdateParameters)

        rospy.Subscriber(f'/{self._vehicle_name}/enable/intersection',
                         Bool, self.cbEnable, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/intersection/phase',
                         String, self.cbPhase, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/intersection/direction',
                         String, self.cbDirection, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/left_wheel_encoder_node/tick',
                         WheelEncoderStamped, self.cbLeftEncoder, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/right_wheel_encoder_node/tick',
                         WheelEncoderStamped, self.cbRightEncoder, queue_size=1)

        self.pub_cmd = rospy.Publisher(
            f'/{self._vehicle_name}/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        self.pub_turn_done = rospy.Publisher(
            f'/{self._vehicle_name}/intersection/turn_done', Bool, queue_size=1)

        rospy.on_shutdown(self.fnShutDown)
        rospy.loginfo(f"[{node_name}] Bereit.")

    def cbUpdateParameters(self, parameters):
        seg = parameters.get("turn_segments", {})
        for d in ("left", "right", "straight"):
            self.turn_segments[d] = [
                {
                    "v":           float(s["v"]),
                    "ticks_left":  float(s["ticks_left"]),
                    "ticks_right": float(s["ticks_right"]),
                    "timeout":     float(s["timeout"]),
                }
                for s in seg.get(d, [])
            ]

    def cbEnable(self, msg):
        self.enable = msg.data

    def cbPhase(self, msg):
        self.phase = msg.data

    def cbDirection(self, msg):
        self.direction = msg.data

    def cbLeftEncoder(self, msg):
        if self.resolution is None:
            self.resolution = msg.resolution
            rospy.loginfo(f"[control_intersection] Encoder-Aufloesung: {self.resolution} Ticks/Umdrehung")
        self._left_ticks = msg.data

    def cbRightEncoder(self, msg):
        if self.resolution is None:
            self.resolution = msg.resolution
            rospy.loginfo(f"[control_intersection] Encoder-Aufloesung: {self.resolution} Ticks/Umdrehung")
        self._right_ticks = msg.data

    # ── Segment-Logik ─────────────────────────────────────────────────────────

    def _reset_segment_reference(self):
        # Tick-Referenzwerte bei JEDEM Segment-Start neu setzen
        self._seg_left_ref   = self._left_ticks
        self._seg_right_ref  = self._right_ticks
        self._seg_start_time = rospy.Time.now()

    def _start_turn(self):
        self._seg_index = 0
        self._reset_segment_reference()

    @staticmethod
    def _wheel_speeds(v, ticks_left, ticks_right):
        # v_left/v_right proportional zu |ticks_left|/|ticks_right|, sodass
        # beide Raeder ihr jeweiliges Ticks-Ziel etwa gleichzeitig erreichen.
        # Vorzeichen von ticks_left/ticks_right legt die Fahrtrichtung des
        # jeweiligen Rads fest (negativ = rueckwaerts).
        a, b = abs(ticks_left), abs(ticks_right)
        total = a + b
        if total <= 0:
            return v, v
        sign_left  = 1.0 if ticks_left  >= 0 else -1.0
        sign_right = 1.0 if ticks_right >= 0 else -1.0
        v_left  = sign_left  * v * 2.0 * a / total
        v_right = sign_right * v * 2.0 * b / total
        return v_left, v_right

    def _segment_cmd(self):
        segments = self.turn_segments.get(self.direction, [])
        if not segments or self._seg_index >= len(segments):
            return 0.0, 0.0, True

        seg = segments[self._seg_index]
        v = seg["v"]
        ticks_left, ticks_right, timeout = seg["ticks_left"], seg["ticks_right"], seg["timeout"]

        # data zaehlt immer aufwaerts, unabhaengig von der tatsaechlichen
        # Drehrichtung - fuer die Stop-Bedingung reicht daher der Betrag.
        delta_left  = abs(self._left_ticks  - self._seg_left_ref)
        delta_right = abs(self._right_ticks - self._seg_right_ref)
        elapsed     = (rospy.Time.now() - self._seg_start_time).to_sec()

        target_reached = delta_left >= abs(ticks_left) and delta_right >= abs(ticks_right)
        timed_out       = elapsed >= timeout

        if target_reached or timed_out:
            if timed_out and not target_reached:
                rospy.logwarn(f"[control_intersection] Segment {self._seg_index} "
                              f"({self.direction}) Timeout ({timeout:.1f}s) - "
                              f"Ticks-Ziel nicht erreicht (links {delta_left:.0f}/{abs(ticks_left):.0f}, "
                              f"rechts {delta_right:.0f}/{abs(ticks_right):.0f})")
            self._seg_index += 1
            if self._seg_index < len(segments):
                self._reset_segment_reference()
                return self._segment_cmd()
            return 0.0, 0.0, True

        v_left, v_right = self._wheel_speeds(v, ticks_left, ticks_right)
        return v_left, v_right, False

    def _compute_cmd(self):
        if self.phase == "Turning":
            v_left, v_right, done = self._segment_cmd()
            return v_left, v_right, done
        # Stopping oder Lane -> Motor aus
        return 0.0, 0.0, False

    def fnShutDown(self):
        self.pub_cmd.publish(Twist2DStamped(v=0.0, omega=0.0))

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.phase == "Turning" and self._last_phase != "Turning":
                self._start_turn()
                self._turn_done_sent = False
                rospy.loginfo(f"[control_intersection] Starte Sequenz: {self.direction}")
            self._last_phase = self.phase

            if self.enable:
                v_left, v_right, done = self._compute_cmd()

                # car_cmd_switch_node nimmt nur Twist2DStamped (v/omega) an ->
                # aus den kommandierten Radgeschwindigkeiten zurueckrechnen.
                twist = Twist2DStamped()
                twist.header.stamp = rospy.Time.now()
                twist.v     = (v_left + v_right) / 2.0
                twist.omega = (v_right - v_left) / WHEEL_BASELINE
                self.pub_cmd.publish(twist)

                if self.phase == "Turning" and done and not self._turn_done_sent:
                    self.pub_turn_done.publish(Bool(data=True))
                    self._turn_done_sent = True
                    rospy.loginfo("[control_intersection] Sequenz fertig -> turn_done")
            rate.sleep()


if __name__ == '__main__':
    node = ControlIntersectionNode('control_intersection_node')
    node.run()
    rospy.spin()
