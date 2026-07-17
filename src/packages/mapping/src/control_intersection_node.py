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
# Segment-Ende wird encoder-basiert bestimmt:
#   Segment-JSON: {"v":..., "omega":..., "ticks":..., "timeout":...}
#   - Geradeaus (|omega| < 0.1): fertig wenn (Δlinks + Δrechts)/2 >= ticks
#   - Drehen: fertig wenn |Δrechts − Δlinks| >= ticks
#   timeout ist ein Sicherheitsnetz falls das Encoder-Ziel nie erreicht wird.
#
# Vorzeichen-Korrektur: Ticks zaehlen laut Hardware IMMER aufwaerts, auch wenn
# sich ein Rad wegen eines starken Differentials rueckwaerts dreht (z.B. bei
# sehr grossem |omega| relativ zu v). Ein einfacher Betrag (delta_links -
# delta_rechts) wuerde die Rueckwaertsdrehung dann faelschlich abziehen statt
# addieren. Deshalb wird die ERWARTETE Drehrichtung jedes Rads separat aus
# v/omega abgeleitet (v_rad = v +- omega*baseline/2) und als Vorzeichen auf
# den jeweiligen Tick-Betrag angewendet, bevor die Segment-Formel greift.
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
                    "v":       float(s["v"]),
                    "omega":   float(s["omega"]),
                    "ticks":   float(s["ticks"]),
                    "timeout": float(s["timeout"]),
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
    def _wheel_signs(v, omega):
        # Erwartete Drehrichtung jedes Rads aus dem Fahrbefehl ableiten
        # (v_rad = v +- omega*baseline/2). Ticks zaehlen immer aufwaerts,
        # unabhaengig von dieser Richtung - das Vorzeichen muss daher separat
        # bestimmt und auf den (immer positiven) Tick-Betrag angewendet werden.
        v_left  = v - omega * WHEEL_BASELINE / 2.0
        v_right = v + omega * WHEEL_BASELINE / 2.0
        return (1.0 if v_left >= 0 else -1.0), (1.0 if v_right >= 0 else -1.0)

    def _segment_cmd(self):
        segments = self.turn_segments.get(self.direction, [])
        if not segments or self._seg_index >= len(segments):
            return 0.0, 0.0, True

        seg = segments[self._seg_index]
        v, omega, ticks, timeout = seg["v"], seg["omega"], seg["ticks"], seg["timeout"]

        # data zaehlt immer aufwaerts -> Betrag der Drehung seit Segment-Start,
        # anschliessend mit dem aus v/omega abgeleiteten Vorzeichen versehen
        delta_left  = abs(self._left_ticks  - self._seg_left_ref)
        delta_right = abs(self._right_ticks - self._seg_right_ref)
        sign_left, sign_right = self._wheel_signs(v, omega)
        signed_left  = sign_left  * delta_left
        signed_right = sign_right * delta_right
        elapsed      = (rospy.Time.now() - self._seg_start_time).to_sec()

        if abs(omega) <= 0.1:
            # Geradeaus-Segment (<=, damit z.B. omega=0.1 fuer "straight" nicht
            # faelschlich als Dreh-Segment mit der Diff-Formel ausgewertet wird)
            progress = (signed_left + signed_right) / 2.0
        elif omega > 0:
            # Linksdrehung: rechtes Rad dreht weiter -> Δrechts > Δlinks
            progress = signed_right - signed_left
        else:
            # Rechtsdrehung: linkes Rad dreht weiter -> Δlinks > Δrechts
            progress = signed_left - signed_right

        target_reached = progress >= ticks
        timed_out       = elapsed >= timeout

        if target_reached or timed_out:
            if timed_out and not target_reached:
                rospy.logwarn(f"[control_intersection] Segment {self._seg_index} "
                              f"({self.direction}) Timeout ({timeout:.1f}s) - "
                              f"Ticks-Ziel nicht erreicht ({progress:.0f}/{ticks:.0f})")
            self._seg_index += 1
            if self._seg_index < len(segments):
                self._reset_segment_reference()
                return self._segment_cmd()
            return 0.0, 0.0, True

        return v, omega, False

    def _compute_cmd(self):
        if self.phase == "Turning":
            v, omega, done = self._segment_cmd()
            return v, omega, done
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
                v, omega, done = self._compute_cmd()
                twist = Twist2DStamped()
                twist.header.stamp = rospy.Time.now()
                twist.v     = v
                twist.omega = omega
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
