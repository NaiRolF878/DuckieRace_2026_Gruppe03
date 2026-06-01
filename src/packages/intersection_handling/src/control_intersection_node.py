#!/usr/bin/env python3
"""
ControlIntersectionNode – Motorbefehle für die 3 Kreuzungsphasen.

Aktiv bei ControlType: Approaching (3), Turning (4), LaneHandover (5).
Bei ControlType.Lane (1) wird nichts publiziert – control_lane_node übernimmt.

Phase APPROACHING:
  Fährt mit fester Geschwindigkeit geradeaus über die rote Linie.
  Dauer wird durch switch_control_node gesteuert (Timer dort).

Phase TURNING:
  left:     omega > 0  (links drehen) bis red_line_side == "right"
  right:    omega < 0  (rechts drehen) bis red_line_side == "left"
  straight: omega = 0  (geradeaus) bis red_line_side == "none"

Phase LANE_HANDOVER:
  Fährt langsam mit sanfter Lenkung aus detect_lane_node,
  bis switch_control_node wieder auf Lane umschaltet.
"""

import os
import rospy
from std_msgs.msg import Float64, Int32, String
from duckietown_msgs.msg import Twist2DStamped
import util

# Import ControlType aus switch_control_node (gleicher Package-Pfad)
from switch_control_node import ControlType


class ControlIntersectionNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        self._vehicle_name = os.environ['VEHICLE_NAME']
        util.init_parameters(node_name, self.cbUpdateParameters)

        # ── Subscriber ────────────────────────────────────────────────────────
        rospy.Subscriber(f'/{self._vehicle_name}/switch/control',
                         Int32, self.cbControl, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/intersection/direction',
                         String, self.cbDirection, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/detect/red_line_side',
                         String, self.cbRedLineSide, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/detect/lane',
                         Float64, self.cbLane, queue_size=1)

        # ── Publisher ─────────────────────────────────────────────────────────
        self.pub_cmd = rospy.Publisher(
            f'/{self._vehicle_name}/car_cmd_switch_node/cmd',
            Twist2DStamped, queue_size=1)

        # ── Zustandsvariablen ─────────────────────────────────────────────────
        self.control_mode  = ControlType.Lane
        self.direction     = "straight"
        self.red_line_side = "none"
        self.lane_error    = 0.0

        # Parameter-Defaults (werden durch cbUpdateParameters überschrieben)
        self.approaching_speed   = 0.20
        self.turning_omega_left  =  1.5   # rad/s nach links
        self.turning_omega_right = -1.5   # rad/s nach rechts
        self.turning_speed       = 0.10   # Vorwärts-Komponente beim Drehen
        self.straight_speed      = 0.20
        self.handover_speed      = 0.15
        self.handover_kp         = 3.0    # sanfter P-Regler für Spur-Übergabe

        rospy.on_shutdown(self._stop)
        rospy.loginfo(f"[{node_name}] Bereit")

    # ── Parameter ─────────────────────────────────────────────────────────────

    def cbUpdateParameters(self, parameters):
        a = parameters["approaching"]
        self.approaching_speed   = a["speed"]["default"]

        t = parameters["turning"]
        self.turning_omega_left  =  t["omega"]["default"]
        self.turning_omega_right = -t["omega"]["default"]
        self.turning_speed       =  t["speed"]["default"]
        self.straight_speed      =  t["straight_speed"]["default"]

        h = parameters["handover"]
        self.handover_speed = h["speed"]["default"]
        self.handover_kp    = h["kp"]["default"]

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def cbControl(self, msg):
        try:
            self.control_mode = ControlType(msg.data)
        except ValueError:
            rospy.logwarn(f"[intersection_ctrl] Unbekannter ControlType: {msg.data}")

    def cbDirection(self, msg):
        self.direction = msg.data

    def cbRedLineSide(self, msg):
        self.red_line_side = msg.data

    def cbLane(self, msg):
        self.lane_error = msg.data

    # ── Motorsignale ──────────────────────────────────────────────────────────

    def _publish(self, v, omega):
        twist       = Twist2DStamped()
        twist.header.stamp = rospy.Time.now()
        twist.v     = v
        twist.omega = omega
        self.pub_cmd.publish(twist)

    def _stop(self):
        self._publish(0.0, 0.0)

    def _compute_command(self):
        """Berechnet v und omega für die aktuelle Kreuzungsphase."""

        if self.control_mode == ControlType.Approaching:
            # Geradeaus über die rote Linie fahren
            return self.approaching_speed, 0.0

        elif self.control_mode == ControlType.Turning:
            if self.direction == "left":
                # Links drehen: positives omega
                return self.turning_speed, self.turning_omega_left
            elif self.direction == "right":
                # Rechts drehen: negatives omega
                return self.turning_speed, self.turning_omega_right
            else:  # "straight"
                # Geradeaus durch die Kreuzung
                return self.straight_speed, 0.0

        elif self.control_mode == ControlType.LaneHandover:
            # Sanfter P-Regler auf Spurmitte; Fehler aus detect_lane_node
            omega = self.handover_kp * self.lane_error
            omega = max(min(omega, 2.0), -2.0)   # Sicherheitsbegrenzung
            return self.handover_speed, omega

        # Lane-Modus → nichts tun (control_lane_node übernimmt)
        return None, None

    # ── Hauptschleife ─────────────────────────────────────────────────────────

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            v, omega = self._compute_command()
            if v is not None:
                self._publish(v, omega)
                rospy.logdebug(f"[intersection_ctrl] {self.control_mode.name} "
                               f"v={v:.2f} ω={omega:.2f} "
                               f"dir={self.direction} red={self.red_line_side}")
            rate.sleep()


if __name__ == '__main__':
    node = ControlIntersectionNode('control_intersection_node')
    node.run()
    rospy.spin()
