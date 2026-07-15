#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────────────────────
# control_lane_node.py  (Challenge 4 – Mapping & Path Finding)
#
# Reiner PID-Spurfolge-Regler. KEINE Haltelinien-Logik mehr:
# An der Kreuzung übernimmt die FSM (switch_control_node + control_intersection_node).
# Diese Node wird dann über /enable/lane = False stillgelegt.
#
# Aktiv nur wenn /enable/lane == True.
# ─────────────────────────────────────────────────────────────────────────────

import os
import rospy
from std_msgs.msg import Float64, Bool
from duckietown_msgs.msg import Twist2DStamped
import util


class ControlLaneNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)

        # Steuerung aktiv? Wird durch switch_control_node über /enable/lane gesetzt
        self.enable = True

        self._vehicle_name = os.environ['VEHICLE_NAME']

        # Parameter aus JSON laden + Live-Update Callback registrieren
        util.init_parameters(node_name, self.cbUpdateParameters)

        # Publisher für Fahrbefehle
        twist_topic = f"/{self._vehicle_name}/car_cmd_switch_node/cmd"
        self.pub_cmd_vel = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)

        # Subscriber: Spurversatz von detect_lane_node
        detect_lane_topic = f"/{self._vehicle_name}/detect/lane"
        self.sub_lane = rospy.Subscriber(detect_lane_topic, Float64, self.cbFollowLane, queue_size=1)

        # Subscriber: Enable-Signal von switch_control_node
        self.sub_enable = rospy.Subscriber(
            f'/{self._vehicle_name}/enable/lane', Bool, self.cbControl, queue_size=1)

        # PID Variablen
        self.lastError = 0
        self.integral  = 0
        self.dt        = 0.1
        # Anti-Windup: begrenzt den aufsummierten Fehler
        self.INTEGRAL_LIMIT = 3.0

        # Steuerwerte
        self.v = 0
        self.a = 0

        rospy.on_shutdown(self.fnShutDown)
        rospy.loginfo(f"[{node_name}] Bereit. Warte auf Spurversatz ...")

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def cbControl(self, msg):
        # True = Lane Following aktiv, False = FSM (Kreuzung) übernimmt
        self.enable = msg.data

    def cbUpdateParameters(self, parameters):
        self.kp      = parameters["pid"]["p"]["default"]
        self.ki      = parameters["pid"]["i"]["default"]
        self.kd      = parameters["pid"]["d"]["default"]
        self.MAX_VEL = parameters["pid"]["max_vel"]["default"]
        self.MIN_VEL = parameters["pid"]["min_vel"]["default"]

    # Spurversatz error im Bereich [-1, +1]:
    # error > 0 → Bot zu weit links  → nach rechts lenken
    # error < 0 → Bot zu weit rechts → nach links lenken
    def cbFollowLane(self, error):
        error = error.data

        # Begrenzung damit PID nicht übersteuert
        error = max(min(error, 2.0), -2.0)

        # P-Anteil
        P = self.kp * error

        # I-Anteil (mit Anti-Windup)
        self.integral += error * self.dt
        self.integral = max(min(self.integral, self.INTEGRAL_LIMIT), -self.INTEGRAL_LIMIT)
        I = self.ki * self.integral

        # D-Anteil
        derivative = (error - self.lastError) / self.dt
        D = self.kd * derivative

        # Gesamtlenkung, begrenzt auf [-3, +3] rad/s
        self.a = max(min(P + I + D, 3), -3)

        # Geschwindigkeit abhängig vom Fehler; MIN_VEL verhindert komplettes Stoppen
        self.v = max(self.MIN_VEL, self.MAX_VEL * (1 - abs(error)))

        self.lastError = error

    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")
        self.pub_cmd_vel.publish(Twist2DStamped(v=0.0, omega=0.0))

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.enable:
                twist = Twist2DStamped()
                twist.header.stamp = rospy.Time.now()
                twist.v     = self.v
                twist.omega = self.a
                self.pub_cmd_vel.publish(twist)
            rate.sleep()


if __name__ == '__main__':
    node = ControlLaneNode('control_lane_node')
    node.run()
    rospy.spin()
