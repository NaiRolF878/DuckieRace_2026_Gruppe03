#!/usr/bin/env python3
"""
ControlLaneNode – PID-Spurfolge-Regler.

Aktiv nur wenn switch_control_node den Zustand Lane (1) publiziert.
Stop-Linien-Logik wurde in switch_control_node / control_intersection_node ausgelagert.
"""

import os
import rospy
from std_msgs.msg import Float64, Int32
from duckietown_msgs.msg import Twist2DStamped
import util
from switch_control_node import ControlType


class ControlLaneNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)

        self.enable        = True
        self._vehicle_name = os.environ['VEHICLE_NAME']
        util.init_parameters(node_name, self.cbUpdateParameters)

        # Publisher
        twist_topic       = f"/{self._vehicle_name}/car_cmd_switch_node/cmd"
        self.pub_cmd_vel  = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)

        # Subscriber
        rospy.Subscriber(f"/{self._vehicle_name}/detect/lane",
                         Float64, self.cbFollowLane, queue_size=1)
        rospy.Subscriber(f"/{self._vehicle_name}/switch/control",
                         Int32, self.cbControl, queue_size=1)

        # PID-Variablen
        self.lastError = 0.0
        self.integral  = 0.0
        self.dt        = 0.1

        # Steuerwerte (werden durch PID berechnet)
        self.v = 0.0
        self.a = 0.0

        rospy.on_shutdown(self.fnShutDown)
        rospy.loginfo(f"[{node_name}] Bereit")

    # ── Parameter ─────────────────────────────────────────────────────────────

    def cbUpdateParameters(self, parameters):
        self.kp      = parameters["pid"]["p"]["default"]
        self.ki      = parameters["pid"]["i"]["default"]
        self.kd      = parameters["pid"]["d"]["default"]
        self.MAX_VEL = parameters["pid"]["max_vel"]["default"]

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def cbControl(self, msg):
        """Nur im Lane-Modus Fahrbefehle senden."""
        self.enable = (msg.data == ControlType.Lane.value)

    def cbFollowLane(self, error_msg):
        """PID auf Spurversatz berechnen."""
        error = error_msg.data

        P = self.kp * error
        self.integral += error * self.dt
        I = self.ki * self.integral
        D = self.kd * (error - self.lastError) / self.dt

        self.a = max(min(P + I + D, 3.0), -3.0)
        self.v = max(0.15, self.MAX_VEL * (1 - abs(error)))

        self.lastError = error
        rospy.logdebug(f"[control_lane] error={error:.3f} v={self.v:.2f} ω={self.a:.2f}")

    # ── Shutdown ──────────────────────────────────────────────────────────────

    def fnShutDown(self):
        rospy.loginfo("[control_lane] Shutdown – stoppe Bot")
        self.pub_cmd_vel.publish(Twist2DStamped(v=0.0, omega=0.0))

    # ── Hauptschleife ─────────────────────────────────────────────────────────

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.enable:
                twist             = Twist2DStamped()
                twist.header.stamp = rospy.Time.now()
                twist.v            = self.v
                twist.omega        = self.a
                self.pub_cmd_vel.publish(twist)
            rate.sleep()


if __name__ == '__main__':
    node = ControlLaneNode('control_lane_node')
    node.run()
    rospy.spin()
