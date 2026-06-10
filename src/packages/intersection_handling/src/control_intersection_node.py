#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────────────────────
# control_intersection_node.py  (Challenge 2 – Intersection Handling)
#
# Faehrt die Kreuzung. Aktiv nur wenn /enable/intersection == True.
# Die Phase kommt von switch_control_node ueber /intersection/phase:
#
#   Approaching – geradeaus ueber die Haltelinie
#   Turning     – abbiegen (omega je Richtung); Ende bestimmt switch_control_node
#   Turning     – abbiegen (omega je Richtung); danach direkt zurueck zu Lane
#
# Publiziert car_cmd_switch_node/cmd.
# ─────────────────────────────────────────────────────────────────────────────

import os
import rospy
from std_msgs.msg import Bool, String
from duckietown_msgs.msg import Twist2DStamped
import util


class ControlIntersectionNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        self._vehicle_name = os.environ['VEHICLE_NAME']

        # ── Defaults VOR init_parameters ──────────────────────────────────────
        self.enable     = False
        self.phase      = "Lane"
        self.direction  = "straight"
        # Param-Defaults
        self.app_speed     = 0.3
        self.turn_speed    = 0.2
        self.turn_omega    = 4.0
        self.straight_speed = 0.3

        util.init_parameters(node_name, self.cbUpdateParameters)

        # ── Subscriber ────────────────────────────────────────────────────────
        rospy.Subscriber(f'/{self._vehicle_name}/enable/intersection',
                         Bool, self.cbEnable, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/intersection/phase',
                         String, self.cbPhase, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/intersection/direction',
                         String, self.cbDirection, queue_size=1)

        # ── Publisher ─────────────────────────────────────────────────────────
        self.pub_cmd = rospy.Publisher(
            f'/{self._vehicle_name}/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)

        rospy.on_shutdown(self.fnShutDown)
        rospy.loginfo(f"[{node_name}] Bereit.")

    # ── Parameter ─────────────────────────────────────────────────────────────

    def cbUpdateParameters(self, parameters):
        a = parameters["approaching"]
        self.app_speed = a["speed"]["default"]
        t = parameters["turning"]
        self.turn_speed     = t["speed"]["default"]
        self.turn_omega     = t["omega"]["default"]
        self.straight_speed = t["straight_speed"]["default"]

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def cbEnable(self, msg):
        self.enable = msg.data

    def cbPhase(self, msg):
        self.phase = msg.data

    def cbDirection(self, msg):
        self.direction = msg.data

    # ── Phasen-Steuerung ────────────────────────────────────────────────────────

    def _compute_cmd(self):
        v, omega = 0.0, 0.0
        if self.phase == "Stopping":
            # An der Haltelinie stehen bleiben
            v, omega = 0.0, 0.0

        elif self.phase == "PreTurnPause":
            # Debug-Pause vor dem Drehen: stehen bleiben
            v, omega = 0.0, 0.0

        elif self.phase == "PostTurnPause":
            # Debug-Pause nach dem Drehen: stehen bleiben
            v, omega = 0.0, 0.0

        elif self.phase == "Approaching":
            v, omega = self.app_speed, 0.0

        elif self.phase == "ExitStraight":
            # Nach dem Drehen geradeaus, bis die FSM zu Lane schaltet
            v, omega = self.straight_speed, 0.0

        elif self.phase == "Turning":
            if self.direction == "left":
                v, omega = self.turn_speed, +self.turn_omega
            elif self.direction == "right":
                v, omega = self.turn_speed, -self.turn_omega
            else:  # straight
                v, omega = self.straight_speed, 0.0

        return v, omega

    def fnShutDown(self):
        self.pub_cmd.publish(Twist2DStamped(v=0.0, omega=0.0))

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.enable:
                v, omega = self._compute_cmd()
                twist = Twist2DStamped()
                twist.header.stamp = rospy.Time.now()
                twist.v     = v
                twist.omega = omega
                self.pub_cmd.publish(twist)
            rate.sleep()


if __name__ == '__main__':
    node = ControlIntersectionNode('control_intersection_node')
    node.run()
    rospy.spin()
