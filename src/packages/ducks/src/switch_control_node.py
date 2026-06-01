#!/usr/bin/env python3

# ─────────────────────────────────────────────────────────────────────────────
# switch_control_node.py
#
# Aufgabe (Challenge 3): Schaltet zwischen Lane- und Obstacle-Modus um und
#          aktiviert die jeweilige Control-Node über ihr Enable-Topic.
#
#   /enable/lane     → control_lane_node      (immer aktiv außer beim Ausweichen?)
#   /enable/obstacle → control_obstacle_node  (nur im Obstacle-Modus)
#
# WICHTIG zur Architektur:
#   control_lane_node fährt IMMER (PID + Stop-Line). control_obstacle_node liefert
#   nur einen additiven Offset. Daher bleibt /enable/lane auch im Obstacle-Modus
#   True – sonst stünde der Bot beim Ausweichen still. /enable/obstacle schaltet
#   lediglich, ob die Obstacle-Node aktiv einen Offset erzeugen darf.
#
# Übergänge:
#   Lane → Obstacle : Ente erkannt        (/detect/duck ≠ -99)
#   Obstacle → Lane : Ausweichen fertig   (/obstacle/done)
# ─────────────────────────────────────────────────────────────────────────────

import os
import rospy
from enum import Enum
from std_msgs.msg import Float64, Bool


class ControlMode(Enum):
    Lane     = 1
    Obstacle = 2


class SwitchControlNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        self._vehicle_name = os.environ['VEHICLE_NAME']

        self._mode = ControlMode.Lane

        # ── Publisher: Enable-Topics ──────────────────────────────────────────
        self.pub_enable_lane = rospy.Publisher(
            f'/{self._vehicle_name}/enable/lane', Bool, queue_size=1)
        self.pub_enable_obstacle = rospy.Publisher(
            f'/{self._vehicle_name}/enable/obstacle', Bool, queue_size=1)

        # ── Subscriber ────────────────────────────────────────────────────────
        rospy.Subscriber(f'/{self._vehicle_name}/detect/duck',
            Float64, self.cbDuckDetected, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/obstacle/done',
            Bool, self.cbObstacleDone, queue_size=1)

        rospy.loginfo(f"[{node_name}] Bereit. Startet im Lane-Modus.")


    def _publish_enable(self):
        # control_lane_node fährt immer (auch beim Ausweichen, da Offset additiv).
        self.pub_enable_lane.publish(Bool(data=True))
        # control_obstacle_node nur im Obstacle-Modus aktiv.
        self.pub_enable_obstacle.publish(
            Bool(data=(self._mode == ControlMode.Obstacle)))


    # ── Callbacks ───────────────────────────────────────────────────────────────

    def cbDuckDetected(self, msg):
        if msg.data != -99.0 and self._mode == ControlMode.Lane:
            rospy.loginfo(f"Ente erkannt (x={msg.data:.2f}) → Obstacle-Modus.")
            self._mode = ControlMode.Obstacle

    def cbObstacleDone(self, msg):
        if msg.data and self._mode == ControlMode.Obstacle:
            rospy.loginfo("Ausweichen abgeschlossen → Lane-Modus.")
            self._mode = ControlMode.Lane


    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self._publish_enable()
            rate.sleep()


if __name__ == '__main__':
    node = SwitchControlNode(node_name='switch_control_node')
    node.run()
    rospy.spin()
