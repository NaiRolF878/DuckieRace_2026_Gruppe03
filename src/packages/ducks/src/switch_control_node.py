#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────────────────────
# switch_control_node.py  (Challenge 3 – Watch out for Ducks)
#
# Schaltet zwischen Lane- und Obstacle-Modus.
#   /enable/lane     → control_lane_node      (fährt IMMER, auch beim Ausweichen)
#   /enable/obstacle → control_obstacle_node  (darf nur im Obstacle-Modus Offset erzeugen)
#
# Warum /enable/lane immer True ist:
#   control_obstacle_node liefert nur einen additiven Lenk-Offset. Die eigentliche
#   Fahrt (PID, Geschwindigkeit, rote Haltelinie) macht control_lane_node. Würde
#   man Lane beim Ausweichen abschalten, bliebe der Bot stehen.
#
# Übergänge:
#   Lane → Obstacle : Zone nah ODER mittel belegt  (/detect/zones)
#   Obstacle → Lane : Ausweichen fertig             (/obstacle/done)
#
# Trigger ist /detect/zones (nicht /detect/duck), damit auch die gelbe Linie
# als Objekt erkannt wird – sie erzeugt keinen Duck-Blob, belegt aber Zonen.
#
# Die rote Haltelinie wird NICHT hier behandelt – sie geht direkt von
# detect_lane_node an control_lane_node (Halte-Automat dort).
#
# ─────────────────────────────────────────────────────────────────────────────

import os
import rospy
from enum import Enum
from std_msgs.msg import Float64, Bool, Float32MultiArray


class ControlMode(Enum):
    Lane     = 1
    Obstacle = 2


class SwitchControlNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        self._vehicle_name = os.environ['VEHICLE_NAME']

        self._mode = ControlMode.Lane

        # ── Publisher ─────────────────────────────────────────────────────────
        self.pub_enable_lane = rospy.Publisher(
            f'/{self._vehicle_name}/enable/lane', Bool, queue_size=1)
        self.pub_enable_obstacle = rospy.Publisher(
            f'/{self._vehicle_name}/enable/obstacle', Bool, queue_size=1)

        # ── Subscriber ────────────────────────────────────────────────────────
        rospy.Subscriber(f'/{self._vehicle_name}/detect/zones',
                         Float32MultiArray, self.cbZones, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/obstacle/done',
                         Bool, self.cbObstacleDone, queue_size=1)

        rospy.loginfo(f"[{node_name}] Bereit – Zustand: Lane")

    # ── Callbacks ───────────────────────────────────────────────────────────────

    def cbZones(self, msg):
        zones = list(msg.data) if len(msg.data) >= 3 else [0.0, 0.0, 0.0]
        nah_oder_mittel = zones[0] > 0.5 or zones[1] > 0.5
        if nah_oder_mittel and self._mode == ControlMode.Lane:
            rospy.loginfo(f"[switch] Zone belegt (nah={zones[0]:.0f} mittel={zones[1]:.0f}) → Obstacle-Modus")
            self._mode = ControlMode.Obstacle

    def cbObstacleDone(self, msg):
        if msg.data and self._mode == ControlMode.Obstacle:
            rospy.loginfo("[switch] Ausweichen abgeschlossen → Lane-Modus")
            self._mode = ControlMode.Lane

    # ── Hauptschleife ──────────────────────────────────────────────────────────

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # control_lane_node ist immer aktiv (siehe Kopfkommentar)
            self.pub_enable_lane.publish(Bool(data=True))
            self.pub_enable_obstacle.publish(
                Bool(data=(self._mode == ControlMode.Obstacle)))
            rate.sleep()


if __name__ == '__main__':
    node = SwitchControlNode('switch_control_node')
    node.run()
    rospy.spin()
