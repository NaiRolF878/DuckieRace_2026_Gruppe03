#!/usr/bin/env python3

# ─────────────────────────────────────────────────────────────────────────────
# switch_control_node.py
#
# Aufgabe: Aktiviert control_lane_node über das /enable/lane Topic.
#
# Für Challenge 1 (Lane Following) ist nur eine Control-Node aktiv.
# Diese Node existiert als Scaffold – sie kann später erweitert werden um
# zwischen mehreren Modi umzuschalten (Intersection, Obstacle Avoidance, etc.).
#
# Aktuell: publiziert /enable/lane = True mit 10 Hz.
# control_lane_node abonniert dieses Topic und fährt nur wenn enable=True.
# ─────────────────────────────────────────────────────────────────────────────

import os
import rospy
from std_msgs.msg import Bool


class SwitchControlNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        self._vehicle_name = os.environ['VEHICLE_NAME']

        # Enable-Topic für control_lane_node
        self.pub_enable_lane = rospy.Publisher(
            f'/{self._vehicle_name}/enable/lane', Bool, queue_size=1)

        rospy.loginfo(f"[{node_name}] Bereit. Lane-Modus dauerhaft aktiv.")


    def run(self):
        # Enable-Topic mit 10 Hz publizieren
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.pub_enable_lane.publish(Bool(data=True))
            rate.sleep()


if __name__ == '__main__':
    node = SwitchControlNode(node_name='switch_control_node')
    node.run()
    rospy.spin()
