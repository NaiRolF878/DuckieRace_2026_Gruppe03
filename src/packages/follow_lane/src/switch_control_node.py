#!/usr/bin/env python3

# ─────────────────────────────────────────────────────────────────────────────
# switch_control_node.py
#
# Aufgabe: Entscheidet welcher Controller aktiv ist und aktiviert/deaktiviert
#          die entsprechenden Control-Nodes über eigene Enable-Topics.
#
# Statt einen Modus-Integer zu publizieren (den andere Nodes importieren müssen)
# sendet diese Node für jede Control-Node ein eigenes Bool-Topic:
#   /enable/lane         → control_lane_node
#   /enable/intersection → control_intersection_node
#   /enable/obstacle     → control_obstacle_node
#
# Vorteile:
#   - Kein Cross-Package-Import von ControlType mehr nötig
#   - Jede Control-Node abonniert nur ihr eigenes Enable-Topic
#   - switch_control_node ist die einzige Stelle die den Modus kennt
#
# Übergänge:
#   Lane → Intersection : rote Linie erkannt + AprilTag sichtbar
#   Intersection → Lane : control_intersection_node meldet /intersection/done
#   Lane → Obstacle     : Ente erkannt (/detect/duck ≠ -99)
#   Obstacle → Lane     : control_obstacle_node meldet /obstacle/done
# ─────────────────────────────────────────────────────────────────────────────

import rospy
from std_msgs.msg import Float64, Int32, Bool
from enum import Enum
import os


class ControlMode(Enum):
    # Interner Zustand
    # Stattdessen: separate Bool-Topics pro Control-Node
    Lane         = 1
    Obstacle     = 2
    Intersection = 3


class SwitchControlNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        self._vehicle_name = os.environ['VEHICLE_NAME']

        # Startzustand: normales Spurfolgen
        self._mode = ControlMode.Lane

        # ── Publisher: je ein Enable-Topic pro Control-Node ───────────────────
        # Jede Control-Node abonniert nur ihr eigenes Topic 
        self.pub_enable_lane = rospy.Publisher(
            f'/{self._vehicle_name}/enable/lane', Bool, queue_size=1)
        self.pub_enable_intersection = rospy.Publisher(
            f'/{self._vehicle_name}/enable/intersection', Bool, queue_size=1)
        self.pub_enable_obstacle = rospy.Publisher(
            f'/{self._vehicle_name}/enable/obstacle', Bool, queue_size=1)

        # ── Subscriber ─────────────────────────────────────────────────────────

        # Rote Linie + AprilTag → Kreuzungserkennung
        rospy.Subscriber(f'/{self._vehicle_name}/detect/stop_line',
            Bool, self.cbStopLine, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/detect/apriltag',
            Int32, self.cbAprilTag, queue_size=1)

        # Kreuzung abgeschlossen → zurück zu Lane
        rospy.Subscriber(f'/{self._vehicle_name}/intersection/done',
            Bool, self.cbIntersectionDone, queue_size=1)

        # Ente erkannt → Obstacle-Modus
        rospy.Subscriber(f'/{self._vehicle_name}/detect/duck',
            Float64, self.cbDuckDetected, queue_size=1)

        # Ausweichen abgeschlossen → zurück zu Lane
        rospy.Subscriber(f'/{self._vehicle_name}/obstacle/done',
            Bool, self.cbObstacleDone, queue_size=1)

        # ── Zustandsvariablen ─────────────────────────────────────────────────
        self._stop_line_active       = False
        self._current_tag_id         = -1
        self._intersection_triggered = False

        rospy.loginfo(f"[{node_name}] Bereit. Startet im Lane-Modus.")


    # ── Hilfsfunktion: Enable-Topics publishen ────────────────────────────────

    def _publish_enable(self):
        # Genau eine Node ist aktiv – die anderen werden deaktiviert
        self.pub_enable_lane.publish(
            Bool(data=(self._mode == ControlMode.Lane)))
        self.pub_enable_intersection.publish(
            Bool(data=(self._mode == ControlMode.Intersection)))
        self.pub_enable_obstacle.publish(
            Bool(data=(self._mode == ControlMode.Obstacle)))


    # ── Callbacks ─────────────────────────────────────────────────────────────

    def cbStopLine(self, msg):
        self._stop_line_active = msg.data
        self._check_intersection()

    def cbAprilTag(self, msg):
        self._current_tag_id = msg.data
        self._check_intersection()

    def _check_intersection(self):
        # Kreuzung nur auslösen wenn rote Linie UND AprilTag gleichzeitig sichtbar
        # → verhindert Fehlauslösungen durch rote Linie alleine (z.B. Gegenspur)
        if (self._stop_line_active
                and self._current_tag_id != -1
                and self._mode == ControlMode.Lane
                and not self._intersection_triggered):

            rospy.loginfo(
                f"Kreuzung erkannt! Tag-ID={self._current_tag_id} "
                f"→ Intersection-Modus.")
            self._intersection_triggered = True
            self._mode = ControlMode.Intersection

    def cbIntersectionDone(self, msg):
        if msg.data and self._mode == ControlMode.Intersection:
            rospy.loginfo("Kreuzung abgeschlossen → Lane-Modus.")
            self._mode                   = ControlMode.Lane
            self._intersection_triggered = False
            self._stop_line_active       = False
            self._current_tag_id         = -1

    def cbDuckDetected(self, msg):
        # Ente erkannt wenn x-Position ≠ -99
        if msg.data != -99.0 and self._mode == ControlMode.Lane:
            rospy.loginfo(f"Ente erkannt (x={msg.data:.2f}) → Obstacle-Modus.")
            self._mode = ControlMode.Obstacle

    def cbObstacleDone(self, msg):
        if msg.data and self._mode == ControlMode.Obstacle:
            rospy.loginfo("Ausweichen abgeschlossen → Lane-Modus.")
            self._mode = ControlMode.Lane


    # ── Hauptschleife ─────────────────────────────────────────────────────────

    def run(self):
        # Enable-Topics mit 10 Hz publizieren
        # → alle Control-Nodes wissen immer ob sie aktiv sein sollen
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self._publish_enable()
            rate.sleep()


if __name__ == '__main__':
    node = SwitchControlNode(node_name='switch_control_node')
    node.run()
    rospy.spin()
