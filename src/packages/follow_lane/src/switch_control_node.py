#!/usr/bin/env python3

# ─────────────────────────────────────────────────────────────────────────────
# switch_control_node.py
#
# Aufgabe: Entscheidet welcher Controller aktiv ist und publiziert den Modus
#          kontinuierlich an alle Control-Nodes.
#
# Modi:
#   Lane         → normales Spurfolgen (control_lane_node)
#   Intersection → Kreuzungsdurchfahrt (control_intersection_node)
#   Obstacle     → Hindernisumfahrung  (control_obstacle_node, Challenge 3)
#
# Übergänge:
#   Lane → Intersection: rote Linie erkannt + AprilTag sichtbar
#   Intersection → Lane: control_intersection_node meldet /intersection/done
#   Lane → Obstacle:     Ente erkannt (Challenge 3, noch nicht implementiert)
#   Obstacle → Lane:     Ente weg     (Challenge 3, noch nicht implementiert)
# ─────────────────────────────────────────────────────────────────────────────

import rospy
from std_msgs.msg import Float64, Int32, Bool
from enum import Enum
import os


class ControlType(Enum):
    Lane         = 1  # normales Spurfolgen
    Obstacle     = 2  # Hindernisumfahrung (Challenge 3)
    Intersection = 3  # Kreuzungsdurchfahrt (Challenge 2)


class SwitchControlNode:
    def __init__(self, node_name):
        # ROS-Node initialisieren
        rospy.init_node(node_name)

        # Fahrzeugnamen aus Umgebungsvariable lesen
        self._vehicle_name = os.environ['VEHICLE_NAME']

        # Startzustand: normales Spurfolgen
        self._control_mode = ControlType.Lane

        # ── Publisher ──────────────────────────────────────────────────────────
        # Publiziert den aktiven Modus kontinuierlich an alle Control-Nodes
        self.pub_control = rospy.Publisher(
            f'/{self._vehicle_name}/switch/control',
            Int32,
            queue_size=1
        )

        # ── Subscriber ─────────────────────────────────────────────────────────

        # Rote Linie erkannt (von detect_lane_node)
        self.sub_stop_line = rospy.Subscriber(
            f'/{self._vehicle_name}/detect/stop_line',
            Bool,
            self.cbStopLine,
            queue_size=1
        )

        # AprilTag erkannt (von detect_apriltag_node)
        # → wird zusammen mit rote Linie für Intersection-Erkennung genutzt
        self.sub_apriltag = rospy.Subscriber(
            f'/{self._vehicle_name}/detect/apriltag',
            Int32,
            self.cbAprilTag,
            queue_size=1
        )

        # Kreuzung abgeschlossen (von control_intersection_node)
        # → schaltet zurück auf Lane Following
        self.sub_intersection_done = rospy.Subscriber(
            f'/{self._vehicle_name}/intersection/done',
            Bool,
            self.cbIntersectionDone,
            queue_size=1
        )

        # Ente erkannt (von detect_duck_node, Challenge 3)
        # Enten-Erkennung: x-Position von detect_duck_node
        self.sub_duckie = rospy.Subscriber(
            f'/{self._vehicle_name}/detect/duck',
            Float64,
            self.cbDuckieDetected,
            queue_size=1
        )

        # Kreuzung abgeschlossen (von control_obstacle_node)
        self.sub_obstacle_done = rospy.Subscriber(
            f'/{self._vehicle_name}/obstacle/done',
            Bool,
            self.cbObstacleDone,
            queue_size=1
        )

        # Spurversatz (von detect_lane_node)
        # → für zukünftige Logik (z.B. Spur verloren erkennen)
        self.sub_lane = rospy.Subscriber(
            f'/{self._vehicle_name}/detect/lane',
            Float64,
            self.cbLaneDetected,
            queue_size=1
        )

        # ── Zustandsvariablen für Kreuzungserkennung ───────────────────────────
        # Kreuzung wird erkannt wenn BEIDE Bedingungen gleichzeitig erfüllt sind:
        #   1. Rote Linie sichtbar
        #   2. AprilTag sichtbar
        # → verhindert Fehlauslösungen durch rote Linie alleine

        # Ist gerade eine rote Linie sichtbar?
        self._stop_line_active = False

        # Ist gerade ein AprilTag sichtbar? (-1 = kein Tag)
        self._current_tag_id = -1

        # Wurde die Kreuzung bereits eingeleitet?
        # → verhindert mehrfaches Auslösen während Bot an roter Linie steht
        self._intersection_triggered = False

        rospy.loginfo(f"[{node_name}] Bereit. Startet im Lane-Modus.")


    # ── Callbacks ─────────────────────────────────────────────────────────────

    def cbStopLine(self, msg):
        # Rote Linie Status aktualisieren
        self._stop_line_active = msg.data
        # Kreuzungserkennung prüfen
        self._check_intersection()


    def cbAprilTag(self, msg):
        # AprilTag Status aktualisieren
        self._current_tag_id = msg.data
        # Kreuzungserkennung prüfen
        self._check_intersection()


    def _check_intersection(self):
        # Kreuzung erkennen: rote Linie UND AprilTag gleichzeitig sichtbar
        # Nur auslösen wenn:
        #   - gerade im Lane-Modus (kein Doppelt-Auslösen)
        #   - noch nicht ausgelöst (kein Doppelt-Auslösen)
        if (self._stop_line_active
                and self._current_tag_id != -1
                and self._control_mode == ControlType.Lane
                and not self._intersection_triggered):

            rospy.loginfo(
                f"Kreuzung erkannt! Tag-ID={self._current_tag_id} "
                f"→ wechsle zu Intersection-Modus."
            )
            self._intersection_triggered = True
            self._control_mode = ControlType.Intersection


    def cbIntersectionDone(self, msg):
        # control_intersection_node meldet Kreuzung abgeschlossen
        if msg.data and self._control_mode == ControlType.Intersection:
            rospy.loginfo("Kreuzung abgeschlossen → zurück zu Lane Following.")
            self._control_mode           = ControlType.Lane
            self._intersection_triggered = False
            self._stop_line_active       = False
            self._current_tag_id         = -1


    def cbDuckieDetected(self, msg):
        # Challenge 3: Ente erkannt → Obstacle-Modus aktivieren
        # msg.data = normierte X-Position der Ente (-99 = keine Ente)
        duck_present = msg.data != -99.0

        if duck_present and self._control_mode == ControlType.Lane:
            rospy.loginfo(f"Ente erkannt (x={msg.data:.2f}) → Obstacle-Modus")
            self._control_mode = ControlType.Obstacle

        elif not duck_present and self._control_mode == ControlType.Obstacle:
            # Wird von control_obstacle_node über /obstacle/done gemeldet
            # → hier nur als Fallback falls done-Message ausbleibt
            pass


    def cbObstacleDone(self, msg):
        # control_obstacle_node meldet Ausweichen abgeschlossen
        if msg.data and self._control_mode == ControlType.Obstacle:
            rospy.loginfo("Ausweichen abgeschlossen → zurück zu Lane Following.")
            self._control_mode = ControlType.Lane

    def cbLaneDetected(self, msg):
        # Spurversatz empfangen – für zukünftige Logik nutzbar
        # Noch nicht implementiert
        pass


    # ── Hauptschleife ─────────────────────────────────────────────────────────

    def run(self):
        # Aktiven Modus kontinuierlich mit 10 Hz publizieren
        # → alle Control-Nodes sind immer über den aktuellen Modus informiert
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            msg_control      = Int32()
            msg_control.data = self._control_mode.value
            self.pub_control.publish(msg_control)
            rate.sleep()


if __name__ == '__main__':
    node = SwitchControlNode(node_name='switch_control_node')
    node.run()
    rospy.spin()
