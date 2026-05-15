#!/usr/bin/env python3

# ─────────────────────────────────────────────────────────────────────────────
# control_obstacle_node.py
#
# Aufgabe: Weicht Enten auf der Fahrbahn aus.
#
# Ausweichlogik:
#   1. Enten-Position (x) und Spurbreite bekannt
#   2. Platz links  = ente_x - center_yellow
#      Platz rechts = center_white - ente_x
#   3. Ausweichentscheidung:
#      platz_rechts > platz_links  → rechts ausweichen
#      platz_links  > platz_rechts → links ausweichen
#      beide gleich (mittig)       → links (StVO: zur gelben Linie)
#      beide < min_side_space      → Gegenspurübernahme (über gelbe Linie)
#   4. Rückkehr: offset wird schrittweise auf 0 reduziert (kontrolliert)
#
# Zustandsautomat:
#   Idle → Evading → Returning → Idle
#
# Abonniert:
#   /detect/duck   (Float64) → x-Position Ente [-1,+1], -99 = keine Ente
#   /detect/lane   (Float64) → Spurversatz für PID-Offset
#   /switch/control(Int32)   → aktiviert/deaktiviert
#
# Publiziert:
#   /detect/lane_corrected (Float64) → korrigierter Spurversatz an control_lane_node
# ─────────────────────────────────────────────────────────────────────────────

import os
import rospy
from std_msgs.msg import Float64, Int32
from enum import Enum
from switch_control_node import ControlType
import util


class EvadeState(Enum):
    Idle      = 1  # kein Hindernis
    Evading   = 2  # weicht aus (offset wird gehalten)
    Returning = 3  # kehrt kontrolliert in Spur zurück (offset → 0)


class ControlObstacleNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        self._vehicle_name = os.environ['VEHICLE_NAME']
        util.init_parameters(node_name, self.cbUpdateParameters)

        # Publisher: korrigierter Spurversatz
        # control_lane_node abonniert dieses Topic statt /detect/lane direkt
        self.pub_lane_corrected = rospy.Publisher(
            f'/{self._vehicle_name}/detect/lane_corrected',
            Float64,
            queue_size=1
        )

        # Subscriber
        self.sub_duck = rospy.Subscriber(
            f'/{self._vehicle_name}/detect/duck',
            Float64, self.cbDuck, queue_size=1)

        self.sub_lane = rospy.Subscriber(
            f'/{self._vehicle_name}/detect/lane',
            Float64, self.cbLane, queue_size=1)

        self.sub_control = rospy.Subscriber(
            f'/{self._vehicle_name}/switch/control',
            Int32, self.cbControl, queue_size=1)

        # Zustandsautomat
        self.evade_state    = EvadeState.Idle
        self.enable         = False

        # Aktueller Ausweich-Offset: wird auf den Spurversatz addiert
        # positiv = nach links, negativ = nach rechts
        self.current_offset = 0.0

        # Letzter bekannter Spurversatz (wird direkt weitergeleitet wenn kein Hindernis)
        self.last_error     = 0.0

        # Letzte bekannte Linienpositionen (aus cbLane geschätzt)
        self._crop_im_size  = 400
        self.center_white   = int(self._crop_im_size * 0.95)
        self.center_yellow  = int(self._crop_im_size * 0.05)

        rospy.loginfo(f"[{node_name}] Obstacle-Steuerung bereit.")


    def cbUpdateParameters(self, parameters):
        # Maximaler Ausweich-Offset (wie weit der Bot von der Spurmitte abweicht)
        self.max_evade_offset      = parameters["evade"]["max_offset"]["default"]

        # Gegenspurübernahme: noch größerer Offset
        self.overtake_offset       = parameters["evade"]["overtake_offset"]["default"]

        # Mindestplatz auf einer Seite für normales Ausweichen
        self.min_side_space        = parameters["evade"]["min_side_space"]["default"]

        # Wie schnell der Offset pro Schritt reduziert wird (Rückkehr)
        self.return_step           = parameters["evade"]["return_step"]["default"]

        # Schwellwert unter dem der Offset als "bei 0" gilt
        self.return_threshold      = parameters["evade"]["return_threshold"]["default"]


    def cbControl(self, msg):
        # Nur aktiv wenn Obstacle-Modus
        if msg.data == ControlType.Obstacle.value:
            self.enable = True
        else:
            self.enable = False
            # Zustand zurücksetzen wenn deaktiviert
            self.evade_state    = EvadeState.Idle
            self.current_offset = 0.0


    def cbLane(self, msg):
        # Spurversatz speichern und Linienpositionen schätzen
        self.last_error = msg.data
        error           = msg.data
        lane_center     = (1 - error) * (self._crop_im_size / 2)
        half_lane       = self._crop_im_size * 0.30
        self.center_white  = int(min(lane_center + half_lane, self._crop_im_size * 0.95))
        self.center_yellow = int(max(lane_center - half_lane, self._crop_im_size * 0.05))


    def cbDuck(self, msg):
        # Enten-Position empfangen und Ausweichentscheidung treffen
        duck_x_norm = msg.data

        if duck_x_norm == -99.0:
            # Keine Ente erkannt
            if self.evade_state == EvadeState.Evading:
                # Ente verschwunden → kontrollierte Rückkehr starten
                rospy.loginfo("Ente weg – kontrollierte Rückkehr in Spur.")
                self.evade_state = EvadeState.Returning
            return

        if self.evade_state != EvadeState.Idle:
            # Bereits am Ausweichen → nicht neu entscheiden
            return

        # Enten-Position in Pixel umrechnen
        duck_x_pixel = int((duck_x_norm + 1) / 2 * self._crop_im_size)

        # Platz auf beiden Seiten berechnen
        platz_links  = duck_x_pixel - self.center_yellow
        platz_rechts = self.center_white - duck_x_pixel

        rospy.loginfo(f"Ente erkannt: x={duck_x_pixel}px | Platz links={platz_links}px, rechts={platz_rechts}px")

        # Ausweichentscheidung
        if platz_links < self.min_side_space and platz_rechts < self.min_side_space:
            # Kein Platz auf beiden Seiten → Gegenspurübernahme
            rospy.logwarn("Kein Platz zum Ausweichen → Gegenspurübernahme!")
            self.current_offset = self.overtake_offset   # weit nach links
            self.evade_state    = EvadeState.Evading

        elif platz_rechts > platz_links:
            # Mehr Platz rechts → nach rechts ausweichen
            rospy.loginfo("Ausweichen: rechts")
            self.current_offset = -self.max_evade_offset  # negativ = rechts
            self.evade_state    = EvadeState.Evading

        elif platz_links > platz_rechts:
            # Mehr Platz links → nach links ausweichen
            rospy.loginfo("Ausweichen: links")
            self.current_offset = self.max_evade_offset   # positiv = links
            self.evade_state    = EvadeState.Evading

        else:
            # Gleich viel Platz (mittig) → StVO: zur gelben Linie (links)
            rospy.loginfo("Ente mittig → Ausweichen links (StVO)")
            self.current_offset = self.max_evade_offset
            self.evade_state    = EvadeState.Evading


    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.enable:

                if self.evade_state == EvadeState.Idle:
                    # Kein Hindernis → Spurversatz direkt weiterleiten
                    self.pub_lane_corrected.publish(Float64(data=self.last_error))

                elif self.evade_state == EvadeState.Evading:
                    # Ausweichen: Offset auf Spurversatz addieren
                    corrected_error = self.last_error + self.current_offset
                    # Begrenzung auf [-1, +1]
                    corrected_error = max(-1.0, min(1.0, corrected_error))
                    self.pub_lane_corrected.publish(Float64(data=corrected_error))

                elif self.evade_state == EvadeState.Returning:
                    # Kontrollierte Rückkehr: Offset schrittweise auf 0 reduzieren
                    if self.current_offset > self.return_threshold:
                        self.current_offset -= self.return_step
                    elif self.current_offset < -self.return_threshold:
                        self.current_offset += self.return_step
                    else:
                        # Offset nahe genug an 0 → Rückkehr abgeschlossen
                        self.current_offset = 0.0
                        self.evade_state    = EvadeState.Idle
                        rospy.loginfo("Rückkehr in Spur abgeschlossen.")

                    corrected_error = self.last_error + self.current_offset
                    corrected_error = max(-1.0, min(1.0, corrected_error))
                    self.pub_lane_corrected.publish(Float64(data=corrected_error))

            rate.sleep()


if __name__ == '__main__':
    node = ControlObstacleNode('control_obstacle_node')
    node.run()
    rospy.spin()
