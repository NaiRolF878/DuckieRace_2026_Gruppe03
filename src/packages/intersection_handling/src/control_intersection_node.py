#!/usr/bin/env python3

# ─────────────────────────────────────────────────────────────────────────────
# control_intersection_node.py
#
# Aufgabe: Steuert den Duckiebot durch eine Kreuzung in vier Phasen:
#
#   Phase 1 – Approaching:
#     Bot fährt vorwärts bis die eigene rote Haltelinie verschwindet.
#     Danach noch kurz weiterfahren (extra_duration) damit der Bot
#     sicher auf der Kreuzungsfläche steht.
#     → Sicherheitsnetz: Timeout falls Linie nie verschwindet
#
#   Phase 2 – Turning:
#     Bot dreht in die gewählte Richtung.
#     Orientierung über rote Linie der Gegenspur:
#       Links  → drehe bis rote Linie RECHTS im Bild erscheint
#       Rechts → drehe bis rote Linie LINKS  im Bild erscheint
#       Gerade → fahre bis rote Linie komplett verschwindet
#     → Sicherheitsnetz: Timeout
#
#   Phase 3 – Lane Handover:
#     Sobald Spurlinien stabil erkannt werden übernimmt PID-Regler.
#     → Sicherheitsnetz: Timeout
#
#   Phase 4 – Done:
#     Meldet Kreuzung abgeschlossen an switch_control_node.
#
# Zustandsautomat:
#   Idle → Approaching → Turning → Done
#
# Abonniert:
#   /detect/apriltag       (Int32)   → Tag-ID für Richtungsentscheidung
#   /detect/lane           (Float64) → Spurversatz für Lane-Handover
#   /detect/stop_line      (Bool)    → eigene Haltelinie sichtbar?
#   /detect/stop_line_side (String)  → auf welcher Seite ist rote Linie?
#   /switch/control        (Int32)   → aktiviert/deaktiviert diese Node
#
# Publiziert:
#   /car_cmd_switch_node/cmd  (Twist2DStamped) → Fahrbefehle
#   /intersection/done        (Bool)           → Kreuzung abgeschlossen
# ─────────────────────────────────────────────────────────────────────────────

import os
import random
import rospy
from std_msgs.msg import Float64, Int32, Bool, String
from duckietown_msgs.msg import Twist2DStamped
from enum import Enum
from switch_control_node import ControlType
import util


class TurnState(Enum):
    Idle        = 1  # wartet auf Aktivierung durch switch_control_node
    Approaching = 2  # fährt vorwärts bis eigene rote Linie verschwindet + Timer
    Turning     = 3  # dreht bis rote Gegenspur-Linie auf korrekter Seite
    Done        = 4  # Kreuzung abgeschlossen, meldet zurück


class ControlIntersectionNode:
    def __init__(self, node_name):
        # ROS-Node initialisieren
        rospy.init_node(node_name)

        # Fahrzeugnamen aus Umgebungsvariable lesen
        self._vehicle_name = os.environ['VEHICLE_NAME']

        # Parameter aus JSON laden und Live-Update registrieren
        util.init_parameters(node_name, self.cbUpdateParameters)

        # ── Publisher ──────────────────────────────────────────────────────────
        self.pub_cmd_vel = rospy.Publisher(
            f'/{self._vehicle_name}/car_cmd_switch_node/cmd',
            Twist2DStamped,
            queue_size=1
        )
        # Meldet Kreuzung abgeschlossen → switch_control_node schaltet zurück
        self.pub_done = rospy.Publisher(
            f'/{self._vehicle_name}/intersection/done',
            Bool,
            queue_size=1
        )

        # ── Subscriber ─────────────────────────────────────────────────────────
        self.sub_apriltag = rospy.Subscriber(
            f'/{self._vehicle_name}/detect/apriltag',
            Int32,
            self.cbAprilTag,
            queue_size=1
        )
        self.sub_lane = rospy.Subscriber(
            f'/{self._vehicle_name}/detect/lane',
            Float64,
            self.cbLane,
            queue_size=1
        )
        self.sub_stop_line = rospy.Subscriber(
            f'/{self._vehicle_name}/detect/stop_line',
            Bool,
            self.cbStopLine,
            queue_size=1
        )
        # Seite der roten Linie: 'none', 'left', 'right', 'both'
        self.sub_stop_line_side = rospy.Subscriber(
            f'/{self._vehicle_name}/detect/stop_line_side',
            String,
            self.cbStopLineSide,
            queue_size=1
        )
        self.sub_control = rospy.Subscriber(
            f'/{self._vehicle_name}/switch/control',
            Int32,
            self.cbControl,
            queue_size=1
        )

        # ── Zustandsvariablen ─────────────────────────────────────────────────
        self.enable            = False
        self.turn_state        = TurnState.Idle
        self.current_direction = None        # 'left', 'right', 'straight'
        self.phase_start_time  = None        # Zeitstempel für Timeout-Berechnung

        # Letzte bekannte Sensorwerte
        self.stop_line_active  = False       # eigene Haltelinie sichtbar?
        self.stop_line_side    = 'none'      # Seite der roten Linie
        self.stable_lane_count = 0           # stabile Frames für Lane Handover
        self.allowed_directions = ['left', 'right', 'straight']  # Fallback

        # Approach-Phase: wurde rote Linie schon einmal gesehen?
        # → stellt sicher dass wir erst auf Verschwinden warten wenn Linie vorher da war
        self.stop_line_was_seen = False
        # Zeitstempel des Verschwindens der roten Linie (für extra_duration)
        self.stop_line_gone_time = None

        rospy.on_shutdown(self.fnShutDown)
        rospy.loginfo(f"[{node_name}] Kreuzungssteuerung bereit.")


    # ── Parameter ─────────────────────────────────────────────────────────────

    def cbUpdateParameters(self, parameters):

        # Phase 1 – Approaching: vorwärts bis Linie weg
        self.approach_v              = parameters["approach"]["v"]["default"]
        # Wie lange nach Verschwinden der Linie noch weiterfahren
        self.approach_extra_duration = parameters["approach"]["extra_duration"]["default"]
        # Sicherheits-Timeout für die Approach-Phase
        self.approach_timeout        = parameters["approach"]["timeout"]["default"]

        # Phase 2 – Turning: drehen bis Gegenspur-Linie auf korrekter Seite
        self.left_v        = parameters["left"]["v"]["default"]
        self.left_omega    = parameters["left"]["omega"]["default"]
        self.right_v       = parameters["right"]["v"]["default"]
        self.right_omega   = parameters["right"]["omega"]["default"]
        self.straight_v    = parameters["straight"]["v"]["default"]
        self.straight_omega= parameters["straight"]["omega"]["default"]
        # Sicherheits-Timeout für die Turning-Phase
        self.turn_timeout  = parameters["turning"]["timeout"]["default"]

        # Phase 3 – Lane Handover
        # Wie klein muss der Spurversatz sein um als stabil zu gelten
        self.lane_handover_threshold = parameters["handover"]["lane_threshold"]["default"]
        # Wie viele aufeinanderfolgende stabile Frames nötig
        self.lane_handover_frames    = parameters["handover"]["stable_frames"]["default"]
        # Sicherheits-Timeout für Lane-Handover-Phase
        self.handover_timeout        = parameters["handover"]["timeout"]["default"]

        # Tag-ID → erlaubte Richtungen Mapping
        self.tag_direction_map = {}
        for key, val in parameters["tag_directions"].items():
            tag_id     = int(key)
            directions = [d.strip() for d in val["default"].split(',')]
            self.tag_direction_map[tag_id] = directions
        print(f"Tag-Richtungen geladen: {self.tag_direction_map}")


    # ── Callbacks ─────────────────────────────────────────────────────────────

    def cbControl(self, msg):
        # Aktivierung durch switch_control_node
        if msg.data == ControlType.Intersection.value:
            if not self.enable:
                rospy.loginfo("Intersection-Modus aktiviert.")
            self.enable = True
        else:
            self.enable = False
            # Zustand zurücksetzen wenn deaktiviert
            if self.turn_state != TurnState.Idle:
                self._reset_state()


    def cbAprilTag(self, msg):
        # AprilTag-ID empfangen → erlaubte Richtungen nachschlagen
        tag_id = msg.data
        if tag_id == -1:
            return
        if tag_id in self.tag_direction_map:
            self.allowed_directions = self.tag_direction_map[tag_id]
            print(f"Tag ID {tag_id} → erlaubte Richtungen: {self.allowed_directions}")
        else:
            rospy.logwarn(f"Unbekannte Tag-ID {tag_id} – alle Richtungen erlaubt")
            self.allowed_directions = ['left', 'right', 'straight']


    def cbStopLine(self, msg):
        # Eigene Haltelinie Status aktualisieren
        self.stop_line_active = msg.data
        # Merken ob Linie schon mal gesehen wurde (für Approach-Phase)
        if msg.data:
            self.stop_line_was_seen = True


    def cbStopLineSide(self, msg):
        # Seite der roten Linie aktualisieren
        self.stop_line_side = msg.data


    def cbLane(self, msg):
        # Spurversatz empfangen – für Lane-Handover-Erkennung in Turning-Phase
        if self.turn_state != TurnState.Turning:
            self.stable_lane_count = 0
            return

        error = abs(msg.data)
        if error < self.lane_handover_threshold:
            self.stable_lane_count += 1
        else:
            self.stable_lane_count = 0


    # ── Hilfsfunktionen ───────────────────────────────────────────────────────

    def _reset_state(self):
        # Alle Zustandsvariablen zurücksetzen
        self.turn_state          = TurnState.Idle
        self.current_direction   = None
        self.phase_start_time    = None
        self.stable_lane_count   = 0
        self.stop_line_was_seen  = False
        self.stop_line_gone_time = None


    def _publish_twist(self, v, omega):
        # Fahrbefehl senden
        twist             = Twist2DStamped()
        twist.header.stamp= rospy.Time.now()
        twist.v           = v
        twist.omega       = omega
        self.pub_cmd_vel.publish(twist)


    def _elapsed(self):
        # Verstrichene Zeit seit Beginn der aktuellen Phase
        if self.phase_start_time is None:
            return 0.0
        return (rospy.Time.now() - self.phase_start_time).to_sec()


    def _check_turn_complete(self):
        # Prüft ob der Bot in der Turning-Phase fertig ausgerichtet ist
        # Bedingung hängt von der gewählten Richtung ab:
        #
        #   Links:   rote Linie erscheint RECHTS → wir schauen in die neue Fahrtrichtung
        #   Rechts:  rote Linie erscheint LINKS  → wir schauen in die neue Fahrtrichtung
        #   Gerade:  rote Linie komplett weg     → Kreuzung überquert
        #
        if self.current_direction == 'left':
            return self.stop_line_side == 'right'
        elif self.current_direction == 'right':
            return self.stop_line_side == 'left'
        elif self.current_direction == 'straight':
            return self.stop_line_side == 'none'
        return False


    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")
        self._publish_twist(0.0, 0.0)


    # ── Hauptschleife ─────────────────────────────────────────────────────────

    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            if self.enable:

                # ── Phase 1: Idle → Approaching ───────────────────────────────
                if self.turn_state == TurnState.Idle:
                    # Richtung wählen und Approach-Phase starten
                    self.current_direction = random.choice(self.allowed_directions)
                    self.phase_start_time  = rospy.Time.now()
                    self.stop_line_was_seen= False
                    self.stop_line_gone_time = None
                    self.turn_state        = TurnState.Approaching
                    rospy.loginfo(f"Starte Kreuzung: Richtung={self.current_direction}")

                # ── Phase 2: Approaching ──────────────────────────────────────
                elif self.turn_state == TurnState.Approaching:
                    # Bot fährt vorwärts bis eigene rote Linie verschwindet
                    # + extra_duration Sicherheitspuffer

                    if self.stop_line_active:
                        # Linie sichtbar → weiterfahren
                        self._publish_twist(self.approach_v, 0.0)

                    elif self.stop_line_was_seen and not self.stop_line_active:
                        # Linie war sichtbar und ist gerade verschwunden
                        if self.stop_line_gone_time is None:
                            # Zeitstempel des Verschwindens merken
                            self.stop_line_gone_time = rospy.Time.now()
                            rospy.loginfo("Rote Linie verschwunden – extra_duration läuft...")

                        extra_elapsed = (rospy.Time.now() - self.stop_line_gone_time).to_sec()

                        if extra_elapsed >= self.approach_extra_duration:
                            # extra_duration abgelaufen → Bot steht sicher auf Kreuzung
                            rospy.loginfo("Bot auf Kreuzung – starte Abbiegemanöver.")
                            self.phase_start_time  = rospy.Time.now()
                            self.stable_lane_count = 0
                            self.turn_state        = TurnState.Turning
                        else:
                            # Noch weiterfahren
                            self._publish_twist(self.approach_v, 0.0)

                    else:
                        # Linie noch nie gesehen → weiterfahren und warten
                        self._publish_twist(self.approach_v, 0.0)

                    # Sicherheits-Timeout Approach-Phase
                    if self._elapsed() >= self.approach_timeout:
                        rospy.logwarn("Approach-Timeout – starte Abbiegemanöver trotzdem.")
                        self.phase_start_time  = rospy.Time.now()
                        self.stable_lane_count = 0
                        self.turn_state        = TurnState.Turning

                # ── Phase 3: Turning ──────────────────────────────────────────
                elif self.turn_state == TurnState.Turning:
                    # Holen der Abbiege-Parameter für aktuelle Richtung
                    if self.current_direction == 'left':
                        v, omega = self.left_v, self.left_omega
                    elif self.current_direction == 'right':
                        v, omega = self.right_v, self.right_omega
                    else:
                        v, omega = self.straight_v, self.straight_omega

                    # Option C: Lane Handover ODER rote Linie auf korrekter Seite
                    lane_stable   = self.stable_lane_count >= self.lane_handover_frames
                    turn_complete = self._check_turn_complete()
                    timeout       = self._elapsed() >= self.turn_timeout

                    if lane_stable:
                        rospy.loginfo(f"Lane Handover – Spur stabil erkannt nach {self._elapsed():.1f}s")
                        self.turn_state = TurnState.Done

                    elif turn_complete:
                        rospy.loginfo(f"Rote Linie auf korrekter Seite – Lane Handover startet")
                        self.phase_start_time  = rospy.Time.now()
                        self.stable_lane_count = 0
                        # Noch kurz weiterfahren bis Lane stabil
                        self._publish_twist(v, 0.0)

                    elif timeout:
                        rospy.logwarn(f"Turn-Timeout nach {self._elapsed():.1f}s – weiter mit Lane Following")
                        self.turn_state = TurnState.Done

                    else:
                        # Noch drehen
                        self._publish_twist(v, omega)

                # ── Phase 4: Done ─────────────────────────────────────────────
                elif self.turn_state == TurnState.Done:
                    # Stoppen, Kreuzung abgeschlossen melden
                    self._publish_twist(0.0, 0.0)
                    rospy.loginfo("Kreuzung abgeschlossen – zurück zu Lane Following.")
                    self.pub_done.publish(Bool(data=True))
                    self._reset_state()
                    self.enable = False

            rate.sleep()


if __name__ == '__main__':
    node = ControlIntersectionNode('control_intersection_node')
    node.run()
    rospy.spin()
