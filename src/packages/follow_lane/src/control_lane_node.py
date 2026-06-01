#!/usr/bin/env python3

import os
import rospy
from std_msgs.msg import Float64, Bool
from duckietown_msgs.msg import Twist2DStamped
from enum import Enum
import util

class StopState(Enum):
    Driving  = 1  # normales Spurfolgen
    Stopping = 2  # rote Linie erkannt, Bot hält für STOP_DURATION Sekunden an
    Cooldown = 3  # nach dem Anhalten kurz weiterfahren ohne erneut zu stoppen

class ControlLaneNode:
    def __init__(self,node_name):
        # ROS-Node initialisieren
        rospy.init_node(node_name)

        # Steuerung aktiv? Wird durch switch_control_node gesetzt
        self.enable = True

        # Fahrzeugnamen aus Umgebungsvariable lesen (z.B. "duckiebot01")
        self._vehicle_name = os.environ['VEHICLE_NAME']

        # Parameter aus der zugehörigen JSON-Konfigurationsdatei laden
        # und Callback für spätere Live-Aktualisierungen registrieren
        util.init_parameters(node_name, self.cbUpdateParameters)

        # Publisher für Fahrbefehle
        twist_topic = f"/{self._vehicle_name}/car_cmd_switch_node/cmd"
        self.pub_cmd_vel = rospy.Publisher(twist_topic, Twist2DStamped, queue_size = 1)

        # Subscriber: bekommt Spurversatz von detect_lane_node
        detect_lane_topic = f"/{self._vehicle_name}/detect/lane"
        self.sub_lane = rospy.Subscriber(detect_lane_topic, Float64, self.cbFollowLane, queue_size=1)

        # Subscriber: wird von switch_control_node aktiviert/deaktiviert
        # Bool True = diese Node ist aktiv, False = deaktiviert
        self.sub_enable = rospy.Subscriber(
            f'/{self._vehicle_name}/enable/lane',
            Bool, self.cbControl, queue_size=1)

        # Subscriber für rote Haltelinie von detect_lane_node
        stop_line_topic = f"/{self._vehicle_name}/detect/stop_line"
        self.sub_stop_line = rospy.Subscriber(stop_line_topic, Bool, self.cbStopLine, queue_size = 1)

        # PID Variablen
        self.lastError = 0       # Fehler vom letzten Schritt (für D-Anteil)
        self.integral  = 0       # aufsummierter Fehler (für I-Anteil)
        self.dt        = 0.1     # Zeit zwischen zwei Schritten (~10 Hz)
        # Anti-Windup: begrenzt den aufsummierten Fehler, damit der I-Anteil
        # bei längerer Abweichung (oder hohem ki) nicht unbegrenzt aufläuft
        self.INTEGRAL_LIMIT = 3.0

        # Steuerwerte
        self.v = 0               # Geschwindigkeit
        self.a = 0               # Winkelgeschwindigkeit (Lenkung)

        # NEU: Zustandsvariablen für die Haltelinien-Logik
        # Drei Zustände: StopState.Driving, StopState.Stopping, StopState.Cooldown
        self.stop_state      = StopState.Driving
        self.STOP_DURATION     = 3.0  # Startwert – wird durch cbUpdateParameters aus JSON überschrieben
        self.COOLDOWN_DURATION = 3.0  # Startwert – wird durch cbUpdateParameters aus JSON überschrieben
        self.stop_start_time = None  # Zeitstempel des Stopps

        rospy.on_shutdown(self.fnShutDown)
        rospy.loginfo(f"[{node_name}] Bereit. Warte auf Spurversatz ...")

#-------------------------------
# Callbacks
#-------------------------------

    def cbControl(self, msg):
        # Empfängt Enable-Signal von switch_control_node
        # True = Lane Following aktiv, False = andere Node übernimmt
        self.enable = msg.data

    def cbUpdateParameters(self, parameters):
        # Prüfung was ankommt
        rospy.logdebug("PARAMETER EMPFANGEN:")
        rospy.logdebug(f"{parameters}")
        
        # PID Parameter aus config laden
        self.kp      = parameters["pid"]["p"]["default"]
        self.ki      = parameters["pid"]["i"]["default"]
        self.kd      = parameters["pid"]["d"]["default"]
        self.MAX_VEL = parameters["pid"]["max_vel"]["default"]
        # Minimalgeschwindigkeit: verhindert dass der Bot durch großen Fehler komplett stoppt
        self.MIN_VEL = parameters["pid"]["min_vel"]["default"]

        # Haltelinien-Parameter aus config laden
        self.STOP_DURATION     = parameters["stop_line"]["stop_duration"]["default"]
        self.COOLDOWN_DURATION = parameters["stop_line"]["cooldown_duration"]["default"]

    # NEU: Callback für rote Haltelinie
    def cbStopLine(self, msg):
        # Im Cooldown-Modus: erneutes Erkennen ignorieren
        if self.stop_state == StopState.Cooldown:
            return

        # Wenn Linie erkannt und wir gerade normal fahren → Stopp einleiten
        if msg.data and self.stop_state == StopState.Driving:
            rospy.loginfo("Rote Haltelinie erkannt – halte 3 Sekunden an.")
            self.stop_state      = StopState.Stopping
            self.stop_start_time = rospy.Time.now()

    # Spurversatz error im Bereich [-1, +1]:
    # error > 0 → Bot zu weit links  → nach rechts lenken
    # error < 0 → Bot zu weit rechts → nach links lenken
    # error = 0 → Bot ist mittig     → geradeaus fahren
    def cbFollowLane(self, error):
        rospy.logdebug(f'received message. enabled : {self.enable}')

        # Wenn die rote Linie erkannt wurde → Geschwindigkeit auf 0 setzen und PID-Berechnung überspringen
        # (verhindert dass der Bot während des Stopps weiter lenkt oder beschleunigt)
        if self.stop_state == StopState.Stopping:
            self.v = 0.0
            self.a = 0.0
            return

        error = error.data

        # Begrenzung damit PID nicht übersteuert
        error = max(min(error, 2.0), -2.0)

        # PID REGELUNG 
        
        # P-Anteil: reagiert auf aktuellen Fehler
        P = self.kp * error

        # I-Anteil: summiert Fehler über Zeit
        # → gleicht systematische Abweichungen aus (z.B. schiefer Kamerawinkel)
        self.integral += error * self.dt
        # Anti-Windup: aufsummierten Fehler begrenzen
        self.integral = max(min(self.integral, self.INTEGRAL_LIMIT), -self.INTEGRAL_LIMIT)
        I = self.ki * self.integral

        # D-Anteil: reagiert auf Änderung des Fehlers
        # → dämpft Überschwingen und macht die Regelung stabiler
        derivative = (error - self.lastError) / self.dt
        D = self.kd * derivative

        # Gesamte Lenkung aus allen drei Anteilen
        self.a = P + I + D

        # Begrenzung der Lenkung auf [-3, +3] rad/s → verhindert „Durchdrehen"
        self.a = max(min(self.a, 3), -3)

        # Geschwindigkeit abhängig vom Fehler reduzieren:
        # Je größer der Spurversatz, desto langsamer fährt der Bot (sicherer in Kurven)
        # MIN_VEL verhindert dass der Bot bei großem Fehler komplett stoppt und stehen bleibt
        self.v = max(self.MIN_VEL, self.MAX_VEL * (1 - abs(error)))

        # Fehler speichern für nächsten Schritt (wird für D-Anteil benötigt)
        self.lastError = error


    def fnShutDown(self):
        # Beim Beenden der Node sicherstellen, dass der Bot sofort stoppt
        rospy.loginfo("Shutting down. cmd_vel will be 0")
        twist = Twist2DStamped(v=0.0, omega=0.0)
        self.pub_cmd_vel.publish(twist)

    def run(self):
        # Hauptschleife mit 10 Hz – publiziert Fahrbefehle basierend auf aktuellem Zustand
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.enable:
                twist = Twist2DStamped()
                twist.header.stamp = rospy.Time.now()

                # Haltelinien-Zustandsautomat:
                # Der Zustand wird durch cbStopLine gesetzt und hier ausgewertet
                if self.stop_state == StopState.Stopping:
                    # Stopp-Zeit läuft → Bot anhalten
                    elapsed = (rospy.Time.now() - self.stop_start_time).to_sec()
                    if elapsed < self.STOP_DURATION:
                        # Noch nicht lange genug gestanden → v und omega auf 0 halten
                        twist.v     = 0.0
                        twist.omega = 0.0
                    else:
                        # Stopp-Zeit abgelaufen → Cooldown starten und weiterfahren
                        rospy.loginfo("3 Sekunden vorbei – fahre weiter.")
                        self.stop_state      = StopState.Cooldown
                        self.stop_start_time = rospy.Time.now()
                        # Integral zurücksetzen, damit der I-Anteil keinen alten Fehler mitschleppt
                        self.integral = 0
                        twist.v     = self.v
                        twist.omega = self.a

                elif self.stop_state == StopState.Cooldown:
                    # Cooldown läuft → normal weiterfahren, aber keine neue Linie erkennen
                    # (verhindert direktes Wiederanhalten nach dem Losfahren)
                    elapsed = (rospy.Time.now() - self.stop_start_time).to_sec()
                    if elapsed >= self.COOLDOWN_DURATION:
                        rospy.loginfo("Cooldown beendet – Haltelinien-Erkennung wieder aktiv.")
                        self.stop_state = StopState.Driving
                    twist.v     = self.v
                    twist.omega = self.a

                else:
                    # Normaler Fahrbetrieb: Steuerwerte aus cbFollowLane direkt senden
                    twist.v     = self.v
                    twist.omega = self.a

                self.pub_cmd_vel.publish(twist)

            rate.sleep()

if __name__ == '__main__':
    # create the node
    node = ControlLaneNode('control_lane_node')
    node.run()
    rospy.spin()
