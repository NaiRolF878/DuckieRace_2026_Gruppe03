#!/usr/bin/env python3

# ─────────────────────────────────────────────────────────────────────────────
# control_obstacle_node.py
#
# Aufgabe (Challenge 3 – Watch out for Ducks):
#   Entscheidet, wie der Bot einer erkannten Ente ausweicht, und reicht das
#   Ergebnis als additiven Lenk-Offset an control_lane_node weiter.
#
# Architektur:
#   - control_lane_node bleibt die EINZIGE Stelle, die PID rechnet.
#   - Diese Node published nur einen additiven Korrekturterm:
#       /obstacle/error_offset (Float64)  →  control_lane_node addiert ihn.
#     Vorteil: keine Duplizierung der Regel-/Geschwindigkeits-/Stop-Line-Logik.
#   - Ein positiver Offset verschiebt die wahrgenommene Spurmitte so, dass der
#     Bot nach LINKS ausweicht; ein negativer Offset → nach RECHTS.
#       (control_lane: error>0 → nach rechts lenken. Damit der Bot nach links
#        fährt, muss der wahrgenommene Fehler kleiner/negativer werden → wir
#        ADDIEREN bei Links-Ausweichen einen negativen Offset. Siehe _signed_offset.)
#
# Eingaben:
#   /detect/duck        (Float64)           x∈[-1,1], -99 = keine Ente
#   /detect/duck_space  (Float32MultiArray) [free_left, free_right] ∈ [0..1]
#   /enable/obstacle    (Bool)              von switch_control_node
#
# Ausgaben:
#   /obstacle/error_offset (Float64)  additiver Lenk-Offset
#   /obstacle/done         (Bool)     Ausweichmanöver abgeschlossen → zurück zu Lane
#
# EvadeState-Automat:
#   Idle      → keine Ente, Offset = 0
#   Evading   → Ente im Weg: Richtung wählen, Offset auf Zielwert rampen, halten
#   Returning → Ente nicht mehr im Weg: Offset schrittweise zurück auf 0
#   → danach /obstacle/done = True und zurück zu Idle.
#
# Ausweichentscheidung (Platz links/rechts der Ente im BEV):
#   - mehr Platz rechts  → rechts ausweichen
#   - mehr Platz links   → links ausweichen
#   - etwa gleich        → links (StVO: links überholen)
#   - kein Platz         → Gegenspurübernahme (links, größerer Offset;
#                          erlaubt da keine anderen Bots auf dem Wendeplatz)
# ─────────────────────────────────────────────────────────────────────────────

import os
import rospy
from enum import Enum
from std_msgs.msg import Float64, Bool, Float32MultiArray
from duckietown_msgs.msg import Twist2DStamped
import util


class EvadeState(Enum):
    Idle      = 1   # keine Ente, kein Offset
    Evading   = 2   # weicht aktiv aus (Offset auf Zielwert)
    Returning = 3   # kehrt kontrolliert zur Spurmitte zurück (Offset → 0)


class ControlObstacleNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        self._vehicle_name = os.environ['VEHICLE_NAME']

        # ── Zustand ───────────────────────────────────────────────────────────
        self.enable        = False
        self.state         = EvadeState.Idle
        self.current_offset = 0.0       # aktuell ausgegebener Offset
        self.target_offset  = 0.0       # Zielwert beim Ausweichen
        self.evade_side     = 'none'    # 'left' / 'right' für Logging

        self.duck_x      = -99.0
        self.free_left   = 1.0
        self.free_right  = 1.0
        self.last_duck_seen = None      # Zeitstempel der letzten Entensichtung

        # Odometrie-Rückkehr (Befehls-Integration)
        self.dt           = 0.1         # Schrittzeit (10 Hz)
        self.last_omega   = 0.0         # zuletzt gesendetes omega (von cmd-Topic)
        self.yaw_integral = 0.0         # akkumulierter Gierwinkel des Ausweichens

        # Defaults – werden von cbUpdateParameters überschrieben
        self.evade_offset       = 0.6   # normaler Ausweich-Offset (|error|-Einheiten)
        self.oncoming_offset    = 1.0   # Offset bei Gegenspurübernahme
        self.space_threshold    = 0.10  # ab wann "mehr Platz" als eindeutig gilt
        self.no_space_limit     = 0.15  # darunter gilt eine Seite als "kein Platz"
        self.ramp_step          = 0.05  # Offset-Änderung pro Zyklus (Rampe)
        self.evade_hold         = 2.0   # s: Manöver nach letzter Sichtung halten
        self.return_mode        = 'pid' # 'pid' (kamerabasiert) oder 'odometry'
        self.yaw_tolerance      = 0.05  # rad: Restgierwinkel, ab dem Rückkehr fertig

        util.init_parameters(node_name, self.cbUpdateParameters)

        # ── Publisher ───────────────────────────────────────────────────────────
        self.pub_offset = rospy.Publisher(
            f'/{self._vehicle_name}/obstacle/error_offset', Float64, queue_size=1)
        self.pub_done = rospy.Publisher(
            f'/{self._vehicle_name}/obstacle/done', Bool, queue_size=1)

        # ── Subscriber ────────────────────────────────────────────────────────
        rospy.Subscriber(f'/{self._vehicle_name}/detect/duck',
            Float64, self.cbDuck, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/detect/duck_space',
            Float32MultiArray, self.cbSpace, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/enable/obstacle',
            Bool, self.cbEnable, queue_size=1)

        # Mitlesen, welches omega control_lane_node tatsächlich sendet
        # → Basis für die Befehls-Odometrie bei der Rückkehr.
        rospy.Subscriber(f'/{self._vehicle_name}/car_cmd_switch_node/cmd',
            Twist2DStamped, self.cbCmd, queue_size=1)

        rospy.loginfo(f"[{node_name}] Bereit. EvadeState=Idle.")


    # ── Parameter ───────────────────────────────────────────────────────────────

    def cbUpdateParameters(self, parameters):
        e = parameters["evade"]
        self.evade_offset    = e["evade_offset"]["default"]
        self.oncoming_offset = e["oncoming_offset"]["default"]
        self.space_threshold = e["space_threshold"]["default"]
        self.no_space_limit  = e["no_space_limit"]["default"]
        self.ramp_step       = e["ramp_step"]["default"]
        # evade_hold: Mindestdauer, die das Ausweichmanöver NACH der letzten
        # positiven Entensichtung gehalten wird, bevor zurückgekehrt wird.
        # Grund: Beim Ausweichen wandert die Ente aus dem Bild/ROI → ohne Hold
        # würde der Bot zu früh zurücklenken, solange er noch neben der Ente ist.
        self.evade_hold      = e["evade_hold"]["default"]

        # Rückkehrmodus: 'pid' (kamerabasiert, Offset-Rampe) oder
        # 'odometry' (Befehls-Integration des gefahrenen omega, spiegelbildlich).
        # return_mode wird als Zahl gespeichert (Slider): 0 = pid, 1 = odometry.
        self.return_mode   = 'odometry' if int(e["return_mode"]["default"]) == 1 else 'pid'
        self.yaw_tolerance = e["yaw_tolerance"]["default"]


    # ── Callbacks ───────────────────────────────────────────────────────────────

    def cbEnable(self, msg):
        self.enable = msg.data

    def cbCmd(self, msg):
        # Zuletzt tatsächlich gesendetes omega merken (Befehls-Odometrie).
        self.last_omega = msg.omega

    def cbSpace(self, msg):
        if len(msg.data) >= 2:
            self.free_left  = msg.data[0]
            self.free_right = msg.data[1]

    def cbDuck(self, msg):
        self.duck_x = msg.data
        if self.duck_x != -99.0:
            self.last_duck_seen = rospy.Time.now()


    # ── Ausweichentscheidung ─────────────────────────────────────────────────────

    def _decide_side_and_offset(self):
        # Wählt Richtung und Ziel-Offset-Betrag anhand des freien Platzes.
        # Rückgabe: (side, signed_offset)
        l, r = self.free_left, self.free_right

        # Kein Platz auf beiden Seiten → Gegenspurübernahme (links, groß)
        if l < self.no_space_limit and r < self.no_space_limit:
            return 'left(oncoming)', self._signed_offset('left', self.oncoming_offset)

        # Eindeutig mehr Platz rechts → rechts ausweichen
        if r - l > self.space_threshold:
            return 'right', self._signed_offset('right', self.evade_offset)

        # Eindeutig mehr Platz links → links ausweichen
        if l - r > self.space_threshold:
            return 'left', self._signed_offset('left', self.evade_offset)

        # Etwa gleich viel Platz → StVO: links überholen
        return 'left', self._signed_offset('left', self.evade_offset)

    def _signed_offset(self, side, magnitude):
        # Vorzeichenkonvention von control_lane_node:
        #   error > 0 → nach rechts lenken,  error < 0 → nach links lenken.
        # Damit der Bot nach LINKS ausweicht, muss der addierte Offset NEGATIV sein.
        return -magnitude if side.startswith('left') else +magnitude


    # ── Offset-Rampe ──────────────────────────────────────────────────────────

    def _ramp_toward(self, target):
        # Bewegt current_offset um höchstens ramp_step in Richtung target.
        diff = target - self.current_offset
        if abs(diff) <= self.ramp_step:
            self.current_offset = target
        else:
            self.current_offset += self.ramp_step * (1 if diff > 0 else -1)

    def _duck_in_way(self):
        # Ente gilt als "im Weg", solange die letzte POSITIVE Sichtung weniger
        # als evade_hold Sekunden zurückliegt. Der aktuelle duck_x wird bewusst
        # NICHT mit einbezogen: beim Ausweichen wandert die Ente aus dem Bild,
        # liefert also -99 – das Manöver muss aber durchgezogen werden, bis der
        # Bot sicher an ihr vorbei ist. Reine Zeit ist hier die robuste Heuristik;
        # ein verlorener Einzeltick (queue_size=1) verlängert das Manöver nur
        # minimal, statt es fälschlich abzubrechen.
        if self.last_duck_seen is None:
            return False
        elapsed = (rospy.Time.now() - self.last_duck_seen).to_sec()
        return elapsed < self.evade_hold


    # ── Zustandsautomat (in run() getaktet) ───────────────────────────────────

    def _step_state_machine(self):
        if self.state == EvadeState.Idle:
            self.current_offset = 0.0
            self.yaw_integral   = 0.0    # akkumulierter Gierwinkel des Ausweichens
            if self._duck_in_way():
                self.evade_side, self.target_offset = self._decide_side_and_offset()
                self.state = EvadeState.Evading
                rospy.loginfo(
                    f"[Evade] Ente → ausweichen nach {self.evade_side} "
                    f"(Ziel-Offset {self.target_offset:+.2f}, Modus={self.return_mode}).")

        elif self.state == EvadeState.Evading:
            # Offset auf Zielwert rampen und halten. Richtung nur aktualisieren,
            # solange die Ente FRISCH sichtbar ist (sonst Richtung einfrieren).
            if self._duck_in_way():
                if self.duck_x != -99.0:
                    self.evade_side, self.target_offset = self._decide_side_and_offset()
                self._ramp_toward(self.target_offset)
                # Tatsächlich gefahrenes omega integrieren (für Odometrie-Rückkehr)
                self.yaw_integral += self.last_omega * self.dt
            else:
                rospy.loginfo(
                    f"[Evade] Ente passiert → Rückkehr ({self.return_mode}). "
                    f"Gierintegral={self.yaw_integral:+.3f} rad.")
                self.state = EvadeState.Returning

        elif self.state == EvadeState.Returning:
            # Ente wieder aufgetaucht? → erneut ausweichen
            if self._duck_in_way():
                self.state = EvadeState.Evading
                return

            if self.return_mode == 'odometry':
                self._return_odometry()
            else:
                self._return_pid()

    def _return_pid(self):
        # Kamerabasiert: Offset auf 0 rampen, Spur-PID findet die Linie wieder.
        self._ramp_toward(0.0)
        if abs(self.current_offset) < 1e-6:
            self._finish_return()

    def _return_odometry(self):
        # Befehls-Odometrie: Gegen-Offset halten und das währenddessen gefahrene
        # omega aufintegrieren, bis der Gierwinkel des Hinwegs ausgeglichen ist.
        # Hinweg-Gierintegral hatte das Vorzeichen von -target_offset-Richtung;
        # wir halten einen entgegengesetzten Offset, bis yaw_integral ~0 ist.
        counter_offset = -self.target_offset
        self._ramp_toward(counter_offset)
        self.yaw_integral += self.last_omega * self.dt

        # Sobald die Gierbewegung neutralisiert ist → fertig
        if abs(self.yaw_integral) < self.yaw_tolerance:
            self.current_offset = 0.0
            self._finish_return()

    def _finish_return(self):
        self.current_offset = 0.0
        self.yaw_integral   = 0.0
        self.state          = EvadeState.Idle
        self.pub_done.publish(Bool(data=True))
        rospy.loginfo("[Evade] Rückkehr abgeschlossen → Idle, /obstacle/done.")


    # ── Hauptschleife ──────────────────────────────────────────────────────────

    def run(self):
        rate = rospy.Rate(10)   # 10 Hz – passend zu dt der PID-Regelung
        while not rospy.is_shutdown():
            if self.enable:
                self._step_state_machine()
            else:
                # Nicht aktiv → Offset hart auf 0, Automat zurücksetzen
                self.current_offset = 0.0
                self.state = EvadeState.Idle

            # Offset immer publizieren (auch 0.0), damit control_lane_node
            # nach Modus-Ende garantiert wieder ohne Offset fährt.
            self.pub_offset.publish(Float64(data=self.current_offset))
            rate.sleep()


if __name__ == '__main__':
    node = ControlObstacleNode('control_obstacle_node')
    node.run()
    rospy.spin()
