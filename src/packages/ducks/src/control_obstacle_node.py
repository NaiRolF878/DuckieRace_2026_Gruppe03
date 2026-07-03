#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────────────────────
# control_obstacle_node.py  (Challenge 3 – Stufen 4, 5, 6)
#
# Zustandsautomat: IDLE → EVADE → [WAIT] → PASS → RETURN → IDLE
#
# Stufe 4 – Ausweichen:
#   Trigger : /detect/zones [nah, mittel, fern] – nah ODER mittel → EVADE
#   Richtung + Stärke: aus /detect/corridor_occupancy (Lückenprofil, nah+mittel-
#   Band) beim EVADE-Eintritt bestimmt, für die Dauer des Manövers eingefroren.
#   Offset zeigt zur Mitte der breitesten freien Lücke, Stärke proportional zum
#   Abstand von der Korridormitte (min. evade_offset_min). Fallback auf die alte
#   duck_x-Heuristik, falls kein Profil vorliegt oder der Korridor komplett belegt ist.
#
# Stufe 5 – Encoder-Rückkehr:
#   Während EVADE+PASS: Radencoder-Ticks akkumulieren (data zählt IMMER aufwärts,
#   Richtung wird aus locked_offset-Vorzeichen abgeleitet, NICHT aus Encoder).
#   RETURN: /obstacle/return_omega publizieren → control_lane_node ersetzt PID-omega.
#   Stopp: Kamera (|lane_error| < threshold, N Frames) primär; Encoder als Backup.
#
# Stufe 6 – Sonderfall anhalten:
#   EVADE-Timeout → WAIT. /obstacle/stop = True → control_lane_node v=0.
#   WAIT: Korridor frei ODER wait_timeout → PASS.
#
# Offset-Konvention (identisch mit control_lane_node):
#   error > 0  → nach rechts lenken
#   neg. Offset → nach links ausweichen
#   pos. Offset → nach rechts ausweichen
# ─────────────────────────────────────────────────────────────────────────────

import os
import rospy
from enum import Enum
from std_msgs.msg import Float64, Bool, Float32MultiArray
from duckietown_msgs.msg import WheelEncoderStamped
import util


class EvadeState(Enum):
    Idle   = 1
    Evade  = 2
    Wait   = 3   # Stufe 6: anhalten wenn EVADE-Timeout
    Pass   = 4
    Return = 5


class ControlObstacleNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        self._vehicle_name = os.environ['VEHICLE_NAME']

        # ── Zustand ───────────────────────────────────────────────────────────
        self.enable         = False
        self.state          = EvadeState.Idle
        self.current_offset = 0.0
        self.locked_offset  = 0.0    # Richtung beim EVADE-Eintritt eingefroren
        self.evade_start    = None
        self.pass_start     = None
        self.wait_start     = None
        self.return_stable  = 0
        self.return_omega_value = 0.0
        self.lane_error     = 0.0
        self.zones          = [0.0, 0.0, 0.0]
        self.duck_x         = -99.0
        self.corridor_occ   = []   # Lückenprofil von detect_lane_node (Stufe 4b)

        # ── Encoder-Zustand (Stufe 5) ─────────────────────────────────────────
        # data zählt bei JEDER Bewegung aufwärts – Richtung aus Fahrbefehl!
        self.left_ticks_prev     = None
        self.right_ticks_prev    = None
        self.pending_ticks_left  = 0.0
        self.pending_ticks_right = 0.0
        self.accumulated_ticks      = 0.0
        self.return_ticks_remaining = 0.0

        # Defaults (defensiv überschrieben durch JSON)
        self.active               = True
        self.evade_offset         = 0.6
        self.evade_offset_min     = 0.25
        self.nachlauf_secs        = 1.5
        self.evade_timeout_secs   = 5.0
        self.return_threshold     = 0.25
        self.return_stable_frames = 5
        self.return_omega         = 0.5
        self.wait_timeout_secs    = 3.0

        util.init_parameters(node_name, self.cbUpdateParameters)

        # ── Publisher ─────────────────────────────────────────────────────────
        self.pub_offset = rospy.Publisher(
            f'/{self._vehicle_name}/obstacle/error_offset', Float64, queue_size=1)
        self.pub_return_omega = rospy.Publisher(
            f'/{self._vehicle_name}/obstacle/return_omega', Float64, queue_size=1)
        self.pub_stop = rospy.Publisher(
            f'/{self._vehicle_name}/obstacle/stop', Bool, queue_size=1)
        self.pub_done = rospy.Publisher(
            f'/{self._vehicle_name}/obstacle/done', Bool, queue_size=1)

        # ── Subscriber ────────────────────────────────────────────────────────
        rospy.Subscriber(f'/{self._vehicle_name}/detect/zones',
                         Float32MultiArray,   self.cbZones,        queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/detect/duck',
                         Float64,             self.cbDuck,         queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/detect/corridor_occupancy',
                         Float32MultiArray,   self.cbCorridorOccupancy, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/detect/lane',
                         Float64,             self.cbLane,         queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/enable/obstacle',
                         Bool,                self.cbEnable,       queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/left_wheel_encoder_node/tick',
                         WheelEncoderStamped, self.cbEncoderLeft,  queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/right_wheel_encoder_node/tick',
                         WheelEncoderStamped, self.cbEncoderRight, queue_size=1)

        rospy.loginfo(f"[{node_name}] Bereit. EvadeState=Idle.")

    # ── Parameter (defensiv) ──────────────────────────────────────────────────

    def cbUpdateParameters(self, parameters):
        def g(group, key, default):
            try:
                return parameters[group][key]["default"]
            except (KeyError, TypeError):
                rospy.logwarn(f"[control_obstacle] {group}.{key} fehlt – nutze {default}")
                return default

        self.active               = int(g("evade", "active",               1))    == 1
        self.evade_offset         =     g("evade", "evade_offset",         0.6)
        self.evade_offset_min     =     g("evade", "evade_offset_min",     0.25)
        self.nachlauf_secs        =     g("evade", "nachlauf_secs",        1.5)
        self.evade_timeout_secs   =     g("evade", "evade_timeout_secs",   5.0)
        self.return_threshold     =     g("evade", "return_threshold",     0.25)
        self.return_stable_frames = int(g("evade", "return_stable_frames", 5))
        self.return_omega         =     g("evade", "return_omega",         0.5)
        self.wait_timeout_secs    =     g("evade", "wait_timeout_secs",    3.0)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def cbEnable(self, msg):
        self.enable = msg.data

    def cbZones(self, msg):
        self.zones = list(msg.data) if len(msg.data) >= 3 else [0.0, 0.0, 0.0]

    def cbDuck(self, msg):
        self.duck_x = msg.data

    def cbCorridorOccupancy(self, msg):
        self.corridor_occ = list(msg.data)

    def cbLane(self, msg):
        self.lane_error = msg.data

    def cbEncoderLeft(self, msg):
        if self.left_ticks_prev is not None:
            self.pending_ticks_left += msg.data - self.left_ticks_prev
        self.left_ticks_prev = msg.data

    def cbEncoderRight(self, msg):
        if self.right_ticks_prev is not None:
            self.pending_ticks_right += msg.data - self.right_ticks_prev
        self.right_ticks_prev = msg.data

    # ── Hilfsfunktionen ───────────────────────────────────────────────────────

    def _zones_active(self):
        """True wenn Zone nah ODER mittel belegt."""
        return self.zones[0] > 0.5 or self.zones[1] > 0.5

    def _find_best_gap(self, occ):
        """
        Sucht die breiteste zusammenhängende freie Lücke im Korridor-Belegungsprofil.
        Rückgabe: (center_frac, width_frac) in [0,1] über den Korridor,
        oder None wenn kein Bin frei ist (Korridor komplett belegt).
        """
        n = len(occ)
        if n == 0:
            return None
        best_width = 0
        best_start = 0
        i = 0
        while i < n:
            if occ[i] < 0.5:
                j = i
                while j < n and occ[j] < 0.5:
                    j += 1
                width = j - i
                if width > best_width:
                    best_width = width
                    best_start = i
                i = j
            else:
                i += 1
        if best_width == 0:
            return None
        center_bin = best_start + best_width / 2.0
        return (center_bin / n, best_width / n)

    def _offset_from_gap(self, gap):
        """
        Wandelt eine gefundene Lücke (center_frac, width_frac) in einen Ausweich-Offset:
        Richtung + Stärke proportional zum Abstand der Lückenmitte von der
        Korridormitte (0.5). Lücke am Rand → voller evade_offset, Lücke nahe der
        Mitte → evade_offset_min als Untergrenze (verhindert ein zu schwaches
        Ausweichen bei einer knapp mittigen Lücke).
        """
        center_frac, _width_frac = gap
        center_signed = (center_frac - 0.5) * 2.0   # -1 (links) .. +1 (rechts)
        magnitude = max(self.evade_offset_min, min(abs(center_signed), 1.0) * self.evade_offset)
        return magnitude if center_signed >= 0.0 else -magnitude

    def _determine_direction(self):
        """
        Ausweichrichtung + -stärke beim EVADE-Eintritt:
          1. Primär: breiteste freie Lücke im Korridor-Belegungsprofil
             (/detect/corridor_occupancy) → Offset zeigt zur Lückenmitte,
             Stärke proportional zum Abstand von der Korridormitte.
          2. Fallback (kein Profil oder Korridor komplett belegt): alte
             duck_x-Heuristik – Objekt rechts → links ausweichen, sonst rechts.
        """
        gap = self._find_best_gap(self.corridor_occ) if self.corridor_occ else None
        if gap is not None:
            offset = self._offset_from_gap(gap)
            rospy.loginfo(
                f"[Evade] Luecke bei {gap[0]*100:.0f}% der Korridorbreite "
                f"(Breite {gap[1]*100:.0f}%) → Offset {offset:+.2f}")
            return offset

        rospy.logwarn("[Evade] Kein Profil / Korridor komplett belegt – Fallback auf duck_x")
        if self.duck_x != -99.0 and self.duck_x >= 0.0:
            return -self.evade_offset
        return +self.evade_offset

    def _consume_ticks(self):
        """Mittlere Ticks seit letztem Aufruf (beide Räder), Zähler zurücksetzen."""
        delta = (self.pending_ticks_left + self.pending_ticks_right) / 2.0
        self.pending_ticks_left  = 0.0
        self.pending_ticks_right = 0.0
        return delta

    # ── Zustandsautomat ───────────────────────────────────────────────────────

    def _step_state_machine(self):
        now         = rospy.Time.now()
        delta_ticks = self._consume_ticks()

        if self.state == EvadeState.Idle:
            self.current_offset     = 0.0
            self.return_stable      = 0
            self.accumulated_ticks  = 0.0
            if self._zones_active():
                self.locked_offset = self._determine_direction()
                self.evade_start   = now
                self.state         = EvadeState.Evade
                rospy.loginfo(
                    f"[Evade] Auslösung – "
                    f"{'links' if self.locked_offset < 0 else 'rechts'}, "
                    f"Offset {self.locked_offset:+.2f}")

        elif self.state == EvadeState.Evade:
            self.current_offset     = self.locked_offset
            self.accumulated_ticks += delta_ticks
            elapsed = (now - self.evade_start).to_sec()

            if elapsed > self.evade_timeout_secs:
                rospy.logwarn(f"[Evade] Timeout {elapsed:.1f}s → WAIT")
                self.wait_start = now
                self.state      = EvadeState.Wait
            elif not self._zones_active():
                rospy.loginfo("[Evade] Korridor frei → PASS")
                self.pass_start = now
                self.state      = EvadeState.Pass

        elif self.state == EvadeState.Wait:
            self.current_offset = 0.0   # kein Offset während Stillstand
            elapsed = (now - self.wait_start).to_sec()

            if not self._zones_active():
                rospy.loginfo("[Evade] WAIT: Korridor frei → PASS")
                self.pass_start = now
                self.state      = EvadeState.Pass
            elif elapsed >= self.wait_timeout_secs:
                rospy.logwarn("[Evade] WAIT Timeout – erzwinge PASS")
                self.pass_start = now
                self.state      = EvadeState.Pass

        elif self.state == EvadeState.Pass:
            self.current_offset     = self.locked_offset
            self.accumulated_ticks += delta_ticks
            elapsed = (now - self.pass_start).to_sec()

            if self._zones_active():
                rospy.loginfo("[Evade] PASS: Objekt wieder da → EVADE")
                self.evade_start = now
                self.state       = EvadeState.Evade
            elif elapsed >= self.nachlauf_secs:
                rospy.loginfo(
                    f"[Evade] Nachlauf vorbei → RETURN "
                    f"(akkum. {self.accumulated_ticks:.0f} Ticks)")
                self.return_ticks_remaining = self.accumulated_ticks
                self.return_stable          = 0
                self.state                  = EvadeState.Return

        elif self.state == EvadeState.Return:
            self.current_offset = 0.0
            self.return_ticks_remaining -= delta_ticks

            # Rückkehrrichtung: entgegengesetzt zur Ausweichrichtung
            return_dir = -1.0 if self.locked_offset > 0 else 1.0
            self.return_omega_value = self.return_omega * return_dir

            if self._zones_active():
                rospy.loginfo("[Evade] RETURN: neues Objekt → EVADE")
                self.return_omega_value = 0.0
                self.locked_offset      = self._determine_direction()
                self.evade_start        = now
                self.accumulated_ticks  = 0.0
                self.state              = EvadeState.Evade
                return

            if abs(self.lane_error) < self.return_threshold:
                self.return_stable += 1
            else:
                self.return_stable = 0

            camera_done  = self.return_stable >= self.return_stable_frames
            encoder_done = self.return_ticks_remaining <= 0

            if camera_done or encoder_done:
                reason = "Kamera" if camera_done else "Encoder-Backup"
                rospy.loginfo(f"[Evade] Rückkehr fertig ({reason}) → IDLE")
                self.return_omega_value = 0.0
                self.state              = EvadeState.Idle
                self.return_stable      = 0
                self.pub_done.publish(Bool(data=True))

    # ── Hauptschleife ─────────────────────────────────────────────────────────

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.enable and self.active:
                self._step_state_machine()
            else:
                self.current_offset     = 0.0
                self.return_omega_value = 0.0
                self.state              = EvadeState.Idle

            self.pub_offset.publish(Float64(data=self.current_offset))
            self.pub_return_omega.publish(Float64(data=self.return_omega_value))
            # Stop-Signal: True nur im WAIT-Zustand (Stufe 6)
            self.pub_stop.publish(Bool(data=(self.state == EvadeState.Wait)))

            rospy.loginfo_throttle(2.0,
                f"[Evade] {self.state.name}  "
                f"off={self.current_offset:+.2f}  "
                f"ret_ω={self.return_omega_value:+.2f}  "
                f"acc={self.accumulated_ticks:.0f}  "
                f"rem={self.return_ticks_remaining:.0f}  "
                f"zones={[int(z) for z in self.zones]}")
            rate.sleep()


if __name__ == '__main__':
    node = ControlObstacleNode('control_obstacle_node')
    node.run()
    rospy.spin()
