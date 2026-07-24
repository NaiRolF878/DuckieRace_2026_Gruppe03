#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────────────────────
# control_obstacle_node.py  (Challenge 3 – Ausweich-Logik)
#
# Zustandsautomat: Idle (inkl. kontinuierlichem Mittel-Zonen-Ausweichen) ->
# Emergency (Nah-Zone, PID-Bypass) -> Return (kurze feste Geradeausfahrt) -> Idle
#
# Abgeloest: die frühere 6-Zustands-Maschine (Idle/Emergency/Evade/Wait/Pass/
# Return), abgeleitet aus dem in avoid_ducks bewaehrten Muster - die
# mittel-Zone braucht KEINEN eigenen Zustand mehr, sie wird bei jedem Tick neu
# als additiver PID-Offset berechnet (kein einmaliges Einfrieren der Richtung,
# kein Timeout, kein Nachlauf/Pass-Schritt). Nur die nah-Zone (Kollisionsgefahr,
# PID muss komplett umgangen werden) braucht weiterhin einen echten Zustand
# (Emergency) mit anschliessender kurzer Geradeausfahrt (Return), um sich vom
# Hindernis zu loesen, bevor wieder normal gelenkt wird.
#
# Gestrichen ggue. der Vorgaenger-Version:
#   - WAIT-Zustand (nur Timeout-Fallback zwischen Evade/Emergency und Pass)
#   - Encoder-Ruecklauf-Tracking (accumulated_ticks/return_ticks_remaining):
#     Ticks liefen waehrend EMERGENCY (Drehen auf der Stelle) mit ein, obwohl
#     Drehen kaum Vorwaertsbewegung erzeugt - das Rueckkehr-Ziel war dadurch
#     kein verlaessliches Mass fuer die tatsaechliche seitliche Auslenkung.
#     Ersetzt durch eine feste, kurze Geradeausfahrt (wie bei avoid_ducks'
#     DRIVE_FORWARD_DISTANCE) statt einer akkumulierten Distanz.
#   - /obstacle/return_omega (Teil-Override) und /obstacle/stop (WAIT):
#     Return nutzt denselben vollen PID-Bypass wie Emergency
#     (/obstacle/emergency_active + /obstacle/emergency_cmd), control_lane_node
#     braucht dadurch keinen dritten Override-Mechanismus mehr.
#
# Drei Zonen (/detect/zones [nah, mittel, fern]):
#   - fern:   nur Beobachtung, kein Eingriff
#   - mittel: kontinuierlicher PID-Offset (kein eigener Zustand)
#   - nah:    Emergency (PID-Bypass) -> Return
#
# Ausweichrichtung + -staerke: IMMER frisch aus /detect/corridor_occupancy
# berechnet (_determine_direction) - auch waehrend Emergency bei jedem Tick
# neu, nicht mehr einmalig beim Zustandseintritt eingefroren. Verhindert, dass
# ein Manoever auf Basis eines einzigen, unter Umstaenden veralteten Moments
# durchgezogen wird.
#
# Offset-Konvention (identisch mit control_lane_node):
#   error > 0  → nach rechts lenken
#   neg. Offset → nach links ausweichen
#   pos. Offset → nach rechts ausweichen
# ─────────────────────────────────────────────────────────────────────────────

import os
import rospy
from enum import Enum
from std_msgs.msg import Float64, Bool, Float32MultiArray, String
from duckietown_msgs.msg import Twist2DStamped
import util


class EvadeState(Enum):
    Idle      = 1   # Normalbetrieb - inkl. kontinuierlichem Mittel-Zonen-Offset
    Emergency = 2   # Nah-Zone: PID-Bypass, feste Drehrate + Wiggle
    Return    = 3   # kurze feste Geradeausfahrt nach Emergency, PID-Bypass


class ControlObstacleNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        self._vehicle_name = os.environ['VEHICLE_NAME']

        # ── Zustand ───────────────────────────────────────────────────────────
        self.state           = EvadeState.Idle
        self.current_offset  = 0.0
        self.emergency_start = None
        self.return_start    = None
        self.free_stable     = 0    # Zonen-frei-Zähler (Emergency → Return)
        self.zones            = [0.0, 0.0, 0.0]
        self.duck_x            = -99.0
        self.corridor_occ      = []   # Lückenprofil von detect_lane_node

        # ── Notfall/Rückkehr (PID-Bypass, siehe emergency_active/emergency_cmd) ──
        self.emergency_v         = 0.0
        self.emergency_omega     = 0.0
        self.wiggle_direction    = -1.0
        self.last_wiggle_time    = None
        self.emergency_direction = 1.0   # jeden Tick neu bestimmt, siehe _step

        # Defaults (defensiv überschrieben durch JSON)
        self.active                 = True
        self.evade_offset           = 0.6
        self.evade_offset_min       = 0.25
        self.free_stable_frames     = 5
        self.emergency_omega_rad    = 1.6
        self.emergency_timeout_secs = 5.0
        self.wiggle_interval_secs   = 0.06
        self.wiggle_power           = 0.07
        self.return_forward_secs    = 1.0
        self.return_forward_speed   = 0.15

        util.init_parameters(node_name, self.cbUpdateParameters)

        # ── Publisher ─────────────────────────────────────────────────────────
        self.pub_offset = rospy.Publisher(
            f'/{self._vehicle_name}/obstacle/error_offset', Float64, queue_size=1)
        self.pub_done = rospy.Publisher(
            f'/{self._vehicle_name}/obstacle/done', Bool, queue_size=1)
        # Aktueller Zustand (Idle/Emergency/Return) – für Debug-Overlays
        self.pub_state = rospy.Publisher(
            f'/{self._vehicle_name}/obstacle/state', String, queue_size=1)
        # PID-Bypass: control_lane_node übernimmt v/omega 1:1, sobald True –
        # gilt jetzt für Emergency UND Return gleichermaßen.
        self.pub_emergency_active = rospy.Publisher(
            f'/{self._vehicle_name}/obstacle/emergency_active', Bool, queue_size=1)
        self.pub_emergency_cmd = rospy.Publisher(
            f'/{self._vehicle_name}/obstacle/emergency_cmd', Twist2DStamped, queue_size=1)

        # ── Subscriber ────────────────────────────────────────────────────────
        rospy.Subscriber(f'/{self._vehicle_name}/detect/zones',
                         Float32MultiArray,   self.cbZones,        queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/detect/duck',
                         Float64,             self.cbDuck,         queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/detect/corridor_occupancy',
                         Float32MultiArray,   self.cbCorridorOccupancy, queue_size=1)

        rospy.loginfo(f"[{node_name}] Bereit. EvadeState=Idle.")

    # ── Parameter (defensiv) ──────────────────────────────────────────────────

    def cbUpdateParameters(self, parameters):
        def g(group, key, default):
            try:
                return parameters[group][key]["default"]
            except (KeyError, TypeError):
                rospy.logwarn(f"[control_obstacle] {group}.{key} fehlt – nutze {default}")
                return default

        self.active                 = int(g("evade", "active",               1))    == 1
        self.evade_offset           =     g("evade", "evade_offset",         0.6)
        self.evade_offset_min       =     g("evade", "evade_offset_min",     0.25)
        self.free_stable_frames     = int(g("evade", "free_stable_frames",   5))
        self.emergency_omega_rad    =     g("evade", "emergency_omega_rad",   1.6)
        self.emergency_timeout_secs =     g("evade", "emergency_timeout_secs", 5.0)
        self.wiggle_interval_secs   =     g("evade", "wiggle_interval_secs",  0.06)
        self.wiggle_power           =     g("evade", "wiggle_power",          0.07)
        self.return_forward_secs    =     g("evade", "return_forward_secs",   1.0)
        self.return_forward_speed   =     g("evade", "return_forward_speed",  0.15)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def cbZones(self, msg):
        self.zones = list(msg.data) if len(msg.data) >= 3 else [0.0, 0.0, 0.0]

    def cbDuck(self, msg):
        self.duck_x = msg.data

    def cbCorridorOccupancy(self, msg):
        self.corridor_occ = list(msg.data)

    # ── Hilfsfunktionen ───────────────────────────────────────────────────────

    def _near_active(self):
        """True wenn NAH-Zone belegt (Notfall-Stufe)."""
        return self.zones[0] > 0.5

    def _mid_active(self):
        """True wenn MITTEL-Zone belegt (kontinuierlicher Offset, kein eigener Zustand)."""
        return self.zones[1] > 0.5

    def _find_best_gap(self, occ):
        """
        Sucht NICHT die breiteste Lücke zwischen zwei Hindernissen, sondern
        vergleicht den freien Abstand vom linken bzw. rechten Korridorrand bis
        zum nächstgelegenen Hindernis - und weicht zur Seite mit mehr Abstand
        aus. Verhindert ein Durchquetschen durch eine schmale Lücke zwischen
        zwei Hindernissen; der Bot fährt stattdessen immer außen an allen
        Hindernissen einer Seite vorbei.
        occ: [left_frac, right_frac] - exakter freier Anteil der Korridorbreite
        von detect_lane_node (_corridor_gap_spacing).
        Rückgabe: (center_frac, width_frac) in [0,1] über den Korridor,
        oder None wenn der komplette Korridor belegt ist.
        """
        if len(occ) < 2:
            return None
        left_frac, right_frac = occ[0], occ[1]
        if left_frac <= 0.0 and right_frac <= 0.0:
            return None
        if left_frac >= right_frac:
            width_frac  = left_frac
            center_frac = left_frac / 2.0
        else:
            width_frac  = right_frac
            center_frac = 1.0 - right_frac / 2.0
        return (center_frac, width_frac)

    def _offset_from_gap(self, gap):
        """
        Wandelt eine gefundene Lücke (center_frac, width_frac) in einen Ausweich-Offset.
        Der Korridor entspricht der Bot-Breite - ein Hindernis irgendwo darin
        heißt grundsätzlich "so nicht durchfahrbar", AUSSER die freie Seite ist
        (fast) so breit wie der ganze Korridor. Die Stärke richtet sich daher
        nach width_frac: breite freie Seite → kleiner Offset reicht, schmale
        freie Seite → voller evade_offset.
        """
        center_frac, width_frac = gap
        center_signed = (center_frac - 0.5) * 2.0   # -1 (links) .. +1 (rechts)
        magnitude = self.evade_offset_min + (1.0 - min(width_frac, 1.0)) * (self.evade_offset - self.evade_offset_min)
        return magnitude if center_signed >= 0.0 else -magnitude

    def _determine_direction(self):
        """
        Ausweichrichtung + -stärke, JEDEN Tick frisch berechnet (siehe
        Kopfkommentar) statt beim Zustandseintritt einmalig eingefroren:
          1. Primär: Seite mit dem größeren freien Abstand vom Korridorrand
             bis zum nächsten Hindernis (/detect/corridor_occupancy).
          2. Fallback (kein Profil oder Korridor komplett belegt): alte
             duck_x-Heuristik – Objekt rechts → links ausweichen, sonst rechts.
        """
        gap = self._find_best_gap(self.corridor_occ) if self.corridor_occ else None
        if gap is not None:
            return self._offset_from_gap(gap)
        if self.duck_x != -99.0 and self.duck_x >= 0.0:
            return -self.evade_offset
        return +self.evade_offset

    # ── Zustandsautomat ───────────────────────────────────────────────────────

    def _step(self):
        now = rospy.Time.now()

        if self.state == EvadeState.Idle:
            # Mittel-Zone braucht keinen eigenen Zustand: solange sie belegt
            # ist, wird bei jedem Tick frisch ein PID-Offset berechnet - kein
            # Timeout, kein Nachlauf, keine Rückkehr-Logik nötig, weil der
            # Offset automatisch auf 0 zurückfällt, sobald die Zone frei ist.
            self.current_offset = self._determine_direction() if self._mid_active() else 0.0

            if self._near_active():
                rospy.logwarn("[Notfall] Hindernis in NAH-Zone – Nothalt + Drehung")
                self.emergency_direction = self._determine_direction()
                self.emergency_start  = now
                self.wiggle_direction = -1.0
                self.last_wiggle_time = now
                self.free_stable      = 0
                self.current_offset   = 0.0
                self.state            = EvadeState.Emergency

        elif self.state == EvadeState.Emergency:
            # Wiggle: v kippt im Wiggle-Intervall das Vorzeichen, damit der Bot
            # beim Drehen auf der Stelle nicht wegen Standreibung hängen bleibt.
            if (now - self.last_wiggle_time).to_sec() > self.wiggle_interval_secs:
                self.wiggle_direction *= -1.0
                self.last_wiggle_time = now
            self.emergency_v = self.wiggle_power * self.wiggle_direction
            # Richtung jeden Tick neu bestimmen statt nur einmal beim Eintritt.
            self.emergency_direction = self._determine_direction()
            self.emergency_omega = (self.emergency_omega_rad if self.emergency_direction >= 0.0
                                     else -self.emergency_omega_rad)

            if self._near_active():
                self.free_stable = 0
            else:
                self.free_stable += 1

            elapsed = (now - self.emergency_start).to_sec()
            timed_out = elapsed > self.emergency_timeout_secs
            if self.free_stable >= self.free_stable_frames or timed_out:
                if timed_out and self.free_stable < self.free_stable_frames:
                    rospy.logwarn(f"[Notfall] Timeout {elapsed:.1f}s – erzwinge RETURN")
                else:
                    rospy.loginfo(
                        f"[Notfall] NAH-Zone frei ({self.free_stable} Frames stabil) → RETURN")
                self.return_start = now
                self.state         = EvadeState.Return

        elif self.state == EvadeState.Return:
            # Kurze feste Geradeausfahrt, um sich physisch vom Hindernis zu
            # lösen - KEIN akkumuliertes Encoder-Ziel mehr (siehe Kopfkommentar).
            self.emergency_v     = self.return_forward_speed
            self.emergency_omega = 0.0

            if self._near_active():
                rospy.logwarn("[Notfall] RETURN: Hindernis wieder in NAH-Zone – zurück zu Emergency")
                self.emergency_direction = self._determine_direction()
                self.emergency_start  = now
                self.wiggle_direction = -1.0
                self.last_wiggle_time = now
                self.free_stable      = 0
                self.state            = EvadeState.Emergency
                return

            elapsed = (now - self.return_start).to_sec()
            if elapsed >= self.return_forward_secs:
                rospy.loginfo("[Notfall] Rückkehr fertig → Idle")
                self.current_offset = 0.0
                self.state           = EvadeState.Idle
                self.pub_done.publish(Bool(data=True))

    # ── Hauptschleife ─────────────────────────────────────────────────────────

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.active:
                self._step()
            else:
                self.current_offset  = 0.0
                self.emergency_v     = 0.0
                self.emergency_omega = 0.0
                self.state            = EvadeState.Idle

            # Gilt jetzt für Emergency UND Return (siehe Kopfkommentar).
            emergency_active = self.state in (EvadeState.Emergency, EvadeState.Return)

            self.pub_offset.publish(Float64(data=self.current_offset))
            self.pub_state.publish(String(data=self.state.name))
            self.pub_emergency_active.publish(Bool(data=emergency_active))
            emergency_cmd = Twist2DStamped()
            if emergency_active:
                emergency_cmd.v     = self.emergency_v
                emergency_cmd.omega = self.emergency_omega
            self.pub_emergency_cmd.publish(emergency_cmd)

            rospy.loginfo_throttle(2.0,
                f"[Evade] {self.state.name}  "
                f"off={self.current_offset:+.2f}  "
                f"zones={[int(z) for z in self.zones]}")
            rate.sleep()


if __name__ == '__main__':
    node = ControlObstacleNode('control_obstacle_node')
    node.run()
    rospy.spin()
