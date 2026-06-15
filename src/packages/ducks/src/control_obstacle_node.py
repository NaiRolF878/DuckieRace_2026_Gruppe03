#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────────────────────
# control_obstacle_node.py  (Challenge 3 – Watch out for Ducks)
#
# Wählt das Ausweichverhalten anhand des BELEGUNGSPROFILS von detect_duck_node
# und reicht das Ergebnis als additiven Lenk-Offset an control_lane_node weiter.
#
# Kombi-Strategie (in dieser Reihenfolge):
#   1. Freier RAND breit genug?  → Feld außen umfahren (bevorzugt, ruhig).
#   2. Sonst breiteste innere LÜCKE breit genug? → dort durchzielen.
#   3. Sonst kein Platz → Gegenspurübernahme (großer Offset, links).
#
# Offset-Konvention (wie control_lane_node):
#   error > 0 → nach rechts lenken. NEGATIVER Offset → Bot weicht LINKS aus.
#
# EvadeState: Idle → Evading → Returning → Idle.
#   Manöver wird nach letzter Sichtung noch evade_hold s gehalten (Feld wandert
#   beim Ausweichen aus dem Bild). Zielrichtung wird dabei eingefroren.
#   Rückkehr: Offset-Rampe auf 0 (kamerabasiert, selbstkorrigierend).
#
# Stufen-Schalter "active": steht er auf 0, gibt die Node immer Offset 0 aus
#   (zum isolierten Testen von Lane+Erkennung ohne Ausweichen).
# Defensiv gegen fehlende JSON-Keys.
# ─────────────────────────────────────────────────────────────────────────────

import os
import rospy
import numpy as np
from enum import Enum
from std_msgs.msg import Float64, Bool, Float32MultiArray
import util


class EvadeState(Enum):
    Idle      = 1
    Evading   = 2
    Returning = 3


class ControlObstacleNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        self._vehicle_name = os.environ['VEHICLE_NAME']

        # ── Zustand ───────────────────────────────────────────────────────────
        self.enable         = False         # von switch_control (/enable/obstacle)
        self.state          = EvadeState.Idle
        self.current_offset = 0.0
        self.target_offset  = 0.0
        self.evade_desc     = 'none'
        self.occupancy      = None
        self.last_duck_seen = None

        # Defaults (defensiv überschrieben)
        self.active          = True
        self.evade_offset    = 0.6
        self.oncoming_offset = 1.0
        self.ramp_step       = 0.05
        self.evade_hold      = 2.0
        self.gap_min_bins    = 6
        self.edge_min_bins   = 5

        util.init_parameters(node_name, self.cbUpdateParameters)

        # ── Publisher ─────────────────────────────────────────────────────────
        self.pub_offset = rospy.Publisher(
            f'/{self._vehicle_name}/obstacle/error_offset', Float64, queue_size=1)
        self.pub_done = rospy.Publisher(
            f'/{self._vehicle_name}/obstacle/done', Bool, queue_size=1)

        # ── Subscriber ────────────────────────────────────────────────────────
        rospy.Subscriber(f'/{self._vehicle_name}/detect/duck_occupancy',
                         Float32MultiArray, self.cbOccupancy, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/detect/duck',
                         Float64, self.cbDuck, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/enable/obstacle',
                         Bool, self.cbEnable, queue_size=1)

        rospy.loginfo(f"[{node_name}] Bereit. EvadeState=Idle.")

    # ── Parameter (defensiv) ─────────────────────────────────────────────────────

    def cbUpdateParameters(self, parameters):
        def g(group, key, default):
            try:
                return parameters[group][key]["default"]
            except (KeyError, TypeError):
                rospy.logwarn(f"[control_obstacle] Parameter {group}.{key} fehlt – nutze {default}")
                return default

        self.active          = int(g("evade", "active", 1)) == 1
        self.evade_offset    = g("evade", "evade_offset", 0.6)
        self.oncoming_offset = g("evade", "oncoming_offset", 1.0)
        self.ramp_step       = g("evade", "ramp_step", 0.05)
        self.evade_hold      = g("evade", "evade_hold", 2.0)
        self.gap_min_bins    = int(g("evade", "gap_min_bins", 6))
        self.edge_min_bins   = int(g("evade", "edge_min_bins", 5))

    # ── Callbacks ───────────────────────────────────────────────────────────────

    def cbEnable(self, msg):
        self.enable = msg.data

    def cbOccupancy(self, msg):
        self.occupancy = np.array(msg.data, dtype=np.float32)

    def cbDuck(self, msg):
        if msg.data != -99.0:
            self.last_duck_seen = rospy.Time.now()

    # ── Lückenanalyse ────────────────────────────────────────────────────────────

    def _free_intervals(self, occ):
        free = []
        n = len(occ); i = 0
        while i < n:
            if occ[i] < 0.5:
                j = i
                while j < n and occ[j] < 0.5:
                    j += 1
                free.append((i, j)); i = j
            else:
                i += 1
        return free

    def _decide_from_occupancy(self):
        occ = self.occupancy
        if occ is None or occ.sum() == 0:
            return None
        n = len(occ)
        center = n / 2.0
        free = self._free_intervals(occ)
        if not free:
            return 'gegenspur', -self.oncoming_offset

        # 1. Freier Rand
        left_edge  = free[0][1] - free[0][0] if free[0][0] == 0 else 0
        right_edge = free[-1][1] - free[-1][0] if free[-1][1] == n else 0
        if max(left_edge, right_edge) >= self.edge_min_bins:
            if right_edge >= left_edge:
                return f'rand-rechts({right_edge})', +self.evade_offset
            return f'rand-links({left_edge})', -self.evade_offset

        # 2. Innere Lücke (die dem Zentrum nächste ausreichende → wenig Lenken)
        usable = [(a, b) for (a, b) in free if (b - a) >= self.gap_min_bins]
        if usable:
            best = min(usable, key=lambda iv: abs((iv[0] + iv[1]) / 2.0 - center))
            gap_center = (best[0] + best[1]) / 2.0
            rel = (gap_center - center) / center
            return (f'luecke[{best[0]}:{best[1]}]',
                    float(np.clip(rel, -1.0, 1.0)) * self.evade_offset)

        # 3. Kein Platz → Gegenspur
        return 'gegenspur', -self.oncoming_offset

    # ── Rampe / Hilfen ──────────────────────────────────────────────────────────

    def _ramp_toward(self, target):
        diff = target - self.current_offset
        if abs(diff) <= self.ramp_step:
            self.current_offset = target
        else:
            self.current_offset += self.ramp_step * (1 if diff > 0 else -1)

    def _duck_in_way(self):
        if self.last_duck_seen is None:
            return False
        return (rospy.Time.now() - self.last_duck_seen).to_sec() < self.evade_hold

    # ── Zustandsautomat ─────────────────────────────────────────────────────────

    def _step_state_machine(self):
        if self.state == EvadeState.Idle:
            self.current_offset = 0.0
            if self._duck_in_way():
                decision = self._decide_from_occupancy()
                if decision is not None:
                    self.evade_desc, self.target_offset = decision
                    self.state = EvadeState.Evading
                    rospy.loginfo(f"[Evade] Feld erkannt → {self.evade_desc}, "
                                  f"Ziel-Offset {self.target_offset:+.2f}")

        elif self.state == EvadeState.Evading:
            if self._duck_in_way():
                if self.occupancy is not None and self.occupancy.sum() > 0:
                    decision = self._decide_from_occupancy()
                    if decision is not None:
                        self.evade_desc, self.target_offset = decision
                self._ramp_toward(self.target_offset)
            else:
                rospy.loginfo("[Evade] Feld passiert → kontrollierte Rueckkehr.")
                self.state = EvadeState.Returning

        elif self.state == EvadeState.Returning:
            if self._duck_in_way():
                self.state = EvadeState.Evading
                return
            self._ramp_toward(0.0)
            if abs(self.current_offset) < 1e-6:
                self.current_offset = 0.0
                self.state = EvadeState.Idle
                self.pub_done.publish(Bool(data=True))
                rospy.loginfo("[Evade] Rueckkehr abgeschlossen → Idle, /obstacle/done.")

    # ── Hauptschleife ──────────────────────────────────────────────────────────

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.enable and self.active:
                self._step_state_machine()
            else:
                self.current_offset = 0.0
                self.state = EvadeState.Idle
            self.pub_offset.publish(Float64(data=self.current_offset))
            rate.sleep()


if __name__ == '__main__':
    node = ControlObstacleNode('control_obstacle_node')
    node.run()
    rospy.spin()
