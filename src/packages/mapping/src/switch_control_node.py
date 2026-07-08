#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────────────────────
# switch_control_node.py  (Challenge 4 – Mapping & Path Finding, urspr. Challenge 2)
#
# Zentraler Zustandsautomat (FSM).
#
# Phasen:
#   Lane        – normales Spurfolgen (control_lane aktiv)
#   Stopping    – an der Haltelinie warten
#   Turning     – abbiegen (Sequenz wird von control_intersection gesteuert)
#
# Aenderung gegenueber Challenge 2: Die Richtung kommt nicht mehr per
# random.choice(allowed_dirs), sondern von explore_control_node/path_planner_node
# ueber /navigation/next_direction (deterministischer Pfad). Ist next_direction
# nicht (mehr) in allowed_dirs enthalten, bleibt der Bot in STOPPING stehen und
# wartet weiter - kein Fallback auf random. Deshalb wird die Richtungspruefung
# aus cbStopLine in _update_state verschoben: sie muss bei jedem STOPPING-Tick
# neu versucht werden (next_direction kann erst NACH dem Anhalten eintreffen),
# nicht nur einmalig beim Uebergang aus LANE.
# ─────────────────────────────────────────────────────────────────────────────

import os
import rospy
from std_msgs.msg import Bool, String
import util


class SwitchControlNode:
    LANE        = "Lane"
    STOPPING    = "Stopping"
    TURNING     = "Turning"

    def __init__(self, node_name):
        rospy.init_node(node_name)
        self._vehicle_name = os.environ['VEHICLE_NAME']

        self.phase            = self.LANE
        self.phase_start_time = rospy.Time.now()
        self.allowed_dirs     = []
        self.direction        = "straight"
        self.next_direction   = ""     # von explore_control_node/path_planner_node
        self.stop_line        = False
        self.turn_done        = False

        # Timing-Defaults
        self.stop_duration   = 2.0
        self.turning_timeout = 8.0

        util.init_parameters(node_name, self.cbUpdateParameters)

        # ── Publisher ─────────────────────────────────────────────────────────
        self.pub_enable_lane = rospy.Publisher(
            f'/{self._vehicle_name}/enable/lane', Bool, queue_size=1)
        self.pub_enable_inter = rospy.Publisher(
            f'/{self._vehicle_name}/enable/intersection', Bool, queue_size=1)
        self.pub_phase = rospy.Publisher(
            f'/{self._vehicle_name}/intersection/phase', String, queue_size=1)
        self.pub_direction = rospy.Publisher(
            f'/{self._vehicle_name}/intersection/direction', String, queue_size=1)

        # ── Subscriber ────────────────────────────────────────────────────────
        rospy.Subscriber(f'/{self._vehicle_name}/detect/stop_line',
                         Bool, self.cbStopLine, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/detect/apriltag/direction',
                         String, self.cbApriltagDirection, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/intersection/turn_done',
                         Bool, self.cbTurnDone, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/navigation/next_direction',
                         String, self.cbNextDirection, queue_size=1)

        rospy.loginfo(f"[{node_name}] Bereit – Zustand: Lane")

    def cbUpdateParameters(self, parameters):
        # Safely get parameters to avoid KeyErrors
        timing = parameters.get("timing", {})
        
        if "stop_duration" in timing:
            self.stop_duration = timing["stop_duration"].get("default", 2.0)
        
        if "turning_timeout" in timing:
            self.turning_timeout = timing["turning_timeout"].get("default", 8.0)

    def cbStopLine(self, msg):
        self.stop_line = msg.data

        if not (msg.data and self.phase == self.LANE):
            return

        if not self.allowed_dirs or self.allowed_dirs == ["unknown"]:
            rospy.loginfo_throttle(2.0,
                "[switch] Rote Linie ohne Tag-Richtung -> keine Kreuzung, fahre weiter")
            return

        rospy.loginfo(f"[switch] Kreuzung (Linie+Tag) -> STOPPING | "
                      f"erlaubte Richtungen: {self.allowed_dirs}")
        self._transition_to(self.STOPPING)

    def cbApriltagDirection(self, msg):
        if self.phase == self.LANE and msg.data and msg.data != "unknown":
            self.allowed_dirs = msg.data.split(",")

    def cbTurnDone(self, msg):
        if msg.data:
            self.turn_done = True

    def cbNextDirection(self, msg):
        self.next_direction = msg.data

    def _transition_to(self, new_phase):
        self.phase            = new_phase
        self.phase_start_time = rospy.Time.now()
        if new_phase == self.TURNING:
            self.turn_done = False
        rospy.loginfo(f"[switch] -> {new_phase}")

    def _update_state(self):
        elapsed = (rospy.Time.now() - self.phase_start_time).to_sec()

        if self.phase == self.STOPPING:
            if elapsed < self.stop_duration:
                return
            if self.next_direction and self.next_direction in self.allowed_dirs:
                self.direction = self.next_direction
                rospy.loginfo(f"[switch] Richtung bestaetigt: {self.direction} "
                              f"(aus {self.allowed_dirs}) -> TURNING")
                self._transition_to(self.TURNING)
            else:
                rospy.logwarn_throttle(1.0,
                    f"[switch] Keine gueltige next_direction ('{self.next_direction}') "
                    f"in erlaubten Richtungen {self.allowed_dirs} - bleibe in STOPPING")

        elif self.phase == self.TURNING:
            if self.turn_done or elapsed >= self.turning_timeout:
                rospy.loginfo("[switch] Turning fertig -> LANE")
                self.allowed_dirs = []
                self._transition_to(self.LANE)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self._update_state()

            lane_active = (self.phase == self.LANE)
            self.pub_enable_lane.publish(Bool(data=lane_active))
            self.pub_enable_inter.publish(Bool(data=not lane_active))
            # direction VOR phase publizieren: graph_state_node loest seinen
            # Graph-Uebergang am Wechsel zu "Turning" aus und liest dabei die
            # zuletzt empfangene direction - die muss also garantiert schon
            # die neue (gerade bestaetigte) Richtung sein.
            self.pub_direction.publish(String(data=self.direction))
            self.pub_phase.publish(String(data=self.phase))

            rate.sleep()


if __name__ == '__main__':
    node = SwitchControlNode('switch_control_node')
    node.run()
    rospy.spin()
