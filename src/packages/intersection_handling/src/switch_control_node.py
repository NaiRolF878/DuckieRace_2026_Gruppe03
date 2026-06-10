#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────────────────────
# switch_control_node.py  (Challenge 2 – Intersection Handling)
#
# Zentraler Zustandsautomat (FSM). 
#
# Phasen:
#   Lane        – normales Spurfolgen (control_lane aktiv)
#   Stopping    – an der Haltelinie warten
#   Turning     – abbiegen (Sequenz wird von control_intersection gesteuert)
# ─────────────────────────────────────────────────────────────────────────────

import os
import random
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
            
        self.direction = random.choice(self.allowed_dirs)
        rospy.loginfo(f"[switch] Kreuzung (Linie+Tag) -> STOPPING | "
                      f"Richtung: {self.direction} (aus {self.allowed_dirs})")
        self._transition_to(self.STOPPING)

    def cbApriltagDirection(self, msg):
        if self.phase == self.LANE and msg.data and msg.data != "unknown":
            self.allowed_dirs = msg.data.split(",")

    def cbTurnDone(self, msg):
        if msg.data:
            self.turn_done = True

    def _transition_to(self, new_phase):
        self.phase            = new_phase
        self.phase_start_time = rospy.Time.now()
        if new_phase == self.TURNING:
            self.turn_done = False
        rospy.loginfo(f"[switch] -> {new_phase}")

    def _update_state(self):
        elapsed = (rospy.Time.now() - self.phase_start_time).to_sec()

        if self.phase == self.STOPPING:
            if elapsed >= self.stop_duration:
                rospy.loginfo(f"[switch] Stopp beendet ({self.stop_duration:.1f}s) -> TURNING")
                self._transition_to(self.TURNING)

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
            self.pub_phase.publish(String(data=self.phase))
            self.pub_direction.publish(String(data=self.direction))

            rate.sleep()


if __name__ == '__main__':
    node = SwitchControlNode('switch_control_node')
    node.run()
    rospy.spin()
