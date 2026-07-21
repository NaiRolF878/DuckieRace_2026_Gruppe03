#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────────────────────
# control_intersection_node.py  (Challenge 4 – Mapping & Path Finding)
#
# Faehrt die Kreuzung. Aktiv nur wenn /enable/intersection == True.
# Die Phase kommt von switch_control_node ueber /intersection/phase:
#
#   Stopping    – stehen bleiben (v=0)
#   Turning     – Abbiege-SEQUENZ abfahren
#
# Zeitbasierte Segmente (wie Challenge 2 / intersection_handling), NICHT
# encoder-basiert: Segment-JSON {"v":..., "omega":..., "duration":...},
# fertig sobald die kumulierte Segmentdauer verstrichen ist. Einfacher und
# robuster als die vorherige Encoder-Variante, die auf dieser Strecke nicht
# zuverlaessig funktionierte.
# ─────────────────────────────────────────────────────────────────────────────

import os
import rospy
from std_msgs.msg import Bool, String
from duckietown_msgs.msg import Twist2DStamped
import util


class ControlIntersectionNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        self._vehicle_name = os.environ['VEHICLE_NAME']

        self.enable     = False
        self.phase      = "Lane"
        self.direction  = "straight"
        self.turn_segments = {"left": [], "right": [], "straight": []}

        self._turn_active     = False
        self._turn_start      = None
        self._last_phase      = "Lane"
        self._turn_done_sent  = False

        util.init_parameters(node_name, self.cbUpdateParameters)

        rospy.Subscriber(f'/{self._vehicle_name}/enable/intersection',
                         Bool, self.cbEnable, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/intersection/phase',
                         String, self.cbPhase, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/intersection/direction',
                         String, self.cbDirection, queue_size=1)

        self.pub_cmd = rospy.Publisher(
            f'/{self._vehicle_name}/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        self.pub_turn_done = rospy.Publisher(
            f'/{self._vehicle_name}/intersection/turn_done', Bool, queue_size=1)

        rospy.on_shutdown(self.fnShutDown)
        rospy.loginfo(f"[{node_name}] Bereit.")

    def cbUpdateParameters(self, parameters):
        seg = parameters.get("turn_segments", {})
        for d in ("left", "right", "straight"):
            self.turn_segments[d] = [
                (float(s["v"]), float(s["omega"]), float(s["duration"]))
                for s in seg.get(d, [])
            ]

    def cbEnable(self, msg):
        self.enable = msg.data

    def cbPhase(self, msg):
        self.phase = msg.data

    def cbDirection(self, msg):
        self.direction = msg.data

    def _segment_cmd(self):
        segments = self.turn_segments.get(self.direction, [])
        if not segments:
            return 0.0, 0.0, True
        elapsed = (rospy.Time.now() - self._turn_start).to_sec()
        acc = 0.0
        for v, omega, dur in segments:
            acc += dur
            if elapsed < acc:
                return v, omega, False
        return 0.0, 0.0, True

    def _compute_cmd(self):
        if self.phase == "Turning":
            v, omega, done = self._segment_cmd()
            return v, omega, done
        # Stopping oder Lane -> Motor aus
        return 0.0, 0.0, False

    def fnShutDown(self):
        self.pub_cmd.publish(Twist2DStamped(v=0.0, omega=0.0))

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.phase == "Turning" and self._last_phase != "Turning":
                self._turn_start = rospy.Time.now()
                self._turn_done_sent = False
                rospy.loginfo(f"[control_intersection] Starte Sequenz: {self.direction}")
            self._last_phase = self.phase

            if self.enable:
                v, omega, done = self._compute_cmd()
                twist = Twist2DStamped()
                twist.header.stamp = rospy.Time.now()
                twist.v     = v
                twist.omega = omega
                self.pub_cmd.publish(twist)

                if self.phase == "Turning" and done and not self._turn_done_sent:
                    self.pub_turn_done.publish(Bool(data=True))
                    self._turn_done_sent = True
                    rospy.loginfo("[control_intersection] Sequenz fertig -> turn_done")
            rate.sleep()


if __name__ == '__main__':
    node = ControlIntersectionNode('control_intersection_node')
    node.run()
    rospy.spin()
