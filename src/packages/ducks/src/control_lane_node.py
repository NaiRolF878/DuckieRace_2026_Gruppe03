#!/usr/bin/env python3

# ─────────────────────────────────────────────────────────────────────────────
# control_lane_node.py
#
# Challenge 1 (Lane Following + rote Haltelinie) als Basis.
# Challenge-3-Erweiterung: additiver Ausweich-Offset von control_obstacle_node.
#
# Diese Node bleibt die EINZIGE Stelle, die PID rechnet, Geschwindigkeit
# reduziert und den Haltelinien-Automaten hält. control_obstacle_node liefert
# nur /obstacle/error_offset, der hier zum Spurfehler addiert wird.
# ─────────────────────────────────────────────────────────────────────────────

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
    def __init__(self, node_name):
        rospy.init_node(node_name)

        # Steuerung aktiv? Wird durch switch_control_node gesetzt
        self.enable = True

        self._vehicle_name = os.environ['VEHICLE_NAME']

        # Parameter aus JSON laden + Live-Update-Callback registrieren
        util.init_parameters(node_name, self.cbUpdateParameters)

        # Publisher für Fahrbefehle
        twist_topic = f"/{self._vehicle_name}/car_cmd_switch_node/cmd"
        self.pub_cmd_vel = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)

        # Subscriber: Spurversatz von detect_lane_node
        detect_lane_topic = f"/{self._vehicle_name}/detect/lane"
        self.sub_lane = rospy.Subscriber(
            detect_lane_topic, Float64, self.cbFollowLane, queue_size=1)

        # Subscriber: Enable von switch_control_node
        self.sub_enable = rospy.Subscriber(
            f'/{self._vehicle_name}/enable/lane', Bool, self.cbControl, queue_size=1)

        # Subscriber: rote Haltelinie von detect_lane_node
        stop_line_topic = f"/{self._vehicle_name}/detect/stop_line"
        self.sub_stop_line = rospy.Subscriber(
            stop_line_topic, Bool, self.cbStopLine, queue_size=1)

        # Challenge 3: additiver Ausweich-Offset von control_obstacle_node
        self.error_offset  = 0.0
        self.return_omega  = 0.0    # Stufe 5: Encoder-Rückkehr überschreibt PID-omega
        self.obstacle_stop = False  # Stufe 6: WAIT-Zustand → v=0
        self.sub_error_offset = rospy.Subscriber(
            f'/{self._vehicle_name}/obstacle/error_offset',
            Float64, self.cbErrorOffset, queue_size=1)
        self.sub_return_omega = rospy.Subscriber(
            f'/{self._vehicle_name}/obstacle/return_omega',
            Float64, self.cbReturnOmega, queue_size=1)
        self.sub_obstacle_stop = rospy.Subscriber(
            f'/{self._vehicle_name}/obstacle/stop',
            Bool, self.cbObstacleStop, queue_size=1)

        # PID-Variablen
        self.lastError = 0
        self.integral  = 0
        self.dt        = 0.1

        # Steuerwerte
        self.v = 0
        self.a = 0

        # Haltelinien-Automat
        self.stop_state        = StopState.Driving
        self.STOP_DURATION     = 3.0
        self.COOLDOWN_DURATION = 3.0
        self.stop_start_time   = None

        rospy.on_shutdown(self.fnShutDown)
        rospy.loginfo(f"[{node_name}] Bereit. Warte auf Spurversatz ...")

#-------------------------------
# Callbacks
#-------------------------------

    def cbControl(self, msg):
        self.enable = msg.data

    def cbErrorOffset(self, msg):
        self.error_offset = msg.data

    def cbReturnOmega(self, msg):
        self.return_omega = msg.data

    def cbObstacleStop(self, msg):
        self.obstacle_stop = msg.data

    def cbUpdateParameters(self, parameters):
        self.kp      = parameters["pid"]["p"]["default"]
        self.ki      = parameters["pid"]["i"]["default"]
        self.kd      = parameters["pid"]["d"]["default"]
        self.MAX_VEL = parameters["pid"]["max_vel"]["default"]
        self.MIN_VEL = parameters["pid"]["min_vel"]["default"]
        self.STOP_DURATION     = parameters["stop_line"]["stop_duration"]["default"]
        self.COOLDOWN_DURATION = parameters["stop_line"]["cooldown_duration"]["default"]

    def cbStopLine(self, msg):
        if self.stop_state == StopState.Cooldown:
            return
        if msg.data and self.stop_state == StopState.Driving:
            rospy.loginfo("Rote Haltelinie erkannt – halte an.")
            self.stop_state      = StopState.Stopping
            self.stop_start_time = rospy.Time.now()

    # Spurversatz error ∈ [-1, +1]:
    #   error > 0 → Bot zu weit links  → nach rechts lenken
    #   error < 0 → Bot zu weit rechts → nach links lenken
    def cbFollowLane(self, error):
        if self.stop_state == StopState.Stopping:
            self.v = 0.0
            self.a = 0.0
            return

        error = error.data

        # NEU: Ausweich-Offset addieren → verschiebt wahrgenommene Spurmitte
        error = error + self.error_offset
        # Begrenzung damit PID nicht übersteuert (etwas weiter wegen Offset)
        error = max(min(error, 2.0), -2.0)

        # PID
        P = self.kp * error
        self.integral += error * self.dt
        I = self.ki * self.integral
        derivative = (error - self.lastError) / self.dt
        D = self.kd * derivative

        self.a = P + I + D
        self.a = max(min(self.a, 3), -3)

        # Geschwindigkeit fehlerabhängig reduzieren, MIN_VEL als Untergrenze
        self.v = max(self.MIN_VEL, self.MAX_VEL * (1 - abs(error)))

        self.lastError = error

    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")
        twist = Twist2DStamped(v=0.0, omega=0.0)
        self.pub_cmd_vel.publish(twist)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.enable:
                twist = Twist2DStamped()
                twist.header.stamp = rospy.Time.now()

                # Stufe 6: WAIT-Signal von control_obstacle_node → vollständiger Stopp
                if self.obstacle_stop:
                    twist.v     = 0.0
                    twist.omega = 0.0

                elif self.stop_state == StopState.Stopping:
                    elapsed = (rospy.Time.now() - self.stop_start_time).to_sec()
                    if elapsed < self.STOP_DURATION:
                        twist.v     = 0.0
                        twist.omega = 0.0
                    else:
                        rospy.loginfo("Stopp vorbei – fahre weiter.")
                        self.stop_state      = StopState.Cooldown
                        self.stop_start_time = rospy.Time.now()
                        self.integral        = 0
                        twist.v     = self.v
                        # Stufe 5: Encoder-Rückkehr überschreibt PID-omega
                        twist.omega = self.return_omega if self.return_omega != 0.0 else self.a

                elif self.stop_state == StopState.Cooldown:
                    elapsed = (rospy.Time.now() - self.stop_start_time).to_sec()
                    if elapsed >= self.COOLDOWN_DURATION:
                        rospy.loginfo("Cooldown beendet – Haltelinien-Erkennung wieder aktiv.")
                        self.stop_state = StopState.Driving
                    twist.v     = self.v
                    twist.omega = self.return_omega if self.return_omega != 0.0 else self.a

                else:
                    twist.v     = self.v
                    # Stufe 5: Encoder-Rückkehr überschreibt PID-omega (nur im RETURN-Zustand aktiv)
                    twist.omega = self.return_omega if self.return_omega != 0.0 else self.a

                self.pub_cmd_vel.publish(twist)

            rate.sleep()


if __name__ == '__main__':
    node = ControlLaneNode('control_lane_node')
    node.run()
    rospy.spin()
