#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────────────────────
# camera_dashboard_node.py  (Challenge 3 – Watch out for Ducks)
#
# Schlankes 2x2-Dashboard:
#   [ Bird's-Eye-View ] [ Enten-BEV (Belegung) ]
#   [ Rote Maske      ] [ Weisse Maske         ]
#
# ─────────────────────────────────────────────────────────────────────────────

import os
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64, Bool, String


class CameraDashboardNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._tile = (300, 300)

        self._img_bev    = self._blank("Warte auf Bird's-Eye-View ...")
        self._img_duck   = self._blank("Warte auf Enten-BEV ...")
        self._img_red    = self._blank("Warte auf Rot-Maske ...")
        self._img_white  = self._blank("Warte auf Weiss-Maske ...")

        self._duck_x        = -99.0
        self._stop_line     = False
        self._obstacle_state = "Idle"

        rospy.Subscriber(f'/{self._vehicle_name}/debug/bird_view',
                         CompressedImage, self._cb_bev, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/debug/duck_bev',
                         CompressedImage, self._cb_duck, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/debug/lane_red',
                         CompressedImage, self._cb_red, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/debug/lane_white',
                         CompressedImage, self._cb_white, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/detect/duck',
                         Float64, self._cb_duck_x, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/detect/stop_line',
                         Bool, self._cb_stop, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/obstacle/state',
                         String, lambda m: setattr(self, '_obstacle_state', m.data), queue_size=1)

        rospy.loginfo(f"[{node_name}] Dashboard gestartet.")

    def _blank(self, label=""):
        t = np.zeros((self._tile[1], self._tile[0], 3), dtype=np.uint8)
        if label:
            cv2.putText(t, label, (10, self._tile[1] // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)
        return t

    def _decode(self, msg):
        return cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)

    def _to_tile(self, img, label):
        t = cv2.resize(img, self._tile)
        if len(t.shape) == 2:
            t = cv2.cvtColor(t, cv2.COLOR_GRAY2BGR)
        cv2.putText(t, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(t, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1, cv2.LINE_AA)
        return t

    def _cb_bev(self, m):
        img = self._decode(m)
        if img is not None: self._img_bev = self._to_tile(img, "BEV")
    def _cb_duck(self, m):
        img = self._decode(m)
        if img is not None: self._img_duck = self._to_tile(img, "Enten-BEV")
    def _cb_red(self, m):
        img = self._decode(m)
        if img is not None: self._img_red = self._to_tile(img, "Rot")
    def _cb_white(self, m):
        img = self._decode(m)
        if img is not None: self._img_white = self._to_tile(img, "Weiss")
    def _cb_duck_x(self, m): self._duck_x = m.data
    def _cb_stop(self, m):   self._stop_line = m.data

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            top = np.hstack([self._img_bev, self._img_duck])
            bot = np.hstack([self._img_red, self._img_white])
            dash = np.vstack([top, bot])

            state_colors = {
                "Idle":   (0, 255, 0),
                "Evade":  (0, 165, 255),
                "Wait":   (0, 0, 255),
                "Pass":   (0, 165, 255),
                "Return": (255, 165, 0),
            }
            mcol = state_colors.get(self._obstacle_state, (255, 255, 255))
            cv2.putText(dash, self._obstacle_state.upper(), (10, dash.shape[0] - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, mcol, 2, cv2.LINE_AA)
            if self._stop_line:
                cv2.putText(dash, "STOP-LINIE", (140, dash.shape[0] - 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)
            if self._duck_x != -99.0:
                cv2.putText(dash, f"Ente x={self._duck_x:+.2f}", (300, dash.shape[0] - 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2, cv2.LINE_AA)

            h, w = dash.shape[:2]
            cv2.line(dash, (w // 2, 0), (w // 2, h), (255, 255, 255), 1)
            cv2.line(dash, (0, h // 2), (w, h // 2), (255, 255, 255), 1)

            cv2.imshow("Camera Dashboard", dash)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            rate.sleep()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    node = CameraDashboardNode('camera_dashboard_node')
    node.run()
