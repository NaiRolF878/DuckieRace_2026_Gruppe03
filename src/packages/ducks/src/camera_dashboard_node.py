#!/usr/bin/env python3

# ─────────────────────────────────────────────────────────────────────────────
# camera_dashboard_node.py
#
# Layout (2×2):
#   ┌──────────────────────┬──────────────────────┐
#   │  Original            │  Bird's-Eye-View     │
#   │  + Modus-Rahmen      │  + Spurmarkierungen  │
#   │  + Enten-Box         │  + ROI-Kasten        │
#   │  + Statuszeile       │                      │
#   ├──────────────────────┼──────────────────────┤
#   │  Gelbe Linie (Maske) │  Weiße Linie (Maske) │
#   └──────────────────────┴──────────────────────┘
#
# Diese Node ist bewusst die einzige Stelle, die noch Reste aus den älteren
# Challenges trägt (z.B. das Annotieren des Originalbilds). Für Challenge 3
# relevant: Modus (Following/Obstacle), rote Haltelinie, Enten-Position.
# ─────────────────────────────────────────────────────────────────────────────

import os
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64, Bool


class CameraDashboardNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        self._vehicle_name = os.environ['VEHICLE_NAME']

        # Kachelgröße – 300×300 → Gesamtfenster 600×600
        self._tile_size = (300, 300)

        # ── Platzhalter für die vier Kacheln ──────────────────────────────────
        self._img_original  = self._blank_tile("Warte auf Original...")
        self._img_annotated = self._blank_tile("Warte auf Bird's-Eye-View...")
        self._img_yellow    = self._blank_tile("Warte auf Gelb-Maske...")
        self._img_white     = self._blank_tile("Warte auf Weiss-Maske...")

        self._raw_original  = None

        # ── Letzte bekannte Detection-Werte ───────────────────────────────────
        self._duck_x          = -99.0   # -99 = keine Ente
        self._stop_line       = False
        self._enable_lane     = True
        self._enable_obstacle = False
        # Haltelinien-Zone im Original (grob, nur zur Anzeige)
        self.red_detection_zone = 0.65

        # ── Bild-Subscriber ───────────────────────────────────────────────────
        rospy.Subscriber(f'/{self._vehicle_name}/debug/original',
            CompressedImage, self._cb_original, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/debug/annotated',
            CompressedImage, self._cb_annotated, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/debug/lane_yellow',
            CompressedImage, self._cb_yellow, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/debug/lane_white',
            CompressedImage, self._cb_white, queue_size=1)

        # ── Detection-Subscriber ──────────────────────────────────────────────
        rospy.Subscriber(f'/{self._vehicle_name}/detect/duck',
            Float64, self._cb_duck, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/detect/stop_line',
            Bool, self._cb_stop_line, queue_size=1)

        # Enable-Topics von switch_control_node
        rospy.Subscriber(f'/{self._vehicle_name}/enable/lane',
            Bool, lambda m: setattr(self, '_enable_lane', m.data), queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/enable/obstacle',
            Bool, lambda m: setattr(self, '_enable_obstacle', m.data), queue_size=1)

        rospy.loginfo(f"[{node_name}] Dashboard gestartet.")


    # ── Hilfsfunktionen ───────────────────────────────────────────────────────

    def _blank_tile(self, label=""):
        tile = np.zeros((self._tile_size[1], self._tile_size[0], 3), dtype=np.uint8)
        if label:
            cv2.putText(tile, label, (10, self._tile_size[1] // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180, 180, 180), 1)
        return tile

    def _decode(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def _to_tile(self, img, label):
        tile = cv2.resize(img, self._tile_size)
        if len(tile.shape) == 2:
            tile = cv2.cvtColor(tile, cv2.COLOR_GRAY2BGR)
        cv2.putText(tile, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(tile, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (255, 255, 255), 1, cv2.LINE_AA)
        return tile

    def _mode_label(self):
        return "Obstacle" if self._enable_obstacle else "Following"

    def _mode_color(self):
        # Grün = Lane, Orange = Obstacle
        return (0, 165, 255) if self._enable_obstacle else (0, 255, 0)


    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _cb_original(self, msg):
        img = self._decode(msg)
        if img is not None:
            self._raw_original = img

    def _cb_annotated(self, msg):
        img = self._decode(msg)
        if img is not None:
            self._img_annotated = self._to_tile(img, "Bird's-Eye-View")

    def _cb_yellow(self, msg):
        img = self._decode(msg)
        if img is not None:
            self._img_yellow = self._to_tile(img, "Gelbe Linie")

    def _cb_white(self, msg):
        img = self._decode(msg)
        if img is not None:
            self._img_white = self._to_tile(img, "Weisse Linie")

    def _cb_duck(self, msg):
        self._duck_x = msg.data

    def _cb_stop_line(self, msg):
        self._stop_line = msg.data


    # ── Originalbild annotieren ───────────────────────────────────────────────

    def _annotate_original(self, img):
        annotated  = img.copy()
        h, w       = annotated.shape[:2]
        mode_color = self._mode_color()

        # 1. Modus-Rahmen
        cv2.rectangle(annotated, (0, 0), (w - 1, h - 1), mode_color, 6)

        # 2. Statuszeile
        cv2.rectangle(annotated, (0, 0), (w, 32), (0, 0, 0), -1)
        cv2.putText(annotated, self._mode_label(),
                    (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.65, mode_color, 2, cv2.LINE_AA)
        stop_color = (0, 0, 255) if self._stop_line else (160, 160, 160)
        stop_text  = "STOP" if self._stop_line else "frei"
        cv2.putText(annotated, f"Linie: {stop_text}",
                    (w - 150, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, stop_color, 2, cv2.LINE_AA)

        # 3. Rote Haltelinie: Box am unteren Bildrand
        if self._stop_line:
            box_top = int(h * self.red_detection_zone)
            cv2.rectangle(annotated, (0, box_top), (w - 1, h - 1), (0, 0, 255), 3)
            label_y = box_top - 6 if box_top > 20 else box_top + 18
            cv2.rectangle(annotated, (0, label_y - 18), (170, label_y + 4), (0, 0, 0), -1)
            cv2.putText(annotated, "ROTE LINIE!",
                        (4, label_y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2, cv2.LINE_AA)

        # 4. Ente: Kreis + Bounding-Box
        if self._duck_x != -99.0:
            duck_x_px = int((self._duck_x + 1) / 2 * w)
            duck_y_px = int(h * 0.65)
            radius    = 50
            cv2.circle(annotated, (duck_x_px, duck_y_px), radius, (0, 165, 255), 2)
            cv2.rectangle(annotated,
                (duck_x_px - radius, duck_y_px - radius),
                (duck_x_px + radius, duck_y_px + radius),
                (0, 0, 255), 2)
            label_y = max(duck_y_px - radius - 6, 42)
            cv2.rectangle(annotated,
                (duck_x_px - radius, label_y - 18),
                (duck_x_px - radius + 80, label_y + 4), (0, 0, 0), -1)
            cv2.putText(annotated, "ENTE!",
                (duck_x_px - radius, label_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2, cv2.LINE_AA)

        return annotated


    # ── Hauptschleife ─────────────────────────────────────────────────────────

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self._raw_original is not None:
                annotated_orig     = self._annotate_original(self._raw_original)
                self._img_original = self._to_tile(annotated_orig, "")

            top_row    = np.hstack([self._img_original, self._img_annotated])
            bottom_row = np.hstack([self._img_yellow,   self._img_white])
            dashboard  = np.vstack([top_row, bottom_row])

            h, w = dashboard.shape[:2]
            cv2.line(dashboard, (w // 2, 0), (w // 2, h), (255, 255, 255), 2)
            cv2.line(dashboard, (0, h // 2), (w, h // 2), (255, 255, 255), 2)

            cv2.imshow("Camera Dashboard", dashboard)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            rate.sleep()

        cv2.destroyAllWindows()


if __name__ == '__main__':
    node = CameraDashboardNode('camera_dashboard_node')
    node.run()
    rospy.spin()
