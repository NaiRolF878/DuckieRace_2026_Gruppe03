#!/usr/bin/env python3

# ─────────────────────────────────────────────────────────────────────────────
# camera_dashboard_node.py
#
# Layout:
#   ┌──────────────────────┬──────────────────────┐
#   │  Original            │  Bird's-Eye-View     │
#   │  + Modus-Rahmen      │  + Spurmarkierungen  │
#   │  + AprilTag-Box      │  + ROI-Kasten        │
#   │  + Statuszeile       │                      │
#   ├──────────────────────┼──────────────────────┤
#   │  Gelbe Linie (Maske) │  Weiße Linie (Maske) │
#   └──────────────────────┴──────────────────────┘
#
# Das Originalbild oben links kombiniert alle Detection-Infos auf einem Bild:
#   - Modus-Rahmen (Farbe je nach Modus)
#   - Tag-ID-Label (/detect/apriltag)
#   - Statuszeile mit Modus + Rote-Linie-Info
# ─────────────────────────────────────────────────────────────────────────────

import os
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32, Bool, String


class CameraDashboardNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        self._vehicle_name = os.environ['VEHICLE_NAME']

        # Kachelgröße im Dashboard – 300×300 → Gesamtfenster 600×600
        self._tile_size = (300, 300)

        # ── Platzhalter für die vier Kacheln ──────────────────────────────────
        self._img_original  = self._blank_tile("Warte auf Original...")
        self._img_annotated = self._blank_tile("Warte auf Bird's-Eye-View...")
        self._img_yellow    = self._blank_tile("Warte auf Gelb-Maske...")
        self._img_white     = self._blank_tile("Warte auf Weiss-Maske...")

        # Rohes Originalbild (wird in run() annotiert)
        self._raw_original  = None

        # ── Letzte bekannte Detection-Werte ───────────────────────────────────
        self._apriltag_id      = -1       # -1 = kein Tag
        self._gate_id          = -1       # -1 = kein Tor sichtbar (Challenge 4, IDs 5-13)
        self._stop_line        = False
        self._enable_lane         = True
        self._enable_intersection = False
        self._enable_obstacle     = False
        self._chosen_direction    = '-'      # gewuerfelte Abbiegerichtung (FSM)
        self._phase               = 'Lane'   # aktuelle FSM-Phase
        # Haltelinien-Detektionszone: untere 15% des Bildes (identisch zu detect_lane_node default)
        self.red_detection_zone   = 0.65  # entspricht grob h*0.65 im Originalbild

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
        rospy.Subscriber(f'/{self._vehicle_name}/detect/apriltag',
            Int32, self._cb_apriltag, queue_size=1)
        # Tor-Tag (5-13, Challenge 4) - separat von der Kreuzungs-Tag-ID oben
        rospy.Subscriber(f'/{self._vehicle_name}/detect/gate/id',
            Int32, lambda m: setattr(self, '_gate_id', m.data), queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/detect/stop_line',
            Bool, self._cb_stop_line, queue_size=1)

        # Enable-Topics von switch_control_node
        rospy.Subscriber(f'/{self._vehicle_name}/enable/lane',
            Bool, lambda m: setattr(self, '_enable_lane', m.data), queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/enable/intersection',
            Bool, lambda m: setattr(self, '_enable_intersection', m.data), queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/enable/obstacle',
            Bool, lambda m: setattr(self, '_enable_obstacle', m.data), queue_size=1)

        # Kreuzung: gewuerfelte Richtung + aktuelle Phase von switch_control_node
        rospy.Subscriber(f'/{self._vehicle_name}/intersection/direction',
            String, lambda m: setattr(self, '_chosen_direction', m.data if m.data else '-'), queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/intersection/phase',
            String, lambda m: setattr(self, '_phase', m.data if m.data else '-'), queue_size=1)

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
        if self._enable_intersection:
            return "Intersection"
        if self._enable_obstacle:
            return "Obstacle"
        return "Following"

    def _mode_color(self):
        # Grün = Lane, Orange = Obstacle, Magenta = Intersection
        if self._enable_intersection:
            return (255, 0, 255)
        if self._enable_obstacle:
            return (0, 165, 255)
        return (0, 255, 0)


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

    def _cb_apriltag(self, msg):
        self._apriltag_id = msg.data

    def _cb_stop_line(self, msg):
        self._stop_line = msg.data


    # ── Originalbild annotieren ───────────────────────────────────────────────

    def _annotate_original(self, img):
        annotated  = img.copy()
        h, w       = annotated.shape[:2]
        mode_color = self._mode_color()

        # ── 1. Modus-Rahmen um das gesamte Bild ──────────────────────────────
        cv2.rectangle(annotated, (0, 0), (w - 1, h - 1), mode_color, 6)

        # ── 2. Statuszeile oben ───────────────────────────────────────────────
        # Schwarzer Balken – Beschriftung startet bei y=22 damit der Modus-Rahmen
        # (Dicke 6) nicht überlappt; daher Balken bei y=0..32
        cv2.rectangle(annotated, (0, 0), (w, 32), (0, 0, 0), -1)
        # Modus links
        cv2.putText(annotated, self._mode_label(),
                    (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.65, mode_color, 2, cv2.LINE_AA)
        # Rote Linie rechts
        stop_color = (0, 0, 255) if self._stop_line else (160, 160, 160)
        cv2.putText(annotated, f"Linie: {'JA' if self._stop_line else 'NEIN'}",
                    (w - 180, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, stop_color, 2, cv2.LINE_AA)

        # ── 2b. Zweite Statuszeile: FSM-Phase + gewuerfelte Abbiegerichtung ───
        cv2.rectangle(annotated, (0, 32), (w, 60), (0, 0, 0), -1)
        cv2.putText(annotated, f"Phase: {self._phase}",
                    (10, 53), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2, cv2.LINE_AA)
        dir_txt = self._chosen_direction.upper() if self._chosen_direction else "-"
        cv2.putText(annotated, f"FAHRE: {dir_txt}",
                    (w - 180, 53), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)

        # ── 2c. Dritte Statuszeile: Tor-Tag-Erkennung (Challenge 4, IDs 5-13) ──
        cv2.rectangle(annotated, (0, 60), (w, 88), (0, 0, 0), -1)
        gate_color = (255, 0, 255) if self._gate_id != -1 else (120, 120, 120)
        gate_txt = f"TOR: {self._gate_id}" if self._gate_id != -1 else "TOR: -"
        cv2.putText(annotated, gate_txt,
                    (10, 81), cv2.FONT_HERSHEY_SIMPLEX, 0.6, gate_color, 2, cv2.LINE_AA)

        # ── 3. Rote Haltelinie: Bounding-Box am unteren Bildrand ─────────────
        if self._stop_line:
            # Roter Kasten über die volle Breite im unteren Drittel des Originalbilds
            box_top = int(h * self.red_detection_zone) if hasattr(self, 'red_detection_zone') else int(h * 0.65)
            cv2.rectangle(annotated, (0, box_top), (w - 1, h - 1), (0, 0, 255), 3)
            label_y = box_top - 6 if box_top > 20 else box_top + 18
            cv2.rectangle(annotated, (0, label_y - 18), (170, label_y + 4), (0, 0, 0), -1)
            cv2.putText(annotated, "ROTE LINIE!",
                        (4, label_y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2, cv2.LINE_AA)

        # ── 4. AprilTag: ID-Label (keine Eckpunkte verfuegbar, nur die ID) ─────
        if self._apriltag_id != -1:
            cv2.rectangle(annotated, (10, 96), (140, 118), (0, 0, 0), -1)
            cv2.putText(annotated, f"Tag ID: {self._apriltag_id}",
                        (14, 112), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (0, 255, 0), 2, cv2.LINE_AA)

        return annotated


    # ── Hauptschleife ─────────────────────────────────────────────────────────

    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            # Originalbild annotieren und als Kachel speichern
            if self._raw_original is not None:
                annotated_orig      = self._annotate_original(self._raw_original)
                self._img_original  = self._to_tile(annotated_orig, "")

            # 2×2 Grid zusammenbauen
            top_row    = np.hstack([self._img_original, self._img_annotated])
            bottom_row = np.hstack([self._img_yellow,   self._img_white])
            dashboard  = np.vstack([top_row, bottom_row])

            # Trennlinien
            h, w = dashboard.shape[:2]
            cv2.line(dashboard, (w // 2, 0), (w // 2, h), (255, 255, 255), 2)
            cv2.line(dashboard, (0, h // 2), (w, h // 2), (255, 255, 255), 2)

            cv2.imshow("Camera Dashboard", dashboard)

            # Einzelfenster: auskommentiert, bei Bedarf aktivieren
            # cv2.imshow("Original annotiert", self._img_original)
            # cv2.imshow("Bird's-Eye-View",    self._img_annotated)
            # cv2.imshow("Gelbe Linie",        self._img_yellow)
            # cv2.imshow("Weisse Linie",       self._img_white)

            # q → schließen
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            rate.sleep()

        cv2.destroyAllWindows()


if __name__ == '__main__':
    node = CameraDashboardNode('camera_dashboard_node')
    node.run()
    rospy.spin()
