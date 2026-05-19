#!/usr/bin/env python3

# ─────────────────────────────────────────────────────────────────────────────
# camera_dashboard_node.py
#
# Aufgabe: Zeigt ein 2×2 Dashboard aller Kameraansichten in einem einzigen
#          OpenCV-Fenster an.
#
# Layout:
#   ┌──────────────────────┬──────────────────────┐
#   │  Original            │  Bird's-Eye-View     │
#   │  + AprilTag Box      │  + Spurmarkierungen  │
#   │  + Enten Box         │  + ROI-Kasten        │
#   │  + Status-Infos      │                      │
#   ├──────────────────────┼──────────────────────┤
#   │  Gelbe Linie         │  Weiße Linie         │
#   │  (HSV-Maske)         │  (HSV-Maske)         │
#   └──────────────────────┴──────────────────────┘
#
# Abonniert (Bilder):
#   /debug/original    (CompressedImage) → Rohes Kamerabild
#   /debug/annotated   (CompressedImage) → Bird's-Eye-View mit Spurmarkierungen
#   /debug/lane_yellow (CompressedImage) → Gelbe Linien-Maske
#   /debug/lane_white  (CompressedImage) → Weiße Linien-Maske
#
# Abonniert (Detection-Topics für Annotierung auf Originalbild):
#   /detect/apriltag       (Int32)   → AprilTag-ID (-1 = kein Tag)
#   /detect/duck           (Float64) → Enten-Position (-99 = keine Ente)
#   /detect/stop_line      (Bool)    → Rote Linie erkannt?
#   /detect/stop_line_side (String)  → Seite der roten Linie
#   /switch/control        (Int32)   → Aktiver Steuerungsmodus
# ─────────────────────────────────────────────────────────────────────────────

import os
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32, Float64, Bool, String


class CameraDashboardNode:
    def __init__(self, node_name):
        # ROS-Node initialisieren
        rospy.init_node(node_name)

        # Fahrzeugnamen aus Umgebungsvariable lesen
        self._vehicle_name = os.environ['VEHICLE_NAME']

        # Einheitliche Größe jeder Kachel im Dashboard in Pixel
        self._tile_size = (400, 400)

        # Platzhalter für die vier Bilder
        self._img_original = self._blank_tile("Warte auf Original...")
        self._img_annotated = self._blank_tile("Warte auf Bird's-Eye-View...")
        self._img_yellow   = self._blank_tile("Warte auf Gelb-Maske...")
        self._img_white    = self._blank_tile("Warte auf Weiss-Maske...")

        # Letztes rohes Originalbild (wird für Annotierung benötigt)
        self._raw_original = None

        # Letzte bekannte Detection-Werte für Annotierung auf Originalbild
        self._apriltag_id       = -1       # -1 = kein Tag
        self._duck_x            = -99.0    # -99 = keine Ente
        self._stop_line         = False
        self._stop_line_side    = 'none'
        self._control_mode      = 1        # 1=Lane, 2=Obstacle, 3=Intersection

        # ── Bild-Subscriber ───────────────────────────────────────────────────
        rospy.Subscriber(f'/{self._vehicle_name}/debug/original',
            CompressedImage, self._cb_original, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/debug/annotated',
            CompressedImage, self._cb_annotated, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/debug/lane_yellow',
            CompressedImage, self._cb_yellow, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/debug/lane_white',
            CompressedImage, self._cb_white, queue_size=1)

        # ── Detection-Subscriber (für Annotierung auf Originalbild) ───────────
        rospy.Subscriber(f'/{self._vehicle_name}/detect/apriltag',
            Int32, self._cb_apriltag, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/detect/duck',
            Float64, self._cb_duck, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/detect/stop_line',
            Bool, self._cb_stop_line, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/detect/stop_line_side',
            String, self._cb_stop_line_side, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/switch/control',
            Int32, self._cb_control, queue_size=1)

        rospy.loginfo(f"[{node_name}] Dashboard gestartet - warte auf Bilder...")


    # ── Hilfsfunktionen ───────────────────────────────────────────────────────

    def _blank_tile(self, label=""):
        # Schwarze Kachel mit Platzhaltertext
        tile = np.zeros((self._tile_size[1], self._tile_size[0], 3), dtype=np.uint8)
        if label:
            cv2.putText(tile, label, (10, self._tile_size[1] // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180, 180, 180), 1)
        return tile

    def _decode(self, msg):
        # Komprimiertes JPEG → OpenCV BGR-Bild
        np_arr = np.frombuffer(msg.data, np.uint8)
        return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def _to_tile(self, img, label):
        # Bild auf Kachelgröße skalieren und Label einzeichnen
        tile = cv2.resize(img, self._tile_size)
        if len(tile.shape) == 2:
            tile = cv2.cvtColor(tile, cv2.COLOR_GRAY2BGR)
        # Label: schwarzer Hintergrund + weißer Text → lesbar auf allen Bildern
        cv2.putText(tile, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(tile, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1, cv2.LINE_AA)
        return tile

    def _mode_label(self):
        # Steuerungsmodus als Text
        modes = {1: "Lane Following", 2: "Obstacle", 3: "Intersection"}
        return modes.get(self._control_mode, "Unknown")

    def _mode_color(self):
        # Farbe je nach Steuerungsmodus
        colors = {1: (0, 255, 0), 2: (0, 165, 255), 3: (255, 0, 255)}
        return colors.get(self._control_mode, (255, 255, 255))


    # ── Bild-Callbacks ────────────────────────────────────────────────────────

    def _cb_original(self, msg):
        # Rohes Kamerabild speichern → wird in run() mit Annotierungen versehen
        img = self._decode(msg)
        if img is not None:
            self._raw_original = img

    def _cb_annotated(self, msg):
        # Bird's-Eye-View mit Spurmarkierungen von detect_lane_node
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
            self._img_white = self._to_tile(img, "Weiße Linie")


    # ── Detection-Callbacks ───────────────────────────────────────────────────

    def _cb_apriltag(self, msg):
        self._apriltag_id = msg.data

    def _cb_duck(self, msg):
        self._duck_x = msg.data

    def _cb_stop_line(self, msg):
        self._stop_line = msg.data

    def _cb_stop_line_side(self, msg):
        self._stop_line_side = msg.data

    def _cb_control(self, msg):
        self._control_mode = msg.data


    # ── Originalbild annotieren ───────────────────────────────────────────────

    def _annotate_original(self, img):
        # Kopie des Originalbildes mit allen Detection-Infos annotieren
        annotated = img.copy()
        h, w = annotated.shape[:2]

        # ── Steuerungsmodus (oben links) ──────────────────────────────────────
        mode_label = self._mode_label()
        mode_color = self._mode_color()
        # Hintergrundbalken für bessere Lesbarkeit
        cv2.rectangle(annotated, (0, 0), (w, 35), (0, 0, 0), -1)
        cv2.putText(annotated, f"Modus: {mode_label}",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, mode_color, 2, cv2.LINE_AA)

        # ── Rote Linie Status (oben rechts) ───────────────────────────────────
        stop_color = (0, 0, 255) if self._stop_line else (100, 100, 100)
        stop_label = f"Rote Linie: {self._stop_line_side}"
        cv2.putText(annotated, stop_label,
                    (w - 280, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, stop_color, 2, cv2.LINE_AA)

        # ── AprilTag Bounding-Box ─────────────────────────────────────────────
        if self._apriltag_id != -1:
            # Grüner Rahmen am Bildrand wenn Tag erkannt
            cv2.rectangle(annotated, (2, 2), (w - 2, h - 2), (0, 255, 0), 3)
            cv2.putText(annotated, f"AprilTag ID: {self._apriltag_id}",
                        (10, h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                        (0, 255, 0), 2, cv2.LINE_AA)

        # ── Enten Bounding-Box ────────────────────────────────────────────────
        if self._duck_x != -99.0:
            # x-Position normiert [-1,+1] → Pixel
            duck_x_pixel = int((self._duck_x + 1) / 2 * w)
            # Geschätzter Kreis um die Ente
            radius = 40
            cv2.circle(annotated, (duck_x_pixel, int(h * 0.7)), radius, (0, 165, 255), 2)
            cv2.rectangle(annotated,
                          (duck_x_pixel - radius, int(h * 0.7) - radius),
                          (duck_x_pixel + radius, int(h * 0.7) + radius),
                          (0, 0, 255), 2)
            cv2.putText(annotated, "ENTE!",
                        (duck_x_pixel - radius, int(h * 0.7) - radius - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)

        return annotated


    # ── Hauptschleife ─────────────────────────────────────────────────────────

    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            # Originalbild mit Detection-Infos annotieren
            if self._raw_original is not None:
                annotated_orig = self._annotate_original(self._raw_original)
                self._img_original = self._to_tile(annotated_orig, "Original")

            # 2×2 Grid zusammenbauen
            top_row    = np.hstack([self._img_original, self._img_annotated])
            bottom_row = np.hstack([self._img_yellow,   self._img_white])
            dashboard  = np.vstack([top_row, bottom_row])

            # Trennlinien einzeichnen
            h, w = dashboard.shape[:2]
            cv2.line(dashboard, (w // 2, 0), (w // 2, h), (255, 255, 255), 2)
            cv2.line(dashboard, (0, h // 2), (w, h // 2), (255, 255, 255), 2)

            cv2.imshow("Camera Dashboard", dashboard)

            # Separate Einzelfenster: auskommentiert, bei Bedarf aktivieren
            # cv2.imshow("Original annotiert", self._img_original)
            # cv2.imshow("Bird's-Eye-View",    self._img_annotated)
            # cv2.imshow("Gelbe Linie",        self._img_yellow)
            # cv2.imshow("Weiße Linie",       self._img_white)

            # q drücken → Dashboard schließen
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            rate.sleep()

        cv2.destroyAllWindows()


if __name__ == '__main__':
    node = CameraDashboardNode('camera_dashboard_node')
    node.run()
    rospy.spin()
