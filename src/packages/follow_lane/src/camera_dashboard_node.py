#!/usr/bin/env python3

# ─────────────────────────────────────────────────────────────────────────────
# camera_dashboard_node.py
#
# Aufgabe: Zeigt ein 2×2 Dashboard aller Kameraansichten in einem einzigen
#          OpenCV-Fenster an. Ersetzt das bisherige cv2.imshow in detect_lane_node.
#
# Layout:
#   ┌─────────────────┬─────────────────┐
#   │  Original       │  Bird's-Eye-View│
#   │  (Kamerabild)   │  (transformiert)│
#   ├─────────────────┼─────────────────┤
#   │  Gelbe Linie    │  Weiße Linie    │
#   │  (HSV-Maske)    │  (HSV-Maske)    │
#   └─────────────────┴─────────────────┘
#
# Abonniert:
#   /debug/original    (CompressedImage) → Rohes Kamerabild
#   /debug/bird_view   (CompressedImage) → Bird's-Eye-View
#   /debug/lane_yellow (CompressedImage) → Gelbe Linien-Maske
#   /debug/lane_white  (CompressedImage) → Weiße Linien-Maske
# ─────────────────────────────────────────────────────────────────────────────

import os
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage


class CameraDashboardNode:
    def __init__(self, node_name):
        # ROS-Node initialisieren
        rospy.init_node(node_name)

        # Fahrzeugnamen aus Umgebungsvariable lesen
        self._vehicle_name = os.environ['VEHICLE_NAME']

        # Einheitliche Größe jeder Kachel im Dashboard in Pixel
        # → alle Bilder werden auf diese Größe skaliert
        self._tile_size = (400, 400)

        # Platzhalter für jedes der vier Bilder
        # → werden mit schwarzem Bild initialisiert bis erste Message ankommt
        self._img_original = self._blank_tile("Warte auf Original...")
        self._img_bird     = self._blank_tile("Warte auf Bird View...")
        self._img_yellow   = self._blank_tile("Warte auf Gelb-Maske...")
        self._img_white    = self._blank_tile("Warte auf Weiss-Maske...")

        # Subscriber für alle vier Kameraansichten
        rospy.Subscriber(
            f'/{self._vehicle_name}/debug/original',
            CompressedImage,
            self._cb_original,
            queue_size=1
        )
        rospy.Subscriber(
            f'/{self._vehicle_name}/debug/bird_view',
            CompressedImage,
            self._cb_bird,
            queue_size=1
        )
        rospy.Subscriber(
            f'/{self._vehicle_name}/debug/lane_yellow',
            CompressedImage,
            self._cb_yellow,
            queue_size=1
        )
        rospy.Subscriber(
            f'/{self._vehicle_name}/debug/lane_white',
            CompressedImage,
            self._cb_white,
            queue_size=1
        )

        rospy.loginfo(f"[{node_name}] Dashboard gestartet - warte auf Bilder...")


    # ── Hilfsfunktionen ───────────────────────────────────────────────────────

    def _blank_tile(self, label=""):
        # Erzeugt eine schwarze Kachel mit zentriertem Label-Text
        # → wird als Platzhalter verwendet bevor das erste Bild ankommt
        tile = np.zeros((self._tile_size[1], self._tile_size[0], 3), dtype=np.uint8)
        if label:
            cv2.putText(
                tile, label,
                (10, self._tile_size[1] // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180, 180, 180), 1
            )
        return tile

    def _decode(self, msg):
        # Komprimiertes JPEG-Bild aus ROS-Message in OpenCV BGR-Bild dekodieren
        np_arr = np.frombuffer(msg.data, np.uint8)
        return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def _to_tile(self, img, label):
        # Bild auf einheitliche Kachelgrösse skalieren
        tile = cv2.resize(img, self._tile_size)

        # Wenn Graustufenbild (Masken) → in BGR konvertieren damit Dashboard einheitlich ist
        if len(tile.shape) == 2:
            tile = cv2.cvtColor(tile, cv2.COLOR_GRAY2BGR)

        # Label oben links einzeichnen: erst schwarzer Hintergrund, dann weisser Text
        # → gute Lesbarkeit auf hellen und dunklen Bildern
        cv2.putText(tile, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,   0,   0  ), 3, cv2.LINE_AA)
        cv2.putText(tile, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1, cv2.LINE_AA)
        return tile


    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _cb_original(self, msg):
        # Rohes Kamerabild empfangen und als Kachel speichern
        img = self._decode(msg)
        if img is not None:
            self._img_original = self._to_tile(img, "Original")

    def _cb_bird(self, msg):
        # Bird's-Eye-View empfangen und als Kachel speichern
        img = self._decode(msg)
        if img is not None:
            self._img_bird = self._to_tile(img, "Bird's-Eye-View")

    def _cb_yellow(self, msg):
        # Gelbe Linien-Maske empfangen und als Kachel speichern
        img = self._decode(msg)
        if img is not None:
            self._img_yellow = self._to_tile(img, "Gelbe Linie")

    def _cb_white(self, msg):
        # Weisse Linien-Maske empfangen und als Kachel speichern
        img = self._decode(msg)
        if img is not None:
            self._img_white = self._to_tile(img, "Weisse Linie")


    # ── Hauptschleife ─────────────────────────────────────────────────────────

    def run(self):
        # 10 Hz - Dashboard wird mit 10 FPS aktualisiert
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            # 2x2 Grid zusammenbauen:
            # np.hstack verbindet Bilder nebeneinander (horizontal)
            # np.vstack verbindet Bilder untereinander  (vertikal)
            top_row    = np.hstack([self._img_original, self._img_bird])
            bottom_row = np.hstack([self._img_yellow,   self._img_white])
            dashboard  = np.vstack([top_row, bottom_row])

            # Trennlinien zwischen den Kacheln einzeichnen (weiss, 2px)
            h, w = dashboard.shape[:2]
            cv2.line(dashboard, (w // 2, 0), (w // 2, h), (255, 255, 255), 2)  # vertikal
            cv2.line(dashboard, (0, h // 2), (w, h // 2), (255, 255, 255), 2)  # horizontal

            cv2.imshow("Camera Dashboard", dashboard)

            # q druecken → Dashboard schliessen
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            rate.sleep()

        cv2.destroyAllWindows()


if __name__ == '__main__':
    node = CameraDashboardNode('camera_dashboard_node')
    node.run()
    rospy.spin()
