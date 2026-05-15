#!/usr/bin/env python3

# ─────────────────────────────────────────────────────────────────────────────
# detect_duck_node.py
#
# Aufgabe: Erkennt Enten auf der Fahrbahn durch Kombination zweier Ansätze:
#
#   Ansatz 1 – Helligkeitsprüfung (Bird's-Eye-View):
#     Der Fahrbahnbereich zwischen gelber und weißer Linie sollte schwarz sein.
#     Helle Pixel in diesem Bereich deuten auf ein Hindernis hin.
#     Morphologie (MORPH_OPEN) filtert kleine Spiegelungsartefakte heraus.
#
#   Ansatz 2 – Hough-Kreise (Originalbild):
#     Enten haben runde Formen. Im Originalbild sind Kreise nicht verzerrt
#     (Bird's-Eye-View würde Kreise zu Ellipsen verzerren).
#
#   Ente erkannt wenn BEIDE Ansätze gleichzeitig anschlagen.
#
#   Position der Ente (x) bestimmt die Ausweichrichtung in control_obstacle_node.
#
# Publiziert:
#   /detect/duck          (Float64)         → normierte x-Position [-1,+1], -99 = keine Ente
#   /debug/duck_detection (CompressedImage) → Debug-Bild mit Bounding-Box
# ─────────────────────────────────────────────────────────────────────────────

import os
import rospy
import numpy as np
import cv2
from std_msgs.msg import Float64
from sensor_msgs.msg import CompressedImage
import util


class DetectDuckNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        self._vehicle_name = os.environ['VEHICLE_NAME']
        util.init_parameters(node_name, self.cbUpdateParameters)

        self.sub_image = rospy.Subscriber(
            f'/{self._vehicle_name}/camera_node/image/compressed',
            CompressedImage, self.cbDetectDuck, queue_size=1)

        # Spurversatz: wird verwendet um Linienpositionen zu schätzen
        self.sub_lane = rospy.Subscriber(
            f'/{self._vehicle_name}/detect/lane',
            Float64, self.cbLane, queue_size=1)

        # x-Position der Ente normiert [-1,+1], -99 = keine Ente
        self.pub_duck = rospy.Publisher(
            f'/{self._vehicle_name}/detect/duck', Float64, queue_size=1)

        # Debug-Bild mit Bounding-Box um erkannte Enten
        self.pub_debug = rospy.Publisher(
            f'/{self._vehicle_name}/debug/duck_detection', CompressedImage, queue_size=1)

        self._crop_im_size = 400
        self.is_running    = False

        # Letzte bekannte Linienpositionen (Startwerte = Bildränder als Fallback)
        self.center_white  = int(self._crop_im_size * 0.95)
        self.center_yellow = int(self._crop_im_size * 0.05)

        rospy.loginfo(f"[{node_name}] Enten-Erkennung bereit.")


    def cbUpdateParameters(self, parameters):
        # Bird's-Eye-View Eckpunkte (identisch zu detect_lane_node)
        self.top_left_x     = parameters["crop_image"]["top_left_x"]["default"]
        self.top_left_y     = parameters["crop_image"]["top_left_y"]["default"]
        self.top_right_x    = parameters["crop_image"]["top_right_x"]["default"]
        self.top_right_y    = parameters["crop_image"]["top_right_y"]["default"]
        self.bottom_left_x  = parameters["crop_image"]["bottom_left_x"]["default"]
        self.bottom_left_y  = parameters["crop_image"]["bottom_left_y"]["default"]
        self.bottom_right_x = parameters["crop_image"]["bottom_right_x"]["default"]
        self.bottom_right_y = parameters["crop_image"]["bottom_right_y"]["default"]

        # Helligkeitsschwellwert: ab welchem Grauwert gilt Pixel als nicht-schwarz
        self.brightness_threshold = parameters["detection"]["brightness_threshold"]["default"]
        # Mindestanteil heller Pixel im ROI für Helligkeits-Erkennung
        self.brightness_ratio     = parameters["detection"]["brightness_ratio"]["default"]
        # Mindestplatz (Pixel) auf einer Seite für normales Ausweichen
        # → weniger Platz → Gegenspurübernahme
        self.min_side_space       = parameters["detection"]["min_side_space"]["default"]

        # Vertikale ROI im Bird's-Eye-View: nur vordere Hälfte prüfen
        # 0.5 = ab Bildmitte, 1.0 = bis ganz unten (direkt vor Bot)
        self.roi_start = parameters["detection"]["roi_start"]["default"]
        self.roi_end   = parameters["detection"]["roi_end"]["default"]

        # Hough-Kreise Parameter (auf Originalbild)
        self.hough_dp         = parameters["hough"]["dp"]["default"]
        self.hough_min_dist   = parameters["hough"]["min_dist"]["default"]
        self.hough_param1     = parameters["hough"]["param1"]["default"]
        self.hough_param2     = parameters["hough"]["param2"]["default"]
        self.hough_min_radius = parameters["hough"]["min_radius"]["default"]
        self.hough_max_radius = parameters["hough"]["max_radius"]["default"]
        # Unterste X% des Originalbildes für Hough-Kreise prüfen
        self.hough_roi_start  = parameters["hough"]["roi_start"]["default"]


    def cbLane(self, msg):
        # Linienpositionen aus Spurversatz schätzen
        # Formel: error = 1 - (lane_center / (crop_size/2))
        # → lane_center = (1 - error) * (crop_size/2)
        error       = msg.data
        lane_center = (1 - error) * (self._crop_im_size / 2)
        half_lane   = self._crop_im_size * 0.30  # ~60% Spurbreite als Faustformel
        self.center_white  = int(min(lane_center + half_lane, self._crop_im_size * 0.95))
        self.center_yellow = int(max(lane_center - half_lane, self._crop_im_size * 0.05))


    def _crop_img(self, img):
        # Bird's-Eye-View Transformation (identisch zu detect_lane_node)
        pts1 = np.float32([
            [self.top_left_x,     self.top_left_y],
            [self.top_right_x,    self.top_right_y],
            [self.bottom_right_x, self.bottom_right_y],
            [self.bottom_left_x,  self.bottom_left_y],
        ])
        pts2 = np.float32([
            [0,                  0],
            [self._crop_im_size, 0],
            [0,                  self._crop_im_size],
            [self._crop_im_size, self._crop_im_size],
        ])
        M = cv2.getPerspectiveTransform(pts1, pts2)
        return cv2.warpPerspective(img, M, (self._crop_im_size, self._crop_im_size))


    def cbDetectDuck(self, image_msg):
        if self.is_running:
            return
        self.is_running = True

        # Bild dekodieren
        np_arr   = np.frombuffer(image_msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Bird's-Eye-View
        img_bird = self._crop_img(cv_image)

        # ── ROI zwischen gelber und weißer Linie ─────────────────────────────
        row_start = int(self._crop_im_size * self.roi_start)
        row_end   = int(self._crop_im_size * self.roi_end)
        col_start = self.center_yellow
        col_end   = self.center_white

        # Sicherheitsprüfung: ROI muss gültige Breite haben
        if col_end <= col_start + 10:
            self.pub_duck.publish(Float64(data=-99.0))
            self.is_running = False
            return

        roi_bird = img_bird[row_start:row_end, col_start:col_end]

        # ── Ansatz 1: Helligkeitsprüfung im Bird's-Eye-View ──────────────────
        gray_roi    = cv2.cvtColor(roi_bird, cv2.COLOR_BGR2GRAY)
        # Gauß-Filter: Spiegelungsartefakte glätten
        blurred     = cv2.GaussianBlur(gray_roi, (5, 5), 0)
        # Schwellwert → Binärmaske heller Pixel
        bright_mask = cv2.threshold(blurred, self.brightness_threshold, 255, cv2.THRESH_BINARY)[1]
        # MORPH_OPEN: kleine Spiegelungs-Pixel entfernen, große Blobs (Ente) behalten
        kernel      = np.ones((5, 5), np.uint8)
        bright_mask = cv2.morphologyEx(bright_mask, cv2.MORPH_OPEN, kernel)

        total_pixels        = bright_mask.size
        bright_pixels       = cv2.countNonZero(bright_mask)
        bright_ratio        = bright_pixels / total_pixels if total_pixels > 0 else 0
        brightness_detected = bright_ratio > self.brightness_ratio

        # ── Ansatz 2: Hough-Kreise auf Originalbild ───────────────────────────
        gray_orig = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        blur_orig = cv2.GaussianBlur(gray_orig, (9, 9), 2)
        circles   = cv2.HoughCircles(
            blur_orig,
            cv2.HOUGH_GRADIENT,
            dp       = self.hough_dp,
            minDist  = self.hough_min_dist,
            param1   = self.hough_param1,
            param2   = self.hough_param2,
            minRadius= self.hough_min_radius,
            maxRadius= self.hough_max_radius
        )

        hough_detected = False
        duck_x_orig    = None
        duck_circles   = []

        if circles is not None:
            circles_arr = np.round(circles[0]).astype(int)
            img_h       = cv_image.shape[0]
            for (x, y, r) in circles_arr:
                # Nur Kreise im unteren Bereich (Fahrbahn sichtbar)
                if y > img_h * self.hough_roi_start:
                    hough_detected = True
                    duck_circles.append((x, y, r))
                    if duck_x_orig is None:
                        duck_x_orig = x  # erste/nächste Ente nehmen

        # ── Kombinierte Entscheidung ───────────────────────────────────────────
        duck_detected = brightness_detected and hough_detected
        print(f"Duck: brightness={brightness_detected}({bright_ratio:.2f}), hough={hough_detected} → {duck_detected}")

        # ── Debug-Bild: Bounding-Box und Infos einzeichnen ────────────────────
        debug_img = cv_image.copy()

        # ROI-Kasten einzeichnen (grün = frei, rot = Ente erkannt)
        roi_color = (0, 0, 255) if duck_detected else (0, 255, 0)
        cv2.putText(debug_img, "ROI (Bird's-Eye)",
                    (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.6, roi_color, 2)

        # Hough-Kreise: Kreis + Bounding-Box + Label einzeichnen
        for (x, y, r) in duck_circles:
            # Kreis
            cv2.circle(debug_img, (x, y), r, (0, 255, 255), 2)
            # Bounding-Box um den Kreis
            box_color = (0, 0, 255) if duck_detected else (255, 255, 0)
            cv2.rectangle(debug_img, (x - r, y - r), (x + r, y + r), box_color, 2)
            # Label
            label = "ENTE!" if duck_detected else "Kreis?"
            cv2.putText(debug_img, label,
                        (x - r, y - r - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)

        # Statuszeilen
        cv2.putText(debug_img, f"Brightness: {bright_ratio:.2f} ({'OK' if brightness_detected else '--'})",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(debug_img, f"Hough: {'OK' if hough_detected else '--'}",
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # ── Position publizieren ──────────────────────────────────────────────
        if duck_detected and duck_x_orig is not None:
            # Normieren: -1 = ganz links, +1 = ganz rechts
            duck_x_norm = (duck_x_orig / cv_image.shape[1]) * 2 - 1
            self.pub_duck.publish(Float64(data=duck_x_norm))
        else:
            self.pub_duck.publish(Float64(data=-99.0))

        # Debug-Bild publizieren
        if self.pub_debug.get_num_connections() > 0:
            debug_msg              = CompressedImage()
            debug_msg.header.stamp = rospy.Time.now()
            debug_msg.format       = "jpeg"
            debug_msg.data         = np.array(cv2.imencode('.jpg', debug_img)[1]).tobytes()
            self.pub_debug.publish(debug_msg)

        self.is_running = False


    def run(self):
        rospy.spin()


if __name__ == '__main__':
    node = DetectDuckNode('detect_duck_node')
    node.run()
