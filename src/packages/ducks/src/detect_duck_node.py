#!/usr/bin/env python3

# ─────────────────────────────────────────────────────────────────────────────
# detect_duck_node.py
#
# Aufgabe (Challenge 3 – Watch out for Ducks):
#   Erkennt starre Enten auf dem Wendeplatz und liefert
#     1. ob eine Ente im Weg ist          → /detect/duck            (Float64)
#     2. wie viel Platz links/rechts ist   → /detect/duck_space      (Float32MultiArray)
#
# Erkennung (zwei Verfahren müssen GLEICHZEITIG anschlagen → wenig Fehlalarme):
#   A) Originalbild:  Hough-Kreise  +  Gelbfilter (Entenfarbe)
#   B) Bird's-Eye-View ROI: Helligkeitsprüfung (freie Fahrbahn ist dunkel;
#      eine Ente hebt die mittlere Helligkeit im ROI deutlich an)
#
#   → Nur wenn (A) UND (B) anschlagen, gilt eine Ente als erkannt.
#
# Geometrie:
#   - Hough/Gelb laufen auf dem ORIGINALBILD: dort sind Enten unverzerrt rund,
#     Kreiserkennung funktioniert dort am besten.
#   - Die Platzmessung links/rechts läuft im BIRD'S-EYE-VIEW: nur dort ist
#     "Platz" metrisch konsistent zur Fahrspur (gleicher Warp wie detect_lane_node).
#
# Veröffentlichte Werte:
#   /detect/duck        : x-Position der nächsten Ente, normiert [-1, +1]
#                         -99.0 = keine Ente erkannt
#   /detect/duck_space  : [free_left, free_right] als Anteil der BEV-Breite [0..1]
#
# Die Ausweich-LOGIK (links/rechts/Gegenspur, Offset-Rampe) liegt bewusst NICHT
# hier, sondern in control_obstacle_node. Diese Node liefert nur Wahrnehmung.
# ─────────────────────────────────────────────────────────────────────────────

import os
import rospy
import numpy as np
import cv2
from std_msgs.msg import Float64, Float32MultiArray
from sensor_msgs.msg import CompressedImage
import util


class DetectDuckNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        self._vehicle_name = os.environ['VEHICLE_NAME']

        # ── Kritisch: alle self.*-Variablen VOR util.init_parameters setzen ──
        # init_parameters ruft cbUpdateParameters sofort auf.
        self._crop_im_size = 400          # identisch zu detect_lane_node (BEV-Quadrat)
        self.is_running    = False
        self.counter       = 0            # erste Frames verwerfen (Kamera instabil)

        # Platzhalter-Defaults, werden von cbUpdateParameters überschrieben
        self.hue_yellow_l = 15;  self.hue_yellow_h = 60
        self.sat_yellow_l = 60;  self.sat_yellow_h = 255
        self.val_yellow_l = 120; self.val_yellow_h = 255

        self.hough_dp          = 1.2
        self.hough_min_dist    = 40
        self.hough_param1      = 100
        self.hough_param2      = 30
        self.hough_min_radius  = 8
        self.hough_max_radius  = 120
        self.min_yellow_in_circle = 0.20   # Anteil gelber Pixel im Kreis
        self.hough_roi_top     = 0.50      # Hough nur auf unterer Bildhälfte

        # Enten-Maske (satter/oranger als Mittellinie) – für Platzmessung
        self.hue_duck_l = 10;  self.hue_duck_h = 35
        self.sat_duck_l = 120; self.sat_duck_h = 255
        self.val_duck_l = 120; self.val_duck_h = 255
        self.duck_min_area = 300           # Mindest-Blobfläche im BEV-ROI

        # BEV-Helligkeits-ROI (untere Bildmitte – direkt vor dem Bot)
        self.bev_roi_top    = 0.45
        self.bev_roi_bottom = 0.95
        self.bev_roi_left   = 0.20
        self.bev_roi_right  = 0.80
        self.brightness_threshold = 60     # mittlere Graustufe; frei = dunkel

        # BEV-Warp-Eckpunkte (werden aus detect_lane_node-Config übernommen)
        self.top_left_x = 159;  self.top_left_y = 218
        self.top_right_x = 441; self.top_right_y = 218
        self.bottom_left_x = 606;  self.bottom_left_y = 382
        self.bottom_right_x = -29; self.bottom_right_y = 382

        # Parameter laden + Live-Update-Callback registrieren
        util.init_parameters(node_name, self.cbUpdateParameters)

        # ── Subscriber: Kamerabild ───────────────────────────────────────────
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self.sub_image = rospy.Subscriber(
            self._camera_topic, CompressedImage, self.cbDetect, queue_size=1)

        # ── Publisher ─────────────────────────────────────────────────────────
        # Entenposition x∈[-1,1]; -99 = keine Ente
        self.pub_duck = rospy.Publisher(
            f'/{self._vehicle_name}/detect/duck', Float64, queue_size=1)
        # Freier Platz [links, rechts] als Anteil [0..1] der BEV-Breite
        self.pub_space = rospy.Publisher(
            f'/{self._vehicle_name}/detect/duck_space', Float32MultiArray, queue_size=1)

        # Debug-Bild (Originalbild mit eingezeichneten Kreisen)
        self.pub_debug_duck = rospy.Publisher(
            f'/{self._vehicle_name}/debug/duck', CompressedImage, queue_size=1)

        # Debug-Bild (BEV mit ROI-Kasten und Platzmessung)
        self.pub_debug_duck_bev = rospy.Publisher(
            f'/{self._vehicle_name}/debug/duck_bev', CompressedImage, queue_size=1)

        rospy.loginfo(f"[{node_name}] Bereit. Warte auf Kamerabild ...")


    # ── Parameter ──────────────────────────────────────────────────────────────

    def cbUpdateParameters(self, parameters):
        y = parameters["yellow"]
        self.hue_yellow_l = y["hl"]["default"]; self.hue_yellow_h = y["hh"]["default"]
        self.sat_yellow_l = y["sl"]["default"]; self.sat_yellow_h = y["sh"]["default"]
        self.val_yellow_l = y["vl"]["default"]; self.val_yellow_h = y["vh"]["default"]

        h = parameters["hough"]
        self.hough_dp         = h["dp"]["default"]
        self.hough_min_dist   = h["min_dist"]["default"]
        self.hough_param1     = h["param1"]["default"]
        self.hough_param2     = h["param2"]["default"]
        self.hough_min_radius = h["min_radius"]["default"]
        self.hough_max_radius = h["max_radius"]["default"]
        self.min_yellow_in_circle = h["min_yellow_ratio"]["default"]
        self.hough_roi_top    = h["roi_top"]["default"]

        d = parameters["duck"]
        self.hue_duck_l = d["hl"]["default"]; self.hue_duck_h = d["hh"]["default"]
        self.sat_duck_l = d["sl"]["default"]; self.sat_duck_h = d["sh"]["default"]
        self.val_duck_l = d["vl"]["default"]; self.val_duck_h = d["vh"]["default"]
        self.duck_min_area = d["min_area"]["default"]

        b = parameters["bev"]
        self.bev_roi_top    = b["roi_top"]["default"]
        self.bev_roi_bottom = b["roi_bottom"]["default"]
        self.bev_roi_left   = b["roi_left"]["default"]
        self.bev_roi_right  = b["roi_right"]["default"]
        self.brightness_threshold = b["brightness_threshold"]["default"]

        c = parameters["crop_image"]
        self.top_left_x     = c["top_left_x"]["default"];     self.top_left_y     = c["top_left_y"]["default"]
        self.top_right_x    = c["top_right_x"]["default"];    self.top_right_y    = c["top_right_y"]["default"]
        self.bottom_left_x  = c["bottom_left_x"]["default"];  self.bottom_left_y  = c["bottom_left_y"]["default"]
        self.bottom_right_x = c["bottom_right_x"]["default"]; self.bottom_right_y = c["bottom_right_y"]["default"]


    # ── Bildverarbeitung ─────────────────────────────────────────────────────

    def crop_img(self, img):
        # Identische BEV-Transformation wie detect_lane_node (gleiche Eckpunkte!).
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

    def _yellow_mask(self, hsv):
        return cv2.inRange(hsv,
            (self.hue_yellow_l, self.sat_yellow_l, self.val_yellow_l),
            (self.hue_yellow_h, self.sat_yellow_h, self.val_yellow_h))

    def detect_circles_original(self, bgr):
        # Verfahren A: Hough-Kreise auf dem Originalbild, gefiltert per Gelbanteil.
        # Rückgabe: Liste (x, y, r) der als "Ente" plausiblen Kreise (Vollbild-Koords).
        #
        # Hough läuft nur auf dem vertikalen ROI (untere Bildhälfte): dort liegen
        # Enten im Fahrweg. Das spart Rechenzeit und unterdrückt Fehlkreise am
        # Horizont (Schilder, andere Bots, Hintergrund).
        h_full = bgr.shape[0]
        roi_y0 = int(h_full * self.hough_roi_top)
        roi    = bgr[roi_y0:, :]

        hsv         = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask_yellow = self._yellow_mask(hsv)
        mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_CLOSE,
                                       np.ones((5, 5), np.uint8))

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.medianBlur(gray, 5)

        circles = cv2.HoughCircles(
            gray, cv2.HOUGH_GRADIENT,
            dp=self.hough_dp, minDist=self.hough_min_dist,
            param1=self.hough_param1, param2=self.hough_param2,
            minRadius=int(self.hough_min_radius), maxRadius=int(self.hough_max_radius))

        ducks = []
        if circles is not None:
            for (x, y, r) in np.uint16(np.around(circles[0])):
                # Kreismaske im ROI-Koordinatensystem bauen und Gelbanteil messen
                circle_mask = np.zeros(mask_yellow.shape, dtype=np.uint8)
                cv2.circle(circle_mask, (int(x), int(y)), int(r), 255, -1)
                area = cv2.countNonZero(circle_mask)
                if area == 0:
                    continue
                yellow_in = cv2.countNonZero(cv2.bitwise_and(mask_yellow, circle_mask))
                if yellow_in / area >= self.min_yellow_in_circle:
                    # y zurück ins Vollbild-Koordinatensystem verschieben
                    ducks.append((int(x), int(y) + roi_y0, int(r)))
        return ducks, mask_yellow

    def check_brightness_bev(self, bev_bgr):
        # Verfahren B: mittlere Helligkeit im BEV-ROI.
        # Freie Fahrbahn ist dunkel → mittlere Helligkeit niedrig.
        # Eine Ente im ROI hebt die mittlere Helligkeit über den Schwellwert.
        gray = cv2.cvtColor(bev_bgr, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape[:2]
        y0 = int(h * self.bev_roi_top);  y1 = int(h * self.bev_roi_bottom)
        x0 = int(w * self.bev_roi_left); x1 = int(w * self.bev_roi_right)
        roi = gray[y0:y1, x0:x1]
        if roi.size == 0:
            return False, 0.0
        mean_val = float(np.mean(roi))
        return mean_val > self.brightness_threshold, mean_val

    def _duck_mask(self, hsv):
        # Separate Maske für die Entenfarbe (satter/oranger als die Mittellinie).
        # Wird für die Platzmessung genutzt, damit die dünne gelbe Mittellinie
        # nicht fälschlich als Ente mitgemessen wird.
        return cv2.inRange(hsv,
            (self.hue_duck_l, self.sat_duck_l, self.val_duck_l),
            (self.hue_duck_h, self.sat_duck_h, self.val_duck_h))

    def measure_free_space_bev(self, bev_bgr):
        # Misst freien Platz links/rechts im BEV anhand der ENTEN-Maske.
        # Rückgabe: (free_left, free_right) als Anteil der BEV-Breite [0..1].
        #
        # Statt der rohen Gelb-Maske wird eine sattere Enten-Maske verwendet und
        # auf zusammenhängende Flächen mit Mindestgröße gefiltert. Die dünne gelbe
        # Mittellinie (schmal, langgezogen) fällt durch den Flächenfilter heraus.
        hsv  = cv2.cvtColor(bev_bgr, cv2.COLOR_BGR2HSV)
        mask = self._duck_mask(hsv)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        h, w = mask.shape[:2]

        # Nur unteren Bereich vor dem Bot betrachten
        y0 = int(h * self.bev_roi_top)
        roi = mask[y0:, :]

        # Zusammenhangskomponenten – nur ausreichend große Blobs gelten als Ente
        num, labels, stats, _ = cv2.connectedComponentsWithStats(roi, connectivity=8)
        duck_cols = []
        for i in range(1, num):  # 0 = Hintergrund
            if stats[i, cv2.CC_STAT_AREA] >= self.duck_min_area:
                x      = stats[i, cv2.CC_STAT_LEFT]
                width  = stats[i, cv2.CC_STAT_WIDTH]
                duck_cols.append((x, x + width))

        if not duck_cols:
            # Keine ausreichend große Ente im BEV → beidseitig "frei"
            return 1.0, 1.0, None

        duck_left  = min(c[0] for c in duck_cols)
        duck_right = max(c[1] for c in duck_cols)
        free_left  = duck_left / w            # Platz links der Ente
        free_right = (w - duck_right) / w     # Platz rechts der Ente
        # Pixelgrenzen (im Vollbild-BEV) für Debug-Visualisierung mitgeben
        box = (int(duck_left), int(y0), int(duck_right), int(h))
        return float(free_left), float(free_right), box


    # ── Hauptcallback ──────────────────────────────────────────────────────────

    def cbDetect(self, image_msg):
        if self.counter <= 3:
            self.counter += 1
            return
        if self.is_running:
            return
        self.is_running = True

        try:
            np_arr   = np.frombuffer(image_msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_image is None:
                # Korrupter Frame: NICHTS publishen, alten Zustand stehen lassen.
                # Ein -99 hier könnte mitten im Ausweichen fälschlich "Ente weg"
                # signalisieren; ein einzelner ausgelassener Tick bei 10 Hz ist
                # dagegen folgenlos, da control_obstacle_node über Zeitstempel arbeitet.
                rospy.logwarn_throttle(5.0, "Frame konnte nicht dekodiert werden – übersprungen.")
                return

            h_orig, w_orig = cv_image.shape[:2]

            # Verfahren A: Hough + Gelb auf Originalbild
            ducks, mask_yellow = self.detect_circles_original(cv_image)

            # Verfahren B: Helligkeit im BEV-ROI
            bev = self.crop_img(cv_image)
            bright_hit, mean_val = self.check_brightness_bev(bev)

            # Beide müssen anschlagen
            duck_present = (len(ducks) > 0) and bright_hit

            best_duck   = None    # (x,y,r) der bestätigten Ente – für Debug-Box
            space_box   = None    # BEV-Pixelbox der Ente – für Debug
            free_left = free_right = 1.0

            if duck_present:
                # Nächste Ente = größter Kreis (am nächsten am Bot)
                best_duck = max(ducks, key=lambda c: c[2])
                x_px, y_px, r = best_duck
                duck_x = (x_px / w_orig) * 2.0 - 1.0          # → [-1, +1]
                self.pub_duck.publish(Float64(data=duck_x))

                free_left, free_right, space_box = self.measure_free_space_bev(bev)
                self.pub_space.publish(
                    Float32MultiArray(data=[free_left, free_right]))

                rospy.loginfo_throttle(1.0,
                    f"Ente erkannt x={duck_x:.2f} "
                    f"(Helligkeit {mean_val:.0f}>{self.brightness_threshold}) "
                    f"frei L={free_left:.2f} R={free_right:.2f}")
            else:
                # Keine Ente → Sentinel -99
                self.pub_duck.publish(Float64(data=-99.0))

            # ── Debug-Bild 1: Originalbild ───────────────────────────────────
            if self.pub_debug_duck.get_num_connections() > 0:
                dbg = cv_image.copy()
                # Hough-ROI-Grenze einzeichnen (alles darunter wird durchsucht)
                roi_y = int(h_orig * self.hough_roi_top)
                cv2.line(dbg, (0, roi_y), (w_orig, roi_y), (255, 255, 0), 1)
                cv2.putText(dbg, "Hough-ROI", (4, roi_y - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1, cv2.LINE_AA)

                # Alle Kandidaten-Kreise (dünn, orange)
                for (x, y, r) in ducks:
                    cv2.circle(dbg, (x, y), r, (0, 165, 255), 1)

                # Bestätigte Ente: dicke Bounding-Box + Kreis + Mittelpunkt
                if best_duck is not None:
                    x, y, r = best_duck
                    cv2.rectangle(dbg, (x - r, y - r), (x + r, y + r), (0, 0, 255), 2)
                    cv2.circle(dbg, (x, y), r, (0, 0, 255), 2)
                    cv2.circle(dbg, (x, y), 3, (0, 0, 255), -1)
                    box_label_y = max(y - r - 6, 14)
                    cv2.putText(dbg, "ENTE", (x - r, box_label_y),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)

                tag = "ENTE erkannt" if duck_present else "frei"
                col = (0, 0, 255) if duck_present else (0, 255, 0)
                cv2.putText(dbg, f"{tag} | BEV-Hell {mean_val:.0f}/{int(self.brightness_threshold)}",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2, cv2.LINE_AA)
                self._publish_compressed(self.pub_debug_duck, dbg)

            # ── Debug-Bild 2: Bird's-Eye-View ────────────────────────────────
            if self.pub_debug_duck_bev.get_num_connections() > 0:
                dbv   = bev.copy()
                hb, wb = dbv.shape[:2]
                # Helligkeits-ROI-Kasten (gelb)
                ry0 = int(hb * self.bev_roi_top);  ry1 = int(hb * self.bev_roi_bottom)
                rx0 = int(wb * self.bev_roi_left); rx1 = int(wb * self.bev_roi_right)
                cv2.rectangle(dbv, (rx0, ry0), (rx1, ry1), (0, 255, 255), 1)
                cv2.putText(dbv, "Hell-ROI", (rx0 + 2, ry0 + 14),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1, cv2.LINE_AA)

                # Enten-Box im BEV (rot) + freie Bereiche links/rechts (grün)
                if space_box is not None:
                    bx0, by0, bx1, by1 = space_box
                    cv2.rectangle(dbv, (bx0, by0), (bx1, by1), (0, 0, 255), 2)
                    # freier Platz links
                    cv2.rectangle(dbv, (0, by0), (bx0, by1), (0, 255, 0), 1)
                    # freier Platz rechts
                    cv2.rectangle(dbv, (bx1, by0), (wb - 1, by1), (0, 255, 0), 1)

                cv2.putText(dbv, f"L={free_left:.2f}  R={free_right:.2f}",
                            (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (255, 255, 255), 2, cv2.LINE_AA)
                self._publish_compressed(self.pub_debug_duck_bev, dbv)

        finally:
            self.is_running = False

    def _publish_compressed(self, publisher, img):
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', img)[1]).tobytes()
        publisher.publish(msg)


if __name__ == '__main__':
    node = DetectDuckNode('detect_duck_node')
    rospy.spin()
