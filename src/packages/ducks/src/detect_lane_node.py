#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────────────────────
# detect_lane_node.py  (Challenge 3 – Watch out for Ducks)
#
# Spurerkennung im Bird's-Eye-View + rote Haltelinie.
# (Unverändert aus der Lane-Following-Basis übernommen)
#
# Aufgaben:
#   • Spurversatz [-1, +1] aus weißer + gelber Linie  → /detect/lane (Float64)
#   • rote Haltelinie erkennen                        → /detect/stop_line (Bool)
#   • Debug-Bilder für camera_dashboard_node          → /debug/...
#
# Performance-Leitlinie (schlank): kein CLAHE / keine Morphologie im Hauptpfad
# (beide als optionale, klar markierte Blöcke zum Einkommentieren erhalten).
# GUI (imshow) läuft NUR im Main-Thread (run_debug), NIE im ROS-Callback –
# das verhindert das sporadische Einfrieren des Debug-Fensters.
# ─────────────────────────────────────────────────────────────────────────────

import os
import rospy
import numpy as np
import cv2
from std_msgs.msg import Float64, Bool, Float32MultiArray
from sensor_msgs.msg import CompressedImage
import util


class DetectLaneNode:
    def __init__(self, node_name):
        # ROS-Node initialisieren
        rospy.init_node(node_name)

        # Fahrzeugnamen aus Umgebungsvariable lesen
        self._vehicle_name = os.environ['VEHICLE_NAME']

        # ── Kritisch: Alle self.*-Variablen VOR util.init_parameters setzen ──
        # util.init_parameters ruft cbUpdateParameters sofort auf. Falls dort ein
        # Fehler passiert, bricht __init__ ab und später definierte Variablen
        # fehlen → AttributeError in den Callbacks.
        self._crop_im_size        = 400
        self.is_running           = False
        self.counter              = 0     # Frame-Zähler: erste 3 Frames verwerfen
        self.last_white_position  = None  # Frame-Tracking für weiße Linie
        self.last_yellow_position = None  # Frame-Tracking für gelbe Linie

        # Lokales Debug-Fenster (imshow). Nur im Main-Thread (run_debug) genutzt.
        # True  = beim Kalibrieren am Bildschirm (zusätzlich imshow-Zeilen einkommentieren)
        self.show_window = True
        # False = immer im Fahrbetrieb/Challenge (Debug-Bilder laufen via /debug-Topics weiter)
        # self.show_window = False
        
        # Platzhalter für Debug-Variablen
        # → verhindert AttributeError, falls run_debug vor dem ersten Frame läuft
        blank       = np.zeros((self._crop_im_size, self._crop_im_size), dtype=np.uint8)
        blank_color = np.zeros((self._crop_im_size, self._crop_im_size, 3), dtype=np.uint8)
        self.img                = blank_color
        self.lane_center        = self._crop_im_size / 2
        self.white_alternative  = int(self._crop_im_size * 0.95)
        self.yellow_alternative = int(self._crop_im_size * 0.05)
        self.center_white       = int(self._crop_im_size * 0.95)
        self.center_yellow      = int(self._crop_im_size * 0.05)
        self.debug_img_white    = blank
        self.debug_img_yellow   = blank
        self.debug_img_red      = blank

        # ── Enten-Erkennung (Challenge 3) – Defaults vor init_parameters ──────
        self.OCC_BINS              = 40
        self.duck_enabled          = True
        self.duck_roi_top          = 0.35
        self.duck_roi_bottom       = 1.00
        self.duck_brightness_thr   = 90
        self.duck_use_otsu         = 1
        self.duck_min_area         = 250
        self.duck_min_w            = 12
        self.duck_min_h            = 12
        self.duck_line_max_aspect  = 4.0
        self.debug_img_duck        = blank_color

        # Parameter aus JSON laden + Live-Update-Callback registrieren
        # NACH den self.*-Variablen – cbUpdateParameters kann jetzt sicher laufen
        util.init_parameters(node_name, self.cbUpdateParameters)

        # ── Subscriber ────────────────────────────────────────────────────────
        # buff_size groß: verhindert, dass sich Kamerabilder auf TCP-Ebene stauen
        # (Kamera-Lag), wenn ein Callback mal länger braucht als ein Frame-Intervall.
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self.sub_image_original = rospy.Subscriber(
            self._camera_topic, CompressedImage, self.cbFindLane,
            queue_size=1, buff_size=2**24)

        # ── Publisher ─────────────────────────────────────────────────────────
        # Spurversatz [-1, +1] an control_lane_node
        self.pub_lane = rospy.Publisher(
            f'/{self._vehicle_name}/detect/lane', Float64, queue_size=1)

        # Rote Haltelinie erkannt (True/False) an switch_control_node
        self.pub_stop_line = rospy.Publisher(
            f'/{self._vehicle_name}/detect/stop_line', Bool, queue_size=1)

        # ── Enten (Challenge 3): Belegungsprofil + Trigger ────────────────────
        self.pub_duck_occupancy = rospy.Publisher(
            f'/{self._vehicle_name}/detect/duck_occupancy', Float32MultiArray, queue_size=1)
        self.pub_duck = rospy.Publisher(
            f'/{self._vehicle_name}/detect/duck', Float64, queue_size=1)

        # ── Debug-Publisher ───────────────────────────────────────────────────
        self.pub_debug_original  = rospy.Publisher(
            f'/{self._vehicle_name}/debug/original',    CompressedImage, queue_size=1)
        self.pub_debug_bird      = rospy.Publisher(
            f'/{self._vehicle_name}/debug/bird_view',   CompressedImage, queue_size=1)
        self.pub_debug_annotated = rospy.Publisher(
            f'/{self._vehicle_name}/debug/annotated',   CompressedImage, queue_size=1)
        self.pub_debug_lane      = rospy.Publisher(
            f'/{self._vehicle_name}/debug/lane_croped', CompressedImage, queue_size=1)
        self.pub_debug_white     = rospy.Publisher(
            f'/{self._vehicle_name}/debug/lane_white',  CompressedImage, queue_size=1)
        self.pub_debug_yellow    = rospy.Publisher(
            f'/{self._vehicle_name}/debug/lane_yellow', CompressedImage, queue_size=1)
        self.pub_debug_red       = rospy.Publisher(
            f'/{self._vehicle_name}/debug/lane_red',    CompressedImage, queue_size=1)
        # Enten-BEV mit Boxen + Belegungsbalken (Challenge 3)
        self.pub_debug_duck      = rospy.Publisher(
            f'/{self._vehicle_name}/debug/duck_bev',    CompressedImage, queue_size=1)

    # ── Parameter ───────────────────────────────────────────────────────────────

    def cbUpdateParameters(self, parameters):
        # Wird beim Start UND bei jeder Schieberegler-Änderung aufgerufen.

        # Weiße Linie (rechte Fahrbahnmarkierung)
        self.hue_white_l        = parameters["white"]["hl"]["default"]
        self.hue_white_h        = parameters["white"]["hh"]["default"]
        self.saturation_white_l = parameters["white"]["sl"]["default"]
        self.saturation_white_h = parameters["white"]["sh"]["default"]
        self.lightness_white_l  = parameters["white"]["vl"]["default"]
        self.lightness_white_h  = parameters["white"]["vh"]["default"]

        # Gelbe Linie (linke, gestrichelte Mittellinie)
        self.hue_yellow_l        = parameters["yellow"]["hl"]["default"]
        self.hue_yellow_h        = parameters["yellow"]["hh"]["default"]
        self.saturation_yellow_l = parameters["yellow"]["sl"]["default"]
        self.saturation_yellow_h = parameters["yellow"]["sh"]["default"]
        self.lightness_yellow_l  = parameters["yellow"]["vl"]["default"]
        self.lightness_yellow_h  = parameters["yellow"]["vh"]["default"]

        # Perspektivtransformation (Bird's-Eye-View Eckpunkte)
        self.top_left_x     = parameters["crop_image"]["top_left_x"]["default"]
        self.top_left_y     = parameters["crop_image"]["top_left_y"]["default"]
        self.top_right_x    = parameters["crop_image"]["top_right_x"]["default"]
        self.top_right_y    = parameters["crop_image"]["top_right_y"]["default"]
        self.bottom_left_x  = parameters["crop_image"]["bottom_left_x"]["default"]
        self.bottom_left_y  = parameters["crop_image"]["bottom_left_y"]["default"]
        self.bottom_right_x = parameters["crop_image"]["bottom_right_x"]["default"]
        self.bottom_right_y = parameters["crop_image"]["bottom_right_y"]["default"]

        # Rote Haltelinie – zwei HSV-Bereiche (Rot liegt an zwei Stellen des Hue-Kreises)
        self.hue_red_l        = parameters["red"]["hl"]["default"]    # Hue 0-10
        self.hue_red_h        = parameters["red"]["hh"]["default"]
        self.hue_red_l2       = parameters["red"]["hl2"]["default"]   # Hue 160-179
        self.hue_red_h2       = parameters["red"]["hh2"]["default"]
        self.saturation_red_l = parameters["red"]["sl"]["default"]
        self.saturation_red_h = parameters["red"]["sh"]["default"]
        self.lightness_red_l  = parameters["red"]["vl"]["default"]
        self.lightness_red_h  = parameters["red"]["vh"]["default"]
        self.red_pixel_threshold   = parameters["red"]["pixel_threshold"]["default"]
        # Vertikale ROI: 0.95 = nur unterste 5% des Bildes prüfen
        self.red_detection_zone    = parameters["red"]["detection_zone"]["default"]
        # Horizontale ROI: nur diesen x-Bereich prüfen (Gegenspur ausschließen)
        self.red_detection_x_start = parameters["red"]["detection_x_start"]["default"]
        self.red_detection_x_end   = parameters["red"]["detection_x_end"]["default"]

        # Frame-Tracking: maximaler Pixelsprung zwischen Frames (gilt für beide Linien)
        self.max_frame_jump = parameters["white"]["max_frame_jump"]["default"]

        # ── Enten-Parameter (Challenge 3) – defensiv, damit fehlende Keys die
        #    Node nicht beim Start abstürzen lassen (Lane läuft sonst weiter) ──
        def gd(group, key, default):
            try:
                return parameters[group][key]["default"]
            except (KeyError, TypeError):
                rospy.logwarn(f"[detect_lane/duck] Parameter {group}.{key} fehlt – nutze {default}")
                return default

        self.duck_enabled         = int(gd("duck", "enabled", 1)) == 1
        self.duck_roi_top         = gd("duck", "roi_top", 0.35)
        self.duck_roi_bottom      = gd("duck", "roi_bottom", 1.0)
        self.duck_brightness_thr  = gd("duck", "brightness_threshold", 90)
        self.duck_use_otsu        = int(gd("duck", "use_otsu", 1))
        self.duck_min_area        = gd("duck", "min_area", 250)
        self.duck_min_w           = gd("duck", "min_w", 12)
        self.duck_min_h           = gd("duck", "min_h", 12)
        self.duck_line_max_aspect = gd("duck", "line_max_aspect", 4.0)

    # ── Bildvorverarbeitung ─────────────────────────────────────────────────────

    def crop_img(self, img):
        # Perspektivtransformation in die Vogelperspektive (Bird's-Eye-View).
        pts1 = np.float32([
            [self.top_left_x,     self.top_left_y],
            [self.top_right_x,    self.top_right_y],
            [self.bottom_right_x, self.bottom_right_y],
            [self.bottom_left_x,  self.bottom_left_y]])
        pts2 = np.float32([
            [0, 0], [self._crop_im_size, 0],
            [0, self._crop_im_size], [self._crop_im_size, self._crop_im_size]])
        M = cv2.getPerspectiveTransform(pts1, pts2)
        return cv2.warpPerspective(img, M, (self._crop_im_size, self._crop_im_size))

    # ── Linienerkennung ─────────────────────────────────────────────────────────

    def get_x_for_driving(self, mask, distance, left_line, last_known=None):
        # Linienposition per Sobel-Kantenerkennung bestimmen.
        #
        # last_known: letzter bekannter Pixelwert dieser Linie (oder None beim ersten Frame).
        #   → None:  Initialisierung – rechteste Kante für Gelb, linkeste für Weiß
        #   → Wert:  nächste Kante zum letzten bekannten Wert wählen
        #            → robuster in engen Kurven und am Wendeplatz (keine Fehlzuordnung,
        #              wenn zwei weiße Kanten gleichzeitig sichtbar sind)

        grad = cv2.Sobel(mask, cv2.CV_16S, 1, 0, ksize=3, scale=1, delta=0,
                         borderType=cv2.BORDER_DEFAULT)
        _, th1 = cv2.threshold(grad, 127, 255, cv2.THRESH_BINARY)

        a = []
        for row in range(distance - 50, distance + 50):
            candidates = np.where(th1[row] == 255)[0]
            if candidates.size == 0:
                continue
            if last_known is not None:
                # Nächste Kante zum letzten bekannten Wert → stabil in Kurven
                a.append(candidates[np.argmin(np.abs(candidates - last_known))])
            elif left_line:
                # Initialisierung Gelb: rechteste Kante = rechter Rand der gelben Linie
                a.append(candidates[-1])
            else:
                # Initialisierung Weiß: linkeste Kante = linker Rand der weißen Linie
                a.append(candidates[0])

        if len(a) > 10:
            return np.median(a)
        return None  # keine Detektion – Aufrufer entscheidet, wie weiter

    def _resolve_line_position(self, raw, last_known, fallback, max_jump, label):
        # Entscheidet, welche x-Position als finale Linienposition gilt.
        #
        # raw        : Ergebnis von get_x_for_driving (Pixel oder None)
        # last_known : self.last_yellow_position bzw. self.last_white_position
        # fallback   : Bildrand-Fallback (yellow_alternative / white_alternative)
        # max_jump   : maximal erlaubter Sprung zwischen Frames
        # label      : 'Yellow' / 'White' für Logging
        #
        # Rückgabe: (finale_position, neuer_last_known)

        # Fall A: keine Detektion
        if raw is None:
            if last_known is not None:
                # last_known ist die beste Schätzung – Anker beibehalten
                rospy.logwarn_throttle(2.0,
                    f"{label}: keine Kanten – halte letzte Position {last_known:.0f}")
                return last_known, last_known
            # Erster Frame ohne Detektion → Bildrand-Fallback, aber NICHT ankern
            # (sonst würde get_x_for_driving danach nach Kanten am Bildrand suchen)
            rospy.logwarn_throttle(2.0,
                f"{label}: keine Kanten und kein Anker – Bildrand-Fallback {fallback}")
            return fallback, None

        # Fall B: Detektion vorhanden, aber kein Anker → jetzt ankern
        if last_known is None:
            return raw, raw

        # Fall C: Detektion + Anker → Sprung prüfen
        # BUGFIX: Früher wurde der Sprung nur geloggt, aber trotzdem voll übernommen
        # (return raw, raw) → Ursache für "springt an der Kreuzung auf die falsche Linie".
        # Jetzt wird die Bewegung sanft auf max_jump begrenzt: die Position darf sich pro
        # Frame höchstens um max_jump Pixel Richtung neuem Wert bewegen.
        jump = raw - last_known
        if abs(jump) > max_jump:
            clamped = last_known + np.sign(jump) * max_jump
            rospy.logwarn_throttle(2.0,
                f"{label}: Sprung zu groß ({abs(jump):.0f}px) – begrenzt auf "
                f"{max_jump:.0f}px → {clamped:.0f}")
            return clamped, clamped
        return raw, raw

    # ── Rote Haltelinie ──────────────────────────────────────────────────────────

    def detect_stop_line(self, hsv):
        # Rote Haltelinie im Bird's-Eye-View erkennen.
        # Rot liegt an zwei Stellen des Hue-Kreises → zwei Bereiche vereinen.
        mask_red_lower = cv2.inRange(hsv,
            (self.hue_red_l,  self.saturation_red_l, self.lightness_red_l),
            (self.hue_red_h,  self.saturation_red_h, self.lightness_red_h))
        mask_red_upper = cv2.inRange(hsv,
            (self.hue_red_l2, self.saturation_red_l, self.lightness_red_l),
            (self.hue_red_h2, self.saturation_red_h, self.lightness_red_h))
        mask_red = cv2.bitwise_or(mask_red_lower, mask_red_upper)

        # Vertikale + horizontale ROI: nur eigene Spur, direkt vor dem Bot.
        # detection_zone     → schneidet oben ab (nur unterer Bildteil)
        # detection_x_start  → schneidet links ab
        # detection_x_end    → schneidet rechts ab
        detection_row_start = int(mask_red.shape[0] * self.red_detection_zone)
        detection_col_start = int(mask_red.shape[1] * self.red_detection_x_start)
        detection_col_end   = int(mask_red.shape[1] * self.red_detection_x_end)
        roi_own = mask_red[detection_row_start:, detection_col_start:detection_col_end]

        red_pixel_count    = cv2.countNonZero(roi_own)
        stop_line_detected = red_pixel_count > self.red_pixel_threshold
        rospy.logdebug(f"Rote Pixel: {red_pixel_count} | Schwelle: "
                       f"{self.red_pixel_threshold} | erkannt: {stop_line_detected}")
        return stop_line_detected, mask_red

    # ── Haupt-Callback ───────────────────────────────────────────────────────────

    def cbFindLane(self, image_msg):
        # Erste 3 Frames verwerfen (Kamera noch nicht stabil)
        if self.counter <= 3:
            self.counter += 1
            return

        # Sperrvariable: kein paralleles Verarbeiten
        if self.is_running:
            return
        self.is_running = True

        try:
            # ── Schritt 1: Bild dekodieren ────────────────────────────────────────
            np_arr   = np.frombuffer(image_msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_image is None:
                rospy.logwarn_throttle(5.0, "Frame nicht dekodierbar – übersprungen.")
                return
            self._cbFindLane_body(image_msg, cv_image)
        finally:
            # Flag IMMER zurücksetzen – auch bei Fehlern. Verhindert, dass ein
            # einzelnes schlechtes Frame die Node dauerhaft einfriert.
            self.is_running = False

    def _cbFindLane_body(self, image_msg, cv_image):

        # Originalbild für Dashboard publizieren
        if self.pub_debug_original.get_num_connections() > 0:
            self._publish_compressed(self.pub_debug_original, cv_image)

        # ── Schritt 2: Bird's-Eye-View ────────────────────────────────────────
        img = self.crop_img(cv_image)

        if self.pub_debug_bird.get_num_connections() > 0:
            self._publish_compressed(self.pub_debug_bird, img)

        # ── Schritt 3 (OPTIONAL): CLAHE – lokaler Helligkeitsausgleich ─────────
        # Standardmäßig AUS (schlanker Hauptpfad). Bei Problemen mit wechselndem
        # Licht den folgenden Block einkommentieren. BGR → LAB → CLAHE nur auf
        # L-Kanal → zurück zu BGR. LAB trennt Helligkeit von Farbe, daher bleibt
        # die HSV-Kalibrierung stabil. Kostet spürbar Rechenzeit pro Frame.
        #
        # lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        # l, a_ch, b_ch = cv2.split(lab)
        # clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        # l = clahe.apply(l)
        # img = cv2.cvtColor(cv2.merge((l, a_ch, b_ch)), cv2.COLOR_LAB2BGR)

        # ── Schritt 4: HSV-Masken ─────────────────────────────────────────────
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        mask_yellow = cv2.inRange(hsv,
            (self.hue_yellow_l, self.saturation_yellow_l, self.lightness_yellow_l),
            (self.hue_yellow_h, self.saturation_yellow_h, self.lightness_yellow_h))

        mask_white = cv2.inRange(hsv,
            (self.hue_white_l, self.saturation_white_l, self.lightness_white_l),
            (self.hue_white_h, self.saturation_white_h, self.lightness_white_h))

        # ── Schritt 4b (OPTIONAL): Morphologie – Lücken in Masken schließen ────
        # Standardmäßig AUS. Bei löchrigen Masken (Schatten) einkommentieren.
        #
        # kernel = np.ones((5, 5), np.uint8)
        # mask_white  = cv2.morphologyEx(mask_white,  cv2.MORPH_CLOSE, kernel)
        # mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_CLOSE, kernel)

        # ── Schritt 5: Linienpositionen ───────────────────────────────────────
        white_alternative  = int(len(img[0]) * 0.95)
        yellow_alternative = int(len(img[0]) * 0.05)
        distance           = int(len(img) * 0.75)

        # Gelbe Linie
        center_yellow_raw = self.get_x_for_driving(
            mask_yellow, distance, left_line=True,
            last_known=self.last_yellow_position)
        center_yellow, self.last_yellow_position = self._resolve_line_position(
            center_yellow_raw, self.last_yellow_position,
            yellow_alternative, self.max_frame_jump, label='Yellow')

        # Weiße Linie (Tracking verhindert Sprünge zur Gegenspur in engen Kurven)
        center_white_raw = self.get_x_for_driving(
            mask_white, distance, left_line=False,
            last_known=self.last_white_position)
        center_white, self.last_white_position = self._resolve_line_position(
            center_white_raw, self.last_white_position,
            white_alternative, self.max_frame_jump, label='White')

        # Plausibilitätsprüfung: weiß muss rechts von gelb liegen
        if center_white <= center_yellow:
            if center_white > int(len(img[0]) * 0.4):
                center_yellow = yellow_alternative
            else:
                center_white = white_alternative

        # ── Schritt 6: Spurversatz berechnen ──────────────────────────────────
        lane_center = (center_white + center_yellow) / 2
        msg_error = Float64()
        msg_error.data = 1 - (lane_center / len(img) * 2)
        self.pub_lane.publish(msg_error)
        # Gedrosseltes Logging statt blockierendem print() (max. 1x/Sekunde)
        rospy.loginfo_throttle(1.0, f"Lane error: {msg_error.data:.3f} range [-1,1]")

        # ── Schritt 7: Rote Haltelinie ────────────────────────────────────────
        stop_line_detected, mask_red = self.detect_stop_line(hsv)
        self.pub_stop_line.publish(Bool(data=stop_line_detected))

        # ── Schritt 7b: Enten-Erkennung (Challenge 3) ─────────────────────────
        # Nutzt dasselbe BEV-Bild (img) → kein zweites Dekodieren/Warpen.
        if self.duck_enabled:
            self._process_ducks(img)
        else:
            # Erkennung aus → kein Trigger (für schichtweises Testen)
            self.pub_duck.publish(Float64(data=-99.0))

        # ── Schritt 8: Debug-Variablen für run_debug sichern ──────────────────
        # WICHTIG: self.img ist das UNANNOTIERTE Bird's-Eye-Bild. Alle Markierungen
        # werden ausschließlich in run_debug auf eine Kopie gezeichnet – so bleibt
        # das gespeicherte Bild sauber und es wird nicht doppelt annotiert.
        self.img                = img
        self.lane_center        = lane_center
        self.white_alternative  = white_alternative
        self.yellow_alternative = yellow_alternative
        self.center_white       = center_white
        self.center_yellow      = center_yellow
        self.stop_line_detected = stop_line_detected
        self.debug_img_white    = mask_white
        self.debug_img_yellow   = mask_yellow
        self.debug_img_red      = mask_red

    # ── Hilfsfunktionen ──────────────────────────────────────────────────────────

    def _publish_compressed(self, publisher, img):
        # OpenCV-Bild als komprimierte ROS-Message senden
        msg              = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format       = "jpeg"
        msg.data         = np.array(cv2.imencode('.jpg', img)[1]).tobytes()
        publisher.publish(msg)

    # ── Enten-Erkennung (Challenge 3) ─────────────────────────────────────────
    # Arbeitet auf dem BEREITS vorhandenen Bird's-Eye-Bild (img) aus cbFindLane.
    # Dadurch wird das Kamerabild nur EINMAL dekodiert und gewarpt (Latenz).

    def _duck_object_mask(self, bev_bgr):
        # Farb-robuste Objektmaske: helle Strukturen auf dunklem Boden.
        gray = cv2.cvtColor(bev_bgr, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        if self.duck_use_otsu:
            _, mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        else:
            _, mask = cv2.threshold(gray, int(self.duck_brightness_thr), 255, cv2.THRESH_BINARY)
        h = mask.shape[0]
        y0 = max(0, int(h * self.duck_roi_top))
        y1 = min(h, int(h * self.duck_roi_bottom))
        mask[:y0, :] = 0
        if y1 < h:
            mask[y1:, :] = 0
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        return mask

    def _duck_blobs(self, mask):
        # Zusammenhangskomponenten mit Formfilter (gegen Rauschen / Linien).
        num, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        blobs = []
        for i in range(1, num):
            x = stats[i, cv2.CC_STAT_LEFT];  y = stats[i, cv2.CC_STAT_TOP]
            w = stats[i, cv2.CC_STAT_WIDTH]; h = stats[i, cv2.CC_STAT_HEIGHT]
            area = stats[i, cv2.CC_STAT_AREA]
            if area < self.duck_min_area:
                continue
            if w < self.duck_min_w or h < self.duck_min_h:
                continue
            if w > 0 and (h / float(w)) > self.duck_line_max_aspect:
                continue
            blobs.append((int(x), int(y), int(w), int(h)))
        return blobs

    def _duck_occupancy(self, blobs, width):
        occ = np.zeros(self.OCC_BINS, dtype=np.float32)
        for (x, y, w, h) in blobs:
            b0 = max(0, min(self.OCC_BINS - 1, int(x / width * self.OCC_BINS)))
            b1 = max(0, min(self.OCC_BINS - 1, int((x + w) / width * self.OCC_BINS)))
            occ[b0:b1 + 1] = 1.0
        return occ

    def _duck_nearest_x(self, blobs, width):
        if not blobs:
            return -99.0
        nearest = max(blobs, key=lambda b: b[1] + b[3])
        cx = nearest[0] + nearest[2] / 2.0
        return (cx / width) * 2.0 - 1.0

    def _process_ducks(self, bev_bgr):
        # Vollständige Enten-Auswertung auf dem BEV. Published Profil + Trigger.
        try:
            w     = bev_bgr.shape[1]
            mask  = self._duck_object_mask(bev_bgr)
            blobs = self._duck_blobs(mask)
            occ   = self._duck_occupancy(blobs, w)
            duck_x = self._duck_nearest_x(blobs, w)

            self.pub_duck_occupancy.publish(Float32MultiArray(data=occ.tolist()))
            self.pub_duck.publish(Float64(data=duck_x))

            if blobs:
                rospy.loginfo_throttle(1.0,
                    f"[duck] {len(blobs)} Blobs, {int(occ.sum())}/{self.OCC_BINS} "
                    f"Spalten belegt, naechste x={duck_x:.2f}")

            # Debug-Bild
            if self.pub_debug_duck.get_num_connections() > 0:
                dbg = bev_bgr.copy()
                h, ww = dbg.shape[:2]
                y0 = int(h * self.duck_roi_top)
                cv2.line(dbg, (0, y0), (ww, y0), (0, 255, 255), 1)
                for (x, y, bw, bh) in blobs:
                    cv2.rectangle(dbg, (x, y), (x + bw, y + bh), (0, 0, 255), 2)
                bar_h = 18
                for i in range(self.OCC_BINS):
                    xa = int(i / self.OCC_BINS * ww)
                    xb = int((i + 1) / self.OCC_BINS * ww)
                    color = (0, 0, 255) if occ[i] > 0.5 else (0, 200, 0)
                    cv2.rectangle(dbg, (xa, h - bar_h), (xb, h), color, -1)
                self._publish_compressed(self.pub_debug_duck, dbg)
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"[duck] Fehler in _process_ducks: {e}")
            # Im Fehlerfall kein Trigger senden (alten Zustand nicht verfälschen)

    def _annotate(self, base):
        # Zeichnet alle Debug-Markierungen auf eine KOPIE des Bird's-Eye-Bildes.
        # Wird nur in run_debug (Main-Thread) aufgerufen.
        d = base.copy()
        h, w = len(d), len(d[0])
        y_look = int(h * 0.75)

        d = cv2.circle(d, (int(self.lane_center), int(h / 2)), 3, (255, 0, 0))
        d = cv2.line(d, (self.white_alternative, 0),
                     (self.white_alternative, h), color=(255, 255, 255))
        d = cv2.line(d, (self.yellow_alternative, 0),
                     (self.yellow_alternative, h), color=(255, 255, 0))
        d = cv2.line(d, (0, y_look + 100), (w, y_look + 100), color=(255, 255, 255))
        d = cv2.line(d, (0, y_look - 100), (w, y_look - 100), color=(255, 255, 255))
        d = cv2.line(d, (int(w / 2), 0), (int(w / 2), h), (0, 255, 0))
        d = cv2.circle(d, (int(self.center_white),  y_look), 5, (255, 255, 255))
        d = cv2.circle(d, (int(self.center_yellow), y_look), 5, (0, 255, 255))

        # ROI-Kasten der Haltelinien-Erkennung (rot)
        roi_top   = int(h * self.red_detection_zone)
        roi_left  = int(w * self.red_detection_x_start)
        roi_right = int(w * self.red_detection_x_end) - 1
        d = cv2.rectangle(d, (roi_left, roi_top),
                          (roi_right, h - 1), (0, 0, 255), 2)
        # Roter Vollrahmen, wenn Haltelinie aktiv
        if getattr(self, 'stop_line_detected', False):
            d = cv2.rectangle(d, (0, 0), (w - 1, h - 1), (0, 0, 255), 5)
        return d

    # ── Debug-Schleife (Main-Thread) ─────────────────────────────────────────────

    def run_debug(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # Ersten Frame abwarten, bevor Debug-Bilder gesendet werden
            if self.counter <= 3:
                rate.sleep()
                continue

            annotated = None  # nur bei Bedarf einmal erzeugen

            if self.pub_debug_lane.get_num_connections() > 0:
                annotated = self._annotate(self.img)
                self._publish_compressed(self.pub_debug_lane, annotated)

            if self.pub_debug_white.get_num_connections() > 0:
                self._publish_compressed(self.pub_debug_white, self.debug_img_white)

            if self.pub_debug_yellow.get_num_connections() > 0:
                self._publish_compressed(self.pub_debug_yellow, self.debug_img_yellow)

            if self.pub_debug_red.get_num_connections() > 0:
                self._publish_compressed(self.pub_debug_red, self.debug_img_red)

            # ── Lokale Debug-Fenster ──────────────────────────────────────────
            # GUI läuft NUR hier im Main-Thread (nie im ROS-Callback) → kein Freeze.
            # Aktivieren über self.show_window = True (oben in __init__).
            if self.show_window:
                if annotated is None:
                    annotated = self._annotate(self.img)
                cv2.imshow(f'{self._vehicle_name} - Bird-Eye annotiert', annotated)
                # Weitere Masken bei Bedarf einkommentieren:
                # cv2.imshow(f'{self._vehicle_name} - Weiss', self.debug_img_white)
                # cv2.imshow(f'{self._vehicle_name} - Gelb',  self.debug_img_yellow)
                # cv2.imshow(f'{self._vehicle_name} - Rot',   self.debug_img_red)
                cv2.waitKey(1)

            rate.sleep()


if __name__ == '__main__':
    node = DetectLaneNode('detect_lane_node')
    node.run_debug()
    rospy.spin()
