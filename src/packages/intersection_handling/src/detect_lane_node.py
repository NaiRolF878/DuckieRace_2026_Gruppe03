#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────────────────────
# detect_lane_node.py  (Challenge 2 – Intersection Handling)
#
# Spurerkennung im Bird's-Eye-View + rote Haltelinie.
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
from std_msgs.msg import Float64, Bool
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
        # Bewegung wird sanft auf max_jump begrenzt statt den Sprung voll zu übernehmen:
        # die Position darf sich pro Frame höchstens um max_jump Pixel Richtung neuem
        # Wert bewegen (verhindert Sprung zur Gegenspur an der Kreuzung).
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

        # ── Schritt 1: Bild dekodieren ────────────────────────────────────────
        np_arr   = np.frombuffer(image_msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

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

        self.is_running = False

    # ── Hilfsfunktionen ──────────────────────────────────────────────────────────

    def _publish_compressed(self, publisher, img):
        # OpenCV-Bild als komprimierte ROS-Message senden
        msg              = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format       = "jpeg"
        msg.data         = np.array(cv2.imencode('.jpg', img)[1]).tobytes()
        publisher.publish(msg)

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
