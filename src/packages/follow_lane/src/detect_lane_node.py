#!/usr/bin/env python3

import os
import rospy
import numpy as np
import cv2
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import CompressedImage
import util

#from duckietown.dtros import DTROS, NodeType

class DetectLaneNode:
    def __init__(self, node_name):
        # ROS-Node initialisieren
        rospy.init_node(node_name)

        # Fahrzeugnamen aus Umgebungsvariable lesen
        self._vehicle_name = os.environ['VEHICLE_NAME']

        # ── Kritisch: Alle self.*-Variablen VOR util.init_parameters setzen ──
        # util.init_parameters ruft cbUpdateParameters sofort auf.
        # Falls dort ein Fehler passiert bricht __init__ ab und alle
        # danach definierten Variablen fehlen → AttributeError in Callbacks.
        self._crop_im_size      = 400
        self.is_running          = False
        self.counter             = 0     # Frame-Zähler: erste 3 Frames verwerfen
        self.last_white_position  = None  # Frame-Tracking für weiße Linie
        self.last_yellow_position = None  # Frame-Tracking für gelbe Linie

        # Platzhalter für Debug-Variablen
        # → verhindert AttributeError wenn run_debug vor erstem Frame läuft
        blank       = np.zeros((self._crop_im_size, self._crop_im_size), dtype=np.uint8)
        blank_color = np.zeros((self._crop_im_size, self._crop_im_size, 3), dtype=np.uint8)
        self.img              = blank_color
        self.lane_center      = self._crop_im_size / 2
        self.white_alternative  = int(self._crop_im_size * 0.95)
        self.yellow_alternative = int(self._crop_im_size * 0.05)
        self.center_white     = int(self._crop_im_size * 0.95)
        self.center_yellow    = int(self._crop_im_size * 0.05)
        self.debug_img_white  = blank
        self.debug_img_yellow = blank
        self.debug_img_red    = blank

        # Parameter aus JSON laden + Live-Update Callback registrieren
        # NACH den self.*-Variablen – cbUpdateParameters kann jetzt sicher aufgerufen werden
        util.init_parameters(node_name, self.cbUpdateParameters)

        # ── Subscriber ────────────────────────────────────────────────────────
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self.sub_image_original = rospy.Subscriber(
            self._camera_topic, CompressedImage, self.cbFindLane, queue_size=1)

        # ── Publisher ─────────────────────────────────────────────────────────
        # Spurversatz [-1, +1] an control_lane_node
        self.pub_lane = rospy.Publisher(
            f'/{self._vehicle_name}/detect/lane', Float64, queue_size=1)

        # Rote Haltelinie erkannt (True/False)
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


    def cbUpdateParameters(self, parameters):
        # Wird beim Start UND bei jeder Schieberegler-Änderung aufgerufen

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
        # Mindestanzahl roter Pixel im ROI
        self.red_pixel_threshold  = parameters["red"]["pixel_threshold"]["default"]
        # Vertikale ROI: 0.85 = nur unterste 15% des Bildes prüfen
        self.red_detection_zone    = parameters["red"]["detection_zone"]["default"]
        # Horizontale ROI: 0.4 = nur rechte 60% prüfen (Gegenspur ignorieren)
        self.red_detection_x_start = parameters["red"]["detection_x_start"]["default"]
        # Rechte Begrenzung: 1.0 = bis Bildrand. < 1.0 schneidet rechts ab
        # (z.B. wenn rechts vom Bot Störungen wie rote Markierungen am Wendeplatz liegen)
        self.red_detection_x_end   = parameters["red"]["detection_x_end"]["default"]

        # Frame-Tracking: maximaler Pixelsprung zwischen Frames (gilt für beide Linien)
        self.max_frame_jump = parameters["white"]["max_frame_jump"]["default"]


    def crop_img(self, img):
        # Bird's-Eye-View Transformation: Trapez der Fahrspur → Quadrat (Draufsicht)
        img = img.copy()

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


    def get_x_for_driving(self, mask, distance, left_line, last_known=None):
        # Linienposition per Sobel-Kantenerkennung bestimmen.
        #
        # last_known: letzter bekannter Pixelwert dieser Linie (oder None beim ersten Frame).
        #   → None:       Initialisierung – rechteste Kante für Gelb, linkeste für Weiß
        #   → Wert:       nächste Kante zum letzten bekannten Wert wählen
        #                 → robuster in engen Kurven und am Wendeplatz (keine Fehlzuordnung
        #                    wenn zwei weiße Kanten sichtbar sind)
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
        return None  # keine Detektion – Aufrufer entscheidet wie weiter

  def _resolve_line_position(self, raw, last_known, fallback, max_jump, label):
        # Entscheidet, welche x-Position als finale Linienposition gilt.
        #
        # raw         : Ergebnis von get_x_for_driving (Pixel oder None)
        # last_known  : self.last_yellow_position bzw. self.last_white_position
        # fallback    : Bildrand-Fallback (yellow_alternative / white_alternative)
        # max_jump    : maximaler erlaubter Sprung zwischen Frames
        # label       : 'Yellow' / 'White' für Logging
        #
        # Rückgabe: (finale_position, neuer_last_known)
        #   neuer_last_known ist None, wenn der aktuelle Wert nicht zum Anker werden soll
        #   (Fall: kein last_known UND keine Detektion → Bildrand, aber nicht ankern).
 
        # Fall A: keine Detektion
        if raw is None:
            if last_known is not None:
                # last_known ist die beste Schätzung – Anker beibehalten
                print(f"{label}: no edges – keeping last position {last_known:.0f}")
                return last_known, last_known
            # Erster Frame ohne Detektion → Bildrand-Fallback, aber NICHT ankern
            # (sonst würde get_x_for_driving danach nach Kanten in Bildrand-Nähe suchen)
            print(f"{label}: no edges and no last known – using image-edge fallback {fallback}")
            return fallback, None
 
        # Fall B: Detektion vorhanden, aber kein Anker → jetzt ankern
        if last_known is None:
            return raw, raw
 
        # Fall C: Detektion + Anker → Sprung prüfen
        jump = abs(raw - last_known)
        if jump > max_jump:
            print(f"{label} jump too large ({jump:.0f}px) – keeping last position")
            return last_known, last_known
        return raw, raw
 
    def detect_stop_line(self, hsv):
        # Rote Haltelinie im Bird's-Eye-View erkennen.
        # Rot liegt an zwei Stellen des Hue-Kreises → zwei Bereiche vereinen.
        mask_red_lower = cv2.inRange(hsv,
            (self.hue_red_l,  self.saturation_red_l, self.lightness_red_l),
            (self.hue_red_h,  self.saturation_red_h, self.lightness_red_h))
        mask_red_upper = cv2.inRange(hsv,
            (self.hue_red_l2, self.saturation_red_l, self.lightness_red_l),
            (self.hue_red_h2, self.saturation_red_h, self.lightness_red_h))
        # Beide Masken kombinieren
        mask_red = cv2.bitwise_or(mask_red_lower, mask_red_upper)

        # Vertikale + horizontale ROI: nur eigene Spur, direkt vor Bot
        # detection_zone     → schneidet oben ab (nur unterer Bildteil)
        # detection_x_start  → schneidet links ab
        # detection_x_end    → schneidet rechts ab
        detection_row_start = int(mask_red.shape[0] * self.red_detection_zone)
        detection_col_start = int(mask_red.shape[1] * self.red_detection_x_start)
        detection_col_end   = int(mask_red.shape[1] * self.red_detection_x_end)
        roi_own = mask_red[detection_row_start:, detection_col_start:detection_col_end]
        
        # Haltelinie erkannt, wenn genug rote Pixel im ROI vorhanden sind
        red_pixel_count    = cv2.countNonZero(roi_own)
        stop_line_detected = red_pixel_count > self.red_pixel_threshold
        print(f"Red pixels: {red_pixel_count} | threshold: {self.red_pixel_threshold} | detected: {stop_line_detected}")

        return stop_line_detected, mask_red


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

        # ── Schritt 3: CLAHE – lokaler Helligkeitsausgleich ───────────────────
        # BGR → LAB → CLAHE nur auf L-Kanal (Helligkeit) → zurück zu BGR
        # Warum LAB: L-Kanal ist von Farbe getrennt → HSV-Kalibrierung bleibt stabil
        lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        l = clahe.apply(l)
        img = cv2.cvtColor(cv2.merge((l, a, b)), cv2.COLOR_LAB2BGR)

        # ── Schritt 4: HSV-Masken ─────────────────────────────────────────────
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        mask_yellow = cv2.inRange(hsv,
            (self.hue_yellow_l, self.saturation_yellow_l, self.lightness_yellow_l),
            (self.hue_yellow_h, self.saturation_yellow_h, self.lightness_yellow_h))
        
        mask_white = cv2.inRange(hsv,
            (self.hue_white_l, self.saturation_white_l, self.lightness_white_l),
            (self.hue_white_h, self.saturation_white_h, self.lightness_white_h))

        # Morphologie: Schatten-Lücken in Masken schließen
        kernel = np.ones((5, 5), np.uint8)
        mask_white  = cv2.morphologyEx(mask_white,  cv2.MORPH_CLOSE, kernel)
        mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_CLOSE, kernel)

        # ── Schritt 5: Linienpositionen ───────────────────────────────────────
        white_alternative  = int(len(img[0]) * 0.95)
        yellow_alternative = int(len(img[0]) * 0.05)

        # ── Gelbe Linie ───────────────────────────────────────────────────────
        center_yellow_raw = self.get_x_for_driving(
            mask_yellow, distance, left_line=True,
            last_known=self.last_yellow_position)
        center_yellow, self.last_yellow_position = self._resolve_line_position(
            center_yellow_raw, self.last_yellow_position,
            yellow_alternative, self.max_frame_jump, label='Yellow')
     
        # ── Weiße Linie ───────────────────────────────────────────────────────
        # Frame-Tracking verhindern Sprünge zur Gegenspur in engen Kurven.
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

        # ── Schritt 6: Spurversatz berechnen ─────────────────────────────────
        lane_center = (center_white + center_yellow) / 2
        msg_error = Float64()
        msg_error.data = 1 - (lane_center / len(img) * 2)
        self.pub_lane.publish(msg_error)
        print(f"Lane error: {msg_error.data:.3f} range [-1,1]")

        # ── Schritt 7: Rote Haltelinie ────────────────────────────────────────
        stop_line_detected, mask_red = self.detect_stop_line(hsv)
        self.pub_stop_line.publish(Bool(data=stop_line_detected))

        # ── Schritt 8: Debug-Variablen speichern ─────────────────────────────
        self.img              = img
        self.lane_center      = lane_center
        self.white_alternative  = white_alternative
        self.yellow_alternative = yellow_alternative
        self.center_white     = center_white
        self.center_yellow    = center_yellow
        self.debug_img_white  = mask_white
        self.debug_img_yellow = mask_yellow
        self.debug_img_red    = mask_red

        # ── Schritt 9: Annotiertes Bild ───────────────────────────────────────
        image = cv2.circle(img, (int(lane_center), int(len(img) / 2)), 3, (255, 0, 0))
        image = cv2.line(image, (white_alternative, 0),
                         (white_alternative, self._crop_im_size), color=(255, 255, 255))
        image = cv2.line(image, (yellow_alternative, 0),
                         (yellow_alternative, self._crop_im_size), color=(255, 255, 0))
        image = cv2.line(image, (0, int(len(img)*0.75)+100),
                         (len(img[0]), int(len(img)*0.75)+100), color=(255, 255, 255))
        image = cv2.line(image, (0, int(len(img)*0.75)-100),
                         (len(img[0]), int(len(img)*0.75)-100), color=(255, 255, 255))
        image = cv2.line(image, (int(len(img[0])/2), 0),
                         (int(len(img[0])/2), len(image)), (0, 255, 0))
        image = cv2.circle(image, (int(center_white),  int(len(img)*0.75)), 5, (255, 255, 255))
        image = cv2.circle(image, (int(center_yellow), int(len(img)*0.75)), 5, (0, 255, 255))

        # ROI-Kasten der Haltelinien-Erkennung (rot)
        roi_top   = int(len(img)    * self.red_detection_zone)
        roi_left  = int(len(img[0]) * self.red_detection_x_start)
        roi_right = int(len(img[0]) * self.red_detection_x_end) - 1
        image = cv2.rectangle(image,
            (roi_left, roi_top), (roi_right, self._crop_im_size-1), (0, 0, 255), 2)
        # Roter Rahmen wenn Haltelinie aktiv
        if stop_line_detected:
            image = cv2.rectangle(image,
                (0, 0), (self._crop_im_size-1, self._crop_im_size-1), (0, 0, 255), 5)

        # Annotiertes Bild publizieren → camera_dashboard_node (oben rechts)
        if self.pub_debug_annotated.get_num_connections() > 0:
            self._publish_compressed(self.pub_debug_annotated, image)

        # Lokale Debug-Ansicht – bei Bedarf einkommentieren:
        # cv2.imshow('Bird\'s-Eye-View annotiert', image)
        # cv2.imshow('Original',    cv_image)
        # cv2.imshow('Weiss-Maske', mask_white)
        # cv2.imshow('Gelb-Maske',  mask_yellow)
        # cv2.imshow('Rot-Maske',   mask_red)
        # cv2.waitKey(1)

        self.is_running = False


    def _publish_compressed(self, publisher, img):
        # Hilfsfunktion: OpenCV-Bild als komprimierte ROS-Message senden
        msg              = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format       = "jpeg"
        msg.data         = np.array(cv2.imencode('.jpg', img)[1]).tobytes()
        publisher.publish(msg)


    def run_debug(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            # Ersten Frame abwarten bevor Debug-Bilder gesendet werden
            if self.counter <= 3:
                rate.sleep()
                continue

            if self.pub_debug_lane.get_num_connections() > 0:
                debug_img = self.img.copy()
                debug_img = cv2.circle(debug_img,
                    (int(self.lane_center), int(len(debug_img)/2)), 3, (255, 0, 0))
                debug_img = cv2.line(debug_img,
                    (self.white_alternative, 0), (self.white_alternative, 1000), color=(255, 255, 255))
                debug_img = cv2.line(debug_img,
                    (self.yellow_alternative, 0), (self.yellow_alternative, 1000), color=(255, 255, 0))
                debug_img = cv2.line(debug_img,
                    (0, int(len(debug_img)*0.75)+100), (len(debug_img[0]), int(len(debug_img)*0.75)+100),
                    color=(255, 255, 255))
                debug_img = cv2.line(debug_img,
                    (0, int(len(debug_img)*0.75)-100), (len(debug_img[0]), int(len(debug_img)*0.75)-100),
                    color=(255, 255, 255))
                debug_img = cv2.line(debug_img,
                    (int(len(debug_img[0])/2), 0), (int(len(debug_img[0])/2), len(debug_img)), (0, 255, 0))
                debug_img = cv2.circle(debug_img,
                    (int(self.center_white),  int(len(debug_img)*0.75)), 5, (255, 255, 255))
                debug_img = cv2.circle(debug_img,
                    (int(self.center_yellow), int(len(debug_img)*0.75)), 5, (0, 255, 255))
                roi_top   = int(len(debug_img)    * self.red_detection_zone)
                roi_left  = int(len(debug_img[0]) * self.red_detection_x_start)
                roi_right = int(len(debug_img[0]) * self.red_detection_x_end) - 1
                debug_img = cv2.rectangle(debug_img,
                    (roi_left, roi_top), (roi_right, self._crop_im_size-1), (0, 0, 255), 2)
                self._publish_compressed(self.pub_debug_lane, debug_img)

            if self.pub_debug_white.get_num_connections() > 0:
                self._publish_compressed(self.pub_debug_white, self.debug_img_white)

            if self.pub_debug_yellow.get_num_connections() > 0:
                self._publish_compressed(self.pub_debug_yellow, self.debug_img_yellow)

            if self.pub_debug_red.get_num_connections() > 0:
                self._publish_compressed(self.pub_debug_red, self.debug_img_red)

            rate.sleep()


if __name__ == '__main__':
    node = DetectLaneNode('detect_lane_node')
    node.run_debug()
    rospy.spin()
