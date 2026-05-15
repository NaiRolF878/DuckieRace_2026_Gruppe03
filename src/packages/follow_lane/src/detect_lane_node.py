#!/usr/bin/env python3

import os
import rospy
import numpy as np
import cv2
from std_msgs.msg import Float64, String, Bool
from sensor_msgs.msg import CompressedImage
from enum import Enum
import yaml
import util

#from duckietown.dtros import DTROS, NodeType

class DetectLaneNode:
    def __init__(self, node_name):
        # ROS-Node initialisieren
        rospy.init_node(node_name)
        
        self._vehicle_name = os.environ['VEHICLE_NAME']
        util.init_parameters(node_name, self.cbUpdateParameters)
                
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self.sub_image_original = rospy.Subscriber(self._camera_topic, CompressedImage, self.cbFindLane, queue_size = 1)
        self.pub_lane = rospy.Publisher(f'/{self._vehicle_name}/detect/lane', Float64, queue_size = 1)

        # NEU: Publisher für rote Haltelinie (True = Linie erkannt)
        self.pub_stop_line      = rospy.Publisher(f'/{self._vehicle_name}/detect/stop_line',      Bool,   queue_size=1)
        # Publisher für die Seite der roten Linie ('none', 'left', 'right', 'both')
        # → wird von control_intersection_node für Abbiegeorientierung genutzt
        self.pub_stop_line_side = rospy.Publisher(f'/{self._vehicle_name}/detect/stop_line_side', String, queue_size=1)
        # Publisher für Spurpositionen → detect_duck_node benötigt diese für ROI
        self.pub_lane_yellow_x = rospy.Publisher(f'/{self._vehicle_name}/detect/lane_yellow_x', Float64, queue_size=1)
        self.pub_lane_white_x  = rospy.Publisher(f'/{self._vehicle_name}/detect/lane_white_x',  Float64, queue_size=1)

        self._crop_im_size = 400
        self.is_running = False
        self.counter = 0

        # Frame-Tracking: letzte bekannte weiße Linienposition
        # → wird verwendet um unplausible Sprünge zwischen Frames zu erkennen
        self.last_white_position = None

        # init debug channels 
        # Publisher für Original- und Bird's-Eye-View Bild (für camera_dashboard_node)
        self.pub_debug_original = rospy.Publisher(f'/{self._vehicle_name}/debug/original',   CompressedImage, queue_size=1)
        self.pub_debug_bird     = rospy.Publisher(f'/{self._vehicle_name}/debug/bird_view', CompressedImage, queue_size=1)
        self.pub_debug_lane   = rospy.Publisher(f'/{self._vehicle_name}/debug/lane_croped',  CompressedImage, queue_size=1)
        self.pub_debug_white  = rospy.Publisher(f'/{self._vehicle_name}/debug/lane_white',   CompressedImage, queue_size=1)
        self.pub_debug_yellow = rospy.Publisher(f'/{self._vehicle_name}/debug/lane_yellow',  CompressedImage, queue_size=1)
        # NEU: Debug-Publisher für rote Linien-Maske
        self.pub_debug_red    = rospy.Publisher(f'/{self._vehicle_name}/debug/lane_red',     CompressedImage, queue_size=1)
        # Speicher für Debug-Bild (wird in run_debug benötigt)
        self.debug_img_red = None


    def cbUpdateParameters(self, parameters):
        # Update white line parameters
        self.hue_white_l        = parameters["white"]["hl"]["default"]
        self.hue_white_h        = parameters["white"]["hh"]["default"]
        self.saturation_white_l = parameters["white"]["sl"]["default"]
        self.saturation_white_h = parameters["white"]["sh"]["default"]
        self.lightness_white_l  = parameters["white"]["vl"]["default"]
        self.lightness_white_h  = parameters["white"]["vh"]["default"]
        
        # Update yellow line parameters
        self.hue_yellow_l        = parameters["yellow"]["hl"]["default"]
        self.hue_yellow_h        = parameters["yellow"]["hh"]["default"]
        self.saturation_yellow_l = parameters["yellow"]["sl"]["default"]
        self.saturation_yellow_h = parameters["yellow"]["sh"]["default"]
        self.lightness_yellow_l  = parameters["yellow"]["vl"]["default"]
        self.lightness_yellow_h  = parameters["yellow"]["vh"]["default"]
        
        # Update perspective transform points
        self.top_left_x     = parameters["crop_image"]["top_left_x"]["default"]
        self.top_left_y     = parameters["crop_image"]["top_left_y"]["default"]
        self.top_right_x    = parameters["crop_image"]["top_right_x"]["default"]
        self.top_right_y    = parameters["crop_image"]["top_right_y"]["default"]
        self.bottom_left_x  = parameters["crop_image"]["bottom_left_x"]["default"]
        self.bottom_left_y  = parameters["crop_image"]["bottom_left_y"]["default"]
        self.bottom_right_x = parameters["crop_image"]["bottom_right_x"]["default"]
        self.bottom_right_y = parameters["crop_image"]["bottom_right_y"]["default"]

        # NEU: Update red stop line parameters
        # Rot liegt im HSV-Farbraum an zwei Stellen des Hue-Kreises:
        #   unterer Bereich: hl  .. hh  (z.B. 0  - 10 )
        #   oberer Bereich:  hl2 .. hh2 (z.B. 160 - 179)
        # Beide Masken werden kombiniert, um alle Rottöne zu erfassen.
        self.hue_red_l        = parameters["red"]["hl"]["default"]
        self.hue_red_h        = parameters["red"]["hh"]["default"]
        self.hue_red_l2       = parameters["red"]["hl2"]["default"]
        self.hue_red_h2       = parameters["red"]["hh2"]["default"]
        self.saturation_red_l = parameters["red"]["sl"]["default"]
        self.saturation_red_h = parameters["red"]["sh"]["default"]
        self.lightness_red_l  = parameters["red"]["vl"]["default"]
        self.lightness_red_h  = parameters["red"]["vh"]["default"]
        # Mindestanzahl roter Pixel im ROI, ab der eine Haltelinie gemeldet wird
        self.red_pixel_threshold = parameters["red"]["pixel_threshold"]["default"]

        # Wo im Bild nach der roten Linie gesucht wird (vertikal)
        # 0.85 = nur die untersten 15% prüfen → Bot hält erst kurz vor der Linie an
        self.red_detection_zone    = parameters["red"]["detection_zone"]["default"]

        # Horizontale ROI-Einschränkung für rote Linie
        # 0.4 = nur die rechten 60% prüfen → Gegenspur-Haltelinie wird ignoriert
        self.red_detection_x_start = parameters["red"]["detection_x_start"]["default"]

        # Minimaler Pixelabstand zwischen gelber und weißer Linie
        # → alles links davon in der weißen Maske wird ausgeblendet (Gegenspur)
        self.min_lane_width  = parameters["white"]["min_lane_width"]["default"]

        # Maximaler Pixelsprung der weißen Linie zwischen zwei Frames
        # → größere Sprünge gelten als Fehlmessung, letzter Wert wird beibehalten
        self.max_frame_jump  = parameters["white"]["max_frame_jump"]["default"]


    def crop_img(self, img):
        img = img.copy()
        
        pts1 = np.float32([
            [self.top_left_x,     self.top_left_y],
            [self.top_right_x,    self.top_right_y],
            [self.bottom_right_x, self.bottom_right_y],
            [self.bottom_left_x,  self.bottom_left_y],])
        
        pts2 = np.float32([[0,0],[self._crop_im_size,0],[0,self._crop_im_size],[self._crop_im_size,self._crop_im_size]])

        M = cv2.getPerspectiveTransform(pts1, pts2)
        return cv2.warpPerspective(img, M, (self._crop_im_size, self._crop_im_size))


    def get_x_for_driving(self, mask, distance, no_lane_value, left_line):
        grad = cv2.Sobel(mask, cv2.CV_16S, 1, 0, ksize=3, scale=1, delta=0, borderType=cv2.BORDER_DEFAULT)
        _,th1 = cv2.threshold(grad, 127, 255, cv2.THRESH_BINARY)

        a = []
        for row in range(distance-50, distance+50):
            if np.where(th1[row] == 255)[0].size == 0:
                continue
            else:
                if left_line:
                    a.append(np.where(th1[row] == 255)[0][-1])
                else:
                    a.append(np.where(th1[row] == 255)[0][0])

        if len(a) > 10:
            return np.median(a)
        else:
            return no_lane_value


    # NEU: Rote Haltelinie im Bird's-Eye-View-Bild erkennen
    def detect_stop_line(self, hsv, cv_image):
        # Zwei HSV-Masken für den unteren und oberen Rot-Bereich erzeugen
        mask_red_lower = cv2.inRange(hsv,
                            (self.hue_red_l,  self.saturation_red_l, self.lightness_red_l),
                            (self.hue_red_h,  self.saturation_red_h, self.lightness_red_h))
        mask_red_upper = cv2.inRange(hsv,
                            (self.hue_red_l2, self.saturation_red_l, self.lightness_red_l),
                            (self.hue_red_h2, self.saturation_red_h, self.lightness_red_h))
        mask_red = cv2.bitwise_or(mask_red_lower, mask_red_upper)

        # ── Eigene Haltelinie erkennen (bestehende ROI) ───────────────────────
        # Vertikale ROI: nur den unteren Teil des Bildes prüfen
        detection_row_start = int(mask_red.shape[0] * self.red_detection_zone)
        # Horizontale ROI: nur die rechte Seite prüfen (eigene Spur)
        detection_col_start = int(mask_red.shape[1] * self.red_detection_x_start)
        roi_own = mask_red[detection_row_start:, detection_col_start:]

        red_pixel_count    = cv2.countNonZero(roi_own)
        stop_line_detected = red_pixel_count > self.red_pixel_threshold

        print(f"Red pixels (own): {red_pixel_count} | threshold: {self.red_pixel_threshold} | detected: {stop_line_detected}")

        # ── Seitenerkennung: auf welcher Seite ist rote Linie sichtbar? ──────
        # Wird von control_intersection_node verwendet um beim Abbiegen zu wissen
        # wann der Bot korrekt ausgerichtet ist:
        #   Links abbiegen  → warte bis rote Linie RECHTS erscheint
        #   Rechts abbiegen → warte bis rote Linie LINKS erscheint
        #   Geradeaus       → warte bis rote Linie unten aus dem Bild verschwindet
        #
        # WICHTIG: Seitenerkennung auf dem ORIGINALBILD (nicht Bird's-Eye-View)
        # → Bird's-Eye-View ist auf eigene Spur kalibriert, Gegenspur liegt
        #   oft am Rand oder außerhalb des transformierten Bereichs
        # → Im Originalbild ist die gesamte Kreuzung inkl. Gegenspur sichtbar
        hsv_original = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask_orig_lower = cv2.inRange(hsv_original,
                            (self.hue_red_l,  self.saturation_red_l, self.lightness_red_l),
                            (self.hue_red_h,  self.saturation_red_h, self.lightness_red_h))
        mask_orig_upper = cv2.inRange(hsv_original,
                            (self.hue_red_l2, self.saturation_red_l, self.lightness_red_l),
                            (self.hue_red_h2, self.saturation_red_h, self.lightness_red_h))
        mask_orig_red = cv2.bitwise_or(mask_orig_lower, mask_orig_upper)

        # Nur den unteren Teil des Originalbildes prüfen (Kreuzungsbereich)
        orig_h, orig_w = mask_orig_red.shape[:2]
        mid            = orig_w // 2
        row_start      = int(orig_h * self.red_detection_zone)

        left_count  = cv2.countNonZero(mask_orig_red[row_start:, :mid])
        right_count = cv2.countNonZero(mask_orig_red[row_start:, mid:])

        left_detected  = left_count  > self.red_pixel_threshold
        right_detected = right_count > self.red_pixel_threshold

        if left_detected and right_detected:
            side = 'both'   # Bot steht direkt auf/vor der Linie
        elif left_detected:
            side = 'left'   # rote Linie auf linker Seite (Gegenspur)
        elif right_detected:
            side = 'right'  # rote Linie auf rechter Seite (eigene Spur)
        else:
            side = 'none'   # keine rote Linie sichtbar

        print(f"Red line side: {side} (left={left_count}, right={right_count})")

        return stop_line_detected, side, mask_red

    def cbFindLane(self, image_msg):
        
        if self.counter <= 3:
            self.counter += 1   
            return

        if self.is_running:
            return
        
        self.is_running = True
        self.conunter = 0

        np_arr = np.frombuffer(image_msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        img = self.crop_img(cv_image)

        # Originalbild fuer Dashboard publizieren (vor jeder Verarbeitung)
        if self.pub_debug_original.get_num_connections() > 0:
            orig_msg = CompressedImage()
            orig_msg.header.stamp = rospy.Time.now()
            orig_msg.format = "jpeg"
            orig_msg.data = np.array(cv2.imencode('.jpg', cv_image)[1]).tobytes()
            self.pub_debug_original.publish(orig_msg)

        # Bird's-Eye-View fuer Dashboard publizieren (direkt nach Transformation)
        if self.pub_debug_bird.get_num_connections() > 0:
            bird_msg = CompressedImage()
            bird_msg.header.stamp = rospy.Time.now()
            bird_msg.format = "jpeg"
            bird_msg.data = np.array(cv2.imencode('.jpg', img)[1]).tobytes()
            self.pub_debug_bird.publish(bird_msg)

        # CLAHE: lokalen Helligkeitsausgleich durchführen bevor wir in HSV konvertieren
        # → macht die Farbsegmentierung robuster gegenüber Schatten und wechselndem Licht
        # Ablauf: BGR → LAB (trennt Helligkeit L von Farbe) → CLAHE nur auf L-Kanal → zurück zu BGR
        lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        l = clahe.apply(l)
        img = cv2.cvtColor(cv2.merge((l, a, b)), cv2.COLOR_LAB2BGR)

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        mask_yellow = cv2.inRange(hsv, 
                           (self.hue_yellow_l, self.saturation_yellow_l, self.lightness_yellow_l), 
                           (self.hue_yellow_h, self.saturation_yellow_h, self.lightness_yellow_h),)
        
        mask_white = cv2.inRange(hsv, 
                           (self.hue_white_l, self.saturation_white_l, self.lightness_white_l), 
                           (self.hue_white_h, self.saturation_white_h, self.lightness_white_h),)

        # Morphologie: kleine Lücken in den Masken schließen die durch Schatten entstehen
        # MORPH_CLOSE = erst Dilatation (Lücken füllen), dann Erosion (Form wiederherstellen)
        kernel = np.ones((5, 5), np.uint8)
        mask_white  = cv2.morphologyEx(mask_white,  cv2.MORPH_CLOSE, kernel)
        mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_CLOSE, kernel)
        
        white_alternative  = int(len(img[0]) * 0.95)
        yellow_alternative = int(len(img[0]) * 0.05)

        # Gelb zuerst bestimmen – wird für den Weißlinien-Gegenspurfilter benötigt
        center_yellow = self.get_x_for_driving(mask_yellow, int(len(img)*0.75), yellow_alternative, left_line=False)

        # Spatial Filter: weiße Linie nur rechts von gelb + min_lane_width suchen
        # → blendet die Gegenspur-Weiße aus die in engen Kurven nahe an eigener Linie liegt
        mask_white_filtered = mask_white.copy()
        search_start = int(center_yellow + self.min_lane_width)
        mask_white_filtered[:, :search_start] = 0  # alles links von search_start ausblenden

        # Weiße Linienposition in der gefilterten Maske bestimmen
        center_white_raw = self.get_x_for_driving(mask_white_filtered, int(len(img)*0.75), white_alternative, left_line=True)

        # Frame-Tracking: Plausibilität des Sprungs prüfen
        # → verhindert dass einzelne Fehlmessungen den Bot abrupt auslenken
        if self.last_white_position is not None:
            jump = abs(center_white_raw - self.last_white_position)
            if jump > self.max_frame_jump:
                # Sprung zu groß → Fehlmessung → letzten bekannten Wert beibehalten
                print(f"White jump too large ({jump:.0f}px > {self.max_frame_jump}px) – keeping last position")
                center_white = self.last_white_position
            else:
                # Sprung plausibel → neuen Wert übernehmen
                center_white = center_white_raw
                self.last_white_position = center_white
        else:
            # Erster Frame: Wert direkt übernehmen (kein Vergleich möglich)
            center_white = center_white_raw
            self.last_white_position = center_white

        if center_white <= center_yellow:
            if center_white > int(len(img[0]) * 0.4):
                center_yellow = yellow_alternative
            else:
                center_white = white_alternative

        lane_center = (center_white + center_yellow) / 2

        msg_error = Float64()
        msg_error.data = 1-(lane_center / len(img) * 2)
        self.pub_lane.publish(msg_error)
        # Spurpositionen für detect_duck_node publizieren
        self.pub_lane_yellow_x.publish(Float64(data=float(center_yellow)))
        self.pub_lane_white_x.publish(Float64(data=float(center_white)))
        print(f"Lane error: {msg_error.data} range [-1,1]")

        # NEU: Rote Haltelinie erkennen und Ergebnis publizieren
        # cv_image = Originalbild für Seitenerkennung, hsv = Bird's-Eye-View für eigene Linie
        stop_line_detected, stop_line_side, mask_red = self.detect_stop_line(hsv, cv_image)
        self.pub_stop_line.publish(Bool(data=stop_line_detected))
        self.pub_stop_line_side.publish(String(data=stop_line_side))

        # saving for debug
        self.img             = img
        self.lane_center     = lane_center
        self.white_alternative  = white_alternative
        self.yellow_alternative = yellow_alternative
        self.center_white    = center_white
        self.center_yellow   = center_yellow
        self.debug_img_white = mask_white
        self.debug_img_yellow = mask_yellow
        self.debug_img_red   = mask_red

        image = cv2.circle(img,(int(lane_center),int(len(img) / 2)),3,(255,0,0))
        image = cv2.line(image, (white_alternative , 0), (white_alternative , self._crop_im_size) ,color=(255,255,255)) 
        image = cv2.line(image, (yellow_alternative , 0), (yellow_alternative , self._crop_im_size) ,color=(255,255,0))
        image = cv2.line(image, (0,int(len(img) * 0.75) + 100) , (len(img[0]),int(len(img) * 0.75) + 100), color=(255,255,255) )
        image = cv2.line(image, (0,int(len(img) * 0.75) - 100) , (len(img[0]),int(len(img) * 0.75) - 100), color=(255,255,255))
        image = cv2.line(image,(int(len(img[0])/2),0),(int(len(img[0])/2),len(image)),(0,255,0))
        image = cv2.circle(image, (int(center_white), int(len(img) * 0.75)),  5,(255,255,255))
        image = cv2.circle(image, (int(center_yellow), int(len(img) * 0.75)), 5,(0,255,255))

        # ROI-Kasten der Haltelinien-Erkennung einzeichnen (rot)
        # zeigt exakt den Bereich in dem nach roter Linie gesucht wird
        # vertikal:    detection_zone    → obere Kante
        # horizontal:  detection_x_start → linke Kante
        roi_top  = int(len(img)    * self.red_detection_zone)
        roi_left = int(len(img[0]) * self.red_detection_x_start)
        image = cv2.rectangle(image, (roi_left, roi_top), (self._crop_im_size - 1, self._crop_im_size - 1), (0, 0, 255), 2)
        # NEU: Bei erkannter Haltelinie → roter Rahmen ums gesamte Bild
        if stop_line_detected:
            image = cv2.rectangle(image, (0, 0), (self._crop_im_size - 1, self._crop_im_size - 1), (0, 0, 255), 5)

        # cv2.imshow entfernt → Darstellung erfolgt im camera_dashboard_node
        self.is_running = False
        
            
    def run_debug(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            # add debug info to lane image
            if self.pub_debug_lane.get_num_connections() > 0:
                debug_img = self.img.copy()
                debug_img = cv2.circle(debug_img,(int(self.lane_center),int(len(debug_img) / 2)),3,(255,0,0))
                debug_img = cv2.line(debug_img, (self.white_alternative , 0), (self.white_alternative , 1000) ,color=(255,255,255)) 
                debug_img = cv2.line(debug_img, (self.yellow_alternative , 0), (self.yellow_alternative , 1000) ,color=(255,255,0))
                debug_img = cv2.line(debug_img, (0,int(len(debug_img) * 0.75) + 100) , (len(debug_img[0]),int(len(debug_img) * 0.75) + 100), color=(255,255,255) )
                debug_img = cv2.line(debug_img, (0,int(len(debug_img) * 0.75) - 100) , (len(debug_img[0]),int(len(debug_img) * 0.75) - 100), color=(255,255,255))
                debug_img = cv2.line(debug_img,(int(len(debug_img[0])/2),0),(int(len(debug_img[0])/2),len(debug_img)),(0,255,0))
                debug_img = cv2.circle(debug_img, (int(self.center_white), int(len(debug_img) * 0.75)),  5,(255,255,255))
                debug_img = cv2.circle(debug_img, (int(self.center_yellow), int(len(debug_img) * 0.75)), 5,(0,255,255))
                # ROI-Kasten der Haltelinien-Erkennung einzeichnen (rot)
                roi_top  = int(len(debug_img)    * self.red_detection_zone)
                roi_left = int(len(debug_img[0]) * self.red_detection_x_start)
                debug_img = cv2.rectangle(debug_img, (roi_left, roi_top), (self._crop_im_size - 1, self._crop_im_size - 1), (0, 0, 255), 2)

                debug_msg = CompressedImage()
                debug_msg.header.stamp = rospy.Time.now()
                debug_msg.format = "jpeg"
                debug_msg.data = np.array(cv2.imencode('.jpg', debug_img)[1]).tobytes()
                self.pub_debug_lane.publish(debug_msg)

            if self.pub_debug_white.get_num_connections() > 0:
                debug_msg = CompressedImage()
                debug_msg.header.stamp = rospy.Time.now()
                debug_msg.format = "jpeg"
                debug_msg.data = np.array(cv2.imencode('.jpg', self.debug_img_white)[1]).tobytes()
                self.pub_debug_white.publish(debug_msg)

            if self.pub_debug_yellow.get_num_connections() > 0:
                debug_msg = CompressedImage()
                debug_msg.header.stamp = rospy.Time.now()
                debug_msg.format = "jpeg"
                debug_msg.data = np.array(cv2.imencode('.jpg', self.debug_img_yellow)[1]).tobytes()
                self.pub_debug_yellow.publish(debug_msg)

            # NEU: Rote-Linie-Maske als Debug-Bild veröffentlichen
            if self.pub_debug_red.get_num_connections() > 0 and self.debug_img_red is not None:
                debug_msg = CompressedImage()
                debug_msg.header.stamp = rospy.Time.now()
                debug_msg.format = "jpeg"
                debug_msg.data = np.array(cv2.imencode('.jpg', self.debug_img_red)[1]).tobytes()
                self.pub_debug_red.publish(debug_msg)

            rate.sleep()
        
if __name__ == '__main__':
    node = DetectLaneNode('detect_lane_node')
    node.run_debug()
    rospy.spin()
