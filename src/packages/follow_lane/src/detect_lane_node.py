#!/usr/bin/env python3

import os
import rospy
import numpy as np
import cv2
from std_msgs.msg import Float64, String
from sensor_msgs.msg import CompressedImage
from enum import Enum
import yaml
import util

#from duckietown.dtros import DTROS, NodeType

class DetectLaneNode:
    def __init__(self, node_name):
        # ROS-Node initialisieren
        rospy.init_node(node_name)
        
        # Fahrzeugnamen aus Umgebungsvariable lesen (z.B. "duckiebot01")
        self._vehicle_name = os.environ['VEHICLE_NAME']

        # Parameter aus der zugehörigen JSON-Konfigurationsdatei laden
        # und Callback für spätere Live-Aktualisierungen registrieren
        util.init_parameters(node_name,self.cbUpdateParameters)
                
        # Topic des Kamera-Streams definieren
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"

        # Subscriber: empfängt komprimierte Kamerabilder und ruft cbFindLane auf
        self.sub_image_original = rospy.Subscriber(self._camera_topic, CompressedImage, self.cbFindLane, queue_size = 1)

        # Publisher: veröffentlicht den berechneten Spurversatz im Bereich [-1, 1]
        # (negativ = zu weit links, positiv = zu weit rechts)
        self.pub_lane = rospy.Publisher(f'/{self._vehicle_name}/detect/lane', Float64, queue_size = 1)

        # Größe des transformierten (Bird's-Eye-View) Ausschnitts in Pixel
        self._crop_im_size = 400

        # Sperrvariable: verhindert parallele Verarbeitung mehrerer Frames
        self.is_running = False

        # Zähler zum Überspringen der ersten Frames nach dem Start
        # (z.B. um unscharfe oder unvollständige Startbilder zu verwerfen)
        self.counter = 0

        # Debug-Publisher: veröffentlichen aufbereitete Zwischenbilder zur Visualisierung
        # Kann z.B. mit rqt_image_view angezeigt werden
        self.pub_debug_lane   = rospy.Publisher(f'/{self._vehicle_name}/debug/lane_croped',  CompressedImage, queue_size=1)
        self.pub_debug_white  = rospy.Publisher(f'/{self._vehicle_name}/debug/lane_white',   CompressedImage, queue_size=1)
        self.pub_debug_yellow = rospy.Publisher(f'/{self._vehicle_name}/debug/lane_yellow',  CompressedImage, queue_size=1)


    def cbUpdateParameters(self, parameters):
        # --- Weiße Linie (rechte Fahrbahnmarkierung) ---
        # HSV-Farbgrenzen für die Erkennung der weißen Linie
        self.hue_white_l        = parameters["white"]["hl"]["default"]
        self.hue_white_h        = parameters["white"]["hh"]["default"]
        self.saturation_white_l = parameters["white"]["sl"]["default"]
        self.saturation_white_h = parameters["white"]["sh"]["default"]
        self.lightness_white_l  = parameters["white"]["vl"]["default"]
        self.lightness_white_h  = parameters["white"]["vh"]["default"]
        
        # --- Gelbe Linie (linke, gestrichelte Mittellinie) ---
        # HSV-Farbgrenzen für die Erkennung der gelben Linie
        self.hue_yellow_l        = parameters["yellow"]["hl"]["default"]
        self.hue_yellow_h        = parameters["yellow"]["hh"]["default"]
        self.saturation_yellow_l = parameters["yellow"]["sl"]["default"]
        self.saturation_yellow_h = parameters["yellow"]["sh"]["default"]
        self.lightness_yellow_l  = parameters["yellow"]["vl"]["default"]
        self.lightness_yellow_h  = parameters["yellow"]["vh"]["default"]
        
        # --- Perspektivtransformation (Bird's-Eye-View) ---
        # Die vier Eckpunkte im Originalbild definieren das Trapez der Fahrbahn,
        # das anschließend in ein Quadrat transformiert wird (Vogelperspektive)
        self.top_left_x     = parameters["crop_image"]["top_left_x"]["default"]
        self.top_left_y     = parameters["crop_image"]["top_left_y"]["default"]
        self.top_right_x    = parameters["crop_image"]["top_right_x"]["default"]
        self.top_right_y    = parameters["crop_image"]["top_right_y"]["default"]
        self.bottom_left_x  = parameters["crop_image"]["bottom_left_x"]["default"]
        self.bottom_left_y  = parameters["crop_image"]["bottom_left_y"]["default"]
        self.bottom_right_x = parameters["crop_image"]["bottom_right_x"]["default"]
        self.bottom_right_y = parameters["crop_image"]["bottom_right_y"]["default"]
  
    def crop_img(self, img):
        img = img.copy()
        
        # Quellpunkte: Trapezform im Kamerabild (perspektivisch verzerrte Fahrbahn)
        # Reihenfolge: oben-links, oben-rechts, unten-rechts, unten-links
        pts1 = np.float32([
            [self.top_left_x,     self.top_left_y],
            [self.top_right_x,    self.top_right_y],
            [self.bottom_right_x, self.bottom_right_y],
            [self.bottom_left_x,  self.bottom_left_y],
        ])
        
        # Zielpunkte: Quadrat der Größe _crop_im_size x _crop_im_size (entzerrte Draufsicht)
        pts2 = np.float32([
            [0,                 0],
            [self._crop_im_size, 0],
            [0,                 self._crop_im_size],
            [self._crop_im_size, self._crop_im_size],
        ])

        # Perspektivtransformationsmatrix berechnen und anwenden
        M = cv2.getPerspectiveTransform(pts1, pts2)
        return cv2.warpPerspective(img, M, (self._crop_im_size, self._crop_im_size))


    def get_x_for_driving(self, mask, distance, no_lane_value, left_line):
        # Horizontalen Sobel-Kantenfilter anwenden, um vertikale Kanten (Linienränder) zu finden
        grad = cv2.Sobel(mask, cv2.CV_16S, 1, 0, ksize=3, scale=1, delta=0, borderType=cv2.BORDER_DEFAULT)

        # Binarisierung: nur starke positive Kanten behalten (heller → dunkler Übergang)
        _, th1 = cv2.threshold(grad, 127, 255, cv2.THRESH_BINARY)

        a = []
        # In einem Streifen von ±50 Pixeln um die gewünschte Messtiefe suchen
        for row in range(distance - 50, distance + 50):
            if np.where(th1[row] == 255)[0].size == 0:
                # Keine Kante in dieser Zeile gefunden → überspringen
                continue
            else:
                if left_line:
                    # Linke Linie (gelb): äußerste rechte Kante nehmen
                    # → liefert den rechten Rand der gelben Linie (Übergang gelb → Fahrbahn)
                    a.append(np.where(th1[row] == 255)[0][-1])
                else:
                    # Rechte Linie (weiß): äußerste linke Kante nehmen
                    # → liefert den linken Rand der weißen Linie (Übergang Fahrbahn → weiß)
                    a.append(np.where(th1[row] == 255)[0][0])

        if len(a) > 10:
            # Genug Messpunkte vorhanden → Median als robuster Schätzwert für die Linienposition
            return np.median(a)
        else:
            # Zu wenige Punkte → Fallback-Wert (Linie nicht erkannt)
            return no_lane_value
        

    def cbFindLane(self, image_msg):
        
        # Erste 3 Frames überspringen (Kamera noch nicht stabil initialisiert)
        if self.counter <= 3:
            self.counter += 1   
            return

        # Verhindert, dass ein neuer Frame verarbeitet wird,
        # solange noch ein vorheriger in Bearbeitung ist (kein Threading-Problem)
        if self.is_running:
            return
        
        self.is_running = True
        self.counter = 0  # Zähler zurücksetzen 

        # Komprimiertes Bild (JPEG) in ein NumPy-Array dekodieren
        np_arr = np.frombuffer(image_msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Bild in Bird's-Eye-View transformieren
        img = self.crop_img(cv_image)

        # In den HSV-Farbraum konvertieren (besser geeignet für Farbsegmentierung)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Binärmaske für gelbe Pixel erzeugen (linke gestrichelte Linie)
        mask_yellow = cv2.inRange(hsv, 
                           (self.hue_yellow_l, self.saturation_yellow_l, self.lightness_yellow_l), 
                           (self.hue_yellow_h, self.saturation_yellow_h, self.lightness_yellow_h))
        
        # Binärmaske für weiße Pixel erzeugen (rechte Linie)
        mask_white = cv2.inRange(hsv, 
                           (self.hue_white_l, self.saturation_white_l, self.lightness_white_l), 
                           (self.hue_white_h, self.saturation_white_h, self.lightness_white_h))
        
        # Fallback-Positionen, falls eine Linie nicht erkannt wird:
        # Weiße Linie → ganz rechts (95 % der Bildbreite)
        # Gelbe Linie → ganz links  (5  % der Bildbreite)
        white_alternative  = int(len(img[0]) * 0.95)
        yellow_alternative = int(len(img[0]) * 0.05)

        # X-Positionen der erkannten Linienränder auf 75 % der Bildhöhe bestimmen
        # (75 % = nah genug an der Kamera für stabile Erkennung, weit genug für Vorausschau)
        center_white  = self.get_x_for_driving(mask_white,  int(len(img) * 0.75), white_alternative,  left_line=True)
        center_yellow = self.get_x_for_driving(mask_yellow, int(len(img) * 0.75), yellow_alternative, left_line=False)

        # Plausibilitätsprüfung: Weiße Linie darf nicht links von gelber liegen
        # → bei Vertauschung wird der weniger plausible Wert durch den Fallback ersetzt
        if center_white <= center_yellow:
            if center_white > int(len(img[0]) * 0.4):
                # Weißlinie zu weit innen → gelbe Linie war wohl falsch erkannt
                center_yellow = yellow_alternative
            else:
                # Gelblinie zu weit innen → weiße Linie war wohl falsch erkannt
                center_white = white_alternative

        # Sollposition = Mittelpunkt zwischen beiden Linienrändern
        lane_center = (center_white + center_yellow) / 2

        # Fehler berechnen: normierter Versatz des Spurzentrums von der Bildmitte
        # Formel: 1 - (lane_center / (Bildbreite/2))
        # Ergebnis: 0 = Mitte, positiv = zu weit links, negativ = zu weit rechts
        msg_error = Float64()
        msg_error.data = 1 - (lane_center / len(img) * 2)

        # Spurversatz als ROS-Message veröffentlichen (wird vom PID-Regler genutzt)
        self.pub_lane.publish(msg_error)
        print(f"Lane error: {msg_error.data} range [-1,1]")


        # Zwischenergebnisse für den Debug-Loop speichern
        self.img             = img
        self.lane_center     = lane_center
        self.white_alternative  = white_alternative
        self.yellow_alternative = yellow_alternative
        self.center_white    = center_white
        self.center_yellow   = center_yellow
        self.debug_img_white  = mask_white
        self.debug_img_yellow = mask_yellow

        # --- Lokale Debug-Visualisierung (cv2.imshow) ---
        # Erkannter Spurmittelpunkt (blau)
        image = cv2.circle(img, (int(lane_center), int(len(img) / 2)), 3, (255, 0, 0))
        # Fallback-Positionen als vertikale Linien (weiß / gelb)
        image = cv2.line(image, (white_alternative,  0), (white_alternative,  self._crop_im_size), color=(255, 255, 255)) 
        image = cv2.line(image, (yellow_alternative, 0), (yellow_alternative, self._crop_im_size), color=(255, 255,   0))
        # Messbereich (±50 px um 75 % Bildhöhe) als horizontale Linien
        image = cv2.line(image, (0, int(len(img) * 0.75) + 100), (len(img[0]), int(len(img) * 0.75) + 100), color=(255, 255, 255))
        image = cv2.line(image, (0, int(len(img) * 0.75) - 100), (len(img[0]), int(len(img) * 0.75) - 100), color=(255, 255, 255))
        # Bildmitte als grüne vertikale Linie (Sollwert)
        image = cv2.line(image, (int(len(img[0]) / 2), 0), (int(len(img[0]) / 2), len(image)), (0, 255, 0))
        # Erkannte Linienpositionen als farbige Kreise
        image = cv2.circle(image, (int(center_white),  int(len(img) * 0.75)), 5, (255, 255, 255))  # weiß
        image = cv2.circle(image, (int(center_yellow), int(len(img) * 0.75)), 5, (0,   255, 255))  # cyan

        cv2.imshow('lane detection', image)
        self.is_running = False
        
        #cv2.imshow('white', mask_white)
        #cv2.imshow('yellow', mask_yellow)
        cv2.waitKey(1)
            
    def run_debug(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            # Debug-Bild mit Spurinformationen über ROS veröffentlichen (z.B. für rqt_image_view)
            if self.pub_debug_lane.get_num_connections() > 0:
                debug_img = self.img.copy()

                # Spurmittelpunkt (blau)
                debug_img = cv2.circle(debug_img, (int(self.lane_center), int(len(debug_img) / 2)), 3, (255, 0, 0))
                # Fallback-Linien
                debug_img = cv2.line(debug_img, (self.white_alternative,  0), (self.white_alternative,  1000), color=(255, 255, 255)) 
                debug_img = cv2.line(debug_img, (self.yellow_alternative, 0), (self.yellow_alternative, 1000), color=(255, 255,   0))
                # Messbereich
                debug_img = cv2.line(debug_img, (0, int(len(debug_img) * 0.75) + 100), (len(debug_img[0]), int(len(debug_img) * 0.75) + 100), color=(255, 255, 255))
                debug_img = cv2.line(debug_img, (0, int(len(debug_img) * 0.75) - 100), (len(debug_img[0]), int(len(debug_img) * 0.75) - 100), color=(255, 255, 255))
                # Bildmitte (grün)
                debug_img = cv2.line(debug_img, (int(len(debug_img[0]) / 2), 0), (int(len(debug_img[0]) / 2), len(debug_img)), (0, 255, 0))
                # Erkannte Linienpositionen
                debug_img = cv2.circle(debug_img, (int(self.center_white),  int(len(debug_img) * 0.75)), 5, (255, 255, 255))
                debug_img = cv2.circle(debug_img, (int(self.center_yellow), int(len(debug_img) * 0.75)), 5, (0,   255, 255))

                # Bild als komprimierte JPEG-Message verpacken und senden
                debug_msg = CompressedImage()
                debug_msg.header.stamp = rospy.Time.now()
                debug_msg.format = "jpeg"
                debug_msg.data = np.array(cv2.imencode('.jpg', debug_img)[1]).tobytes()
                self.pub_debug_lane.publish(debug_msg)

            # Weiße-Linie-Maske als Debug-Bild veröffentlichen
            if self.pub_debug_white.get_num_connections() > 0:
                debug_msg = CompressedImage()
                debug_msg.header.stamp = rospy.Time.now()
                debug_msg.format = "jpeg"
                debug_msg.data = np.array(cv2.imencode('.jpg', self.debug_img_white)[1]).tobytes()
                self.pub_debug_white.publish(debug_msg)

            # Gelbe-Linie-Maske als Debug-Bild veröffentlichen
            if self.pub_debug_yellow.get_num_connections() > 0:
                debug_msg = CompressedImage()
                debug_msg.header.stamp = rospy.Time.now()
                debug_msg.format = "jpeg"
                debug_msg.data = np.array(cv2.imencode('.jpg', self.debug_img_yellow)[1]).tobytes()
                self.pub_debug_yellow.publish(debug_msg)

            rate.sleep()
        
if __name__ == '__main__':
    node = DetectLaneNode('detect_lane_node')
    node.run_debug()
    rospy.spin()
