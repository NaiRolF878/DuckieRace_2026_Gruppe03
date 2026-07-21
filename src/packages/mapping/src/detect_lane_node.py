#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────────────────────
# detect_lane_node.py  (Challenge 4 – Mapping & Path Finding)
#
# Spurerkennung + rote Haltelinie in EINER Node (ein Bild-Decode + eine
# Perspektivtransformation pro Frame statt zwei separate Nodes - spart Zeit
# auf der begrenzten Rechenleistung des Bots).
#
# Spur: Sobel-Kantenerkennung auf einer festen Erkennungszeile
# (detection_row_factor). Gelb wird zuerst gesucht (Referenzpunkt), die weiße
# Maske wird dann links von "gelb + min_lane_width" ausgeblendet (Korridor-
# Filter) - verhindert, dass Weiß auf der Gegenspur/am Wendeplatz greift.
# Wird Weiß auf der Erkennungszeile nicht gefunden, wandert die Zeile
# schrittweise nach unten (naeher am Bot), bevor auf den Bildrand-Alternativwert
# zurueckgefallen wird.
#
# Haltelinie: Rot-Pixel-Schwellwert in einer ROI (unteres Bilddrittel).
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
        rospy.init_node(node_name)
        self._vehicle_name = os.environ['VEHICLE_NAME']

        self._crop_im_size = 400
        self.is_running = False
        self.counter = 0

        # Platzhalter fuer Debug-Variablen, bevor der erste Frame verarbeitet ist
        self.img = np.zeros((self._crop_im_size, self._crop_im_size, 3), dtype=np.uint8)
        self.lane_center = self._crop_im_size / 2
        self.white_alternative = int(self._crop_im_size * 0.95)
        self.yellow_alternative = int(self._crop_im_size * 0.05)
        self.center_white = self.white_alternative
        self.center_yellow = self.yellow_alternative
        self.debug_img_white = np.zeros((self._crop_im_size, self._crop_im_size), dtype=np.uint8)
        self.debug_img_yellow = np.zeros((self._crop_im_size, self._crop_im_size), dtype=np.uint8)
        self.debug_img_red = np.zeros((self._crop_im_size, self._crop_im_size, 3), dtype=np.uint8)
        self.used_detection_row = int(self._crop_im_size * 0.75)
        self.used_detection_row_white = self.used_detection_row
        self.used_detection_row_yellow = self.used_detection_row

        util.init_parameters(node_name, self.cbUpdateParameters)

        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self.sub_image_original = rospy.Subscriber(
            self._camera_topic, CompressedImage, self.cbFindLane, queue_size=1)

        self.pub_lane = rospy.Publisher(
            f'/{self._vehicle_name}/detect/lane', Float64, queue_size=1)
        self.pub_stop_line = rospy.Publisher(
            f'/{self._vehicle_name}/detect/stop_line', Bool, queue_size=1)
        self.pub_debug_lane = rospy.Publisher(
            f'/{self._vehicle_name}/debug/lane_croped', CompressedImage, queue_size=1)
        self.pub_debug_white = rospy.Publisher(
            f'/{self._vehicle_name}/debug/lane_white', CompressedImage, queue_size=1)
        self.pub_debug_yellow = rospy.Publisher(
            f'/{self._vehicle_name}/debug/lane_yellow', CompressedImage, queue_size=1)
        self.pub_debug_red = rospy.Publisher(
            f'/{self._vehicle_name}/debug/lane_red', CompressedImage, queue_size=1)

        rospy.loginfo(f"[{node_name}] Bereit.")

    # ── Parameter ─────────────────────────────────────────────────────────────

    def cbUpdateParameters(self, parameters):
        self.detection_row_factor = parameters["detection_row_factor"]["default"]
        self.min_lane_width = parameters["min_lane_width"]["default"]

        self.hue_white_l = parameters["white"]["hl"]["default"]
        self.hue_white_h = parameters["white"]["hh"]["default"]
        self.saturation_white_l = parameters["white"]["sl"]["default"]
        self.saturation_white_h = parameters["white"]["sh"]["default"]
        self.lightness_white_l = parameters["white"]["vl"]["default"]
        self.lightness_white_h = parameters["white"]["vh"]["default"]

        self.hue_yellow_l = parameters["yellow"]["hl"]["default"]
        self.hue_yellow_h = parameters["yellow"]["hh"]["default"]
        self.saturation_yellow_l = parameters["yellow"]["sl"]["default"]
        self.saturation_yellow_h = parameters["yellow"]["sh"]["default"]
        self.lightness_yellow_l = parameters["yellow"]["vl"]["default"]
        self.lightness_yellow_h = parameters["yellow"]["vh"]["default"]

        self.top_left_x = parameters["crop_image"]["top_left_x"]["default"]
        self.top_left_y = parameters["crop_image"]["top_left_y"]["default"]
        self.top_right_x = parameters["crop_image"]["top_right_x"]["default"]
        self.top_right_y = parameters["crop_image"]["top_right_y"]["default"]
        self.bottom_left_x = parameters["crop_image"]["bottom_left_x"]["default"]
        self.bottom_left_y = parameters["crop_image"]["bottom_left_y"]["default"]
        self.bottom_right_x = parameters["crop_image"]["bottom_right_x"]["default"]
        self.bottom_right_y = parameters["crop_image"]["bottom_right_y"]["default"]

        # Rot liegt an zwei Stellen des Hue-Kreises (red1 = 0-10, red2 = 170-180)
        self.hue_red1_l = parameters["red1"]["hl"]["default"]
        self.hue_red1_h = parameters["red1"]["hh"]["default"]
        self.saturation_red1_l = parameters["red1"]["sl"]["default"]
        self.saturation_red1_h = parameters["red1"]["sh"]["default"]
        self.lightness_red1_l = parameters["red1"]["vl"]["default"]
        self.lightness_red1_h = parameters["red1"]["vh"]["default"]

        self.hue_red2_l = parameters["red2"]["hl"]["default"]
        self.hue_red2_h = parameters["red2"]["hh"]["default"]
        self.saturation_red2_l = parameters["red2"]["sl"]["default"]
        self.saturation_red2_h = parameters["red2"]["sh"]["default"]
        self.lightness_red2_l = parameters["red2"]["vl"]["default"]
        self.lightness_red2_h = parameters["red2"]["vh"]["default"]

        self.thresh_red_pixels = parameters["detection"]["thresh"]["default"]
        # Erkennungszone der Haltelinie (Anteil der Bild-Hoehe/-Breite) - ueber
        # die JSON einstellbar statt fest im Code, damit sie sich ohne
        # Code-Aenderung an die Kamera-Montage/Strecke anpassen laesst.
        self.red_detection_zone    = parameters["detection"]["zone"]["default"]
        self.red_detection_x_start = parameters["detection"]["x_start"]["default"]
        self.red_detection_x_end   = parameters["detection"]["x_end"]["default"]

    # ── Bildvorverarbeitung ──────────────────────────────────────────────────

    def crop_img(self, img):
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

    def get_x_for_driving(self, mask, distance, no_lane_value, left_line):
        # Sobel-Kante in x-Richtung auf einem Zeilenfenster um "distance".
        # left_line=True (Gelb): rechteste Kante der Zeile (rechter Rand der
        # gestrichelten Mittellinie). left_line=False (Weiss): linkeste Kante
        # (linker Rand der weissen Aussenlinie, naeher an der Spurmitte).
        grad = cv2.Sobel(mask, cv2.CV_16S, 1, 0, ksize=3, scale=1, delta=0,
                         borderType=cv2.BORDER_DEFAULT)
        _, th1 = cv2.threshold(grad, 127, 255, cv2.THRESH_BINARY)

        a = []
        for row in range(max(0, distance - 50), min(len(mask), distance + 50)):
            candidates = np.where(th1[row] == 255)[0]
            if candidates.size == 0:
                continue
            a.append(candidates[-1] if left_line else candidates[0])

        if len(a) > 10:
            return np.median(a)
        return no_lane_value

    def fnGetRedMask(self, hsv_roi):
        mask_red1 = cv2.inRange(hsv_roi,
            (self.hue_red1_l, self.saturation_red1_l, self.lightness_red1_l),
            (self.hue_red1_h, self.saturation_red1_h, self.lightness_red1_h))
        mask_red2 = cv2.inRange(hsv_roi,
            (self.hue_red2_l, self.saturation_red2_l, self.lightness_red2_l),
            (self.hue_red2_h, self.saturation_red2_h, self.lightness_red2_h))
        return cv2.bitwise_or(mask_red1, mask_red2)

    def detect_stop_line(self, img, hsv):
        # ROI ueber JSON konfigurierbar: detection_zone schneidet oben ab (nur
        # unterer Bildteil), x_start/x_end schneiden links/rechts ab (eigene
        # Spur, Gegenspur ausschliessen).
        height, width = img.shape[:2]
        row_start = int(height * self.red_detection_zone)
        col_start = int(width * self.red_detection_x_start)
        col_end   = int(width * self.red_detection_x_end)
        mask_red = self.fnGetRedMask(hsv[row_start:, col_start:col_end])
        num_red_pixels = cv2.countNonZero(mask_red)

        debug_img = img[row_start:, col_start:col_end].copy()
        debug_img[mask_red > 0] = (0, 0, 255)
        cv2.putText(debug_img, f"Red pixels: {num_red_pixels}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        self.debug_img_red = debug_img

        return num_red_pixels > self.thresh_red_pixels

    # ── Haupt-Callback ───────────────────────────────────────────────────────

    def cbFindLane(self, image_msg):
        detection_row_factor = self.detection_row_factor

        if self.counter <= 3:
            self.counter += 1
            return
        if self.is_running:
            return
        self.is_running = True
        try:
            np_arr = np.frombuffer(image_msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_image is None:
                rospy.logwarn_throttle(5.0, "Frame nicht dekodierbar - uebersprungen.")
                return

            img = self.crop_img(cv_image)
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            mask_yellow = cv2.inRange(hsv,
                (self.hue_yellow_l, self.saturation_yellow_l, self.lightness_yellow_l),
                (self.hue_yellow_h, self.saturation_yellow_h, self.lightness_yellow_h))
            mask_white = cv2.inRange(hsv,
                (self.hue_white_l, self.saturation_white_l, self.lightness_white_l),
                (self.hue_white_h, self.saturation_white_h, self.lightness_white_h))

            white_alternative = int(len(img[0]) * 0.95)
            yellow_alternative = int(len(img[0]) * 0.05)

            # Gelb fix am konfigurierten detection_row_factor
            yellow_row_factor = self.detection_row_factor
            center_yellow = self.get_x_for_driving(
                mask_yellow, int(len(img) * yellow_row_factor), yellow_alternative, left_line=True)

            # Weisse Maske: alles links von gelb + min_lane_width ausblenden
            mask_white_filtered = mask_white.copy()
            corridor_start = max(0, int(center_yellow) + self.min_lane_width)
            mask_white_filtered[:, :corridor_start] = 0

            # Weiss sucht weiter unten, falls auf der Standardzeile nicht gefunden
            center_white = self.get_x_for_driving(
                mask_white_filtered, int(len(img) * detection_row_factor),
                white_alternative, left_line=False)
            while center_white == white_alternative and detection_row_factor <= 0.95:
                detection_row_factor += 0.05
                center_white = self.get_x_for_driving(
                    mask_white_filtered, int(len(img) * detection_row_factor),
                    white_alternative, left_line=False)

            self.used_detection_row_white = int(len(img) * detection_row_factor)
            self.used_detection_row_yellow = int(len(img) * yellow_row_factor)
            self.used_detection_row = self.used_detection_row_white

            # Plausibilitaetspruefung: weiss muss rechts von gelb liegen
            if center_white <= center_yellow:
                if center_white > int(len(img[0]) * 0.4):
                    center_yellow = yellow_alternative
                else:
                    center_white = white_alternative

            lane_center = (center_white + center_yellow) / 2
            msg_error = Float64(data=1 - (lane_center / len(img) * 2))
            self.pub_lane.publish(msg_error)
            rospy.loginfo_throttle(1.0, f"Lane error: {msg_error.data:.3f} range [-1,1]")

            # Haltelinie - nutzt dasselbe BEV-Bild/HSV wie die Spurerkennung
            # oben (kein zweiter Decode/Warp fuer eine separate Node).
            stop_line_detected = self.detect_stop_line(img, hsv)
            self.pub_stop_line.publish(Bool(data=stop_line_detected))

            self.img = img
            self.lane_center = lane_center
            self.white_alternative = white_alternative
            self.yellow_alternative = yellow_alternative
            self.center_white = center_white
            self.center_yellow = center_yellow
            self.debug_img_white = mask_white
            self.debug_img_yellow = mask_yellow
        except Exception as e:
            rospy.logerr(f"cbFindLane failed: {e}")
        finally:
            self.is_running = False

    # ── Debug-Schleife ───────────────────────────────────────────────────────

    def _publish_compressed(self, publisher, img):
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', img)[1]).tobytes()
        publisher.publish(msg)

    def run_debug(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.pub_debug_lane.get_num_connections() > 0:
                debug_img = self.img.copy()
                debug_img = cv2.circle(
                    debug_img, (int(self.lane_center), int(len(debug_img) / 2)), 3, (255, 0, 0))
                debug_img = cv2.line(debug_img, (self.white_alternative, 0),
                                      (self.white_alternative, self._crop_im_size), color=(255, 255, 255))
                debug_img = cv2.line(debug_img, (self.yellow_alternative, 0),
                                      (self.yellow_alternative, self._crop_im_size), color=(255, 255, 0))
                debug_img = cv2.line(debug_img, (0, self.used_detection_row + 50),
                                      (len(debug_img[0]), self.used_detection_row + 50), color=(255, 255, 255))
                debug_img = cv2.line(debug_img, (0, self.used_detection_row - 50),
                                      (len(debug_img[0]), self.used_detection_row - 50), color=(255, 255, 255))
                debug_img = cv2.line(debug_img, (int(len(debug_img[0]) / 2), 0),
                                      (int(len(debug_img[0]) / 2), len(debug_img)), (0, 255, 0))
                debug_img = cv2.circle(debug_img, (int(self.center_white), self.used_detection_row_white), 5, (255, 255, 255))
                debug_img = cv2.circle(debug_img, (int(self.center_yellow), self.used_detection_row_yellow), 5, (0, 255, 255))
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
