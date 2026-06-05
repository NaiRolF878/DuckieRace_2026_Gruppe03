#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────────────────────
# detect_red_lane_node.py  (Challenge 2 – Intersection Handling, Node B)
#
# Gegenspur-/Querlinie beim Abbiegen, im ORIGINALBILD. Liefert das Abbruch-
# kriterium für die Turning-Phase. Weil die Drehrichtung bekannt ist (vom Tag),
# wird gezielt NUR in der erwarteten Region gesucht – das ignoriert die anderen
# roten Linien der Kreuzung.
#
#   left     -> erwarte rote Linie RECHTS
#   right    -> erwarte rote Linie LINKS
#   straight -> fertig, wenn KEINE rote Linie mehr im Bild
#
# "erst leer, dann Wiederauftauchen": Zu Drehbeginn koennen eigene/fremde Linien
# noch in der Zielregion liegen. Darum wird turn_complete erst gemeldet, NACHDEM
# die Zielregion einmal frei war und dann wieder Rot erscheint.
#
# Subscribt:
#   /{vehicle}/enable/intersection      (Bool)   – nur dann aktiv rechnen
#   /{vehicle}/intersection/phase       (String) – "Turning" relevant
#   /{vehicle}/intersection/direction   (String) – left/right/straight
# Publiziert:
#   /{vehicle}/intersection/turn_complete (Bool)
#   /{vehicle}/debug/red_lane             (CompressedImage)
#
# UMSCHALTEN AUF ZEITGESTEUERTES ABBIEGEN: in switch_control_node die regions-
# basierte Auswertung aus- und den zeitbasierten Block einkommentieren. Diese
# Node darf weiterlaufen, ihr Signal wird dann ignoriert.
# ─────────────────────────────────────────────────────────────────────────────

import os
import rospy
import numpy as np
import cv2
from std_msgs.msg import Bool, String
from sensor_msgs.msg import CompressedImage
import util


class DetectRedLaneNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        self._vehicle_name = os.environ['VEHICLE_NAME']

        # ── self.*-Defaults VOR init_parameters ───────────────────────────────
        self.is_running   = False
        self.counter      = 0
        self.debug_img    = np.zeros((100, 100, 3), dtype=np.uint8)
        self.enabled      = False
        self.phase        = "Lane"
        self.direction    = "straight"
        self._region_was_empty = False
        self._prev_phase  = "Lane"
        # Defaults (aus JSON überschrieben)
        self.region_threshold = 800
        self.detection_zone   = 0.0
        self.region_split_lo  = 0.35
        self.region_split_hi  = 0.65

        util.init_parameters(node_name, self.cbUpdateParameters)

        # ── Subscriber ────────────────────────────────────────────────────────
        self.sub_enable = rospy.Subscriber(
            f'/{self._vehicle_name}/enable/intersection', Bool, self.cbEnable, queue_size=1)
        self.sub_phase = rospy.Subscriber(
            f'/{self._vehicle_name}/intersection/phase', String, self.cbPhase, queue_size=1)
        self.sub_direction = rospy.Subscriber(
            f'/{self._vehicle_name}/intersection/direction', String, self.cbDirection, queue_size=1)
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self.sub_image = rospy.Subscriber(
            self._camera_topic, CompressedImage, self.cbImage, queue_size=1)

        # ── Publisher ─────────────────────────────────────────────────────────
        self.pub_turn_complete = rospy.Publisher(
            f'/{self._vehicle_name}/intersection/turn_complete', Bool, queue_size=1)
        self.pub_debug = rospy.Publisher(
            f'/{self._vehicle_name}/debug/red_lane', CompressedImage, queue_size=1)

        rospy.loginfo(f"[{node_name}] Bereit.")

    # ── Parameter ─────────────────────────────────────────────────────────────

    def cbUpdateParameters(self, parameters):
        r = parameters["red"]
        self.hue_red_l        = r["hl"]["default"]
        self.hue_red_h        = r["hh"]["default"]
        self.hue_red_l2       = r["hl2"]["default"]
        self.hue_red_h2       = r["hh2"]["default"]
        self.saturation_red_l = r["sl"]["default"]
        self.saturation_red_h = r["sh"]["default"]
        self.lightness_red_l  = r["vl"]["default"]
        self.lightness_red_h  = r["vh"]["default"]

        g = parameters["region"]
        self.region_threshold = g["threshold"]["default"]
        self.detection_zone   = g["detection_zone"]["default"]
        self.region_split_lo  = g["split_lo"]["default"]
        self.region_split_hi  = g["split_hi"]["default"]

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def cbEnable(self, msg):
        self.enabled = msg.data

    def cbPhase(self, msg):
        # Beim Eintritt in Turning das "erst leer"-Flag zuruecksetzen
        if msg.data == "Turning" and self._prev_phase != "Turning":
            self._region_was_empty = False
            rospy.loginfo("[red_lane] Turning gestartet – warte auf Gegenspur-Linie")
        self._prev_phase = msg.data
        self.phase = msg.data

    def cbDirection(self, msg):
        self.direction = msg.data

    # ── Rote-Pixel / Flächen ───────────────────────────────────────────────────

    def _red_mask(self, img_bgr):
        hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        m1 = cv2.inRange(hsv,
                         (self.hue_red_l,  self.saturation_red_l, self.lightness_red_l),
                         (self.hue_red_h,  self.saturation_red_h, self.lightness_red_h))
        m2 = cv2.inRange(hsv,
                         (self.hue_red_l2, self.saturation_red_l, self.lightness_red_l),
                         (self.hue_red_h2, self.saturation_red_h, self.lightness_red_h))
        mask = cv2.bitwise_or(m1, m2)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        return cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    @staticmethod
    def _largest_blob_area(region_mask):
        # Groesste zusammenhaengende rote Flaeche (robust gegen mehrere Linien)
        if cv2.countNonZero(region_mask) == 0:
            return 0
        n, _, stats, _ = cv2.connectedComponentsWithStats(region_mask, connectivity=8)
        if n <= 1:
            return 0
        return int(stats[1:, cv2.CC_STAT_AREA].max())

    # ── Haupt-Callback ────────────────────────────────────────────────────────

    def cbImage(self, image_msg):
        if self.counter <= 3:
            self.counter += 1
            return
        if self.is_running:
            return
        self.is_running = True
        try:
            np_arr   = np.frombuffer(image_msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            h, w = cv_image.shape[:2]

            mask = self._red_mask(cv_image)
            y0   = int(h * self.detection_zone)
            sub  = mask[y0:, :]
            x_lo = int(w * self.region_split_lo)
            x_hi = int(w * self.region_split_hi)
            left_area  = self._largest_blob_area(sub[:, :x_lo])
            right_area = self._largest_blob_area(sub[:, x_hi:])
            any_area   = self._largest_blob_area(sub)

            turn_complete = False
            # Nur im Turning sinnvoll; Signal wird ohnehin nur dann genutzt
            if self.enabled and self.phase == "Turning":
                if self.direction == "left":
                    present = right_area >= self.region_threshold
                    if not present and not self._region_was_empty:
                        self._region_was_empty = True
                    turn_complete = self._region_was_empty and present
                elif self.direction == "right":
                    present = left_area >= self.region_threshold
                    if not present and not self._region_was_empty:
                        self._region_was_empty = True
                    turn_complete = self._region_was_empty and present
                else:  # straight: fertig wenn kein Rot mehr
                    turn_complete = any_area < self.region_threshold

            self.pub_turn_complete.publish(Bool(data=turn_complete))

            # ── Debug ─────────────────────────────────────────────────────────
            debug_img = cv_image.copy()
            cv2.line(debug_img, (x_lo, 0), (x_lo, h), (255, 255, 0), 1)
            cv2.line(debug_img, (x_hi, 0), (x_hi, h), (255, 255, 0), 1)
            if y0 > 0:
                cv2.line(debug_img, (0, y0), (w, y0), (0, 0, 255), 1)
            cv2.rectangle(debug_img, (0, 0), (440, 110), (0, 0, 0), -1)
            cv2.putText(debug_img, f"Phase: {self.phase}  Dir: {self.direction}",
                        (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(debug_img, f"links: {left_area}  rechts: {right_area}  any: {any_area}",
                        (10, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)
            cv2.putText(debug_img, f"leer gesehen: {self._region_was_empty}",
                        (10, 76), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            if turn_complete:
                cv2.putText(debug_img, "TURN COMPLETE", (10, 100),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            self.debug_img = debug_img

            # Lokale Debug-Ansicht – bei Bedarf einkommentieren:
            # cv2.imshow("Red Lane (Turn)", debug_img)
            # cv2.waitKey(1)
        except Exception as e:
            rospy.logerr(f"[detect_red_lane] Fehler: {e}")
        finally:
            self.is_running = False

    def _publish_compressed(self, publisher, img):
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data   = np.array(cv2.imencode('.jpg', img)[1]).tobytes()
        publisher.publish(msg)

    def run_debug(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.counter > 3 and self.pub_debug.get_num_connections() > 0:
                self._publish_compressed(self.pub_debug, self.debug_img)
            rate.sleep()


if __name__ == '__main__':
    node = DetectRedLaneNode('detect_red_lane_node')
    node.run_debug()
    rospy.spin()
