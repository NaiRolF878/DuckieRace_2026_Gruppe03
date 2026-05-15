#!/usr/bin/env python3

# ─────────────────────────────────────────────────────────────────────────────
# detect_apriltag_node.py
#
# Aufgabe: Kamerabild auf AprilTags prüfen und erkannte Tag-ID publizieren.
#          Wird an Kreuzungen verwendet um die erlaubten Abbiegerichtungen
#          aus der JSON-Konfiguration zu lesen.
#
# Publiziert:
#   /detect/apriltag  (Int32) → Tag-ID des erkannten Schildes (-1 = kein Tag)
# ─────────────────────────────────────────────────────────────────────────────

import os
import rospy
import numpy as np
import cv2
from std_msgs.msg import Int32
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32
from pupil_apriltags import Detector
import util


class DetectApriltagNode:
    def __init__(self, node_name):
        # ROS-Node initialisieren
        rospy.init_node(node_name)

        # Fahrzeugnamen aus Umgebungsvariable lesen
        self._vehicle_name = os.environ['VEHICLE_NAME']

        # Parameter aus JSON laden und Live-Update registrieren
        util.init_parameters(node_name, self.cbUpdateParameters)

        # AprilTag-Detektor initialisieren
        # families='tag36h11' ist die Duckietown-Standard Tag-Familie
        self._detector = Detector(families='tag36h11')

        # Subscriber: empfängt Kamerabilder
        self.sub_image = rospy.Subscriber(
            f'/{self._vehicle_name}/camera_node/image/compressed',
            CompressedImage,
            self.cbDetectTag,
            queue_size=1
        )

        # Publisher: sendet erkannte Tag-ID (-1 = kein Tag sichtbar)
        self.pub_apriltag = rospy.Publisher(
            f'/{self._vehicle_name}/detect/apriltag',
            Int32,
            queue_size=1
        )

        # Sperrvariable: verhindert parallele Verarbeitung
        self.is_running = False

        rospy.loginfo(f"[{node_name}] AprilTag-Erkennung bereit.")


    def cbUpdateParameters(self, parameters):
        # Minimale Fläche des Tags im Bild in Pixel
        # → kleine/weit entfernte Tags werden ignoriert
        self.min_tag_area = parameters["detection"]["min_tag_area"]["default"]

        # Kamera-Intrinsics für genauere Positionsschätzung
        # → werden für det.pose_t (Abstand) benötigt, optional
        self.camera_fx = parameters["detection"]["camera_fx"]["default"]
        self.camera_fy = parameters["detection"]["camera_fy"]["default"]
        self.camera_cx = parameters["detection"]["camera_cx"]["default"]
        self.camera_cy = parameters["detection"]["camera_cy"]["default"]


    def cbDetectTag(self, image_msg):
        # Sperrvariable prüfen
        if self.is_running:
            return
        self.is_running = True

        # Bild dekodieren
        np_arr = np.frombuffer(image_msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # In Graustufen konvertieren → AprilTag-Detektor benötigt Graustufenbild
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # AprilTags im Bild suchen
        detections = self._detector.detect(gray)

        tag_id = -1  # Standardwert: kein Tag erkannt

        if detections:
            # Wenn mehrere Tags sichtbar → den größten nehmen
            # (größter Tag = nächster Tag = relevantester)
            best = None
            best_area = 0

            for det in detections:
                # Fläche des Tags aus den vier Eckpunkten berechnen
                corners = det.corners.astype(np.int32)
                area = cv2.contourArea(corners)

                if area > best_area and area > self.min_tag_area:
                    best_area = area
                    best = det

            if best is not None:
                tag_id = best.tag_id
                print(f"AprilTag erkannt: ID={tag_id}, Fläche={best_area:.0f}px²")

                # Bounding-Box um erkannten Tag zeichnen (für Debug)
                if self.pub_debug_apriltag.get_num_connections() > 0:
                    debug_img = cv_image.copy()
                    corners   = best.corners.astype(int)
                    # Viereck um Tag zeichnen
                    cv2.polylines(debug_img, [corners], isClosed=True, color=(0, 255, 0), thickness=2)
                    # Tag-ID als Text über dem Tag
                    top_left = corners[0]
                    cv2.putText(debug_img, f"ID: {tag_id}",
                        (top_left[0], top_left[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2
                    )
                    # Bounding-Box als Rechteck
                    x, y, w, h = cv2.boundingRect(corners)
                    cv2.rectangle(debug_img, (x, y), (x + w, y + h), (0, 200, 255), 2)
                    debug_msg              = CompressedImage()
                    debug_msg.header.stamp = rospy.Time.now()
                    debug_msg.format       = "jpeg"
                    debug_msg.data         = np.array(cv2.imencode('.jpg', debug_img)[1]).tobytes()
                    self.pub_debug_apriltag.publish(debug_msg)

        # ── Debug-Bild: Bounding-Box um erkannte Tags ────────────────────────
        if self.pub_debug.get_num_connections() > 0:
            debug_img = cv_image.copy()
            if detections:
                for det in detections:
                    corners = det.corners.astype(int)
                    # Bounding-Box um den Tag zeichnen
                    x_min = corners[:, 0].min()
                    x_max = corners[:, 0].max()
                    y_min = corners[:, 1].min()
                    y_max = corners[:, 1].max()
                    # Grün wenn erkannt und groß genug, gelb sonst
                    color = (0, 255, 0) if (best is not None and det.tag_id == best.tag_id) else (0, 255, 255)
                    cv2.rectangle(debug_img, (x_min, y_min), (x_max, y_max), color, 2)
                    # Tag-ID als Label
                    cv2.putText(debug_img, f"ID: {det.tag_id}",
                                (x_min, y_min - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                    # Mittelpunkt markieren
                    cx, cy = int(det.center[0]), int(det.center[1])
                    cv2.circle(debug_img, (cx, cy), 5, color, -1)

            debug_msg              = CompressedImage()
            debug_msg.header.stamp = rospy.Time.now()
            debug_msg.format       = "jpeg"
            debug_msg.data         = np.array(cv2.imencode('.jpg', debug_img)[1]).tobytes()
            self.pub_debug.publish(debug_msg)

        # Tag-ID publizieren (-1 wenn kein Tag erkannt)
        self.pub_apriltag.publish(Int32(data=tag_id))

        self.is_running = False


    def run(self):
        rospy.spin()


if __name__ == '__main__':
    node = DetectApriltagNode('detect_apriltag_node')
    node.run()
