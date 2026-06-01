#!/usr/bin/env python3
"""
DetectApriltagNode – AprilTag-Erkennung + rote Haltelinie auf Original-Kamerabild.

Publiziert:
  /{vehicle}/detect/intersection        (Bool)   – rote Linie UND Tag gleichzeitig sichtbar
  /{vehicle}/detect/apriltag/direction  (String) – erlaubte Richtungen, kommagetrennt
                                                   z.B. "left,straight" oder "unknown"
  /{vehicle}/detect/apriltag/id         (Int32)  – erkannte Tag-ID (-1 = keine)
  /{vehicle}/detect/red_line_side       (String) – Position der roten Linie im Bild:
                                                   "none" | "left" | "center" | "right"
  /{vehicle}/debug/apriltag             (CompressedImage)
  /{vehicle}/debug/apriltag_red         (CompressedImage)

Hinweis zur Tag-Richtungs-Zuordnung:
  Wird aus detect_apriltag_node.json geladen (Schlüssel "tag_directions"):
    { "56": ["left", "straight", "right"], "58": ["right"] ... }
  Die Werte dürfen beliebig viele der Strings "left", "straight", "right" enthalten.
  switch_control_node wählt zufällig eine erlaubte Richtung.
"""

import json
import os
import rospy
import numpy as np
import cv2
from std_msgs.msg import String, Bool, Int32
from sensor_msgs.msg import CompressedImage
import util

try:
    from pupil_apriltags import Detector
    APRILTAG_AVAILABLE = True
except ImportError:
    rospy.logwarn("[detect_apriltag] pupil_apriltags nicht installiert! "
                  "Tag-Erkennung deaktiviert.")
    APRILTAG_AVAILABLE = False


class DetectApriltagNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        self._node_name    = node_name
        self._vehicle_name = os.environ['VEHICLE_NAME']

        # Parameter laden (rote Linie HSV-Werte)
        util.init_parameters(node_name, self.cbUpdateParameters)

        # Tag-Richtungs-Mapping separat aus JSON lesen
        self._load_tag_directions()

        # AprilTag-Detektor (tag36h11 = Duckietown-Standard)
        if APRILTAG_AVAILABLE:
            self.detector = Detector(
                families='tag36h11',
                nthreads=1,
                quad_decimate=2.0,   # Downsampling für Geschwindigkeit
                quad_sigma=0.0,
                refine_edges=1,
                decode_sharpening=0.25,
            )

        # ── Subscriber ────────────────────────────────────────────────────────
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        rospy.Subscriber(self._camera_topic, CompressedImage,
                         self.cbImage, queue_size=1)

        # ── Publisher ─────────────────────────────────────────────────────────
        self.pub_intersection = rospy.Publisher(
            f'/{self._vehicle_name}/detect/intersection', Bool, queue_size=1)
        self.pub_direction = rospy.Publisher(
            f'/{self._vehicle_name}/detect/apriltag/direction', String, queue_size=1)
        self.pub_tag_id = rospy.Publisher(
            f'/{self._vehicle_name}/detect/apriltag/id', Int32, queue_size=1)
        self.pub_red_side = rospy.Publisher(
            f'/{self._vehicle_name}/detect/red_line_side', String, queue_size=1)

        # Debug
        self.pub_debug     = rospy.Publisher(
            f'/{self._vehicle_name}/debug/apriltag', CompressedImage, queue_size=1)
        self.pub_debug_red = rospy.Publisher(
            f'/{self._vehicle_name}/debug/apriltag_red', CompressedImage, queue_size=1)

        # ── Interne Zustandsvariablen ─────────────────────────────────────────
        self.is_running    = False
        self.debug_img     = None
        self.debug_red_img = None

        # Stabilitätsfilter: Tag-ID muss N Frames in Folge gleich sein
        # bevor sie als gültig gilt.
        # _stable_id    = aktuell bestätigte ID  (-1 = keine)
        # _candidate_id = ID die gerade gezählt wird
        # _candidate_count = wie viele Frames in Folge schon dieselbe ID
        self._stable_id        = -1
        self._candidate_id     = -1
        self._candidate_count  = 0
        self.stability_required = 3   # wird durch cbUpdateParameters überschrieben

        rospy.loginfo(f"[{node_name}] Bereit. Tag-Mapping: {self.tag_directions}")

    # ── Konfiguration ─────────────────────────────────────────────────────────

    def _load_tag_directions(self):
        """Liest das Tag→Richtungs-Mapping aus der JSON-Konfigurationsdatei."""
        path = os.path.join(os.path.dirname(__file__),
                            f"../config/{self._node_name}.json")
        with open(path, 'r') as f:
            config = json.load(f)
        # Schlüssel sind Strings im JSON → in int konvertieren
        raw = config.get("tag_directions", {})
        self.tag_directions = {int(k): v for k, v in raw.items()}

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
        self.red_pixel_threshold = r["pixel_threshold"]["default"]
        self.red_detection_zone  = r["detection_zone"]["default"]

        t = parameters["tag_filter"]
        self.stability_required = int(t["stability_frames"]["default"])
        self.pos_x_min          = t["pos_x_min"]["default"]  # rel. Bildbreite [0..1]
        self.pos_x_max          = t["pos_x_max"]["default"]
        self.pos_y_max          = t["pos_y_max"]["default"]  # rel. Bildhöhe   [0..1]
        self.min_tag_area       = t["min_area"]["default"]   # px²

    # ── Rote-Linie-Erkennung ──────────────────────────────────────────────────

    def _detect_red_line(self, img_bgr):
        """
        Erkennt rote Linie im Original-Kamerabild.

        Rückgabe:
          detected (bool)  – True wenn genug rote Pixel im unteren ROI
          side     (str)   – "none" | "left" | "center" | "right"
                             (basierend auf Schwerpunkt aller roten Pixel)
          mask     (ndarray) – binäre Maske für Debug-Bild
        """
        hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        h, w = img_bgr.shape[:2]

        # Rot liegt im HSV-Kreis an zwei Stellen → zwei Masken kombinieren
        mask_lower = cv2.inRange(
            hsv,
            (self.hue_red_l,  self.saturation_red_l, self.lightness_red_l),
            (self.hue_red_h,  self.saturation_red_h, self.lightness_red_h))
        mask_upper = cv2.inRange(
            hsv,
            (self.hue_red_l2, self.saturation_red_l, self.lightness_red_l),
            (self.hue_red_h2, self.saturation_red_h, self.lightness_red_h))
        mask_red = cv2.bitwise_or(mask_lower, mask_upper)

        # Morphologie: kleine Rausch-Pixel entfernen
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)

        # ROI für Haltelinien-Erkennung: unterster Bereich des Bildes
        roi_start = int(h * self.red_detection_zone)
        roi = mask_red[roi_start:, :]
        red_count = cv2.countNonZero(roi)
        detected  = red_count > self.red_pixel_threshold

        # Seite bestimmen: Schwerpunkt aller roten Pixel (gesamtes Bild)
        side = "none"
        moments = cv2.moments(mask_red)
        if moments["m00"] > 50:        # mindestens 50 rote Pixel insgesamt
            cx = int(moments["m10"] / moments["m00"])
            if cx < w // 3:
                side = "left"
            elif cx > 2 * w // 3:
                side = "right"
            else:
                side = "center"

        return detected, side, mask_red

    # ── AprilTag-Erkennung ────────────────────────────────────────────────────

    @staticmethod
    def _tag_area(tag):
        """Shoelace-Formel: Fläche des erkannten Tag-Vierecks in px²."""
        c = tag.corners
        return 0.5 * abs(
            (c[0][0] * c[1][1] - c[1][0] * c[0][1]) +
            (c[1][0] * c[2][1] - c[2][0] * c[1][1]) +
            (c[2][0] * c[3][1] - c[3][0] * c[2][1]) +
            (c[3][0] * c[0][1] - c[0][0] * c[3][1])
        )

    def _is_valid_intersection_tag(self, tag, img_h, img_w):
        """
        Drei-Ebenen-Filter:
          1. Whitelist  – nur IDs die im tag_directions-Mapping stehen
          2. Position   – Tag-Mittelpunkt muss in der erwarteten Bildregion liegen
          3. Mindestgröße – zu kleine Tags (weit weg oder Rauschen) ignorieren
        """
        # Ebene 1: Whitelist
        if tag.tag_id not in self.tag_directions:
            return False, "not in whitelist"

        # Ebene 2: Position
        cx_rel = tag.center[0] / img_w
        cy_rel = tag.center[1] / img_h
        if not (self.pos_x_min <= cx_rel <= self.pos_x_max):
            return False, f"x={cx_rel:.2f} outside [{self.pos_x_min},{self.pos_x_max}]"
        if cy_rel > self.pos_y_max:
            return False, f"y={cy_rel:.2f} > {self.pos_y_max} (zu weit unten)"

        # Ebene 3: Mindestgröße
        area = self._tag_area(tag)
        if area < self.min_tag_area:
            return False, f"area={area:.0f} < {self.min_tag_area}"

        return True, "ok"

    def _update_stability(self, candidate_id):
        """
        Stabilitätsfilter: candidate_id muss stability_required Frames
        in Folge auftreten, bevor _stable_id aktualisiert wird.
        Bei -1 (kein Tag sichtbar) wird der Kandidat-Zähler zurückgesetzt,
        aber die stabile ID bleibt noch kurz erhalten (wird erst nach einem
        vollständigen neuen Durchlauf überschrieben).
        """
        if candidate_id == -1:
            # Kein gültiger Tag → Zähler zurücksetzen, stabile ID behalten
            self._candidate_id    = -1
            self._candidate_count = 0
            # Stabile ID erst löschen wenn sie konsistent fehlt
            # (robuster als sofortiges Löschen bei einem einzigen verpassten Frame)
            return self._stable_id

        if candidate_id == self._candidate_id:
            self._candidate_count += 1
        else:
            # Neue ID → Zähler neu starten
            self._candidate_id    = candidate_id
            self._candidate_count = 1

        if self._candidate_count >= self.stability_required:
            self._stable_id = self._candidate_id

        return self._stable_id

    def _detect_apriltags(self, img_bgr):
        """
        Robuste AprilTag-Erkennung mit drei Filterstufen.

        Bei mehreren sichtbaren Tags:
          → alle durch den Drei-Ebenen-Filter (Whitelist + Position + Größe)
          → vom Rest den größten (= nächsten) nehmen
          → Ergebnis durch Stabilitätsfilter schicken

        Rückgabe:
          tag_detected (bool)
          tag_id       (int)    – stabile ID, -1 wenn keine
          direction    (str)    – erlaubte Richtungen kommagetrennt
          debug_img    (ndarray)
        """
        if not APRILTAG_AVAILABLE:
            self._update_stability(-1)
            return False, -1, "unknown", img_bgr.copy()

        gray      = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        all_tags  = self.detector.detect(gray)
        debug_img = img_bgr.copy()
        img_h, img_w = img_bgr.shape[:2]

        # Alle Tags im Debug-Bild grau einzeichnen (auch verworfene)
        for tag in all_tags:
            corners = tag.corners.astype(int)
            for i in range(4):
                cv2.line(debug_img,
                         tuple(corners[i]), tuple(corners[(i + 1) % 4]),
                         (128, 128, 128), 1)
            cv2.putText(debug_img, f"#{tag.tag_id}",
                        (int(tag.center[0]) - 15, int(tag.center[1])),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (128, 128, 128), 1)

        # ── Ebenen 1-3 anwenden ───────────────────────────────────────────────
        valid_tags = []
        for tag in all_tags:
            ok, reason = self._is_valid_intersection_tag(tag, img_h, img_w)
            if ok:
                valid_tags.append(tag)
            else:
                rospy.logdebug(f"[apriltag] Tag #{tag.tag_id} verworfen: {reason}")

        # ── Besten auswählen (größter = nächster) ─────────────────────────────
        if valid_tags:
            best      = max(valid_tags, key=self._tag_area)
            candidate = best.tag_id

            # Besten Tag grün hervorheben
            corners = best.corners.astype(int)
            for i in range(4):
                cv2.line(debug_img,
                         tuple(corners[i]), tuple(corners[(i + 1) % 4]),
                         (0, 255, 0), 2)
            cx, cy = int(best.center[0]), int(best.center[1])
            cv2.putText(debug_img, f"ID:{candidate}",
                        (cx - 20, cy - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(debug_img,
                        f"cand {self._candidate_count}/{self.stability_required}",
                        (cx - 20, cy + 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 220, 0), 1)

            if len(valid_tags) > 1:
                # Warnung: mehrere gültige Tags sichtbar
                ids_str = str([t.tag_id for t in valid_tags])
                rospy.logwarn(f"[apriltag] {len(valid_tags)} gültige Tags sichtbar "
                              f"({ids_str}) → nehme größten: #{candidate}")
                cv2.putText(debug_img, f"MULTI-TAG! nahm #{candidate}",
                            (10, img_h - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 2)
        else:
            candidate = -1

        # ── Stabilitätsfilter ─────────────────────────────────────────────────
        stable_id = self._update_stability(candidate)

        if stable_id == -1:
            return False, -1, "unknown", debug_img

        allowed   = self.tag_directions.get(stable_id, ["straight"])
        direction = ",".join(allowed)

        cv2.putText(debug_img, f"STABLE:{stable_id} → {direction}",
                    (10, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 180), 2)

        return True, stable_id, direction, debug_img

    # ── Haupt-Callback ────────────────────────────────────────────────────────

    def cbImage(self, image_msg):
        if self.is_running:
            return
        self.is_running = True

        try:
            np_arr   = np.frombuffer(image_msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # 1. Rote Linie erkennen
            red_detected, red_side, mask_red = self._detect_red_line(cv_image)

            # 2. AprilTags erkennen
            tag_detected, tag_id, direction, debug_img = self._detect_apriltags(cv_image)

            # 3. Kreuzung = rote Linie UND AprilTag gleichzeitig sichtbar
            intersection = red_detected and tag_detected

            # 4. Publizieren
            self.pub_intersection.publish(Bool(data=intersection))
            self.pub_direction.publish(String(data=direction))
            self.pub_tag_id.publish(Int32(data=tag_id))
            self.pub_red_side.publish(String(data=red_side))

            # ROI-Linie und Status in Debug-Bild einzeichnen
            h, w = cv_image.shape[:2]
            roi_y = int(h * self.red_detection_zone)
            cv2.line(debug_img, (0, roi_y), (w, roi_y), (0, 0, 255), 1)
            status = f"INTERSECTION" if intersection else f"red={red_detected} tag={tag_detected}"
            color  = (0, 0, 255) if intersection else (200, 200, 200)
            cv2.putText(debug_img, status, (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            cv2.putText(debug_img, f"side={red_side}", (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

            if intersection:
                cv2.rectangle(debug_img, (0, 0), (w - 1, h - 1), (0, 0, 255), 4)

            self.debug_img     = debug_img
            self.debug_red_img = mask_red

            rospy.logdebug(f"[apriltag] red={red_detected}({red_side}) "
                           f"tag={tag_detected}(ID={tag_id}) "
                           f"dir={direction} intersection={intersection}")

        except Exception as e:
            rospy.logerr(f"[detect_apriltag] Fehler: {e}")
        finally:
            self.is_running = False

    # ── Debug-Schleife ────────────────────────────────────────────────────────

    def run_debug(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.pub_debug.get_num_connections() > 0 and self.debug_img is not None:
                msg = CompressedImage()
                msg.header.stamp = rospy.Time.now()
                msg.format = "jpeg"
                msg.data   = np.array(cv2.imencode('.jpg', self.debug_img)[1]).tobytes()
                self.pub_debug.publish(msg)

            if self.pub_debug_red.get_num_connections() > 0 and self.debug_red_img is not None:
                msg = CompressedImage()
                msg.header.stamp = rospy.Time.now()
                msg.format = "jpeg"
                msg.data   = np.array(cv2.imencode('.jpg', self.debug_red_img)[1]).tobytes()
                self.pub_debug_red.publish(msg)

            rate.sleep()


if __name__ == '__main__':
    node = DetectApriltagNode('detect_apriltag_node')
    node.run_debug()
    rospy.spin()
