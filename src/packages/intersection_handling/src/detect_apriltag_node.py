#!/usr/bin/env python3
"""
DetectApriltagNode – reine AprilTag-Erkennung auf dem Original-Kamerabild.

Die rote-Linien-Erkennung wurde in detect_stop_line_node ausgelagert
(Zwei-Schichten-Architektur: rote Linie = Anhalten, Tag = Richtung).

Publiziert:
  /{vehicle}/detect/apriltag/direction  (String) – erlaubte Richtungen, kommagetrennt
  /{vehicle}/detect/apriltag/id         (Int32)  – erkannte/gemerkte Tag-ID (-1 = keine)
  /{vehicle}/debug/apriltag             (CompressedImage)

Tag-Gedächtnis:
  Tag und rote Linie sind oft nicht gleichzeitig sichtbar (Kurve, seitlicher Tag).
  Ein NAHER Tag (Fläche >= tag_memory_min_area) wird für tag_memory_seconds
  gemerkt. switch_control_node kombiniert dieses Gedächtnis mit dem stop_line-Signal.

Tag-Familie: aus JSON "tag_families". Diese Strecke: ["tagStandard52h13"].
"""

import json
import os
import rospy
import numpy as np
import cv2
from std_msgs.msg import String, Int32
from sensor_msgs.msg import CompressedImage
import util

try:
    from pupil_apriltags import Detector
    APRILTAG_AVAILABLE = True
    APRILTAG_PKG = "pupil_apriltags"
except ImportError:
    try:
        from dt_apriltags import Detector
        APRILTAG_AVAILABLE = True
        APRILTAG_PKG = "dt_apriltags"
    except ImportError:
        print("[detect_apriltag] WARNUNG: weder pupil_apriltags noch dt_apriltags "
              "installiert! Installiere mit: pip3 install pupil-apriltags  (oder dt-apriltags)")
        APRILTAG_AVAILABLE = False
        APRILTAG_PKG = None


class DetectApriltagNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        self._node_name    = node_name
        self._vehicle_name = os.environ['VEHICLE_NAME']

        util.init_parameters(node_name, self.cbUpdateParameters)
        self._load_tag_config()

        self.detectors = []
        if APRILTAG_AVAILABLE:
            for family in self.tag_families:
                try:
                    det = Detector(families=family, nthreads=1, quad_decimate=2.0,
                                   quad_sigma=0.0, refine_edges=1, decode_sharpening=0.25)
                    self.detectors.append((family, det))
                    rospy.loginfo(f"[detect_apriltag] Detektor geladen: {family}")
                except Exception as e:
                    rospy.logwarn(f"[detect_apriltag] Familie {family} nicht verfügbar: {e}")

        self.pub_direction = rospy.Publisher(
            f'/{self._vehicle_name}/detect/apriltag/direction', String, queue_size=1)
        self.pub_tag_id = rospy.Publisher(
            f'/{self._vehicle_name}/detect/apriltag/id', Int32, queue_size=1)
        self.pub_debug = rospy.Publisher(
            f'/{self._vehicle_name}/debug/apriltag', CompressedImage, queue_size=1)

        self.is_running   = False
        self.debug_img    = None
        self._frame_count = 0
        self._stable_id        = -1
        self._candidate_id     = -1
        self._candidate_count  = 0
        self._mem_tag_id    = -1
        self._mem_direction = "unknown"
        self._mem_time      = None

        self.frame_skip          = getattr(self, 'frame_skip', 3)
        self.show_window         = getattr(self, 'show_window', False)
        self.stability_required  = getattr(self, 'stability_required', 3)
        self.tag_memory_seconds  = getattr(self, 'tag_memory_seconds', 3.0)
        self.tag_memory_min_area = getattr(self, 'tag_memory_min_area', 1500)

        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        rospy.Subscriber(self._camera_topic, CompressedImage,
                         self.cbImage, queue_size=1, buff_size=2**24)

        if APRILTAG_AVAILABLE:
            rospy.loginfo(f"[{node_name}] Bereit (Paket: {APRILTAG_PKG}). "
                          f"Familien: {self.tag_families}  Mapping: {self.tag_directions}")
        else:
            rospy.logwarn(f"[{node_name}] Läuft OHNE Tag-Erkennung – Library fehlt!")

    def _load_tag_config(self):
        path = os.path.join(os.path.dirname(__file__), f"../config/{self._node_name}.json")
        with open(path, 'r') as f:
            config = json.load(f)
        raw = config.get("tag_directions", {})
        self.tag_directions = {int(k): v for k, v in raw.items()}
        self.tag_families   = config.get("tag_families", ["tagStandard52h13"])

    def cbUpdateParameters(self, parameters):
        t = parameters["tag_filter"]
        self.stability_required = int(t["stability_frames"]["default"])
        self.pos_filter_enabled = bool(t["pos_filter_enabled"]["default"])
        self.pos_x_min          = t["pos_x_min"]["default"]
        self.pos_x_max          = t["pos_x_max"]["default"]
        self.pos_y_max          = t["pos_y_max"]["default"]
        self.min_tag_area       = t["min_area"]["default"]
        p = parameters["performance"]
        self.frame_skip  = max(1, int(p["frame_skip"]["default"]))
        self.show_window = bool(p["show_window"]["default"])
        m = parameters["tag_memory"]
        self.tag_memory_seconds  = m["seconds"]["default"]
        self.tag_memory_min_area = m["min_area"]["default"]

    @staticmethod
    def _tag_area(tag):
        c = tag.corners
        return 0.5 * abs(
            (c[0][0] * c[1][1] - c[1][0] * c[0][1]) +
            (c[1][0] * c[2][1] - c[2][0] * c[1][1]) +
            (c[2][0] * c[3][1] - c[3][0] * c[2][1]) +
            (c[3][0] * c[0][1] - c[0][0] * c[3][1]))

    def _is_valid_tag(self, tag, img_h, img_w):
        if tag.tag_id not in self.tag_directions:
            return False, "not in whitelist"
        if self.pos_filter_enabled:
            cx_rel = tag.center[0] / img_w
            cy_rel = tag.center[1] / img_h
            if not (self.pos_x_min <= cx_rel <= self.pos_x_max):
                return False, f"x={cx_rel:.2f}"
            if cy_rel > self.pos_y_max:
                return False, f"y={cy_rel:.2f}"
        if self._tag_area(tag) < self.min_tag_area:
            return False, "zu klein"
        return True, "ok"

    def _update_stability(self, candidate_id):
        if candidate_id == -1:
            self._candidate_id    = -1
            self._candidate_count = 0
            return self._stable_id
        if candidate_id == self._candidate_id:
            self._candidate_count += 1
        else:
            self._candidate_id    = candidate_id
            self._candidate_count = 1
        if self._candidate_count >= self.stability_required:
            self._stable_id = self._candidate_id
        return self._stable_id

    def _detect_tags(self, img_bgr):
        debug_img = img_bgr.copy()
        if not APRILTAG_AVAILABLE:
            self._update_stability(-1)
            return False, -1, "unknown", debug_img, 0.0
        gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        all_tags = []
        for family, det in self.detectors:
            for tag in det.detect(gray):
                all_tags.append((family, tag))
        img_h, img_w = img_bgr.shape[:2]
        for family, tag in all_tags:
            corners = tag.corners.astype(int)
            for i in range(4):
                cv2.line(debug_img, tuple(corners[i]),
                         tuple(corners[(i + 1) % 4]), (128, 128, 128), 1)
        valid_tags = [tag for family, tag in all_tags
                      if self._is_valid_tag(tag, img_h, img_w)[0]]
        best_area = 0.0
        if valid_tags:
            best      = max(valid_tags, key=self._tag_area)
            candidate = best.tag_id
            best_area = self._tag_area(best)
            corners = best.corners.astype(int)
            for i in range(4):
                cv2.line(debug_img, tuple(corners[i]),
                         tuple(corners[(i + 1) % 4]), (0, 255, 0), 2)
            cx, cy = int(best.center[0]), int(best.center[1])
            cv2.putText(debug_img, f"ID:{candidate} area:{int(best_area)}",
                        (cx - 20, cy - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            if len(valid_tags) > 1:
                ids_str = str([t.tag_id for t in valid_tags])
                rospy.logwarn(f"[apriltag] mehrere gültige Tags {ids_str} → größter: #{candidate}")
        else:
            candidate = -1
        stable_id = self._update_stability(candidate)
        if stable_id == -1:
            return False, -1, "unknown", debug_img, best_area
        allowed   = self.tag_directions.get(stable_id, ["straight"])
        direction = ",".join(allowed)
        return True, stable_id, direction, debug_img, best_area

    def cbImage(self, image_msg):
        if self.is_running:
            return
        self._frame_count += 1
        if self._frame_count % self.frame_skip != 0:
            return
        self.is_running = True
        try:
            np_arr   = np.frombuffer(image_msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            tag_detected, tag_id, direction, debug_img, tag_area = self._detect_tags(cv_image)

            now = rospy.Time.now()
            if tag_detected and tag_area >= self.tag_memory_min_area:
                self._mem_tag_id    = tag_id
                self._mem_direction = direction
                self._mem_time      = now
            mem_age = (now - self._mem_time).to_sec() if self._mem_time else 999.0
            mem_valid = (self._mem_tag_id != -1) and (mem_age <= self.tag_memory_seconds)

            if tag_detected:
                out_id, out_dir = tag_id, direction
            elif mem_valid:
                out_id, out_dir = self._mem_tag_id, self._mem_direction
            else:
                out_id, out_dir = -1, "unknown"
            self.pub_direction.publish(String(data=out_dir))
            self.pub_tag_id.publish(Int32(data=out_id))

            h, w = cv_image.shape[:2]
            cv2.rectangle(debug_img, (0, 0), (470, 130), (0, 0, 0), -1)
            id_txt = str(out_id) if out_id != -1 else "None"
            cv2.putText(debug_img, f"Tag ID: {id_txt}", (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            allowed_txt = out_dir.replace(",", ", ") if out_id != -1 else "-"
            cv2.putText(debug_img, f"Erlaubt: {allowed_txt}", (10, 58),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
            mem_txt = (f"Gedaechtnis: ID {self._mem_tag_id}, Alter {mem_age:.1f}s"
                       if self._mem_tag_id != -1 else "Gedaechtnis: leer")
            mem_col = (0, 255, 0) if mem_valid else (120, 120, 120)
            cv2.putText(debug_img, mem_txt, (10, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, mem_col, 1)
            src_txt = "live" if tag_detected else ("Gedaechtnis" if mem_valid else "-")
            cv2.putText(debug_img, f"Quelle: {src_txt}", (10, 116),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            self.debug_img = debug_img

            # ╔══════════════════════════════════════════════════════════════╗
            # ║  DEBUG-FENSTER – zum Abschalten die 2 Zeilen auskommentieren   ║
            # ╚══════════════════════════════════════════════════════════════╝
            cv2.imshow("AprilTag", debug_img)
            cv2.waitKey(1)
            # ── Ende Debug-Fenster ──────────────────────────────────────────
        except Exception as e:
            rospy.logerr(f"[detect_apriltag] Fehler: {e}")
        finally:
            self.is_running = False

    def run_debug(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.pub_debug.get_num_connections() > 0 and self.debug_img is not None:
                msg = CompressedImage()
                msg.header.stamp = rospy.Time.now()
                msg.format = "jpeg"
                msg.data   = np.array(cv2.imencode('.jpg', self.debug_img)[1]).tobytes()
                self.pub_debug.publish(msg)
            rate.sleep()


if __name__ == '__main__':
    node = DetectApriltagNode('detect_apriltag_node')
    node.run_debug()
    rospy.spin()
