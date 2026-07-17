#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────────────────────
# detect_apriltag_node.py  (Challenge 4 – Mapping & Path Finding)
#
# Reine AprilTag-Erkennung auf dem Originalbild. Die rote Haltelinie erkennt
# detect_lane_node (Bird's-Eye). Hier: welcher Kreuzungs-Tag (1-4, welche Richtungen
# erlaubt) und welcher Tor-Tag (5-13, Zielorte auf den Kanten).
#
# Publiziert:
#   /{vehicle}/detect/apriltag/direction (String) – erlaubte Richtungen, kommagetrennt
#                                                    z.B. "left,straight" – "unknown" wenn keiner
#   /{vehicle}/detect/apriltag/id        (Int32)  – Kreuzungs-Tag-ID 1-4 (-1 = keiner)
#   /{vehicle}/detect/gate/id            (Int32)  – Tor-Tag-ID 5-13 (-1 = keiner)
#   /{vehicle}/debug/apriltag            (CompressedImage)
#
# Tag-Gedächtnis: Tag und rote Linie sind selten gleichzeitig sichtbar (Kurve,
#   seitliches Schild). Ein naher Tag (Fläche >= tag_memory.min_area) wird
#   tag_memory.seconds lang gemerkt, damit switch_control_node beim Erreichen
#   der Haltelinie die Richtung noch kennt.
#
# Tag-Familie: aus JSON "tag_families". Diese Strecke: ["tagStandard52h13"].
# ─────────────────────────────────────────────────────────────────────────────

import json
import os
import rospy
import numpy as np
import cv2
from std_msgs.msg import String, Int32
from sensor_msgs.msg import CompressedImage
import util

# pupil_apriltags ist der primäre Weg (läuft auf eurer aktuellen Hardware/numpy).
# dt_apriltags als Fallback für andere Container – schadet nicht.
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
        print("[detect_apriltag] WARNUNG: keine AprilTag-Library gefunden "
              "(pip3 install pupil-apriltags).")
        APRILTAG_AVAILABLE = False
        APRILTAG_PKG = None


class DetectApriltagNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        self._node_name    = node_name
        self._vehicle_name = os.environ['VEHICLE_NAME']

        # ── self.*-Defaults VOR init_parameters (cbUpdateParameters läuft sofort) ──
        self.is_running          = False
        self.counter             = 0
        self.debug_img           = np.zeros((100, 100, 3), dtype=np.uint8)
        self._stable_id          = -1
        self._candidate_id       = -1
        self._candidate_count    = 0
        self._miss_count         = 0
        self._mem_tag_id         = -1
        self._mem_direction      = "unknown"
        self._mem_time           = None
        self._chosen_direction   = "-"   # von der FSM gewuerfelte Abbiegerichtung
        self._last_intersection_phase = "Lane"
        self._gate_candidate_id    = -1
        self._gate_candidate_count = 0
        # Filter-/Memory-Defaults (werden aus JSON überschrieben)
        self.stability_required  = 3
        self.pos_filter_enabled  = True
        self.pos_x_min           = 0.5
        self.pos_x_max           = 1.0
        self.pos_y_max           = 0.85
        self.min_tag_area        = 200
        self.tag_memory_seconds  = 3.0
        self.tag_memory_min_area = 1500

        # Tag-Richtungen + Familie aus JSON (separat, nicht im parameters-Block)
        self._load_tag_config()

        util.init_parameters(node_name, self.cbUpdateParameters)

        # Detektor(en) – nur benötigte Familie(n), spart Last
        self.detectors = []
        if APRILTAG_AVAILABLE:
            for family in self.tag_families:
                try:
                    self.detectors.append((family, Detector(
                        families=family, nthreads=1, quad_decimate=2.0,
                        quad_sigma=0.0, refine_edges=1, decode_sharpening=0.25)))
                    rospy.loginfo(f"[detect_apriltag] Detektor geladen: {family}")
                except Exception as e:
                    rospy.logwarn(f"[detect_apriltag] Familie {family} nicht verfügbar: {e}")

        # ── Subscriber ────────────────────────────────────────────────────────
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self.sub_image = rospy.Subscriber(
            self._camera_topic, CompressedImage, self.cbImage, queue_size=1)
        # Gewuerfelte Abbiegerichtung von der FSM (nur fuer die Debug-Anzeige)
        self.sub_chosen = rospy.Subscriber(
            f'/{self._vehicle_name}/intersection/direction', String,
            self.cbChosenDirection, queue_size=1)
        # Fuer harten Reset des Tag-Gedaechtnisses beim Start einer neuen Kante
        # (Turning->Lane) - verhindert, dass Alt-Daten der verlassenen Kreuzung
        # in die naechste Kante durchsickern (siehe graph_state_node-Diskussion).
        self.sub_intersection_phase = rospy.Subscriber(
            f'/{self._vehicle_name}/intersection/phase', String,
            self.cbIntersectionPhase, queue_size=1)

        # ── Publisher ─────────────────────────────────────────────────────────
        self.pub_direction = rospy.Publisher(
            f'/{self._vehicle_name}/detect/apriltag/direction', String, queue_size=1)
        self.pub_tag_id = rospy.Publisher(
            f'/{self._vehicle_name}/detect/apriltag/id', Int32, queue_size=1)
        # Tor-Tags (5-13): ID wenn erkannt, sonst -1. Fuer graph_state_node (Challenge 4).
        self.pub_gate_id = rospy.Publisher(
            f'/{self._vehicle_name}/detect/gate/id', Int32, queue_size=1)
        # Zusaetzlich fuer camera_dashboard_node (erwartet /detect/apriltag als Int32)
        self.pub_tag_dash = rospy.Publisher(
            f'/{self._vehicle_name}/detect/apriltag', Int32, queue_size=1)
        self.pub_debug = rospy.Publisher(
            f'/{self._vehicle_name}/debug/apriltag', CompressedImage, queue_size=1)

        if APRILTAG_AVAILABLE:
            rospy.loginfo(f"[{node_name}] Bereit (Paket: {APRILTAG_PKG}). "
                          f"Familien: {self.tag_families}  Mapping: {self.tag_directions}")
        else:
            rospy.logwarn(f"[{node_name}] Läuft OHNE Tag-Erkennung – Library fehlt!")

    # ── Konfiguration ─────────────────────────────────────────────────────────

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

        m = parameters["tag_memory"]
        self.tag_memory_seconds  = m["seconds"]["default"]
        self.tag_memory_min_area = m["min_area"]["default"]

    def cbChosenDirection(self, msg):
        # Von der FSM gewuerfelte Abbiegerichtung (nur Anzeige)
        self._chosen_direction = msg.data if msg.data else "-"

    def cbIntersectionPhase(self, msg):
        phase = msg.data
        if phase == "Lane" and self._last_intersection_phase == "Turning":
            # Neue Kante beginnt: jede Erinnerung an den zuletzt gesehenen
            # Kreuzungs-Tag verwerfen, damit sie nicht faelschlich fuer die
            # naechste Kreuzung weiterverwendet wird.
            self._stable_id       = -1
            self._candidate_id    = -1
            self._candidate_count = 0
            self._miss_count      = 0
            self._mem_tag_id      = -1
            self._mem_direction   = "unknown"
            self._mem_time        = None
            rospy.loginfo("[detect_apriltag] Neue Kante – Tag-Gedaechtnis zurueckgesetzt")
        self._last_intersection_phase = phase

    # ── Tag-Hilfsfunktionen ────────────────────────────────────────────────────

    @staticmethod
    def _tag_area(tag):
        c = tag.corners
        return 0.5 * abs(
            (c[0][0]*c[1][1] - c[1][0]*c[0][1]) + (c[1][0]*c[2][1] - c[2][0]*c[1][1]) +
            (c[2][0]*c[3][1] - c[3][0]*c[2][1]) + (c[3][0]*c[0][1] - c[0][0]*c[3][1]))

    def _is_valid_tag(self, tag, img_h, img_w):
        if tag.tag_id not in self.tag_directions:
            return False
        if self.pos_filter_enabled:
            cx_rel = tag.center[0] / img_w
            cy_rel = tag.center[1] / img_h
            if not (self.pos_x_min <= cx_rel <= self.pos_x_max):
                return False
            if cy_rel > self.pos_y_max:
                return False
        if self._tag_area(tag) < self.min_tag_area:
            return False
        return True

    def _update_stability(self, candidate_id):
        # Tag-ID muss stability_required Frames in Folge gleich sein, bevor sie gilt.
        # Symmetrisch dazu verfaellt _stable_id wieder auf -1, wenn stability_required
        # Frames in Folge KEIN Tag mehr sichtbar ist – ohne diesen Verfall bliebe der
        # zuletzt gesehene Tag unbegrenzt "erkannt" (kein Timeout, anders als beim
        # expliziten tag_memory unten).
        if candidate_id == -1:
            self._candidate_id    = -1
            self._candidate_count = 0
            self._miss_count     += 1
            if self._miss_count >= self.stability_required:
                self._stable_id = -1
            return self._stable_id
        self._miss_count = 0
        if candidate_id == self._candidate_id:
            self._candidate_count += 1
        else:
            self._candidate_id    = candidate_id
            self._candidate_count = 1
        if self._candidate_count >= self.stability_required:
            self._stable_id = self._candidate_id
        return self._stable_id

    def _detect_gate_id(self, all_tags):
        # Tor-Tags (5-13) erscheinen NICHT in tag_directions und durchlaufen daher
        # nicht die Positions-/Richtungsfilter der Kreuzungs-Tags. Kriterium hier:
        # Mindestflaeche (gleiche Schwelle wie bei den Kreuzungs-Tags) UND dieselbe
        # Stabilitaetspruefung (stability_required Frames in Folge) wie bei den
        # Kreuzungs-Tags - graph_state_node uebernimmt einen gemeldeten Tor-Tag
        # dauerhaft und ueberschreibt ihn nie wieder, daher darf hier keine
        # Einzelframe-Fehlerkennung (Bewegungsunschaerfe, Rauschen) durchrutschen.
        candidates = [t for t in all_tags
                      if 5 <= t.tag_id <= 13 and self._tag_area(t) >= self.min_tag_area]
        raw_id = max(candidates, key=self._tag_area).tag_id if candidates else -1

        if raw_id == -1:
            self._gate_candidate_id    = -1
            self._gate_candidate_count = 0
            return -1
        if raw_id == self._gate_candidate_id:
            self._gate_candidate_count += 1
        else:
            self._gate_candidate_id    = raw_id
            self._gate_candidate_count = 1
        if self._gate_candidate_count >= self.stability_required:
            return raw_id
        return -1

    def _detect_tags(self, img_bgr):
        debug_img = img_bgr.copy()
        if not APRILTAG_AVAILABLE:
            self._update_stability(-1)
            return False, -1, "unknown", debug_img, 0.0, -1

        gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        all_tags = []
        for _, det in self.detectors:
            all_tags.extend(det.detect(gray))
        img_h, img_w = img_bgr.shape[:2]

        # Roh erkannte Tags grau
        for tag in all_tags:
            corners = tag.corners.astype(int)
            for i in range(4):
                cv2.line(debug_img, tuple(corners[i]), tuple(corners[(i+1) % 4]), (128, 128, 128), 1)

        gate_id = self._detect_gate_id(all_tags)

        valid = [t for t in all_tags if self._is_valid_tag(t, img_h, img_w)]
        best_area = 0.0
        if valid:
            best      = max(valid, key=self._tag_area)
            candidate = best.tag_id
            best_area = self._tag_area(best)
            corners = best.corners.astype(int)
            for i in range(4):
                cv2.line(debug_img, tuple(corners[i]), tuple(corners[(i+1) % 4]), (0, 255, 0), 2)
            cx, cy = int(best.center[0]), int(best.center[1])
            cv2.putText(debug_img, f"ID:{candidate} A:{int(best_area)}", (cx-20, cy-12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            if len(valid) > 1:
                rospy.logwarn(f"[apriltag] mehrere gueltige Tags {[t.tag_id for t in valid]} "
                              f"-> groesster: #{candidate}")
        else:
            candidate = -1

        stable_id = self._update_stability(candidate)
        if stable_id == -1:
            return False, -1, "unknown", debug_img, best_area, gate_id
        direction = ",".join(self.tag_directions.get(stable_id, ["straight"]))
        return True, stable_id, direction, debug_img, best_area, gate_id

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

            detected, tag_id, direction, debug_img, tag_area, gate_id = self._detect_tags(cv_image)

            # ── Tag-Gedächtnis ────────────────────────────────────────────────
            now = rospy.Time.now()
            if detected and tag_area >= self.tag_memory_min_area:
                self._mem_tag_id, self._mem_direction, self._mem_time = tag_id, direction, now
            mem_age   = (now - self._mem_time).to_sec() if self._mem_time else 999.0
            mem_valid = (self._mem_tag_id != -1) and (mem_age <= self.tag_memory_seconds)

            if detected:
                out_id, out_dir = tag_id, direction
            elif mem_valid:
                out_id, out_dir = self._mem_tag_id, self._mem_direction
            else:
                out_id, out_dir = -1, "unknown"

            self.pub_direction.publish(String(data=out_dir))
            self.pub_tag_id.publish(Int32(data=out_id))
            self.pub_tag_dash.publish(Int32(data=out_id))
            self.pub_gate_id.publish(Int32(data=gate_id))

            # ── Debug-Legende ─────────────────────────────────────────────────
            cv2.rectangle(debug_img, (0, 0), (470, 160), (0, 0, 0), -1)
            cv2.putText(debug_img, f"Tag ID: {out_id if out_id != -1 else 'None'}",
                        (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(debug_img, f"Erlaubt: {out_dir.replace(',', ', ') if out_id != -1 else '-'}",
                        (10, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
            mem_txt = (f"Gedaechtnis: ID {self._mem_tag_id}, Alter {mem_age:.1f}s"
                       if self._mem_tag_id != -1 else "Gedaechtnis: leer")
            cv2.putText(debug_img, mem_txt, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 255, 0) if mem_valid else (120, 120, 120), 1)
            cv2.putText(debug_img, f"Quelle: {'live' if detected else ('Gedaechtnis' if mem_valid else '-')}",
                        (10, 116), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            # Gewuerfelte Abbiegerichtung – gruen wenn sie zu den erlaubten passt
            allowed_list = out_dir.split(",") if out_id != -1 else []
            chosen_ok = self._chosen_direction in allowed_list
            chosen_col = (0, 255, 0) if chosen_ok else (0, 165, 255)
            cv2.putText(debug_img, f"FAHRE: {self._chosen_direction.upper()}",
                        (10, 146), cv2.FONT_HERSHEY_SIMPLEX, 0.6, chosen_col, 2)
            if gate_id != -1:
                cv2.putText(debug_img, f"TOR: {gate_id}", (350, 146),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
            self.debug_img = debug_img

            # Lokale Debug-Ansicht – zum Abschalten die 2 Zeilen auskommentieren:
            cv2.imshow("AprilTag", debug_img)
            cv2.waitKey(1)
        except Exception as e:
            rospy.logerr(f"[detect_apriltag] Fehler: {e}")
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
    node = DetectApriltagNode('detect_apriltag_node')
    node.run_debug()
    rospy.spin()
