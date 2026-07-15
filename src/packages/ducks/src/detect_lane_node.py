#!/usr/bin/env python3

import os
import rospy
import numpy as np
import cv2
from std_msgs.msg import Float64, Bool, Float32MultiArray, String
from sensor_msgs.msg import CompressedImage
import util


class _Kalman1D:
    # Einfacher 1D-Kalman-Filter (Position + Geschwindigkeit) zum Glätten der
    # Enten-x-Position und Überbrücken kurzer Erkennungsaussetzer (z.B. Ente
    # kurz außerhalb des Sichtbereichs) statt sofort auf "keine Ente" zu springen.
    def __init__(self, process_var, measurement_var):
        self.q = process_var
        self.r = measurement_var
        self.x = 0.0
        self.v = 0.0
        self.p = np.eye(2) * 1000.0
        self.initialized = False

    def reset(self):
        self.initialized = False
        self.v = 0.0
        self.p = np.eye(2) * 1000.0

    def predict(self, dt):
        self.x += self.v * dt
        F = np.array([[1.0, dt], [0.0, 1.0]])
        Q = np.array([[dt**3 / 3, dt**2 / 2],
                       [dt**2 / 2, dt]]) * self.q
        self.p = F @ self.p @ F.T + Q
        return self.x

    def update(self, z):
        if not self.initialized:
            self.x, self.v = z, 0.0
            self.p = np.eye(2)
            self.initialized = True
            return self.x
        H = np.array([1.0, 0.0])
        state = np.array([self.x, self.v])
        y = z - H @ state
        S = H @ self.p @ H.T + self.r
        K = (self.p @ H) / S
        state = state + K * y
        self.p = self.p - np.outer(K, H @ self.p)
        self.x, self.v = state
        return self.x


class DetectLaneNode:
    OCC_BINS = 40  # Auflösung des Enten-Belegungsprofils (x-Spalten)
    GAP_BINS = 20  # Auflösung des Korridor-Lückenprofils (x-Spalten, nur Fahrkorridor)
    # Morphologie-Kernel ändern sich nie – einmal anlegen statt pro Frame neu.
    _MORPH_KERNEL_3 = np.ones((3, 3), np.uint8)
    _MORPH_KERNEL_5 = np.ones((5, 5), np.uint8)

    def __init__(self, node_name):
        # ROS-Node initialisieren
        rospy.init_node(node_name)

        # Fahrzeugnamen aus Umgebungsvariable lesen
        self._vehicle_name = os.environ['VEHICLE_NAME']

        # ── Kritisch: Alle self.*-Variablen VOR util.init_parameters setzen ──
        self._crop_im_size      = 400
        self.is_running          = False
        self.counter             = 0     # Frame-Zähler: erste 3 Frames verwerfen
        self.last_white_position  = None  # Frame-Tracking für weiße Linie

        # CLAHE-Objekt ändert sich nie – einmal anlegen statt pro Frame neu.
        self._clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

        # Platzhalter für Debug-Variablen
        blank       = np.zeros((self._crop_im_size, self._crop_im_size), dtype=np.uint8)
        blank_color = np.zeros((self._crop_im_size, self._crop_im_size, 3), dtype=np.uint8)
        self.debug_img_white     = blank
        self.debug_img_red       = blank
        self.debug_img_annotated = blank_color

        # ── Enten-Erkennung (Challenge 3) – Defaults VOR init_parameters ──────
        self.duck_enabled         = True
        self.duck_roi_top         = 0.35
        self.duck_roi_bottom      = 1.00
        self.duck_min_area        = 250
        self.duck_min_w           = 12
        self.duck_min_h           = 12
        self.debug_img_duck       = blank_color
        self.debug_img_duck_original = blank_color

        # Kalman-Filter für die Enten-x-Position (Glättung + Aussetzer-Überbrückung)
        self.duck_kf_process_var     = 0.01
        self.duck_kf_measurement_var = 0.05
        self.duck_kf_max_missed      = 5
        self._duck_kf           = _Kalman1D(self.duck_kf_process_var, self.duck_kf_measurement_var)
        self._duck_kf_last_t    = None
        self._duck_missed_count = 0

        # ── Hindernis-Farbbereiche (gelb/grün) – Defaults VOR init_parameters ──
        self.yellow_hl, self.yellow_hh = 20, 35
        self.yellow_sl, self.yellow_sh = 80, 255
        self.yellow_vl, self.yellow_vh = 80, 255
        self.green_hl, self.green_hh   = 40, 85
        self.green_sl, self.green_sh   = 60, 255
        self.green_vl, self.green_vh   = 40, 255

        # ── Weiß-Follow-Modus (Stufe 2) ──────────────────────────────────────
        self.white_follow_offset_px = 150

        # ── Zonen-Erkennung (Stufe 3) ─────────────────────────────────────────
        # Korridor "wandert" mit der tatsächlichen Fahrlinie (lane_center) mit,
        # symmetrische Breite = Bot-Breite + Ausweich-Spielraum, NICHT die ganze
        # Spur – reicht dadurch nie über die eigene Spur hinaus (siehe Diskussion).
        self.zone_corridor_width_px    = 200
        self.zone_far_y_min            = 0.20
        self.zone_far_y_max            = 0.45
        self.zone_mid_y_min            = 0.45
        self.zone_mid_y_max            = 0.70
        self.zone_near_y_min           = 0.70
        self.zone_near_y_max           = 0.95
        self.zone_pixel_threshold_frac = 0.05

        # Zustand von control_obstacle_node (Idle/Evade/Wait/Pass/Return) – nur fürs Debug-Overlay
        self.obstacle_state = "Idle"

        # Parameter aus JSON laden + Live-Update Callback registrieren
        util.init_parameters(node_name, self.cbUpdateParameters)

        # ── Subscriber ────────────────────────────────────────────────────────
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self.sub_image_original = rospy.Subscriber(
            self._camera_topic, CompressedImage, self.cbFindLane, queue_size=1, buff_size=2**24)
        self.sub_obstacle_state = rospy.Subscriber(
            f'/{self._vehicle_name}/obstacle/state', String, self.cbObstacleState, queue_size=1)

        # ── Publisher ─────────────────────────────────────────────────────────
        # Spurversatz [-1, +1] an control_lane_node
        self.pub_lane = rospy.Publisher(
            f'/{self._vehicle_name}/detect/lane', Float64, queue_size=1)

        # Rote Haltelinie erkannt (True/False)
        self.pub_stop_line = rospy.Publisher(
            f'/{self._vehicle_name}/detect/stop_line', Bool, queue_size=1)

        # ── Enten-Publisher (Challenge 3) ─────────────────────────────────────
        # x der nächsten Ente [-1,1]; -99 = keine
        self.pub_duck = rospy.Publisher(
            f'/{self._vehicle_name}/detect/duck', Float64, queue_size=1)
        # Zonen-Belegung [nah, mittel, fern] ∈ {0.0, 1.0}
        self.pub_zones = rospy.Publisher(
            f'/{self._vehicle_name}/detect/zones', Float32MultiArray, queue_size=1)
        # Korridor-Lückenprofil (GAP_BINS Spalten, nur Fahrkorridor, nah+mittel-Band,
        # gleiche Maske wie Zonen → gelbe Linie zählt als belegt). Für control_obstacle_node,
        # um den Ausweich-Offset aus der tatsächlich freien Lücke zu berechnen.
        self.pub_corridor_occupancy = rospy.Publisher(
            f'/{self._vehicle_name}/detect/corridor_occupancy', Float32MultiArray, queue_size=1)

        # ── Debug-Publisher ───────────────────────────────────────────────────
        self.pub_debug_original  = rospy.Publisher(
            f'/{self._vehicle_name}/debug/original',    CompressedImage, queue_size=1)
        self.pub_debug_bird      = rospy.Publisher(
            f'/{self._vehicle_name}/debug/bird_view',   CompressedImage, queue_size=1)
        self.pub_debug_annotated = rospy.Publisher(
            f'/{self._vehicle_name}/debug/annotated',   CompressedImage, queue_size=1)
        self.pub_debug_white     = rospy.Publisher(
            f'/{self._vehicle_name}/debug/lane_white',  CompressedImage, queue_size=1)
        self.pub_debug_red       = rospy.Publisher(
            f'/{self._vehicle_name}/debug/lane_red',    CompressedImage, queue_size=1)
        # Enten-Debug-Bild (BEV mit Zonen + Belegungsbalken + projizierten Positionen)
        self.pub_debug_duck      = rospy.Publisher(
            f'/{self._vehicle_name}/debug/duck_bev',    CompressedImage, queue_size=1)
        # Enten-Debug-Bild (Originalbild mit erkannten Boxen, vor der BEV-Transformation)
        self.pub_debug_duck_original = rospy.Publisher(
            f'/{self._vehicle_name}/debug/duck_original', CompressedImage, queue_size=1)


    def cbUpdateParameters(self, parameters):
        # Wird beim Start UND bei jeder Schieberegler-Änderung aufgerufen

        # Weiße Linie (rechte Fahrbahnmarkierung)
        self.hue_white_l        = parameters["white"]["hl"]["default"]
        self.hue_white_h        = parameters["white"]["hh"]["default"]
        self.saturation_white_l = parameters["white"]["sl"]["default"]
        self.saturation_white_h = parameters["white"]["sh"]["default"]
        self.lightness_white_l  = parameters["white"]["vl"]["default"]
        self.lightness_white_h  = parameters["white"]["vh"]["default"]

        # Perspektivtransformation (Bird's-Eye-View Eckpunkte)
        self.top_left_x     = parameters["crop_image"]["top_left_x"]["default"]
        self.top_left_y     = parameters["crop_image"]["top_left_y"]["default"]
        self.top_right_x    = parameters["crop_image"]["top_right_x"]["default"]
        self.top_right_y    = parameters["crop_image"]["top_right_y"]["default"]
        self.bottom_left_x  = parameters["crop_image"]["bottom_left_x"]["default"]
        self.bottom_left_y  = parameters["crop_image"]["bottom_left_y"]["default"]
        self.bottom_right_x = parameters["crop_image"]["bottom_right_x"]["default"]
        self.bottom_right_y = parameters["crop_image"]["bottom_right_y"]["default"]
        # Homographie nur hier (Start + Kalibrier-Änderung) neu berechnen, nicht pro Frame.
        self._bev_M = self._compute_bev_matrix()

        # Rote Haltelinie – zwei HSV-Bereiche
        self.hue_red_l        = parameters["red"]["hl"]["default"]
        self.hue_red_h        = parameters["red"]["hh"]["default"]
        self.hue_red_l2       = parameters["red"]["hl2"]["default"]
        self.hue_red_h2       = parameters["red"]["hh2"]["default"]
        self.saturation_red_l = parameters["red"]["sl"]["default"]
        self.saturation_red_h = parameters["red"]["sh"]["default"]
        self.lightness_red_l  = parameters["red"]["vl"]["default"]
        self.lightness_red_h  = parameters["red"]["vh"]["default"]
        self.red_pixel_threshold  = parameters["red"]["pixel_threshold"]["default"]
        self.red_detection_zone    = parameters["red"]["detection_zone"]["default"]
        self.red_detection_x_start = parameters["red"]["detection_x_start"]["default"]
        self.red_detection_x_end   = parameters["red"]["detection_x_end"]["default"]

        self.max_frame_jump = parameters["white"]["max_frame_jump"]["default"]

        # ── Enten-Parameter (defensiv: fehlende Keys → Default) ───────────────
        def gd(group, key, default):
            try:
                return parameters[group][key]["default"]
            except (KeyError, TypeError):
                rospy.logwarn(f"[detect_lane/duck] Parameter {group}.{key} fehlt – nutze {default}")
                return default
        self.duck_enabled         = int(gd("duck", "enabled", 1)) == 1
        self.duck_roi_top         = gd("duck", "roi_top", 0.35)
        self.duck_roi_bottom      = gd("duck", "roi_bottom", 1.0)
        self.duck_min_area        = gd("duck", "min_area", 250)
        self.duck_min_w           = gd("duck", "min_w", 12)
        self.duck_min_h           = gd("duck", "min_h", 12)

        self.duck_kf_process_var     = gd("duck", "kf_process_var", 0.01)
        self.duck_kf_measurement_var = gd("duck", "kf_measurement_var", 0.05)
        self.duck_kf_max_missed      = int(gd("duck", "kf_max_missed_frames", 5))
        self._duck_kf.q = self.duck_kf_process_var
        self._duck_kf.r = self.duck_kf_measurement_var

        self.yellow_hl = gd("obstacle_color", "yellow_hl", 20)
        self.yellow_hh = gd("obstacle_color", "yellow_hh", 35)
        self.yellow_sl = gd("obstacle_color", "yellow_sl", 80)
        self.yellow_sh = gd("obstacle_color", "yellow_sh", 255)
        self.yellow_vl = gd("obstacle_color", "yellow_vl", 80)
        self.yellow_vh = gd("obstacle_color", "yellow_vh", 255)
        self.green_hl  = gd("obstacle_color", "green_hl",  40)
        self.green_hh  = gd("obstacle_color", "green_hh",  85)
        self.green_sl  = gd("obstacle_color", "green_sl",  60)
        self.green_sh  = gd("obstacle_color", "green_sh",  255)
        self.green_vl  = gd("obstacle_color", "green_vl",  40)
        self.green_vh  = gd("obstacle_color", "green_vh",  255)

        self.white_follow_offset_px = gd("white_follow", "offset_px", 150)

        self.zone_corridor_width_px    = gd("zones", "corridor_width_px",  200)
        self.zone_far_y_min            = gd("zones", "far_y_min",            0.20)
        self.zone_far_y_max            = gd("zones", "far_y_max",            0.45)
        self.zone_mid_y_min            = gd("zones", "mid_y_min",            0.45)
        self.zone_mid_y_max            = gd("zones", "mid_y_max",            0.70)
        self.zone_near_y_min           = gd("zones", "near_y_min",           0.70)
        self.zone_near_y_max           = gd("zones", "near_y_max",           0.95)
        self.zone_pixel_threshold_frac = gd("zones", "pixel_threshold_frac", 0.05)


    def cbObstacleState(self, msg):
        self.obstacle_state = msg.data


    def _compute_bev_matrix(self):
        # Homographie Original-Kamerabild → BEV-Quadrat. Wird NUR in
        # cbUpdateParameters() aufgerufen (Start + Kalibrier-Änderungen über die
        # GUI), NICHT pro Frame – das Ergebnis liegt in self._bev_M gecacht,
        # da sich die Kalibrierwerte sonst so gut wie nie ändern.
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
        return cv2.getPerspectiveTransform(pts1, pts2)

    def crop_img(self, img):
        # Bird's-Eye-View Transformation: Trapez der Fahrspur → Quadrat (Draufsicht).
        # warpPerspective liest nur aus img, verändert es nie – keine Kopie nötig.
        return cv2.warpPerspective(img, self._bev_M, (self._crop_im_size, self._crop_im_size))

    def _project_points_to_bev(self, points):
        # points: Liste von (x, y) im Original-Kamerabild → BEV-Pixelkoordinaten,
        # per derselben (gecachten) Homographie wie crop_img(). Für Enten-
        # Bodenkontaktpunkte, damit die Objekterkennung im unverzerrten
        # Originalbild laufen kann, aber die Position trotzdem im
        # Fahrkorridor-Koordinatensystem landet.
        pts = np.float32(points).reshape(-1, 1, 2)
        bev_pts = cv2.perspectiveTransform(pts, self._bev_M)
        return [(float(p[0][0]), float(p[0][1])) for p in bev_pts]


    def get_x_for_driving(self, mask, distance, left_line, last_known=None):
        grad = cv2.Sobel(mask, cv2.CV_16S, 1, 0, ksize=3, scale=1, delta=0,
                         borderType=cv2.BORDER_DEFAULT)
        _, th1 = cv2.threshold(grad, 127, 255, cv2.THRESH_BINARY)

        a = []
        for row in range(distance - 50, distance + 50):
            candidates = np.where(th1[row] == 255)[0]
            if candidates.size == 0:
                continue
            if last_known is not None:
                a.append(candidates[np.argmin(np.abs(candidates - last_known))])
            elif left_line:
                a.append(candidates[-1])
            else:
                a.append(candidates[0])

        if len(a) > 10:
            return np.median(a)
        return None


    def _resolve_line_position(self, raw, last_known, fallback, max_jump, label):
        if raw is None:
            if last_known is not None:
                print(f"{label}: no edges – keeping last position {last_known:.0f}")
                return last_known, last_known
            print(f"{label}: no edges and no last known – using image-edge fallback {fallback}")
            return fallback, None
        if last_known is None:
            return raw, raw
        jump = abs(raw - last_known)
        if jump > max_jump:
            print(f"{label} jump too large ({jump:.0f}px) – keeping last position")
            return last_known, last_known
        return raw, raw


    def detect_stop_line(self, hsv):
        mask_red_lower = cv2.inRange(hsv,
            (self.hue_red_l,  self.saturation_red_l, self.lightness_red_l),
            (self.hue_red_h,  self.saturation_red_h, self.lightness_red_h))
        mask_red_upper = cv2.inRange(hsv,
            (self.hue_red_l2, self.saturation_red_l, self.lightness_red_l),
            (self.hue_red_h2, self.saturation_red_h, self.lightness_red_h))
        mask_red = cv2.bitwise_or(mask_red_lower, mask_red_upper)

        detection_row_start = int(mask_red.shape[0] * self.red_detection_zone)
        detection_col_start = int(mask_red.shape[1] * self.red_detection_x_start)
        detection_col_end   = int(mask_red.shape[1] * self.red_detection_x_end)
        roi_own = mask_red[detection_row_start:, detection_col_start:detection_col_end]

        red_pixel_count    = cv2.countNonZero(roi_own)
        stop_line_detected = red_pixel_count > self.red_pixel_threshold
        print(f"Red pixels: {red_pixel_count} | threshold: {self.red_pixel_threshold} | detected: {stop_line_detected}")

        return stop_line_detected, mask_red


    # ──────────────────────────────────────────────────────────────────────────
    #  ENTEN-ERKENNUNG (Challenge 3) – integriert, nutzt dasselbe BEV-Bild
    # ──────────────────────────────────────────────────────────────────────────

    def _color_obstacle_mask(self, bev_bgr):
        # Hindernis = gelb ODER grün (Enten + gelbe Mittellinie zählen gleich,
        # unbunte Reflexionen/Kleberest fallen automatisch raus).
        blurred = cv2.GaussianBlur(bev_bgr, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask_yellow = cv2.inRange(hsv,
            (self.yellow_hl, self.yellow_sl, self.yellow_vl),
            (self.yellow_hh, self.yellow_sh, self.yellow_vh))
        mask_green = cv2.inRange(hsv,
            (self.green_hl, self.green_sl, self.green_vl),
            (self.green_hh, self.green_sh, self.green_vh))
        mask = cv2.bitwise_or(mask_yellow, mask_green)
        return cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._MORPH_KERNEL_3)

    def _duck_object_mask(self, obstacle_mask):
        # ROI-Zuschnitt auf einer bereits berechneten Farbmaske (Kopie, damit
        # der Aufrufer dieselbe Maske noch unbeschnitten weiterverwenden kann).
        mask = obstacle_mask.copy()
        h = mask.shape[0]
        y0 = max(0, int(h * self.duck_roi_top))
        y1 = min(h, int(h * self.duck_roi_bottom))
        mask[:y0, :] = 0
        if y1 < h:
            mask[y1:, :] = 0
        return mask

    def _duck_blobs(self, mask):
        # Zusammenhangskomponenten mit Formfilter (gegen Rauschen/Linien).
        num, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        blobs = []
        for i in range(1, num):
            x = stats[i, cv2.CC_STAT_LEFT];  y = stats[i, cv2.CC_STAT_TOP]
            w = stats[i, cv2.CC_STAT_WIDTH]; h = stats[i, cv2.CC_STAT_HEIGHT]
            area = stats[i, cv2.CC_STAT_AREA]
            if area < self.duck_min_area:
                continue
            if w < self.duck_min_w or h < self.duck_min_h:
                continue
            blobs.append((int(x), int(y), int(w), int(h)))
        return blobs

    def _duck_occupancy_from_bev(self, projected, width):
        # projected: Liste von (bx_left, bx_right, by, orig_box) – bereits ins
        # BEV projizierte Bodenkontakt-Kanten (siehe _process_ducks).
        occ = np.zeros(self.OCC_BINS, dtype=np.float32)
        for (bx_left, bx_right, _by, _box) in projected:
            b0 = max(0, min(self.OCC_BINS - 1, int(bx_left  / width * self.OCC_BINS)))
            b1 = max(0, min(self.OCC_BINS - 1, int(bx_right / width * self.OCC_BINS)))
            if b1 < b0:
                b0, b1 = b1, b0
            occ[b0:b1 + 1] = 1.0
        return occ

    def _update_duck_kalman(self, raw_x):
        # Glättet die Enten-x-Position und überbrückt kurze Aussetzer per Kalman-
        # Filter (Predict/Update), statt bei jedem verpassten Frame sofort auf
        # "keine Ente" (-99) zu springen. raw_x = -99.0 heißt "diesen Frame nichts erkannt".
        now = rospy.Time.now().to_sec()
        dt  = (now - self._duck_kf_last_t) if self._duck_kf_last_t is not None else 0.1
        self._duck_kf_last_t = now

        if raw_x != -99.0:
            self._duck_kf.predict(dt)
            self._duck_missed_count = 0
            return self._duck_kf.update(raw_x)

        if self._duck_kf.initialized and self._duck_missed_count < self.duck_kf_max_missed:
            self._duck_missed_count += 1
            return self._duck_kf.predict(dt)

        self._duck_kf.reset()
        self._duck_missed_count = 0
        return -99.0

    def _process_ducks(self, orig_bgr, bev_bgr, bev_width):
        # Enten-Erkennung im UNVERZERRTEN Originalbild (kein Verschwinden am
        # BEV-Trapezrand, keine Höhen-Verzerrung durch die Homographie). Nur der
        # Bodenkontaktpunkt jeder Box (unterste Kante) wird anschließend per
        # _project_points_to_bev() ins Fahrkorridor-Koordinatensystem übersetzt.
        try:
            mask  = self._duck_object_mask(self._color_obstacle_mask(orig_bgr))
            blobs = self._duck_blobs(mask)

            projected = []
            if blobs:
                ground_pts = []
                for (x, y, w, h) in blobs:
                    ground_pts.append((x, y + h))          # unten links
                    ground_pts.append((x + w, y + h))      # unten rechts
                bev_pts = self._project_points_to_bev(ground_pts)
                for i, box in enumerate(blobs):
                    bx_l, by_l = bev_pts[2 * i]
                    bx_r, by_r = bev_pts[2 * i + 1]
                    bx_left, bx_right = sorted((bx_l, bx_r))
                    projected.append((bx_left, bx_right, max(by_l, by_r), box))

            occ = self._duck_occupancy_from_bev(projected, bev_width)  # nur fuers Debug-Bild

            raw_duck_x = -99.0
            if projected:
                nearest = max(projected, key=lambda p: p[2])  # größtes BEV-y = am nächsten
                bx_center = (nearest[0] + nearest[1]) / 2.0
                raw_duck_x = max(-1.0, min(1.0, (bx_center / bev_width) * 2.0 - 1.0))

            duck_x = self._update_duck_kalman(raw_duck_x)
            self.pub_duck.publish(Float64(data=duck_x))

            if blobs:
                rospy.loginfo_throttle(1.0,
                    f"[duck] {len(blobs)} Blobs (Originalbild), naechste x={duck_x:.2f}")

            # Debug 1: Originalbild mit erkannten Boxen (vor der BEV-Transformation)
            dbg_orig = orig_bgr.copy()
            ho, wo = dbg_orig.shape[:2]
            y0 = int(ho * self.duck_roi_top)
            y1 = int(ho * self.duck_roi_bottom)
            cv2.line(dbg_orig, (0, y0), (wo, y0), (0, 255, 255), 1)
            if y1 < ho:
                cv2.line(dbg_orig, (0, y1), (wo, y1), (0, 255, 255), 1)
            for (x, y, bw, bh) in blobs:
                cv2.rectangle(dbg_orig, (x, y), (x + bw, y + bh), (0, 0, 255), 2)
            self.debug_img_duck_original = dbg_orig

            # Debug 2: BEV mit projizierten Positionen + Belegungsbalken
            # (Basis für den Zonen-Overlay in _process_zones)
            dbg = bev_bgr.copy()
            hh, ww = dbg.shape[:2]
            for (bx_left, bx_right, by, _box) in projected:
                cx = int((bx_left + bx_right) / 2)
                cy = int(max(0, min(hh - 1, by)))
                cv2.circle(dbg, (cx, cy), 5, (0, 0, 255), -1)
            bar_h = 18
            for i in range(self.OCC_BINS):
                xa = int(i / self.OCC_BINS * ww)
                xb = int((i + 1) / self.OCC_BINS * ww)
                color = (0, 0, 255) if occ[i] > 0.5 else (0, 200, 0)
                cv2.rectangle(dbg, (xa, hh - bar_h), (xb, hh), color, -1)
            cv2.putText(dbg, "frei / blockiert", (4, hh - bar_h - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
            self.debug_img_duck = dbg
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"[duck] Fehler in _process_ducks: {e}")


    # ──────────────────────────────────────────────────────────────────────────
    #  ZONEN-ERKENNUNG (Stufe 3) – drei Bereiche im Fahrkorridor
    # ──────────────────────────────────────────────────────────────────────────

    def _corridor_gap_profile(self, mask, x0, x1, y0, y1):
        # Bins den Fahrkorridor (nah+mittel-Band) in GAP_BINS Spalten und markiert
        # jede Spalte als belegt/frei – dieselbe Maske wie die Zonen (gelbe Linie
        # zählt also mit als Hindernis, keine separate Farbfilterung).
        profile = np.zeros(self.GAP_BINS, dtype=np.float32)
        band_w = x1 - x0
        if band_w <= 0 or y1 <= y0:
            return profile
        band = mask[y0:y1, x0:x1]
        for i in range(self.GAP_BINS):
            bx0 = int(i / self.GAP_BINS * band_w)
            bx1 = max(bx0 + 1, int((i + 1) / self.GAP_BINS * band_w))
            col  = band[:, bx0:bx1]
            area = max(1, col.shape[0] * col.shape[1])
            profile[i] = 1.0 if (cv2.countNonZero(col) / area) > self.zone_pixel_threshold_frac else 0.0
        return profile

    def _process_zones(self, bev_bgr, mask):
        # mask: bereits berechnete Farbmaske (aus cbFindLane, ohne ROI-Beschnitt) –
        # gleiche Gelb/Grün-Erkennung wie bei den Enten, keine Neuberechnung.
        try:
            H, W = bev_bgr.shape[:2]

            # Korridor x-Grenzen relativ zur tatsächlichen Fahrlinie (lane_center,
            # dieselbe Position wie die magenta Ziellinie im annotierten Bild) –
            # symmetrisch um die Bot-Breite + Ausweich-Spielraum, NICHT die ganze
            # Spur. Reicht dadurch nie über den eigenen Fahrbereich hinaus in die
            # Gegenspur/gelbe Linie. last_white_position ist die Position vom
            # Vorframe (weiße Linie wird in cbFindLane erst NACH der Zonen-
            # Verarbeitung neu bestimmt) – ein Frame Verzögerung ist bei 10-30 Hz
            # vernachlässigbar.
            white_x     = self.last_white_position if self.last_white_position is not None else W * 0.95
            lane_center = white_x - self.white_follow_offset_px
            half_w      = self.zone_corridor_width_px / 2.0
            x0 = int(max(0, lane_center - half_w))
            x1 = int(min(W, lane_center + half_w))

            zone_defs = [
                ('nah',    self.zone_near_y_min, self.zone_near_y_max),
                ('mittel', self.zone_mid_y_min,  self.zone_mid_y_max),
                ('fern',   self.zone_far_y_min,  self.zone_far_y_max),
            ]

            results = {}
            for name, y_min_f, y_max_f in zone_defs:
                y0_z = int(y_min_f * H)
                y1_z = int(y_max_f * H)
                roi  = mask[y0_z:y1_z, x0:x1]
                area = max(1, (y1_z - y0_z) * (x1 - x0))
                results[name] = cv2.countNonZero(roi) / area > self.zone_pixel_threshold_frac

            near_occ, mid_occ, far_occ = (
                results['nah'], results['mittel'], results['fern'])

            self.pub_zones.publish(Float32MultiArray(data=[
                float(near_occ), float(mid_occ), float(far_occ)]))

            rospy.loginfo_throttle(1.0,
                f"[zones] nah={'X' if near_occ else 'O'}  "
                f"mittel={'X' if mid_occ else 'O'}  "
                f"fern={'X' if far_occ else 'O'}")

            # ── Korridor-Lückenprofil (nah+mittel-Band zusammen) ──────────────
            # Für control_obstacle_node: Ausweich-Offset aus der tatsächlich
            # freien Lücke statt aus einem festen Wert berechnen.
            y0_gap = int(self.zone_mid_y_min * H)
            y1_gap = int(self.zone_near_y_max * H)
            gap_profile = self._corridor_gap_profile(mask, x0, x1, y0_gap, y1_gap)
            self.pub_corridor_occupancy.publish(Float32MultiArray(data=gap_profile.tolist()))

            # Zonen als halbtransparente Rechtecke auf duck_bev zeichnen – alle drei
            # Füllungen in EINEM Overlay/Blend statt einer Kopie pro Zone.
            dbg = self.debug_img_duck.copy()
            overlay = dbg.copy()
            for name, y_min_f, y_max_f in zone_defs:
                y0_z  = int(y_min_f * H)
                y1_z  = int(y_max_f * H)
                color = (0, 0, 255) if results[name] else (0, 200, 0)
                cv2.rectangle(overlay, (x0, y0_z), (x1, y1_z), color, -1)
            dbg = cv2.addWeighted(overlay, 0.25, dbg, 0.75, 0)
            for name, y_min_f, y_max_f in zone_defs:
                y0_z  = int(y_min_f * H)
                y1_z  = int(y_max_f * H)
                color = (0, 0, 255) if results[name] else (0, 200, 0)
                cv2.rectangle(dbg, (x0, y0_z), (x1, y1_z), color, 2)
                cv2.putText(dbg, name, (x0 + 4, y0_z + 18),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

            # Lückenprofil als Balken am unteren Korridorrand (für Kalibrierung)
            bar_h = 10
            for i in range(self.GAP_BINS):
                bx0 = x0 + int(i / self.GAP_BINS * (x1 - x0))
                bx1 = x0 + int((i + 1) / self.GAP_BINS * (x1 - x0))
                color = (0, 0, 255) if gap_profile[i] > 0.5 else (0, 200, 0)
                cv2.rectangle(dbg, (bx0, y1_gap - bar_h), (bx1, y1_gap), color, -1)
            cv2.putText(dbg, "Luecke", (x0 + 4, y1_gap - bar_h - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)

            self.debug_img_duck = dbg

        except Exception as e:
            rospy.logwarn_throttle(5.0, f"[zones] Fehler in _process_zones: {e}")


    def cbFindLane(self, image_msg):
        # Erste 3 Frames verwerfen (Kamera noch nicht stabil)
        if self.counter <= 3:
            self.counter += 1
            return

        # Sperrvariable: kein paralleles Verarbeiten
        if self.is_running:
            return
        self.is_running = True
        try:
            # ── Schritt 1: Bild dekodieren ────────────────────────────────────
            np_arr   = np.frombuffer(image_msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_image is None:
                rospy.logwarn_throttle(5.0, "Frame nicht dekodierbar – übersprungen.")
                return

            if self.pub_debug_original.get_num_connections() > 0:
                self._publish_compressed(self.pub_debug_original, cv_image)

            # ── Schritt 2: Bird's-Eye-View ────────────────────────────────────
            img = self.crop_img(cv_image)

            if self.pub_debug_bird.get_num_connections() > 0:
                self._publish_compressed(self.pub_debug_bird, img)

            # ── ZONEN: BEV-Ansicht (vor CLAHE) ───────────────────────────────────
            # Farbmaske (gelb/grün) für die Zonen-Erkennung (Korridor-Fahrbahn).
            obstacle_mask = self._color_obstacle_mask(img)
            # ── ENTEN: im unverzerrten Originalbild (kein BEV-Trapez-Sichtfeldlimit,
            # keine Höhen-Verzerrung), Bodenkontaktpunkte werden ins BEV projiziert.
            if self.duck_enabled:
                self._process_ducks(cv_image, img, img.shape[1])
            else:
                self.pub_duck.publish(Float64(data=-99.0))
            self._process_zones(img, obstacle_mask)  # immer aktiv; overlay auf debug_img_duck

            # ── Schritt 3: CLAHE – lokaler Helligkeitsausgleich ───────────────
            lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
            l, a, b = cv2.split(lab)
            l = self._clahe.apply(l)
            img = cv2.cvtColor(cv2.merge((l, a, b)), cv2.COLOR_LAB2BGR)

            # ── Schritt 4: HSV-Masken ─────────────────────────────────────────
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            mask_white = cv2.inRange(hsv,
                (self.hue_white_l, self.saturation_white_l, self.lightness_white_l),
                (self.hue_white_h, self.saturation_white_h, self.lightness_white_h))

            mask_white = cv2.morphologyEx(mask_white, cv2.MORPH_CLOSE, self._MORPH_KERNEL_5)

            # ── Schritt 5: Linienpositionen ───────────────────────────────────
            white_alternative = int(len(img[0]) * 0.95)
            distance          = int(len(img) * 0.75)

            center_white_raw = self.get_x_for_driving(
                mask_white, distance, left_line=False,
                last_known=self.last_white_position)
            center_white, self.last_white_position = self._resolve_line_position(
                center_white_raw, self.last_white_position,
                white_alternative, self.max_frame_jump, label='White')

            # ── Schritt 6: Spurversatz berechnen ──────────────────────────────
            # Zielposition = fester Abstand links von der weißen Linie
            lane_center = center_white - self.white_follow_offset_px
            msg_error = Float64()
            msg_error.data = 1 - (lane_center / len(img) * 2)
            self.pub_lane.publish(msg_error)
            print(f"Lane error: {msg_error.data:.3f} range [-1,1]")

            # ── Schritt 7: Rote Haltelinie ────────────────────────────────────
            stop_line_detected, mask_red = self.detect_stop_line(hsv)
            self.pub_stop_line.publish(Bool(data=stop_line_detected))

            # ── Schritt 8: Debug-Variablen speichern ──────────────────────────
            self.debug_img_white = mask_white
            self.debug_img_red   = mask_red

            # ── Schritt 9: Annotiertes Bild ───────────────────────────────────
            image = cv2.circle(img, (int(lane_center), int(len(img) / 2)), 3, (255, 0, 0))
            image = cv2.line(image, (white_alternative, 0),
                             (white_alternative, self._crop_im_size), color=(255, 255, 255))
            image = cv2.line(image, (0, int(len(img)*0.75)+100),
                             (len(img[0]), int(len(img)*0.75)+100), color=(255, 255, 255))
            image = cv2.line(image, (0, int(len(img)*0.75)-100),
                             (len(img[0]), int(len(img)*0.75)-100), color=(255, 255, 255))
            image = cv2.line(image, (int(len(img[0])/2), 0),
                             (int(len(img[0])/2), len(image)), (0, 255, 0))
            image = cv2.circle(image, (int(center_white), int(len(img)*0.75)), 5, (255, 255, 255))
            # Magenta-Linie = Sollposition (weiße Linie - Offset)
            target_x = int(center_white - self.white_follow_offset_px)
            image = cv2.line(image, (target_x, 0), (target_x, self._crop_im_size),
                             color=(255, 0, 255))

            roi_top   = int(len(img)    * self.red_detection_zone)
            roi_left  = int(len(img[0]) * self.red_detection_x_start)
            roi_right = int(len(img[0]) * self.red_detection_x_end) - 1
            image = cv2.rectangle(image,
                (roi_left, roi_top), (roi_right, self._crop_im_size-1), (0, 0, 255), 2)
            if stop_line_detected:
                image = cv2.rectangle(image,
                    (0, 0), (self._crop_im_size-1, self._crop_im_size-1), (0, 0, 255), 5)

            self.debug_img_annotated = image
            if self.pub_debug_annotated.get_num_connections() > 0:
                self._publish_compressed(self.pub_debug_annotated, image)
        finally:
            self.is_running = False


    def _publish_compressed(self, publisher, img):
        msg              = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format       = "jpeg"
        msg.data         = np.array(cv2.imencode('.jpg', img)[1]).tobytes()
        publisher.publish(msg)


    def run_debug(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.counter <= 3:
                rate.sleep()
                continue

            if self.pub_debug_white.get_num_connections() > 0:
                self._publish_compressed(self.pub_debug_white, self.debug_img_white)

            if self.pub_debug_red.get_num_connections() > 0:
                self._publish_compressed(self.pub_debug_red, self.debug_img_red)

            # Enten-Debug-Bild (BEV mit Boxen + Belegungsbalken) + Zustands-Overlay
            duck_debug_img = self.debug_img_duck.copy()
            cv2.putText(duck_debug_img, f"Zustand: {self.obstacle_state}", (4, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2, cv2.LINE_AA)

            if self.pub_debug_duck.get_num_connections() > 0:
                self._publish_compressed(self.pub_debug_duck, duck_debug_img)

            if self.pub_debug_duck_original.get_num_connections() > 0:
                self._publish_compressed(self.pub_debug_duck_original, self.debug_img_duck_original)

            # Lokale Debug-Fenster (nur für Standalone-Tests) – bei Bedarf auskommentieren
            cv2.imshow("duck_bev", duck_debug_img)
            cv2.imshow("annotated", self.debug_img_annotated)
            cv2.imshow("duck_original", self.debug_img_duck_original)
            cv2.waitKey(1)

            rate.sleep()


if __name__ == '__main__':
    node = DetectLaneNode('detect_lane_node')
    node.run_debug()
    rospy.spin()
