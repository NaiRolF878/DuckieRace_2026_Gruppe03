#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────────────────────
# calibrate_bev.py – BEV-Trapez-Kalibrierung per Schachbrett
#
# Kein ROS-Node, sondern ein Kommandozeilen-Werkzeug: berechnet die acht
# crop_image-Eckpunkte fuer detect_lane_node.json aus einem Foto eines flach
# auf der Fahrbahn liegenden Schachbretts mit bekannter Kaestchengroesse,
# statt die vier Trapez-Ecken manuell zu erraten.
#
# Vorgehen:
#   1. cv2.findChessboardCorners() findet die inneren Eckpunkte im Bild.
#   2. Aus den bekannten realen Abstaenden (Kaestchengroesse in cm) wird eine
#      Homographie Bildpixel -> reale Bodenkoordinaten (cm) berechnet.
#   3. Ein gewuenschtes Rechteck auf der Bodenebene (Standard: die Flaeche des
#      Schachbretts selbst; per --fov-* auch ein groesserer Bereich moeglich,
#      da die Homographie fuer die GESAMTE flache Ebene gilt, nicht nur die
#      Flaeche des Bretts) wird zurueck in Bildpixel projiziert.
#   4. Vorschau-BEV-Bild wird erzeugt (identische Transform-Logik wie
#      _compute_bev_matrix()/crop_img() in detect_lane_node.py) – zur
#      Kontrolle VOR jeder Uebernahme in die JSON.
#
# WICHTIG – Achsen-Orientierung ist nicht automatisch bekannt: OpenCV waehlt
# den Startpunkt/die Achsen des Schachbrett-Rasters nicht zwingend so, dass
# "Spalte" = seitlich und "Zeile" = vorwaerts ist. Vorschaubild ansehen und
# bei Bedarf --flip-x / --flip-y / --transpose ergaenzen und erneut laufen
# lassen, bis die Vorschau eine gerade Draufsicht zeigt.
#
# WICHTIG – Namens-Tausch in detect_lane_node.json: _compute_bev_matrix()
# ordnet die JSON-Felder "bottom_left"/"bottom_right" VERTAUSCHT den BEV-Ecken
# zu (bottom_right -> BEV unten-LINKS, bottom_left -> BEV unten-RECHTS). Dieses
# Skript beruecksichtigt das bereits beim Schreiben der Ausgabewerte.
#
# Verwendung:
#   python3 calibrate_bev.py foto.jpg --cols 8 --rows 5 --square-cm 3.5
#
#   --cols/--rows: Anzahl INNERER Eckpunkte (nicht Kaestchen!) - ein Brett mit
#   9x6 Kaestchen hat 8x5 innere Eckpunkte.
# ─────────────────────────────────────────────────────────────────────────────

import argparse
import json
import os
import sys

import cv2
import numpy as np


def find_board_corners(gray, cols, rows):
    pattern_size = (cols, rows)
    found, corners = cv2.findChessboardCorners(
        gray, pattern_size,
        flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not found:
        raise RuntimeError(
            f"Schachbrett ({cols}x{rows} innere Eckpunkte) nicht gefunden – "
            "Beleuchtung, Winkel oder --cols/--rows pruefen.")
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    return corners.reshape(-1, 2)


def build_world_points(cols, rows, square_cm, flip_x, flip_y, transpose):
    # Reale Bodenkoordinaten (cm) je innerem Eckpunkt, Reihenfolge identisch
    # zur Rueckgabe von findChessboardCorners (zeilenweise: j*cols+i).
    pts = np.zeros((cols * rows, 2), dtype=np.float32)
    for j in range(rows):
        for i in range(cols):
            x, y = float(i) * square_cm, float(j) * square_cm
            if transpose:
                x, y = y, x
            pts[j * cols + i] = [x, y]
    if flip_x:
        pts[:, 0] = pts[:, 0].max() - pts[:, 0]
    if flip_y:
        pts[:, 1] = pts[:, 1].max() - pts[:, 1]
    return pts


def compute_crop_image_params(image_path, cols, rows, square_cm,
                               fov_width_cm, fov_forward_cm, fov_offset_x_cm,
                               fov_offset_y_cm, flip_x, flip_y, transpose,
                               crop_im_size=400):
    img = cv2.imread(image_path)
    if img is None:
        raise RuntimeError(f"Bild nicht lesbar: {image_path}")
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    corners = find_board_corners(gray, cols, rows)
    world_pts = build_world_points(cols, rows, square_cm, flip_x, flip_y, transpose)

    H_img_to_world, _ = cv2.findHomography(corners, world_pts)
    H_world_to_img = np.linalg.inv(H_img_to_world)

    if fov_width_cm is None:
        fov_width_cm = (cols - 1) * square_cm
    if fov_forward_cm is None:
        fov_forward_cm = (rows - 1) * square_cm

    # Vier Eckpunkte des gewuenschten Sichtbereichs in realen Bodenkoordinaten:
    # world_x = seitlich, world_y = Entfernung vom Bot (0 = nah, groesser = weiter weg)
    near_left  = [fov_offset_x_cm,               fov_offset_y_cm]
    near_right = [fov_offset_x_cm + fov_width_cm, fov_offset_y_cm]
    far_left   = [fov_offset_x_cm,               fov_offset_y_cm + fov_forward_cm]
    far_right  = [fov_offset_x_cm + fov_width_cm, fov_offset_y_cm + fov_forward_cm]

    world_rect = np.float32([near_left, near_right, far_left, far_right]).reshape(-1, 1, 2)
    img_pts = cv2.perspectiveTransform(world_rect, H_world_to_img).reshape(-1, 2)
    near_left_px, near_right_px, far_left_px, far_right_px = img_pts

    # Namens-Tausch beachten (siehe Kopfkommentar): bottom_right-Feld -> BEV
    # unten-links (= near_left), bottom_left-Feld -> BEV unten-rechts (= near_right).
    params = {
        "top_left_x":     float(far_left_px[0]),  "top_left_y":     float(far_left_px[1]),
        "top_right_x":    float(far_right_px[0]), "top_right_y":    float(far_right_px[1]),
        "bottom_right_x": float(near_left_px[0]),  "bottom_right_y": float(near_left_px[1]),
        "bottom_left_x":  float(near_right_px[0]), "bottom_left_y":  float(near_right_px[1]),
    }

    pts1 = np.float32([
        far_left_px, far_right_px, near_left_px, near_right_px,
    ])
    pts2 = np.float32([
        [0, 0], [crop_im_size, 0], [0, crop_im_size], [crop_im_size, crop_im_size],
    ])
    M = cv2.getPerspectiveTransform(pts1, pts2)
    preview = cv2.warpPerspective(img, M, (crop_im_size, crop_im_size))

    debug_img = img.copy()
    cv2.drawChessboardCorners(debug_img, (cols, rows), corners.reshape(-1, 1, 2), True)
    for label, pt in [("TL", far_left_px), ("TR", far_right_px),
                       ("BL(near_left)", near_left_px), ("BR(near_right)", near_right_px)]:
        p = (int(pt[0]), int(pt[1]))
        cv2.circle(debug_img, p, 6, (0, 0, 255), -1)
        cv2.putText(debug_img, label, (p[0] + 8, p[1]), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 0, 255), 2, cv2.LINE_AA)

    return params, preview, debug_img


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("image", help="Foto mit flach liegendem Schachbrett")
    ap.add_argument("--cols", type=int, required=True, help="innere Eckpunkte, Breite")
    ap.add_argument("--rows", type=int, required=True, help="innere Eckpunkte, Hoehe")
    ap.add_argument("--square-cm", type=float, required=True, help="Kaestchengroesse in cm")
    ap.add_argument("--fov-width-cm", type=float, default=None,
                     help="gewuenschte BEV-Breite in cm (Standard: Brettbreite)")
    ap.add_argument("--fov-forward-cm", type=float, default=None,
                     help="gewuenschte BEV-Tiefe in cm (Standard: Brettlaenge)")
    ap.add_argument("--fov-offset-x-cm", type=float, default=0.0,
                     help="seitlicher Versatz des Sichtbereichs relativ zum Brett-Ursprung")
    ap.add_argument("--fov-offset-y-cm", type=float, default=0.0,
                     help="Versatz in Fahrtrichtung relativ zum Brett-Ursprung")
    ap.add_argument("--flip-x", action="store_true", help="X-Achse umkehren, falls Vorschau seitenverkehrt ist")
    ap.add_argument("--flip-y", action="store_true", help="Y-Achse umkehren, falls Vorschau vorne/hinten vertauscht ist")
    ap.add_argument("--transpose", action="store_true", help="Zeilen/Spalten tauschen, falls Vorschau um 90 Grad verdreht ist")
    ap.add_argument("--crop-im-size", type=int, default=400, help="BEV-Ausgabegroesse in px (muss zu detect_lane_node.py passen)")
    ap.add_argument("--json", default=None, help="Optional: direkt in diese detect_lane_node.json schreiben")
    args = ap.parse_args()

    params, preview, debug_img = compute_crop_image_params(
        args.image, args.cols, args.rows, args.square_cm,
        args.fov_width_cm, args.fov_forward_cm,
        args.fov_offset_x_cm, args.fov_offset_y_cm,
        args.flip_x, args.flip_y, args.transpose, args.crop_im_size)

    base = os.path.splitext(args.image)[0]
    preview_path = f"{base}_bev_preview.png"
    debug_path = f"{base}_corners_debug.png"
    cv2.imwrite(preview_path, preview)
    cv2.imwrite(debug_path, debug_img)

    print("Berechnete crop_image-Werte:")
    for key in ["top_left_x", "top_left_y", "top_right_x", "top_right_y",
                "bottom_left_x", "bottom_left_y", "bottom_right_x", "bottom_right_y"]:
        print(f"  {key}: {params[key]:.1f}")
    print(f"\nVorschau-BEV gespeichert unter: {preview_path}")
    print(f"Erkannte Eckpunkte + Ziel-Ecken gespeichert unter: {debug_path}")
    print("\nBitte beide Bilder ansehen, BEVOR die Werte uebernommen werden:")
    print("  - Zeigt die Vorschau eine gerade, unverzerrte Draufsicht? Falls nicht:")
    print("    --flip-x / --flip-y / --transpose ausprobieren und erneut laufen lassen.")

    if args.json:
        with open(args.json, "r") as f:
            config = json.load(f)
        crop = config["parameters"]["default"]["crop_image"]
        for key, value in params.items():
            crop[key]["default"] = int(round(value))
        with open(args.json, "w") as f:
            json.dump(config, f, indent=4)
        print(f"\n{args.json} aktualisiert.")


if __name__ == "__main__":
    main()
