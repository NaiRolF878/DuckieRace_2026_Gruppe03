# Challenge 3 – Watch out for Ducks

Aufbauend auf eurer Lane-Following-Basis. Die Entenerkennung ist DIREKT in
detect_lane_node integriert (ein Bild-Decode/Warp → spart Latenz). Das Ausweichen
liegt als separate control_obstacle_node vor.

## Architektur
- detect_lane_node.py     → Spur (weiße Linie) + rote Haltelinie + Zonen- und
  Entenerkennung (im Originalbild, Bodenkontaktpunkt per Homographie ins BEV
  projiziert, Kalman-gefiltert) + Korridor-Lückenprofil
  Published: /detect/lane, /detect/stop_line, /detect/duck, /detect/zones,
             /detect/corridor_occupancy, /debug/duck_bev, /debug/duck_original
             (+ bestehende Debug-Bilder)
- control_lane_node.py    → PID-Fahrt + Stop-Automat, addiert /obstacle/error_offset
- control_obstacle_node.py→ Zustandsautomat (IDLE→EVADE→[WAIT]→PASS→RETURN),
  Ausweichrichtung + -stärke aus dem Korridor-Lückenprofil, publiziert Zustand
  als Klartext auf /obstacle/state (Debug-Overlay)
- switch_control_node.py  → Lane/Obstacle-Umschaltung
- camera_dashboard_node.py→ schlankes 2x2-Debug-Dashboard
- configuration_node.py / util.py

Details zu Architektur und Zustandsautomat: siehe `CHALLENGE3_DOKU.md`.

## Start
    export VEHICLE_NAME=<botname>
    cd src
    ./ducks.sh

## Schichtweise testen (Testtag!)
Enten-/Zonen-Erkennung läuft immer mit, beeinflusst die Fahrt aber nur, wenn
`evade.active = 1` (control_obstacle_node.json) gesetzt ist.

STUFE 1 – Fährt der Bot + rote Linie?
  control_obstacle_node.json: evade.active = 0     (Erkennung an, Ausweichen aus)
  → reines Lane-Following (weiße Linie) + Halt an roter Linie.

STUFE 2 – Enten/Zonen richtig erkannt? (noch ohne Ausweichen)
  weiterhin evade.active = 0, jetzt gezielt die Erkennung selbst prüfen:
  → /debug/duck_original: rote Boxen auf Enten im Originalbild?
  → /debug/duck_bev: projizierte Positionen (rote Punkte) + Zonen (nah/mittel/
    fern) korrekt eingefärbt? Lücken-Balken am unteren Korridorrand plausibel?
    Mittellinie löst als Zonen-Hindernis aus (Absicht, siehe Erkennungsprinzip
    unten) – hier obstacle_color/min_area justieren.

STUFE 3 – Ausweichen
  evade.active = 1

## Erkennung kalibrieren (in detect_lane_node.json, Gruppe "obstacle_color" + "duck")
- obstacle_color.yellow_hl/hh/sl/sh/vl/vh, green_* : HSV-Farbbereiche für
  Enten/gelbe Linie (Erkennung läuft im Originalbild, nicht mehr im BEV).
- duck.min_area / min_w / min_h : Mindestgröße eines Enten-Blobs
  (Originalbild-Pixel – andere Skala als früher im BEV, neu kalibrieren!).
- duck.roi_top / roi_bottom : vertikaler Auswertebereich, jetzt bezogen auf die
  Originalbild-Höhe (nicht mehr BEV) – ebenfalls neu kalibrieren.
- duck.kf_process_var / kf_measurement_var / kf_max_missed_frames : Kalman-Filter
  für die Enten-x-Position (Glättung + Überbrückung kurzer Aussetzer).

## Zonen & Lückenprofil kalibrieren (detect_lane_node.json, Gruppe "zones")
- corridor_width_px : Breite des überwachten Fahrkorridors, symmetrisch um die
  tatsächliche Fahrlinie (lane_center) zentriert – entspricht Bot-Breite +
  Ausweich-Spielraum, NICHT die ganze Spur (sonst löst der Bot ständig unnötig aus).
- near_y_min/max, mid_y_min/max, far_y_min/max : die drei Zonen.
- pixel_threshold_frac : ab wann eine Zone/ein Lücken-Bin als belegt gilt.
- Das Korridor-Lückenprofil (/detect/corridor_occupancy) nutzt dieselbe Maske
  wie die Zonen (nah+mittel-Band) – die gelbe Linie zählt hier bewusst als
  Hindernis, damit sie nie als Ausweichweg gewählt wird.

## Ausweichen kalibrieren (control_obstacle_node.json, Gruppe "evade")
- evade_offset / evade_offset_min : maximale / minimale Ausweichstärke
  (siehe control_obstacle_node.md für Details).
- nachlauf_secs : Halten nach letzter Sichtung (zu kurz → kehrt zu früh zurück).
- evade_timeout_secs / wait_timeout_secs : Timeouts für den Sonderfall Anhalten.
- free_stable_frames : wie viele Frames der Korridor hintereinander frei sein muss,
  bevor EVADE/WAIT tatsächlich verlassen wird (gegen Flackern der Farberkennung).
- return_omega / return_threshold / return_stable_frames : Encoder-Rückkehr.

## Erkennungsprinzip
Farberkennung gelb/grün (HSV) im **unverzerrten Originalbild** → kein Verschwinden
am BEV-Trapezrand, keine Höhen-Verzerrung durch die Perspektiv-Transformation.
Nur der Bodenkontaktpunkt jeder erkannten Box wird per Homographie ins BEV
projiziert. Fängt gelbe UND grüne Bonus-Enten sowie die gelbe Mittellinie (bewusst
nicht unterschieden). Das Korridor-Lückenprofil bestimmt, wo im (schmalen,
bot-breiten) Fahrkorridor noch Platz ist; control_obstacle_node weicht zur Mitte
der breitesten freien Lücke aus (Fallback: einfache Rechts/Links-Heuristik nach
Entenposition, falls kein Profil vorliegt oder der Korridor komplett belegt ist).

## Vorzeichen-Check
Negativer Offset = Ausweichen nach links (Annahme). Weicht der Bot falsch herum
aus → Vorzeichen in control_obstacle_node._offset_from_gap bzw.
._determine_direction (Fallback-Zweig) prüfen.
