# Challenge 3 – Watch out for Ducks

Aufbauend auf eurer Lane-Following-Basis. Die Entenerkennung ist DIREKT in
detect_lane_node integriert (ein Bild-Decode/Warp → spart Latenz). Das Ausweichen
liegt als separate control_obstacle_node vor.

## Architektur
- detect_lane_node.py     → Spur (weiße Linie) + rote Haltelinie + Zonen- und
  Entenerkennung + Korridor-Lückenprofil
  Published: /detect/lane, /detect/stop_line, /detect/duck, /detect/zones,
             /detect/corridor_occupancy, /debug/duck_bev (+ bestehende Debug-Bilder)
- control_lane_node.py    → PID-Fahrt + Stop-Automat, addiert /obstacle/error_offset
- control_obstacle_node.py→ Zustandsautomat (IDLE→EVADE→[WAIT]→PASS→RETURN),
  Ausweichrichtung + -stärke aus dem Korridor-Lückenprofil
- switch_control_node.py  → Lane/Obstacle-Umschaltung
- camera_dashboard_node.py→ schlankes 2x2-Debug-Dashboard
- configuration_node.py / util.py

Details zu Architektur und Zustandsautomat: siehe `CHALLENGE3_DOKU.md`.

## Start
    export VEHICLE_NAME=<botname>
    cd src
    ./ducks.sh

## Schichtweise testen (Testtag!)
STUFE 1 – Fährt der Bot + rote Linie?
  detect_lane_node.json:      duck.enabled = 0     (keine Enten-Trigger)
  → reines Lane-Following (weiße Linie) + Halt an roter Linie.

STUFE 2 – Enten/Zonen richtig erkannt? (noch ohne Ausweichen)
  duck.enabled = 1
  control_obstacle_node.json: evade.active = 0     (Erkennung an, Ausweichen aus)
  → Dashboard /debug/duck_bev: rote Boxen auf Enten? Zonen (nah/mittel/fern)
    korrekt eingefärbt? Lücken-Balken am unteren Korridorrand plausibel?
    Mittellinie NICHT als Ente-Blob, aber als Zonen-Hindernis? Hier
    brightness/min_area justieren.

STUFE 3 – Ausweichen
  evade.active = 1

## Erkennung kalibrieren (in detect_lane_node.json, Gruppe "duck")
- use_otsu = 1 : Helligkeitsschwelle automatisch (empfohlen). Sonst use_otsu=0
  und brightness_threshold manuell.
- min_area / min_w / min_h : Mindestgröße eines Enten-Blobs.
- line_max_aspect : höher → mehr linienartige Strukturen verworfen (gegen
  Mittellinien-Reste im Enten-Blob). Zu hoch verwirft schmale Enten.
- roi_top / roi_bottom : vertikaler Auswertebereich im BEV.

## Zonen & Lückenprofil kalibrieren (detect_lane_node.json, Gruppe "zones")
- corridor_x_min/max : Breite des überwachten Fahrkorridors.
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
- return_omega / return_threshold / return_stable_frames : Encoder-Rückkehr.

## Erkennungsprinzip
Farb-robust über Helligkeit (Enten heben sich vom dunklen Boden ab) → fängt
gelbe UND andersfarbige Bonus-Enten. Das Korridor-Lückenprofil bestimmt, wo im
Fahrkorridor noch Platz ist; control_obstacle_node weicht zur Mitte der
breitesten freien Lücke aus (Fallback: einfache Rechts/Links-Heuristik nach
Entenposition, falls kein Profil vorliegt oder der Korridor komplett belegt ist).

## Vorzeichen-Check
Negativer Offset = Ausweichen nach links (Annahme). Weicht der Bot falsch herum
aus → Vorzeichen in control_obstacle_node._offset_from_gap bzw.
._determine_direction (Fallback-Zweig) prüfen.
