# Challenge 3 – Watch out for Ducks

Aufbauend auf eurer Lane-Following-Basis. Die Entenerkennung ist DIREKT in
detect_lane_node integriert (ein Bild-Decode/Warp → spart Latenz). Das Ausweichen
liegt als separate control_obstacle_node vor.

## Architektur
- detect_lane_node.py     → Spur + rote Haltelinie + ENTENERKENNUNG (Belegungsprofil)
  Published: /detect/lane, /detect/stop_line, /detect/duck, /detect/duck_occupancy,
             /debug/duck_bev (+ bestehende Debug-Bilder)
- control_lane_node.py    → PID-Fahrt + Stop-Automat, addiert /obstacle/error_offset
- control_obstacle_node.py→ Kombi-Ausweichstrategie auf dem Belegungsprofil
- switch_control_node.py  → Lane/Obstacle-Umschaltung
- camera_dashboard_node.py→ schlankes 2x2-Debug-Dashboard
- configuration_node.py / util.py 


## Start
    export VEHICLE_NAME=<botname>
    cd src
    ./ducks.sh

## Schichtweise testen (Testtag!)
STUFE 1 – Fährt der Bot + rote Linie?
  detect_lane_node.json:      duck.enabled = 0     (keine Enten-Trigger)
  → reines Lane-Following + Halt an roter Linie.

STUFE 2 – Enten richtig erkannt? (noch ohne Ausweichen)
  duck.enabled = 1
  control_obstacle_node.json: evade.active = 0     (Erkennung an, Ausweichen aus)
  → Dashboard /debug/duck_bev: rote Boxen auf Enten? Belegungsbalken korrekt?
    Mittellinie NICHT als Ente? Hier brightness/min_area justieren.

STUFE 3 – Ausweichen
  evade.active = 1

## Erkennung kalibrieren (in detect_lane_node.json, Gruppe "duck")
- use_otsu = 1 : Helligkeitsschwelle automatisch (empfohlen). Sonst use_otsu=0
  und brightness_threshold manuell.
- min_area / min_w / min_h : Mindestgröße eines Enten-Blobs.
- line_max_aspect : höher → mehr linienartige Strukturen verworfen (gegen
  Mittellinien-Reste). Zu hoch verwirft schmale Enten.
- roi_top / roi_bottom : vertikaler Auswertebereich im BEV.

## Ausweichen kalibrieren (control_obstacle_node.json, Gruppe "evade")
- evade_offset / oncoming_offset : Ausweichstärke normal / Gegenspur.
- gap_min_bins / edge_min_bins : Mindestbreite innere Lücke / freier Rand (von 40).
- evade_hold : Halten nach letzter Sichtung (zu kurz → kehrt zu früh zurück).
- ramp_step : Auf-/Abbau-Tempo des Offsets.

## Erkennungsprinzip
Farb-robust über Helligkeit (Enten heben sich vom dunklen Boden ab) → fängt
gelbe UND andersfarbige Bonus-Enten. Belegungsprofil über die Fahrbahnbreite:
Enten nebeneinander blockieren verschiedene x-Spalten, hintereinander dieselben.
control_obstacle wählt: freier Rand umfahren → sonst breiteste Lücke → sonst Gegenspur.

## Offen (NICHT in diesem Code gelöst)
Auf dem Wendeplatz fehlt die gelbe Mittellinie → die Spurführung hat dort keine
gelbe Referenz. Dieser Code ändert die Spurführung nicht, nur das Ausweichen.

## Vorzeichen-Check
Negativer Offset = Ausweichen nach links (Annahme). Weicht der Bot falsch herum
aus → Vorzeichen in control_obstacle_node._decide_from_occupancy drehen.
