# Challenge 2 – Intersection Handling

> Baut auf Challenge 1 auf · ROS 1 (Noetic) · Python 3 · OpenCV · pupil-apriltags

Der Duckiebot erkennt Kreuzungen über rote Haltelinie + AprilTag, wählt zufällig eine erlaubte Abbiegerichtung und navigiert durch die Kreuzung mit einer kombinierten Strategie aus Gegenspur-Orientierung und Lane Handover.

---

## Inhaltsverzeichnis

- [Dateien](#dateien)
- [Systemüberblick](#systemüberblick)
- [Nodes](#nodes)
- [Konfigurationsparameter](#konfigurationsparameter)
- [Setup & Starten](#setup--starten)
- [Kalibrierung](#kalibrierung)
- [Bekannte Probleme & Lösungen](#bekannte-probleme--lösungen)

---

## Dateien

| Datei | Typ | Beschreibung |
|---|---|---|
| `detect_apriltag_node.py` | Node | AprilTag-Erkennung auf Originalbild, Bounding-Box Debug |
| `control_intersection_node.py` | Node | Kreuzungsnavigation: Approaching, Turning, Lane Handover |
| `switch_control_node.py` | Node | Erweitert um `Intersection`-Modus (Wert 3) |
| `detect_lane_node.py` | Node | Erweitert um Seitenerkennung der roten Linie |
| `detect_apriltag_node.json` | Config | Tag-Erkennungsparameter, Kamera-Intrinsics |
| `control_intersection_node.json` | Config | Abbiege-Parameter, Handover-Schwellwerte, Tag-Richtungen |

**Abhängigkeit:** Alle Nodes aus Challenge 1 werden weiterhin benötigt.

---

## Systemüberblick

```
Kamera (Originalbild)
    │
    ├──▶ detect_apriltag_node → /detect/apriltag  (Int32, Tag-ID)
    │                         → /debug/apriltag   (Bounding-Box)
    │
    └──▶ detect_lane_node     → /detect/stop_line      (Bool)
                              → /detect/stop_line_side (String)
                              → /detect/lane            (Float64)

switch_control_node
    ◀── /detect/stop_line      (rote Linie erkannt?)
    ◀── /detect/apriltag       (AprilTag sichtbar?)
    ◀── /intersection/done     (Kreuzung abgeschlossen?)
    ──▶ /switch/control        (1=Lane, 2=Obstacle, 3=Intersection)

control_intersection_node
    ◀── /detect/apriltag       (Richtung nachschlagen)
    ◀── /detect/lane           (Lane Handover erkennen)
    ◀── /detect/stop_line      (Approaching-Phase)
    ◀── /detect/stop_line_side (Turning-Phase: Orientierung)
    ◀── /switch/control        (aktiviert/deaktiviert)
    ──▶ /car_cmd_switch_node/cmd
    ──▶ /intersection/done
```

---

## Nodes

### detect\_apriltag\_node

Erkennt AprilTags auf dem **Originalbild** (nicht Bird's-Eye-View – Perspektivtransformation würde Tags verzerren).

**Warum `pupil-apriltags`?**
Schneller als die Standard-`apriltag`-Library, gleiche API, aktiv gewartet. Kein Docker oder ROS-Package nötig.

```bash
pip install pupil-apriltags
```

**Bei mehreren sichtbaren Tags:** Der größte Tag (= nächster Tag) gewinnt. Kleinere Tags unter `min_tag_area` werden ignoriert.

**Debug-Bild:**
- Grüne Bounding-Box = erkannter und verwendeter Tag
- Gelbe Bounding-Box = sichtbar aber zu klein/nicht verwendet
- Label mit Tag-ID eingeblendet

**Publiziert:**

| Topic | Typ | Beschreibung |
|---|---|---|
| `/detect/apriltag` | `Int32` | Tag-ID (`-1` = kein Tag sichtbar) |
| `/debug/apriltag` | `CompressedImage` | Kamerabild mit Bounding-Box |

---

### detect\_lane\_node (Erweiterung)

Zusätzlich zur Challenge-1-Funktionalität:

**Neue Methode `detect_stop_line(hsv, cv_image)`** – zwei getrennte Pipelines:

| Pipeline | Bild | Zweck |
|---|---|---|
| Eigene Haltelinie | Bird's-Eye-View | Erkennt wann Bot stoppen soll |
| Seitenerkennung | Originalbild | Erkennt auf welcher Seite Gegenspur-Linie liegt |

**Warum Originalbild für Seitenerkennung?**
Bird's-Eye-View ist auf die eigene Spur kalibriert – die Gegenspur liegt am Rand oder außerhalb des transformierten Bereichs. Im Originalbild ist die gesamte Kreuzung inkl. Gegenspur sichtbar.

**Seitenwerte:**

| Wert | Bedeutung |
|---|---|
| `none` | Keine rote Linie sichtbar |
| `left` | Rote Linie auf linker Bildseite (Gegenspur) |
| `right` | Rote Linie auf rechter Bildseite (eigene Spur) |
| `both` | Bot steht direkt auf/vor der Linie |

**Neu publiziert:**

| Topic | Typ | Beschreibung |
|---|---|---|
| `/detect/stop_line_side` | `String` | Seite der roten Linie |

---

### switch\_control\_node (Erweiterung)

**Kreuzungserkennung:** Kreuzung wird nur ausgelöst wenn **rote Linie UND AprilTag gleichzeitig** sichtbar sind – verhindert Fehlauslösungen durch rote Linie alleine (z.B. Gegenspur-Haltelinie).

**Neuer Zustand `Intersection` (Wert 3):**

```
Lane ──(stop_line=True AND apriltag≠-1)──▶ Intersection
                                               │
                                    control_intersection_node
                                    navigiert durch Kreuzung
                                               │
                                    /intersection/done=True
                                               │
Lane ◀─────────────────────────────────────────┘
```

---

### control\_intersection\_node

Navigiert den Bot durch die Kreuzung in vier Phasen:

```
Idle
  │ switch_control aktiviert
  ▼
Phase 1 – Approaching
  Bot fährt vorwärts bis eigene rote Linie verschwindet
  + extra_duration Sicherheitspuffer (Bot steht sicher auf Kreuzung)
  Sicherheitsnetz: approach_timeout
  │
  ▼
Phase 2 – Turning
  Bot dreht in gewählte Richtung
  Orientierung über rote Gegenspur-Linie (Originalbild):
    Links  → drehe bis stop_line_side == 'right'
    Rechts → drehe bis stop_line_side == 'left'
    Gerade → fahre bis stop_line_side == 'none'
  Gleichzeitig: Lane Handover wenn Spur stabil erkannt
  Sicherheitsnetz: turn_timeout
  │
  ▼
Done
  /intersection/done → switch_control_node schaltet zurück auf Lane
```

**Warum rote Gegenspur-Linie als Orientierung?**
Kein Schlupf-Problem wie bei reiner Odometrie. Funktioniert unabhängig von Kreuzungsgröße und Bodenbelag. Passt sich automatisch an T-Kreuzungen und 4-Wege-Kreuzungen an.

**Option C – Lane Handover:**
- Spur stabil für `stable_frames` aufeinanderfolgende Frames → sofortiger Handover
- Timeout als Sicherheitsnetz falls Spur nicht gefunden wird

---

## Konfigurationsparameter

### detect\_apriltag\_node.json

| Parameter | Default | Beschreibung |
|---|---|---|
| `min_tag_area` | 1000 px² | Mindestfläche des Tags – zu kleine Tags werden ignoriert |
| `camera_fx/fy` | 320 | Kamera-Brennweite (für optionale Positionsschätzung) |
| `camera_cx/cy` | 320 / 240 | Kamera-Hauptpunkt |

### control\_intersection\_node.json

#### `approach` – Vorfahrtsphase

| Parameter | Default | Beschreibung |
|---|---|---|
| `v` | 0.2 m/s | Geschwindigkeit beim Vorwärtsfahren |
| `extra_duration` | 0.5 s | Zeit nach Verschwinden der Linie bis zum Abbiegen |
| `timeout` | 4.0 s | Sicherheits-Timeout falls Linie nie verschwindet |

#### `left` / `right` / `straight` – Abbiegen

| Parameter | Default | Beschreibung |
|---|---|---|
| `v` | 0.2 m/s | Geschwindigkeit beim Abbiegen |
| `omega` | ±2.0 rad/s | Lenkwinkel (positiv = links, negativ = rechts) |

#### `turning` – Turning-Phase

| Parameter | Default | Beschreibung |
|---|---|---|
| `timeout` | 6.0 s | Sicherheits-Timeout für die gesamte Drehphase |

#### `handover` – Lane Handover

| Parameter | Default | Beschreibung |
|---|---|---|
| `lane_threshold` | 0.4 | Max. Spurversatz für "stabile Spur" |
| `stable_frames` | 5 | Anzahl stabiler Frames für Handover |
| `timeout` | 3.0 s | Sicherheits-Timeout für Handover-Phase |

#### `tag_directions` – Tag-ID → Richtungen

```json
"tag_directions": {
    "1": {"default": "left,straight"},
    "2": {"default": "right,straight"},
    "3": {"default": "left,right,straight"},
    "4": {"default": "left,right"}
}
```

Wird vom Dozenten festgelegt – nur hier anpassen, kein Code nötig.

---

## Setup & Starten

```bash
# Zusätzlich zu Challenge 1:
pip install pupil-apriltags

# Nodes starten (zusätzlich zu Challenge 1)
python3 src/detect_apriltag_node.py
python3 src/control_intersection_node.py
```

`switch_control_node.py` und `detect_lane_node.py` aus Challenge 1 ersetzen – neue Versionen verwenden.

---

## Kalibrierung

### AprilTag-Erkennung kalibrieren

1. `configuration_node` → Node `detect_apriltag_node` → Gruppe `detection`
2. Debug Image `/debug/apriltag` auswählen
3. `min_tag_area` erhöhen bis keine Fehlerkennungen aus der Ferne

### Abbiege-Parameter kalibrieren

1. `configuration_node` → Node `control_intersection_node`
2. Gruppe `approach`: `v` und `extra_duration` so einstellen dass Bot mittig auf Kreuzung steht
3. Gruppe `left`/`right`/`straight`: `omega` und `v` so einstellen dass Drehung sauber ist
4. Gruppe `handover`: `lane_threshold` und `stable_frames` anpassen

### Tag-IDs eintragen

Sobald Tag-IDs vom Dozenten bekannt sind in `control_intersection_node.json` unter `tag_directions` eintragen.

---

## Bekannte Probleme & Lösungen

| Problem | Ursache | Lösung |
|---|---|---|
| Kreuzung wird nicht erkannt | Nur rote Linie oder nur AprilTag sichtbar | Beide müssen gleichzeitig erkannt werden |
| Bot dreht zu weit / zu wenig | `omega` oder `v` falsch kalibriert | Abbiege-Parameter im `configuration_node` anpassen |
| Lane Handover zu früh | `lane_threshold` zu groß | Schwellwert verringern |
| Bot bleibt auf Kreuzung stehen | Spur nicht gefunden + Timeout | `turn_timeout` erhöhen oder Kalibrierung der Spurerkennung prüfen |
| Falsche Richtung abgebogen | Tag-ID falsch gemappt | `tag_directions` in JSON prüfen |
