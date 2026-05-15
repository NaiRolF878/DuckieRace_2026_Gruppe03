# Challenge 1 – Lane Following

> ROS 1 (Noetic) · Ubuntu 20.04 · Python 3 · OpenCV

Der Duckiebot erkennt die gelbe (links) und weiße (rechts) Spurlinie, folgt der Spur per PID-Regler und hält an roten Haltelinien für 3 Sekunden an.

---

## Inhaltsverzeichnis

- [Dateien](#dateien)
- [Systemüberblick](#systemüberblick)
- [Nodes](#nodes)
- [Konfigurationsparameter](#konfigurationsparameter)
- [Bot-spezifische Parameter](#bot-spezifische-parameter)
- [Setup & Starten](#setup--starten)
- [Kalibrierung](#kalibrierung)
- [Bekannte Probleme & Lösungen](#bekannte-probleme--lösungen)

---

## Dateien

| Datei | Typ | Beschreibung |
|---|---|---|
| `detect_lane_node.py` | Node | Spurerkennung, CLAHE, rote Linie, Spatial Filter, Frame-Tracking |
| `control_lane_node.py` | Node | PID-Regler, StopState-Automat, MIN_VEL |
| `switch_control_node.py` | Node | Steuerungsmodus-Umschalter (Lane / Obstacle / Intersection) |
| `configuration_node.py` | Node | Live-Kalibrierungs-GUI mit JSON-Persistenz |
| `camera_dashboard_node.py` | Node | 2×2 Kamera-Dashboard (Original, Bird's-Eye-View, Gelb, Weiß) |
| `util.py` | Hilfsfunktionen | Parameter laden, mergen, live updaten |
| `detect_lane_node.json` | Config | HSV-Parameter, Perspektive, rote Linie, Gegenspurfilter |
| `control_lane_node.json` | Config | PID, MIN/MAX_VEL, Stop/Cooldown-Dauer |

---

## Systemüberblick

```
Kamera (/camera_node/image/compressed)
    │
    ▼
┌──────────────────────┐     ┌──────────────────────┐
│  detect_lane_node    │     │  configuration_node  │
│                      │     │  (Tkinter GUI)       │
│  Bird's-Eye-View     │     │                      │
│  CLAHE               │◀────│  Schieberegler aus   │
│  HSV-Masken          │     │  *.json gebaut       │
│  Morphologie         │     └──────────────────────┘
│  Spatial Filter      │          /update_parameters
│  Frame-Tracking      │
│  Rote Linie (2x ROI) │
└──────┬───────────────┘
       │ /detect/lane           (Float64) Spurversatz [-1,+1]
       │ /detect/stop_line      (Bool)    Rote Linie erkannt
       │ /detect/stop_line_side (String)  Seite der roten Linie
       ▼
┌──────────────────────┐     ┌──────────────────────┐
│  control_lane_node   │◀────│  switch_control_node │
│                      │     │                      │
│  PID-Regler          │     │  Lane=1              │
│  StopState-Automat   │     │  Obstacle=2          │
│  MIN_VEL             │     │  Intersection=3      │
└──────┬───────────────┘     └──────────────────────┘
       │ /car_cmd_switch_node/cmd (Twist2DStamped)
       ▼
    Motoren

┌──────────────────────┐
│ camera_dashboard_node│  → cv2.imshow (2×2 Grid)
│  oben links:  Original        │
│  oben rechts: Bird's-Eye-View │
│  unten links: Gelb-Maske      │
│  unten rechts: Weiß-Maske     │
└──────────────────────┘
```

---

## Nodes

### detect\_lane\_node

Verarbeitet jeden Kameraframe und erkennt Spurlinien sowie die rote Haltelinie.

**Pipeline pro Frame:**

| Schritt | Was | Warum |
|---|---|---|
| Bird's-Eye-View | Perspektivtransformation | Spurlinien werden parallel → einfachere Erkennung |
| CLAHE | Lokaler Helligkeitsausgleich im LAB-Farbraum | Schatten und Lichtunterschiede kompensieren ohne HSV-Kalibrierung zu verschieben |
| HSV-Masken | `cv2.inRange` für Gelb und Weiß | HSV stabiler als BGR für Farbsegmentierung |
| Morphologie | `MORPH_CLOSE` | Lücken durch Schatten in Masken schließen |
| Spatial Filter | Weiß-Maske links von `center_yellow + min_lane_width` ausblenden | Gegenspur-Weiß in engen Kurven ignorieren |
| Frame-Tracking | Sprung > `max_frame_jump` → letzten Wert behalten | Einzelne Fehlmessungen abfangen |
| Rote Linie (eigen) | Vertikale + horizontale ROI auf Bird's-Eye-View | Nur eigene Spur, nur direkt vor Bot |
| Rote Linie (Seite) | Linke/rechte Bildhälfte auf Originalbild | Gegenspur-Linie für Kreuzungsnavigation erkennen |

**Warum zwei HSV-Masken für Rot?**
Rot liegt im HSV-Farbraum an zwei Stellen: Hue 0–10 (orangerot) und Hue 160–179 (blaurot). Nur mit beiden Masken werden alle Rottöne erfasst.

**Publiziert:**

| Topic | Typ | Beschreibung |
|---|---|---|
| `/detect/lane` | `Float64` | Spurversatz: `0` = mittig, `+1` = ganz links, `-1` = ganz rechts |
| `/detect/stop_line` | `Bool` | `True` wenn eigene Haltelinie im ROI erkannt |
| `/detect/stop_line_side` | `String` | `none`, `left`, `right`, `both` – Seite der roten Linie |
| `/debug/original` | `CompressedImage` | Rohes Kamerabild |
| `/debug/bird_view` | `CompressedImage` | Bird's-Eye-View |
| `/debug/lane_white` | `CompressedImage` | Weiß-Maske |
| `/debug/lane_yellow` | `CompressedImage` | Gelb-Maske |
| `/debug/lane_red` | `CompressedImage` | Rot-Maske |

---

### control\_lane\_node

**PID-Formel:**
```
P = kp * error
I = ki * Σ(error * dt)      # I-Anteil wird beim Stopp zurückgesetzt
D = kd * (error - lastError) / dt

omega = P + I + D            # begrenzt auf [-3, +3]
v     = max(MIN_VEL, MAX_VEL * (1 - |error|))
```

**Warum `v = max(MIN_VEL, MAX_VEL * (1 - |error|))`?**
In Kurven (großer Fehler) fährt der Bot langsamer – sicherer und stabiler. `MIN_VEL` verhindert dass der Bot bei maximalem Fehler komplett stoppt.

**StopState-Zustandsautomat:**

```
Driving ──(rote Linie erkannt)──▶ Stopping (v=0, omega=0)
                                       │ 3s abgelaufen
                                       ▼
Driving ◀──(Cooldown abgelaufen)── Cooldown
                                   (weiterfahren, neue Linien ignorieren)
```

**Warum Integral beim Stopp zurücksetzen?**
Während des Stopps akkumuliert der I-Anteil Fehler → beim Anfahren würde das einen Lenkruck erzeugen.

---

### switch\_control\_node

Publiziert den aktiven Modus kontinuierlich mit 10 Hz:

| Wert | Modus | Aktive Node |
|---|---|---|
| `1` | `Lane` | `control_lane_node` |
| `2` | `Obstacle` | `control_obstacle_node` |
| `3` | `Intersection` | `control_intersection_node` |

---

### configuration\_node

Vollständig datengetrieben – neue Parameter in der JSON erscheinen automatisch als Schieberegler.

**Warum bot-spezifisch speichern?**
`save_parameters()` erkennt ob die JSON die neue Struktur (`default` + Bot-Name) hat:
- Neue Struktur → nur den bot-spezifischen Block überschreiben, andere Bots bleiben unberührt
- Alte Struktur → direkt überschreiben (Rückwärtskompatibilität)

---

### camera\_dashboard\_node

Zeigt alle 4 Kameraansichten in einem 2×2 Grid:

```
┌──────────────┬──────────────┐
│  Original    │  Bird's-Eye  │
├──────────────┼──────────────┤
│  Gelb-Maske  │  Weiß-Maske  │
└──────────────┴──────────────┘
```

Mit `q` schließen.

---

### util.py

**Parameter-Merge-Logik:**
```
JSON (neue Struktur):
  default    → Basiswerte für alle Bots
  dorette    → Überschreibungen nur für dorette

Beim Start:
  merged = deep_merge(default, dorette)
  → dorette-Werte überschreiben default
  → nicht genannte Parameter kommen aus default
```

**Bug-Fix gegenüber Original:**
Der Callback wurde im Original für alle Nodes aufgerufen, auch wenn die Message für eine andere Node bestimmt war. Korrigiert: `if msg['node'] == node_name`.

---

## Konfigurationsparameter

### detect\_lane\_node.json

#### `crop_image` – Perspektivtransformation

| Parameter | Default | Beschreibung |
|---|---|---|
| `top_left_x/y` | 159 / 218 | Obere linke Ecke der Fahrspur im Kamerabild |
| `top_right_x/y` | 441 / 218 | Obere rechte Ecke |
| `bottom_left_x/y` | 606 / 382 | Untere linke Ecke |
| `bottom_right_x/y` | -29 / 382 | Untere rechte Ecke |

#### `white` – Weiße Linie (HSV) + Gegenspurfilter

| Parameter | Default | Beschreibung |
|---|---|---|
| `hl` / `hh` | 0 / 255 | Hue Unter- / Obergrenze |
| `sl` / `sh` | 0 / 41 | Saturation Unter- / Obergrenze |
| `vl` / `vh` | 161 / 255 | Value Unter- / Obergrenze |
| `min_lane_width` | 50 px | Mindestabstand gelb→weiß – blendet Gegenspur aus |
| `max_frame_jump` | 80 px | Max. Pixelsprung zwischen Frames |

#### `yellow` – Gelbe Linie (HSV)

| Parameter | Default | Beschreibung |
|---|---|---|
| `hl` / `hh` | 15 / 60 | Hue Unter- / Obergrenze |
| `sl` / `sh` | 60 / 255 | Saturation Unter- / Obergrenze |
| `vl` / `vh` | 120 / 255 | Value Unter- / Obergrenze |

#### `red` – Rote Haltelinie

| Parameter | Default | Beschreibung |
|---|---|---|
| `hl` / `hh` | 0 / 10 | Hue unterer Rot-Bereich |
| `hl2` / `hh2` | 160 / 179 | Hue oberer Rot-Bereich |
| `sl` / `sh` | 100 / 255 | Saturation Unter- / Obergrenze |
| `vl` / `vh` | 100 / 255 | Value Unter- / Obergrenze |
| `pixel_threshold` | 500 | Mindestanzahl roter Pixel |
| `detection_zone` | 0.85 | Vertikale ROI: unterste 15% prüfen |
| `detection_x_start` | 0.4 | Horizontale ROI: rechte 60% prüfen |

### control\_lane\_node.json

#### `pid` – PID-Regler

| Parameter | Default | Bot-spezifisch? | Beschreibung |
|---|---|---|---|
| `p` | 8.0 | ✅ | Proportionalbeiwert |
| `i` | 0.0 | ✅ | Integralbeiwert |
| `d` | 6.0 | ✅ | Differentialbeiwert |
| `max_vel` | 0.5 m/s | ❌ | Maximalgeschwindigkeit |
| `min_vel` | 0.1 m/s | ❌ | Minimalgeschwindigkeit |

#### `stop_line` – Haltelinien-Logik

| Parameter | Default | Beschreibung |
|---|---|---|
| `stop_duration` | 3.0 s | Wartezeit an der roten Linie |
| `cooldown_duration` | 3.0 s | Sperrzeit nach dem Stopp |

---

## Bot-spezifische Parameter

Jeder Bot hat eigene Einträge in `detect_lane_node.json` (HSV-Werte) und `control_lane_node.json` (PID). Beim Start liest `util.py` den `VEHICLE_NAME` und merged automatisch:

**Verfügbare Bots:** `donald`, `daisy`, `tick`, `tack`, `trick`, `gustav`, `dorette`, `dagobert`, `daffy`, `gundel`

**Workflow beim Kalibrieren eines neuen Bots:**
1. `export VEHICLE_NAME=dorette` setzen
2. Nodes starten → default-Parameter werden geladen
3. Im `configuration_node` kalibrieren
4. Schieberegler bewegen → Werte werden automatisch unter `dorette` in der JSON gespeichert
5. Beim nächsten Start mit demselben Bot werden die kalibrierten Werte geladen

---

## Setup & Starten

```bash
# ROS-Umgebung + Bot setzen (Beispiel: dorette)
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://dorette.local:11311
export VEHICLE_NAME=dorette

# Nodes starten (je ein Terminal)
python3 src/detect_lane_node.py
python3 src/control_lane_node.py
python3 src/switch_control_node.py
python3 src/configuration_node.py    # Kalibrierungs-GUI
python3 src/camera_dashboard_node.py # 2x2 Debug-Ansicht
```

---

## Kalibrierung

### 1. Perspektivtransformation

1. `configuration_node` → Node `detect_lane_node` → Gruppe `crop_image`
2. Debug Image `/debug/bird_view` auswählen
3. Eckpunkte so einstellen dass die Fahrspur im transformierten Bild als Rechteck erscheint

### 2. HSV-Farbbereiche

1. Debug Image `/debug/lane_white` oder `/debug/lane_yellow`
2. `vl` hochschieben bis Hintergrund verschwindet
3. `sh` runterschieben bis Linie vollständig erkannt wird
4. Werte werden automatisch bot-spezifisch gespeichert

### 3. PID kalibrieren

1. `i = 0` lassen
2. `p` erhöhen bis Bot der Spur folgt
3. `d` erhöhen bis Schwingen aufhört
4. `i` nur bei dauerhaftem Versatz leicht erhöhen

### 4. Haltelinie kalibrieren

1. `pixel_threshold` erhöhen bis keine Fehlalarme auf gerader Strecke
2. `detection_zone` anpassen (Richtung 1.0 = Bot hält später an)
3. `detection_x_start` erhöhen wenn Gegenspur-Haltelinie auslöst

---

## Bekannte Probleme & Lösungen

| Problem | Ursache | Lösung |
|---|---|---|
| Falsche weiße Linie in engen Kurven | Gegenspur nah an eigener Spur | `min_lane_width` erhöhen |
| Weiße Linie springt bei Lichtreflexen | Fehlmessungen einzelner Frames | `max_frame_jump` verringern |
| Gegenspur-Haltelinie löst Stopp aus | Horizontale ROI zu breit | `detection_x_start` erhöhen |
| Bot hält zu früh an | Vertikale ROI zu weit oben | `detection_zone` erhöhen |
| Bot fährt nach Neustart nicht | `KeyError` in `cbUpdateParameters` | Alle `.json`-Dateien auf Bot übertragen |
| HSV-Kalibrierung nach Botwechsel falsch | Anderer Bot, andere Kamera | Bot-spezifische Werte im `configuration_node` kalibrieren und speichern |
