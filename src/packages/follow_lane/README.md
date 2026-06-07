# follow_lane – Challenge 1: Lane Following

ROS 1 (Noetic) Paket für autonomes Spurfolgen auf einem Duckiebot. Der Bot
folgt der Fahrspur (gelbe Mittellinie links, weiße Außenlinie rechts) und hält
an roten Haltelinien für 3 Sekunden an.

Getestet unter Ubuntu 20.04 mit ROS Noetic.

---

## Inhaltsverzeichnis

1. [Überblick](#überblick)
2. [Paketstruktur](#paketstruktur)
3. [Voraussetzungen](#voraussetzungen)
4. [Start](#start)
5. [Architektur & Datenfluss](#architektur--datenfluss)
6. [Nodes im Detail](#nodes-im-detail)
7. [ROS-Topics](#ros-topics)
8. [Kalibrierung & Parameter-Tuning](#kalibrierung--parameter-tuning)
9. [Troubleshooting](#troubleshooting)

---

## Überblick

Das Paket besteht aus vier produktiven Nodes plus zwei Hilfs-Nodes für
Konfiguration und Visualisierung. Der Kern-Datenfluss ist:

```
Kamera → detect_lane_node → /detect/lane → control_lane_node → Fahrbefehl
```

`detect_lane_node` analysiert das Kamerabild und berechnet den Spurversatz
(ein Wert zwischen -1 und +1). `control_lane_node` regelt daraus per PID die
Lenkung. `switch_control_node` aktiviert den Lane-Controller. Die roten
Haltelinien werden ebenfalls von `detect_lane_node` erkannt und über einen
Zustandsautomaten in `control_lane_node` verarbeitet.

---

## Paketstruktur

```
follow_lane/
├── follow_lane.sh                    # Bash-Launcher
├── src/
│   ├── detect_lane_node.py           # Spurerkennung (Kern)
│   ├── control_lane_node.py          # PID-Regelung + Haltelinien-Automat
│   ├── switch_control_node.py        # Aktiviert den Lane-Controller
│   ├── camera_dashboard_node.py      # Visualisierung (2×2 Dashboard)
│   ├── configuration_node.py         # GUI für Live-Parameter-Tuning
│   └── util.py                       # Parameter-Laden + Bot-spezifisches Merging
├── config/
│   ├── detect_lane_node.json         # HSV, BEV-Eckpunkte, Haltelinien-ROI
│   └── control_lane_node.json        # PID-Werte, Stopp-Zeiten
└── launch/
    └── follow_lane.launch            # ROS-Launch-Datei
```

---

## Voraussetzungen

- ROS Noetic, Catkin-Workspace mit gebautem `follow_lane`-Paket
- Laufender `roscore` (auf dem Duckiebot bereits vorhanden – wird **nicht**
  vom Launcher gestartet)
- Laufender Kamera-Treiber, der auf
  `/<VEHICLE_NAME>/camera_node/image/compressed` published
- Umgebungsvariable `VEHICLE_NAME` gesetzt (z.B. `dorette`):

```bash
export VEHICLE_NAME=dorette
```

Alle Nodes lesen den Fahrzeugnamen aus dieser Variable. Ohne sie brechen die
Nodes beim Start mit einem `KeyError` ab.

---

## Start

### Variante A – roslaunch (empfohlen)

```bash
roslaunch follow_lane follow_lane.launch
```

`roslaunch` startet alle vier Nodes, bündelt die Logs und beendet bei `Strg+C`
automatisch alles sauber – inklusive des Shutdown-Handlers in
`control_lane_node`, der den Bot auf `v=0` setzt.

### Variante B – Bash-Script

```bash
./follow_lane.sh
```

Das Script prüft zuerst, ob der ROS-Master erreichbar ist, startet dann die
Nodes und fängt `Strg+C` ab, um alle Prozesse sauber zu beenden.

### Konfigurations-GUI (optional, separat)

Zum Live-Tuning der Parameter (siehe unten):

```bash
rosrun follow_lane configuration_node.py
```

---

## Architektur & Datenfluss

```
                  /camera_node/image/compressed
                              │
                              ▼
                   ┌──────────────────────┐
                   │   detect_lane_node    │
                   │  ───────────────────  │
                   │  BEV-Transformation   │
                   │  CLAHE (LAB)          │
                   │  HSV-Masken gelb/weiß │
                   │  Sobel-Kanten         │
                   │  Frame-Tracking       │
                   └──────────┬────────────┘
                              │
              ┌───────────────┼────────────────────┐
              │               │                    │
        /detect/lane    /detect/stop_line     /debug/* (Bilder)
        (Float64)         (Bool)                    │
              │               │                    ▼
              │               │           ┌──────────────────────┐
              │               │           │ camera_dashboard_node │
              │               │           └──────────────────────┘
              ▼               ▼
       ┌────────────────────────────┐         ┌─────────────────────┐
       │     control_lane_node      │◄────────│ switch_control_node │
       │  ────────────────────────  │/enable/ │  (/enable/lane=True)│
       │  PID-Regelung              │  lane   └─────────────────────┘
       │  Haltelinien-Automat       │
       └─────────────┬──────────────┘
                     │
                     ▼
       /car_cmd_switch_node/cmd (Twist2DStamped)
```

Wichtig: Die Detection-Seite ist **zustandslos** (jeder Frame wird unabhängig
verarbeitet), die Control-Seite **hält Zustand** (PID-Integral,
Haltelinien-Automat). Diese Trennung ist bewusst so gewählt.

---

## Nodes im Detail

### detect_lane_node.py

Das Herzstück. Pipeline pro Kamerabild:

1. **Bird's-Eye-View** – Perspektivtransformation des Trapez-Spurausschnitts
   in eine Draufsicht (400×400). Eckpunkte sind kalibrierbar.
2. **CLAHE im LAB-Farbraum** – lokaler Helligkeitsausgleich nur auf dem
   L-Kanal, damit Schatten die HSV-Erkennung nicht stören und die
   Farbkalibrierung stabil bleibt.
3. **HSV-Masken** für Gelb und Weiß.
4. **Morphologie (MORPH_CLOSE)** – schließt Lücken in den Masken, die durch
   Schatten entstehen.
5. **Sobel-Kantenerkennung** – findet die Linienposition. Mit `last_known`
   wird jeweils die Kante gewählt, die dem letzten bekannten Wert am nächsten
   liegt (robuster in engen Kurven und am Wendeplatz, wo zwei weiße Kanten
   sichtbar sein können).
6. **Frame-Tracking** für beide Linien – Sprünge größer als `max_frame_jump`
   werden verworfen, der letzte gültige Wert beibehalten.
7. **Plausibilitätsprüfung** – Weiß muss rechts von Gelb liegen.
8. **Spurversatz** berechnen und auf `/detect/lane` publizieren.

Die Fallback-Logik steckt zentral in `_resolve_line_position`: Bei fehlender
Detektion wird der letzte bekannte Wert gehalten; nur beim allerersten Frame
ohne Anker greift der Bildrand-Fallback (ohne sich darauf festzulegen).

Die rote Haltelinie wird über einen zweidimensionalen ROI im BEV erkannt
(vertikal + horizontal einschränkbar) und als Bool publiziert.

### control_lane_node.py

PID-Regler plus Haltelinien-Zustandsautomat.

**PID:** Die Geschwindigkeit sinkt mit wachsendem Spurversatz
(`v = max(MIN_VEL, MAX_VEL * (1 - |error|))`) – der Bot fährt also in Kurven
langsamer. Der I-Anteil hat einen Anti-Windup-Clamp (`INTEGRAL_LIMIT = 3.0`),
damit er bei hohem `ki` nicht unbegrenzt aufläuft.

**Haltelinien-Automat:** `Driving → Stopping (3s) → Cooldown → Driving`. Im
Cooldown wird keine neue rote Linie erkannt, damit der Bot nach dem Anfahren
nicht sofort wieder stoppt. Beim Anfahren wird das PID-Integral
zurückgesetzt.

### switch_control_node.py

Minimaler Scaffold. Publiziert dauerhaft `/enable/lane = True` mit 10 Hz, womit
`control_lane_node` aktiv bleibt. Kann später erweitert werden, um zwischen
mehreren Modi (Intersection, Obstacle Avoidance) umzuschalten.

### camera_dashboard_node.py

Zeigt ein 2×2-Dashboard (600×600): Original mit Annotationen (Modus-Rahmen,
AprilTag- und Enten-Bounding-Boxen, Rote-Linie-Box, Statuszeile), Bird's-Eye-
View, Gelb-Maske, Weiß-Maske. Bewusst challenge-übergreifend gehalten – die
Subscriber für AprilTag/Ente bleiben funktionsfähig, auch wenn in Challenge 1
nichts darauf published.

### configuration_node.py

Tkinter-GUI zum Live-Tuning. Liest dynamisch alle JSON-Dateien aus dem
`config/`-Ordner, baut Dropdowns für Node + Parametergruppe und für jeden
Parameter einen Schieberegler. Änderungen werden sofort per ROS an die
laufenden Nodes gesendet **und** in die JSON zurückgeschrieben.

### util.py

Lädt Parameter und merged bot-spezifische Overrides über die `default`-Werte
(`_deep_merge`). Registriert den Live-Update-Callback und stellt sicher, dass
nur die richtige Node ihre Parameter-Updates erhält.

---

## ROS-Topics

Alle Topics sind mit `/<VEHICLE_NAME>/` präfixiert.

### Abonniert (Eingänge)

| Topic | Typ | Node | Zweck |
|---|---|---|---|
| `camera_node/image/compressed` | CompressedImage | detect_lane | Kamerabild |
| `detect/lane` | Float64 | control_lane | Spurversatz [-1, +1] |
| `detect/stop_line` | Bool | control_lane | Rote Linie erkannt |
| `enable/lane` | Bool | control_lane | Controller aktiv? |

### Publiziert (Ausgänge)

| Topic | Typ | Node | Zweck |
|---|---|---|---|
| `detect/lane` | Float64 | detect_lane | Spurversatz [-1, +1] |
| `detect/stop_line` | Bool | detect_lane | Rote Linie erkannt |
| `enable/lane` | Bool | switch_control | Aktiviert control_lane |
| `car_cmd_switch_node/cmd` | Twist2DStamped | control_lane | Fahrbefehl (v, omega) |

### Debug-Bilder (von detect_lane_node)

`debug/original`, `debug/bird_view`, `debug/annotated`, `debug/lane_croped`,
`debug/lane_white`, `debug/lane_yellow`, `debug/lane_red` – alle CompressedImage.
Werden nur publiziert, wenn ein Subscriber verbunden ist.

---

## Kalibrierung & Parameter-Tuning

Parameter liegen in `config/*.json`. Sie lassen sich **live** über die
`configuration_node`-GUI verändern (Schieberegler) – Änderungen wirken sofort
und werden gespeichert.

### Bot-spezifische Parameter

Die JSON-Struktur trennt `default` (für alle Bots) von bot-spezifischen
Blöcken (z.B. `dorette`, `daffy`). Beim Laden wird gemergt: bot-spezifische
Werte überschreiben die Defaults, nicht genannte Werte bleiben aus `default`.
So kann jeder Bot eigene HSV- und PID-Werte haben, ohne die anderen zu stören.
`save_parameters()` schreibt nur den bot-spezifischen Block zurück.

### detect_lane_node.json

| Gruppe | Parameter | Bedeutung |
|---|---|---|
| `crop_image` | top/bottom × left/right (x, y) | BEV-Trapez-Eckpunkte |
| `yellow` / `white` | hl, hh, sl, sh, vl, vh | HSV-Grenzen der Maske |
| `white` | `max_frame_jump` | max. Pixelsprung pro Frame (beide Linien) |
| `red` | hl, hh, hl2, hh2, sl, sh, vl, vh | HSV (Rot, zwei Hue-Bereiche) |
| `red` | `pixel_threshold` | Mindest-Pixelzahl für Erkennung |
| `red` | `detection_zone` | schneidet oben ab (0.85 = unterste 15%) |
| `red` | `detection_x_start` | schneidet links ab (0.4 = rechte 60%) |
| `red` | `detection_x_end` | schneidet rechts ab (1.0 = bis Rand) |

**Hinweis zu den Rot-Parametern:** Rot liegt an zwei Stellen des Hue-Kreises
(nahe 0 und nahe 180), deshalb zwei Bereiche (`hl/hh` und `hl2/hh2`), die zu
einer Maske vereint werden. Der ROI lässt sich in drei Richtungen
einschränken, um z.B. rote Markierungen am Wendeplatz auszublenden.

### control_lane_node.json

| Gruppe | Parameter | Bedeutung |
|---|---|---|
| `pid` | `p`, `i`, `d` | PID-Verstärkungen |
| `pid` | `max_vel`, `min_vel` | Geschwindigkeitsgrenzen |
| `stop_line` | `stop_duration` | Haltezeit an roter Linie (Sek.) |
| `stop_line` | `cooldown_duration` | Sperrzeit nach Anfahren (Sek.) |

### Tuning-Reihenfolge (Empfehlung)

1. **BEV-Eckpunkte** zuerst – ein falsch kalibrierter Bird's-Eye-View macht
   alles andere unbrauchbar. Im Dashboard die `bird_view`-Kachel prüfen.
2. **HSV-Masken** für Gelb und Weiß – im Dashboard die Masken-Kacheln nutzen,
   bis nur die Linien sichtbar sind und Rauschen weg ist.
3. **Rote Linie** – `pixel_threshold` und ROI so einstellen, dass nur die
   eigene Haltelinie auslöst.
4. **PID** zuletzt – mit `p` beginnen (Spurfolgen), dann `d` gegen
   Überschwingen. `i` meist 0 lassen; nur bei systematischem Versatz leicht
   erhöhen.

---

## Troubleshooting

**Nodes starten, aber nichts passiert / keine Messages**
ROS-Master läuft nicht oder `ROS_MASTER_URI` falsch. Prüfen mit
`rostopic list`. Kommt ein Fehler, läuft kein roscore.

**`KeyError: 'VEHICLE_NAME'`**
Umgebungsvariable nicht gesetzt: `export VEHICLE_NAME=<botname>`.

**Bot lenkt nicht / fährt nicht los**
`/detect/lane` prüfen (`rostopic echo`). Kommen Werte? Falls nicht, hängt es
in `detect_lane_node` – BEV oder HSV-Masken kontrollieren. Außerdem prüfen, ob
`/enable/lane` auf `True` steht.

**Bot stoppt ständig / erkennt überall rote Linien**
`red/pixel_threshold` zu niedrig oder ROI zu groß. ROI über `detection_zone`,
`detection_x_start` und `detection_x_end` enger ziehen.

**Bot springt in Kurven auf die Gegenspur**
`max_frame_jump` zu hoch – verkleinern, damit große Sprünge verworfen werden.
Im Dashboard auf `logwarn`-Meldungen ("jump too large") achten.

**Debug-Logs sehen**
Per-Frame-Ausgaben laufen über `rospy.logdebug` (standardmäßig still). Zum
Aktivieren die Node mit `log_level=rospy.DEBUG` starten oder zur Laufzeit über
`rqt_logger_level` hochdrehen. Echte Warnungen (verlorene Linien) erscheinen
als `logwarn` ohnehin.

---

*Stand: Challenge 1 (Lane Following). Die Architektur ist auf spätere
Erweiterung um Intersection- und Obstacle-Handling vorbereitet
(switch_control_node, Enable-Topics, challenge-übergreifendes Dashboard).*
