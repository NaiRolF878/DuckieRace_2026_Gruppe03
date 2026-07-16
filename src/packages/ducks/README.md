# Challenge 3 – Watch out for Ducks

Wendeplatz mit starren Enten befahren, ohne eine umzufahren. Baut auf der
Spurführung aus Challenge 1 (Lane Following + rote Haltelinie) auf und ergänzt
Zonen-Erkennung und einen vollständigen Ausweich-Automaten.

Plattform: Duckiebot **tick** mit ROS 1 (Noetic) unter Ubuntu 20.04. Ohne Duckietown-Shell.

> Ausführliche Strategie- und Code-Dokumentation: **[CHALLENGE3_DOKU.md](CHALLENGE3_DOKU.md)**

---

## Ordnerstruktur

Alle Nodes liegen flach in `src/`, die zugehörigen Konfigurationen in `config/`.
`util.py` lädt die JSONs relativ über `../config/`, daher muss diese
Nebeneinander-Anordnung erhalten bleiben.

```
ducks/
├── src/
│   ├── detect_lane_node.py        # Kamera: BEV, weiße Linie, Zonen, Enten
│   ├── control_lane_node.py       # PID-Spurregelung + Haltelinien-Automat
│   ├── control_obstacle_node.py   # Ausweich-Zustandsautomat (5 Zustände)
│   ├── switch_control_node.py     # Umschaltung Lane ↔ Obstacle
│   ├── camera_dashboard_node.py   # Debug-Visualisierung (2×2-Dashboard)
│   ├── configuration_node.py      # Live-Parameter-GUI (tkinter)
│   ├── util.py                    # Parameter laden/mergen, Live-Updates
│   └── ducks.sh                   # Launcher (startet alle Nodes)
└── config/
    ├── detect_lane_node.json      # HSV, BEV-Trapez, Zonen, white_follow
    ├── control_lane_node.json     # PID, Haltelinien-Timing
    └── control_obstacle_node.json # Ausweich-Parameter, Timeouts, Encoder-Rückkehr
```

---

## Starten

Voraussetzung: die Umgebungsvariable `VEHICLE_NAME` muss gesetzt sein.

```bash
export VEHICLE_NAME=tick
cd ducks/src
./ducks.sh
```

Der Launcher startet alle Nodes als Hintergrundprozesse und beendet sie
gemeinsam bei `Ctrl-C`.

`configuration_node.py` wird **nicht** automatisch gestartet (eigene GUI, optional).
Bei Bedarf separat starten:

```bash
python3 configuration_node.py
```

---

## Funktionsweise

### Wahrnehmung (`detect_lane_node.py`)

Verarbeitet das Kamerabild vollständig in einer einzigen Node:

1. **Bird's-Eye-View (BEV)** – perspektivisch entzerrtes 400×400-px-Bild
2. **Weiße Linie** – HSV-Filter → Position der rechten Fahrbahnmarkierung
3. **Spurversatz** – `lane_center = center_white - offset_px`, normiert auf [-1, +1]
4. **Rote Haltelinie** – zwei HSV-Bereiche decken den roten Farbton ab
5. **Entenerkennung** – Farberkennung (gelb/grün) im **unverzerrten Originalbild**
   (kein Sichtfeldlimit/Verzerrung durch die BEV-Trapez-Transformation); nur der
   Bodenkontaktpunkt jeder erkannten Box wird per Homographie ins BEV projiziert.
   Ein Kalman-Filter glättet die x-Position und überbrückt kurze Erkennungsaussetzer.
6. **Zonen-Belegung** – drei Zonen (nah/mittel/fern) prüfen im Fahrkorridor (BEV)
   dieselben reprojizierten Boden-Kontaktpunkte aus Schritt 5 (keine eigene,
   zweite Farberkennung auf dem BEV-Bild)

Erkennt gezielt **gelbe und grüne Objekte** – Enten **und** die gelbe Mittellinie,
ohne sie zu unterscheiden (beide lösen dieselbe Reaktion aus). Unbunte Reflexionen/
Klebereste auf der Fahrbahn fallen automatisch raus, da sie nicht in den Farbbereich fallen.

### Ausweichen (`control_obstacle_node.py`)

Enthält den **5-Zustands-Automaten**:

```
         Zone nah/mittel        Zonen leer        Nachlauf ab
IDLE ───────belegt────────► EVADE ──────────► PASS ───────────► RETURN ──fertig──► IDLE
                              │                  ↑
                           Timeout          frei / Timeout
                              ▼                  │
                            WAIT (v=0) ──────────┘
```

- **IDLE:** Normalbetrieb, kein Eingriff
- **EVADE:** Ausweich-Offset aktiv, Encoder-Ticks werden akkumuliert
- **WAIT:** Bot stoppt vollständig (Korridor blockiert, Stufe 6); Timeout erzwingt Weiterfahrt
- **PASS:** Offset bleibt aktiv (Nachlauf), Ticks akkumulieren weiter
- **RETURN:** Offset = 0, Encoder-basierte Rückkehr aktiv bis Kamera weiße Linie findet (Stufe 5)

Die Node sendet **keine Fahrbefehle direkt**, sondern publiziert drei Steuersignale:
- `error_offset` – verschiebt die wahrgenommene Spurmitte → PID lenkt automatisch
- `return_omega` – überschreibt PID-omega während Encoder-Rückkehr
- `stop` – setzt v=0 im WAIT-Zustand

**Ausweichrichtung + -stärke** wird beim Eintritt in EVADE einmalig eingefroren:
- Primär aus `/detect/corridor_occupancy` (Lückenprofil über den Fahrkorridor):
  Offset zeigt zur Mitte der breitesten freien Lücke, Stärke proportional zum
  Abstand der Lücke von der Korridormitte (`evade_offset_min` … `evade_offset`)
- Fallback (kein Profil / Korridor komplett belegt) – alte `duck_x`-Heuristik:
  Ente rechts von BEV-Mitte → links ausweichen, Ente links → rechts ausweichen,
  kein Blob (z.B. gelbe Linie) → rechts als sicherer Standard

### Spurführung (`control_lane_node.py`)

**Einzige Node**, die den Fahrbefehl an den Bot sendet. Priorität:

```
1. obstacle/stop = True      →  v=0, omega=0             (WAIT-Zustand)
2. Rote Haltelinie erkannt   →  v=0, omega=0             (Haltelinien-Automat)
3. return_omega ≠ 0          →  v=PID, omega=return_omega (Encoder-Rückkehr)
4. Normalbetrieb             →  v=PID, omega=PID
```

### Umschaltung (`switch_control_node.py`)

| Übergang | Auslöser |
|----------|----------|
| Lane → Obstacle | Zone **nah** oder **mittel** belegt (`/detect/zones`) |
| Obstacle → Lane | Ausweichen abgeschlossen (`/obstacle/done`) |

`/enable/lane` bleibt auch im Obstacle-Modus **immer aktiv**, weil
`control_lane_node` die eigentliche Fahrt (inkl. addiertem Offset) ausführt.

---

## Topic-Übersicht

Alle Topics mit Prefix `/tick/` (Bot-Name).

| Topic | Typ | Von → Nach |
|-------|-----|-----------|
| `/tick/detect/lane` | `Float64` | detect_lane → control_lane |
| `/tick/detect/stop_line` | `Bool` | detect_lane → control_lane |
| `/tick/detect/duck` | `Float64` | detect_lane → control_obstacle |
| `/tick/detect/zones` | `Float32MultiArray` | detect_lane → control_obstacle, switch_control |
| `/tick/detect/corridor_occupancy` | `Float32MultiArray` | detect_lane → control_obstacle |
| `/tick/obstacle/error_offset` | `Float64` | control_obstacle → control_lane |
| `/tick/obstacle/return_omega` | `Float64` | control_obstacle → control_lane |
| `/tick/obstacle/stop` | `Bool` | control_obstacle → control_lane |
| `/tick/obstacle/done` | `Bool` | control_obstacle → switch_control |
| `/tick/obstacle/state` | `String` | control_obstacle → detect_lane, camera_dashboard (Debug-Overlay: Idle/Evade/Wait/Pass/Return) |
| `/tick/enable/lane` | `Bool` | switch_control → control_lane |
| `/tick/enable/obstacle` | `Bool` | switch_control → control_obstacle |
| `/tick/car_cmd_switch_node/cmd` | `Twist2DStamped` | control_lane → Bot |

Debug-Bilder (`CompressedImage`): `/tick/debug/original`, `/tick/debug/annotated`,
`/tick/debug/bird_view`, `/tick/debug/lane_white`, `/tick/debug/lane_red`,
`/tick/debug/duck_bev`, `/tick/debug/duck_original` (Originalbild mit erkannten
Enten-Boxen, vor der BEV-Transformation).

---

## Parameter justieren

Alle Parameter liegen in `config/*.json` und lassen sich zur Laufzeit über
`configuration_node.py` per Schieberegler ändern (kein Neustart nötig).
Es gibt nur noch einen `"default"`-Block – keine bot-spezifischen Abschnitte.

Wichtige Stellschrauben:

**`detect_lane_node.json`**
- `white_follow.offset_px` – Sollabstand zur weißen Linie in BEV-Pixeln (Standard: 150)
- `white.vl / vh` – Helligkeitsbereich für weiße Linie (HSV value)
- `obstacle_color.yellow_*` / `green_*` – HSV-Farbbereiche für Enten/gelbe Linie
  (Hue/Saturation/Value je für Gelb und Grün, ersetzt die frühere Helligkeits-Schwelle)
- `duck.kf_process_var` / `kf_measurement_var` / `kf_max_missed_frames` – Kalman-Filter
  für die Enten-x-Position (Glättung + Aussetzer-Überbrückung)
- `zones.pixel_threshold_frac` – ab wann eine Zone als belegt gilt (Standard: 0.05 = 5%)
- `zones.corridor_width_px` – Breite des überwachten Fahrkorridors, **symmetrisch um
  die Bildmitte des BEV-Bilds fixiert** (nicht die ganze Spur, und unabhängig von der
  weißen Linie – bleibt dadurch auch bei kurzzeitig verlorener Linienerkennung stabil)
  – entspricht Bot-Breite + Ausweich-Spielraum (Standard: 300 px)

**`control_obstacle_node.json`**
- `evade.evade_offset` – maximale Stärke des Ausweich-Offsets, Lücke am Korridorrand (Standard: 0.6)
- `evade.evade_offset_min` – minimale Stärke, Lücke nahe Korridormitte (Standard: 0.25)
- `evade.nachlauf_secs` – Nachlauf nach letzter Objekt-Sichtung (Standard: 1.5 s)
- `evade.return_omega` – Drehrate bei Encoder-Rückkehr (Standard: 0.5 rad/s)
- `evade.evade_timeout_secs` – Max. Zeit im EVADE bevor WAIT (Standard: 5.0 s)
- `evade.wait_timeout_secs` – Max. Wartezeit im WAIT (Standard: 3.0 s)
- `evade.free_stable_frames` – wie viele Frames der Korridor **hintereinander** frei
  sein muss, bevor EVADE/WAIT wirklich verlassen wird (Standard: 5, gegen Flackern)
- `evade.active` – Gesamte Ausweichlogik ein (1) / aus (0)

**`control_lane_node.json`**
- `pid.p / i / d` – PID-Faktoren für Spurfolgen
- `pid.max_vel / min_vel` – Geschwindigkeitsgrenzen (m/s)
- `stop_line.stop_duration` – Standzeit an roter Linie (s)
- `stop_line.cooldown_duration` – Wartezeit bis nächste rote Linie auslöst (s)

---

## Kalibrierung am Bot (empfohlene Reihenfolge)

1. **Weiße Linie prüfen.** `/tick/debug/lane_white` ansehen – nur die weiße
   Fahrbahnmarkierung soll hell erscheinen. `white.vl/vh` anpassen.

2. **Spurabstand einstellen.** `white_follow.offset_px` so wählen, dass der Bot
   mittig in seiner Fahrspur fährt (150 px ist Startwert, größer = näher an Linie).

3. **Farbbereiche kalibrieren.** `/tick/debug/duck_original` (Originalbild mit
   erkannten Boxen) und `/tick/debug/duck_bev` ansehen – `obstacle_color.yellow_*`/
   `green_*` so einstellen, dass nur echte Enten + gelbe Linie erfasst werden,
   Fahrbahn und Klebereste schwarz bleiben.

4. **Zonen/Korridor kalibrieren.** `/tick/debug/duck_bev` ansehen – das
   Korridor-Rechteck ist symmetrisch um die Bildmitte zentriert (unabhängig von
   weißer Linie/magenta Ziellinie). `zones.corridor_width_px` auf Bot-Breite +
   Ausweich-Spielraum einstellen
   (**nicht** die ganze Spur – sonst löst der Bot ständig unnötig aus).
   `pixel_threshold_frac` danach: Ente im Weg → Zone soll auf 1 springen,
   leere Fahrbahn → Zone soll 0 bleiben.

5. **Ausweichstärke einstellen.** `evade_offset` bestimmt, wie weit der Bot
   ausweicht. Zu niedrig → streift Ente; zu hoch → verlässt Fahrbahn.

6. **Entprellung einstellen.** `free_stable_frames` so wählen, dass der Zustand
   nicht bei kurzem Flackern der Farberkennung sofort wieder verlässt (Standard 5
   Frames ≈ 0,5 s bei 10 Hz); euer `Zustand:`-Overlay im `duck_bev`-Bild bzw. im
   Dashboard zeigt live, ob EVADE stabil bleibt.

7. **Rückkehr einstellen.** `return_omega` und `return_threshold` so wählen,
   dass der Bot nach dem Manöver sauber zurück auf die weiße Linie findet.
