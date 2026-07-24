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
│   ├── control_obstacle_node.py   # Ausweich-Zustandsautomat (3 Zustände)
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
6. **Zonen-Belegung** – drei Zonen (nah/mittel/fern) prüfen rein geometrisch
   (Rechteck-Überlappung, kein Flächen-Threshold), ob dieselben reprojizierten
   Boden-Kontaktpunkte aus Schritt 5 im Korridor-x-Bereich und auf/vor der
   jeweiligen Zonentiefe liegen (keine eigene, zweite Farberkennung auf dem
   BEV-Bild)

Erkennt gezielt **gelbe und grüne Objekte** – Enten **und** die gelbe Mittellinie,
ohne sie zu unterscheiden (beide lösen dieselbe Reaktion aus). Unbunte Reflexionen/
Klebereste auf der Fahrbahn fallen automatisch raus, da sie nicht in den Farbbereich fallen.

### Ausweichen (`control_obstacle_node.py`)

Enthält den **3-Zustands-Automaten** (vereinfacht ggü. einer früheren
6-Zustands-Version – siehe Kopfkommentar der Datei für die Begründung). Die
drei Zonen (nah/mittel/fern) lösen weiterhin **unterschiedliche** Reaktionen aus:
- **fern:** nur Beobachtung, kein Eingriff. Erkennung auf große Distanz ist
  weniger zuverlässig, und da der Korridor genau der Bot-Breite entspricht,
  gibt es ohnehin kein "sanftes" Teil-Ausweichen – jede Reaktion müsste
  praktisch dieselbe Stärke haben wie in der mittel-Zone, nur früher
  ausgelöst auf Basis unsichererer Daten.
- **mittel:** kontinuierlicher PID-Offset – **kein eigener Zustand**, wird
  bei jedem Tick neu berechnet, solange die Zone belegt ist.
- **nah:** Notfall (EMERGENCY) – umgeht die PID komplett, feste Drehrate.

```
                    Zone nah                NAH-Zone frei /
IDLE ────────────────belegt────────► EMERGENCY ──Timeout────► RETURN ──fertig──► IDLE
 (Zone mittel: kontinuierlicher                                  │
  PID-Offset, kein Zustandswechsel)                    NAH-Zone wieder belegt
                                                                  │
                                                                  ▼
                                                              EMERGENCY
```

- **IDLE:** Normalbetrieb. Ist die mittel-Zone belegt, fließt trotzdem bei
  jedem Tick ein frisch berechneter Ausweich-Offset ein (siehe unten) – ohne
  Timeout, ohne Nachlauf, ohne Rückkehr-Logik, weil der Offset automatisch
  auf 0 zurückfällt, sobald die Zone wieder frei ist.
- **EMERGENCY:** nah-Zone – feste Drehrate (`emergency_omega_rad`) + Wiggle
  (v kippt im `wiggle_interval_secs`-Takt das Vorzeichen, gegen Standreibung
  beim Drehen auf der Stelle), umgeht die PID komplett. Verlässt den Zustand,
  sobald die nah-Zone `free_stable_frames` lang stabil frei ist, oder nach
  `emergency_timeout_secs` als Failsafe.
- **RETURN:** kurze, feste Geradeausfahrt (`return_forward_secs` bei
  `return_forward_speed`), ebenfalls per PID-Bypass – löst den Bot physisch
  vom Hindernis, bevor wieder normal gelenkt wird. Danach zurück zu IDLE.

Gestrichen ggü. der früheren Version: der WAIT-Zustand (reiner
Timeout-Fallback) und das Encoder-Rückkehr-Tracking (Ticks liefen während des
Drehens auf der Stelle mit ein, obwohl Drehen kaum Vorwärtsbewegung erzeugt –
das Rückkehr-Ziel war dadurch kein verlässliches Maß für die tatsächliche
seitliche Auslenkung). RETURN nutzt stattdessen dieselbe feste, kurze
Geradeausfahrt wie bei `avoid_ducks`' `DRIVE_FORWARD_DISTANCE`.

Die Node sendet **keine Fahrbefehle direkt** (außer im PID-Bypass), sondern
publiziert Steuersignale:
- `error_offset` – verschiebt die wahrgenommene Spurmitte → PID lenkt automatisch
- `emergency_active` / `emergency_cmd` – in EMERGENCY **und** RETURN übernimmt
  `control_lane_node` `emergency_cmd` (v+omega) 1:1, PID greift nicht ein

**Ausweichrichtung + -stärke** wird bei **jedem Tick neu** berechnet (nicht
mehr beim Zustandseintritt eingefroren) – reagiert dadurch auch während eines
laufenden Manövers auf eine sich verändernde Lücke:
- Primär aus `/detect/corridor_occupancy` (Lückenprofil über den Fahrkorridor,
  der genau der Bot-Breite entspricht): gewählt wird die Seite mit dem
  **größeren freien Abstand vom Korridorrand bis zum nächsten Hindernis**
  (nicht die breiteste Lücke zwischen zwei Hindernissen – so wird nie zwischen
  zwei Objekten hindurchgequetscht, sondern immer außen an ihnen vorbei). Der
  rechte Rand wird zusätzlich durch die live erkannte weiße Linie begrenzt
  (`white_line_margin_px`). Stärke richtet sich nach der Breite dieser freien
  Seite relativ zur Korridorbreite: fast der ganze Korridor frei → kleiner
  Offset (`evade_offset_min`), nur ein schmaler Rest frei → voller `evade_offset`
- Fallback (kein Profil / Korridor komplett belegt) – alte `duck_x`-Heuristik:
  Ente rechts von BEV-Mitte → links ausweichen, Ente links → rechts ausweichen,
  kein Blob (z.B. gelbe Linie) → rechts als sicherer Standard
- Im EMERGENCY-Zustand wird nur das **Vorzeichen** dieser Berechnung genutzt
  (Richtung), die Stärke ist immer die feste `emergency_omega_rad`.

### Spurführung (`control_lane_node.py`)

**Einzige Node**, die den Fahrbefehl an den Bot sendet. Priorität:

```
1. emergency_active = True   →  v/omega = emergency_cmd  (NOTFALL + RÜCKKEHR, umgeht PID)
2. Rote Haltelinie erkannt   →  v=0, omega=0             (Haltelinien-Automat)
3. Normalbetrieb             →  v=PID (inkl. error_offset), omega=PID
```

### Umschaltung (`switch_control_node.py`)

Seit der Vereinfachung von `control_obstacle_node.py` (kontinuierlicher
Mittel-Zonen-Offset statt eigenem EVADE-Zustand, siehe oben) hört
`control_obstacle_node` nicht mehr auf `/enable/obstacle` – die Node
berechnet ihren Offset immer selbstständig, gesteuert nur noch über
`evade.active` in der Config. `switch_control_node.py` läuft unverändert
weiter (schadet nicht), sein `/enable/obstacle`-Publish hat aber aktuell
keinen Abonnenten mehr.

`/enable/lane` bleibt **immer aktiv**, weil `control_lane_node` die
eigentliche Fahrt (inkl. addiertem Offset bzw. PID-Bypass) ausführt.

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
| `/tick/obstacle/done` | `Bool` | control_obstacle → switch_control |
| `/tick/obstacle/state` | `String` | control_obstacle → detect_lane, camera_dashboard (Debug-Overlay: Idle/Emergency/Return) |
| `/tick/obstacle/emergency_active` | `Bool` | control_obstacle → control_lane (PID-Bypass, EMERGENCY **und** RETURN) |
| `/tick/obstacle/emergency_cmd` | `Twist2DStamped` | control_obstacle → control_lane (v/omega bei aktivem Bypass) |
| `/tick/enable/lane` | `Bool` | switch_control → control_lane |
| `/tick/enable/obstacle` | `Bool` | switch_control → *(kein Abonnent mehr – control_obstacle_node läuft seit der Vereinfachung selbstständig, gesteuert nur noch über `evade.active` in der Config)* |
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
- `zones.corridor_width_px` – Breite des überwachten Fahrkorridors, **symmetrisch um
  die Bildmitte des BEV-Bilds fixiert** (nicht die ganze Spur, und unabhängig von der
  weißen Linie – bleibt dadurch auch bei kurzzeitig verlorener Linienerkennung stabil)
  – entspricht der Bot-Breite (Standard: 300 px)
- `zones.white_line_margin_px` – Sicherheitsabstand, um den der rechte
  Korridorrand für die Lücken-Suche zusätzlich durch die live erkannte weiße
  Linie begrenzt wird (Standard: 20 px)

**`control_obstacle_node.json`**
- `evade.active` – Gesamte Ausweichlogik ein (1) / aus (0)
- `evade.evade_offset` – maximale Stärke des kontinuierlichen Ausweich-Offsets
  (mittel-Zone), nur ein schmaler Rest des Korridors frei (Standard: 0.6)
- `evade.evade_offset_min` – minimale Stärke, fast der ganze Korridor frei
  (Standard: 0.25)
- `evade.free_stable_frames` – wie viele Frames die nah-Zone **hintereinander**
  frei sein muss, bevor EMERGENCY wirklich verlassen wird (Standard: 5, gegen Flackern)
- `evade.emergency_omega_rad` – feste Drehrate im NOTFALL (nah-Zone), umgeht die
  PID (Standard: 1.6 rad/s)
- `evade.emergency_timeout_secs` – hartes Zeitlimit für NOTFALL, falls die
  nah-Zone nie stabil frei wird (Failsafe → RETURN, Standard: 5.0 s)
- `evade.wiggle_interval_secs` – wie oft `v` im NOTFALL das Vorzeichen wechselt,
  gegen Standreibung beim Drehen auf der Stelle (Standard: 0.06 s)
- `evade.wiggle_power` – Stärke des Wiggle-Ausschlags (Standard: 0.07)
- `evade.return_forward_secs` – Dauer der festen Geradeausfahrt in RETURN, um
  sich physisch vom Hindernis zu lösen (Standard: 1.0 s)
- `evade.return_forward_speed` – Geschwindigkeit während RETURN (Standard: 0.15 m/s)

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
   weißer Linie/magenta Ziellinie). `zones.corridor_width_px` auf die Bot-Breite
   einstellen (**nicht** die ganze Spur – sonst löst der Bot ständig unnötig aus).
   `zones.white_line_margin_px` so wählen, dass der Bot beim Ausweichen nie über
   die weiße Linie fährt. Die Zonen-Belegung selbst ist rein geometrisch (Ente
   im Korridor-Bereich → Zone springt auf 1) – kein Schwellwert zum Tunen nötig.

5. **Ausweichstärke einstellen.** `evade_offset` bestimmt, wie weit der Bot
   ausweicht. Zu niedrig → streift Ente; zu hoch → verlässt Fahrbahn.

6. **Entprellung einstellen.** `free_stable_frames` so wählen, dass EMERGENCY
   nicht bei kurzem Flackern der Farberkennung in der nah-Zone sofort wieder
   verlassen wird (Standard 5 Frames ≈ 0,5 s bei 10 Hz); euer `Zustand:`-Overlay
   im `duck_bev`-Bild bzw. im Dashboard zeigt live, ob EMERGENCY stabil bleibt.

7. **Rückkehr einstellen.** `return_forward_secs`/`return_forward_speed` so
   wählen, dass der Bot nach dem Notfall-Manöver das Hindernis sicher hinter
   sich lässt, bevor die normale PID-Spurführung wieder übernimmt.
