# Challenge 3 – Watch out for Ducks

Wendeplatz mit starren Enten befahren, ohne eine umzufahren. Baut auf der
Spurführung aus Challenge 1 (Lane Following + rote Haltelinie) auf und ergänzt
Entenerkennung und ein Ausweichmanöver.

Plattform: Duckiebot mit ROS 1 (Noetic) unter Ubuntu 20.04. Ohne Duckietown-Shell.

---

## Ordnerstruktur

Alle Nodes liegen flach in `src/`, die zugehörigen Konfigurationen in `config/`.
`util.py` lädt die JSONs relativ über `../config/`, daher muss diese
Nebeneinander-Anordnung erhalten bleiben.

```
ducks/
├── src/
│   ├── detect_lane_node.py        # Spurversatz + rote Haltelinie (Basis aus Ch.1)
│   ├── detect_duck_node.py        # Entenerkennung (Hough + Gelb + BEV-Helligkeit)
│   ├── control_lane_node.py       # PID-Spurregelung + Stop-Line-Automat
│   ├── control_obstacle_node.py   # Ausweichlogik (EvadeState-Automat)
│   ├── switch_control_node.py     # Umschaltung Lane <-> Obstacle
│   ├── camera_dashboard_node.py   # Debug-Visualisierung (2x2-Dashboard)
│   ├── configuration_node.py      # Live-Parameter-GUI (tkinter)
│   ├── util.py                    # Parameter laden/mergen, Live-Updates
│   └── ducks.sh                   # Launcher (startet alle Nodes)
└── config/
    ├── detect_lane_node.json
    ├── detect_duck_node.json
    ├── control_lane_node.json
    └── control_obstacle_node.json
```

---

## Starten

Voraussetzung: die Umgebungsvariable `VEHICLE_NAME` muss gesetzt sein.

```bash
export VEHICLE_NAME=dorette        # eigenen Bot-Namen eintragen
cd ducks/src
./ducks.sh
```

Der Launcher startet sechs Nodes als Hintergrundprozesse und beendet sie
gemeinsam bei `Ctrl-C`: `detect_lane`, `detect_duck`, `control_lane`,
`control_obstacle`, `switch_control` und `camera_dashboard`.

`configuration_node.py` wird NICHT automatisch gestartet (eigene GUI). Bei
Bedarf separat starten:

```bash
python3 configuration_node.py
```

---

## Funktionsweise

### Wahrnehmung

`detect_lane_node` liefert den Spurversatz auf `/detect/lane` (Bereich
`[-1, +1]`) und die rote Haltelinie auf `/detect/stop_line`. Die Spurerkennung
arbeitet im Bird's-Eye-View (BEV), einer perspektivisch entzerrten 400×400-Ansicht.

`detect_duck_node` erkennt Enten über **zwei Verfahren, die gleichzeitig
anschlagen müssen** (reduziert Fehlalarme):

1. **Originalbild:** Hough-Kreise, gefiltert über einen Gelbanteil im Kreis.
   Läuft nur auf der unteren Bildhälfte (Fahrweg), spart Rechenzeit und
   unterdrückt Fehlkreise am Horizont.
2. **BEV-ROI:** Helligkeitsprüfung. Freie Fahrbahn ist dunkel; eine Ente hebt
   die mittlere Helligkeit im ROI über den Schwellwert.

Veröffentlicht wird die Position der nächsten Ente auf `/detect/duck`
(`-99.0` = keine Ente) und der freie Platz links/rechts der Ente im BEV auf
`/detect/duck_space`.

### Ausweichen

`control_obstacle_node` enthält den **EvadeState-Automaten**:

```
Idle  ──Ente erkannt──▶  Evading  ──Ente passiert──▶  Returning  ──fertig──▶  Idle
```

Richtungswahl anhand des freien Platzes:

| Situation                | Reaktion                                  |
|--------------------------|-------------------------------------------|
| Mehr Platz rechts        | rechts ausweichen                         |
| Mehr Platz links         | links ausweichen                          |
| Etwa gleich viel Platz   | links (StVO: links überholen)             |
| Kein Platz auf beiden    | Gegenspurübernahme (links, größerer Offset)|

Die Node berechnet die Spurregelung **nicht selbst**, sondern published nur
einen additiven Lenk-Offset auf `/obstacle/error_offset`. `control_lane_node`
addiert diesen zum Spurfehler. So bleibt die PID-Regelung an einer einzigen
Stelle. Der Offset wird über eine Rampe sanft auf- und wieder abgebaut.

Damit das Manöver nicht abbricht, während der Bot noch neben der Ente steht
(die Ente wandert beim Ausweichen aus dem Bild), wird das Ausweichen nach der
letzten Entensichtung noch für `evade_hold` Sekunden gehalten. Die einmal
gewählte Richtung wird dabei eingefroren.

### Rückkehr zur Spur

Zwei Modi, umschaltbar über den Parameter `return_mode`:

- **`pid` (Default, 0):** Kamerabasiert. Der Offset wird auf 0 gerampt, die
  Spur-PID-Regelung findet die Linie selbst wieder. Selbstkorrigierend.
- **`odometry` (1):** Befehls-Integration. Das tatsächlich gesendete `omega`
  wird während des Ausweichens aufintegriert und bei der Rückkehr spiegelbildlich
  ausgeglichen. Hinweis: Dies ist kein Encoder-Feedback, sondern eine Integration
  der gesendeten Befehle – Radschlupf bleibt unsichtbar und summiert sich als
  Fehler. Nur verwenden, wenn der kamerabasierte Modus die Linie beim Ausweichen
  ganz verliert.

### Umschaltung

`switch_control_node` schaltet zwischen Lane- und Obstacle-Modus:

- **Lane → Obstacle:** Ente erkannt (`/detect/duck` ≠ -99)
- **Obstacle → Lane:** Ausweichen abgeschlossen (`/obstacle/done`)

`/enable/lane` bleibt auch im Obstacle-Modus aktiv, weil `control_lane_node`
die Fahrt (inkl. addiertem Offset) weiterhin ausführt. `/enable/obstacle`
steuert nur, ob die Obstacle-Node einen Offset erzeugen darf.

---

## Topic-Übersicht

| Topic                         | Typ                 | Von → Nach                          |
|-------------------------------|---------------------|-------------------------------------|
| `/detect/lane`                | Float64             | detect_lane → control_lane          |
| `/detect/stop_line`           | Bool                | detect_lane → control_lane          |
| `/detect/duck`                | Float64             | detect_duck → control_obstacle, switch_control |
| `/detect/duck_space`          | Float32MultiArray   | detect_duck → control_obstacle      |
| `/obstacle/error_offset`      | Float64             | control_obstacle → control_lane     |
| `/obstacle/done`              | Bool                | control_obstacle → switch_control   |
| `/enable/lane`                | Bool                | switch_control → control_lane       |
| `/enable/obstacle`            | Bool                | switch_control → control_obstacle   |
| `/car_cmd_switch_node/cmd`    | Twist2DStamped      | control_lane → Bot (+ Odometrie-Mitlesen) |

Debug-Bilder (`CompressedImage`): `/debug/original`, `/debug/annotated`
(BEV), `/debug/lane_yellow`, `/debug/lane_white`, `/debug/duck`,
`/debug/duck_bev`.

---

## Parameter justieren

Alle Parameter liegen in den JSON-Dateien unter `config/` und lassen sich zur
Laufzeit über `configuration_node.py` per Schieberegler ändern. Jeder Bot kann
über einen eigenen Block (z.B. `"dorette": {...}`) vom `default` abweichende
Werte erhalten; nicht genannte Werte werden aus `default` übernommen.

Wichtige Stellschrauben:

- **`detect_duck_node`** – `duck` (HSV-Bereich der Entenfarbe, Richtung
  Orange voreingestellt), `bev.brightness_threshold` (Helligkeitsschwelle),
  `hough` (Kreiserkennung), `hough.roi_top` (ab wo gesucht wird).
- **`control_obstacle_node`** – `evade.evade_offset` (Stärke des Ausweichens),
  `evade.evade_hold` (wie lange das Manöver gehalten wird),
  `evade.return_mode` (0 = pid, 1 = odometry), `evade.space_threshold`.
- **`control_lane_node`** – `pid` (P/I/D), `max_vel`/`min_vel`,
  `stop_line.stop_duration`/`cooldown_duration`.

---

## Kalibrierung am Bot (Reihenfolge)

1. **BEV-Trapez prüfen.** `crop_image`-Eckpunkte in `detect_duck_node.json`
   müssen mit denen in `detect_lane_node.json` übereinstimmen, sonst passt die
   Platzmessung geometrisch nicht zur Spur.
2. **Entenfarbe.** Im Dashboard `/debug/duck` und `/debug/duck_bev` ansehen und
   die `duck`-HSV-Werte so einstellen, dass nur die Ente (nicht die gelbe
   Mittellinie) als rote Box erkannt wird.
3. **Helligkeitsschwelle.** `brightness_threshold` so wählen, dass eine Ente im
   ROI sicher anschlägt, leere Fahrbahn aber nicht.
4. **`evade_hold`.** An Geschwindigkeit und Entengröße anpassen: zu kurz → Bot
   lenkt zu früh zurück, zu lang → fährt unnötig weit auf der falschen Bahn.
5. **Rückkehrmodus.** Mit `pid` beginnen. Nur auf `odometry` wechseln, wenn der
   Bot die Linie beim Ausweichen sichtbar ganz verliert.

### Vorzeichen-Check (Odometrie)

Die Odometrie-Rückkehr nimmt an, dass `omega > 0` einer Linksdrehung entspricht
(ROS-Standard). Im Log beim Linksausweichen prüfen, ob `Gierintegral` positiv
wird. Falls nicht, ist die Konvention des Bots umgekehrt – die Rückkehr
funktioniert dann trotzdem, da das Integral nur gegen sich selbst ausgeglichen
wird, aber der Check schafft Klarheit.
