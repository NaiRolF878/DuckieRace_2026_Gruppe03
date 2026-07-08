# Challenge 4 – Mapping & Path Finding

> ROS 1 (Noetic) · Ubuntu 20.04 · Python 3 · OpenCV · pupil_apriltags · tkinter

Der Duckiebot bekommt einen Stadtgraphen als JSON. In **Phase 1 (Mapping)**
fährt er selbstständig (DFS) alle Kanten des Graphen ab und findet dabei
farbige **Tore** (AprilTag-IDs 5–13), die er auf Graphenkanten mappt. In
**Phase 2 (Planung)** wird automatisch die optimale Reihenfolge berechnet, in
der alle Tore angefahren werden sollen. Nach manueller Bestätigung im
Debug-Fenster fährt der Bot in **Phase 3 (Delivery)** diese Reihenfolge ab –
**nur Phase 3 wird auf Zeit bewertet**, daher ist gute Pfadoptimierung
entscheidend.

Dieses Paket baut auf `intersection_handling` (Challenge 2) auf: die
Wahrnehmungs- und Steuerungs-Nodes (Spurfolgen, Kreuzung fahren) bleiben in
ihrer Grundstruktur erhalten, kommen aber ohne Zufallsentscheidungen aus –
die Fahrtrichtung an jeder Kreuzung wird jetzt deterministisch von der
Graph-/Pfadplanungs-Logik vorgegeben.

---

## Inhaltsverzeichnis

- [Dateien](#dateien)
- [Grundidee](#grundidee)
- [Die drei Phasen](#die-drei-phasen)
- [Systemüberblick](#systemüberblick)
- [Graph-Format](#graph-format)
- [Tag-ID ↔ Wort-Übersetzung](#tag-id--wort-übersetzung)
- [Nodes](#nodes)
- [Topics](#topics)
- [Konfigurationsparameter](#konfigurationsparameter)
- [Encoder-Kalibrierung](#encoder-kalibrierung)
- [Setup & Starten](#setup--starten)
- [Vor dem Challenge-Tag](#vor-dem-challenge-tag)
- [Bekannte Einschränkungen](#bekannte-einschränkungen)

---

## Dateien

| Datei | Typ | Beschreibung |
|---|---|---|
| `detect_lane_node.py` | Node | Spurerkennung + rote Haltelinie (unverändert aus Challenge 2) |
| `detect_apriltag_node.py` | Node | Kreuzungs-Tags (1–4) **+ Tor-Tags (5–13)** |
| `switch_control_node.py` | Node | FSM – Richtung kommt jetzt von `next_direction`, kein `random.choice` mehr |
| `control_lane_node.py` | Node | PID-Spurregler (unverändert) |
| `control_intersection_node.py` | Node | Fährt die Kreuzung – Segment-Ende jetzt **encoder-basiert** (Ticks) statt zeitbasiert |
| `graph_state_node.py` | Node | **Neu.** Verwaltet Graph-Zustand (Position, besuchte Kanten, gefundene Tore) |
| `explore_control_node.py` | Node | **Neu.** Phase 1: DFS-Exploration über alle Graphkanten |
| `path_planner_node.py` | Node | **Neu.** Phase 2+3: Dijkstra + TSP-Planung, fährt die Tore ab |
| `debug_graph_node.py` | Node | **Neu.** tkinter-Dashboard: Karte, Bot-Position, Delivery-Pfad, Start-Button |
| `configuration_node.py` | Node | Live-Kalibrierungs-GUI (überspringt jetzt Configs ohne `parameters`-Block) |
| `camera_dashboard_node.py` | Node | Kamera-Debug-Dashboard (unverändert) |
| `util.py` | Hilfsfunktionen | Parameter laden, default+Bot mergen, live updaten |
| `config/mapping_node.json` | Config | **Neu.** Graph, Start-Knoten, Layout, Planungsmodus |
| `config/*.json` | Config | je Node ein Parameter-Satz (nur `default` + `track`) |

---

## Grundidee

Drei Bausteine, die schon aus Challenge 2 bekannt sind, bleiben unverändert:
Spurerkennung, AprilTag-Erkennung, PID-Spurregler und die Kreuzungs-FSM. Neu
hinzu kommt eine **Graph-Schicht**, die sich um alles kümmert, was mit der
*Stadtkarte* zu tun hat:

- `graph_state_node` ist das Gedächtnis: es weiß, wo der Bot gerade ist,
  welche Kanten schon befahren wurden und wo die Tore liegen.
- `explore_control_node` und `path_planner_node` sind die zwei "Gehirne" für
  Phase 1 bzw. Phase 2/3 – sie entscheiden, wohin der Bot als Nächstes soll,
  und geben das als Tag-Richtung an dieselbe FSM (`switch_control_node`)
  weiter, die auch in Challenge 2 schon existiert.
- `debug_graph_node` macht den ganzen Prozess sichtbar und ist der einzige
  Ort, an dem ein Mensch eingreift (Button "Delivery starten").

Die eigentliche Fahr-Logik (Spur halten, Kreuzung anfahren, abbiegen) bleibt
komplett unverändert gegenüber Challenge 2 – nur *woher* die Abbiegerichtung
kommt, hat sich geändert: früher `random.choice`, jetzt die Graph-Logik.

---

## Die drei Phasen

| Phase | Name | Bewertet? | Node(s) am Steuer |
|---|---|---|---|
| 1 | **Mapping** | Ja (Vollständigkeit) | `explore_control_node` |
| 2 | **Planung** | Nein | `path_planner_node` (läuft im Hintergrund, sobald Phase 1 fertig ist) |
| 3 | **Delivery** | Ja (Zeit!) | `path_planner_node` (erst nach Knopfdruck) |

Übergang Phase 1 → 3 erfolgt **nur nach manueller Bestätigung** ("Delivery
starten"-Button im Debug-Fenster, aktiv sobald `exploration_done == True`).

---

## Systemüberblick

```
                          Kamera
            (/camera_node/image/compressed)
                            |
            +---------------+---------------+
            v                               v
      detect_lane                     detect_apriltag
       |     |                         |    |
     lane  stop_line          Kreuzungs-Tag  Tor-Tag
       |     |                (1-4)  (id)   (5-13)
       |     |                    |    |       |
       |     |                    v    v       v
       |     |              +------------------------+
       |     |              |    graph_state_node     |  <-- Graph-Gedaechtnis
       |     |              | current_node/edge,       |
       |     |              | visited_edges, gate_map,  |
       |     |              | exit_directions (Tag->Wort)|
       |     |              +------------------------+
       |     |                 |          |        |
       |     |          current_node  visited_edges exit_directions
       |     |                 v          v        v
       |     |     +----------------+  +-------------------+
       |     |     | explore_control|  |  path_planner      |  <-- Phase 1 / 2+3
       |     |     |  (DFS, Phase1) |  | (Dijkstra+TSP)      |
       |     |     +----------------+  +-------------------+
       |     |                 |                  |
       |     |            next_direction (Wort: left/right/straight)
       |     |                 +--------+---------+
       |     |                          v
       |     |         +------------------------------------------+
       |     +-------->|           switch_control  (FSM)          |
       |                |   Kreuzung? . next_direction . Phase    |
       |                +------------------------------------------+
       |                   | enable/lane     | enable/intersection
       |                   |                 | phase . direction
       v                   v                 v
   control_lane              control_intersection (jetzt Encoder-basiert)
       |                       |         |
       |                       |     turn_done
       +-------------+---------+
                     v
           /car_cmd_switch_node/cmd  ->  Motoren

                                          debug_graph_node (tkinter)
                                     abonniert alle /graph/* und /navigation/*
                                     Topics, zeigt Karte + Status, publiziert
                                     nur /navigation/start_delivery
```

---

## Graph-Format

Der Graph ist **ungerichtet und symmetrisch**: Spur und Gegenspur teilen sich
denselben AprilTag. Die Einmündungs-Nummern (1–4) an jeder Kreuzung sind
**physikalisch fest**:

- Tag 1 und Tag 3 liegen sich gegenüber (geradeaus)
- Tag 2 ist rechts von Tag 1, Tag 4 ist links von Tag 1

```json
{
  "graph": {
    "A": { "1": ["B", "1"], "2": ["C", "2"], "3": ["C", "1"], "4": ["B", "2"] },
    "B": { "1": ["A", "1"], "2": ["A", "4"], "3": ["C", "4"] },
    "C": { "1": ["A", "3"], "2": ["A", "2"], "4": ["B", "3"] }
  },
  "mapping_start_node": "A",
  "delivery_start_node": "A",
  "path_planning": { "mode": "optimal", "fallback": "nearest_neighbor" },
  "debug_layout": { "node_positions": {} }
}
```

`"A": { "2": ["C", "2"] }` heißt: An Kreuzung A, Ausfahrt Tag 2 gewählt →
Kante führt zu Knoten C → dort kommt der Bot über dessen Tag 2 an
(symmetrisch, da hier zufällig beide Enden dieselbe Nummer tragen – das ist
kein Muss, siehe `"3": ["C", "1"]` im selben Beispiel).

- **Kreuzungs-Tags:** IDs 1–4 (an Kreuzungen, codieren die Einmündung)
- **Tor-Tags:** IDs 5–13 (auf den Kanten zwischen Kreuzungen, codieren Ziele)
- **Kanten-Normalisierung:** Kante `(A, tag 2)` und `(C, tag 2)` sind dieselbe
  physikalische Kante → beim Speichern (`visited_edges`) wird immer
  `[kleinerer_Knotenname, dessen_Tag]` verwendet.
- **`node_positions`:** Pixel-Koordinaten (Canvas 900×600) fürs Debug-Fenster.
  Leeres Dict `{}` → automatisches Kreislayout (Mittelpunkt 450/300, Radius
  220px).

---

## Tag-ID ↔ Wort-Übersetzung

Das ist der zentrale Kniff, der die neue Graph-Logik mit der unveränderten
Challenge-2-FSM verbindet:

- `explore_control_node`/`path_planner_node` denken in **Tag-IDs** (Graph-Keys,
  z.B. "welche Ausfahrt an Knoten A ist noch unbesucht?").
- `switch_control_node`/`control_intersection_node` erwarten weiterhin
  **Wörter** (`left`/`right`/`straight`), genau wie in Challenge 2.

Die Übersetzung übernimmt `graph_state_node`: es kennt sowohl den aktuell
sichtbaren **Einfahrt-Tag** (`/detect/apriltag/id`) als auch die feste
Einmündungs-Geometrie, und berechnet daraus für jede mögliche Ausfahrt am
aktuellen Knoten das passende Wort:

```
Ausfahrt-Tag = Einfahrt-Tag + 2   (mod 4)  ->  "straight"
Ausfahrt-Tag = Einfahrt-Tag + 1   (mod 4)  ->  "right"
Ausfahrt-Tag = Einfahrt-Tag - 1   (mod 4)  ->  "left"
```

Das Ergebnis wird als `/graph/exit_directions` publiziert (JSON-Dict
`{"<tag>": "<wort>", ...}`). `explore_control_node` und `path_planner_node`
lesen diesen Topic und übersetzen ihre Tag-ID-Entscheidung erst unmittelbar
vor dem Publizieren auf `/navigation/next_direction` in ein Wort – und zwar
**bei jedem Tick neu**, nicht einmalig zwischengespeichert, damit kleine
Zeitverzögerungen (neuer Knoten schon bekannt, aber Einfahrt-Tag der nächsten
Kreuzung noch nicht sichtbar) sich von selbst auflösen statt eine falsche
Entscheidung einzufrieren.

Ist der gewünschte Ausfahrt-Tag zufällig identisch mit dem aktuellen
Einfahrt-Tag (Sonderfall: der Bot müsste sofort wieder zurück durch dieselbe
Einmündung), gibt es dafür kein Wort (Offset 0) – diese Ausfahrt wird von der
DFS bewusst übersprungen (siehe `_first_actionable_exit` in
`explore_control_node.py`).

---

## Nodes

### graph_state_node — das Graph-Gedächtnis
Lädt `mapping_node.json` **direkt** (nicht über `util.init_parameters`, da
andere JSON-Struktur ohne `parameters`-Block). Verwaltet `current_node`,
`current_edge`, `visited_edges` und `gate_map`. Der Graph-Übergang wird genau
beim Wechsel von `/intersection/phase` nach `"Turning"` ausgelöst (garantiert
frische Richtung, siehe Kommentar im Code).
**Publiziert:** `/graph/current_node`, `/graph/current_edge`,
`/graph/visited_edges`, `/graph/gate_map`, `/graph/exit_directions`

### explore_control_node — Phase 1 (DFS)
Wählt an jeder Kreuzung die erste noch unbesuchte, aktuell wählbare Ausfahrt.
Sind alle Ausgänge eines Knotens besucht, sucht eine BFS über bereits
befahrene Kanten den nächsten Knoten mit unbesuchten Ausgängen und fährt nur
den ersten Schritt dorthin (wird bei der nächsten Ankunft neu berechnet).
Meldet `exploration_done = True`, sobald alle Kanten besucht sind, und gibt
danach die Kontrolle über `/navigation/phase` ab.
**Publiziert:** `/navigation/next_direction`, `/navigation/exploration_done`,
`/navigation/phase` (nur solange Phase 1 aktiv ist)

### path_planner_node — Phase 2+3 (Dijkstra + TSP)
Berechnet fortlaufend (sobald Phase 1 fertig ist) mit einer eigenen
Dijkstra-Implementierung (nur `heapq`) die optimale Reihenfolge aller
gefundenen Tore ab `delivery_start_node` – bei `mode: "optimal"` per
Brute-Force über alle Permutationen (`itertools.permutations`, automatischer
Fallback auf `nearest_neighbor` bei mehr als 10 Toren), sonst greedy nach
kürzester Distanz. Nach Knopfdruck (`start_delivery`) fährt es die Route ab
und erkennt abgefahrene Tore daran, dass `current_edge` mit einem
`gate_map`-Eintrag übereinstimmt.
**Publiziert:** `/navigation/next_direction`, `/navigation/phase` (nur
während Delivery), `/navigation/delivery_progress`

### debug_graph_node — tkinter-Dashboard
Zeigt den statischen Graphen, live wachsende grüne "befahren"-Kanten, den
geplanten Delivery-Pfad (blau gestrichelt mit Pfeilen), die aktuelle
Bot-Position (gelb) sowie Tor-Symbole mit Häkchen für bereits abgefahrene
Tore. Rechtes Panel zeigt Phase, Position, Fortschritt, gefundene Tore,
geplante Reihenfolge und den Start-Button. ROS-Callbacks aktualisieren
ausschließlich State-Variablen; gezeichnet wird nur im Hauptthread über
`root.after(200, update_canvas)`.
**Publiziert:** `/navigation/start_delivery` (bei Klick auf den Button)

### detect_apriltag_node (Erweiterung)
Zusätzlich zur unveränderten Kreuzungs-Tag-Logik (1–4) wird jeder erkannte
Tag im Bereich 5–13 als Tor-Tag ausgewertet (nur Mindestflächen-Filter, keine
Positions-/Stabilitäts-Filterung wie bei den Kreuzungs-Tags, da Tore an
beliebiger Stelle im Bild auftauchen können).
**Zusätzlich publiziert:** `/detect/gate/id` (Int32, -1 wenn keiner sichtbar)

### control_intersection_node (Umbau auf Encoder)
Segment-Ende wird nicht mehr über `duration` (Zeit), sondern über
Radencoder-Ticks bestimmt: Geradeaus-Segmente über den Mittelwert der
Ticks beider Räder, Dreh-Segmente über die Ticks-**Differenz** zwischen den
Rädern. `timeout` bleibt als Sicherheitsnetz erhalten, falls das Ticks-Ziel
nie erreicht wird. Tick-Referenzwerte werden bei **jedem** Segment-Start neu
gesetzt.

### switch_control_node (Anpassung)
Einzige fachliche Änderung: `random.choice(allowed_dirs)` wurde durch
`/navigation/next_direction` ersetzt. Ist die empfangene Richtung nicht (mehr)
in `allowed_dirs` enthalten, bleibt der Bot in `STOPPING` und wartet weiter –
**kein Fallback auf Zufall**, da Challenge 4 einen deterministischen Pfad
verlangt. Alle anderen FSM-Zustände sind unverändert.

---

## Topics

| Topic | Typ | Von → Nach |
|---|---|---|
| `/detect/lane` | Float64 | detect_lane → control_lane |
| `/detect/stop_line` | Bool | detect_lane → switch_control |
| `/detect/apriltag/direction` | String | detect_apriltag → switch_control |
| `/detect/apriltag/id` | Int32 | detect_apriltag → graph_state |
| `/detect/gate/id` | Int32 | detect_apriltag → graph_state |
| `/graph/current_node` | String | graph_state → explore, path_planner, debug |
| `/graph/current_edge` | String (JSON) | graph_state → path_planner, debug |
| `/graph/visited_edges` | String (JSON) | graph_state → explore, debug |
| `/graph/gate_map` | String (JSON) | graph_state → path_planner, debug |
| `/graph/exit_directions` | String (JSON) | graph_state → explore, path_planner |
| `/navigation/next_direction` | String (Wort) | explore / path_planner → switch_control |
| `/navigation/phase` | String | explore / path_planner → alle |
| `/navigation/exploration_done` | Bool | explore → debug |
| `/navigation/start_delivery` | Bool | debug (Button) → path_planner |
| `/navigation/delivery_progress` | String (JSON) | path_planner → debug |
| `/intersection/phase` | String | switch_control → control_intersection, graph_state |
| `/intersection/direction` | String | switch_control → control_intersection, graph_state |
| `/intersection/turn_done` | Bool | control_intersection → switch_control |
| `/enable/lane`, `/enable/intersection` | Bool | switch_control → control_lane / control_intersection |
| `/left_wheel_encoder_node/tick`, `/right_wheel_encoder_node/tick` | WheelEncoderStamped | Hardware → control_intersection |
| `/car_cmd_switch_node/cmd` | Twist2DStamped | control_lane / control_intersection → Motoren |

---

## Konfigurationsparameter

### mapping_node.json (kein `parameters`-Block, wird direkt geladen)

| Feld | Zweck |
|---|---|
| `graph` | Stadtgraph: `{Knoten: {Tag: [Nachbar, Nachbar-Tag]}}` |
| `mapping_start_node` | Startknoten für die Exploration (Phase 1) |
| `delivery_start_node` | Startknoten für die Delivery (darf abweichen) |
| `path_planning.mode` | `"optimal"` (Brute-Force) oder `"nearest_neighbor"` (Greedy) |
| `path_planning.fallback` | Modus, auf den bei >10 Toren automatisch gewechselt wird |
| `debug_layout.node_positions` | Pixel-Koordinaten je Knoten, `{}` = automatisches Kreislayout |

### control_intersection_node.json

| Parameter | Zweck |
|---|---|
| `turn_segments.left/right/straight` | Segment-Sequenz je Richtung: `{v, omega, ticks, timeout}` |

### detect_apriltag_node.json

| Parameter | Zweck |
|---|---|
| `tag_directions` | Kreuzungs-Tag (1–4) → erlaubte Richtungen (Wörter) |
| `tag_memory.seconds/min_area` | Wie lange ein naher Tag "gemerkt" wird |
| `tag_filter.*` | Stabilitäts-/Positionsfilter für Kreuzungs-Tags (gilt **nicht** für Tor-Tags) |

### switch_control_node.json

| Parameter | Zweck |
|---|---|
| `timing.stop_duration` | Haltezeit an der roten Linie |
| `timing.turning_timeout` | Sicherheits-Timeout fürs Abbiegen |

### control_lane_node.json / detect_lane_node.json

Wie in Challenge 2 (PID-Parameter bzw. HSV-Schwellen/Bird's-Eye-Eckpunkte);
enthalten jetzt nur noch `default` + `track` (alle anderen Bot-Blöcke
entfernt).

---

## Encoder-Kalibrierung

| Parameter | Wert |
|---|---|
| Topics | `/left_wheel_encoder_node/tick`, `/right_wheel_encoder_node/tick` |
| Typ | `duckietown_msgs/WheelEncoderStamped` (`data` = kumulative Ticks, `resolution` aus Message gelesen) |
| `wheel_radius` | 0.0318 m |
| `wheel_baseline` | 0.1 m |
| Strecke/Tick | ≈ 1.48 mm |
| Startwerte | 90°-Drehung ≈ 750 Ticks-Differenz, 0.3 m geradeaus ≈ 200 Ticks |

`data` zählt laut Hardware **immer aufwärts**, unabhängig von der
tatsächlichen Drehrichtung des Rads. `control_intersection_node.py` leitet
deshalb pro Segment aus `v`/`omega` (mit `wheel_baseline`) die *erwartete*
Drehrichtung jedes einzelnen Rads her und verrechnet den gemessenen
Ticks-Betrag mit diesem Vorzeichen, bevor die Geradeaus-/Dreh-Formel
angewendet wird – relevant z.B. beim scharfen Rechts-Segment, bei dem eines
der Räder rechnerisch leicht rückwärts läuft (siehe Kommentare in
`_wheel_signs`/`_segment_cmd`).

---

## Setup & Starten

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://track.local:11311
export VEHICLE_NAME=track

# Alles über den Launcher starten (empfohlen):
launchers/mapping.sh

# Oder einzeln (je ein Terminal):
rosrun mapping detect_lane_node.py
rosrun mapping detect_apriltag_node.py
rosrun mapping graph_state_node.py
rosrun mapping switch_control_node.py
rosrun mapping explore_control_node.py
rosrun mapping path_planner_node.py
rosrun mapping control_lane_node.py
rosrun mapping control_intersection_node.py
rosrun mapping debug_graph_node.py
```

---

## Vor dem Challenge-Tag

1. **`mapping_node.json` eintragen:** echter Graph der Strecke,
   `mapping_start_node`, `delivery_start_node`, `node_positions`
   (Pixel-Koordinaten fürs Debug-Fenster) und `path_planning.mode`
   (`"optimal"` für die Challenge, `"nearest_neighbor"` zum schnellen Testen).
2. **Encoder-Ticks kalibrieren:** Startwerte in `control_intersection_node.json`
   sind Schätzungen (siehe oben) – vor Ort mit echten Segmenten feintunen.
3. **Tor-Tag-Erkennung prüfen:** `detect_apriltag_node.json` nutzt für Tore
   denselben `tag_filter.min_area`-Schwellwert wie für Kreuzungs-Tags; je nach
   Tag-Größe/Anbringung ggf. separat justieren.
4. **Ablauf testen:** Alle Nodes starten → Debug-Fenster prüfen → Bot
   exploriert selbstständig → Tore erscheinen live im Dashboard → nach
   Exploration-Ende geplanten Pfad prüfen → "Delivery starten" drücken.

---

## Bekannte Einschränkungen

- Die Encoder-Logik (`control_intersection_node.py`) leitet für jedes Rad aus
  `v`/`omega` (mit `WHEEL_BASELINE = 0.1 m`) dessen erwartete Drehrichtung ab
  und verrechnet den (immer positiven) Ticks-Betrag mit diesem Vorzeichen –
  relevant z.B. beim scharfen Rechts-Segment (`v=0.15, omega=-3.2`), bei dem
  das rechte Rad rechnerisch leicht rückwärts läuft. `WHEEL_BASELINE` ist ein
  angenommener Konstantwert (nicht pro Bot kalibriert); bei spürbaren
  Abweichungen zwischen Bots ggf. anpassen. Die Herleitung gilt pro Segment
  (konstantes `v`/`omega` für die gesamte Segmentdauer) – ein Vorzeichenwechsel
  *innerhalb* eines Segments wird nicht unterstützt.
- `debug_graph_node`s Delivery-Pfad-Visualisierung (Ebene 3) berechnet die
  Route mit einer eigenen, unabhängigen Dijkstra-Instanz rein für die
  Darstellung – sie beeinflusst nicht die tatsächliche Fahrt (die kommt
  ausschließlich von `path_planner_node`).
- Bei mehr als 10 Toren wechselt die Planung automatisch von `"optimal"`
  (Brute-Force, sonst nicht mehr praktikabel) auf `"nearest_neighbor"`.
- `explore_control_node`s DFS kann sich in einem konstruierten Extremfall
  theoretisch festfahren: wenn die letzte verbleibende unbesuchte Ausfahrt an
  einem Knoten zufällig genau der Einmündung entspricht, über die der Bot
  gerade dort angekommen ist (U-Turn, siehe "Tag-ID ↔ Wort-Übersetzung"), UND
  gleichzeitig kein anderer Knoten mit unbesuchten Ausgängen über bereits
  befahrene Kanten erreichbar ist. Das kann nur an einer echten Sackgasse
  (Knoten mit nur einer einzigen Verbindung) auftreten; in allen getesteten
  Graphen (inkl. Wendeschleifen, siehe unten) löst es sich automatisch durch
  Backtracking auf.

**Wendeschleifen / Selbstschleifen (Knoten verbindet sich mit sich selbst,
z.B. ein Wendehammer-Loop wie auf manchen Strecken):** werden unterstützt.
Ein Knoten `H` mit `"3": ["H", "4"]` (Tag 3 führt über die Schleife zurück zu
`H` selbst, Ankunft über Tag 4) wird von allen drei Nodes (`graph_state_node`,
`explore_control_node`, `debug_graph_node`) korrekt als **eine** Kante
behandelt, die durch Befahren in EINER Richtung als vollständig erkundet gilt
– genau wie bei einer normalen Kante zwischen zwei Knoten. Ohne diese
Sonderbehandlung (Normalisierung nach Tag statt nach Knotenname, da beide
Enden derselbe Knoten sind) würden beide Richtungen der Schleife fälschlich
als zwei separate, unabhängig zu besuchende Kanten gezählt – das führt direkt
in einen Deadlock, da die zweite "Kante" exakt dem Tag entspricht, über den
der Bot gerade angekommen ist (U-Turn-Ausschluss, siehe oben).

Im `debug_graph_node`-Dashboard wird eine Selbstschleife als kleiner Kreis
oberhalb des Knotens gezeichnet (statisch grau, befahren grün, im geplanten
Delivery-Pfad blau gestrichelt) statt als Linie – eine Linie von einem
Knoten zu sich selbst hätte die Länge 0 und wäre unsichtbar gewesen. Tor-Symbole
auf einer Schleifen-Kante werden entsprechend auf diesem Loop-Kreis platziert,
nicht auf dem Knoten selbst.
