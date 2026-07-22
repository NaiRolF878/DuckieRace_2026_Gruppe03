# Challenge 4 – Mapping & Path Finding

> ROS 1 (Noetic) · Ubuntu 20.04 · Python 3 · OpenCV · pupil_apriltags · tkinter

Der Duckiebot bekommt einen Stadtgraphen als JSON. In **Phase 1 (Mapping)**
fährt er selbstständig (DFS) alle Kanten des Graphen ab und findet dabei
farbige **Tore** (AprilTag-IDs 5–13), die er auf Graphenkanten mappt. In
**Phase 2 (Planung)** wird automatisch die optimale Reihenfolge berechnet, in
der alle Tore angefahren werden sollen – **außer** im Debug-Fenster wurde eine
**vorgegebene Reihenfolge** eingetragen (z.B. weil die Challenge eine feste
Abliefer-Reihenfolge vorschreibt): dann wird diese Reihenfolge unverändert
übernommen und nur noch der kürzeste Weg *zwischen* den vorgegebenen
Stationen per Dijkstra berechnet, nicht mehr die Reihenfolge selbst. Nach
manueller Bestätigung im Debug-Fenster fährt der Bot in **Phase 3
(Delivery)** diese Reihenfolge ab – **nur Phase 3 wird auf Zeit bewertet**,
daher ist gute Pfadoptimierung entscheidend.

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
- [Setup & Starten](#setup--starten)
- [Vor dem Challenge-Tag](#vor-dem-challenge-tag)
- [Bekannte Einschränkungen](#bekannte-einschränkungen)

---

## Dateien

| Datei | Typ | Beschreibung |
|---|---|---|
| `detect_lane_node.py` | Node | Spurerkennung (zeilenbasierte Sobel-Kantenerkennung) + rote Haltelinie in einer Node (ein Bild-Decode/Warp pro Frame) |
| `detect_apriltag_node.py` | Node | Kreuzungs-Tags (1–4) **+ Tor-Tags (5–13)** |
| `switch_control_node.py` | Node | FSM – Richtung kommt jetzt von `next_direction`, kein `random.choice` mehr |
| `control_lane_node.py` | Node | PID-Spurregler mit Kurven-Drosselung (Neuaufbau nach `explore_duckietown_ii`-Vorbild) |
| `control_intersection_node.py` | Node | Fährt die Kreuzung – zeitbasierte Segmente (`v`/`omega`/`duration`), Start ereignisgesteuert über `/intersection/turn_start` |
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
   control_lane              control_intersection (zeitbasiert, wie C2)
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
`current_edge`, `visited_edges` und `gate_map`. Der Graph-Übergang wird durch
`/intersection/turn_start` ausgelöst (nicht mehr durch den Wechsel von
`/intersection/phase` nach `"Turning"` – das frühere Vertrauen auf "phase
wird kurz vor direction publiziert" hat in der Praxis trotzdem eine
veraltete Richtung der vorherigen Abbiegung durchrutschen lassen, siehe
`fehler.md`: physische Fahrt korrekt, im Graph gebuchte Kante nicht).
`turn_start` liefert Start-Signal + bestätigte Richtung atomar in einer
Nachricht, wie bei `control_intersection_node`.

Verfolgt zusätzlich `predicted_entry_tag`: den Eingangs-Tag der Kreuzung, an
der der Bot gerade steht, rein aus der eigenen Kartenverfolgung berechnet
(bereits bei der vorherigen Abbiegeentscheidung bekannt, lange bevor die
Kamera überhaupt etwas sehen muss). Kann die Kamera den Tag diesmal nicht
lesen (`current_entry_tag` bleibt `None`), springt `_effective_entry_tag()`
auf diese Vorhersage ein – ohne diesen Fallback bliebe `exit_directions`
leer und `next_direction` dauerhaft `""` (garantierter Deadlock in
`switch_control_node`, siehe dort).

**Widerspruchsprüfung:** Liest die Kamera einen Tag, der der Vorhersage
widerspricht, wird der Vorhersage vertraut, nicht der Kamera. Kreuzungs-Tags
1/3 (bzw. 2/4) liegen sich geometrisch gegenüber – wird z.B. der eigentliche
Einfahrt-Tag nur mit Bitfehlern gelesen und deshalb verworfen (siehe
`detect_apriltag_node`), während gleichzeitig der gegenüberliegende Tag
sauber lesbar ist, würde die Kamera sonst einen technisch "gültigen", aber
falschen Tag liefern – mit Folgefehler in der Wort-Übersetzung (z.B. "links"
statt korrekt "rechts").
**Publiziert:** `/graph/current_node`, `/graph/current_edge`,
`/graph/visited_edges`, `/graph/gate_map`, `/graph/exit_directions`,
`/graph/allowed_directions` (Fallback-Quelle für `switch_control_node`)

**Notfall-Korrektur der Tor-Zuordnung:** `gate_map` wird bei jedem neu
entdeckten Tor automatisch nach `mapping_node.json` (Feld `"gate_map"`)
zurückgeschrieben. Ein dort bereits vorhandener Eintrag wird von einer
späteren Live-Erkennung nie überschrieben (siehe `cbGateId`). Damit lässt
sich ein falsch erkanntes Tor mitten im Lauf korrigieren: Datei von Hand
editieren (ID tauschen/Eintrag entfernen), im Dashboard auf "Tor-Zuordnung
neu laden" klicken (`/graph/reload_gate_map`) – `gate_map` wird komplett
durch den neu eingelesenen Stand ersetzt, `current_node`/`visited_edges`
bleiben unberührt (kein Neustart der Exploration nötig).

### explore_control_node — Phase 1 (DFS)
Wählt an jeder Kreuzung die erste noch unbesuchte, aktuell wählbare Ausfahrt.
Sind alle Ausgänge eines Knotens besucht, sucht eine BFS über den **vollen**
Graphen (nicht nur bereits befahrene Kanten – das führte zu einem
Deadlock, siehe "Bekannte Einschränkungen") den nächsten Knoten mit
unbesuchten Ausgängen und fährt nur den ersten Schritt dorthin (wird bei der
nächsten Ankunft neu berechnet).
Meldet `exploration_done = True`, sobald alle Kanten besucht sind, leert
dabei explizit `next_direction` (verhindert, dass eine veraltete Richtung
von der letzten Abbiegung zufällig zur nächsten Kreuzung passt und der Bot
ungewollt weiterfährt) und gibt danach die Kontrolle über `/navigation/phase`
ab. `debug_graph_node` zeigt in diesem Moment ein Popup ("Erkundung
abgeschlossen").
**Publiziert:** `/navigation/next_direction`, `/navigation/exploration_done`,
`/navigation/phase` (nur solange Phase 1 aktiv ist)

### path_planner_node — Phase 2+3 (Dijkstra + TSP)
Berechnet fortlaufend (sobald Phase 1 fertig ist) mit einer eigenen
Dijkstra-Implementierung (nur `heapq`) die Reihenfolge aller gefundenen Tore
ab `delivery_start_node`. Ist `path_planning.gate_order` (Config oder live
per `/navigation/gate_order` vom Dashboard) **nicht leer**, wird diese
Reihenfolge unverändert übernommen (`_plan_fixed_order`) – sonst wird sie
selbst optimiert: bei `mode: "optimal"` per Brute-Force über alle
Permutationen (`itertools.permutations`, automatischer Fallback auf
`nearest_neighbor` bei mehr als 10 Toren), sonst greedy nach kürzester
Distanz. In beiden Fällen berechnet Dijkstra die kürzesten Wege *zwischen*
den (vorgegebenen oder optimierten) Stationen. Nach Knopfdruck
(`start_delivery`) fährt es die Route ab und erkennt abgefahrene Tore daran,
dass `current_edge` mit einem `gate_map`-Eintrag übereinstimmt.
**Publiziert:** `/navigation/next_direction`, `/navigation/phase` (nur
während Delivery), `/navigation/delivery_progress`

### debug_graph_node — tkinter-Dashboard
Zeigt den statischen Graphen, live wachsende grüne "befahren"-Kanten, den
geplanten Delivery-Pfad (blau gestrichelt mit Pfeilen), die aktuelle
Bot-Position (gelb) sowie Tor-Symbole mit Häkchen für bereits abgefahrene
Tore. Rechtes Panel zeigt Phase, Position, Fortschritt, gefundene Tore,
geplante Reihenfolge, ein Eingabefeld für eine **vorgegebene Tor-Reihenfolge**
(kommagetrennte Gate-IDs, z.B. `5,9,3` – schreibt in `mapping_node.json`
zurück und publiziert live auf `/navigation/gate_order`), einen Button
**"Tor-Zuordnung neu laden"** (Notfall-Korrektur, siehe `graph_state_node`),
einen Button **"Erkundung neu starten"** (setzt `visited_edges` zurück,
`gate_map` bleibt – für den Fall, dass ein Tor übersehen wurde) und den
Start-Button. Zeigt außerdem einmalig ein Popup ("Erkundung abgeschlossen"),
sobald `exploration_done` auf `True` wechselt. ROS-Callbacks aktualisieren
ausschließlich State-Variablen; gezeichnet wird nur im Hauptthread über
`root.after(200, update_canvas)`.
**Publiziert:** `/navigation/start_delivery` (bei Klick auf den Button),
`/graph/reload_gate_map` (bei Klick auf "Tor-Zuordnung neu laden"),
`/graph/reset_exploration` (bei Klick auf "Erkundung neu starten")

### tools/preview_mapping_graph.py — Vorschau ohne ROS
Standalone-Skript (kein rospy-Import, laeuft mit reinem `python3`), zeichnet
Knoten/Kanten/Tor-Zuordnung aus einer `mapping_node.json` exakt wie
`debug_graph_node`, um eine vor Ort editierte Datei (neue Kreuzung,
verschobenes Tor) kurz zu pruefen, ohne den ROS-Stack hochzufahren:
```
python3 src/packages/mapping/tools/preview_mapping_graph.py [pfad/zur/mapping_node.json]
```
Ohne Argument wird die Datei aus `config/` verwendet. Meldet auf der Konsole
Referenzfehler (z.B. Tippfehler bei Knotennamen in `graph`, `debug_layout`
oder `gate_map`) und legt zusaetzlich einen Screenshot `..._preview.png`
neben der json-Datei ab.

### detect_lane_node / control_lane_node (Neuaufbau)
Ersetzt die Challenge-2-Implementierung, weil die Spurführung auf dieser
Strecke unzuverlässig war (Spur verloren, unerklärliches Stehenbleiben) –
neu aufgebaut nach dem Vorbild eines anderen Teams. Übernommen wurde bewusst
**nur** die Wahrnehmungs-/Regelungs-Ebene, nicht deren Navigationskonzept
(dort ein vorab berechneter Plan statt Live-Graph-Verfolgung) –
`graph_state_node`, `explore_control_node`, `path_planner_node`,
`control_intersection_node` und `switch_control_node` bleiben unverändert,
die Topic-Schnittstellen (`/detect/lane`, `/detect/stop_line`, `/enable/lane`)
ebenfalls.

- **`detect_lane_node`**: zeilenbasierte Sobel-Kantenerkennung statt der
  bisherigen Ankerpunkt-Logik. Gelb wird zuerst auf einer festen
  `detection_row_factor`-Zeile gesucht, die weiße Maske wird links von
  "Gelb + `min_lane_width`" ausgeblendet (Korridor-Filter gegen die
  Gegenspur), und die Suchzeile für Weiß wandert schrittweise nach unten,
  falls dort nichts gefunden wird. Haltelinie (Rot-Pixel-Schwellwert in einer
  ROI, unteres Bilddrittel) läuft bewusst **in derselben Node** statt in
  einer eigenen – ein Bild-Decode/Bird's-Eye-Warp pro Frame statt zwei
  (Performance auf der begrenzten Rechenleistung des Bots).
- **`control_lane_node`**: gleicher PID wie zuvor, zusätzlich
  `speed_curve_factor`/`min_speed_factor` – die Geschwindigkeit sinkt jetzt
  gezielt mit dem Betrag des Spurfehlers (Kurven-Drosselung) statt nur linear
  über `min_vel` begrenzt zu werden.
  `/debug/lane_croped` zeigt zusätzlich eine Bounding-Box der
  Haltelinien-Erkennungszone (rot/dick wenn erkannt, sonst dünner Umriss zur
  Kalibrierung).

### detect_apriltag_node (Erweiterung)
Zusätzlich zur unveränderten Kreuzungs-Tag-Logik (1–4) wird jeder erkannte
Tag im Bereich 5–13 als Tor-Tag ausgewertet (nur Mindestflächen-Filter, keine
Positions-/Stabilitäts-Filterung wie bei den Kreuzungs-Tags, da Tore an
beliebiger Stelle im Bild auftauchen können).

**Hamming-Filter (`tag.hamming == 0`):** akzeptiert nur exakt dekodierte Tags,
für Kreuzungs- **und** Tor-Tags. Ein Tag, der nur mit Bitfehler-Korrektur
lesbar war, kann eine ANDERE, ebenfalls gültige ID ergeben (z.B. echte ID 2
wird als 3 gelesen) – und bleibt dabei über mehrere Frames stabil falsch
(kein Rauschen, das sich rausmittelt). Genau das führt zu einer falsch
aufgezeichneten Kante im Graph-Modell (Bot fährt z.B. tatsächlich A2→C,
`graph_state_node` bucht es aber als A3→C).

`/debug/apriltag` zeigt jetzt auch für erkannte Tor-Tags eine Bounding-Box
(pink, direkt am Tag statt nur als Text an fester Position) und wurde
entschlackt – die Tag-Gedächtnis-Interna (Alter, Quelle) werden nicht mehr
eingeblendet, nur noch Tag-ID, erlaubte Richtungen und gewählte Fahrtrichtung.
**Zusätzlich publiziert:** `/detect/gate/id` (Int32, -1 wenn keiner sichtbar)

### control_intersection_node
Segment-Logik 1:1 wie in `intersection_handling`: jedes Segment gibt
`{v, omega, duration}` vor, Segment-Ende wird über die **kumulierte**
Segmentdauer seit Start der Sequenz bestimmt (kein Timeout nötig, da rein
zeitgesteuert). Ein zwischenzeitlicher Umbau auf encoder-basierte (Ticks)
Segment-Enden wurde wieder rückgängig gemacht, da er auf dieser Strecke
nicht zuverlässig funktionierte.

**Start-Trigger (`/intersection/turn_start`, nicht mehr Polling):** Bündelt
Richtung + Start-Zeitpunkt in einer Nachricht. Vorher wurde der Start der
Sequenz durch Beobachten von `/intersection/phase == "Turning"` erkannt,
während die Richtung separat über `/intersection/direction` kam – zwei
unabhängige Topics ohne garantierte Verarbeitungsreihenfolge. Kam die
`phase`-Nachricht zuerst an, startete die Sequenz mit der Richtung der
**vorherigen** Kreuzung (Bot bog z.B. rechts ab, obwohl "geradeaus" geplant
war). `switch_control_node` legt die Richtung beim Übergang Stopping→Turning
fest und ändert sie danach nie wieder – `control_intersection_node` wartet
jetzt einfach (Motor aus), bis `turn_start` eintrifft, statt mit einer
möglicherweise veralteten Richtung loszufahren.

### switch_control_node (Anpassung)
Einzige fachliche Änderung: `random.choice(allowed_dirs)` wurde durch
`/navigation/next_direction` ersetzt. Ist die empfangene Richtung nicht (mehr)
in `allowed_dirs` enthalten, bleibt der Bot in `STOPPING` und wartet weiter –
**kein Fallback auf Zufall**, da Challenge 4 einen deterministischen Pfad
verlangt. Alle anderen FSM-Zustände sind unverändert.

**Graph-Fallback gegen permanentes Steckenbleiben:** `allowed_dirs` stammt
normalerweise aus der Live-Tag-Erkennung. Kann die Kamera den Tag an dieser
Kreuzung gar nicht (mehr) lesen, würde der Bot sonst für immer in `STOPPING`
warten (z.B. wenn eine veraltete Richtung "im Speicher" hängen bleibt). Die
Node weicht daher auf `/graph/allowed_directions` (von `graph_state_node`,
rein aus der Kartenverfolgung vorhergesagt) aus: sofort, falls beim
`STOPPING`-Eintritt gar keine Live-Richtung vorliegt, sonst nach
`stopping_fallback_timeout` Sekunden (Default 6.0, Config `timing`), falls
die eingefrorene Live-Richtung nicht zu `next_direction` passt.

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
| `/graph/allowed_directions` | String (Worte, kommagetrennt) | graph_state → switch_control (Fallback) |
| `/graph/reload_gate_map` | Bool | debug (Button) → graph_state (Notfall-Korrektur) |
| `/graph/reset_exploration` | Bool | debug (Button) → graph_state, explore (Erkundung wiederholen) |
| `/navigation/next_direction` | String (Wort) | explore / path_planner → switch_control |
| `/navigation/phase` | String | explore / path_planner → alle |
| `/navigation/exploration_done` | Bool | explore → debug |
| `/navigation/start_delivery` | Bool | debug (Button) → path_planner |
| `/navigation/delivery_progress` | String (JSON) | path_planner → debug |
| `/intersection/phase` | String | switch_control → control_intersection, graph_state |
| `/intersection/direction` | String | switch_control → (nur historisch, kein Abonnent mehr) |
| `/intersection/turn_start` | String (Wort) | switch_control → control_intersection, graph_state, debug (einmalig pro Abbiegung) |
| `/intersection/turn_done` | Bool | control_intersection → switch_control |
| `/enable/lane`, `/enable/intersection` | Bool | switch_control → control_lane / control_intersection |
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
| `turn_segments.left/right/straight` | Segment-Sequenz je Richtung: `{v, omega, duration}` |

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

### control_lane_node.json

| Parameter | Zweck |
|---|---|
| `pid.p/i/d` | PID-Verstärkung auf den Spurfehler |
| `pid.max_vel` | Grundgeschwindigkeit auf gerader Strecke |
| `pid.speed_curve_factor` | Wie stark `v` mit `\|error\|` sinkt (Kurven-Drosselung) |
| `pid.min_speed_factor` | Untergrenze der Drosselung (Anteil von `max_vel`), verhindert Stillstand in engen Kurven |

### detect_lane_node.json

| Parameter | Zweck |
|---|---|
| `crop_image.*` | Bird's-Eye-Eckpunkte (Perspektivtransformation) |
| `detection_row_factor` | Zeile (Anteil der Bildhöhe), auf der Weiß gesucht wird – wandert nach unten, falls dort nichts gefunden wird |
| `min_lane_width` | Korridor-Filter: blendet die weiße Maske links von "Gelb + `min_lane_width`" aus (gegen die Gegenspur) |
| `white.*` / `yellow.*` | HSV-Schwellen der beiden Linienfarben |
| `red1.*` / `red2.*` | HSV-Schwellen für Rot (Haltelinie, liegt an zwei Stellen des Hue-Kreises) |
| `detection.thresh` | Rot-Pixel-Schwellwert in der ROI, ab dem die Haltelinie als erkannt gilt |

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
2. **Abbiege-Segmente kalibrieren:** `turn_segments` in
   `control_intersection_node.json` sind zeitbasiert (`v`/`omega`/`duration`,
   wie in Challenge 2) – vor Ort an die tatsächliche Strecke feintunen.
3. **Tor-Tag-Erkennung prüfen:** `detect_apriltag_node.json` nutzt für Tore
   denselben `tag_filter.min_area`-Schwellwert wie für Kreuzungs-Tags; je nach
   Tag-Größe/Anbringung ggf. separat justieren.
4. **Ablauf testen:** Alle Nodes starten → Debug-Fenster prüfen → Bot
   exploriert selbstständig → Tore erscheinen live im Dashboard → nach
   Exploration-Ende geplanten Pfad prüfen → "Delivery starten" drücken.

---

## Bekannte Einschränkungen

- `control_intersection_node` ist rein zeitbasiert (`duration` je Segment) –
  driftet daher wie in Challenge 2 mit Akkustand/Bodenhaftung/Temperatur;
  `turn_segments` müssen vor Ort an die tatsächliche Strecke kalibriert
  werden (kein automatischer Ausgleich).
- `debug_graph_node`s Delivery-Pfad-Visualisierung (Ebene 3) berechnet die
  Route mit einer eigenen, unabhängigen Dijkstra-Instanz rein für die
  Darstellung – sie beeinflusst nicht die tatsächliche Fahrt (die kommt
  ausschließlich von `path_planner_node`).
- Bei mehr als 10 Toren wechselt die Planung automatisch von `"optimal"`
  (Brute-Force, sonst nicht mehr praktikabel) auf `"nearest_neighbor"`.
- ~~`explore_control_node`s DFS kann sich festfahren, wenn kein anderer Knoten
  mit unbesuchten Ausgängen über bereits befahrene Kanten erreichbar ist~~ –
  **behoben**: `_find_backtrack_path()` sucht jetzt über den **vollen**
  Graphen (komplett aus `mapping_node.json` bekannt), nicht mehr nur über
  bereits befahrene Kanten. Die alte Einschränkung war kein rein
  theoretisches Randproblem, sondern ist beim Testen aufgetreten (Bot blieb
  dauerhaft in `STOPPING`, `next_direction` leer, siehe `fehler.md`). Für
  genau diesen Fall gibt es jetzt zusätzlich den Button "Erkundung neu
  starten" im Dashboard (`/graph/reset_exploration`) – setzt nur
  `visited_edges` zurück, `gate_map` (gefundene Tore) bleibt erhalten.

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
