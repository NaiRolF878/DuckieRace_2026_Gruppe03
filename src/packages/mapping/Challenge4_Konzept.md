# Challenge 4 – Mapping & Path Finding
## Konzept-Dokument

---

## Ziel

Der Duckiebot bekommt einen Stadtgraphen als JSON. Er fährt systematisch alle
Kanten ab (Mapping), findet dabei farbige Tore (AprilTag-IDs 5–13) und mappt
sie auf Graphenkanten. Anschließend berechnet er automatisch den optimalen Pfad
und fährt alle Tore in der richtigen Reihenfolge ab (Delivery).

**Nur Delivery wird auf Zeit bewertet** → Pfadoptimierung ist entscheidend.

---

## Die drei Phasen

| Phase | Name | Bewertet? | Beschreibung |
|---|---|---|---|
| 1 | Mapping | Ja (Vollständigkeit) | Alle Kanten abfahren, Tore finden und auf Kanten mappen |
| 2 | Planung | Nein | Optimalen Pfad berechnen, manuell bestätigen |
| 3 | Delivery | Ja (Zeit!) | Tore in optimaler Reihenfolge anfahren |

Übergang Phase 1 → 3 nur nach **manueller Bestätigung** im Debug-Fenster.

---

## Graph-Format

### Eigenschaften

- **Ungerichtet und symmetrisch:** Spur und Gegenspur haben denselben Tag.
  `A: { "2": ["C","2"] }` bedeutet auch `C: { "2": ["A","2"] }`.
- **Einmündungs-Nummern sind physikalisch fest:**
  - Tag 1 und Tag 3 liegen sich gegenüber (geradeaus)
  - Tag 2 ist rechts von Tag 1
  - Tag 4 ist links von Tag 1 (= rechts von Tag 3)
- **Kreuzungs-Tags:** IDs 1–4 (an Kreuzungen, codieren Einmündung)
- **Tor-Tags:** IDs 5–13 (auf Kanten, codieren Zielorte)

### JSON-Format

```json
{
  "graph": {
    "A": {
      "1": ["B", "1"],
      "2": ["C", "2"],
      "3": ["C", "1"],
      "4": ["B", "2"]
    },
    "B": {
      "1": ["A", "1"],
      "2": ["A", "4"],
      "3": ["C", "4"]
    },
    "C": {
      "1": ["A", "3"],
      "2": ["A", "2"],
      "4": ["B", "3"]
    }
  },
  "mapping_start_node": "A",
  "delivery_start_node": "A",
  "path_planning": {
    "mode": "optimal",
    "fallback": "nearest_neighbor"
  },
  "debug_layout": {
    "node_positions": {
      "A": [450, 200],
      "B": [700, 450],
      "C": [200, 450]
    }
  }
}
```

### Lese-Schlüssel

`"A": { "2": ["C", "2"] }` bedeutet:
An Kreuzung A, Tag 2 gesehen → Kante führt zu Knoten C → dort kommt
der Bot über Tag 2 an (symmetrisch).

### node_positions – Pixel-Koordinaten

Das Debug-Fenster hat eine Canvas von **900×600 Pixel**.
Koordinaten gehen von oben-links (0,0) nach unten-rechts (900,600).

| Position auf Strecke | Koordinaten |
|---|---|
| Links oben | `[80, 80]` |
| Mitte oben | `[450, 80]` |
| Rechts oben | `[820, 80]` |
| Links Mitte | `[80, 300]` |
| Mitte | `[450, 300]` |
| Rechts Mitte | `[820, 300]` |
| Links unten | `[80, 520]` |
| Mitte unten | `[450, 520]` |
| Rechts unten | `[820, 520]` |

Falls `node_positions` leer `{}` → automatisches Kreislayout
(Knoten gleichmäßig auf Kreis, Radius 220px, Mittelpunkt 450/300).

---

## Encoder-Kalibrierung (Bot: `track`, bereits verifiziert)

| Parameter | Wert |
|---|---|
| Topic links | `/track/left_wheel_encoder_node/tick` |
| Topic rechts | `/track/right_wheel_encoder_node/tick` |
| Typ | `duckietown_msgs/WheelEncoderStamped` |
| `resolution` | 135 Ticks/Umdrehung (aus Message-Feld lesen) |
| `wheel_radius` | 0.0318 m |
| `wheel_baseline` | 0.1 m |
| Strecke/Tick | ≈ 1.48 mm |
| Drehwinkel | `Δθ = (s_rechts − s_links) / baseline` |
| Richtung | `data` zählt IMMER aufwärts – Richtung aus v/omega ableiten |
| Fahrbefehl | `/track/car_cmd_switch_node/cmd` (`Twist2DStamped`) |

---

## Node-Architektur

### Neue Nodes

```
graph_state_node        – Graphen-Zustand zur Laufzeit (Fundament)
explore_control_node    – Phase 1: DFS über alle Kanten
path_planner_node       – Phase 2+3: optimaler Pfad + Delivery
debug_graph_node        – tkinter Dashboard: Graph, Pfad, Bot-Position
```

### Angepasste Nodes

```
switch_control_node          – Richtung von /navigation/next_direction statt random
control_intersection_node    – Encoder (ticks + timeout) statt Zeit (duration)
detect_apriltag_node         – Tor-Tags 5–13 → /detect/gate/id
```

### Vollständige Topic-Übersicht (neu)

| Topic | Typ | Publisher | Subscriber |
|---|---|---|---|
| `/track/graph/current_node` | `String` | graph_state_node | alle |
| `/track/graph/current_edge` | `String` (JSON) | graph_state_node | alle |
| `/track/graph/visited_edges` | `String` (JSON) | graph_state_node | explore, debug |
| `/track/graph/gate_map` | `String` (JSON) | graph_state_node | path_planner, debug |
| `/track/navigation/next_direction` | `String` | explore / path_planner | switch_control |
| `/track/navigation/phase` | `String` | explore / path_planner | alle |
| `/track/navigation/exploration_done` | `Bool` | explore_control | debug |
| `/track/navigation/start_delivery` | `Bool` | debug (Button) | path_planner |
| `/track/navigation/delivery_progress` | `String` (JSON) | path_planner | debug |
| `/track/detect/gate/id` | `Int32` | detect_apriltag | graph_state |

---

## Node-Details

### graph_state_node

Verwaltet den kompletten Graphen-Zustand zur Laufzeit.

**Zustand:**
- `current_node` – zuletzt bestätigte Kreuzung (via AprilTag 1–4)
- `current_edge` – aktuell befahrene Kante `{"from": "A", "tag": "2"}`
- `visited_edges` – Liste aller besuchten Kanten `[["A","2"], ...]`
- `gate_map` – gefundene Tore `{"5": {"node": "A", "tag": "2"}, ...}`

**Logik (ungerichtet/symmetrisch):**
1. Start: `current_node = mapping_start_node` aus JSON
2. AprilTag 1–4 erkannt → in `graph[current_node]` nachschlagen →
   neuer `current_node` bekannt, alte Kante als besucht markieren
3. Bot verlässt Kreuzung → `current_edge` setzen
4. Tor-Tag 5–13 erkannt + `current_edge` gesetzt →
   in `gate_map` eintragen (nur einmal pro Kante)

Da der Graph symmetrisch ist: Kante `(A, tag 2)` und `(C, tag 2)` sind
dieselbe physikalische Kante → beim Eintragen normalisieren
(immer den lexikographisch kleineren Knoten zuerst).

---

### explore_control_node

Steuert Phase 1 (Mapping) mit DFS.

**Logik:**
1. Aktiv nur wenn `phase == "exploration"`
2. An jeder Kreuzung: alle Ausgänge von `current_node` prüfen
3. Ersten Ausgang wählen dessen Kante noch nicht in `visited_edges` → publishen
4. Alle Ausgänge besucht: Backtrack (BFS über bekannte Kanten zurück zu
   Knoten mit unbesuchten Ausgängen)
5. Alle Kanten besucht: `exploration_done = True`, `phase = "waiting"`

---

### path_planner_node

Phase 2 (Planung) und Phase 3 (Delivery).

**Dijkstra:**
Berechnet alle paarweisen kürzesten Distanzen zwischen relevanten Knoten
(Delivery-Startknoten + alle Tor-Positionen). Kantengewicht = 1.

**Planungs-Modi:**

| Modus | Beschreibung | Komplexität |
|---|---|---|
| `optimal` | Brute-Force alle Permutationen | 9! = 362.880 → ~0.3s auf RPi |
| `nearest_neighbor` | Greedy TSP-Heuristik | O(n²), Fallback |

Bei `mode: "optimal"` und mehr als 10 Toren automatisch auf
`nearest_neighbor` wechseln (Sicherheitsgrenze).

**Delivery-Logik:**
1. Warte auf `start_delivery = True`
2. Setze `current_node = delivery_start_node` aus JSON
3. Berechne optimale Reihenfolge der Tore
4. Folge Dijkstra-Pfad, published an jeder Kreuzung die nächste Tag-ID
5. Tor gilt als abgefahren wenn Bot die entsprechende Kante befährt
6. `delivery_progress` laufend aktualisieren

---

### debug_graph_node (tkinter Dashboard)

**Fenster-Layout:** 1200×650px gesamt

**Canvas links (900×600px) – drei überlagerte Ebenen:**

| Ebene | Farbe | Inhalt |
|---|---|---|
| 1 (statisch) | Grau, 2px | Alle Graphkanten aus JSON |
| 2 (Mapping) | Grün, 3px | Abgefahrene Kanten (wächst live) |
| 3 (Delivery) | Blau gestrichelt, 3px | Geplanter Delivery-Pfad |

**Zusätzlich auf Canvas:**
- Kreise (r=20): alle Knoten, grau gefüllt
- Hervorgehobener Kreis (r=24, gelb): aktueller Bot-Knoten
- Knotenname als weißer Text in Kreisen
- Tor-Symbole auf Kanten: farbiges Rechteck (12×20px) mit Gate-ID
  Farben: 5=Magenta, 6=Cyan, 7=Orange, 8=Gelb, 9=Rot, 10=Lila,
          11=Hellgrün, 12=Pink, 13=Türkis
- Bereits abgefahrene Tore: Häkchen-Symbol über dem Tor-Rechteck

**Status-Panel rechts (280px):**
- Phase (Exploration / Waiting / Delivery)
- Aktueller Knoten (Bot-Position)
- Aktuelle Kante
- Besuchte Kanten: X / Y (Gesamt)
- Gefundene Tore: Liste mit Kanten-Zuordnung
- Geplante Reihenfolge (nach Planung)
- **Button "Delivery starten"** – grün, nur aktiv wenn `exploration_done`
- Abgefahrene Tore (Phase 3, mit Häkchen)

**Update:** `root.after(200, update_canvas)` – 5 Hz, nur im Hauptthread.

---

### control_intersection_node (Encoder-Umbau)

**Neues Segment-Format:**
```json
{
  "v": 0.2,
  "omega": 2.5,
  "ticks": 450,
  "timeout": 4.0
}
```

**Encoder-Logik:**
- Tick-Referenz bei **jedem Segment-Start** neu setzen
- Geradeaus: `(Δlinks + Δrechts) / 2 >= ticks`
- Drehen: `|Δrechts − Δlinks| >= ticks`
- Richtung aus Vorzeichen v/omega ableiten (data zählt immer aufwärts)
- `timeout` als Sicherheitsnetz

**Richtwerte für erste Kalibrierung:**
- 90° Drehung ≈ 750 Ticks-Differenz
- 0.3 m geradeaus ≈ 200 Ticks

---

### switch_control_node (Anpassung)

Einzige Änderung: Abonniert `/track/navigation/next_direction`.
Nimmt diese Richtung wenn sie in `allowed_dirs` enthalten ist.
Kein Fallback auf `random.choice` in Challenge 4.

---

### detect_apriltag_node (Anpassung)

Zusätzlicher Publisher `/track/detect/gate/id` (`Int32`):
- Wert = Tag-ID wenn ID in 5–13, sonst -1
- Bestehende Richtungslogik (Tags 1–4) bleibt unverändert

---

## JSON-Vereinfachung bestehender Config-Files

Aus allen bestehenden JSON-Configs alle Bot-Blöcke außer `default`
und `track` entfernen.

Betrifft:
- `control_lane_node.json`
- `detect_lane_node.json`
- `detect_apriltag_node.json`
- `switch_control_node.json`
- `control_intersection_node.json`

---

## Implementierungsreihenfolge

1. JSON-Bereinigung (bestehende Configs)
2. `control_intersection_node.py` – Encoder-Umbau
3. `detect_apriltag_node.py` – Tor-Erkennung (IDs 5–13)
4. `mapping_node.json` – neue Config erstellen
5. `graph_state_node.py`
6. `explore_control_node.py`
7. `switch_control_node.py` – Anpassung
8. `path_planner_node.py`
9. `debug_graph_node.py`

---

## Hinweise für den Challenge-Tag

**Vor dem Start in `mapping_node.json` eintragen:**
- `mapping_start_node`: Startknoten für die Exploration
- `delivery_start_node`: Startknoten für die Delivery (darf abweichen)
- `node_positions`: Pixel-Koordinaten der Kreuzungen im Debug-Fenster
- `path_planning.mode`: `"optimal"` für Challenge, `"nearest_neighbor"` zum Testen

**Ablauf:**
1. Alle Nodes starten → Debug-Fenster öffnet sich
2. Bot exploriert selbstständig alle Kanten (DFS)
3. Tore werden live im Dashboard eingetragen
4. "Delivery starten"-Button wird aktiv wenn Exploration fertig
5. Geplanten Pfad im Dashboard prüfen
6. Button drücken → Bot fährt Tore in optimaler Reihenfolge ab
