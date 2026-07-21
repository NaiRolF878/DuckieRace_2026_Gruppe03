# Challenge 4 – Mapping & Path Finding: Architektur-Übersicht

Kurzfassung für die Präsentation. Erklärt, welche Node was tut, was sie
rausgibt, und wo welche Entscheidung getroffen wird.

---

## Grundidee in einem Satz

Der Bot bekommt eine **Landkarte als Graph** (JSON), fährt sie in Phase 1
selbstständig komplett ab und merkt sich dabei, wo farbige **Tore** liegen;
danach berechnet er die **kürzeste Route**, um alle Tore in der besten
Reihenfolge abzuliefern, und fährt diese Route nach einem Knopfdruck ab.

Die eigentliche Fahr-Mechanik (Spur halten, an der Kreuzung anhalten,
abbiegen) ist **1:1 aus Challenge 2 übernommen** – neu ist ausschließlich die
Schicht, die *entscheidet, wohin* der Bot als Nächstes abbiegen soll. Früher
hat die FSM dafür gewürfelt (`random.choice`), jetzt sagt ihr die
Graph-Logik exakt, welche Richtung sie nehmen soll.

---

## Die drei Phasen

1. **Mapping (bewertet: Vollständigkeit).** Der Bot fährt per
   Tiefensuche (DFS) systematisch jede Kante des Graphen ab, bis keine
   unbesuchte Kante mehr übrig ist. Dabei werden alle Tore erkannt und auf
   die Kante gemappt, auf der sie liegen.
2. **Planung (nicht bewertet).** Sobald Phase 1 fertig ist, berechnet eine
   Node im Hintergrund automatisch die kürzeste Reihenfolge, um alle
   gefundenen Tore abzuliefern (Dijkstra + Rundreise-Optimierung) – **außer**
   im Debug-Fenster wurde eine vorgegebene Reihenfolge eingetragen (z.B. weil
   die Challenge eine feste Abliefer-Reihenfolge vorschreibt), dann wird
   diese übernommen und nur noch der kürzeste Weg dazwischen berechnet. Das
   Ergebnis wird im Debug-Fenster angezeigt, damit ein Mensch es vor dem
   Losfahren prüfen kann.
3. **Delivery (bewertet: Zeit!).** Erst nach Klick auf "Delivery starten"
   fährt der Bot die geplante Route tatsächlich ab.

---

## Die Nodes im Einzelnen

### Wahrnehmung (unverändert aus Challenge 2, minimal erweitert)

**`detect_lane_node`** – Spur + rote Haltelinie, exakt wie in Challenge 2.

**`detect_apriltag_node`** – erkennt weiterhin die Kreuzungs-Tags (1–4) für
die erlaubten Abbiegerichtungen, zusätzlich jetzt auch **Tor-Tags (5–13)**:
jede erkannte ID in diesem Bereich wird auf einem eigenen Topic
(`/detect/gate/id`) gemeldet, unabhängig von der bestehenden
Richtungs-Logik.

### Das neue Graph-Gedächtnis

**`graph_state_node`** – der Knotenpunkt der gesamten neuen Logik
- Lädt die Stadtkarte (`mapping_node.json`) direkt beim Start.
- Verfolgt live: wo steht der Bot gerade (`current_node`), welche Kante
  befährt er gerade (`current_edge`), welche Kanten wurden schon besucht,
  welche Tore wurden auf welcher Kante gefunden.
- **Löst das zentrale Übersetzungsproblem:** Die Graph-Logik denkt in
  Tag-IDs ("nimm Ausfahrt 3"), die alte FSM aus Challenge 2 versteht aber nur
  Wörter ("rechts"/"links"/"geradeaus"). `graph_state_node` kennt sowohl den
  gerade sichtbaren Einfahrt-Tag als auch die feste Kreuzungs-Geometrie
  (Tag 2 = rechts von Tag 1, Tag 4 = links von Tag 1, Tag 3 = gegenüber) und
  übersetzt jede mögliche Ausfahrt am aktuellen Knoten automatisch in das
  passende Wort.

### Entscheidung: zwei "Gehirne" für zwei Phasen

**`explore_control_node`** – Phase 1 (Tiefensuche)
- Wählt an jeder Kreuzung die erste noch unbesuchte Ausfahrt.
- Sind alle Ausgänge eines Knotens abgefahren, sucht es über bereits
  bekannte (befahrene) Kanten den kürzesten Weg zurück zu einem Knoten mit
  offenen Ausgängen – klassisches DFS-Backtracking.
- Meldet ans Dashboard, wenn wirklich jede Kante besucht wurde.

**`path_planner_node`** – Phase 2+3 (kürzeste Route)
- Berechnet mit einer eigenen Dijkstra-Implementierung die Distanz zwischen
  dem Start und jedem gefundenen Tor.
- Ist eine Reihenfolge vorgegeben (Dashboard-Eingabe / Config), wird sie
  unverändert übernommen. Sonst probiert es (Brute-Force bei bis zu 10
  Toren, sonst eine Greedy-Heuristik) alle sinnvollen Reihenfolgen durch und
  wählt die mit der kürzesten Gesamtstrecke.
- Fährt nach dem Startsignal die Route ab und erkennt automatisch, wenn ein
  Tor abgeliefert wurde (die dafür nötige Kante wurde tatsächlich befahren).

Beide Nodes geben ihre Entscheidung als **Wort-Richtung** an dieselbe FSM
(`switch_control_node`) weiter wie in Challenge 2 – die FSM selbst musste
dafür nur an einer einzigen Stelle geändert werden.

### Steuerung (fast unverändert)

**`switch_control_node`** – dieselbe FSM wie in Challenge 2 (Lane / Stopping
/ Turning). Einzige Änderung: Die Abbiegerichtung kommt nicht mehr per Würfel,
sondern von `explore_control_node`/`path_planner_node`. Passt die gewählte
Richtung nicht (mehr) zu dem, was an der Kreuzung erlaubt ist, bleibt der Bot
stehen und wartet – **kein Zufalls-Fallback**, weil Challenge 4 einen
eindeutig bestimmten Weg verlangt. Kann die Kamera den Kreuzungs-Tag an
dieser Kreuzung gar nicht lesen, weicht die Node stattdessen auf die vom
Graphen deterministisch vorhergesagten erlaubten Richtungen aus
(`graph_state_node` → `/graph/allowed_directions`, siehe dessen
`predicted_entry_tag`) – kein Würfeln, aber auch kein permanentes
Stehenbleiben nur weil ein einzelnes Foto misslingt.

**`control_lane_node`** – unverändert (PID-Spurregler).

**`control_intersection_node`** – fährt die Abbiege-Sequenzen wie in
Challenge 2, erkennt das Ende eines Segments aber jetzt an den
**Radencoder-Ticks** statt an einer festen Zeit – präziser und weniger
anfällig für Akkustand/Bodenhaftung.

### Sichtbarmachen

**`debug_graph_node`** – tkinter-Dashboard, der einzige Ort mit
Bedienelement. Zeigt die komplette Karte, wächst live grün mit jeder neu
befahrenen Kante, zeigt die aktuelle Bot-Position und (nach der Planung) den
vorgeschlagenen Delivery-Pfad als gestrichelten Pfeil. Der Button "Delivery
starten" wird erst aktiv, wenn Phase 1 wirklich abgeschlossen ist.

---

## Wer redet mit wem (Datenfluss)

```
                 Kamera
                   │
   ┌───────────────┼───────────────────┐
   ▼               ▼                   ▼
detect_lane   detect_apriltag  (Kreuzungs-Tag + Tor-Tag)
   │  │              │    │
   │  │ stop_line    │    │ gate/id
   │  │        apriltag/id│
   │  │              ▼    ▼
   │  │      ┌──────────────────────┐
   │  │      │   graph_state_node    │  ← Graph-Gedaechtnis + Tag<->Wort
   │  │      └──────────────────────┘
   │  │        │        │        │
   │  │   current_node  │  exit_directions
   │  │        ▼        ▼        ▼
   │  │  ┌────────────┐   ┌───────────────┐
   │  │  │explore_ctrl│   │ path_planner   │  ← Phase 1 / Phase 2+3
   │  │  │  (DFS)     │   │ (Dijkstra+TSP) │
   │  │  └────────────┘   └───────────────┘
   │  │        │                 │
   │  │        └───next_direction┘  (Wort: left/right/straight)
   │  │                 │
   │  ▼                 ▼
   │      ┌─────────────────────────────────────┐
   │      │        switch_control (FSM)          │  ← unveraendert aus C2
   │      │  Kreuzung? next_direction? Phase?    │
   │      └─────────────────────────────────────┘
   │          │ enable/lane    │ enable/intersection
   ▼          ▼                ▼
control_lane            control_intersection (Encoder statt Zeit)
   │                          │
   └──────────┬───────────────┘
              ▼
      car_cmd_switch_node/cmd  →  Motoren

debug_graph_node haengt an ALLEN /graph/*- und /navigation/*-Topics und
publiziert nur den Start-Delivery-Knopfdruck zurueck.
```

---

## Die wichtigsten Design-Entscheidungen (gut für Nachfragen)

**Warum eine eigene Node nur fürs Graph-Gedächtnis?**
Damit `explore_control_node` und `path_planner_node` (die zwei "Phasen-Gehirne")
nicht jede ihre eigene Vorstellung von "wo bin ich, was habe ich schon
gesehen" pflegen müssen. Eine einzige Quelle der Wahrheit verhindert, dass
sich die beiden Phasen widersprechen.

**Warum wird die Fahrtrichtung als Wort und nicht als Tag-ID an die FSM
übergeben?**
Weil `switch_control_node`/`control_intersection_node` aus Challenge 2 exakt
so übernommen werden sollten wie sie sind (bewährte, getestete Logik). Die
Übersetzung von Tag-ID auf Wort passiert daher zentral an einer Stelle
(`graph_state_node`), nicht verteilt über mehrere Nodes.

**Warum wird die nächste Fahrtrichtung bei jedem Tick neu berechnet statt
einmal beim Ankommen an einer Kreuzung entschieden?**
Weil zwischen "Bot ist logisch am neuen Knoten angekommen" (Graph-Update) und
"Kamera sieht den Tag der neuen Kreuzung" ein kleiner Zeitversatz liegt. Eine
einmalige Entscheidung könnte in genau diesem Fenster auf veralteten Daten
beruhen und sich festfressen. Eine Entscheidung, die jeden Tick neu aus dem
aktuellen Zustand berechnet wird, korrigiert sich von selbst.

**Warum kein Zufalls-Fallback mehr in `switch_control_node`?**
In Challenge 2 war eine zufällige Richtung völlig in Ordnung. In Challenge 4
gibt es aber einen konkreten Plan (erst "jede Kante erkunden", dann "kürzeste
Tor-Route") – eine zufällige Abweichung davon wäre nicht einfach suboptimal,
sondern schlicht falsch (falscher Knoten im Graph-Modell). Lieber wartet der
Bot, bis die richtige Richtung feststeht.

**Warum Encoder statt fester Zeit beim Abbiegen?**
Zeitbasierte Segmente hängen von Akkustand, Bodenhaftung und Batterietemperatur
ab und driften über den Tag. Encoder-Ticks messen die tatsächlich gefahrene
Strecke/Drehung direkt am Rad – robuster, auch wenn die Startwerte weiterhin
vor Ort kalibriert werden müssen.

---

## Tag-Bereiche

| Bereich | Bedeutung |
|:---:|---|
| 1–4 | Kreuzungs-Tags: codieren die Einmündung (physikalisch fest: 1↔3 gegenüber, 2 rechts von 1, 4 links von 1) |
| 5–13 | Tor-Tags: codieren Zielorte auf den Kanten zwischen Kreuzungen |

Erlaubte Richtungen je Kreuzungs-Tag stehen wie in Challenge 2 in
`config/detect_apriltag_node.json` (`tag_directions`).
