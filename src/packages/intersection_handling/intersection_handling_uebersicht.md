# Challenge 2 – Intersection Handling: Architektur-Übersicht

Erklärt, welche Node was tut, was sie
rausgibt, und wo welche Entscheidung getroffen wird.

---

## Grundidee in einem Satz

Die **rote Haltelinie** sagt dem Bot *wann* er an einer Kreuzung anhalten muss,
der **AprilTag** sagt ihm *welche Richtungen* erlaubt sind, und ein zentraler
**Zustandsautomat (FSM)** entscheidet daraus, *was* der Bot tut: anhalten,
zufällig eine erlaubte Richtung wählen, abbiegen und zurück in die Spur finden.

Dahinter steht ein bewusstes Architektur-Prinzip aus der offiziellen
Duckietown-Pipeline: **Zwei Schichten.** Die rote Linie ist der Trigger
(Anhalten), der Tag liefert die Richtung. Beide sind getrennte Signale und
werden erst in der FSM zusammengeführt.

---

## Der Ablauf an einer Kreuzung (die vier Phasen)

Der Bot durchläuft vier Zustände. Nur **einer** ist zu jedem Zeitpunkt aktiv:

1. **Lane** – normales Spurfolgen. Der Bot fährt der Spur nach und wartet auf
   eine Kreuzung.
2. **Approaching** – rote Linie + Tag erkannt. Der Bot fährt geradeaus über die
   Haltelinie, bis er sicher drüber steht.
3. **Turning** – der Bot biegt in die (zufällig gewählte) erlaubte Richtung ab.
4. **Handover** – nach dem Abbiegen lenkt der Bot sanft zurück in die Spur;
   sobald die Spur stabil erkannt wird, geht es zurück zu **Lane**.

---

## Die Nodes im Einzelnen

Das Paket trennt strikt zwischen **Wahrnehmung** (erkennt etwas, trifft *keine*
Fahrentscheidung) und **Steuerung** (fährt, trifft *keine* Erkennung). Die
einzige Stelle mit Entscheidungslogik ist die FSM.

### Wahrnehmung (Perception)

**`detect_lane_node`** – Augen für die Spur und die Haltelinie
- Wandelt das Kamerabild in die Vogelperspektive (Bird's-Eye-View) um und
  gleicht Helligkeit aus (CLAHE).
- Findet die gelbe (Mittel-) und weiße (Außen-) Linie und berechnet daraus den
  **Spurversatz**: wie weit der Bot von der Spurmitte abweicht.
- Findet zusätzlich die **rote Haltelinie** im unteren Bildbereich
  (abstandsgenau, weil Bird's-Eye).
- **Gibt raus:**
  - `/detect/lane` (Zahl von -1 bis +1) – Spurversatz
  - `/detect/stop_line` (true/false) – rote Linie nah genug zum Anhalten

**`detect_apriltag_node`** – liest die Verkehrsschilder
- Erkennt AprilTags der Familie *tagStandard52h13* (selbst gestaltet, IDs 1–4)
  im Original-Kamerabild.
- Jede Tag-ID steht für eine erlaubte Richtungs-Kombination (z.B. „nur links
  oder geradeaus").
- **Tag-Gedächtnis:** Tag und rote Linie sind selten gleichzeitig im Bild (der
  Tag steht seitlich, die Linie kommt erst später). Darum merkt sich die Node
  einen nahen Tag für einige Sekunden, damit die Richtung beim Anhalten noch
  bekannt ist.
- **Positionsfilter:** Nur Tags in der rechten Bildhälfte zählen (Rechtsverkehr,
  Schild steht rechts).
- **Gibt raus:**
  - `/detect/apriltag/direction` (Text, z.B. "left,straight") – erlaubte Richtungen
  - `/detect/apriltag/id` (Zahl) – erkannte Tag-ID

**`detect_red_lane_node`** – erkennt, wann das Abbiegen fertig ist
- Schaut beim Abbiegen im Original-Kamerabild gezielt in **die erwartete
  Bildregion**: Bei Linksdrehung erscheint die neue Querlinie rechts, bei
  Rechtsdrehung links.
- Trick gegen das Chaos an der Kreuzung (dort sind *mehrere* rote Linien
  sichtbar): Es wird nur in der erwarteten Region gesucht, die größte
  zusammenhängende rote Fläche genommen, und das Signal erst gegeben, *nachdem
  die Region einmal frei war und dann wieder Rot erscheint* ("erst leer, dann
  Wiederauftauchen"). Das ignoriert die eigene und fremde Linien zuverlässig.
- **Gibt raus:**
  - `/intersection/turn_complete` (true/false) – Abbiegen abgeschlossen

### Entscheidung (das Gehirn)

**`switch_control_node`** – der Zustandsautomat (FSM), **hier fallen alle
Entscheidungen**
- Ist die *einzige* Node mit Zustandslogik. Alle Erkennungs-Nodes liefern nur
  Signale; hier wird entschieden, was sie bedeuten.
- **Kreuzungs-Entscheidung:** Wenn (rote Linie erkannt) UND (eine Tag-Richtung
  bekannt) und der Bot im Zustand *Lane* ist → Kreuzung! Eine rote Linie *ohne*
  Tag wird ignoriert (der Bot fährt weiter) – sicheres Verhalten.
- **Richtungswahl:** Beim Eintritt in die Kreuzung wird *einmal* zufällig eine
  der erlaubten Richtungen gewürfelt.
- **Phasensteuerung:** Schaltet zwischen Lane → Approaching → Turning →
  Handover → Lane und entscheidet, wann welche Phase endet.
- **Aktiviert/deaktiviert die Steuerungs-Nodes:** Im Lane-Zustand ist
  `control_lane` aktiv; sobald eine Kreuzung erkannt wird, wird `control_lane`
  abgeschaltet und `control_intersection` übernimmt komplett.
- **Gibt raus:**
  - `/enable/lane` (true/false) – ist der Spurfolger aktiv?
  - `/enable/intersection` (true/false) – ist die Kreuzungssteuerung aktiv?
  - `/intersection/phase` (Text) – aktuelle Phase
  - `/intersection/direction` (Text) – die gewählte Abbiegerichtung

### Steuerung (Motoren)

**`control_lane_node`** – fährt der Spur nach
- Reiner PID-Regler: bekommt den Spurversatz und steuert Geschwindigkeit und
  Lenkung, um mittig zu bleiben.
- Aktiv nur, wenn `/enable/lane` true ist (also im Lane-Zustand).
- **Gibt raus:** `/car_cmd_switch_node/cmd` (Fahrbefehl an die Motoren)

**`control_intersection_node`** – fährt durch die Kreuzung
- Aktiv nur, wenn `/enable/intersection` true ist.
- Fährt je nach Phase: Approaching = geradeaus, Turning = drehen (Richtung je
  nach gewählter Abbiegerichtung), Handover = sanft mit P-Regler zurück in die
  Spur.
- **Gibt raus:** `/car_cmd_switch_node/cmd` (Fahrbefehl an die Motoren)

**`camera_dashboard_node`** – Visualisierung fürs Debuggen (greift nicht ins
Fahren ein).

---

## Wer redet mit wem (Datenfluss)

```
                 Kamera
                   │
   ┌───────────────┼─────────────────────────┐
   ▼               ▼                          ▼
detect_lane   detect_apriltag        detect_red_lane
   │  │              │                        │
   │  │ stop_line    │ direction/id           │ turn_complete
   │  └──────┐       │                        │
   │ lane    ▼       ▼                         ▼
   │      ┌─────────────────────────────────────┐
   │      │        switch_control (FSM)          │  ← ENTSCHEIDUNGEN
   │      │  Kreuzung? Richtung? Welche Phase?   │
   │      └─────────────────────────────────────┘
   │          │ enable/lane    │ enable/intersection
   │          │ phase          │ direction
   ▼          ▼                ▼
control_lane            control_intersection
   │                          │
   └──────────┬───────────────┘
              ▼
      car_cmd_switch_node/cmd  →  Motoren
```

---

## Die wichtigsten Design-Entscheidungen (gut für Nachfragen)

**Warum rote Linie UND Tag, nicht nur eines?**
Fehlauslösung (Kreuzung wo keine ist) und verpasste Kreuzung zählen in der
Bewertung gleich. Die Kombination beider Signale minimiert beide Fehlerarten.
Entspricht der offiziellen Duckietown-Pipeline.

**Warum ein Tag-Gedächtnis?**
Weil Tag (steht seitlich/hoch) und rote Linie (am Boden) fast nie gleichzeitig
im Bild sind. Ohne Gedächtnis würde die Kreuzung nie als solche erkannt.

**Warum sucht detect_red_lane nur in einer Region?**
An der Kreuzung sind mehrere rote Linien sichtbar. Da die Drehrichtung bekannt
ist (vom Tag), weiß man vorher, *wo* die relevante Linie auftauchen muss – das
ignoriert alle anderen.

**Warum eine einzige FSM-Node für alle Entscheidungen?**
Frühere Versuche mit verteilter Logik führten zu Fehlern. Jetzt liefern alle
Erkennungs-Nodes nur Signale, und genau eine Stelle entscheidet – das ist
leichter zu debuggen und vorhersehbar.

**Warum wird control_lane an der Kreuzung komplett abgeschaltet?**
Damit sich nicht zwei Steuerungen widersprechen. Sobald eine Kreuzung erkannt
wird, hat allein die Kreuzungssteuerung das Sagen.

---

## Umschaltbare Varianten (Fallback am Prüfungstag)

Zwei Stellen lassen sich per Kommentar zwischen zwei Strategien umschalten,
falls die primäre Variante zickt:

- **Approaching-Ende:** primär *distanzbasiert* (fahre bis Haltelinie weg) –
  alternativ *zeitgesteuert*.
- **Turning-Ende:** primär *regionsbasiert* (detect_red_lane meldet fertig) –
  alternativ *zeitgesteuert* (feste Drehzeit pro Richtung).

Beide Umschalter sitzen in `switch_control_node`, klar markiert.

---

## Tag-Mapping (IDs → erlaubte Richtungen)

| Tag-ID | Erlaubte Richtungen        |
|:------:|:---------------------------|
| 1      | links, geradeaus, rechts   |
| 2      | rechts, links              |
| 3      | geradeaus, links           |
| 4      | geradeaus, rechts          |

(Der Bot wird physisch in die Abbiegerichtung gelegt; daraus ergibt sich, welche
Richtungen pro Tag möglich sind.)
