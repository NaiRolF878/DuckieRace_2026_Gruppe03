# Challenge 2 – Intersection Handling

> ROS 1 (Noetic) · Ubuntu 20.04 · Python 3 · OpenCV · pupil_apriltags

Der Duckiebot folgt der Spur (wie Challenge 1), erkennt an Kreuzungen die **rote
Haltelinie** und das zugehörige **AprilTag**, hält an, wählt zufällig eine der
vom Tag erlaubten Richtungen und biegt ab. Ein zentraler Zustandsautomat (FSM)
steuert den Ablauf, das Abbiegen besteht aus einer frei definierbaren
**Segment-Sequenz** pro Richtung (Einfahrt → Drehung → Ausfahrt).

---

## Inhaltsverzeichnis

- [Dateien](#dateien)
- [Grundidee](#grundidee)
- [Systemüberblick](#systemüberblick)
- [Die Phasen](#die-phasen)
- [Nodes](#nodes)
- [Topics](#topics)
- [Konfigurationsparameter](#konfigurationsparameter)
- [Bot-spezifische Parameter](#bot-spezifische-parameter)
- [Tag-Mapping](#tag-mapping)
- [Kalibrierung](#kalibrierung)
- [Mögliche Optimierungen](#mögliche-optimierungen)
- [Bekannte Probleme & Lösungen](#bekannte-probleme--lösungen)

---

## Dateien

| Datei | Typ | Beschreibung |
|---|---|---|
| `detect_lane_node.py` | Node | Spurerkennung (CLAHE, Frame-Tracking) + rote Haltelinie (Bird's-Eye) |
| `detect_apriltag_node.py` | Node | AprilTag-Erkennung, Tag-Gedächtnis, Positionsfilter |
| `switch_control_node.py` | Node | **FSM** – trifft alle Entscheidungen, schaltet Steuerungs-Nodes |
| `control_lane_node.py` | Node | PID-Spurregler (reiner PID) |
| `control_intersection_node.py` | Node | Fährt die Kreuzung: Stopping (stehen) + Turning (Segment-Sequenz) |
| `configuration_node.py` | Node | Live-Kalibrierungs-GUI mit JSON-Persistenz |
| `camera_dashboard_node.py` | Node | Kamera-Dashboard (Debug-Ansicht, zeigt Phase + Richtung) |
| `util.py` | Hilfsfunktionen | Parameter laden, default+Bot mergen, live updaten |
| `*.json` | Config | je Node ein Parameter-Satz |

---

## Grundidee

**Zwei-Schichten-Prinzip** (wie die offizielle Duckietown-Pipeline):

- Die **rote Haltelinie** ist der Trigger – sie sagt *wann* angehalten wird.
- Der **AprilTag** liefert die Richtung – er sagt *welche* Abbiegungen erlaubt sind.
- Die **FSM** führt beide Signale zusammen und entscheidet, *was* der Bot tut.

Eine Kreuzung wird nur ausgelöst, wenn rote Linie **und** Tag-Richtung vorliegen.
Eine rote Linie ohne Tag wird ignoriert (Bot fährt weiter) – sicheres Default.

---

## Systemüberblick

```
                          Kamera
            (/camera_node/image/compressed)
                            |
            +---------------+---------------+
            v                               v
      detect_lane                     detect_apriltag
       |     |                              |
     lane  stop_line               direction / id
       |     |                              |
       |     +----------+          +--------+
       v                v          v
       |        +------------------------------------------+
       |        |           switch_control  (FSM)          |  <-- Entscheidungen
       |        |   Kreuzung? . Richtung wuerfeln . Phase  |
       |        +------------------------------------------+
       |           | enable/lane     | enable/intersection
       |           |                 | phase . direction
       v           v                 v
   control_lane              control_intersection
       |                       |         |
       |                       |     turn_done
       |                       |  (meldet: Sequenz fertig)
       +-------------+---------+
                     v
           /car_cmd_switch_node/cmd  ->  Motoren
```

Strikte Trennung: **Wahrnehmungs-Nodes** erkennen nur und liefern Signale.
**Steuerungs-Nodes** fahren nur. Entscheidungen fallen ausschließlich in der FSM.

---

## Die Phasen

Der Ablauf an einer Kreuzung ist bewusst einfach gehalten – nur drei Zustände:

| Phase | Was passiert | Aktive Steuerung |
|---|---|---|
| **Lane** | Normales Spurfolgen, wartet auf Kreuzung | control_lane |
| **Stopping** | An der Haltelinie anhalten (v=0) für `stop_duration` | control_intersection |
| **Turning** | Abbiege-Sequenz abfahren (Einfahrt -> Drehung -> Ausfahrt) | control_intersection |
| -> **Lane** | control_intersection meldet `turn_done`, FSM schaltet zurück | control_lane |

### Wie funktioniert das Abbiegen?

Das Abbiegen besteht aus einer **Segment-Sequenz** pro Richtung, definiert in
`control_intersection_node.json`. Jedes Segment hat drei Werte:

- `v` – Vorwärtsgeschwindigkeit
- `omega` – Drehrate (positiv = links, negativ = rechts)
- `duration` – wie lange dieses Segment gefahren wird

Beispiel Rechtsabbiegen (drei Segmente):
1. **Einfahrt:** Kurz geradeaus in die Kreuzung (v=0.2, omega=0.2, 1.1s)
2. **Drehung:** Hart rechts lenken (v=0.15, omega=-3.2, 0.5s)
3. **Ausfahrt:** Kurz geradeaus raus (v=0.2, omega=0.1, 0.5s)

control_intersection fährt die Segmente der Reihe nach ab und meldet am Ende
über `/intersection/turn_done` an die FSM, dass das Manöver fertig ist. Die FSM
schaltet dann zurück auf Lane und der PID-Spurregler übernimmt.

**Alle Zeiten leben an einer Stelle** (in der control_intersection JSON). Die
FSM zählt keine eigenen Zeiten für das Abbiegen – sie wartet nur auf das
`turn_done`-Signal. Ein `turning_timeout` in der FSM dient als Sicherheitsnetz,
falls das Signal ausbleibt.

---

## Nodes

### detect_lane_node
Bird's-Eye-View, CLAHE-Helligkeitsausgleich, Frame-Tracking gegen
Linien-Sprünge. Erkennt gelbe + weiße Spurlinie (-> Spurversatz) und die rote
Haltelinie im unteren Bildbereich.
**Publiziert:** `/detect/lane` (Float64), `/detect/stop_line` (Bool)

### detect_apriltag_node
Erkennt AprilTags (Familie *tagStandard52h13*, IDs 1–4) im Originalbild.
**Tag-Gedächtnis:** ein naher Tag wird einige Sekunden gemerkt (Tag und rote
Linie sind selten gleichzeitig sichtbar). **Positionsfilter:** nur rechte
Bildhälfte. Das Debug-Fenster zeigt die erkannte ID, die erlaubten Richtungen
und die gewählte Fahrtrichtung ("FAHRE: ...", grün wenn passend).
**Publiziert:** `/detect/apriltag/direction` (String), `/detect/apriltag/id` (Int32)

### switch_control_node — die FSM (Entscheidungsebene)
Einzige Node mit Zustandslogik. Drei Zustände: Lane, Stopping, Turning.
- **Kreuzungs-Erkennung:** rote Linie **und** bekannte Tag-Richtung im
  Lane-Zustand -> Kreuzung. Rote Linie ohne Tag -> ignoriert.
- **Richtungswahl:** beim Eintritt einmalig zufällig aus den erlaubten Richtungen.
- **Turning-Ende:** wartet auf `turn_done` von control_intersection.
- **Aktiviert/deaktiviert** control_lane und control_intersection.
**Publiziert:** `/enable/lane` (Bool), `/enable/intersection` (Bool),
`/intersection/phase` (String), `/intersection/direction` (String)

### control_lane_node
Reiner PID-Spurregler (Anti-Windup, MIN_VEL). Aktiv nur bei `/enable/lane == True`.
**Publiziert:** `/car_cmd_switch_node/cmd` (Twist2DStamped)

### control_intersection_node
Fährt die Kreuzung: in der Stopping-Phase steht der Bot still, in der
Turning-Phase wird die Segment-Sequenz der gewählten Richtung abgefahren.
Meldet am Ende `/intersection/turn_done`. Aktiv nur bei `/enable/intersection == True`.
**Publiziert:** `/car_cmd_switch_node/cmd` (Twist2DStamped),
`/intersection/turn_done` (Bool)

### configuration_node / camera_dashboard_node
Live-Kalibrierung (Schieberegler, schreibt in die JSONs) bzw.
Debug-Visualisierung. Das Dashboard zeigt die aktuelle Phase und die
gewählte Abbiegerichtung. Greifen nicht ins Fahren ein.

---

## Topics

| Topic | Typ | Von -> Nach |
|---|---|---|
| `/detect/lane` | Float64 | detect_lane -> control_lane |
| `/detect/stop_line` | Bool | detect_lane -> FSM |
| `/detect/apriltag/direction` | String | detect_apriltag -> FSM |
| `/detect/apriltag/id` | Int32 | detect_apriltag -> (kein Subscriber) |
| `/detect/apriltag` | Int32 | detect_apriltag -> Dashboard (separate Kopie der Tag-ID fürs Dashboard) |
| `/intersection/phase` | String | FSM -> control_intersection, Dashboard |
| `/intersection/direction` | String | FSM -> control_intersection, detect_apriltag, Dashboard |
| `/intersection/turn_done` | Bool | control_intersection -> FSM |
| `/enable/lane` | Bool | FSM -> control_lane |
| `/enable/intersection` | Bool | FSM -> control_intersection |
| `/car_cmd_switch_node/cmd` | Twist2DStamped | control_lane / control_intersection -> Motoren |

---

## Konfigurationsparameter

### switch_control_node.json

| Parameter | Zweck |
|---|---|
| `timing.stop_duration` | Haltezeit an der roten Linie (Stopping-Phase) |
| `timing.turning_timeout` | Sicherheits-Timeout: falls `turn_done` ausbleibt, schaltet die FSM nach dieser Zeit trotzdem zu Lane |

### control_intersection_node.json

| Parameter | Zweck |
|---|---|
| `turn_segments.left` | Segment-Sequenz fürs Linksabbiegen (Liste von {v, omega, duration}) |
| `turn_segments.right` | Segment-Sequenz fürs Rechtsabbiegen |
| `turn_segments.straight` | Segment-Sequenz fürs Geradeausfahren |

Jedes Segment definiert Geschwindigkeit (`v`), Drehrate (`omega`) und Dauer
(`duration`). Die Segmente werden der Reihe nach abgefahren: Einfahrt -> Drehung
-> Ausfahrt. Anzahl der Segmente pro Richtung ist frei wählbar.

### detect_lane_node.json

| Parameter | Zweck |
|---|---|
| `yellow`/`white`/`red` | HSV-Schwellen der Linien |
| `crop_image` | Bird's-Eye-Eckpunkte |
| `red.detection_zone` | wie weit vorn die rote Linie gesucht wird (kleiner = früher) |
| `red.pixel_threshold` | Mindestanzahl roter Pixel für eine gültige Erkennung |

### detect_apriltag_node.json

| Parameter | Zweck |
|---|---|
| `tag_memory.seconds` | Gedächtnis-Dauer eines gesehenen Tags |
| `tag_memory.min_area` | Mindestfläche damit ein Tag ins Gedächtnis kommt |
| `tag_filter.stability_frames` | Frames in Folge für stabile Tag-ID |
| `tag_filter.pos_x_min/max` | Positionsfilter (rechte Bildhälfte) |

### control_lane_node.json

| Parameter | Zweck |
|---|---|
| `pid.p/i/d` | PID-Reglerparameter |
| `pid.max_vel/min_vel` | Geschwindigkeitsgrenzen |

---

## Bot-spezifische Parameter

`util.py` lädt aus jeder JSON zuerst den `default`-Block und überschreibt ihn
mit einem bot-spezifischen Block, falls vorhanden:

```
parameters.default            -> gilt für alle Bots
parameters.<vehicle_name>     -> Overrides für diesen Bot
```

`detect_lane` und `control_lane` haben die Bot-Struktur (echte Kalibrierung pro
Bot). `control_intersection` und `switch_control` sind flach bzw. haben nur
`default` (für alle Bots gleich).

---

## Tag-Mapping

| Tag-ID | Erlaubte Richtungen |
|:---:|---|
| 1 | links, geradeaus, rechts |
| 2 | rechts, links |
| 3 | geradeaus, links |
| 4 | geradeaus, rechts |

Anpassbar in `config/detect_apriltag_node.json` unter `tag_directions`.

---

## Kalibrierung

1. `configuration_node` + Debug-Fenster starten.
2. **Spurfarben:** Gelb-/Weiß-Maske beobachten, HSV einstellen.
   Blasses Klebeband: `yellow.sl` runter, `white.sh` runter.
3. **Rote Linie:** Rot-Maske prüfen, `pixel_threshold` + `detection_zone` einstellen.
4. **AprilTag:** Im Debug-Fenster ID + Fläche prüfen; `tag_memory.min_area` anpassen.
5. **Abbiegen:** Die Segment-Sequenzen in `control_intersection_node.json` justieren.
   Pro Richtung drei Schritte (Einfahrt/Drehung/Ausfahrt) mit v, omega, duration.
   Vorgehen: Segment-Werte ändern, Node neu starten, Manöver beobachten, wiederholen.
   Die Werte der anderen Gruppe als Startwerte nutzen und an die eigene Kreuzung anpassen.

---

## Mögliche Optimierungen

Der aktuelle Ablauf ist bewusst **zeitgesteuert** (robust, einfach zu
kalibrieren). Für mehr Präzision und Unabhängigkeit von festen Zeiten gibt es
drei sinnvolle Ausbaustufen:

1. **Turning mit Abbruchkriterium statt fester Zeit.**
   Statt die Drehzeit ablaufen zu lassen, die Drehung beenden, sobald ein
   sichtbares Kriterium erfüllt ist – z.B. die **rote Linie der Gegenspur** im
   erwarteten Bildbereich (bei Linksdrehung im linken unteren Bereich, bei
   Rechtsdrehung im rechten). Vorteil: unabhängig von Drehgeschwindigkeit,
   Akkustand und Reibung.

2. **Ausfahrt mit Abbruchkriterium statt fester Zeit.**
   Das letzte Segment (Ausfahrt geradeaus) beenden, sobald die **Lane Detection
   wieder eine gültige Spur meldet**, statt nach fester Dauer. Vorteil: passt
   sich automatisch an unterschiedliche Kreuzungsgrößen und Anfahrwinkel an.
   Voraussetzung: detect_lane muss ein verlässliches "Spur gefunden / nicht
   gefunden"-Signal liefern.

3. **Ausrichten an der roten Haltelinie (Orthogonalität).**
   Vor dem Losfahren den Bot so drehen, dass er **senkrecht zur roten
   Haltelinie** steht. Aus der Neigung der roten Linie im Kamerabild lässt sich
   der Schräglagewinkel ableiten; der Bot dreht im Stand, bis die Linie
   waagerecht erscheint. Vorteil: löst das Problem an der Wurzel – ein schräg
   angekommener Bot startet trotzdem gerade und fährt sauber durch die Kreuzung.

---

## Bekannte Probleme & Lösungen

| Problem | Ursache | Lösung |
|---|---|---|
| `Unable to start XML-RPC server, port 0` | `ROS_IP` falsch / Docker-Bridge / hängender Prozess | richtige `ROS_IP` setzen, `pkill -9 -f node.py`, ggf. Neustart |
| Verzögertes Dashboard-Bild, imshow flüssig | Dashboard baut mehrere Topics über ROS zusammen; imshow zeichnet lokal im Prozess | normal; fürs Debuggen imshow vertrauen |
| "verwende default" trotz Kalibrierung | Meldung stammt von einer Node mit flacher JSON | normal, kein Fehler |
| JSON-Fehler `Expecting property name` | Komma-Fehler beim Editieren (trailing comma) | `python3 -m json.tool <datei>` zeigt die Stelle |
| Bot findet nach Linkskurve die Spur nicht | weite Kurve, nach dem Drehen nur Kreuzungsfläche im Bild | Ausfahrt-Segment verlängern (duration hoch) |
| Bot kommt schräg an der Haltelinie an | Spurführung vor der Kreuzung nicht perfekt | langfristig: Orthogonal-Ausrichtung (siehe Optimierungen) |
