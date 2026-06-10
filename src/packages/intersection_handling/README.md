# Challenge 2 – Intersection Handling

> ROS 1 (Noetic) · Ubuntu 20.04 · Python 3 · OpenCV · pupil_apriltags

Der Duckiebot folgt der Spur (wie Challenge 1), erkennt an Kreuzungen die **rote
Haltelinie** und das zugehörige **AprilTag**, hält an, wählt zufällig eine der
vom Tag erlaubten Richtungen und biegt ab. Ein zentraler Zustandsautomat (FSM)
steuert den gesamten Kreuzungs-Ablauf. Das Abbiegen ist **zeitgesteuert**
(feste Zeiten pro Richtung).

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
- [Setup & Starten](#setup--starten)
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
| `control_intersection_node.py` | Node | Fährt die Kreuzung (Stopping / Approaching / Turning / ExitStraight) |
| `configuration_node.py` | Node | Live-Kalibrierungs-GUI mit JSON-Persistenz |
| `camera_dashboard_node.py` | Node | Kamera-Dashboard (Debug-Ansicht, zeigt Phase + Richtung) |
| `util.py` | Hilfsfunktionen | Parameter laden, default+Bot mergen, live updaten |
| `*.json` | Config | je Node ein HSV-/Parameter-Satz (default + bot-spezifisch) |

---

## Grundidee

**Zwei-Schichten-Prinzip** (wie die offizielle Duckietown-Pipeline):

- Die **rote Haltelinie** ist der Trigger – sie sagt *wann* angehalten wird.
- Der **AprilTag** liefert die Richtung – er sagt *welche* Abbiegungen erlaubt sind.
- Die **FSM** führt beide Signale zusammen und entscheidet, *was* der Bot tut.

Beide Signale sind bewusst getrennt und werden erst in der FSM kombiniert. Eine
Kreuzung wird nur ausgelöst, wenn rote Linie **und** Tag-Richtung vorliegen.
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
       |                            |
       +-------------+--------------+
                     v
           /car_cmd_switch_node/cmd  ->  Motoren
```

Strikte Trennung: **Wahrnehmungs-Nodes** erkennen nur und liefern Signale.
**Steuerungs-Nodes** fahren nur. Entscheidungen fallen ausschließlich in der FSM.

---

## Die Phasen

Der Bot durchläuft an einer Kreuzung diese Phasen. Immer genau eine ist aktiv.

| Phase | Was passiert | Aktive Steuerung |
|---|---|---|
| **Lane** | Normales Spurfolgen, wartet auf Kreuzung | control_lane |
| **Stopping** | An der Haltelinie anhalten (v=0) für `stop_duration` | control_intersection |
| **Approaching** | Geradeaus über die Haltelinie fahren | control_intersection |
| **(PreTurnPause)** | *Debug:* vor dem Drehen anhalten (`pre_turn_pause` > 0) | control_intersection |
| **Turning** | Zeitgesteuert abbiegen (feste Zeit pro Richtung) | control_intersection |
| **(PostTurnPause)** | *Debug:* nach dem Drehen anhalten (`pre_turn_pause` > 0) | control_intersection |
| **ExitStraight** | Geradeaus aus der Kreuzung fahren (feste Zeit pro Richtung), bis die Spur wieder im Bild ist | control_intersection |
| -> **Lane** | Übergabe an den PID-Spurregler | control_lane |

**Warum ExitStraight?** Nach einer weiten Linkskurve liegt erst die schwarze
Kreuzungsfläche im Kamerabild – der PID hätte keine Spur zum Einregeln. Die
ExitStraight-Phase fährt darum nach dem Drehen noch eine feste Strecke geradeaus
(länger bei links, kurz bei rechts), bis die Spur sicher sichtbar ist, und
übergibt erst dann an control_lane.

**Debug-Pausen:** PreTurnPause und PostTurnPause werden beide über denselben
Parameter `pre_turn_pause` gesteuert. Für den Wettkampf auf `0` setzen → beide
Pausen werden übersprungen, der Ablauf läuft flüssig durch.

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
und die gewählte Fahrtrichtung ("FAHRE: ...", grün wenn sie zu den erlaubten passt).
**Publiziert:** `/detect/apriltag/direction` (String), `/detect/apriltag/id` (Int32)

### switch_control_node — die FSM (Entscheidungsebene)
Einzige Node mit Zustandslogik.
- **Kreuzungs-Erkennung:** rote Linie **und** bekannte Tag-Richtung im
  Lane-Zustand -> Kreuzung. Rote Linie ohne Tag -> ignoriert.
- **Richtungswahl:** beim Eintritt einmalig zufällig aus den erlaubten Richtungen.
- **Phasensteuerung:** Lane -> Stopping -> Approaching -> Turning -> ExitStraight -> Lane
  (Abbiegen und Ausfahren zeitgesteuert pro Richtung).
- **Aktiviert/deaktiviert** control_lane und control_intersection.
**Publiziert:** `/enable/lane` (Bool), `/enable/intersection` (Bool),
`/intersection/phase` (String), `/intersection/direction` (String)

### control_lane_node
Reiner PID-Spurregler (Anti-Windup, MIN_VEL). Aktiv nur bei `/enable/lane == True`.
**Publiziert:** `/car_cmd_switch_node/cmd` (Twist2DStamped)

### control_intersection_node
Fährt die Kreuzung je nach Phase: Stopping/PreTurnPause/PostTurnPause = stehen,
Approaching = geradeaus, Turning = drehen (omega je Richtung),
ExitStraight = geradeaus aus der Kreuzung. Aktiv nur bei `/enable/intersection == True`.
**Publiziert:** `/car_cmd_switch_node/cmd` (Twist2DStamped)

### configuration_node / camera_dashboard_node
Live-Kalibrierung (Schieberegler, schreibt in die JSONs) bzw.
Debug-Visualisierung. Das Dashboard zeigt zusätzlich die aktuelle Phase und die
gewählte Abbiegerichtung. Greifen nicht ins Fahren ein.

---

## Topics

| Topic | Typ | Von -> Nach |
|---|---|---|
| `/detect/lane` | Float64 | detect_lane -> control_lane |
| `/detect/stop_line` | Bool | detect_lane -> FSM |
| `/detect/apriltag/direction` | String | detect_apriltag -> FSM |
| `/detect/apriltag/id` | Int32 | detect_apriltag -> (Dashboard) |
| `/intersection/phase` | String | FSM -> control_intersection, Dashboard |
| `/intersection/direction` | String | FSM -> control_intersection, detect_apriltag, Dashboard |
| `/enable/lane` | Bool | FSM -> control_lane |
| `/enable/intersection` | Bool | FSM -> control_intersection |
| `/car_cmd_switch_node/cmd` | Twist2DStamped | control_lane / control_intersection -> Motoren |

---

## Konfigurationsparameter

| Node | Gruppe | Zweck |
|---|---|---|
| detect_lane | `yellow`/`white`/`red` | HSV-Schwellen der Linien; `crop_image` = Bird's-Eye-Eckpunkte |
| detect_lane | `red.detection_zone` | wie weit vorn die rote Linie gesucht wird (kleiner = früher) |
| detect_apriltag | `tag_memory` | Gedächtnis-Dauer + Mindestfläche eines "nahen" Tags |
| detect_apriltag | `tag_filter` | Stabilitäts-Frames, Positionsfilter, Mindestgröße |
| switch_control | `timing.stop_duration` | Haltezeit an der roten Linie |
| switch_control | `timing.pre_turn_pause` | Debug-Pausen vor + nach dem Drehen (0 = aus) |
| switch_control | `timing.approaching_timeout` | Sicherheits-Timeout fürs Approaching |
| switch_control | `approach_min_time` | Fahrzeit in die Kreuzung vor dem Drehen, je Richtung (links länger) |
| switch_control | `turn_time` | Drehzeit je Richtung (links/rechts/straight) |
| switch_control | `exit_time` | Geradeaus-Zeit nach dem Drehen je Richtung (links länger) |
| control_intersection | `approaching`/`turning` | Geschwindigkeiten + Drehrate |
| control_lane | `pid` | PID-Werte, MIN/MAX_VEL |

---

## Bot-spezifische Parameter

`util.py` lädt aus jeder JSON zuerst den `default`-Block und überschreibt ihn
mit einem bot-spezifischen Block, falls vorhanden:

```
parameters.default            -> gilt für alle Bots
parameters.<vehicle_name>     -> Overrides für diesen Bot (z.B. HSV, Timings)
```

`detect_lane` und `control_lane` haben die Bot-Struktur (echte Kalibrierung pro
Bot). `control_intersection` ist flach (für alle Bots gleich). Bei flacher
Struktur meldet util "verwende default" – das ist dort normal.

**Wichtig:** Der `configuration_node` schreibt die JSON beim Speichern neu. Hat
sie einen `default`-Block, legt er einen Block mit dem aktuellen Bot-Namen an;
ist sie flach (kein `default`), bleibt sie flach.

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

## Setup & Starten

```bash
# ROS-Umgebung + Bot setzen (Beispiel: track)
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://track.local:11311
export VEHICLE_NAME=track

# Alles über den Launcher starten (empfohlen):
launchers/intersection_handling.sh

# Oder einzeln (je ein Terminal):
rosrun intersection_handling detect_lane_node.py
rosrun intersection_handling detect_apriltag_node.py
rosrun intersection_handling switch_control_node.py
rosrun intersection_handling control_lane_node.py
rosrun intersection_handling control_intersection_node.py
rosrun intersection_handling configuration_node.py     # Kalibrierungs-GUI
rosrun intersection_handling camera_dashboard_node.py  # Debug-Ansicht
```

`ROS_IP` muss auf die eigene IP zeigen (nicht die Docker-Bridge `172.17.x.x`).
Empfehlung im `netzwerk.sh`:
```bash
export ROS_IP=$(ip route get <BOT-IP> | grep -oP 'src \K[0-9.]+')
```

---

## Kalibrierung

1. `configuration_node` + `camera_dashboard_node` (oder die imshow-Fenster) starten.
2. **Spurfarben:** Gelb-/Weiß-Maske beobachten, HSV so einstellen, dass die Linie
   sauber erkannt wird. Blasses Klebeband: `yellow.sl` runter, `white.sh` runter.
3. **Rote Linie:** Rot-Maske prüfen, `pixel_threshold` + `detection_zone` einstellen.
4. **AprilTag:** Im AprilTag-Fenster ID + Fläche prüfen; `tag_memory.min_area` so,
   dass ein naher Tag zuverlässig gemerkt wird.
5. **Abbiegen (zeitgesteuert):** `pre_turn_pause` hochsetzen, um nach dem Drehen
   die Position zu sehen. `turn_time` je Richtung einstellen, bis der Bot korrekt
   in der Zielspur landet. Dann `exit_time` je Richtung so, dass der Bot nach dem
   Drehen weit genug geradeaus fährt, bis die Spur im Bild ist (links länger).
   Zum Schluss `pre_turn_pause` = 0 für den flüssigen Lauf.

---

## Mögliche Optimierungen

Der aktuelle Ablauf ist bewusst **zeitgesteuert** (robust, einfach zu
kalibrieren). Für mehr Präzision und Unabhängigkeit von festen Zeiten gibt es
drei sinnvolle Ausbaustufen:

1. **Turning mit Abbruchkriterium statt fester Zeit.**
   Statt `turn_time` ablaufen zu lassen, die Drehung beenden, sobald ein
   sichtbares Kriterium erfüllt ist – z.B. die **rote Linie der Gegenspur** im
   erwarteten Bildbereich (bei Linksdrehung im linken unteren Bereich, bei
   Rechtsdrehung im rechten). Vorteil: unabhängig von Drehgeschwindigkeit,
   Akkustand und Reibung. Erfordert eine zusätzliche Erkennung (eine eigene
   Node oder eine Region in detect_lane), die nur in der Turning-Phase aktiv ist.

2. **ExitStraight mit Abbruchkriterium statt fester Zeit.**
   Statt `exit_time` ablaufen zu lassen, geradeaus fahren, **bis die Lane
   Detection wieder eine gültige Spur meldet**, und erst dann an den PID
   übergeben. Vorteil: passt sich automatisch an unterschiedliche
   Kreuzungsgrößen und Anfahrwinkel an. Voraussetzung: detect_lane muss ein
   verlässliches "Spur gefunden / nicht gefunden"-Signal liefern (kein
   Fallback-Wert, der eine Spur vortäuscht).

3. **Ausrichten an der roten Haltelinie (Orthogonalität).**
   Vor dem Durchfahren den Bot so drehen, dass er **senkrecht zur roten
   Haltelinie** steht. Aus der Neigung der roten Linie im Kamerabild lässt sich
   der Schräglagewinkel ableiten; der Bot dreht im Stand, bis die Linie
   waagerecht erscheint. Vorteil: löst das Problem an der Wurzel – ein schräg
   angekommener Bot startet trotzdem gerade und fährt sauber durch die Kreuzung,
   statt über die Linien hinauszufahren. Aufwändigste, aber sauberste Lösung.

---

## Bekannte Probleme & Lösungen

| Problem | Ursache | Lösung |
|---|---|---|
| `Unable to start XML-RPC server, port 0` | `ROS_IP` falsch / Docker-Bridge / hängender Prozess | richtige `ROS_IP` setzen, `pkill -9 -f node.py`, ggf. Neustart |
| Verzögertes Dashboard-Bild, imshow aber flüssig | Dashboard baut mehrere Topics über WLAN zusammen; imshow zeichnet lokal im Prozess | normal; fürs Debuggen imshow vertrauen. WLAN-Last senken |
| "verwende default" trotz Kalibrierung | Meldung stammt von einer Node mit flacher JSON (kein default-Block) | normal, kein Fehler |
| Gelbe/grüne Linie wird als weiß erkannt | Klebeband zu wenig gesättigt | `yellow.sl` runter, `white.sh` runter; ggf. farbkräftigeres Band |
| Bot steht trotz `enable/lane=True` | control_lane nicht gestartet / JSON-Parameter fehlt (z.B. min_vel) | `rosnode list` prüfen, control_lane einzeln starten und Fehler lesen |
| JSON-Fehler `Expecting property name` | Komma-Fehler beim Editieren (trailing comma) | `python3 -m json.tool <datei>` zeigt die Stelle |
| Bot findet nach Linkskurve die Spur nicht | weite Kurve, nach dem Drehen nur Kreuzungsfläche im Bild | `exit_time.left` erhöhen, bis Spur sicher sichtbar |
