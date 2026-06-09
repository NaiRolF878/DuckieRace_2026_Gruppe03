# Challenge 2 – Intersection Handling

> ROS 1 (Noetic) · Ubuntu 20.04 · Python 3 · OpenCV · pupil_apriltags

Der Duckiebot folgt der Spur (wie Challenge 1), erkennt an Kreuzungen die **rote
Haltelinie** und das zugehörige **AprilTag**, hält an, wählt zufällig eine der
vom Tag erlaubten Richtungen und biegt ab. Ein zentraler Zustandsautomat (FSM)
steuert den gesamten Kreuzungs-Ablauf.

---

## Inhaltsverzeichnis

- [Dateien](#dateien)
- [Grundidee](#grundidee)
- [Systemüberblick](#systemüberblick)
- [Die vier Phasen](#die-vier-phasen)
- [Nodes](#nodes)
- [Topics](#topics)
- [Konfigurationsparameter](#konfigurationsparameter)
- [Bot-spezifische Parameter](#bot-spezifische-parameter)
- [Tag-Mapping](#tag-mapping)
- [Setup & Starten](#setup--starten)
- [Kalibrierung](#kalibrierung)
- [Umschaltbare Varianten](#umschaltbare-varianten)
- [Bekannte Probleme & Lösungen](#bekannte-probleme--lösungen)

---

## Dateien

| Datei | Typ | Beschreibung |
|---|---|---|
| `detect_lane_node.py` | Node | Spurerkennung (CLAHE, Frame-Tracking) + rote Haltelinie (Bird's-Eye) |
| `detect_apriltag_node.py` | Node | AprilTag-Erkennung, Tag-Gedächtnis, Positionsfilter |
| `detect_red_lane_node.py` | Node | Gegenspur-Linie beim Abbiegen (Abbruchkriterium Turning) |
| `switch_control_node.py` | Node | **FSM** – trifft alle Entscheidungen, schaltet Steuerungs-Nodes |
| `control_lane_node.py` | Node | PID-Spurregler (reiner PID, kein StopState) |
| `control_intersection_node.py` | Node | Fährt die Kreuzung (Approaching / Turning / Handover) |
| `configuration_node.py` | Node | Live-Kalibrierungs-GUI mit JSON-Persistenz |
| `camera_dashboard_node.py` | Node | Kamera-Dashboard (Debug-Ansicht) |
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

---

## Systemüberblick

```
                          Kamera
            (/camera_node/image/compressed)
                            │
        ┌───────────────────┼────────────────────────┐
        ▼                   ▼                         ▼
  detect_lane         detect_apriltag         detect_red_lane
   │     │                  │                         │
 lane  stop_line     direction / id            turn_complete
   │     │                  │                         │
   │     └────────┐         │                         │
   ▼              ▼         ▼                          ▼
   │        ┌──────────────────────────────────────────┐
   │        │           switch_control  (FSM)           │  ◄── Entscheidungen
   │        │   Kreuzung? · Richtung würfeln · Phase    │
   │        └──────────────────────────────────────────┘
   │           │ enable/lane     │ enable/intersection
   │           │                 │ phase · direction
   ▼           ▼                 ▼
control_lane              control_intersection
   │                            │
   └─────────────┬──────────────┘
                 ▼
       /car_cmd_switch_node/cmd  →  Motoren
```

Strikte Trennung: **Wahrnehmungs-Nodes** erkennen nur und liefern Signale.
**Steuerungs-Nodes** fahren nur. Entscheidungen fallen ausschließlich in der FSM.

---

## Die vier Phasen

| Phase | Was passiert | Aktive Steuerung |
|---|---|---|
| **Lane** | Normales Spurfolgen, wartet auf Kreuzung | control_lane |
| **Approaching** | Geradeaus über die Haltelinie fahren | control_intersection |
| **Turning** | In gewählte Richtung abbiegen | control_intersection |
| **Handover** | Sanft zurück in die Spur, bis stabil → Lane | control_intersection |

---

## Nodes

### detect_lane_node
Bird's-Eye-View, CLAHE-Helligkeitsausgleich, Frame-Tracking gegen
Linien-Sprünge. Erkennt gelbe + weiße Spurlinie (→ Spurversatz) und die rote
Haltelinie im unteren Bildbereich.
**Publiziert:** `/detect/lane` (Float64), `/detect/stop_line` (Bool)

### detect_apriltag_node
Erkennt AprilTags (Familie *tagStandard52h13*, IDs 1–4) im Originalbild.
**Tag-Gedächtnis:** ein naher Tag wird einige Sekunden gemerkt (Tag und rote
Linie sind selten gleichzeitig sichtbar). **Positionsfilter:** nur rechte
Bildhälfte (Rechtsverkehr). **Stabilitätsfilter:** ID muss mehrere Frames stabil
sein.
**Publiziert:** `/detect/apriltag/direction` (String), `/detect/apriltag/id` (Int32)

### detect_red_lane_node
Abbruchkriterium für die Turning-Phase. Sucht die Gegenspur-/Querlinie im
Originalbild **nur in der erwarteten Region** (links-drehen → rechts suchen,
rechts-drehen → links). Nimmt die größte zusammenhängende rote Fläche und meldet
"fertig" erst nach dem Prinzip *erst leer sehen, dann Wiederauftauchen* – robust
gegen die mehreren roten Linien an einer Kreuzung.
**Publiziert:** `/intersection/turn_complete` (Bool)

### switch_control_node — die FSM (Entscheidungsebene)
Einzige Node mit Zustandslogik.
- **Kreuzungs-Erkennung:** rote Linie **und** bekannte Tag-Richtung im
  Lane-Zustand → Kreuzung. Rote Linie ohne Tag → ignoriert (Bot fährt weiter).
- **Richtungswahl:** beim Eintritt einmalig zufällig aus den erlaubten Richtungen.
- **Phasensteuerung:** Lane → Approaching → Turning → Handover → Lane.
- **Aktiviert/deaktiviert** control_lane und control_intersection.
**Publiziert:** `/enable/lane` (Bool), `/enable/intersection` (Bool),
`/intersection/phase` (String), `/intersection/direction` (String)

### control_lane_node
Reiner PID-Spurregler (Anti-Windup, MIN_VEL). **Kein** StopState mehr – an der
Kreuzung übernimmt die FSM. Aktiv nur bei `/enable/lane == True`.
**Publiziert:** `/car_cmd_switch_node/cmd` (Twist2DStamped)

### control_intersection_node
Fährt die Kreuzung je nach Phase: Approaching = geradeaus, Turning = drehen
(Richtung je nach Wahl), Handover = sanfter P-Regler zurück in die Spur. Aktiv
nur bei `/enable/intersection == True`.
**Publiziert:** `/car_cmd_switch_node/cmd` (Twist2DStamped)

### configuration_node / camera_dashboard_node
Live-Kalibrierung (Schieberegler, schreibt in die JSONs) bzw.
Debug-Visualisierung. Greifen nicht ins Fahren ein.

---

## Topics

| Topic | Typ | Von → Nach |
|---|---|---|
| `/detect/lane` | Float64 | detect_lane → control_lane, control_intersection, FSM |
| `/detect/stop_line` | Bool | detect_lane → FSM |
| `/detect/apriltag/direction` | String | detect_apriltag → FSM |
| `/detect/apriltag/id` | Int32 | detect_apriltag → (Dashboard) |
| `/intersection/turn_complete` | Bool | detect_red_lane → FSM |
| `/intersection/phase` | String | FSM → control_intersection, detect_red_lane |
| `/intersection/direction` | String | FSM → control_intersection, detect_red_lane |
| `/enable/lane` | Bool | FSM → control_lane |
| `/enable/intersection` | Bool | FSM → control_intersection |
| `/car_cmd_switch_node/cmd` | Twist2DStamped | control_lane / control_intersection → Motoren |

---

## Konfigurationsparameter

Jede Node hat eine `<node>.json` mit `default`-Werten (und optional
bot-spezifischen Overrides). Die wichtigsten:

| Node | Gruppe | Zweck |
|---|---|---|
| detect_lane | `yellow`/`white`/`red` | HSV-Schwellen der Linien; `crop_image` = Bird's-Eye-Eckpunkte |
| detect_apriltag | `tag_memory` | Gedächtnis-Dauer + Mindestfläche eines „nahen" Tags |
| detect_apriltag | `tag_filter` | Stabilitäts-Frames, Positionsfilter (rechte Hälfte), Mindestgröße |
| detect_red_lane | `region` | Schwelle + Regionsgrenzen (`split_lo`/`split_hi`) für die Suche |
| switch_control | `timing` | Phasen-Dauern und Timeouts |
| switch_control | `turn_time` | Drehzeiten pro Richtung (nur zeitgesteuerte Variante) |
| switch_control | `handover` | Wann gilt die Spur als wieder stabil? |
| control_intersection | `approaching`/`turning`/`handover` | Geschwindigkeiten + Drehrate |
| control_lane | `pid` | PID-Werte, MIN/MAX_VEL |

---

## Bot-spezifische Parameter

`util.py` lädt aus jeder JSON zuerst den `default`-Block und überschreibt ihn
mit einem bot-spezifischen Block, falls vorhanden:

```
parameters.default            → gilt für alle Bots
parameters.<vehicle_name>     → Overrides für diesen Bot (z.B. HSV-Kalibrierung)
```

Beim Start wird gemeldet, ob ein bot-spezifischer Block geladen wurde oder
default verwendet wird. **Hinweis:** Die neuen Intersection-Nodes haben nur
`default` – die Meldung „verwende default" ist dort normal und korrekt.

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
# ROS-Umgebung + Bot setzen (Beispiel: donald)
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://donald.local:11311
export VEHICLE_NAME=donald

# Alles über den Launcher starten (empfohlen):
launchers/intersection_handling.sh

# Oder einzeln (je ein Terminal):
rosrun intersection_handling detect_lane_node.py
rosrun intersection_handling detect_apriltag_node.py
rosrun intersection_handling detect_red_lane_node.py
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

1. `configuration_node` + `camera_dashboard_node` starten.
2. **Spurfarben:** Gelb-Maske (`/debug/lane_yellow`) und Weiß-Maske
   (`/debug/lane_white`) beobachten, HSV-Schieberegler so einstellen, dass die
   jeweilige Linie sauber erkannt wird und die andere nicht mitkommt.
3. **Rote Linie:** Rot-Maske (`/debug/lane_red`) prüfen, `pixel_threshold` und
   `detection_zone` einstellen.
4. **AprilTag:** Im AprilTag-Debug-Bild ID + Fläche prüfen; `tag_memory.min_area`
   so wählen, dass ein „naher" Tag zuverlässig gemerkt wird.
5. **Abbiegen:** Im Red-Lane-Debug die roten Pixel links/rechts beobachten und
   `region.threshold` einstellen.

Werte werden über den `configuration_node` in die jeweilige JSON persistiert.

---

## Umschaltbare Varianten

In `switch_control_node.py` (`_update_state`) per Kommentar umschaltbar:

- **Approaching-Ende:** distanzbasiert (Standard) ↔ zeitgesteuert
- **Turning-Ende:** regionsbasiert via detect_red_lane (Standard) ↔ zeitgesteuert
  (feste Drehzeit pro Richtung aus `turn_time`)

So lässt sich am Prüfungstag in Sekunden auf die robustere Variante wechseln.

---

## Parameter
**switch_control_node.json — alle Parameter**
**timing:**

- **stop_duration** — wie lange der Bot an der roten Haltelinie steht (Phase Stopping)
  pre_turn_pause — Debug-Pause vor dem Drehen; 0 = aus
- **approaching_timeout** — Sicherheitsnetz: nach so vielen Sekunden geht Approaching zwangsweise weiter,     falls die Linie nie „verschwindet"
- **approaching_duration** — nur relevant für die zeitgesteuerte Approaching-Variante B (aktuell auskommentiert)
- **turning_timeout** — Sicherheitsnetz fürs Drehen: nach so vielen Sekunden endet Turning zwangsweise, falls Node B nie „fertig" meldet
- **handover_timeout** — Sicherheitsnetz fürs Einscheren

**approach_min_time** (richtungsabhängig — das, was wir getrennt haben):

- **left** — wie weit der Bot nach dem Verschwinden der Linie noch geradeaus fährt, bevor er links dreht. Größer = weiter in die Kreuzung. Gegen „dreht zu früh nach links".
- **right** — dasselbe für rechts. Kleiner, weil engere Kurve.
- **straight** — Mindest-Fahrzeit bei Geradeausfahrt.

turn_time (nur für zeitgesteuerte Turning-Variante B):

left / right / straight — feste Drehdauer pro Richtung. Nur aktiv, wenn ihr die regionsbasierte Variante (Node B) auskommentiert.

**handover:**

**lane_stable_threshold** — wie mittig die Spur sein muss, damit sie als „wieder gefunden" gilt (kleiner = strenger)
**lane_stable_required** — wie viele Frames in Folge die Spur stabil sein muss, bevor zurück zu Lane geschaltet wird

**detect_apriltag_node.json — alle Parameter**
**tag_memory:**

**seconds** — wie lange ein gesehener Tag „im Gedächtnis" bleibt, nachdem er aus dem Bild verschwunden ist. Größer = überbrückt längere Lücken zwischen Tag und roter Linie, aber riskiert, einen veralteten Tag zu verwenden.
**min_area** — wie groß (nah) ein Tag sein muss, damit er ins Gedächtnis aufgenommen wird. Größer = nur sehr nahe Tags zählen (verhindert, dass ein weit entfernter Tag einer anderen Kreuzung gemerkt wird).

**tag_filter:**

stability_frames — wie viele Frames in Folge dieselbe Tag-ID erkannt werden muss, bevor sie als gültig gilt. Größer = robuster gegen Fehlerkennungen, aber träger.
pos_filter_enabled — 1 = nur Tags in einem bestimmten Bildbereich zählen (Positionsfilter an), 0 = alle Tags.
pos_x_min / pos_x_max — horizontaler Bereich (0.0 = links, 1.0 = rechts), in dem Tags akzeptiert werden. Default 0.5–1.0 = nur rechte Bildhälfte (Schild steht rechts).
pos_y_max — bis zu welcher Bildhöhe Tags zählen (0.0 = oben, 1.0 = unten). Filtert z.B. Tags, die zu weit unten/nah sind.
min_area — Mindestgröße, damit ein Tag überhaupt erkannt wird (kleiner Filter gegen winzige Fehldetektionen). Nicht zu verwechseln mit tag_memory.min_area — dieser hier ist die generelle Erkennungsschwelle, der andere die strengere Schwelle fürs Gedächtnis.

---

## Bekannte Probleme & Lösungen

| Problem | Ursache | Lösung |
|---|---|---|
| `Unable to start XML-RPC server, port 0` | `ROS_IP` falsch / Docker-Bridge / hängender Prozess | richtige `ROS_IP` setzen, alte Prozesse killen, ggf. Neustart |
| Verzögertes Kamerabild | WLAN-Last (mehrere Nodes ziehen den Stream) | `frame_skip` erhöhen, imshow-Fenster aus, Nodes ggf. auf den Bot verlagern |
| „verwende default" trotz Bot-Block | Meldung stammt von einer Intersection-Node (hat nur default) | normal, kein Fehler |
| Bot steht trotz `enable/lane=True` | control_lane nicht gestartet / sendet `v=0` | `rosnode list` prüfen, control_lane einzeln starten und Fehler lesen |
