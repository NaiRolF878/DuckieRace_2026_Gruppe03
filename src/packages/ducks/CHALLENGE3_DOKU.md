# Challenge 3 – Watch out for Ducks: Technische Dokumentation

> Für Teammitglieder, die in den Code einsteigen und verstehen wollen, wie alles zusammenspielt.

---

## 1. Das Problem in einem Satz

Der Bot soll Enten (und gelbe Linien) erkennen, ausweichen, und danach sicher auf die Spur zurückfinden – ohne die gelbe Mittellinie zu überfahren und ohne die Grundfunktion (Spurfolgen, rote Haltelinie) zu zerstören.

---

## 2. Strategie: Warum so und nicht anders?

### Das Kernproblem der alten Lösung

Die alte Spurführung navigierte zwischen gelber Linie (links) und weißer Linie (rechts). Das hat zwei unlösbare Probleme erzeugt:

- **Gelbe Linie ≈ gelbe Ente** – optisch nicht zu unterscheiden
- Auf dem Wendeplatz gibt es keine gelbe Linie – der Bot hatte keine Referenz

### Die neue Strategie

**Nur noch die weiße Linie als Referenz** – die gelbe Linie wird komplett aus der Spurlogik entfernt.

**Gelbe Linie = Objekt** – sie löst dasselbe Ausweichen aus wie eine Ente. Dadurch muss sie nie unterschieden werden – beide bekommen die gleiche Reaktion.

**Zonenbasierte Erkennung** – statt Farbe wird Helligkeit genutzt. Helle Objekte auf dunklem Untergrund lösen aus. Das erkennt gelbe Enten, andersfarbige Enten, gelbe Linien – alles, was im Weg ist.

---

## 3. Architektur: Welche Dateien gibt es?

```
src/packages/ducks/
├── src/
│   ├── detect_lane_node.py       ← Kamera: BEV, Spurerkennung, Zonen, Enten
│   ├── control_lane_node.py      ← PID-Regler: fährt den Bot
│   ├── control_obstacle_node.py  ← Zustandsautomat: Ausweichlogik
│   ├── switch_control_node.py    ← Schalter: wer darf gerade steuern?
│   ├── util.py                   ← Parameter-Loader (JSON → Python)
│   ├── configuration_node.py     ← Live-Slider-GUI für Parameter
│   └── camera_dashboard_node.py  ← 2×2 Debug-Dashboard
│
└── config/
    ├── detect_lane_node.json      ← Parameter für Kamera/Erkennung
    ├── control_lane_node.json     ← PID-Parameter, Haltelinien-Timing
    └── control_obstacle_node.json ← Ausweich-Parameter (Offsets, Timeouts)
```

---

## 4. Was macht jede Node?

### `detect_lane_node.py` – Die Augen des Bots

Nimmt das Kamerabild und macht daraus Steuersignale:

1. **Bird's-Eye-View (BEV)** – Bild wird perspektivisch entzerrt (400×400 px, Blick von oben)
2. **Weiße Linie finden** – HSV-Filter → Position der weißen Linie im Bild
3. **Spurversatz berechnen** – `error = 1 - (lane_center / 400 * 2)`, Bereich [-1, +1]
   - `error > 0` → Bot zu weit links → muss nach rechts lenken
   - `error < 0` → Bot zu weit rechts → muss nach links lenken
4. **Weißlinien-Follow mit festem Offset** – Zielposition = weiße Linie minus `offset_px` Pixel
5. **Rote Haltelinie erkennen** → Bool-Signal
6. **Entenerkennung** → horizontale Position des Blobs (`duck_x`, -99 = kein Objekt)
7. **Zonen-Belegung im BEV** – 3 Zonen (nah/mittel/fern) auf Helligkeit prüfen

**Publizierte Topics:**

| Topic | Typ | Inhalt |
|-------|-----|--------|
| `/tick/detect/lane` | `Float64` | Spurversatz [-1, +1] |
| `/tick/detect/stop_line` | `Bool` | Rote Linie sichtbar? |
| `/tick/detect/duck` | `Float64` | x-Position des Entenkopfs (-99 = kein Blob) |
| `/tick/detect/zones` | `Float32MultiArray` | [nah, mittel, fern] je 0.0 oder 1.0 |

---

### `control_lane_node.py` – Der Fahrer

Diese Node ist die **einzige**, die den Fahrbefehl sendet. Sie berechnet aus dem Spurversatz mittels **PID-Regler** v (Geschwindigkeit) und omega (Lenkung).

**Priorität der Fahrbefehle** (von oben nach unten, erstes Zutreffende gewinnt):

```
1. obstacle_stop = True      → v=0, omega=0  (Ente blockiert Weg, Stufe 6)
2. StopState.Stopping        → v=0, omega=0  (rote Haltelinie erkannt)
3. return_omega ≠ 0          → v=PID, omega=return_omega  (Encoder-Rückkehr, Stufe 5)
4. Normalbetrieb             → v=PID, omega=PID
```

**Subscriptions:**

| Topic | Kommt von | Wofür |
|-------|-----------|-------|
| `/tick/detect/lane` | detect_lane_node | Spurversatz → PID-Eingang |
| `/tick/detect/stop_line` | detect_lane_node | Haltelinien-Automat |
| `/tick/enable/lane` | switch_control_node | Node ein/aus |
| `/tick/obstacle/error_offset` | control_obstacle_node | Ausweich-Offset (Stufe 4) |
| `/tick/obstacle/return_omega` | control_obstacle_node | Encoder-Rückkehr omega (Stufe 5) |
| `/tick/obstacle/stop` | control_obstacle_node | Vollstopp (Stufe 6) |

**Der Ausweich-Offset** – das Kernprinzip des Ausweichens:

```
Normalbetrieb:  error = Spurversatz                → Bot folgt weißer Linie
Ausweichen:     error = Spurversatz + error_offset  → Bot meint, er ist verschoben
                                                       PID korrigiert → Bot weicht aus
```

Der Offset verschiebt die wahrgenommene Spurmitte. Der Bot "glaubt", er wäre zu weit rechts (oder links) und lenkt entsprechend.

---

### `control_obstacle_node.py` – Der Stratege

Enthält den **Zustandsautomaten** für das gesamte Ausweichmanöver. Sendet keine Fahrbefehle direkt – steuert nur über die drei Topics, die `control_lane_node` verarbeitet.

---

### `switch_control_node.py` – Der Schalter

Entscheidet, ob `control_obstacle_node` aktiv ist. Publiziert `enable/lane` (immer True) und `enable/obstacle` (nur True wenn Objekt in Zonen erkannt).

**Wichtig:** `control_lane_node` läuft immer! Nur `control_obstacle_node` wird ein/ausgeschaltet.

---

## 5. Der Zustandsautomat (das Herzstück)

```
                    ┌─────────────────────────────────────────────────┐
                    │                                                 │
                    ▼                                                 │
              ┌──────────┐   nah/mittel       ┌──────────┐           │
              │   IDLE   │ ─── belegt ──────► │  EVADE   │           │
              │          │                    │ Ausweichen│           │
              └──────────┘                    └──────────┘           │
                    ▲                          │         │            │
                    │                  frei    │     Timeout          │
                    │                          ▼         ▼            │
                    │                    ┌──────────┐ ┌──────────┐   │
                    │                    │   PASS   │ │   WAIT   │   │
                    │                    │ Nachlauf │ │  v = 0   │   │
                    │                    └──────────┘ └──────────┘   │
                    │                          │    frei / Timeout    │
                    │               Nachlauf   │ ◄────────────────────┘
                    │               abgelaufen ▼
                    │                    ┌──────────┐
                    └────────── fertig ─ │  RETURN  │
                                         │ Encoder- │
                                         │ Rückkehr │
                                         └──────────┘
```

### Was in jedem Zustand passiert:

#### IDLE – Normalbetrieb
- `error_offset = 0` → kein Eingriff, Bot folgt der weißen Linie normal
- Übergang → EVADE: Zone **nah** oder **mittel** ist belegt

#### EVADE – Ausweichen
- `error_offset = ±locked_offset` → Bot weicht zur freien Seite aus
- Richtung wird beim Eintritt einmal festgelegt und bleibt für das gesamte Manöver **eingefroren** (auch wenn die Ente sich bewegt)
- Gleichzeitig: Encoder-Ticks akkumulieren (für spätere Rückkehr)
- Übergang → PASS: alle Zonen leer
- Übergang → WAIT: Timeout überschritten (Bot ist zu lange am Ausweichen)

#### WAIT – Anhalten (Stufe 6)
- `obstacle/stop = True` → `control_lane_node` setzt v=0
- Bot wartet bis Weg frei oder `wait_timeout_secs` abgelaufen
- Übergang → PASS: Zonen leer ODER Timeout

#### PASS – Nachlauf
- `error_offset = locked_offset` (Offset bleibt noch aktiv, damit Bot sicher an Ente vorbeifährt)
- Encoder-Ticks akkumulieren weiter
- Übergang → EVADE: Ente taucht wieder auf
- Übergang → RETURN: `nachlauf_secs` abgelaufen

#### RETURN – Encoder-Rückkehr (Stufe 5)
- `error_offset = 0` (kein Spurversatz-Eingriff mehr)
- `return_omega` wird publiziert → Bot dreht in entgegengesetzter Richtung wie beim Ausweichen
- Ticks werden heruntergezählt (gespiegelte Bewegung zur Rückkehr)
- Übergang → IDLE: Kamera sieht weiße Linie (primär) ODER Encoder-Ticks aufgebraucht (Backup)
- Übergang → EVADE: neue Ente erkannt

### Ausweichrichtung – wie wird sie bestimmt?

```python
duck_x >= 0  →  Ente rechts von BEV-Mitte  →  offset negativ  →  nach links ausweichen
duck_x <  0  →  Ente links  von BEV-Mitte  →  offset positiv  →  nach rechts ausweichen
duck_x = -99 →  kein Blob (z.B. gelbe Linie) →  offset positiv  →  rechts als sicherer Standard
```

---

## 6. Die Zonen im Bird's-Eye-View

```
BEV-Bild (400 × 400 px, Blick von oben)

  0 ─────────────────────────────── 400
  │        [Fahrtrichtung]          │
  │                                 │
  │  ...... FERN (fern_y_min/max)   │  ← Frühwarnung
  │  ...... MITTEL (mid_y_min/max)  │  ← Ausweichen auslösen
  │  ...... NAH   (near_y_min/max)  │  ← Ausweichen auslösen
  │                                 │
  │         [Bot ist hier]          │
400─────────────────────────────── 400
  │← corridor_x_min   x_max →│

```

- Die Zonen decken nur den **Fahrkorridor** ab (nicht das volle Bild)
- Erkennung: Helligkeits-Threshold (helle Pixel > `pixel_threshold_frac` der Zonenfläche → belegt)
- Erkennt alles Helle: gelbe Enten, andersfarbige Enten, gelbe Linie

---

## 7. Encoder-Rückkehr – wie funktioniert das genau?

### Das Problem mit Encodern beim Duckiebot

Der Encoder liefert eine **kumulative Zählzahl** (`data`), die immer aufwärts zählt – egal ob vorwärts oder rückwärts gefahren wird. Die Richtung ist aus den Encoder-Daten allein **nicht** ablesbar.

**Lösung:** Die Richtung wird aus dem Vorzeichen von `locked_offset` abgeleitet:
- Bot hat nach links ausgewichen (`locked_offset < 0`) → Rückkehr nach rechts (`return_omega > 0`)
- Bot hat nach rechts ausgewichen (`locked_offset > 0`) → Rückkehr nach links (`return_omega < 0`)

### Ablauf

```
EVADE + PASS: delta_ticks = (Δlinks + Δrechts) / 2   →  accumulated_ticks += delta
                                                          (jeder Schritt des Ausweichens wird gezählt)

RETURN:       return_ticks_remaining = accumulated_ticks   (Start mit gespiegeltem Betrag)
              jeder Schritt: return_ticks_remaining -= delta_ticks
              Abbruch wenn:
                a) |lane_error| < return_threshold für N aufeinanderfolgende Frames  (primär)
                b) return_ticks_remaining ≤ 0                                         (Backup)
```

Die Encoder-Genauigkeit ist dabei **unkritisch**: Schlupf verlängert höchstens die Rückkehr minimal. Die Kamera als primäre Abbruchbedingung korrigiert alles.

---

## 8. Parameter-Übersicht – was kann ich wo einstellen?

Alle Parameter sind in den JSON-Dateien unter `config/` und können **live** über den `configuration_node` angepasst werden (kein Neustart nötig).

### `detect_lane_node.json`

| Gruppe | Parameter | Beschreibung |
|--------|-----------|-------------|
| `white_follow` | `offset_px` | Sollabstand zur weißen Linie in BEV-Pixeln (Standard: 150) |
| `white` | `vl`, `vh`, `sl`, `sh` | HSV-Bereich für weiße Linie |
| `duck` | `enabled` | Entenerkennung ein/aus |
| `zones` | `corridor_x_min/max` | Breite des Fahrkorridors (0–1, relativ zu 400px) |
| `zones` | `near/mid/far_y_min/max` | Lage der drei Zonen (0 = oben = fern, 1 = unten = nah) |
| `zones` | `pixel_threshold_frac` | Anteil heller Pixel ab dem eine Zone als belegt gilt (Standard: 0.05 = 5%) |

### `control_obstacle_node.json`

| Parameter | Standard | Beschreibung |
|-----------|----------|-------------|
| `active` | 1 | Gesamte Ausweichlogik ein/aus |
| `evade_offset` | 0.6 | Stärke des Ausweichens (wird zum Spurversatz addiert) |
| `nachlauf_secs` | 1.5 s | Wie lange der Bot nach dem letzten Objekt-Kontakt noch mit Offset weiterfährt |
| `evade_timeout_secs` | 5.0 s | Max. Zeit im EVADE-Zustand bevor WAIT ausgelöst wird |
| `return_threshold` | 0.25 | Spurversatz unterhalb dem die Rückkehr als abgeschlossen gilt |
| `return_stable_frames` | 5 | Wie viele Frames der Versatz < threshold sein muss (Entprellung) |
| `return_omega` | 0.5 | Drehrate bei der Encoder-Rückkehr [rad/s] |
| `wait_timeout_secs` | 3.0 s | Wie lange der Bot im WAIT-Zustand bleibt bevor er weiterfährt |

### `control_lane_node.json`

| Parameter | Beschreibung |
|-----------|-------------|
| `pid.p/i/d` | PID-Faktoren für Spurfolgen |
| `pid.max_vel / min_vel` | Geschwindigkeitsgrenzen |
| `stop_line.stop_duration` | Standzeit an roter Linie [s] |
| `stop_line.cooldown_duration` | Wartezeit bis nächste rote Linie auslöst [s] |

---

## 9. ROS-Topic-Übersicht (vollständig)

> Alle Topics verwenden Prefix `/tick/` (Bot-Name = `tick`)

```
detect_lane_node
    publish:  /tick/detect/lane          Float64        Spurversatz [-1,+1]
              /tick/detect/stop_line     Bool           Rote Linie sichtbar
              /tick/detect/duck          Float64        Enten-x-Position (-99 = kein Blob)
              /tick/detect/zones         Float32MultiArray  [nah, mittel, fern]

control_lane_node
    subscribe: /tick/detect/lane         ← PID-Eingang
               /tick/detect/stop_line    ← Haltelinien-Automat
               /tick/enable/lane         ← Ein/Aus von switch_control_node
               /tick/obstacle/error_offset ← Ausweich-Offset
               /tick/obstacle/return_omega ← Encoder-Rückkehr-omega
               /tick/obstacle/stop        ← Vollstopp-Signal
    publish:  /tick/car_cmd_switch_node/cmd  Twist2DStamped  Fahrbefehl (v, omega)

control_obstacle_node
    subscribe: /tick/detect/zones        ← Auslöser
               /tick/detect/duck         ← Richtungsbestimmung
               /tick/detect/lane         ← Abbruchbedingung RETURN
               /tick/enable/obstacle     ← Ein/Aus von switch_control_node
               /tick/left_wheel_encoder_node/tick   WheelEncoderStamped
               /tick/right_wheel_encoder_node/tick  WheelEncoderStamped
    publish:  /tick/obstacle/error_offset  Float64  Ausweich-Offset
              /tick/obstacle/return_omega  Float64  Rückkehr-omega
              /tick/obstacle/stop          Bool     Vollstopp-Signal
              /tick/obstacle/done          Bool     Ausweichen abgeschlossen

switch_control_node
    subscribe: /tick/detect/zones        ← Umschalten Lane→Obstacle
               /tick/obstacle/done       ← Umschalten Obstacle→Lane
    publish:  /tick/enable/lane          Bool  (immer True)
              /tick/enable/obstacle      Bool  (True wenn Zonen belegt)
```

---

## 10. Debug-Möglichkeiten

### Debug-Bilder (Topics)

| Topic | Inhalt |
|-------|--------|
| `/debug/original` | Rohbild der Kamera |
| `/debug/annotated` | Bild mit erkannten Linien eingezeichnet |
| `/debug/bird_view` | BEV ohne Annotation |
| `/debug/lane_white` | Maske der weißen Linie |
| `/debug/lane_red` | Maske der roten Linie |
| `/debug/duck_bev` | BEV mit Zonen und Enten-Bounding-Box |

```bash
# Debug-Bild anzeigen (auf dem Bot ausführen oder mit richtigem ROS_MASTER_URI):
rosrun image_view image_view image:=/debug/duck_bev
```

### Zustand des Ausweich-Automaten verfolgen

```bash
# Zeigt alle 2s den aktuellen Zustand + Werte im Terminal:
rostopic echo /tick/obstacle/error_offset
rostopic echo /tick/obstacle/stop
rostopic echo /tick/detect/zones
```

### Log-Meldungen im Terminal

`control_obstacle_node` loggt alle Zustandswechsel:
```
[Evade] Auslösung – rechts, Offset +0.60
[Evade] Korridor frei → PASS
[Evade] Nachlauf vorbei → RETURN (akkum. 87 Ticks)
[Evade] Rückkehr fertig (Kamera) → IDLE
```

---

## 11. Häufige Probleme & Lösungen

| Problem | Ursache | Lösung |
|---------|---------|--------|
| Bot weicht aus, obwohl keine Ente da | `pixel_threshold_frac` zu niedrig | Wert erhöhen (z.B. 0.08–0.12) |
| Bot erkennt Ente nicht | `pixel_threshold_frac` zu hoch | Wert senken; Zonen-Position prüfen |
| Rückkehr zu kurz / zu weit | `return_omega` falsch | Anpassen; primär greift die Kamera |
| Bot dreht sich beim Rückkehren zu stark | `return_omega` zu hoch | Senken (z.B. 0.3) |
| Bot hält dauerhaft an (WAIT) | Erkennung falsch positiv | `wait_timeout_secs` kürzer; `pixel_threshold_frac` prüfen |
| Nachlauf zu lang/kurz | `nachlauf_secs` | Anpassen nach Fahrgeschwindigkeit |
| Weiße Linie wird nicht gefunden | HSV-Parameter `white.vl/vh` falsch | Via Configuration-Node live anpassen |

---

## 12. Schnellübersicht: Dateien und ihre Kernaufgabe

```
detect_lane_node.py   →  Kamera auswerten, Signale publizieren
control_lane_node.py  →  PID rechnen, Fahrbefehl senden (EINZIGE Stelle!)
control_obstacle.py   →  Zustandsautomat, Ausweich-Offset berechnen
switch_control.py     →  enable/lane + enable/obstacle schalten
util.py               →  JSON-Parameter laden (nie direkt anfassen)
config/*.json         →  Alle einstellbaren Werte, live änderbar
```
