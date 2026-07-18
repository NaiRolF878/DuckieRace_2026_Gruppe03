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

**Zonenbasierte Erkennung** – Farberkennung (gelb/grün, HSV) statt Helligkeit. Gelbe Enten, grüne Bonus-Enten und die gelbe Mittellinie lösen aus – unbunte Reflexionen/Klebereste auf der Fahrbahn fallen automatisch raus, weil sie nicht in den Farbbereich fallen (mit reiner Helligkeitsschwelle war das nicht unterscheidbar).

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
3. **Spurversatz berechnen:**
   ```
   lane_center = center_white - offset_px       (Zielposition = weiße Linie minus Abstand)
   error = 1 - (lane_center / 400 * 2)          (Normierung auf [-1, +1])
   ```
   - `error > 0` → Bot zu weit links → muss nach rechts lenken
   - `error < 0` → Bot zu weit rechts → muss nach links lenken
4. **Rote Haltelinie erkennen** → Bool-Signal
5. **Entenerkennung** → Farberkennung (gelb/grün) im **unverzerrten Originalbild**
   (kein Sichtfeldlimit durch das BEV-Trapez, keine Höhen-Verzerrung durch die
   Homographie). Nur der Bodenkontaktpunkt jeder erkannten Box wird per
   Homographie ins BEV projiziert → `duck_x`. Ein Kalman-Filter glättet die
   Position und überbrückt kurze Erkennungsaussetzer (statt sofort auf "keine
   Ente" zu springen).
6. **Zonen-Belegung im BEV** – 3 Zonen (nah/mittel/fern) prüfen im Fahrkorridor
   dieselben reprojizierten Boden-Kontaktpunkte aus Schritt 5 (keine eigene
   Farberkennung); der Korridor selbst ist **schmal** und fest um die
   Bildmitte zentriert (entspricht der Bot-Breite, nicht die ganze Spur)

**Publizierte Topics:**

| Topic | Typ | Inhalt |
|-------|-----|--------|
| `/tick/detect/lane` | `Float64` | Spurversatz [-1, +1] |
| `/tick/detect/stop_line` | `Bool` | Rote Linie sichtbar? |
| `/tick/detect/duck` | `Float64` | Kalman-gefilterte x-Position der nächsten Ente ([-1,+1]; -99 = kein Blob) |
| `/tick/detect/zones` | `Float32MultiArray` | `[nah, mittel, fern]` je 0.0 oder 1.0 |
| `/tick/detect/corridor_occupancy` | `Float32MultiArray` | `[links_frei, rechts_frei]` - exakter freier Pixel-Anteil der Korridorbreite (kein Bin-Raster), nah+mittel-Band, gleiche Maske wie Zonen (gelbe Linie zählt als belegt) |

---

### `control_lane_node.py` – Der Fahrer

Diese Node ist die **einzige**, die den Fahrbefehl sendet. Sie berechnet aus dem Spurversatz mittels **PID-Regler** `v` (Geschwindigkeit) und `omega` (Lenkung).

**Priorität der Fahrbefehle** (von oben nach unten, erstes Zutreffende gewinnt):

```
1. emergency_active = True → v/omega = emergency_cmd   (NOTFALL, Stufe 4a, umgeht PID)
2. obstacle_stop = True     → v=0, omega=0              (WAIT-Zustand, Stufe 6)
3. StopState.Stopping       → v=0, omega=0              (rote Haltelinie erkannt)
4. return_omega ≠ 0         → v=PID, omega=return_omega (Encoder-Rückkehr, Stufe 5)
5. Normalbetrieb            → v=PID, omega=PID
```

**Subscriptions:**

| Topic | Kommt von | Wofür |
|-------|-----------|-------|
| `/tick/detect/lane` | detect_lane_node | Spurversatz → PID-Eingang |
| `/tick/detect/stop_line` | detect_lane_node | Haltelinien-Automat |
| `/tick/enable/lane` | switch_control_node | Node ein/aus |
| `/tick/obstacle/error_offset` | control_obstacle_node | Ausweich-Offset (Stufe 4b) |
| `/tick/obstacle/return_omega` | control_obstacle_node | Encoder-Rückkehr omega (Stufe 5) |
| `/tick/obstacle/stop` | control_obstacle_node | Vollstopp-Signal (Stufe 6) |
| `/tick/obstacle/emergency_active` | control_obstacle_node | Notfall aktiv? (Stufe 4a) |
| `/tick/obstacle/emergency_cmd` | control_obstacle_node | v/omega-Vorgabe im Notfall, umgeht PID |

**Der Ausweich-Offset** – das Kernprinzip des Ausweichens:

```
Normalbetrieb:  error = Spurversatz                → Bot folgt weißer Linie
Ausweichen:     error = Spurversatz + error_offset  → Bot "glaubt", er ist verschoben
                                                       PID korrigiert → Bot weicht aus
```

Der Offset verschiebt die wahrgenommene Spurmitte. Der PID-Regler bemerkt den künstlichen Fehler und lenkt den Bot in die gewünschte Richtung – ohne die Fahrtgeschwindigkeit separat anpassen zu müssen.

---

### `control_obstacle_node.py` – Der Stratege

Enthält den **Zustandsautomaten** für das gesamte Ausweichmanöver. Sendet **keine Fahrbefehle direkt** (außer im Notfall) – steuert über die Topics, die `control_lane_node` verarbeitet.

**Publizierte Topics:**

| Topic | Typ | Inhalt |
|-------|-----|--------|
| `/tick/obstacle/error_offset` | `Float64` | Ausweich-Offset (0 = Normalbetrieb) |
| `/tick/obstacle/return_omega` | `Float64` | Rückkehr-Lenkung (0 = Normalbetrieb) |
| `/tick/obstacle/stop` | `Bool` | True nur im WAIT-Zustand |
| `/tick/obstacle/done` | `Bool` | True wenn Manöver abgeschlossen |
| `/tick/obstacle/emergency_active` | `Bool` | True nur im EMERGENCY-Zustand |
| `/tick/obstacle/emergency_cmd` | `Twist2DStamped` | v/omega-Vorgabe im Notfall (Wiggle + feste Drehrate) |
| `/tick/obstacle/state` | `String` | Aktueller Zustand als Klartext (`Idle`/`Evade`/`Wait`/`Pass`/`Return`) – für Debug-Overlays in detect_lane_node und camera_dashboard_node |

---

### `switch_control_node.py` – Der Schalter

Entscheidet, ob `control_obstacle_node` aktiv ist. Publisht `enable/lane` (immer True) und `enable/obstacle` (True wenn Zone nah/mittel belegt).

**Wichtig:** `control_lane_node` läuft **immer**! Nur `control_obstacle_node` wird ein-/ausgeschaltet. Würde der Lane-Node beim Ausweichen abgeschaltet, bliebe der Bot stehen.

---

## 5. Der Zustandsautomat (das Herzstück)

In `control_obstacle_node.py`, Klasse `EvadeState`, Methode `_step_state_machine()`.

Die drei Zonen lösen bewusst **unterschiedliche** Reaktionen aus (nicht mehr
alle drei gleich):
- **fern:** nur Beobachtung, kein Zustandswechsel. Erkennung auf Distanz ist
  weniger zuverlässig, und da der Korridor der Bot-Breite entspricht, gäbe es
  ohnehin kein "sanftes" Teil-Ausweichen – nur dieselbe Reaktion wie mittel,
  bloß früher auf unsichereren Daten ausgelöst.
- **mittel → EVADE:** normales Ausweichen, PID-Offset.
- **nah → EMERGENCY:** Notfall, umgeht die PID komplett (Vorbild: andere
  Gruppe, deren nächste Zone löst ebenfalls einen Sofort-Nothalt aus).

```
              Zone nah          ┌──────────────────────────────────────────┐
        ┌───────────────────►┌─────────────┐                              │
        │                    │  EMERGENCY  │──NAH-Zone frei (free_stable)─┐│
┌───────┴───┐  Zone mittel    │ v=Wiggle    │                             ││
│   IDLE    │───────────────► │ ω=fest, PID │◄─── nah-Zone belegt ────┐   ││
│ offset=0  │                 │  umgangen   │      (aus PASS/RETURN)  │   ││
└───────────┘                 └─────────────┘                        │   ▼▼
      ▲                                                               │ ┌──────┐
      │ fertig (Kamera/                                               │ │ PASS │
      │ Encoder/Timeout)                                              │ │offset│
      │                                                               │ │=±X   │
┌───────────┐  nachlauf_secs abgelaufen                                │ └──┬───┘
│  RETURN   │◄───────────────────────────────────────────────────────┘    │
│ offset=0  │                                            Zone mittel      │
│ ret_ω≠0   │◄──────────── zurück zu EVADE ───────────────wieder belegt───┤
└───────────┘                                                             │
      ▲                                                                   │
      │ evade_timeout_secs         Zone leer (free_stable)                │
      │                    ┌──────────────────────────────────────────────┘
┌───────────┐  Timeout    ▼
│   EVADE   │────────────►┌──────────┐
│ offset=±X │             │   WAIT   │
│Ticks akkum│◄────────────│  v=0     │
└───────────┘ Zone leer/   │stop=True │
              wait_timeout └──────────┘
```

(Kurz zusammengefasst: IDLE verzweigt je nach Zone zu EMERGENCY oder EVADE;
beide führen über PASS zu RETURN zurück zu IDLE; EVADE kann bei Timeout über
WAIT ebenfalls in PASS münden; aus PASS/RETURN heraus kann ein neues
Hindernis wieder zurück in EMERGENCY oder EVADE springen – Details in der
Zustandsbeschreibung unten.)

### Was in jedem Zustand passiert:

#### IDLE – Normalbetrieb
- `error_offset = 0` → kein Eingriff, Bot folgt der weißen Linie normal
- Encoder-Akkumulator wird auf 0 zurückgesetzt
- **Übergang → EMERGENCY:** Zone **nah** ist belegt
- **Übergang → EVADE:** Zone **mittel** ist belegt (nah hat Vorrang, falls beides)

#### EMERGENCY – Notfall (nah-Zone)
- PID wird komplett umgangen: `control_lane_node` übernimmt `emergency_cmd`
  (v+omega) 1:1, solange `emergency_active=True`
- `omega = ±emergency_omega_rad` (feste Drehrate, Vorzeichen von der
  Richtungsberechnung wie bei EVADE, siehe unten – nur das Vorzeichen zählt,
  nicht die Stärke)
- `v` = Wiggle: kippt alle `wiggle_interval_secs` das Vorzeichen
  (`wiggle_power`) – schnelles Vor-Zurück-Wackeln, überwindet die
  Standreibung der Räder beim Drehen auf der Stelle
- Encoder-Ticks werden akkumuliert (für spätere Rückkehr)
- **Übergang → PASS:** NAH-Zone `free_stable_frames` Frames **hintereinander** leer
- **Übergang → WAIT:** `emergency_timeout_secs` überschritten (Failsafe, falls
  die nah-Zone nie stabil frei wird, z.B. dauerhafte Fehlerkennung)

#### EVADE – Ausweichen (mittel-Zone)
- `error_offset = ±locked_offset` → Bot weicht zur freien Seite aus
- Die Ausweichrichtung wird beim Eintritt **einmalig eingefroren** (auch wenn die Ente sich danach bewegt)
- Encoder-Ticks werden akkumuliert (für spätere Rückkehr)
- **Übergang → PASS:** Zonen `free_stable_frames` Frames **hintereinander** leer
  (Standard 5 ≈ 0,5s bei 10Hz – ein einzelner flackernder Frame reicht nicht,
  verhindert vorzeitiges Verlassen bei kurzzeitig unstabiler Farberkennung)
- **Übergang → WAIT:** `evade_timeout_secs` überschritten

#### WAIT – Anhalten (Stufe 6)
- `obstacle/stop = True` → `control_lane_node` setzt v=0, omega=0
- Bot wartet bis Weg frei oder Timeout abgelaufen
- Offset während WAIT = 0 (kein Ausweichen, Bot steht still)
- **Übergang → PASS:** Zonen `free_stable_frames` Frames hintereinander leer
  **oder** `wait_timeout_secs` abgelaufen (harter Notausgang, ungebremst)

#### PASS – Nachlauf
- `error_offset = locked_offset` (Offset bleibt aktiv, damit Bot sicher vorbei fährt)
- Encoder-Ticks akkumulieren weiter
- **Übergang → EMERGENCY:** nah-Zone belegt (Vorrang)
- **Übergang → EVADE:** mittel-Zone belegt
- **Übergang → RETURN:** `nachlauf_secs` abgelaufen

#### RETURN – Encoder+Kamera-Rückkehr (Stufe 5)
- `error_offset = 0` (kein künstlicher Spurversatz mehr)
- `return_omega ≠ 0` publiziert → Bot dreht in **entgegengesetzter** Richtung zu EVADE
- `return_ticks_remaining` wird mit jedem Encoder-Delta dekrementiert
- **Übergang → IDLE:** Kamera sieht weiße Linie (primär) **oder** Ticks
  aufgebraucht (Backup) **oder** `return_timeout_secs` überschritten
  (Failsafe, falls weder Kamera noch Encoder je "fertig" melden – verhindert
  endloses Drehen)
- **Übergang → EMERGENCY:** nah-Zone belegt (Vorrang)
- **Übergang → EVADE:** mittel-Zone belegt (neue Ente erkannt)

### Ausweichrichtung – wie wird sie bestimmt?

Primär aus dem **Korridor-Lückenabstand** (`/detect/corridor_occupancy`,
2 Werte `[links_frei, rechts_frei]` als exakter Pixel-Anteil der Korridorbreite,
nah+mittel-Band, rechter Rand zusätzlich durch die live erkannte weiße Linie
begrenzt via `white_line_margin_px`). Der Korridor
entspricht der Bot-Breite – ein Hindernis irgendwo darin heißt grundsätzlich
"so nicht durchfahrbar", außer die freie Seite ist (fast) so breit wie der
ganze Korridor. `_find_best_gap()` sucht deshalb NICHT die breiteste Lücke
zwischen zwei Hindernissen, sondern vergleicht den freien Abstand vom linken
bzw. rechten Korridorrand bis zum nächstgelegenen Hindernis und wählt die
Seite mit mehr Abstand (kein Durchquetschen zwischen zwei Hindernissen – der
Bot fährt immer außen an allen Hindernissen einer Seite vorbei).
`_offset_from_gap()` berechnet daraus Richtung und Stärke:

```
(center_frac, width_frac) = Mitte und Breite der freien Seite, je als Anteil [0,1] der Korridorbreite
center_signed = (center_frac - 0.5) * 2                      (-1 links .. +1 rechts, gibt die Richtung)
offset = evade_offset_min + (1 - width_frac) * (evade_offset - evade_offset_min)
         (Vorzeichen von center_signed → links = negativ, rechts = positiv)
```

- Freie Seite fast so breit wie der ganze Korridor (`width_frac` → 1) → Offset
  nahe `evade_offset_min` (kleiner Ausschlag reicht).
- Nur ein schmaler Rest frei (`width_frac` → 0) → Offset nahe `evade_offset`
  (voller Ausschlag, der Bot braucht seine ganze Breite zum Vorbeikommen).
- **Fallback** (kein Profil verfügbar, oder Korridor komplett belegt – kein Bin
  frei): alte `duck_x`-Heuristik –

```
duck_x ∈ [0, +1]  →  Ente rechts von BEV-Mitte  →  offset negativ  →  nach links ausweichen
duck_x ∈ [-1, 0)  →  Ente links  von BEV-Mitte  →  offset positiv  →  nach rechts ausweichen
duck_x = -99      →  kein Blob (z.B. gelbe Linie) →  offset positiv  →  rechts als sicherer Standard
```

Die Richtung/Stärke wird nur beim **Eintritt** in EVADE (oder beim Neustart aus
RETURN heraus) bestimmt und für die Dauer des Manövers eingefroren – kein
Nachjustieren während EVADE/PASS, um Oszillation zu vermeiden (siehe Abschnitt 5).

---

## 6. Die Zonen im Bird's-Eye-View

Im BEV-Bild ist y=0 oben (was weit vor dem Bot liegt) und y=400 unten (direkt vor dem Bot). Der Bot selbst ist am unteren Bildrand.

```
x=0                                                    x=400
 │                 BEV-Bild (400 × 400 px)               │
 │             ← Fahrtrichtung / weit vorne →            │
y=0 ──────────────────────────────────────────────────── y=0
 │   ░░░░░░░░░░░░░░░  FERN  ░░░░░░░░░░░░░░░░░░░░░░░░░   │  y ≈ 20%–45%  (nur gemessen,
 │                                                       │               löst kein Ausweichen aus)
 │              ████ MITTEL ████                         │  y ≈ 45%–70%  → EVADE auslösen
 │              ████  NAH   ████                         │  y ≈ 70%–95%  → EVADE auslösen
 │                                                       │
y=400 ─────────────────────────────────────────────── y=400
                    ↑         ↑
              x0 = Bildmitte    x1 = Bildmitte
                  - width/2          + width/2
              ←   Korridor: entspricht der Bot-   →
                  Breite, NICHT die ganze Spur!
```

Der Korridor ist **schmal** und **fest an der Bildmitte** verankert (`x = W/2`
im BEV-Bild), `zones.corridor_width_px` die Gesamtbreite symmetrisch drumherum.
Bewusst unabhängig von der weißen Linie/`last_white_position`: würde der
Korridor daran hängen, bliebe er bei kurzzeitig verlorener Linienerkennung an
der zuletzt bekannten (ggf. veralteten) Position stehen. Die Bildmitte trifft
die tatsächliche Spur in der Praxis gut genug (siehe reprojizierte
Enten-Position in Abschnitt zur Enten-Erkennung).
Würde der Korridor stattdessen die ganze Spur abdecken (feste Bildanteile
`corridor_x_min/max`), löst **jede** Ente irgendwo in der Spur aus – auch wenn
sie objektiv nicht im Fahrweg des Bots steht – und schlimmer: Die gewählte
Ausweichrichtung (siehe Abschnitt 5) könnte jenseits der gelben Linie in der
Gegenspur liegen, der Bot würde also genau dorthin lenken.

**Wie eine Zone als belegt gilt:**
- Keine eigene Farberkennung und keine Maske/Flächen-Threshold hier – rein
  geometrische Prüfung auf den bereits im Originalbild erkannten und ins BEV
  reprojizierten Enten-Bodenkontaktpunkten (`bx_left`, `bx_right`, `by`; siehe
  Abschnitt zur Enten-Erkennung):
  ```
  belegt = (bx_right >= x0 UND bx_left <= x1)   # X-Überlappung mit Korridor
           UND by <= y1_zone                     # Ente auf/vor dieser Zonentiefe
  ```
- Kein Schwellwert nötig: eine schmale Ente in einer breiten Zone wird genauso
  zuverlässig erkannt wie eine breite (die frühere Flächen-Prozent-Prüfung
  konnte das bei schmalen Enten knapp verfehlen)

**Erkennt gezielt Gelb/Grün:** gelbe Enten, grüne Bonus-Enten, gelbe Mittellinie –
unbunte Reflexionen/Klebereste auf der Fahrbahn fallen automatisch raus, da sie
außerhalb der Farbbereiche liegen (mit der früheren reinen Helligkeitsschwelle
nicht unterscheidbar).

**FERN-Zone:** wird erkannt und publiziert, löst aber derzeit **kein Ausweichen** aus. Sie könnte in Zukunft zur Geschwindigkeitsreduktion ("Frühwarnung") genutzt werden.

---

## 7. Encoder-Rückkehr – wie funktioniert das genau?

### Das Problem mit Encodern beim Duckiebot

Der Encoder liefert eine **kumulative Zählzahl** (`data`), die immer aufwärts zählt – egal ob vorwärts oder rückwärts gefahren wird. Richtungserkennung aus den Encoder-Daten allein ist **nicht möglich**.

**Lösung:** Die Rückkehrrichtung wird aus dem Vorzeichen von `locked_offset` abgeleitet:
- Bot hat nach links ausgewichen (`locked_offset < 0`) → Rückkehr nach rechts → `return_omega > 0`
- Bot hat nach rechts ausgewichen (`locked_offset > 0`) → Rückkehr nach links → `return_omega < 0`

### Ablauf der Tick-Akkumulation

```
EVADE + PASS:
  Encoder-Callbacks → pending_ticks_left  += Δlinks
                      pending_ticks_right += Δrechts
  _step_state_machine() → delta = (pending_links + pending_rechts) / 2
                           accumulated_ticks += delta

RETURN (Start):
  return_ticks_remaining = accumulated_ticks   ← gespiegelter Betrag

RETURN (jeder Step):
  return_ticks_remaining -= delta_ticks

Abbruch wenn:
  a) |lane_error| < return_threshold  für  return_stable_frames  aufeinanderfolgende Frames
     → PRIMÄRE Bedingung: Kamera sieht weiße Linie wieder
  b) return_ticks_remaining ≤ 0
     → BACKUP: Encoder-Strecke aufgebraucht
```

Die Encoder-Genauigkeit ist dabei **unkritisch**: Schlupf verlängert die Rückkehr höchstens minimal. Die Kamera als primäre Abbruchbedingung kompensiert alle Ungenauigkeiten.

---

## 8. Parameter-Übersicht – was kann ich wo einstellen?

Alle Parameter sind in den JSON-Dateien unter `config/` und können **live** über den `configuration_node` angepasst werden (kein Neustart nötig).

### `config/detect_lane_node.json`

| Gruppe | Parameter | Standard | Beschreibung |
|--------|-----------|----------|-------------|
| `white_follow` | `offset_px` | 150 px | Sollabstand zur weißen Linie (BEV-Pixel) |
| `white` | `vl`, `vh` | 161, 255 | Helligkeitsbereich für weiße Linie (HSV value) |
| `white` | `sl`, `sh` | 0, 41 | Sättigungsbereich für weiße Linie |
| `duck` | `roi_top/bottom` | 0.35 / 1.0 | Vertikaler Auswertebereich – bezogen auf die **Originalbild**-Höhe |
| `duck` | `min_area`, `min_w`, `min_h` | 250, 12, 12 | Mindestgröße eines Enten-Blobs – **Originalbild**-Pixel |
| `duck` | `kf_process_var` | 0.01 | Kalman-Filter: erwartetes "Rauschen" der Positions-Änderung |
| `duck` | `kf_measurement_var` | 0.05 | Kalman-Filter: erwartetes Messrauschen der Roh-Erkennung |
| `duck` | `kf_max_missed_frames` | 5 | Max. Frames ohne Erkennung, bevor auf "keine Ente" (-99) zurückgefallen wird |
| `obstacle_color` | `yellow_hl/hh/sl/sh/vl/vh` | 20/35/80/255/80/255 | HSV-Bereich für Gelb (Enten + Mittellinie) |
| `obstacle_color` | `green_hl/hh/sl/sh/vl/vh` | 40/85/60/255/40/255 | HSV-Bereich für Grün (Bonus-Enten) |
| `zones` | `corridor_width_px` | 300 px | Breite des Fahrkorridors, **symmetrisch um die BEV-Bildmitte fixiert** – entspricht der Bot-Breite, NICHT die ganze Spur |
| `zones` | `far_y_min/max` | 0.20 / 0.45 | FERN-Zone (oben im BEV) |
| `zones` | `mid_y_min/max` | 0.45 / 0.70 | MITTEL-Zone |
| `zones` | `near_y_min/max` | 0.70 / 0.95 | NAH-Zone (direkt vor Bot) |
| `zones` | `white_line_margin_px` | 20 px | Sicherheitsabstand: rechter Korridorrand für die Lücken-Suche zusätzlich durch die live erkannte weiße Linie begrenzt |

### `config/control_obstacle_node.json`

| Parameter | Standard | Beschreibung |
|-----------|----------|-------------|
| `active` | 1 | Gesamte Ausweichlogik ein (1) / aus (0) |
| `evade_offset` | 0.6 | Maximale Stärke des Ausweichens (nur schmaler Rest des Korridors frei); wird zum PID-Fehler addiert |
| `evade_offset_min` | 0.25 | Minimale Stärke des Ausweichens (fast der ganze Korridor frei); verhindert zu schwaches Ausweichen |
| `nachlauf_secs` | 1.5 s | Zeit im PASS-Zustand nach letztem Objekt-Kontakt |
| `evade_timeout_secs` | 5.0 s | Max. Zeit im EVADE bevor WAIT ausgelöst wird |
| `free_stable_frames` | 5 | Frames die der Korridor hintereinander frei sein muss, bevor EVADE/WAIT tatsächlich verlassen wird (Entprellung gegen Flackern) |
| `return_threshold` | 0.25 | Spurversatz-Schwelle für Rückkehr-Abbruch |
| `return_stable_frames` | 5 | Frames die Versatz < threshold sein muss (Entprellung) |
| `return_omega` | 0.5 rad/s | Drehrate bei der Encoder-Rückkehr |
| `wait_timeout_secs` | 3.0 s | Zeit im WAIT bis erzwungenes PASS |
| `return_timeout_secs` | 5.0 s | Hartes Zeitlimit für RETURN – Failsafe gegen endloses Drehen, falls weder Kamera noch Encoder je "fertig" melden |
| `emergency_omega_rad` | 1.6 rad/s | Feste Drehrate im NOTFALL (nah-Zone), umgeht die PID |
| `emergency_timeout_secs` | 5.0 s | Hartes Zeitlimit für NOTFALL, falls die nah-Zone nie stabil frei wird (Failsafe → WAIT) |
| `wiggle_interval_secs` | 0.06 s | Wie oft `v` im NOTFALL das Vorzeichen wechselt (Wiggle gegen Standreibung beim Drehen auf der Stelle) |
| `wiggle_power` | 0.07 | Stärke des Wiggle-Ausschlags |

### `config/control_lane_node.json`

| Parameter | Beschreibung |
|-----------|-------------|
| `pid.p / i / d` | PID-Faktoren für Spurfolgen |
| `pid.max_vel / min_vel` | Geschwindigkeitsgrenzen (m/s) |
| `stop_line.stop_duration` | Standzeit an roter Linie (s) |
| `stop_line.cooldown_duration` | Wartezeit bis nächste rote Linie auslöst (s) |

---

## 9. ROS-Topic-Übersicht (vollständig)

> Bot-Name = `tick` → alle Topics beginnen mit `/tick/`

```
detect_lane_node
    subscribe: /tick/camera_node/image/compressed  ← Rohbild
               /tick/obstacle/state                ← Zustand (nur fürs Debug-Overlay)
    publish:   /tick/detect/lane            Float64           Spurversatz [-1,+1]
               /tick/detect/stop_line       Bool              Rote Linie sichtbar
               /tick/detect/duck            Float64           Kalman-gefilterte Enten-x ([-1,+1]; -99 = kein Blob)
               /tick/detect/zones           Float32MultiArray [nah, mittel, fern] ∈ {0,1}
               /tick/detect/corridor_occupancy Float32MultiArray [links_frei, rechts_frei] (exakter Anteil, für Ausweich-Offset)
               /tick/debug/original         CompressedImage   Rohbild
               /tick/debug/annotated        CompressedImage   Bild mit Linien eingezeichnet
               /tick/debug/bird_view        CompressedImage   BEV ohne Annotation
               /tick/debug/lane_white       CompressedImage   Weiß-Maske
               /tick/debug/lane_red         CompressedImage   Rot-Maske
               /tick/debug/duck_bev         CompressedImage   BEV mit Zonen + projizierten Positionen + Zustand
               /tick/debug/duck_original    CompressedImage   Originalbild mit erkannten Enten-Boxen

control_lane_node
    subscribe: /tick/detect/lane              ← PID-Eingang
               /tick/detect/stop_line         ← Haltelinien-Automat
               /tick/enable/lane              ← Ein/Aus
               /tick/obstacle/error_offset    ← Ausweich-Offset
               /tick/obstacle/return_omega    ← Encoder-Rückkehr-omega
               /tick/obstacle/stop            ← Vollstopp-Signal
    publish:   /tick/car_cmd_switch_node/cmd  Twist2DStamped  Fahrbefehl (v, omega)

control_obstacle_node
    subscribe: /tick/detect/zones                        ← Auslöser
               /tick/detect/duck                         ← Richtungs-Fallback
               /tick/detect/corridor_occupancy           ← Richtung + Stärke (primär)
               /tick/detect/lane                         ← Abbruchbedingung RETURN
               /tick/enable/obstacle                     ← Ein/Aus
               /tick/left_wheel_encoder_node/tick        WheelEncoderStamped
               /tick/right_wheel_encoder_node/tick       WheelEncoderStamped
    publish:   /tick/obstacle/error_offset    Float64   Ausweich-Offset (0 = inaktiv)
               /tick/obstacle/return_omega    Float64   Rückkehr-omega (0 = inaktiv)
               /tick/obstacle/stop            Bool      True nur in WAIT
               /tick/obstacle/done            Bool      True wenn Manöver fertig
               /tick/obstacle/state           String    Zustand als Klartext (Debug-Overlay)

switch_control_node
    subscribe: /tick/detect/zones     ← Lane→Obstacle wenn nah/mittel belegt
               /tick/obstacle/done    ← Obstacle→Lane wenn fertig
    publish:   /tick/enable/lane      Bool  (immer True)
               /tick/enable/obstacle  Bool  (True wenn Zonen nah/mittel aktiv)
```

---

## 10. Debug-Möglichkeiten

### Debug-Bilder live ansehen

```bash
# Originalbild mit erkannten Enten-Boxen (vor der BEV-Transformation):
rosrun image_view image_view image:=/tick/debug/duck_original

# BEV mit eingezeichneten Zonen, projizierten Positionen und Zustands-Overlay:
rosrun image_view image_view image:=/tick/debug/duck_bev

# Bild mit erkannter weißer und roter Linie:
rosrun image_view image_view image:=/tick/debug/annotated

# Nur Weiß-Maske (zum Kalibrieren der HSV-Werte):
rosrun image_view image_view image:=/tick/debug/lane_white
```

Alternativ startet `detect_lane_node.py` standalone auch lokale `cv2.imshow`-
Fenster (`duck_bev`, `annotated`, `duck_original`) — kein separates
`rqt_image_view`/`image_view` nötig, siehe `run_debug()`.

### Zustand des Ausweich-Automaten verfolgen

```bash
# Aktueller Spurversatz (0 = Mitte, ±1 = Rand):
rostopic echo /tick/detect/lane

# Zonen-Belegung ([nah, mittel, fern], 0=frei, 1=belegt):
rostopic echo /tick/detect/zones

# Ausweich-Offset (0 = Normalbetrieb, ≠0 = Ausweichen aktiv):
rostopic echo /tick/obstacle/error_offset

# Ob Bot aktuell angehalten ist (WAIT-Zustand):
rostopic echo /tick/obstacle/stop

# Zustand als Klartext (Idle/Evade/Wait/Pass/Return):
rostopic echo /tick/obstacle/state
```

### Log-Meldungen im Terminal lesen

`control_obstacle_node` loggt jeden Zustandswechsel. Typischer Ablauf:

```
[Evade] Auslösung – rechts, Offset +0.60
[Evade] Korridor frei → PASS
[Evade] Nachlauf vorbei → RETURN (akkum. 87 Ticks)
[Evade] Rückkehr fertig (Kamera) → IDLE

# Wenn Bot zu lange ausweicht:
[Evade] Timeout 5.1s → WAIT
[Evade] WAIT: Korridor frei → PASS

# Alle 2s Status-Log (throttled):
[Evade] Evade  off=+0.60  ret_ω=+0.00  acc=42  rem=0  zones=[0, 1, 0]
```

---

## 11. Häufige Probleme & Lösungen

| Problem | Ursache | Lösung |
|---------|---------|--------|
| Bot weicht aus obwohl keine Ente da | Korridor zu breit, oder Fehlerkennung im Originalbild | `corridor_width_px` verkleinern (entspricht Bot-Breite, nicht die ganze Spur); `/tick/debug/duck_original` prüfen, ob dort fälschlich etwas erkannt wird |
| Bot fährt beim Ausweichen über die weiße/gelbe Linie | Korridor reicht über die eigene Spur hinaus | `corridor_width_px` verkleinern (Korridor ist symmetrisch um die Bildmitte); `white_line_margin_px` erhöhen, damit der rechte Rand der Lücken-Suche mehr Abstand zur weißen Linie hält |
| EVADE startet und endet sofort wieder (flackert) | Farberkennung liefert einzelne unstabile Frames, kein Entprellen | `free_stable_frames` erhöhen (Standard 5) |
| Ente wird nicht erkannt | `obstacle_color`-Bereiche zu eng, oder Mindestgrößen (`duck.min_area/min_w/min_h`) zu hoch | Senken/erweitern; `/tick/debug/duck_original` ansehen: wird die Box im Originalbild überhaupt erkannt? |
| Ente "verschwindet" schnell, obwohl noch sichtbar | (Altes Problem der reinen BEV-Erkennung, durch Originalbild-Erkennung + Homographie-Projektion behoben) | Falls trotzdem noch auffällig: `duck.kf_max_missed_frames` erhöhen |
| Rückkehr endet zu früh | `return_threshold` zu hoch oder `return_stable_frames` zu niedrig | Threshold senken (z.B. 0.15); Frames erhöhen (z.B. 8) |
| Rückkehr dreht zu weit | `return_omega` zu hoch | Senken (z.B. 0.3 rad/s) |
| Bot hält dauerhaft an (WAIT) | Falsch-Positiv-Erkennung | `wait_timeout_secs` kürzer setzen; `/tick/debug/duck_original` auf Fehlerkennungen prüfen |
| Nachlauf zu lang/kurz | `nachlauf_secs` nicht zur Geschwindigkeit passend | Anpassen nach tatsächlicher Fahrgeschwindigkeit |
| Weiße Linie nicht gefunden | HSV-Parameter `white.vl/vh` falsch kalibriert | Via configuration_node live anpassen; `/tick/debug/lane_white` ansehen |
| Bot folgt Linie mit falschem Abstand | `white_follow.offset_px` falsch | Anpassen (150 px = Standard; größer = näher an weiße Linie) |

---

## 12. Schnellübersicht: Dateien und ihre Kernaufgabe

```
detect_lane_node.py       →  Kamera auswerten, alle Signale publizieren
control_lane_node.py      →  PID rechnen, Fahrbefehl senden (EINZIGE Stelle!)
control_obstacle_node.py  →  Zustandsautomat, Ausweich-Offset + Encoder-Rückkehr
switch_control_node.py    →  enable/lane (immer) + enable/obstacle schalten
util.py                   →  JSON-Parameter laden (nicht direkt anfassen)
config/*.json             →  Alle einstellbaren Werte, live über GUI änderbar
```
