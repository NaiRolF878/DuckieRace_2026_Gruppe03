# Duckiebot – Challenge 1: Lane Following

> ROS 1 (Noetic) · Ubuntu 20.04 · Python 3 · OpenCV

Dieses Paket implementiert das autonome Spurfolgen für den Duckiebot. Der Bot erkennt die gelbe (links) und weiße (rechts) Spurlinie, folgt der Spur per PID-Regler und hält an roten Haltelinien für 3 Sekunden an.

---

## Inhaltsverzeichnis

- [Systemüberblick](#systemüberblick)
- [Warum diese Architektur?](#warum-diese-architektur)
- [Nodes](#nodes)
  - [detect\_lane\_node](#detect_lane_node)
  - [control\_lane\_node](#control_lane_node)
  - [switch\_control\_node](#switch_control_node)
  - [configuration\_node](#configuration_node)
  - [util.py](#utilpy)
- [Konfigurationsparameter](#konfigurationsparameter)
- [Setup & Starten](#setup--starten)
- [Kalibrierung](#kalibrierung)
- [Bekannte Probleme & Lösungen](#bekannte-probleme--lösungen)

---

## Systemüberblick

```
Kamera (/camera_node/image/compressed)
    │
    ▼
┌─────────────────────┐        ┌───────────────────────┐
│  detect_lane_node   │        │  configuration_node   │
│                     │        │  (Tkinter GUI)        │
│  Bird's-Eye-View    │        │                       │
│  CLAHE              │        │  Liest *.json aus     │
│  HSV-Masken         │        │  /config/ und baut    │
│  Morphologie        │        │  Schieberegler        │
│  Spatial Filter     │        └───────────┬───────────┘
│  Frame-Tracking     │                    │ /update_parameters
│  Rote Linie         │◀───────────────────┘
└──────────┬──────────┘
           │ /detect/lane      (Float64) Spurversatz [-1, +1]
           │ /detect/stop_line (Bool)    Rote Linie erkannt
           ▼
┌─────────────────────┐        ┌───────────────────────┐
│  control_lane_node  │◀───────│  switch_control_node  │
│                     │        │                       │
│  PID-Regler         │        │  Entscheidet ob Lane  │
│  StopState-Automat  │        │  oder Obstacle aktiv  │
└──────────┬──────────┘        └───────────────────────┘
           │ /car_cmd_switch_node/cmd (Twist2DStamped)
           ▼
        Motoren
```

---

## Warum diese Architektur?

### Warum separate Nodes statt einem Skript?

ROS-Nodes laufen als eigenständige Prozesse und kommunizieren über Topics. Das hat für uns zwei konkrete Vorteile:

1. **Einzelne Nodes können neu gestartet werden** ohne das gesamte System zu stoppen – beim Debuggen auf dem echten Bot sehr nützlich.
2. **Der `configuration_node` kann auf dem Notebook laufen** während die anderen Nodes auf dem Bot laufen. Da wir Docker (noch) nicht verwenden, ist das unsere Lösung für die räumliche Trennung von Bot und Entwicklungsrechner.

### Warum JSON-Konfiguration statt `rosparam`?

Alle kalibrierbaren Parameter liegen in `.json`-Dateien unter `config/`. Der `configuration_node` liest diese Dateien automatisch und baut die GUI daraus auf. Das bedeutet:

- **Neue Parameter erscheinen automatisch als Schieberegler** – kein Code im `configuration_node` muss angefasst werden.
- **Parameter können live geändert werden** ohne irgendetwas neu zu starten.
- Das war wichtig weil wir die HSV-Werte und PID-Parameter auf der echten Strecke kalibrieren mussten.

### Warum `StopState` als Enum und nicht als String?

Der erste Entwurf nutzte String-Vergleiche (`if self.stop_state == 'driving'`). Das Problem: ein Tippfehler wie `'Driving'` führt zu einem **stillen Bug** – keine Fehlermeldung, der Bot verhält sich einfach falsch. Die Enum-Lösung schlägt bei falschen Werten sofort mit einem `AttributeError` an. Das Pattern haben wir von `ControlType` in `switch_control_node.py` übernommen wo es bereits so gemacht wurde.

---

## Nodes

### detect\_lane\_node

**Aufgabe:** Kamerabild Frame für Frame verarbeiten und Spurversatz sowie rote Haltelinie erkennen.

#### Warum Bird's-Eye-View?

Das Kamerabild ist perspektivisch verzerrt – die Spurlinien laufen im Bild zusammen wie Eisenbahnschienen. Im Bird's-Eye-View (Vogelperspektive) sind beide Linien **parallel und gleich breit**, unabhängig von der Entfernung. Das macht die Breitenberechnung und damit den Spurversatz deutlich stabiler.

Die Transformation wird über vier kalibrierbare Eckpunkte gesteuert, die das Trapez der Fahrspur im Originalbild definieren.

#### Warum CLAHE?

Auf der realen Strecke gibt es Lichtwechsel und Schatten die die Helligkeit lokal stark verändern. Weiß im Schatten sieht im HSV-Bild nicht mehr wie Weiß aus – die Maske schlägt fehl.

CLAHE (**C**ontrast **L**imited **A**daptive **H**istogram **E**qualization) gleicht die Helligkeit **lokal** aus: das Bild wird in 8×8 Kacheln geteilt und jede Kachel wird unabhängig angeglichen. Ein Schatten auf einer Seite beeinflusst die andere Seite nicht.

**Warum LAB statt direkt HSV?** Im LAB-Farbraum ist der Helligkeitskanal `L` vollständig von den Farbkanälen `A` und `B` getrennt. CLAHE wird nur auf `L` angewendet – die Farbe bleibt unverändert. In HSV sind Helligkeit und Farbe stärker gekoppelt, was die mühsam kalibrierte HSV-Maske verschieben würde.

#### Warum Morphologie (MORPH_CLOSE)?

Schatten erzeugen kleine Lücken in den Farbmasken – die Linie sieht im Binärbild unterbrochen aus. `MORPH_CLOSE` (erst Dilatation, dann Erosion) schließt diese Lücken ohne die Linienform wesentlich zu verändern.

#### Warum zwei HSV-Masken für Rot?

Rot liegt im HSV-Farbraum an **zwei Stellen** des Hue-Kreises (0–360°):
- Hue **0–10**: orangerotes Rot
- Hue **160–179**: blaurotes Rot

Eine einzelne Maske würde entweder orangerote oder blaurote Haltelinien verpassen. Wir kombinieren beide Masken mit `bitwise_or`.

#### Warum Spatial Filter für die weiße Linie?

Auf engen 180°-Kurven sieht der Bird's-Eye-View so aus:

```
|gelb| eigene Spur |weiß|schmal|weiß| Gegenspur |gelb|
                    ↑ richtig   ↑ falsch
```

Der Code suchte bisher die linkeste Kante der weißen Maske – das war bei schmalem Abstand oft die **falsche** (äußere) weiße Linie der Gegenspur.

Die Lösung: die Weiß-Maske wird **vor der Suche** eingeschränkt. Alles links von `center_yellow + min_lane_width` wird auf 0 gesetzt. Die eigene weiße Linie liegt immer rechts davon.

#### Warum Frame-Tracking für die weiße Linie?

Der Spatial Filter alleine löst strukturelle Gegenspurprobleme. Einzelne Ausreißer durch Lichtreflexe oder kurze Fehlmessungen können aber trotzdem vorkommen. Das Frame-Tracking prüft ob die neue Position plausibel ist: liegt der Sprung zur letzten Position über `max_frame_jump` Pixel, wird der letzte bekannte Wert beibehalten statt die Fehlmessung zu übernehmen.

#### Warum ROI für die rote Haltelinie?

Zwei separate Probleme führten zur zweidimensionalen ROI:

**Vertikal (`detection_zone`):** Wir wollten nicht dass der Bot 2 Meter vor der Linie stoppt. Mit `detection_zone = 0.85` wird nur der unterste 15%-Streifen des Bildes geprüft – der Bot hält erst an wenn die Linie wirklich direkt vor ihm liegt.

**Horizontal (`detection_x_start`):** Die Haltelinie der Gegenspur (auf der linken Bildseite) wurde erkannt und löste einen ungewollten Stopp aus. Mit `detection_x_start = 0.4` werden nur die rechten 60% des Bildes geprüft.

#### Topics

| Topic | Typ | Beschreibung |
|---|---|---|
| `/detect/lane` | `Float64` | Spurversatz: `0.0` = mittig, `+1.0` = ganz links, `-1.0` = ganz rechts |
| `/detect/stop_line` | `Bool` | `True` wenn rote Haltelinie im ROI erkannt |
| `/debug/lane_croped` | `CompressedImage` | Bird's-Eye-View mit Hilfslinien (Kalibrierung) |
| `/debug/lane_white` | `CompressedImage` | Binärmaske weiße Linie |
| `/debug/lane_yellow` | `CompressedImage` | Binärmaske gelbe Linie |
| `/debug/lane_red` | `CompressedImage` | Binärmaske rote Linie |

---

### control\_lane\_node

**Aufgabe:** Spurversatz per PID-Regler in Fahrbefehle umrechnen und Haltelinien-Logik verwalten.

#### Warum PID-Regler?

Ein einfacher P-Regler (`omega = kp * error`) reagiert auf den aktuellen Fehler, schwingt aber in Kurven über. Der PID-Regler ergänzt zwei weitere Anteile:

- **I-Anteil:** summiert den Fehler über die Zeit auf. Gleicht systematische Abweichungen aus – zum Beispiel wenn die Kamera leicht schräg montiert ist und der Bot dauerhaft etwas zu weit links fährt.
- **D-Anteil:** reagiert auf die **Änderungsrate** des Fehlers. Merkt wenn der Bot sich zu schnell zur Mitte bewegt und dämpft die Bewegung ab bevor er überschwingt.

#### Warum `v = MAX_VEL * (1 - |error|)`?

Bei großem Spurversatz soll der Bot **langsamer** werden – in engen Kurven ist schnelles Fahren bei großem Fehler gefährlich und macht die Regelung instabil. Diese Formel reduziert die Geschwindigkeit proportional zum Fehler: bei `error = 0` (perfekt mittig) fährt der Bot mit `MAX_VEL`, bei `error = 1` (maximaler Versatz) steht er.

#### Warum wird das Integral beim Losfahren zurückgesetzt?

Während der Bot 3 Sekunden an der roten Linie steht, akkumuliert der I-Anteil weiter Fehler. Beim Anfahren würde dieser aufgelaufene Wert sofort eine starke Lenkbewegung auslösen. Das Integral wird daher beim Übergang von `Stopping` → `Cooldown` auf 0 zurückgesetzt.

#### Warum Cooldown nach dem Stopp?

Direkt nach dem Anfahren ist die rote Linie noch kurz im Bild sichtbar. Ohne Cooldown würde der Bot sofort wieder stoppen. Der Cooldown-Zustand ignoriert für `cooldown_duration` Sekunden alle neuen roten Linien.

#### Warum `v = 0` in `cbFollowLane` statt nur im `run()`-Loop?

Der `run()`-Loop läuft mit 10 Hz und setzt `twist.v = 0` im `Stopping`-Zustand. Der PID-Callback `cbFollowLane` kann aber asynchron feuern und würde `self.v` trotzdem aktualisieren. Wir setzen deshalb auch in `cbFollowLane` direkt `self.v = 0` wenn der Zustand `Stopping` ist – doppelt abgesichert.

#### StopState-Zustandsautomat

```
Driving  ──(rote Linie erkannt)──▶  Stopping
                                        │
                                   v=0, omega=0
                                   Timer läuft
                                        │
                                   (3s vorbei)
                                        │
                                        ▼
Driving  ◀──(Cooldown abgelaufen)──  Cooldown
                                     normal fahren,
                                     Linie ignorieren
```

#### Topics

| Topic | Typ | Beschreibung |
|---|---|---|
| `/car_cmd_switch_node/cmd` | `Twist2DStamped` | `v` = Geschwindigkeit, `omega` = Lenkung |

---

### switch\_control\_node

**Aufgabe:** Entscheidet welcher Controller (Lane oder Obstacle) aktiv ist.

Publiziert kontinuierlich den aktiven Modus. Alle Controller-Nodes abonnieren diesen Topic und aktivieren oder deaktivieren sich selbst entsprechend.

**Warum so und nicht direktes Enable/Disable?**  
Mit diesem Muster kann später ein neuer Controller einfach hinzugefügt werden indem er den `switch/control`-Topic abonniert – ohne dass `switch_control_node` selbst geändert werden muss.

Die Callbacks `cbDuckieDetected` und `cbLaneDetected` sind noch leer und werden für **Challenge 3** (Hindernisse) implementiert.

---

### configuration\_node

**Aufgabe:** Live-Kalibrierung aller Parameter über eine Tkinter GUI.

**Warum datengetrieben?**  
Der `configuration_node` enthält keinen einzigen Parameter-Namen hart kodiert. Er liest alle `.json`-Dateien aus `config/` und baut die GUI automatisch daraus auf. Wenn wir einen neuen Parameter in die JSON eintragen, erscheint er beim nächsten Start automatisch als Schieberegler – ohne Änderung am `configuration_node`.

**Warum Tkinter?**  
Tkinter ist in Python vorinstalliert, kein zusätzliches Paket nötig. Für unseren Zweck (Schieberegler, Dropdowns) reicht es völlig aus.

---

### util.py

Gemeinsam genutzte Hilfsfunktionen für alle Nodes.

**`init_parameters(node_name, callback)`**  
Lädt beim Start die Parameter aus der zugehörigen JSON und registriert einen ROS-Subscriber für Live-Updates. So muss jede Node nur diese eine Funktion aufrufen statt den Lade- und Subscribe-Code zu duplizieren.

> ⚠️ **Bekannter Bug:** In `callback_wrapper` steht `callback_update_parameters(parameters)` außerhalb des `if msg['node'] == node_name`-Blocks. Das bedeutet jede Parameter-Message (egal für welche Node) löst den Callback in allen Nodes aus. Es funktioniert zufällig weil alle Nodes die gleiche Parameterstruktur erwarten – sollte aber korrigiert werden.

---

## Konfigurationsparameter

### detect\_lane\_node.json

#### `crop_image` – Perspektivtransformation

| Parameter | Default | Min | Max | Beschreibung |
|---|---|---|---|---|
| `top_left_x/y` | 159 / 218 | -100 | 1000 | Obere linke Ecke der Fahrspur im Kamerabild |
| `top_right_x/y` | 441 / 218 | -100 | 1000 | Obere rechte Ecke |
| `bottom_left_x/y` | 606 / 382 | -100 | 1000 | Untere linke Ecke |
| `bottom_right_x/y` | -29 / 382 | -100 | 1000 | Untere rechte Ecke |

#### `yellow` – Gelbe Linie (HSV)

| Parameter | Default | Beschreibung |
|---|---|---|
| `hl` / `hh` | 15 / 60 | Hue Unter- / Obergrenze |
| `sl` / `sh` | 60 / 255 | Saturation Unter- / Obergrenze |
| `vl` / `vh` | 120 / 255 | Value (Helligkeit) Unter- / Obergrenze |

#### `white` – Weiße Linie (HSV)

| Parameter | Default | Beschreibung |
|---|---|---|
| `hl` / `hh` | 0 / 255 | Hue Unter- / Obergrenze |
| `sl` / `sh` | 0 / 41 | Saturation Unter- / Obergrenze |
| `vl` / `vh` | 161 / 255 | Value Unter- / Obergrenze |
| `min_lane_width` | 50 px | Mindestabstand gelb→weiß – blendet Gegenspur in engen Kurven aus |
| `max_frame_jump` | 80 px | Max. Pixelsprung zwischen Frames – filtert Ausreißer |

#### `red` – Rote Haltelinie (HSV)

| Parameter | Default | Beschreibung |
|---|---|---|
| `hl` / `hh` | 0 / 10 | Hue unterer Rot-Bereich (Hue-Kreis Anfang) |
| `hl2` / `hh2` | 160 / 179 | Hue oberer Rot-Bereich (Hue-Kreis Ende) |
| `sl` / `sh` | 100 / 255 | Saturation Unter- / Obergrenze |
| `vl` / `vh` | 100 / 255 | Value Unter- / Obergrenze |
| `pixel_threshold` | 500 | Mindestanzahl roter Pixel für Linienerkennung |
| `detection_zone` | 0.85 | Vertikale ROI: unterste `15%` des Bildes prüfen |
| `detection_x_start` | 0.4 | Horizontale ROI: nur rechte `60%` prüfen |

### control\_lane\_node.json

#### `pid` – PID-Regler

| Parameter | Default | Beschreibung |
|---|---|---|
| `p` | 8.0 | Proportionalbeiwert |
| `i` | 0.0 | Integralbeiwert (0 = deaktiviert) |
| `d` | 6.0 | Differentialbeiwert |
| `max_vel` | 0.5 m/s | Maximalgeschwindigkeit bei `error = 0` |

#### `stop_line` – Haltelinien-Logik

| Parameter | Default | Beschreibung |
|---|---|---|
| `stop_duration` | 3.0 s | Wartezeit an der roten Linie |
| `cooldown_duration` | 3.0 s | Sperrzeit nach dem Stopp |

---

## Setup & Starten

### Voraussetzungen

```bash
# ROS-Umgebung laden
source /opt/ros/noetic/setup.bash

# Fahrzeugnamen setzen
export VEHICLE_NAME=duckiebot01

# Sicherstellen dass roscore läuft
roscore &
```

### Nodes starten

```bash
# Terminal 1 – Spurerkennung (auf dem Bot oder Notebook)
python3 src/detect_lane_node.py

# Terminal 2 – PID-Regler
python3 src/control_lane_node.py

# Terminal 3 – Steuerungsumschalter
python3 src/switch_control_node.py

# Terminal 4 – Kalibrierungs-GUI (empfohlen: auf dem Notebook)
python3 src/configuration_node.py
```

> **Hinweis:** Die Nodes starten auf dem Notebook und kommunizieren über das WLAN mit dem Bot. Dafür muss `ROS_MASTER_URI` auf den Bot zeigen:
> ```bash
> export ROS_MASTER_URI=http://<BOT_IP>:11311
> ```

---

## Kalibrierung

### Perspektivtransformation kalibrieren

1. `configuration_node` starten, Node `detect_lane_node`, Gruppe `crop_image`
2. Debug Image `/debug/lane_croped` auswählen
3. Eckpunkte so einstellen dass die Fahrspur im transformierten Bild als **Rechteck** erscheint und beide Linien **parallel** laufen

### HSV-Farbbereiche kalibrieren

1. Debug Image `/debug/lane_white` oder `/debug/lane_yellow` auswählen
2. `vl` hochschieben bis Hintergrund verschwindet, `vh` auf 255 lassen
3. `sl` und `sh` anpassen bis die Linie vollständig weiß ist
4. Für rote Linie: `/debug/lane_red`, beide Hue-Bereiche nacheinander einstellen

### PID kalibrieren

Empfohlene Reihenfolge:
1. `i = 0` lassen
2. `p` erhöhen bis der Bot der Spur folgt (aber noch schwingt)
3. `d` erhöhen bis das Schwingen aufhört
4. `i` nur bei dauerhaftem seitlichem Versatz leicht erhöhen

### Haltelinie kalibrieren

1. `pixel_threshold` erhöhen bis keine Fehlalarme auf gerader Strecke
2. `detection_zone` anpassen (Richtung 1.0 = Bot hält später/näher an)
3. `detection_x_start` erhöhen wenn Gegenspur-Haltelinie auslöst

---

## Bekannte Probleme & Lösungen

| Problem | Ursache | Lösung |
|---|---|---|
| Falsche weiße Linie in engen Kurven | Gegenspur nah an eigener Spur | `min_lane_width` erhöhen |
| Weiße Linie springt bei Lichtreflexen | Einzelne Fehlmessungen | `max_frame_jump` verringern |
| Gegenspur-Haltelinie löst Stopp aus | Horizontale ROI zu breit | `detection_x_start` erhöhen |
| Bot hält zu früh an | Vertikale ROI zu weit oben | `detection_zone` erhöhen |
| Bot fährt nach Neustart nicht | Fehlender Parameter in JSON → `KeyError` in `cbUpdateParameters` | Alle `.json`-Dateien auf den Bot übertragen |
| Spurerkennung bei Schatten instabil | CLAHE alleine reicht nicht | HSV-Werte auf der aktuellen Strecke neu kalibrieren |
| `cbUpdateParameters` crasht still | Bug in `util.py`: Callback feuert für alle Nodes | Bekannter Bug, Workaround: alle Nodes haben dieselbe JSON-Struktur |
