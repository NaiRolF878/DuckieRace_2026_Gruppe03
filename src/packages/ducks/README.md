# Challenge 3 – Watch out for Ducks

> Baut auf Challenge 1 auf · ROS 1 (Noetic) · Python 3 · OpenCV

Der Duckiebot erkennt starre Enten-Hindernisse auf der Fahrbahn und weicht ihnen aus. Die Ausweichrichtung wird anhand des verfügbaren Platzes links und rechts der Ente gewählt. Bei zu wenig Platz auf beiden Seiten wird die Gegenspur übernommen.

---

## Inhaltsverzeichnis

- [Dateien](#dateien)
- [Systemüberblick](#systemüberblick)
- [Nodes](#nodes)
- [Erkennungsprinzip](#erkennungsprinzip)
- [Ausweichlogik](#ausweichlogik)
- [Konfigurationsparameter](#konfigurationsparameter)
- [Setup & Starten](#setup--starten)
- [Kalibrierung](#kalibrierung)
- [Bekannte Probleme & Lösungen](#bekannte-probleme--lösungen)

---

## Dateien

| Datei | Typ | Beschreibung |
|---|---|---|
| `detect_duck_node.py` | Node | Enten-Erkennung: Helligkeit + Hough-Kreise |
| `control_obstacle_node.py` | Node | Ausweichlogik mit kontrollierter Rückkehr |
| `switch_control_node.py` | Node | Erweitert: schaltet bei Ente auf Obstacle-Modus |
| `detect_duck_node.json` | Config | Helligkeitsschwellwerte, Hough-Parameter, ROI |
| `control_obstacle_node.json` | Config | Ausweich-Offsets, Rückkehr-Parameter |

**Abhängigkeit:** Alle Nodes aus Challenge 1 werden weiterhin benötigt.

---

## Systemüberblick

```
Kamera (Originalbild + Bird's-Eye-View)
    │
    └──▶ detect_duck_node
           │  Ansatz 1: Helligkeit im Bird's-Eye-View ROI
           │  Ansatz 2: Hough-Kreise auf Originalbild
           │  Beide müssen anschlagen → Ente erkannt
           │
           ├──▶ /detect/duck         (Float64, x-Position [-1,+1], -99=keine)
           └──▶ /debug/duck_detection (Bounding-Box Debug)

switch_control_node
    ◀── /detect/duck    (Ente erkannt? → Obstacle-Modus)
    ──▶ /switch/control (1=Lane, 2=Obstacle, 3=Intersection)

control_obstacle_node
    ◀── /detect/duck    (x-Position der Ente)
    ◀── /detect/lane    (aktueller Spurversatz)
    ◀── /switch/control (aktiviert/deaktiviert)
    ──▶ /detect/lane_corrected (korrigierter Versatz an control_lane_node)

control_lane_node
    ◀── /detect/lane           (normaler Betrieb)
    ◀── /detect/lane_corrected (Obstacle-Modus: mit Ausweich-Offset)
```

---

## Nodes

### detect\_duck\_node

Kombiniert zwei unabhängige Erkennungsansätze – beide müssen gleichzeitig anschlagen um eine Ente zu melden. Das reduziert Fehlalarme stark.

#### Ansatz 1 – Helligkeitsprüfung (Bird's-Eye-View)

**Idee:** Der Fahrbahnbereich zwischen gelber und weißer Linie ist schwarz. Helle Pixel in diesem Bereich deuten auf ein Hindernis hin.

```
|gelb|  [ROI: sollte schwarz sein]  |weiß|
          ↑                    ↑
     center_yellow        center_white
     (aus Spurversatz geschätzt)
```

**Pipeline:**
1. ROI ausschneiden: zwischen `center_yellow` und `center_white`, vordere Hälfte (konfigurierbar)
2. Gauß-Filter: Spiegelungsartefakte glätten
3. Schwellwert → Binärmaske heller Pixel
4. `MORPH_OPEN`: kleine Spiegelungs-Pixel entfernen, große Blobs (Ente) behalten
5. Anteil heller Pixel > `brightness_ratio` → Helligkeit erkannt

**Warum nur vordere Hälfte?**
Weiter entfernte Hindernisse geben dem Bot Zeit zu reagieren bevor ausgewichen werden muss.

#### Ansatz 2 – Hough-Kreise (Originalbild)

**Idee:** Enten haben runde Formen. `cv2.HoughCircles` erkennt kreisförmige Objekte.

**Warum Originalbild statt Bird's-Eye-View?**
Die Perspektivtransformation verzerrt Kreise zu Ellipsen – `HoughCircles` sucht explizit nach Kreisen und würde sie nicht mehr finden.

**Nur untere Bildhälfte prüfen:** Enten befinden sich auf der Fahrbahn → nur im unteren Bildbereich relevant. Deckenlampen, Fenster etc. werden ignoriert.

#### Kombinierte Entscheidung

```
brightness_detected AND hough_detected → Ente erkannt ✅
brightness_detected OR  hough_detected → kein Alarm (Fehlalarm verhindert)
```

#### Enten-Position

Die x-Position des Hough-Kreismittelpunkts wird auf `[-1, +1]` normiert und publiziert:
- `-1.0` = ganz links
- `0.0` = Bildmitte
- `+1.0` = ganz rechts
- `-99.0` = keine Ente erkannt

**Debug-Bild:**
- Gelber Kreis = Hough-Kreis gefunden aber noch kein Alarm
- Rote Bounding-Box + "ENTE!" = Ente bestätigt (beide Ansätze)
- Statuszeilen: Helligkeitswert und Hough-Status

**Publiziert:**

| Topic | Typ | Beschreibung |
|---|---|---|
| `/detect/duck` | `Float64` | x-Position der Ente (`-99` = keine Ente) |
| `/debug/duck_detection` | `CompressedImage` | Debug-Bild mit Bounding-Box |

---

### control\_obstacle\_node

Empfängt die Enten-Position und berechnet einen Ausweich-Offset der auf den normalen Spurversatz addiert wird. Der PID-Regler in `control_lane_node` denkt die Spurmitte hat sich verschoben und lenkt entsprechend.

**Warum Offset statt direkter Steuerung?**
Der PID-Regler läuft weiter normal – er gleicht Schwankungen aus, hält die Spur und kehrt nach dem Ausweichen automatisch zurück. Es muss kein separater Controller implementiert werden.

#### EvadeState-Zustandsautomat

```
Idle ──(Ente erkannt)──▶ Evading
                            │ Ausweich-Offset halten
                            │ Ente verschwunden
                            ▼
Idle ◀──(offset=0)────── Returning
                            Offset schrittweise → 0
                            (kontrollierte Rückkehr)
```

#### Ausweichentscheidung

```
Platz links  = ente_x_pixel - center_yellow
Platz rechts = center_white - ente_x_pixel

platz_rechts > platz_links          → nach rechts (offset = -max_offset)
platz_links  > platz_rechts         → nach links  (offset = +max_offset)
beide gleich (Ente mittig)          → nach links  (StVO: zur gelben Linie)
beide < min_side_space              → Gegenspurübernahme (offset = +overtake_offset)
```

**Warum bei gleich viel Platz nach links?**
Entspricht der deutschen Straßenverkehrsordnung – bei Hindernissen auf der Fahrbahn weicht man zur Mittellinie (gelbe Linie) aus, nicht zum Rand.

**Gegenspurübernahme:**
Da während der Challenge keine anderen Bots auf der Strecke sind, ist die Gegenspur frei. Der Bot fährt weit genug nach links um an der Ente vorbeizukommen und kehrt danach kontrolliert zurück.

**Kontrollierte Rückkehr:**
Offset wird pro Schritt um `return_step` reduziert bis er unter `return_threshold` liegt → sanfte Rückkehr ohne Lenkruck.

**Publiziert:**

| Topic | Typ | Beschreibung |
|---|---|---|
| `/detect/lane_corrected` | `Float64` | Spurversatz + Ausweich-Offset |

---

### switch\_control\_node (Erweiterung)

Schaltet auf `Obstacle`-Modus (Wert 2) wenn `/detect/duck` einen Wert ≠ `-99` meldet:

```
Lane ──(duck≠-99)──▶ Obstacle
                         │ control_obstacle_node aktiv
                         │ duck=-99 (Ente weg)
Lane ◀───────────────────┘
```

---

## Erkennungsprinzip

```
Bird's-Eye-View:           Originalbild:
                            
|gelb|[ROI schwarz?]|weiß|  ┌──────────────────┐
       ↓ hell?               │                  │
  Helligkeit erkannt         │    (  Kreis?  )  │
                             │                  │
                             └──────────────────┘
                              Hough erkannt
       
       BEIDE → Ente bestätigt
```

---

## Ausweichlogik

```
Ente bei x=100px, center_yellow=50, center_white=350

Platz links  = 100 - 50  = 50 px
Platz rechts = 350 - 100 = 250 px

→ Mehr Platz rechts → nach rechts ausweichen
  offset = -max_offset
  corrected_error = lane_error + offset
  → PID lenkt nach rechts
```

---

## Konfigurationsparameter

### detect\_duck\_node.json

#### `detection` – Helligkeitsprüfung

| Parameter | Default | Beschreibung |
|---|---|---|
| `brightness_threshold` | 60 | Grauwert ab dem Pixel als hell gilt (0=schwarz, 255=weiß) |
| `brightness_ratio` | 0.15 | Mindestanteil heller Pixel im ROI (15%) |
| `min_side_space` | 40 px | Mindestplatz für normales Ausweichen |
| `roi_start` | 0.5 | Vertikale ROI-Obergrenze im Bird's-Eye-View |
| `roi_end` | 1.0 | Vertikale ROI-Untergrenze (1.0 = bis ganz unten) |

#### `hough` – Hough-Kreise

| Parameter | Default | Beschreibung |
|---|---|---|
| `dp` | 1.0 | Akkumulator-Auflösung (1 = gleich wie Bild) |
| `min_dist` | 30 px | Mindestabstand zwischen zwei Kreisen |
| `param1` | 50 | Canny-Kantenschwelle (höher = weniger empfindlich) |
| `param2` | 30 | Akkumulator-Schwelle (niedriger = mehr Kreise) |
| `min_radius` | 10 px | Minimaler Kreisradius |
| `max_radius` | 80 px | Maximaler Kreisradius |
| `roi_start` | 0.4 | Unterhalb welchem Bildanteil nach Kreisen gesucht wird |

#### `crop_image` – Bird's-Eye-View

Gleiche Werte wie `detect_lane_node.json` verwenden – muss kalibriert übereinstimmen.

### control\_obstacle\_node.json

| Parameter | Default | Beschreibung |
|---|---|---|
| `max_offset` | 0.4 | Ausweich-Offset für normales Ausweichen |
| `overtake_offset` | 0.8 | Offset für Gegenspurübernahme |
| `min_side_space` | 40 px | Unter diesem Wert → Gegenspurübernahme |
| `return_step` | 0.05 | Offset-Reduktion pro Schritt (10 Hz = 0.5/s) |
| `return_threshold` | 0.02 | Offset gilt als 0 wenn unter diesem Wert |

---

## Setup & Starten

```bash
# Zusätzlich zu Challenge 1:

# Nodes starten (zusätzlich zu Challenge 1)
python3 src/detect_duck_node.py
python3 src/control_obstacle_node.py
```

`switch_control_node.py` aus Challenge 1/2 ersetzen – neue Version verwenden.

---

## Kalibrierung

### Enten-Erkennung kalibrieren

1. `configuration_node` → Node `detect_duck_node` → Gruppe `detection`
2. Debug Image `/debug/duck_detection` auswählen
3. Bot auf Strecke ohne Ente → `brightness_ratio` erhöhen bis keine Fehlalarme
4. Bot vor Ente → `brightness_ratio` senken bis Ente erkannt wird
5. Gruppe `hough`: `param2` senken wenn Hough-Kreise nicht gefunden werden

### Ausweich-Parameter kalibrieren

1. `configuration_node` → Node `control_obstacle_node`
2. `max_offset` so einstellen dass Bot sauber an Ente vorbeifährt
3. `return_step`: größer = schnellere Rückkehr, kleiner = sanftere Rückkehr
4. `min_side_space` anpassen je nach Entenbreite

### ROI-Parameter kalibrieren

1. `roi_start` erhöhen wenn der Bot zu früh auf weit entfernte Enten reagiert
2. `hough.roi_start` erhöhen wenn Fehlalarme durch Objekte im oberen Bildbereich

---

## Bekannte Probleme & Lösungen

| Problem | Ursache | Lösung |
|---|---|---|
| Fehlalarme durch Spiegelungen | Boden reflektiert Licht | `brightness_ratio` erhöhen, `MORPH_OPEN` filtert kleine Reflexe |
| Ente wird nicht erkannt | Hough-Kreise zu streng | `param2` senken (empfindlicher), `min_radius` anpassen |
| Hough-Kreise findet falsche Objekte | ROI zu groß | `hough.roi_start` erhöhen |
| Ausweichen zu stark | `max_offset` zu groß | Wert reduzieren |
| Rückkehr ruckartig | `return_step` zu groß | Wert verringern |
| Gegenspurübernahme obwohl Platz da | `min_side_space` zu groß | Wert reduzieren |
| Ente blockiert gesamte Fahrbahn | Kein Platz auf beiden Seiten | Gegenspurübernahme aktiv – funktioniert da keine anderen Bots auf Strecke |
