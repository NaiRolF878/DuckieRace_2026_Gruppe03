# Avoid Ducks (Challenge 3)

Dieses Paket implementiert das autonome Ausweichen von Hindernissen – speziell von Enten (Rubber Ducks) auf der Fahrbahn. Im Gegensatz zu einer hierarchischen State-Machine vereint dieses Paket die komplette Logik von Spurfolge (Lane Following) und Hinderniserkennung (YOLO) direkt in einer zonenbasierten Wahrnehmung.

**Herkunft:** Das Grundgerüst dieses Pakets (Homographie-Zonenmodell, YOLO-Integration, Zustandsautomat) stammt von Magnusneumann. Wir haben uns bewusst für dieses Paket als unsere Lösung für die Enten-Ausweichung entschieden (statt unserer eigenen `ducks`-Package-Logik) und bauen es seither weiter aus (siehe "Änderungen" unten).

## Architektur & Technische Funktionsweise

Die Herausforderung besteht hier darin, klassische Bildverarbeitung (Linien-Tracking) mit neuronalen Netzen in Echtzeit zu kombinieren, ohne dass sich die Steuerungssignale widersprechen. Statt einer simplen P-Regelung nutzt dieses System ein projiziertes Vogelperspektiven-Modell (Homographie).

### 1. Objekterkennung (YOLO) - `detect_ducks_node.py`
Dieser Node integriert ein trainiertes YOLO-Modell (You Only Look Once), um Objekte im Kamerabild zu klassifizieren und zu lokalisieren.
- **Inferenz:** Das Kamerabild wird durch das neuronale Netz geschickt. Das Netz liefert Bounding Boxen (`Polygon`-Nachrichten) und Konfidenz-Werte (Probability) für gefundene Enten zurück.
- **Publisher:** Die Boxen der Enten werden auf dem Topic `/{vehicle}/detect/duck_obstacles` veröffentlicht, wo sie vom Haupt-Node (Avoidance Node) verarbeitet werden.
- Das fertig annotierte Debug-Bild wird aktuell **nicht** veröffentlicht (`pub_debug_duck.publish(...)` ist auskommentiert) – nur lokal im Speicher erzeugt.

### 2. Zonen-Logik & Regelung - `duck_avoidance_node.py`
Dieses Node ersetzt den klassischen Lane-Follower und Switch-Control komplett. Es projiziert das 2D-Kamerabild mithilfe von Intrinsics und Homographie in den 3D-Raum (bzw. auf die 2D-Bodenebene), um echte Metriken (Zentimeter) zu erhalten.

Die Fahrbahn direkt vor dem Roboter wird in **drei physische Zonen** aufgeteilt (z.B. Zone 1: ganz nah, Zone 2: mittel, Zone 3: fern).
- **Perzeption (Segmentierung):**
  - Die Kamera extrahiert mithilfe von HSV-Masken weiße (rechter Rand) und gelbe (Mittelstreifen) Pixel.
  - Das System überprüft mathematisch (via `shapely.geometry`), wie viele weiße/gelbe Pixel oder detektierte Enten-Bounding-Boxen in die jeweiligen Zonen fallen.
- **Lane Following (Spurfolge):**
  - Das Lenken (`omega`) passiert hierbei rein basierend darauf, welche Zonen blockiert sind!
  - Blockiert der rechte Bildrand (weiße Linie) die Zone, lenkt er nach links. Blockiert der gelbe Mittelstreifen die Zone, lenkt er nach rechts.
- **Enten-Ausweichen (Duck Avoidance):**
  - Sobald eine Ente in einer der Zonen vor dem Roboter registriert wird, greift sofort eine Ausweich- oder Bremslogik ein.
  - Befindet sich eine Ente in Zone 1 (kritisch nah), stoppt der Roboter sofort (`v=0`).
  - Je nachdem, in welcher Zone (links/rechts/mitte) eine Ente in der Ferne erkannt wird, lenkt der Roboter geschmeidig in die gegenüberliegende freie Zone aus, um das Objekt rechtzeitig zu umfahren (Wiggle-Bewegung / Ausweichmanöver).

Debug-Fenster (`cv2.imshow("Duck Avoidance Challange", ...)`) zeigt die drei Zonen (rot = Gefahr, gelb = frei), erkannte Enten (grün) sowie unten die aktuelle FSM-Aktion in Klartext. Wie das Original ist auch dieser Node nur **lokal** sichtbar (kein ROS-Topic-Publish des Debug-Bilds), braucht also einen Bildschirm direkt am Bot oder X-Forwarding.

### 3. Effizienz und Hardware
Das Ausführen neuronaler Netze auf Edge-Devices ist rechenintensiv. Das Enten-Netz (YOLO) wertet die Bilder daher oft asynchron oder in gedrosselter Auflösung aus, während die Zonen-Segmentierung (Homographie & HSV) für die reine Spurführung in hoher Taktfrequenz (`buffer_size`) weiterläuft, um die dynamische Stabilität in Kurven und beim Umfahren der Hindernisse zu garantieren.

## Änderungen (2026-07-22)

Nach der Übernahme des Pakets war das Team mit der Zuverlässigkeit des Ausweichens noch nicht zufrieden. Folgende Anpassungen wurden vorgenommen:

### Konfigurierbare Parameter statt Hardcoding — `config/duck_avoidance_node.json`
Alle Wackel-/Such-Parameter waren vorher fest im Code (`wiggle_power = 0.08` usw.). Jetzt in einer JSON-Datei, live per `/update_parameters` änderbar (gleiches Prinzip wie in den anderen Packages, siehe `util.py`):

| Parameter | Bedeutung |
|---|---|
| `wiggle.power` | Stärke des Vor/Zurück-"Wackelns" beim Drehen auf der Stelle (überwindet Rollmoment) |
| `wiggle.interval_seconds` | Wie oft die Wackel-Richtung wechselt |
| `search.escape_omega` | Dreh-Geschwindigkeit während des Ausweichens |
| `search.inversion_cooldown_seconds` | Mindestabstand zwischen zwei Richtungs-Korrekturen während des Ausweichens |
| `memory.duck_seconds` | Positions-Gedächtnis für Enten-Bounding-Boxen (siehe unten) |

### Debug-Overlay zeigt die tatsächliche FSM-Aktion
Der Text im Debug-Fenster kam vorher direkt aus den rohen Motorbefehlen ("Ich würde gerne: fahren und links" – bei jedem Wackel-Tick praktisch identisch, unabhängig vom Zustand). Jetzt zeigt `_state_action_text()` die tatsächliche Aktion in Klartext: "Freie Fahrt", "Spurkorrektur", "Weiche aus wegen Ente (rechts)", "Fahre an Ente vorbei".

### Kurzes Positions-Gedächtnis für Enten (`cb_ducks`)
`detect_ducks_node` publiziert bewusst auch **leere** Nachrichten, sobald das YOLO-Modell in einem Frame nichts findet. Ohne Gedächtnis würde die Zonen-Gefahrenprüfung dadurch bei jedem verpassten Frame sofort "frei" melden, obwohl die Ente nur kurz nicht erkannt wurde (Bild verwackelt, kurzzeitige Verdeckung).

`cb_ducks` matcht eingehende Bounding-Boxen gegen die zuletzt bekannten (Bildpixel-Abstand < 50px = dieselbe Ente) und hält eine unbestätigte, aber noch frische Box am Leben, statt sie sofort zu verwerfen. `cb_image` räumt zusätzlich bei jedem Frame Boxen auf, die zu lange nicht mehr bestätigt wurden. Beide Fenster nutzen denselben konfigurierbaren Wert `memory.duck_seconds` (Standard 0.5s, siehe Parameter-Tabelle oben).

**Kurzzeitig ausprobiert und wieder verworfen:** Eine Variante mit Positions-Gedächtnis in echten **Weltkoordinaten** (inverse Homographie, Odometrie-Korrektur bei Bot-Drehung, Sonderbehandlung für den toten Winkel näher als Zone 0) wurde gebaut und mit simulierter Odometrie erfolgreich getestet, im Refactor vom 2026-07-23 aber zugunsten der einfacheren Bildkoordinaten-Variante wieder zurückgebaut.

**Kurzzeitig ausprobiert und wieder verworfen:** Ein Zustand, der vor der Richtungs-Entscheidung erst kurz nach links UND rechts testet (statt sofort per Heuristik zu entscheiden), hat sich in der Praxis nicht bewährt und wurde wieder entfernt – die Ausweichrichtung wird weiterhin direkt aus der Heuristik (Entenposition/Linienfarbe) plus nachträglichem Inversions-Check bestimmt, wie ursprünglich.
