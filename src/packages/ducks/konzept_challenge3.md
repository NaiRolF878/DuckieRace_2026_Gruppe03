# Konzept – Challenge 3: Watch out for Ducks

Stand der Diskussion. Dieses Dokument hält die Strategie fest, **bevor** Code
geschrieben wird. Es ist bewusst als Diskussions- und Testgrundlage gedacht,
nicht als fertige Spezifikation.

---

## 1. Grundidee in einem Satz

Der Bot orientiert sich **nur an der weißen Seitenlinie** (fester seitlicher
Abstand). Alles, was in einen definierten **Störbereich** vor dem Bot gerät –
Ente **oder** gelbe Linie – löst ein **Ausweichen** aus, weg vom Objekt. Dadurch
wird die gelbe Linie nie überfahren, ohne dass sie je gemessen werden muss.

---

## 2. Warum dieser Ansatz (Lehren aus den Tests)

- Die alte Spurführung (Mitte zwischen gelb und weiß) hatte zwei Probleme:
  gelbe Linie und gelbe Enten sind farblich nicht trennbar, und auf dem
  Wendeplatz fehlt die gelbe Linie ganz.
- Beobachtung beim Test: Mit reinem Follow-Lane ist der Bot **gut durch den
  Wendeplatz gefahren**. Die weiße Linie reicht als Referenz.
- Schlüsselerkenntnis: Wenn die gelbe Linie **dasselbe Ausweichen** auslöst wie
  eine Ente, müssen Linie und Ente **nicht mehr unterschieden** werden – beide
  bekommen dieselbe Reaktion (weg davon). Das löst das Trennungsproblem auf,
  an dem alle vorherigen Ansätze gescheitert sind.

---

## 3. Spurführung (Follow-Lane)

- Referenz ist **ausschließlich die weiße Linie**.
- Der Bot hält einen **festen seitlichen Abstand** zur weißen Linie.
- Die tatsächliche Spurbreite ist dabei egal – der Abstand zur weißen Linie
  ist die einzige Regelgröße.
- Das vorhandene `last_known`-Tracking der Linienposition bleibt wichtig, um
  kurze Aussetzer (Linie zeitweise nicht sichtbar) zu überbrücken.

**Offen / zu kalibrieren:**
- Die weiße Linie ist **immer rechts**, solange der Bot im Follow-Modus ist –
  das ist eine feste Annahme, keine Variable (siehe auch Abschnitt 7a: wechselt
  sie die Seite, ist das ein Fehlerindikator für eine Sackgasse, kein normaler
  Zustand). Der Rundkurs am Wendeplatz ändert nur die Krümmung der Linie,
  nicht ihre Seite.
- Genauer Sollabstand in BEV-Pixeln.
- In engen Kurven (Wendeplatz) ist die weiße Linie stärker gekrümmt – das
  Tracking muss damit zurechtkommen; ggf. mögliche Winkelverzerrung des
  gemessenen Abstands beachten.

---

## 4. Störbereich & Zonen

- Im Bird's-Eye-View werden **gestaffelte Zonen** vor dem Bot definiert:
  **nah / mittel / fern**.
- Die Zonen decken **nur den Fahrkorridor** ab (nicht die volle Bildbreite),
  damit Objekte weit neben der Spur nicht unnötig auslösen.
- Eine Zone gilt als **belegt**, wenn dort ein Objekt erkannt wird
  (Ente oder gelbe Linie).
- Staffelung erlaubt Vorausschau: ferne Zone = Frühwarnung (z.B. Tempo raus),
  nahe Zone = Ausweichen auslösen.

**Erkennung der Objekte in den Zonen:**
- Farb-robust über Helligkeit (helle Objekte auf dunklem Boden) – erkennt
  gelbe UND andersfarbige (Bonus-)Enten.
- Die gelbe Linie wird NICHT herausgefiltert – sie SOLL ja auslösen.

**Offen / zu kalibrieren:**
- Geometrie der drei Zonen (Größe, Abstände, Breite des Korridors).
- Wie viele belegte Pixel/Fläche = "Zone belegt".

---

## 5. Ausweichen

- **Auslöser:** Objekt in einer Zone des Störbereichs.
- **Richtung:** hängt davon ab, **von welcher Seite** das Objekt in den
  Störbereich kommt – der Bot weicht zur freien Seite aus.
- Die **gelbe Linie als Objekt** sorgt dafür, dass ein Ausweichen Richtung
  gelb automatisch gestoppt/umgekehrt wird → gelbe Linie wird nie überfahren.
- **Während des Ausweichens** wird das Manöver NICHT abgebrochen, auch wenn eine
  weitere Ente in den Bereich kommt. Erst wenn der Korridor wieder frei ist.
- Nach der letzten Sichtung fährt der Bot einen **festen Nachlauf** weiter,
  um sicher am Hindernis vorbei zu sein, bevor die Rückkehr beginnt.

**Sonderfall – kein Platz:**
- Wenn der erlaubte Korridor (zwischen weißer Linie und gelber Grenze) komplett
  blockiert ist: **Anhalten und warten**, bis wieder frei.
  (Gegenspur ist NICHT erlaubt, gelbe Linie ist harte Grenze.)

---

## 6. Rückkehr zur Spur (Encoder-gestützt)

- Das Ausweichmanöver wird über die **Radencoder** mitgeschrieben
  (zurückgelegte Drehung/Strecke).
- Rückkehr: Der Bot dreht über die Encoder-Werte zurück – **bis die Kamera die
  weiße Linie wieder sieht**. Die Kamera ist die Abbruchbedingung, der Encoder
  treibt die Bewegung.
- Dadurch ist die Encoder-Genauigkeit unkritisch: Schlupf verlängert die
  Drehung höchstens minimal, das Ziel ("weiße Linie sichtbar") bleibt korrekt.
- **Keine** festen Drehwinkel, **keine** festen Zeitwerte – ausdrücklich
  Encoder-basiert.

**Reihenfolge der Rückkehr (wichtig):**
1. Korridor frei (Zone leer) UND fester Nachlauf gefahren
2. ERST DANN: per Encoder zurückdrehen, bis Kamera weiße Linie hat
3. Übergang zurück in Follow-Lane (Kamera übernimmt Feinkorrektur)

→ Nicht "weiße Linie kurz gesehen → sofort zurück", sonst Abbruch zu früh.

---

## 7. Zustandsautomat

```
   ┌─────────────────────────────────────────────────────────┐
   │                                                         │
   ▼                                                         │
FOLGEN ──Objekt in Zone──► AUSWEICHEN ──Korridor frei──► PASSIEREN
(weiße Linie,              (weg vom Objekt,              (fester
 fester Abstand)           Richtung = freie Seite,        Nachlauf)
   ▲                       hält bis frei)                    │
   │                                                         ▼
   └──────────── ZURÜCKKEHREN ◄───────────────────────────────┘
       (Encoder zurück, bis Kamera weiße Linie sieht)

   Sonderfall in AUSWEICHEN: Korridor ganz blockiert → ANHALTEN & WARTEN
```

---

## 7a. Sackgassen-Erkennung und Umfahren

**Problem:** Bei dicht gestaffelten Enten (Hindernisse in einer durchgehenden
Diagonale über den Korridor) können die Lücken zwischen ihnen zu eng sein, um
hindurchzufahren. Der Bot versucht dann ggf. so weit auszuweichen/sich zu
drehen, dass er sich faktisch "verkeilt" – eine Sackgasse. Kritisch dabei:
Dreht sich der Bot zu weit, wechselt die weiße Linie die Bildseite, was die
gesamte Spurführungs-Annahme ("weiß ist rechts") bricht UND der Streckenregel
widerspricht, dass der Wendehammer nur in vorgegebener Richtung durchfahren
werden darf (nicht an der Einfahrt wieder verlassen werden darf).

**Erkennung (Indikator):**
- Der Bot summiert während des Ausweichens fortlaufend seine Eigendrehung
  (aus den Encodern).
- Eine Sackgasse liegt vor, wenn diese akkumulierte Drehung zu groß wird UND/
  ODER die weiße Linie auf der "falschen" Bildseite erscheint (sie sollte
  immer rechts sein – erscheint sie links, hat sich der Bot zu weit gedreht).
- Dieser Indikator ist eine reine Tatsachenfeststellung aus vorhandenen
  Messgrößen, keine Vorhersage – robuster als der Versuch, eine Sackgasse im
  Voraus zu erkennen.

**Reaktion – Außenrand-Fahren statt Rückwärtsfahren oder Drehen:**
- Sobald der Indikator anschlägt, wechselt der Bot in einen eigenen Modus:
  Er fährt NICHT zurück (kein Rückwärtsfahren ohne Rückwärtssicht) und dreht
  sich NICHT weiter (würde die Streckenrichtung verletzen).
- Stattdessen fährt er am ÄUSSEREN Rand der gesamten Hindernisreihe entlang
  (nah an der gelben Grenze, ohne sie zu überfahren), statt zu versuchen,
  durch einzelne Lücken zu navigieren.
- **Mindeststrecke aus Encodern:** Die Strecke, die der Bot zurücklegen musste,
  bis der Sackgassen-Indikator anschlug, wird als Mindestfahrstrecke für das
  Außenrand-Fahren verwendet (plus Sicherheitsaufschlag, z.B. Faktor 1,5).
  Das verhindert, dass der Bot sofort wieder in dieselbe Lücke einlenkt.
- **Selbstkorrigierend durch laufende Erkennung:** Die normale Zonen-Erkennung
  bleibt während des gesamten Außenrand-Fahrens aktiv. Schlägt sie nach
  Ablauf der Mindeststrecke erneut an (Hindernis weiterhin im Weg), wird die
  Mindeststrecke nicht neu berechnet, sondern einfach weiter am Rand entlang-
  gefahren (ggf. mit wachsendem Sicherheitsaufschlag bei wiederholtem
  Anschlagen). Dadurch muss die ursprüngliche Encoder-Schätzung nicht exakt
  sein – sie muss nur das sofortige Zurückpendeln verhindern, den Rest
  übernimmt die Kamera/Zonen-Erkennung als Wächter.
- **Abbruch:** wie beim normalen Ausweichen – Korridor in der Zonen-Erkennung
  durchgehend frei → zurück zu FOLGEN (weiße Linie, fester Abstand).

**Warum nicht Drehen oder Rückwärtsfahren:**
- Drehen (bis keine Ente mehr sichtbar) würde "weiß ist rechts" umkehren und
  den Bot zudem in die falsche Streckenrichtung bringen (Wendehammer darf nicht
  an der Einfahrt verlassen werden).
- Rückwärtsfahren ist riskant, weil die Kamera nach vorne zeigt – der Bot
  könnte beim Zurücksetzen unbemerkt eine Ente touchieren oder die gelbe
  Grenze verlassen. Eine "exakt rückwärts abspielen"-Variante scheitert daran,
  dass die normale Abbruchbedingung (Kamera sieht Weiß) in der Sackgasse nicht
  eindeutig ist (kurzes Weiß-Aufblitzen ≠ sicher draußen).

---

## 8. Technische Voraussetzungen (vor dem Bau prüfen!)

1. **Radencoder-Topics** – VERIFIZIERT:
   - `/track/left_wheel_encoder_node/tick`
   - `/track/right_wheel_encoder_node/tick`
   - Typ: `duckietown_msgs/WheelEncoderStamped`, Feld `data` (kumulative Ticks),
     Feld `resolution` = 135 (Ticks/Umdrehung, beide Räder).
   - **Keine Richtungserkennung:** `data` zählt bei JEDER Bewegung aufwärts,
     egal ob vor- oder rückwärts. Drehrichtung MUSS aus dem gesendeten
     Fahrbefehl (Vorzeichen v/omega) abgeleitet werden.
2. **Kinematik** (aus rosparam, bot-spezifisch kalibriert):
   - radius = 0.0318 m, baseline = 0.1 m, trim = 0.0, gain = 1.0 (Standard).
   - Strecke pro Tick ≈ 1.48 mm. Drehwinkel: Δθ = (s_rechts − s_links) / baseline.
3. **Fahrbefehl-Ausgabe:** `/track/car_cmd_switch_node/cmd`
   (`duckietown_msgs/Twist2DStamped`, Felder v, omega). NICHT auf andere
   car_cmd-Topics schreiben (joy_mapper, lane_controller, etc. – die werden
   vom switch_node ignoriert/überschrieben).
4. Weiße Linie zuverlässig sichtbar – auch auf dem Wendeplatz?
5. Latenz im Griff (buff_size gesetzt, kein Bildstau).

---

## 9. Offene Punkte / Risiken

- **Weiße Linie verschwindet beim Ausweichen nach innen** → während des
  Manövers evtl. keine Spurreferenz. Wird durch "erst Korridor frei + Nachlauf,
  dann zurück" abgefedert, sollte aber im Test beobachtet werden.
- **Tracking der weißen Linie** am Wendeplatz (starke Krümmung im Rundkurs) –
  bleibt die Erkennung stabil rechts, oder reißt das Tracking ab? (Die Seite
  selbst ist eine feste Annahme, siehe Abschnitt 3 – wechselt sie tatsächlich,
  ist das der Sackgassen-Fehlerfall aus Abschnitt 7a, kein Normalzustand.)
- **Zonen-Geometrie** ist reine Kalibriersache, am Bot zu justieren.
- **Anhalten-und-warten**: Laut Einschätzung der Prüfer gibt es **immer genug
  Platz** → dieser Sonderfall ist praktisch unkritisch, keine eigene
  Vorbeitast-Logik nötig. ABER: Ein Erkennungsfehler (Schatten, Reflex, gelbe
  Linie falsch interpretiert) könnte den Korridor fälschlich als blockiert
  melden und den Bot dauerhaft anhalten lassen.
  **Gewählte Lösung:** Ein **Sicherheits-Timeout** im Anhalten-Zustand – steht
  der Bot zu lange, **fährt er vorsichtig weiter / versucht das Ausweichen
  erneut**, statt dauerhaft stehen zu bleiben. Absicherung gegen Fehlerkennung,
  nicht gegen echte Blockaden.
- **Gelbe Linie als Trigger**: Sensitivität so einstellen, dass sie zuverlässig
  auslöst, ohne dass der Bot ständig "vor der eigenen Spur flüchtet".
- **Sackgassen-Mechanismus (Abschnitt 7a)** ist noch nicht am Bot getestet –
  Schwellwert für "zu viel Drehung" und Sicherheitsaufschlag-Faktor sind reine
  Kalibrierwerte.

---

## 10. Empfohlene Baureihenfolge (für die knappen Testtage)

1. **Encoder verifizieren** (Topics da? Werte plausibel?). ✅ ERLEDIGT.
2. **Follow-Lane auf weiße Linie + festen Abstand** umbauen, isoliert testen.
3. **Zonen-Erkennung** im BEV (Debug-Bild: Zonen + belegt/frei anzeigen),
   ohne Ausweichen – nur visuell prüfen, ob Zonen korrekt belegt werden.
4. **Ausweichen** dazu (Richtung nach Objektseite), mit kamerabasierter
   Rückkehr zuerst.
5. **Encoder-Rückkehr** ergänzen, wenn der Rest stabil läuft.
6. **Gelbe Linie als Trigger** scharf schalten und Sonderfall "anhalten" testen.
7. **Sackgassen-Erkennung & Außenrand-Fahren** (Abschnitt 7a) ergänzen, wenn
   der Rest stabil läuft.

Jede Stufe einzeln testbar halten (Schalter zum Aktivieren/Deaktivieren).
