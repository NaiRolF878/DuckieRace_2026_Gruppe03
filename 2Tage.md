# A2 – Ermittlung des maximalen hydrostatischen Innendrucks

## Ziel

Ziel dieser Berechnung ist die Ermittlung des maximal zu erwartenden Innendrucks innerhalb der Mud Bucket.

Der ermittelte Druck dient als Lastannahme für die Dimensionierung des Gehäuses sowie für die Berechnung der auf das Schließsystem wirkenden Kräfte.

---

## Lastfall

Der betrachtete Lastfall beschreibt einen vollständig verstopften Ablauf der Mud Bucket während eines Wet-Tripping-Vorgangs.

Es wird angenommen, dass:

- keine aktive Förderung durch die Spülpumpen erfolgt,
- der Ablauf der Mud Bucket vollständig blockiert ist,
- die Bohrspülung ausschließlich durch die hydrostatische Flüssigkeitssäule innerhalb des Bohrgestänges belastet wird.

Da während des Trennvorgangs keine Pumpenleistung anliegt, stellt dieser Lastfall den maximal zu erwartenden Innendruck für die Konzeptphase dar.

---

## Annahmen

| Parameter | Symbol | Wert |
|-----------|--------|------:|
| Bemessungsfall | – | Range 3 Double |
| Flüssigkeitssäule | h | 28 m |
| Dichte der Bohrspülung | ρ | 2.000 kg/m³ |
| Erdbeschleunigung | g | 9,81 m/s² |

---

## Berechnungsgrundlage

Der hydrostatische Druck ergibt sich aus

\[
p=\rho \cdot g \cdot h
\]

---

## Berechnung

\[
p
=
2000
\cdot
9,81
\cdot
28
=
549\,360\;Pa
\]

\[
p
=
0,549\;MPa
=
5,49\;bar
\]

Für die weitere Auslegung wird konservativ auf

> **6 bar**

aufgerundet.

---

## Ergebnis

Als Bemessungsdruck für sämtliche weiteren Berechnungen wird angesetzt

> **p = 6 bar**

---

## Diskussion der Annahmen

Die Berechnung berücksichtigt ausschließlich den hydrostatischen Druck der im Bohrgestänge befindlichen Bohrspülung.

Dynamische Druckspitzen, die beispielsweise durch Pumpenbetrieb oder schnelle Bewegungen des Gestänges entstehen können, werden in der Konzeptphase nicht berücksichtigt.

Sicherheitsbeiwerte werden bewusst nicht auf den Lastfall selbst angewendet, sondern erst bei der Auslegung der jeweiligen Bauteile berücksichtigt. Dadurch wird eine doppelte Berücksichtigung von Sicherheitsreserven vermieden.

---

# A2.1 – Ermittlung der resultierenden Öffnungskraft

## Ziel

Auf Grundlage des Bemessungsdrucks wird die resultierende Öffnungskraft auf die obere Gehäusehälfte bestimmt.

Diese Kraft bildet die Grundlage für die spätere Dimensionierung der Hydraulikzylinder sowie der Scharnierlagerung.

---

## Innendurchmesser der Mud Bucket

Der vorläufige Innendurchmesser ergibt sich aus dem größten Tool-Joint-Durchmesser sowie der beidseitigen radialen Bladderhöhe.

| Parameter | Wert |
|-----------|------:|
| Größter Tool Joint OD | 219,1 mm |
| Radiale Bladderhöhe | 50 mm |

\[
D_i
=
219,1
+
2 \cdot 50
=
319,1\;mm
\]

---

## Projizierte Druckfläche

Für die überschlägige Berechnung wird eine gleichmäßige Druckverteilung auf die projizierte Innenfläche angenommen.

\[
A
=
L_{MB}
\cdot
D_i
\]

\[
A
=
1115
\cdot
319,1
=
355\,797\;mm^2
\]

\[
A
=
0,356\;m^2
\]

---

## Öffnungskraft

Die resultierende Kraft ergibt sich aus

\[
F
=
p
\cdot
A
\]

\[
F
=
0,6
\cdot
355\,797
=
213\,478\;N
\]

\[
F
\approx
214\;kN
\]

---

## Ergebnis

Die gesamte resultierende Öffnungskraft beträgt

> **F = 214 kN**

Diese Kraft wirkt gleichmäßig verteilt auf die obere Gehäusehälfte.

Sie stellt **nicht** die erforderliche Hydraulikzylinderkraft dar.

Für die Dimensionierung der Hydraulikzylinder ist zusätzlich das Momentengleichgewicht um die Scharnierachse zu betrachten.

---

# A3 – Vorbereitung der Dimensionierung der Hydraulikzylinder

## Ziel

Ziel der folgenden Berechnung ist die Ermittlung der erforderlichen Schließkraft der Hydraulikzylinder.

Da die Öffnungskraft nicht direkt in den Zylindern wirkt, erfolgt die Dimensionierung über das statische Momentengleichgewicht.

---

## Berechnungsprinzip

Die aus dem Innendruck resultierende Öffnungskraft greift im Schwerpunkt der projizierten Druckfläche an.

Die Hydraulikzylinder erzeugen ein entgegengesetztes Schließmoment.

Für den Gleichgewichtszustand gilt

\[
\sum M = 0
\]

Hierzu werden im nächsten Entwicklungsschritt die tatsächlichen Hebelarme aus dem CAD-Modell bestimmt.

---

## Erforderliche CAD-Parameter

Für die Berechnung werden folgende geometrische Größen benötigt:

- Lage der Scharnierachse
- Angriffspunkt der resultierenden Druckkraft
- Lage der Zylinderanlenkung
- Öffnungswinkel der Gehäusehälfte
- Anzahl der Hydraulikzylinder
- Einbaulage der Zylinder

Erst nach Vorliegen dieser Daten kann die erforderliche Zylinderkraft berechnet werden.

---

# Zusammenfassung der vorläufigen Auslegungsparameter

| Parameter | Wert | Status |
|-----------|------:|:------:|
| Funktionale Mindestlänge | 1115 mm | Berechnet |
| Größter Tool Joint OD | 219,1 mm | API |
| Vorläufiger Innendurchmesser | 319,1 mm | Berechnet |
| Maximaler Innendruck | 6 bar | Berechnet |
| Projizierte Druckfläche | 0,356 m² | Berechnet |
| Resultierende Öffnungskraft | 214 kN | Berechnet |
| Erforderliche Zylinderkraft | folgt | CAD |

---

# 8.5 Ergebnisse des Kapitels

Im Rahmen der Konzeptentwicklung wurden die wesentlichen geometrischen und mechanischen Randbedingungen der Mud Bucket festgelegt.

Die wichtigsten Ergebnisse lauten:

- Funktionale Mindestlänge der Mud Bucket: **1115 mm**
- Vorläufiger Innendurchmesser: **319 mm**
- Bemessungsdruck: **6 bar**
- Resultierende Öffnungskraft: **214 kN**
- Grundlagen für die Auslegung der Hydraulikzylinder geschaffen

Die dargestellten Berechnungen bilden die Grundlage für die weitere konstruktive Entwicklung sowie die Dimensionierung der tragenden Komponenten.

Die endgültige Auslegung der Hydraulikzylinder erfolgt nach Abschluss der CAD-Konstruktion auf Grundlage der tatsächlichen Hebelverhältnisse.
