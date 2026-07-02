# 8 Konstruktion und technische Auslegung

## 8.1 Zielsetzung

Nach Auswahl des Vorzugskonzepts erfolgt die konstruktive Auslegung der Mud Bucket.

Ziel dieses Kapitels ist die nachvollziehbare Herleitung der wesentlichen Auslegungsparameter für die Konzeptphase. Die dargestellten Berechnungen bilden die Grundlage für die anschließende CAD-Konstruktion sowie für weiterführende Festigkeits-, Funktions- und Sicherheitsnachweise.

Da sich das System zum Zeitpunkt der Erstellung noch in der Konzeptentwicklung befindet, werden einzelne Randbedingungen zunächst durch begründete Annahmen beschrieben. Diese Werte dienen als Startpunkt für die Konstruktion und werden im weiteren Entwicklungsverlauf durch CAD-Daten, Berechnungen, Versuche oder Lieferanteninformationen verifiziert.

---

# 8.2 Auslegungsrandbedingungen

Die konstruktive Auslegung erfolgt auf Grundlage der in Tabelle 8-1 definierten Randbedingungen.

## Tabelle 8-1: Auslegungsrandbedingungen

| Parameter | Wert | Status | Begründung |
|-----------|------|:------:|------------|
| Einsatzbereich | Wet Tripping | Festgelegt | Projektdefinition |
| Pipe Body | 2⅞" bis 6⅝" Drill Pipe | Festgelegt | Projektdefinition |
| Rohrverbindungen | API Standard Connections | Festgelegt | API Spec 7 |
| Bemessungsfall | Range 3 Double | Festgelegt | Größter Standard-Anwendungsfall |
| Umgebungstemperatur | −20 °C bis +40 °C | Festgelegt | Einsatzgebiet |
| Werkstoff | S355J2 | Festgelegt | Unternehmensstandard |
| Hydraulikdruck | max. 150 bar | Festgelegt | Standardhydraulik |
| Bladderdruck | 2 bar | Vorläufig | Vergleichbare Systeme |
| Maximaler Innendruck | 6 bar | Vorläufig | Berechnung A2 |
| Axiale Bladderbreite | 50 mm | Vorläufig | CAD-Konzept |
| Radiale Bladderhöhe | 50 mm | Vorläufig | Nutgeometrie und Bladderbefestigung |
| Freiraum nach Trennung | 100 mm | Festgelegt | Projektvorgabe |
| Fertigungsreserve | 0 mm | Festgelegt | Funktionale Mindestabmessung |

---

# 8.3 Auslegungsgeometrie

## 8.3.1 Bemessungsfall

Die Mud Bucket wird für Standard Drill Pipes gemäß API 5DP im Größenbereich von 2⅞" bis 6⅝" ausgelegt.

Als Bemessungsfall wird die größte innerhalb dieses Größenbereichs vorkommende Standardverbindung verwendet.

Dadurch wird sichergestellt, dass sämtliche kleineren Rohrgrößen ohne konstruktive Änderungen innerhalb des vorgesehenen Arbeitsbereiches verwendet werden können.

Die geometrischen Kenndaten der Tool Joints werden den Tabellen der API Spec 7 entnommen.

---

## 8.3.2 Bemessungstabelle

Die Tabelle enthält die für die Konzeptphase verwendeten Bemessungswerte.

| Pipe Body | Verbindung | Tool Joint OD DF | Pin Length LP | Box Length LB | Combined Length L | Bemessungsfall |
|-----------|------------|-----------------:|--------------:|--------------:|------------------:|:--------------:|
| 2⅞" | NC31 | 4.125" | 10.50" | 5.75" | 16.25" | |
| 3½" | NC38 | 5.000" | 12.00" | 6.50" | 18.50" | |
| 4½" | NC50 | 6.625" | 14.00" | 7.50" | 21.50" | |
| 5" | NC56 | 7.250" | 15.50" | 8.00" | 23.50" | |
| 5½" | NC61 | 8.250" | 16.50" | 8.50" | 25.00" | |
| 6⅝" | 6⅝ FH | 8.625" | 18.00" | 9.00" | 27.00" | ✓ |

> **Hinweis:** Die dargestellten Werte dienen als Arbeitsgrundlage für die Konzeptphase und werden vor Abschluss der Konstruktion mit den Originalwerten der API Spec 7-2 abgeglichen.

---

# 8.4 Berechnungen

Die nachfolgenden Berechnungen dienen der überschlägigen Auslegung der Mud Bucket während der Konzeptphase.

Ziel ist die Ermittlung belastbarer Ausgangswerte für die CAD-Konstruktion sowie die nachfolgende konstruktive Detailauslegung.

---

# A1 – Ermittlung der funktionalen Mindestlänge der Mud Bucket

## Ziel

Ermittlung der funktional erforderlichen Mindestlänge der Mud Bucket, damit der Trennvorgang einer Drill-Pipe-Verbindung vollständig innerhalb des Gehäuses erfolgen kann.

---

## Berechnungsgrundlage

Als Bemessungsfall wird die größte innerhalb des Auslegungsbereichs vorkommende Standardverbindung verwendet.

Die funktionale Mindestlänge setzt sich aus den folgenden Bereichen zusammen:

1. **Combined Length (L)**  
   Aufnahme der vollständig verschraubten bzw. gelösten Verbindung.

2. **Box Length (LB)**  
   Zusätzlicher Hub, den der Pin zurücklegen muss, um die Box vollständig zu verlassen.

3. **Funktionsfreiraum**  
   Nach vollständigem Trennen der Verbindung wird ein Freiraum von 100 mm vorgesehen. Dieser berücksichtigt den Übergabebereich zwischen Pipe Handler und Top Drive.

4. **Bladder links und rechts**  
   Vorläufige axiale Breite von jeweils 50 mm.

5. **Fertigungsreserve**  
   Für die Ermittlung der funktionalen Mindestlänge wird zunächst keine Fertigungsreserve berücksichtigt.

---

## Berechnungsformel

\[
L_{MB}
=
L
+
LB
+
100
+
2 \cdot B_{Bladder}
+
2 \cdot R_{Fertigung}
\]

mit

| Symbol | Bedeutung |
|---------|-----------|
| \(L_{MB}\) | Funktionale Mindestlänge der Mud Bucket |
| \(L\) | Combined Length |
| \(LB\) | Box Length |
| \(B_{Bladder}\) | Axiale Bladderbreite |
| \(R_{Fertigung}\) | Fertigungsreserve |

---

## Eingangsgrößen

| Parameter | Wert |
|-----------|------:|
| Combined Length (L) | 685,8 mm |
| Box Length (LB) | 228,6 mm |
| Freiraum | 100 mm |
| Bladder links | 50 mm |
| Bladder rechts | 50 mm |
| Fertigungsreserve | 0 mm |

---

## Berechnung

\[
L_{MB}
=
685,8
+
228,6
+
100
+
50
+
50
+
0
=
1114,4 \; \text{mm}
\]

---

## Ergebnis

Die Berechnung ergibt eine funktionale Mindestlänge von

> **L<sub>MB,min</sub> = 1115 mm**

Dieser Wert stellt die konstruktive Untergrenze für die Mud Bucket dar.

Die tatsächliche Gehäuselänge kann im weiteren Entwicklungsverlauf aufgrund konstruktiver Randbedingungen, Fertigungsanforderungen oder Bauraumoptimierungen geringfügig größer ausfallen.

Die funktionale Herleitung der Mindestlänge bleibt hiervon unberührt.

---

## Zusammenfassung A1

| Ergebnis | Wert |
|-----------|------:|
| Funktionale Mindestlänge | **1115 mm** |
| Bemessungsfall | 6⅝" FH |
| Combined Length | 685,8 mm |
| Box Length | 228,6 mm |
| Freiraum | 100 mm |
| Axiale Bladderbreite | 2 × 50 mm |
| Fertigungsreserve | 0 mm |

Die ermittelte Mindestlänge bildet die Grundlage für die weitere konstruktive Auslegung der Mud Bucket und dient insbesondere als Eingangsgröße für die Druck- und Kraftberechnungen der folgenden Abschnitte.