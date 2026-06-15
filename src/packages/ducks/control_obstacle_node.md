# control_obstacle_node.json

Dokumentation der Parameter in `control_obstacle_node.json` für die Node `control_obstacle_node`.

## Zweck

`control_obstacle_node` entscheidet, wie der Duckiebot beim Erkennen eines Hindernisses (Ente) ausweichen soll. Die Node berechnet keinen eigenen Lenkwinkel, sondern gibt einen additiven Offset auf `/obstacle/error_offset` aus, den `control_lane_node` zur normalen Spurregelung hinzufügt.

## Struktur der JSON

Die Datei folgt diesem Schema:

```json
{
  "debug_image_topics": [],
  "parameters": {
    "default": {
      "evade": {
        "active": { "default": 1, "min": 0, "max": 1 },
        "evade_offset": { "default": 0.6, "min": 0.0, "max": 2.0 },
        "oncoming_offset": { "default": 1.0, "min": 0.0, "max": 2.0 },
        "ramp_step": { "default": 0.05, "min": 0.0, "max": 1.0 },
        "evade_hold": { "default": 2.0, "min": 0.0, "max": 10.0 },
        "gap_min_bins": { "default": 6, "min": 1, "max": 40 },
        "edge_min_bins": { "default": 5, "min": 1, "max": 40 }
      }
    }
  }
}
```

## Parameterbeschreibung

### `active`

- Typ: `0` oder `1`
- Bedeutung: Schaltet das Ausweichverhalten ein oder aus.
- `1`: Node gibt bei Erkennung einen Offset aus.
- `0`: Node gibt immer `0` als Offset aus.
- Anwendungsfall: Teste Lane-Following ohne Ausweichen, aber mit aktiver Hindernis-Erkennung.

### `evade_offset`

- Standard: `0.6`
- Bedeutung: Stärke des seitlichen Ausweich-Offsets bei normalem Ausweichen.
- Verhalten: Wird verwendet, wenn ein freier Rand breit genug ist oder eine innere Lücke benutzt wird.
- Effekt: Größerer Wert bedeutet stärkeres Ausweichen nach links oder rechts.

### `oncoming_offset`

- Standard: `1.0`
- Bedeutung: Offset für die Notfall-Strategie, wenn kein ausreichender freier Weg vorhanden ist.
- Verhalten: Führt zu einem stärkeren Links-Offset, um die Ente notfalls auf der Gegenspur zu umfahren.
- Effekt: Erhöht die Größe des Ausweichmanövers in kritischen Situationen.

### `ramp_step`

- Standard: `0.05`
- Bedeutung: Schrittweite für die sanfte Anpassung des aktuellen Offsets in Richtung Ziel-Offset.
- Verhalten: Der Offset wird nicht auf einmal gesetzt, sondern in kleinen Schritten gerampt.
- Effekt: Kleinere Werte führen zu ruhigeren, langsameren Übergängen. Größere Werte machen das Ausweichen und Zurückfahren abrupt.

### `evade_hold`

- Standard: `2.0`
- Bedeutung: Zeit in Sekunden, die das Ausweichmanöver nach der letzten Duck-Erkennung gehalten wird.
- Verhalten: Hält das Ausweichen auch dann aufrecht, wenn die Ente kurzzeitig nicht mehr erkannt wird, weil sie aus dem Bild wandert.
- Effekt: Ein zu kleiner Wert kann dazu führen, dass der Bot zu früh zurück auf die Spur lenkt. Ein zu großer Wert verlängert das Ausweichen unnötig.

### `gap_min_bins`

- Standard: `6`
- Bedeutung: Mindestbreite einer inneren freien Lücke im Erkennungsprofil, damit sie als Ausweichweg genutzt wird.
- Verhalten: Nur Lücken mit mindestens dieser Breite werden als mögliche Durchfahrten angenommen.
- Effekt: Größere Werte reduzieren die Bereitschaft, durch schmale Lücken zu fahren.

### `edge_min_bins`

- Standard: `5`
- Bedeutung: Mindestbreite eines freien Randbereichs (links oder rechts), um den Rand auszuweichen.
- Verhalten: Wenn der erste oder letzte freie Bereich im Profil mindestens so breit ist, wird dieser Rand bevorzugt benutzt.
- Effekt: Höhere Werte machen die Node vorsichtiger gegenüber Randausweichbewegungen.

## Ausweichstrategie

Die Node trifft Entscheidungen in dieser Reihenfolge:

1. Prüft, ob ein linker oder rechter Rand breit genug ist (`edge_min_bins`). Wenn ja, weicht sie außen vorbei.
2. Wenn kein Rand breit genug ist, sucht sie die breiteste innere Lücke, die mindestens `gap_min_bins` breit ist.
3. Wenn kein Platz gefunden wird, wählt sie die Notfallstrategie mit `oncoming_offset`.

Das Ergebnis ist ein beschriebener Ausweichgrund plus ein Ziel-Offset.

## Verhalten des Zustandsautomaten

- `Idle`: Kein Ausweichen aktiv, Offset ist `0`.
- `Evading`: Sobald eine Ente erkannt wurde, rammelt der Offset langsam auf den Zielwert.
- `Returning`: Wenn die Ente weg ist, wird der Offset auf `0` zurückgerampt.

## Hinweis zu JSON-Kommentaren

Standard-JSON erlaubt keine Kommentare. Wenn du trotzdem erklärenden Text in der Datei haben möchtest, kannst du stattdessen ein Feld wie `description` oder `help` hinzufügen.

Beispiel:

```json
{
  "description": "Parameter für control_obstacle_node: Ausweichverhalten, Rampen und Lückenbreiten.",
  "debug_image_topics": [],
  "parameters": { ... }
}
```
