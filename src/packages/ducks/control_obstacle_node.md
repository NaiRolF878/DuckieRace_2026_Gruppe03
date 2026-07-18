# control_obstacle_node.json

Dokumentation der Parameter in `control_obstacle_node.json` für die Node `control_obstacle_node`.

> Für die Gesamtarchitektur, den Zustandsautomaten und die Ausweichrichtung-
> Berechnung siehe **[CHALLENGE3_DOKU.md](CHALLENGE3_DOKU.md)**. Diese Datei
> beschreibt nur die einzelnen JSON-Parameter.

## Zweck

`control_obstacle_node` entscheidet, wie der Duckiebot beim Erkennen eines Hindernisses (Ente oder gelbe Linie) ausweichen soll. Die Node berechnet keinen eigenen Lenkwinkel, sondern gibt einen additiven Offset auf `/obstacle/error_offset` aus, den `control_lane_node` zur normalen Spurregelung hinzufügt.

## Struktur der JSON

```json
{
  "debug_image_topics": [],
  "parameters": {
    "default": {
      "evade": {
        "active":               { "default": 1,    "min": 0,   "max": 1    },
        "evade_offset":         { "default": 0.6,  "min": 0.0, "max": 2.0  },
        "evade_offset_min":     { "default": 0.25, "min": 0.0, "max": 2.0  },
        "nachlauf_secs":        { "default": 1.5,  "min": 0.0, "max": 10.0 },
        "evade_timeout_secs":   { "default": 5.0,  "min": 1.0, "max": 30.0 },
        "return_threshold":     { "default": 0.25, "min": 0.0, "max": 1.0  },
        "return_stable_frames": { "default": 5,    "min": 1,   "max": 30   },
        "return_omega":         { "default": 0.5,  "min": 0.0, "max": 3.0  },
        "wait_timeout_secs":    { "default": 3.0,  "min": 0.5, "max": 15.0 }
      }
    }
  }
}
```

## Parameterbeschreibung

### `active`
- Typ: `0` oder `1`
- `1`: Node gibt bei Erkennung einen Offset aus.
- `0`: Node gibt immer `0` als Offset aus (Erkennung in `detect_lane_node` läuft trotzdem weiter).
- Anwendungsfall: Lane-Following isoliert testen, ohne Ausweichen.

### `evade_offset`
- Standard: `0.6`
- Maximale Stärke des seitlichen Ausweich-Offsets – wird verwendet, wenn die
  gefundene freie Lücke am Rand des Fahrkorridors liegt.
- Größerer Wert → stärkeres Ausweichen nach links oder rechts.

### `evade_offset_min`
- Standard: `0.25`
- Minimale Stärke des Offsets – wird verwendet, wenn die freie Lücke nahe der
  Korridormitte liegt. Verhindert ein zu schwaches Ausweichen bei einer knapp
  mittigen Lücke.

### `nachlauf_secs`
- Standard: `1.5`
- Zeit in Sekunden, die der Ausweich-Offset nach der letzten Objekt-Sichtung
  noch gehalten wird (Zustand `PASS`), bevor die Encoder-Rückkehr beginnt.
- Zu kurz → Bot kehrt zurück, bevor er sicher am Hindernis vorbei ist.

### `evade_timeout_secs`
- Standard: `5.0`
- Maximale Zeit im Zustand `EVADE`, bevor in `WAIT` (Anhalten) gewechselt wird.

### `return_threshold`
- Standard: `0.25`
- Schwellwert für den Spurversatz (`/detect/lane`), unterhalb dessen die
  Rückkehr als abgeschlossen gilt (Kamera sieht die weiße Linie wieder).

### `return_stable_frames`
- Standard: `5`
- Anzahl aufeinanderfolgender Frames, die der Spurversatz unter
  `return_threshold` bleiben muss, bevor die Rückkehr als fertig gilt
  (Entprellung gegen kurze Fehlerkennungen).

### `return_omega`
- Standard: `0.5` rad/s
- Drehrate während der Encoder-gestützten Rückkehr (Zustand `RETURN`).

### `wait_timeout_secs`
- Standard: `3.0`
- Maximale Wartezeit im Zustand `WAIT`, bevor trotz weiterhin belegter Zone
  erzwungen mit `PASS` fortgefahren wird (Sicherung gegen dauerhafte
  Fehlerkennung, siehe `konzept_challenge3.md` Abschnitt 9).

## Ausweichstrategie (Kurzfassung)

1. `control_obstacle_node` liest aus `/detect/corridor_occupancy` (`[links_frei,
   rechts_frei]`, exakter Pixel-Anteil der Korridorbreite, von
   `detect_lane_node` berechnet) den freien Abstand vom linken bzw. rechten
   Korridorrand bis zum nächstgelegenen Hindernis und wählt die Seite mit mehr
   Abstand (nicht die breiteste Lücke ZWISCHEN zwei Hindernissen – kein
   Durchquetschen).
2. Der Offset zeigt zur Mitte dieser freien Seite; die Stärke skaliert zwischen
   `evade_offset_min` und `evade_offset`, je nachdem wie breit diese freie Seite
   relativ zur Korridorbreite ist.
3. Ist kein Profil vorhanden oder der Korridor komplett belegt (keine Lücke
   gefunden), fällt die Node auf die einfachere `duck_x`-Heuristik zurück
   (Objekt rechts → links ausweichen, sonst rechts).

Details zum Zustandsautomaten (`IDLE → EVADE → [WAIT] → PASS → RETURN`) stehen
in `CHALLENGE3_DOKU.md`, Abschnitt 5.

## Hinweis zu JSON-Kommentaren

Standard-JSON erlaubt keine Kommentare. Wenn du trotzdem erklärenden Text in der Datei haben möchtest, kannst du stattdessen ein Feld wie `description` oder `help` hinzufügen.
