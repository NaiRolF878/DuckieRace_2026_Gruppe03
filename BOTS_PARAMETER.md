# Bot-spezifische Parameter

Übersicht aller Parameter, die in den `config/*.json`-Dateien tatsächlich pro
Bot überschrieben werden (Fleet-Blöcke in `follow_lane`/`intersection_handling`).
Alle nicht hier aufgeführten Parameter (z. B. `crop_image`, `red`, `stop_line`,
`evade`, `tag_directions`, ...) sind nirgends bot-spezifisch und gelten für
alle Bots gleich aus `default`.

| Parameter | donald | daisy | tick | track | trick | gustav | dorette | dagobert | daffy | gundel |
|---|---|---|---|---|---|---|---|---|---|---|
| pid.p | 8.0 | 8.0 | 5.5 | 8.0 | 8.0 | 8.0 | 8.0 | 8.0 | 8.0 | 8.0 |
| pid.i | 0.0 | 0.0 | 0.15 | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 |
| pid.d | 6.0 | 6.0 | 2.75 | 6.0 | 6.0 | 6.0 | 6.0 | 6.0 | 6.0 | 6.0 |
| pid.max_vel | – | – | – | **0.2** | – | – | – | – | – | – |
| pid.min_vel | – | – | – | **0.1** | – | – | – | – | – | – |
| white.hl | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| white.hh | 255 | 255 | 255 | 255 | 255 | 255 | 255 | 255 | 255 | 255 |
| white.sl | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| white.sh | 41 | 41 | 41 | 41 | 41 | 41 | 41 | 41 | 41 | 41 |
| white.vl | 161 | 161 | 161 | 161 | 161 | 161 | 161 | 161 | 161 | 161 |
| white.vh | 255 | 255 | 255 | 255 | 255 | 255 | 255 | 255 | 255 | 255 |
| yellow.hl | 15 | 15 | 15 | 15 | 15 | 15 | 15 | 15 | 15 | 15 |
| yellow.hh | 60 | 60 | 60 | 60 | 60 | 60 | 60 | 60 | 60 | 60 |
| yellow.sl | 60 | 60 | 60 | 60 | 60 | 60 | 60 | 60 | 60 | 60 |
| yellow.sh | 255 | 255 | 255 | 255 | 255 | 255 | 255 | 255 | 255 | 255 |
| yellow.vl | 120 | 120 | 120 | 120 | 120 | 120 | 120 | 120 | 120 | 120 |
| yellow.vh | 255 | 255 | 255 | 255 | 255 | 255 | 255 | 255 | 255 | 255 |

