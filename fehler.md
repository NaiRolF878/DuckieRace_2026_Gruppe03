root@duckie3-ThinkPad-L15-Gen-4:~/DuckieRace# launchers/mapping.sh 
[INFO] [1784724774.698293]: [explore_control_node] Bereit. 5 Kanten zu erkunden.
[INFO] [1784724774.830965]: [switch_control_node] Bereit – Zustand: Lane
[INFO] [1784724774.840146]: [graph_state_node] Bereit. Start-Knoten: C
[INFO] [1784724774.849720]: [detect_lane_node] Bereit.
[INFO] [1784724774.877286]: [path_planner_node] Bereit. Delivery-Start: C
[INFO] [1784724774.957674]: Lane error: -0.117 range [-1,1]
[INFO] [1784724775.167416]: [debug_graph_node] Bereit.
[INFO] [1784724775.992987]: Lane error: -0.117 range [-1,1]
[INFO] [1784724777.019080]: Lane error: -0.117 range [-1,1]
[INFO] [1784724778.050528]: Lane error: -0.117 range [-1,1]
[INFO] [1784724779.051252]: Lane error: -0.117 range [-1,1]
[INFO] [1784724780.084410]: Lane error: -0.117 range [-1,1]
[INFO] [1784724780.450163]: [detect_apriltag] Detektor geladen: tagStandard52h13
[INFO] [1784724780.612198]: [detect_apriltag_node] Bereit (Paket: dt_apriltags). Familien: ['tagStandard52h13']  Mapping: {1: ['left', 'straight', 'right'], 2: ['right', 'left'], 3: ['straight', 'left'], 4: ['straight', 'right']}
[WARN] [1784724780.722620]: [apriltag] Tag 2 verworfen (hamming=2 - nur mit Bitfehler-Korrektur dekodiert)
[INFO] [1784724781.117168]: Lane error: -0.117 range [-1,1]
[INFO] [1784724782.118019]: Lane error: -0.117 range [-1,1]
[WARN] [1784724782.753256]: [apriltag] Tag 2 verworfen (hamming=2 - nur mit Bitfehler-Korrektur dekodiert)
[INFO] [1784724783.149798]: Lane error: -0.117 range [-1,1]
[INFO] [1784724784.154319]: Lane error: -0.117 range [-1,1]
[INFO] [1784724784.562401]: [control_lane_node] Bereit. Warte auf Spurversatz ...
[INFO] [1784724784.619139]: [control_intersection_node] Bereit.
[WARN] [1784724784.785769]: [apriltag] Tag 2 verworfen (hamming=2 - nur mit Bitfehler-Korrektur dekodiert)
[INFO] [1784724785.183324]: Lane error: -0.083 range [-1,1]
[INFO] [1784724785.350368]: [switch] Kreuzung (Linie+Tag) -> STOPPING | erlaubte Richtungen (Graph-Fallback): ['straight', 'right']
[INFO] [1784724785.362227]: [switch] -> Stopping
[INFO] [1784724786.183939]: Lane error: 0.025 range [-1,1]
[WARN] [1784724786.882200]: [apriltag] Tag 3 verworfen (hamming=1 - nur mit Bitfehler-Korrektur dekodiert)
[INFO] [1784724787.215382]: Lane error: 0.026 range [-1,1]
[INFO] [1784724787.458739]: [switch] Richtung bestaetigt: right (aus ['straight', 'right']) -> TURNING
[INFO] [1784724787.469528]: [switch] -> Turning
[INFO] [1784724787.469867]: [graph_state] C --1(right)--> A
[INFO] [1784724787.469877]: [control_intersection] Starte Sequenz: right
[INFO] [1784724788.216533]: Lane error: 0.000 range [-1,1]
[INFO] [1784724789.249813]: Lane error: 0.000 range [-1,1]
[INFO] [1784724790.029956]: [control_intersection] Sequenz fertig -> turn_done
[INFO] [1784724790.058709]: [switch] Turning fertig -> LANE
[INFO] [1784724790.077841]: [switch] -> Lane
[INFO] [1784724790.090652]: [detect_apriltag] Neue Kante – Tag-Gedaechtnis zurueckgesetzt
[INFO] [1784724790.254037]: Lane error: -0.351 range [-1,1]
[INFO] [1784724790.858327]: [graph_state] Tor 6 -> Kante {'node': 'C', 'tag': '1'}
[INFO] [1784724791.281389]: Lane error: -0.147 range [-1,1]
[INFO] [1784724792.285959]: Lane error: -0.123 range [-1,1]
[INFO] [1784724793.286558]: Lane error: 0.085 range [-1,1]
[INFO] [1784724794.289771]: Lane error: 0.081 range [-1,1]
[INFO] [1784724795.314753]: Lane error: -0.242 range [-1,1]
[INFO] [1784724796.347777]: Lane error: -0.522 range [-1,1]
[WARN] [1784724796.866302]: [graph_state] Live-Tag (1) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (3) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724796.966105]: [graph_state] Live-Tag (1) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (3) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724797.066024]: [graph_state] Live-Tag (1) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (3) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724797.166410]: [graph_state] Live-Tag (1) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (3) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724797.266264]: [graph_state] Live-Tag (1) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (3) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724797.366199]: [graph_state] Live-Tag (1) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (3) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[INFO] [1784724797.381405]: Lane error: -0.230 range [-1,1]
[WARN] [1784724797.466351]: [graph_state] Live-Tag (1) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (3) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724797.566385]: [graph_state] Live-Tag (1) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (3) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724797.666125]: [graph_state] Live-Tag (1) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (3) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724797.765971]: [graph_state] Live-Tag (1) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (3) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724797.866192]: [graph_state] Live-Tag (1) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (3) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724797.966067]: [graph_state] Live-Tag (1) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (3) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724798.066111]: [graph_state] Live-Tag (1) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (3) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724798.165910]: [graph_state] Live-Tag (1) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (3) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724798.266178]: [graph_state] Live-Tag (1) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (3) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724798.366209]: [graph_state] Live-Tag (1) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (3) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[INFO] [1784724798.413958]: Lane error: -0.091 range [-1,1]
[WARN] [1784724798.466169]: [graph_state] Live-Tag (1) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (3) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724798.566277]: [graph_state] Live-Tag (1) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (3) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724798.666024]: [graph_state] Live-Tag (1) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (3) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724798.766284]: [graph_state] Live-Tag (1) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (3) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724798.866059]: [graph_state] Live-Tag (1) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (3) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724798.967066]: [graph_state] Live-Tag (1) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (3) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724799.066048]: [graph_state] Live-Tag (1) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (3) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[INFO] [1784724799.113003]: [switch] Kreuzung (Linie+Tag) -> STOPPING | erlaubte Richtungen (Live-Tag): ['left', 'straight', 'right']
[INFO] [1784724799.123671]: [switch] -> Stopping
[WARN] [1784724799.166213]: [graph_state] Live-Tag (1) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (3) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724799.266024]: [graph_state] Live-Tag (1) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (3) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724799.366278]: [graph_state] Live-Tag (1) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (3) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[INFO] [1784724799.448305]: Lane error: -0.042 range [-1,1]
[WARN] [1784724799.466105]: [graph_state] Live-Tag (1) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (3) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724799.566498]: [graph_state] Live-Tag (1) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (3) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724799.666018]: [graph_state] Live-Tag (1) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (3) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724799.766323]: [graph_state] Live-Tag (1) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (3) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724799.866127]: [graph_state] Live-Tag (1) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (3) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724799.966228]: [graph_state] Live-Tag (1) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (3) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724800.066081]: [graph_state] Live-Tag (1) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (3) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724800.166197]: [graph_state] Live-Tag (1) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (3) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724800.266154]: [graph_state] Live-Tag (1) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (3) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724800.366385]: [graph_state] Live-Tag (1) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (3) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724800.466224]: [graph_state] Live-Tag (1) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (3) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[INFO] [1784724800.481756]: Lane error: -0.059 range [-1,1]
[WARN] [1784724800.566296]: [graph_state] Live-Tag (1) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (3) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724800.666409]: [graph_state] Live-Tag (1) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (3) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724800.766584]: [graph_state] Live-Tag (1) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (3) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724800.866399]: [graph_state] Live-Tag (1) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (3) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724800.966489]: [graph_state] Live-Tag (1) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (3) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724801.066315]: [graph_state] Live-Tag (1) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (3) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[INFO] [1784724801.158780]: [switch] Richtung bestaetigt: straight (aus ['left', 'straight', 'right']) -> TURNING
[WARN] [1784724801.166149]: [graph_state] Live-Tag (1) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (3) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[INFO] [1784724801.174432]: [switch] -> Turning
[WARN] [1784724801.174487]: [graph_state] Live-Tag (1) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (3) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[INFO] [1784724801.174697]: [control_intersection] Starte Sequenz: straight
[INFO] [1784724801.198636]: [graph_state] A --1(straight)--> B
[INFO] [1784724801.515032]: Lane error: 0.083 range [-1,1]
[INFO] [1784724802.550149]: Lane error: 0.000 range [-1,1]
[INFO] [1784724803.577067]: Lane error: 0.147 range [-1,1]
[INFO] [1784724803.730011]: [control_intersection] Sequenz fertig -> turn_done
[INFO] [1784724803.758726]: [switch] Turning fertig -> LANE
[INFO] [1784724803.774126]: [switch] -> Lane
[INFO] [1784724803.790055]: [detect_apriltag] Neue Kante – Tag-Gedaechtnis zurueckgesetzt
[INFO] [1784724803.812550]: [graph_state] Tor 10 -> Kante {'node': 'A', 'tag': '1'}
[INFO] [1784724804.610278]: Lane error: 0.257 range [-1,1]
[WARN] [1784724805.278998]: [apriltag] Tag 2 verworfen (hamming=1 - nur mit Bitfehler-Korrektur dekodiert)
[WARN] [1784724805.466190]: [graph_state] Live-Tag (2) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (1) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724805.566572]: [graph_state] Live-Tag (2) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (1) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[INFO] [1784724805.611807]: Lane error: 0.037 range [-1,1]
[WARN] [1784724805.666138]: [graph_state] Live-Tag (2) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (1) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724805.766374]: [graph_state] Live-Tag (2) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (1) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724805.866146]: [graph_state] Live-Tag (2) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (1) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724805.966568]: [graph_state] Live-Tag (2) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (1) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724806.066142]: [graph_state] Live-Tag (2) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (1) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724806.166272]: [graph_state] Live-Tag (2) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (1) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724806.266076]: [graph_state] Live-Tag (2) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (1) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724806.366432]: [graph_state] Live-Tag (2) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (1) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724806.466080]: [graph_state] Live-Tag (2) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (1) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724806.566469]: [graph_state] Live-Tag (2) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (1) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[INFO] [1784724806.644320]: Lane error: 0.027 range [-1,1]
[WARN] [1784724806.666188]: [graph_state] Live-Tag (2) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (1) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724806.766561]: [graph_state] Live-Tag (2) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (1) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724806.866344]: [graph_state] Live-Tag (2) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (1) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[INFO] [1784724806.913248]: [switch] Kreuzung (Linie+Tag) -> STOPPING | erlaubte Richtungen (Live-Tag): ['right', 'left']
[INFO] [1784724806.924500]: [switch] -> Stopping
[WARN] [1784724806.966364]: [graph_state] Live-Tag (2) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (1) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724807.066294]: [graph_state] Live-Tag (2) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (1) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724807.166283]: [graph_state] Live-Tag (2) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (1) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724807.266101]: [graph_state] Live-Tag (2) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (1) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724807.366244]: [graph_state] Live-Tag (2) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (1) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724807.465983]: [graph_state] Live-Tag (2) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (1) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724807.566658]: [graph_state] Live-Tag (2) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (1) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[INFO] [1784724807.645927]: Lane error: -0.133 range [-1,1]
[WARN] [1784724807.666092]: [graph_state] Live-Tag (2) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (1) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724807.766466]: [graph_state] Live-Tag (2) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (1) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724807.866132]: [graph_state] Live-Tag (2) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (1) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724807.966098]: [graph_state] Live-Tag (2) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (1) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724808.066165]: [graph_state] Live-Tag (2) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (1) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724808.166311]: [graph_state] Live-Tag (2) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (1) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724808.266719]: [graph_state] Live-Tag (2) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (1) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724808.366149]: [graph_state] Live-Tag (2) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (1) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724808.466500]: [graph_state] Live-Tag (2) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (1) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724808.566227]: [graph_state] Live-Tag (2) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (1) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724808.666556]: [graph_state] Live-Tag (2) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (1) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[INFO] [1784724808.678480]: Lane error: -0.135 range [-1,1]
[WARN] [1784724808.766366]: [graph_state] Live-Tag (2) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (1) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[WARN] [1784724808.866574]: [graph_state] Live-Tag (2) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (1) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[INFO] [1784724808.958825]: [switch] Richtung bestaetigt: right (aus ['right', 'left']) -> TURNING
[WARN] [1784724808.966072]: [graph_state] Live-Tag (2) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (1) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[INFO] [1784724808.972970]: [switch] -> Turning
[WARN] [1784724808.973217]: [graph_state] Live-Tag (2) widerspricht der aus der letzten Abbiegung vorhergesagten Einfahrt (1) - vertraue der Vorhersage (z.B. gegenueberliegender statt tatsaechlicher Einfahrt-Tag gelesen).
[INFO] [1784724808.973359]: [control_intersection] Starte Sequenz: right
[INFO] [1784724808.995911]: [graph_state] B --2(right)--> A
[INFO] [1784724809.710876]: Lane error: 0.000 range [-1,1]
[INFO] [1784724810.744061]: Lane error: 0.000 range [-1,1]
[INFO] [1784724811.529959]: [control_intersection] Sequenz fertig -> turn_done
[INFO] [1784724811.558724]: [switch] Turning fertig -> LANE
[INFO] [1784724811.573675]: [switch] -> Lane
[INFO] [1784724811.585373]: [detect_apriltag] Neue Kante – Tag-Gedaechtnis zurueckgesetzt
[INFO] [1784724811.610890]: [graph_state] Tor 8 -> Kante {'node': 'B', 'tag': '2'}
[INFO] [1784724811.744761]: Lane error: -0.163 range [-1,1]
[INFO] [1784724812.775743]: Lane error: -0.045 range [-1,1]
[INFO] [1784724813.777689]: Lane error: -0.009 range [-1,1]
[INFO] [1784724814.778796]: Lane error: -0.262 range [-1,1]
[WARN] [1784724815.146077]: [apriltag] Tag 4 verworfen (hamming=2 - nur mit Bitfehler-Korrektur dekodiert)
[INFO] [1784724815.809970]: Lane error: 0.039 range [-1,1]
[INFO] [1784724816.811792]: Lane error: -0.114 range [-1,1]
[WARN] [1784724817.811812]: [apriltag] Tag 4 verworfen (hamming=1 - nur mit Bitfehler-Korrektur dekodiert)
[INFO] [1784724817.812701]: Lane error: 0.000 range [-1,1]
[INFO] [1784724817.911773]: [switch] Kreuzung (Linie+Tag) -> STOPPING | erlaubte Richtungen (Live-Tag): ['straight', 'right']
[INFO] [1784724817.922490]: [switch] -> Stopping
[INFO] [1784724818.844735]: Lane error: 0.000 range [-1,1]
[INFO] [1784724819.845403]: Lane error: -0.272 range [-1,1]
[INFO] [1784724819.958788]: [switch] Richtung bestaetigt: straight (aus ['straight', 'right']) -> TURNING
[INFO] [1784724819.973476]: [switch] -> Turning
[INFO] [1784724819.973523]: [graph_state] A --2(straight)--> C
[INFO] [1784724819.973766]: [control_intersection] Starte Sequenz: straight
[INFO] [1784724820.848528]: Lane error: 0.000 range [-1,1]
[INFO] [1784724821.862976]: Lane error: 0.000 range [-1,1]
[INFO] [1784724822.530004]: [control_intersection] Sequenz fertig -> turn_done
[INFO] [1784724822.558742]: [switch] Turning fertig -> LANE
[INFO] [1784724822.628015]: [switch] -> Lane
[INFO] [1784724822.635652]: [switch] Kreuzung (Linie+Tag) -> STOPPING | erlaubte Richtungen (Graph-Fallback): ['right', 'left']
[INFO] [1784724822.651801]: [detect_apriltag] Neue Kante – Tag-Gedaechtnis zurueckgesetzt
[INFO] [1784724822.675269]: [switch] -> Stopping
[INFO] [1784724822.878193]: Lane error: -0.538 range [-1,1]
[INFO] [1784724823.914582]: Lane error: -0.500 range [-1,1]
[WARN] [1784724824.758855]: [switch] Keine gueltige next_direction ('straight') in erlaubten Richtungen ['right', 'left'] - bleibe in STOPPING
[INFO] [1784724824.945045]: Lane error: -0.500 range [-1,1]
[WARN] [1784724825.758885]: [switch] Keine gueltige next_direction ('straight') in erlaubten Richtungen ['right', 'left'] - bleibe in STOPPING
[INFO] [1784724825.945382]: Lane error: -0.500 range [-1,1]
[WARN] [1784724826.858878]: [switch] Keine gueltige next_direction ('straight') in erlaubten Richtungen ['right', 'left'] - bleibe in STOPPING
[INFO] [1784724826.975728]: Lane error: -0.500 range [-1,1]
[WARN] [1784724827.958903]: [switch] Keine gueltige next_direction ('straight') in erlaubten Richtungen ['right', 'left'] - bleibe in STOPPING
[INFO] [1784724827.976533]: Lane error: -0.500 range [-1,1]
[INFO] [1784724828.977401]: Lane error: -0.500 range [-1,1]
[WARN] [1784724829.059003]: [switch] Keine gueltige next_direction ('straight') in erlaubten Richtungen ['right', 'left'] - bleibe in STOPPING
[INFO] [1784724829.978627]: Lane error: -0.500 range [-1,1]
[WARN] [1784724830.158858]: [switch] Keine gueltige next_direction ('straight') in erlaubten Richtungen ['right', 'left'] - bleibe in STOPPING
[INFO] [1784724830.978806]: Lane error: -0.500 range [-1,1]
[WARN] [1784724831.258785]: [switch] Keine gueltige next_direction ('straight') in erlaubten Richtungen ['right', 'left'] - bleibe in STOPPING
[INFO] [1784724831.979188]: Lane error: -0.500 range [-1,1]
[WARN] [1784724832.258890]: [switch] Keine gueltige next_direction ('straight') in erlaubten Richtungen ['right', 'left'] - bleibe in STOPPING
[INFO] [1784724833.009890]: Lane error: -0.500 range [-1,1]
[WARN] [1784724833.259004]: [switch] Keine gueltige next_direction ('straight') in erlaubten Richtungen ['right', 'left'] - bleibe in STOPPING
[INFO] [1784724834.011613]: Lane error: -0.500 range [-1,1]
