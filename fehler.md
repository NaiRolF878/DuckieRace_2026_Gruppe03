root@duckie3-ThinkPad-L15-Gen-4:~/DuckieRace# launchers/mapping.sh 
[INFO] [1784730140.054177]: [path_planner] Lade mapping_node.json von: /root/DuckieRace/src/packages/mapping/config/mapping_node.json
[INFO] [1784730140.064879]: [graph_state] Lade mapping_node.json von: /root/DuckieRace/src/packages/mapping/config/mapping_node.json
[INFO] [1784730140.336443]: [detect_lane_node] Bereit.
[INFO] [1784730140.444224]: [explore_control_node] Bereit. 5 Kanten zu erkunden.
[INFO] [1784730140.448892]: [switch_control_node] Bereit – Zustand: Lane
[INFO] [1784730140.462735]: Lane error: -0.050 range [-1,1]
[INFO] [1784730140.583237]: [graph_state_node] Bereit. Start-Knoten: C
[INFO] [1784730140.606189]: [path_planner_node] Bereit. Delivery-Start: C
[INFO] [1784730140.707096]: [debug_graph_node] Bereit.
[INFO] [1784730141.467030]: Lane error: -0.050 range [-1,1]
[INFO] [1784730142.469004]: Lane error: -0.050 range [-1,1]
[INFO] [1784730143.502749]: Lane error: -0.050 range [-1,1]
[INFO] [1784730144.533201]: Lane error: -0.050 range [-1,1]
[INFO] [1784730145.101215]: [detect_apriltag] Detektor geladen: tagStandard52h13
[INFO] [1784730145.279080]: [detect_apriltag_node] Bereit (Paket: dt_apriltags). Familien: ['tagStandard52h13']  Mapping: {1: ['left', 'straight', 'right'], 2: ['right', 'left'], 3: ['straight', 'left'], 4: ['straight', 'right']}
[INFO] [1784730145.534635]: Lane error: -0.050 range [-1,1]
[INFO] [1784730146.538318]: Lane error: -0.050 range [-1,1]
[INFO] [1784730147.566840]: Lane error: -0.050 range [-1,1]
[INFO] [1784730148.567349]: Lane error: -0.050 range [-1,1]
[INFO] [1784730149.567421]: Lane error: -0.050 range [-1,1]
[INFO] [1784730149.898978]: [control_lane_node] Bereit. Warte auf Spurversatz ...
[INFO] [1784730149.962068]: [control_intersection_node] Bereit.
[INFO] [1784730150.572310]: Lane error: -0.015 range [-1,1]
[WARN] [1784730150.700944]: [apriltag] Tag 4 verworfen (hamming=1 - nur mit Bitfehler-Korrektur dekodiert)
[INFO] [1784730151.597429]: Lane error: -0.002 range [-1,1]
[INFO] [1784730151.611920]: [switch] Kreuzung (Linie+Graph) -> STOPPING | erlaubte Richtungen: ['straight', 'right']
[INFO] [1784730151.627445]: [switch] -> Stopping
[INFO] [1784730152.600073]: Lane error: 0.170 range [-1,1]
[INFO] [1784730153.632365]: Lane error: 0.170 range [-1,1]
[INFO] [1784730153.670044]: [switch] Richtung: right (aus Planung; Graph erlaubt: ['straight', 'right']) -> TURNING
[INFO] [1784730153.683501]: [switch] -> Turning
[INFO] [1784730153.683640]: [graph_state] C --1(right)--> A
[INFO] [1784730153.683739]: [control_intersection] Starte Sequenz: right
[INFO] [1784730154.667608]: Lane error: -0.768 range [-1,1]
[INFO] [1784730155.697701]: Lane error: -0.597 range [-1,1]
[INFO] [1784730156.280432]: [control_intersection] Sequenz fertig -> turn_done
[INFO] [1784730156.370085]: [switch] Turning fertig -> LANE
[INFO] [1784730156.384168]: [switch] -> Lane
[INFO] [1784730156.399926]: [detect_apriltag] Neue Kante – Tag-Gedaechtnis zurueckgesetzt
[INFO] [1784730156.733439]: Lane error: 0.182 range [-1,1]
[WARN] [1784730157.364305]: [apriltag] Tag 1 verworfen (hamming=2 - nur mit Bitfehler-Korrektur dekodiert)
[INFO] [1784730157.739165]: Lane error: 0.007 range [-1,1]
[INFO] [1784730158.235961]: [switch] Kreuzung (Linie+Graph) -> STOPPING | erlaubte Richtungen: ['straight', 'left']
[INFO] [1784730158.251539]: [switch] -> Stopping
[INFO] [1784730158.763374]: Lane error: 0.042 range [-1,1]
[INFO] [1784730159.765385]: Lane error: 0.041 range [-1,1]
[INFO] [1784730160.270074]: [switch] Richtung: straight (aus Planung; Graph erlaubt: ['straight', 'left']) -> TURNING
[INFO] [1784730160.285710]: [switch] -> Turning
[INFO] [1784730160.285947]: [control_intersection] Starte Sequenz: straight
[INFO] [1784730160.285968]: [graph_state] A --1(straight)--> B
[INFO] [1784730160.799702]: Lane error: 0.060 range [-1,1]
[INFO] [1784730161.832146]: Lane error: 0.000 range [-1,1]
[INFO] [1784730162.862525]: Lane error: 0.274 range [-1,1]
[INFO] [1784730162.880352]: [control_intersection] Sequenz fertig -> turn_done
[INFO] [1784730162.970083]: [switch] Turning fertig -> LANE
[INFO] [1784730162.982170]: [switch] -> Lane
[INFO] [1784730162.999784]: [detect_apriltag] Neue Kante – Tag-Gedaechtnis zurueckgesetzt
[INFO] [1784730163.863617]: Lane error: 0.041 range [-1,1]
[INFO] [1784730164.895312]: Lane error: 0.047 range [-1,1]
[INFO] [1784730165.929380]: Lane error: -0.135 range [-1,1]
[INFO] [1784730166.929617]: Lane error: -0.185 range [-1,1]
[INFO] [1784730167.960122]: Lane error: 0.015 range [-1,1]
[INFO] [1784730168.993831]: Lane error: -0.006 range [-1,1]
[INFO] [1784730169.994082]: Lane error: -0.292 range [-1,1]
[WARN] [1784730170.727919]: [apriltag] Tag 4 verworfen (hamming=1 - nur mit Bitfehler-Korrektur dekodiert)
[INFO] [1784730171.026808]: Lane error: -0.188 range [-1,1]
[INFO] [1784730172.063233]: Lane error: -0.025 range [-1,1]
[INFO] [1784730172.727909]: [switch] Kreuzung (Linie+Graph) -> STOPPING | erlaubte Richtungen: ['left', 'straight', 'right']
[INFO] [1784730172.745683]: [switch] -> Stopping
[INFO] [1784730173.096042]: Lane error: 0.027 range [-1,1]
[INFO] [1784730174.129249]: Lane error: 0.025 range [-1,1]
[INFO] [1784730174.770068]: [switch] Richtung: right (aus Planung; Graph erlaubt: ['left', 'straight', 'right']) -> TURNING
[INFO] [1784730174.781531]: [switch] -> Turning
[INFO] [1784730174.781805]: [graph_state] B --2(right)--> A
[INFO] [1784730174.781963]: [control_intersection] Starte Sequenz: right
[INFO] [1784730175.136876]: Lane error: 0.025 range [-1,1]
[INFO] [1784730176.161518]: Lane error: -0.755 range [-1,1]
[INFO] [1784730177.194612]: Lane error: -0.184 range [-1,1]
[INFO] [1784730177.380443]: [control_intersection] Sequenz fertig -> turn_done
[INFO] [1784730177.470147]: [switch] Turning fertig -> LANE
[INFO] [1784730177.482170]: [switch] -> Lane
[INFO] [1784730177.502928]: [detect_apriltag] Neue Kante – Tag-Gedaechtnis zurueckgesetzt
[WARN] [1784730177.593604]: [apriltag] Tag 1 verworfen (hamming=1 - nur mit Bitfehler-Korrektur dekodiert)
[INFO] [1784730178.225965]: Lane error: 0.150 range [-1,1]
[INFO] [1784730179.227403]: Lane error: -0.025 range [-1,1]
[INFO] [1784730179.430640]: [switch] Kreuzung (Linie+Graph) -> STOPPING | erlaubte Richtungen: ['straight', 'right']
[INFO] [1784730179.445657]: [switch] -> Stopping
[INFO] [1784730180.263224]: Lane error: 0.015 range [-1,1]
[INFO] [1784730181.294702]: Lane error: 0.015 range [-1,1]
[INFO] [1784730181.470128]: [switch] Richtung: straight (aus Planung; Graph erlaubt: ['straight', 'right']) -> TURNING
[INFO] [1784730181.484600]: [switch] -> Turning
[INFO] [1784730181.484728]: [graph_state] A --2(straight)--> C
[INFO] [1784730181.484846]: [control_intersection] Starte Sequenz: straight
[INFO] [1784730182.301299]: Lane error: 0.000 range [-1,1]
[INFO] [1784730183.329461]: Lane error: 0.000 range [-1,1]
[INFO] [1784730184.080485]: [control_intersection] Sequenz fertig -> turn_done
[INFO] [1784730184.170397]: [switch] Turning fertig -> LANE
[INFO] [1784730184.182488]: [switch] -> Lane
[INFO] [1784730184.200609]: [detect_apriltag] Neue Kante – Tag-Gedaechtnis zurueckgesetzt
[INFO] [1784730184.361911]: Lane error: 0.042 range [-1,1]
[INFO] [1784730185.391236]: Lane error: 0.046 range [-1,1]
[INFO] [1784730186.391861]: Lane error: 0.020 range [-1,1]
[INFO] [1784730187.393710]: Lane error: 0.005 range [-1,1]
[INFO] [1784730188.424032]: Lane error: 0.210 range [-1,1]
[INFO] [1784730189.425611]: Lane error: 0.267 range [-1,1]
[INFO] [1784730190.426023]: Lane error: 0.219 range [-1,1]
[INFO] [1784730191.458919]: Lane error: 0.206 range [-1,1]
[INFO] [1784730192.490043]: Lane error: 0.229 range [-1,1]
[INFO] [1784730193.523685]: Lane error: 0.198 range [-1,1]
[WARN] [1784730194.426090]: [apriltag] Tag 3 verworfen (hamming=2 - nur mit Bitfehler-Korrektur dekodiert)
[INFO] [1784730194.556246]: Lane error: -0.105 range [-1,1]
[INFO] [1784730195.589130]: Lane error: -0.135 range [-1,1]
[INFO] [1784730196.589382]: Lane error: 0.105 range [-1,1]
[INFO] [1784730197.589501]: Lane error: 0.095 range [-1,1]
[WARN] [1784730198.290761]: [apriltag] Tag 3 verworfen (hamming=1 - nur mit Bitfehler-Korrektur dekodiert)
[INFO] [1784730198.290853]: [switch] Kreuzung (Linie+Graph) -> STOPPING | erlaubte Richtungen: ['right', 'left']
[INFO] [1784730198.311087]: [switch] -> Stopping
[INFO] [1784730198.624545]: Lane error: 0.249 range [-1,1]
[INFO] [1784730199.655854]: Lane error: 0.250 range [-1,1]
[INFO] [1784730200.370101]: [switch] Richtung: straight (aus Planung; Graph erlaubt: ['right', 'left']) -> TURNING
[INFO] [1784730200.388395]: [switch] -> Turning
[INFO] [1784730200.388729]: [graph_state] C --4(straight)--> B
[INFO] [1784730200.389082]: [control_intersection] Starte Sequenz: straight
[INFO] [1784730200.688161]: Lane error: 0.257 range [-1,1]
[WARN] [1784730200.888335]: [apriltag] Tag 3 verworfen (hamming=1 - nur mit Bitfehler-Korrektur dekodiert)
[INFO] [1784730201.688190]: Lane error: 0.400 range [-1,1]
[INFO] [1784730202.721780]: Lane error: 0.625 range [-1,1]
[INFO] [1784730202.980382]: [control_intersection] Sequenz fertig -> turn_done
[INFO] [1784730203.070135]: [switch] Turning fertig -> LANE
[INFO] [1784730203.083345]: [switch] -> Lane
[INFO] [1784730203.103354]: [detect_apriltag] Neue Kante – Tag-Gedaechtnis zurueckgesetzt
[INFO] [1784730203.754701]: Lane error: 0.368 range [-1,1]
[INFO] [1784730204.788654]: Lane error: -0.048 range [-1,1]
[INFO] [1784730205.820337]: Lane error: -0.020 range [-1,1]
[INFO] [1784730206.854976]: Lane error: 0.019 range [-1,1]
[INFO] [1784730207.885260]: Lane error: 0.211 range [-1,1]
[INFO] [1784730208.886032]: Lane error: 0.199 range [-1,1]
[INFO] [1784730209.888361]: Lane error: 0.228 range [-1,1]
[INFO] [1784730210.920073]: Lane error: 0.130 range [-1,1]
[WARN] [1784730211.053655]: [apriltag] Tag 3 verworfen (hamming=1 - nur mit Bitfehler-Korrektur dekodiert)
[INFO] [1784730211.953715]: Lane error: 0.111 range [-1,1]
[INFO] [1784730212.655050]: [switch] Kreuzung (Linie+Graph) -> STOPPING | erlaubte Richtungen: ['straight', 'left']
[INFO] [1784730212.671180]: [switch] -> Stopping
[INFO] [1784730212.776076]: [explore_control] Exploration abgeschlossen - alle Kanten besucht, Bot steht
[INFO] [1784730212.954082]: Lane error: 0.085 range [-1,1]
[WARN] [1784730213.093149]: [apriltag] Tag 3 verworfen (hamming=1 - nur mit Bitfehler-Korrektur dekodiert)
[INFO] [1784730213.986876]: Lane error: 0.090 range [-1,1]
[WARN] [1784730214.770237]: [switch] Noch keine next_direction von der Planung - bleibe in STOPPING
[INFO] [1784730214.987190]: Lane error: 0.090 range [-1,1]
[WARN] [1784730215.131365]: [apriltag] Tag 3 verworfen (hamming=1 - nur mit Bitfehler-Korrektur dekodiert)
[WARN] [1784730215.770259]: [switch] Noch keine next_direction von der Planung - bleibe in STOPPING
[INFO] [1784730215.987317]: Lane error: 0.089 range [-1,1]
[WARN] [1784730216.870638]: [switch] Noch keine next_direction von der Planung - bleibe in STOPPING
[INFO] [1784730217.018416]: Lane error: 0.085 range [-1,1]
[WARN] [1784730217.152947]: [apriltag] Tag 3 verworfen (hamming=1 - nur mit Bitfehler-Korrektur dekodiert)
[WARN] [1784730217.970278]: [switch] Noch keine next_direction von der Planung - bleibe in STOPPING
[INFO] [1784730218.053713]: Lane error: 0.085 range [-1,1]
[WARN] [1784730219.070295]: [switch] Noch keine next_direction von der Planung - bleibe in STOPPING
[INFO] [1784730219.084625]: Lane error: 0.157 range [-1,1]
[INFO] [1784730220.084926]: Lane error: -0.542 range [-1,1]
[WARN] [1784730220.170213]: [switch] Noch keine next_direction von der Planung - bleibe in STOPPING
[WARN] [1784730220.718554]: [apriltag] Tag 4 verworfen (hamming=1 - nur mit Bitfehler-Korrektur dekodiert)
[INFO] [1784730221.085865]: Lane error: -0.180 range [-1,1]
[WARN] [1784730221.170277]: [switch] Noch keine next_direction von der Planung - bleibe in STOPPING
[INFO] [1784730222.086901]: Lane error: -0.183 range [-1,1]
[WARN] [1784730222.270248]: [switch] Noch keine next_direction von der Planung - bleibe in STOPPING
[INFO] [1784730223.088693]: Lane error: -0.240 range [-1,1]
[WARN] [1784730223.370253]: [switch] Noch keine next_direction von der Planung - bleibe in STOPPING
[INFO] [1784730224.121350]: Lane error: -0.240 range [-1,1]
[WARN] [1784730224.470291]: [switch] Noch keine next_direction von der Planung - bleibe in STOPPING
[INFO] [1784730225.155750]: Lane error: -0.183 range [-1,1]
[WARN] [1784730225.570257]: [switch] Noch keine next_direction von der Planung - bleibe in STOPPING
[INFO] [1784730226.162146]: Lane error: -0.240 range [-1,1]
[WARN] [1784730226.670287]: [switch] Noch keine next_direction von der Planung - bleibe in STOPPING
[INFO] [1784730227.189187]: Lane error: -0.177 range [-1,1]
[WARN] [1784730227.670362]: [switch] Noch keine next_direction von der Planung - bleibe in STOPPING
[INFO] [1784730228.221144]: Lane error: -0.177 range [-1,1]
[WARN] [1784730228.770560]: [switch] Noch keine next_direction von der Planung - bleibe in STOPPING
[INFO] [1784730229.098430]: [debug_graph] 'Bot versetzt' gedrueckt
[INFO] [1784730229.098883]: [path_planner] Bot-Neupositionierung bestaetigt - Route wird ab delivery_start_node=C (neu) geplant
[INFO] [1784730229.098887]: [graph_state] Bot von Hand an delivery_start_node neu positioniert: current_node B -> C (current_edge/Tag-Zustand geleert)
[INFO] [1784730229.102976]: [path_planner] Geplante Reihenfolge: ['6', '7', '8', '9', '10']
[INFO] [1784730229.224810]: Lane error: -0.175 range [-1,1]
[WARN] [1784730229.870234]: [switch] Noch keine next_direction von der Planung - bleibe in STOPPING
[INFO] [1784730230.254670]: Lane error: -0.176 range [-1,1]
[WARN] [1784730230.970198]: [switch] Noch keine next_direction von der Planung - bleibe in STOPPING
[INFO] [1784730231.283995]: Lane error: -0.175 range [-1,1]
[WARN] [1784730231.970283]: [switch] Noch keine next_direction von der Planung - bleibe in STOPPING
[INFO] [1784730232.284211]: Lane error: -0.175 range [-1,1]
[WARN] [1784730233.070189]: [switch] Noch keine next_direction von der Planung - bleibe in STOPPING
[INFO] [1784730233.285362]: Lane error: -0.175 range [-1,1]
[WARN] [1784730234.070377]: [switch] Noch keine next_direction von der Planung - bleibe in STOPPING
[INFO] [1784730234.316133]: Lane error: -0.177 range [-1,1]
[WARN] [1784730235.170246]: [switch] Noch keine next_direction von der Planung - bleibe in STOPPING
[INFO] [1784730235.319901]: Lane error: -0.176 range [-1,1]
[WARN] [1784730236.170353]: [switch] Noch keine next_direction von der Planung - bleibe in STOPPING
[INFO] [1784730236.349958]: Lane error: -0.176 range [-1,1]
[WARN] [1784730237.270622]: [switch] Noch keine next_direction von der Planung - bleibe in STOPPING
[INFO] [1784730237.350104]: Lane error: -0.176 range [-1,1]
[INFO] [1784730238.353010]: Lane error: -0.176 range [-1,1]
[WARN] [1784730238.370216]: [switch] Noch keine next_direction von der Planung - bleibe in STOPPING
[INFO] [1784730239.370247]: Lane error: -0.179 range [-1,1]
[WARN] [1784730239.470252]: [switch] Noch keine next_direction von der Planung - bleibe in STOPPING
[INFO] [1784730240.387573]: Lane error: -0.180 range [-1,1]
[WARN] [1784730240.470276]: [switch] Noch keine next_direction von der Planung - bleibe in STOPPING
[INFO] [1784730241.390993]: Lane error: -0.177 range [-1,1]
[WARN] [1784730241.570272]: [switch] Noch keine next_direction von der Planung - bleibe in STOPPING
[INFO] [1784730242.401303]: Lane error: -0.175 range [-1,1]
[WARN] [1784730242.670184]: [switch] Noch keine next_direction von der Planung - bleibe in STOPPING
[INFO] [1784730243.417957]: Lane error: -0.177 range [-1,1]
[WARN] [1784730243.670272]: [switch] Noch keine next_direction von der Planung - bleibe in STOPPING
[INFO] [1784730244.420504]: Lane error: -0.181 range [-1,1]
[WARN] [1784730244.670549]: [switch] Noch keine next_direction von der Planung - bleibe in STOPPING
[INFO] [1784730245.447964]: Lane error: -0.177 range [-1,1]
[WARN] [1784730245.770392]: [switch] Noch keine next_direction von der Planung - bleibe in STOPPING
[INFO] [1784730246.451662]: Lane error: -0.183 range [-1,1]
[WARN] [1784730246.870109]: [switch] Noch keine next_direction von der Planung - bleibe in STOPPING
[INFO] [1784730247.452008]: Lane error: -0.175 range [-1,1]
[WARN] [1784730247.870365]: [switch] Noch keine next_direction von der Planung - bleibe in STOPPING
[INFO] [1784730248.455332]: Lane error: -0.176 range [-1,1]
[WARN] [1784730248.970262]: [switch] Noch keine next_direction von der Planung - bleibe in STOPPING
[INFO] [1784730249.054397]: [debug_graph] 'Delivery starten' gedrueckt
[INFO] [1784730249.103044]: [path_planner] Delivery gestartet
[INFO] [1784730249.170102]: [switch] Richtung: right (aus Planung; Graph erlaubt: ['straight', 'right']) -> TURNING
[INFO] [1784730249.190556]: [switch] -> Turning
[INFO] [1784730249.190857]: [graph_state] C --1(right)--> A
[INFO] [1784730249.190941]: [control_intersection] Starte Sequenz: right
[INFO] [1784730249.466985]: Lane error: -0.185 range [-1,1]
[WARN] [1784730249.890265]: [apriltag] Tag 4 verworfen (hamming=2 - nur mit Bitfehler-Korrektur dekodiert)
[INFO] [1784730250.485951]: Lane error: -0.436 range [-1,1]
[INFO] [1784730251.533170]: Lane error: 0.703 range [-1,1]
[INFO] [1784730251.780669]: [control_intersection] Sequenz fertig -> turn_done
[INFO] [1784730251.870149]: [switch] Turning fertig -> LANE
[INFO] [1784730251.898661]: [switch] -> Lane
[INFO] [1784730251.927734]: [detect_apriltag] Neue Kante – Tag-Gedaechtnis zurueckgesetzt
[INFO] [1784730252.551489]: Lane error: 0.219 range [-1,1]
[INFO] [1784730253.581345]: Lane error: -0.589 range [-1,1]
[INFO] [1784730254.643319]: Lane error: -0.120 range [-1,1]
[INFO] [1784730255.653959]: Lane error: 0.340 range [-1,1]
[INFO] [1784730256.661360]: Lane error: 0.269 range [-1,1]
[INFO] [1784730257.694620]: Lane error: 0.035 range [-1,1]
[WARN] [1784730257.792220]: [apriltag] Tag 1 verworfen (hamming=1 - nur mit Bitfehler-Korrektur dekodiert)
[INFO] [1784730258.439953]: [switch] Kreuzung (Linie+Graph) -> STOPPING | erlaubte Richtungen: ['straight', 'left']
[INFO] [1784730258.457466]: [switch] -> Stopping
[INFO] [1784730258.715330]: Lane error: 0.005 range [-1,1]
[INFO] [1784730259.720406]: Lane error: -0.002 range [-1,1]
[INFO] [1784730260.470161]: [switch] Richtung: left (aus Planung; Graph erlaubt: ['straight', 'left']) -> TURNING
[INFO] [1784730260.499480]: [switch] -> Turning
[INFO] [1784730260.499824]: [graph_state] A --2(left)--> C
[INFO] [1784730260.499838]: [control_intersection] Starte Sequenz: left
[INFO] [1784730260.725000]: Lane error: 0.000 range [-1,1]
[INFO] [1784730261.745735]: Lane error: 0.002 range [-1,1]
[INFO] [1784730262.750282]: Lane error: -0.857 range [-1,1]
[INFO] [1784730263.786287]: Lane error: -0.240 range [-1,1]
[INFO] [1784730264.080405]: [control_intersection] Sequenz fertig -> turn_done
[INFO] [1784730264.170183]: [switch] Turning fertig -> LANE
[INFO] [1784730264.195814]: [switch] -> Lane
[INFO] [1784730264.234992]: [detect_apriltag] Neue Kante – Tag-Gedaechtnis zurueckgesetzt
[INFO] [1784730264.799794]: Lane error: -0.107 range [-1,1]
[INFO] [1784730265.823408]: Lane error: 0.060 range [-1,1]
[INFO] [1784730266.829404]: Lane error: 0.075 range [-1,1]
[INFO] [1784730267.850437]: Lane error: 0.220 range [-1,1]
[INFO] [1784730268.903720]: Lane error: 0.267 range [-1,1]
[INFO] [1784730269.926887]: Lane error: 0.201 range [-1,1]
[INFO] [1784730270.949455]: Lane error: 0.224 range [-1,1]
[INFO] [1784730271.969672]: Lane error: 0.226 range [-1,1]
[INFO] [1784730272.987108]: Lane error: 0.213 range [-1,1]
[INFO] [1784730273.531826]: [path_planner] Tor 6 abgefahren (Tag erneut bestaetigt) (1/5)
[WARN] [1784730273.617179]: [apriltag] Tag 3 verworfen (hamming=1 - nur mit Bitfehler-Korrektur dekodiert)
[INFO] [1784730273.987992]: Lane error: -0.095 range [-1,1]
[INFO] [1784730275.012052]: Lane error: 0.093 range [-1,1]
[INFO] [1784730276.013901]: Lane error: 0.172 range [-1,1]
[INFO] [1784730277.041058]: Lane error: -0.022 range [-1,1]
[WARN] [1784730277.410882]: [apriltag] Tag 3 verworfen (hamming=2 - nur mit Bitfehler-Korrektur dekodiert)
[INFO] [1784730277.584027]: [switch] Kreuzung (Linie+Graph) -> STOPPING | erlaubte Richtungen: ['right', 'left']
[INFO] [1784730277.607025]: [switch] -> Stopping
[INFO] [1784730278.048964]: Lane error: 0.261 range [-1,1]
[INFO] [1784730279.096913]: Lane error: 0.262 range [-1,1]
[INFO] [1784730279.670164]: [switch] Richtung: left (aus Planung; Graph erlaubt: ['right', 'left']) -> TURNING
[INFO] [1784730279.693104]: [switch] -> Turning
[INFO] [1784730279.693367]: [graph_state] C --1(left)--> A
[INFO] [1784730279.693528]: [control_intersection] Starte Sequenz: left
[INFO] [1784730280.153983]: Lane error: 0.282 range [-1,1]
[WARN] [1784730280.157686]: [apriltag] Tag 3 verworfen (hamming=1 - nur mit Bitfehler-Korrektur dekodiert)
[INFO] [1784730281.177397]: Lane error: 0.455 range [-1,1]
[INFO] [1784730282.208277]: Lane error: -0.320 range [-1,1]
[INFO] [1784730283.214476]: Lane error: 0.676 range [-1,1]
[INFO] [1784730283.280349]: [control_intersection] Sequenz fertig -> turn_done
[INFO] [1784730283.370206]: [switch] Turning fertig -> LANE
[INFO] [1784730283.394417]: [switch] -> Lane
[INFO] [1784730283.419828]: [detect_apriltag] Neue Kante – Tag-Gedaechtnis zurueckgesetzt
[INFO] [1784730284.241022]: Lane error: 0.240 range [-1,1]
[INFO] [1784730285.247708]: Lane error: 0.047 range [-1,1]
[WARN] [1784730286.180203]: [apriltag] Tag 1 verworfen (hamming=1 - nur mit Bitfehler-Korrektur dekodiert)
[INFO] [1784730286.250241]: Lane error: 0.025 range [-1,1]
[INFO] [1784730286.942877]: [switch] Kreuzung (Linie+Graph) -> STOPPING | erlaubte Richtungen: ['straight', 'left']
[INFO] [1784730286.974796]: [switch] -> Stopping
[INFO] [1784730287.282602]: Lane error: 0.103 range [-1,1]
[INFO] [1784730288.313211]: Lane error: 0.090 range [-1,1]
[INFO] [1784730289.070078]: [switch] Richtung: straight (aus Planung; Graph erlaubt: ['straight', 'left']) -> TURNING
[INFO] [1784730289.090885]: [switch] -> Turning
[INFO] [1784730289.091121]: [control_intersection] Starte Sequenz: straight
[INFO] [1784730289.091173]: [graph_state] A --1(straight)--> B
[INFO] [1784730289.342635]: Lane error: 0.145 range [-1,1]
[INFO] [1784730290.344308]: Lane error: -0.802 range [-1,1]
[INFO] [1784730291.375010]: Lane error: 0.157 range [-1,1]
[INFO] [1784730291.680488]: [control_intersection] Sequenz fertig -> turn_done
[INFO] [1784730291.770495]: [switch] Turning fertig -> LANE
[INFO] [1784730291.790962]: [switch] -> Lane
[INFO] [1784730291.812807]: [detect_apriltag] Neue Kante – Tag-Gedaechtnis zurueckgesetzt
[INFO] [1784730292.423053]: Lane error: 0.080 range [-1,1]
[INFO] [1784730293.441707]: Lane error: 0.068 range [-1,1]
[INFO] [1784730294.442229]: Lane error: -0.022 range [-1,1]
[INFO] [1784730295.513038]: Lane error: -0.146 range [-1,1]
[INFO] [1784730295.525278]: [path_planner] Tor 7 abgefahren (Tag erneut bestaetigt) (2/5)
[INFO] [1784730296.562396]: Lane error: 0.024 range [-1,1]
[INFO] [1784730297.565430]: Lane error: -0.031 range [-1,1]
[INFO] [1784730298.574604]: Lane error: -0.278 range [-1,1]
[INFO] [1784730299.628608]: Lane error: -0.177 range [-1,1]
[WARN] [1784730300.014343]: [apriltag] Tag 4 verworfen (hamming=1 - nur mit Bitfehler-Korrektur dekodiert)
[INFO] [1784730300.635828]: Lane error: 0.079 range [-1,1]
[INFO] [1784730301.541754]: [switch] Kreuzung (Linie+Graph) -> STOPPING | erlaubte Richtungen: ['left', 'straight', 'right']
[INFO] [1784730301.567233]: [switch] -> Stopping
[INFO] [1784730301.646231]: Lane error: 0.020 range [-1,1]
[INFO] [1784730302.674336]: Lane error: -0.135 range [-1,1]
[INFO] [1784730303.570097]: [switch] Richtung: straight (aus Planung; Graph erlaubt: ['left', 'straight', 'right']) -> TURNING
[INFO] [1784730303.595642]: [switch] -> Turning
[INFO] [1784730303.596053]: [graph_state] B --3(straight)--> C
[INFO] [1784730303.596270]: [control_intersection] Starte Sequenz: straight
[INFO] [1784730303.706017]: Lane error: -0.135 range [-1,1]
[INFO] [1784730304.767519]: Lane error: -0.768 range [-1,1]
[INFO] [1784730305.772508]: Lane error: 0.036 range [-1,1]
[INFO] [1784730306.180613]: [control_intersection] Sequenz fertig -> turn_done
[INFO] [1784730306.270095]: [switch] Turning fertig -> LANE
[INFO] [1784730306.286958]: [switch] -> Lane
[INFO] [1784730306.305307]: [detect_apriltag] Neue Kante – Tag-Gedaechtnis zurueckgesetzt
[INFO] [1784730306.789537]: Lane error: 0.100 range [-1,1]
[INFO] [1784730307.795106]: Lane error: 0.000 range [-1,1]
[INFO] [1784730308.803561]: Lane error: 0.165 range [-1,1]
[INFO] [1784730309.805909]: Lane error: -0.096 range [-1,1]
[INFO] [1784730310.806393]: Lane error: -0.343 range [-1,1]
[WARN] [1784730310.876683]: [apriltag] Tag 4 verworfen (hamming=1 - nur mit Bitfehler-Korrektur dekodiert)
[INFO] [1784730311.808673]: Lane error: 0.032 range [-1,1]
[INFO] [1784730312.853607]: Lane error: 0.065 range [-1,1]
[WARN] [1784730312.939868]: [apriltag] Tag 4 verworfen (hamming=1 - nur mit Bitfehler-Korrektur dekodiert)
[INFO] [1784730313.291098]: [switch] Kreuzung (Linie+Graph) -> STOPPING | erlaubte Richtungen: ['straight', 'right']
[INFO] [1784730313.334825]: [switch] -> Stopping
[INFO] [1784730313.869706]: Lane error: 0.012 range [-1,1]
[INFO] [1784730314.906845]: Lane error: 0.012 range [-1,1]
[INFO] [1784730315.370078]: [switch] Richtung: right (aus Planung; Graph erlaubt: ['straight', 'right']) -> TURNING
[INFO] [1784730315.390179]: [switch] -> Turning
[INFO] [1784730315.390277]: [control_intersection] Starte Sequenz: right
[INFO] [1784730315.390466]: [graph_state] C --1(right)--> A
[INFO] [1784730315.943424]: Lane error: 0.000 range [-1,1]
[INFO] [1784730316.945581]: Lane error: 0.000 range [-1,1]
[INFO] [1784730317.969978]: Lane error: -0.245 range [-1,1]
[INFO] [1784730317.980519]: [control_intersection] Sequenz fertig -> turn_done
[INFO] [1784730318.070148]: [switch] Turning fertig -> LANE
[INFO] [1784730318.090580]: [switch] -> Lane
[INFO] [1784730318.118848]: [detect_apriltag] Neue Kante – Tag-Gedaechtnis zurueckgesetzt
[INFO] [1784730318.970534]: Lane error: -0.125 range [-1,1]
[WARN] [1784730319.060968]: [apriltag] Tag 1 verworfen (hamming=1 - nur mit Bitfehler-Korrektur dekodiert)
[INFO] [1784730319.973196]: Lane error: 0.059 range [-1,1]
[INFO] [1784730320.342991]: [switch] Kreuzung (Linie+Graph) -> STOPPING | erlaubte Richtungen: ['straight', 'left']
[INFO] [1784730320.356575]: [switch] -> Stopping
[INFO] [1784730321.001530]: Lane error: 0.020 range [-1,1]
[INFO] [1784730322.004353]: Lane error: 0.020 range [-1,1]
[INFO] [1784730322.370169]: [switch] Richtung: left (aus Planung; Graph erlaubt: ['straight', 'left']) -> TURNING
[INFO] [1784730322.393080]: [switch] -> Turning
[INFO] [1784730322.393221]: [graph_state] A --2(left)--> C
[INFO] [1784730322.393393]: [control_intersection] Starte Sequenz: left
[INFO] [1784730323.038519]: Lane error: 0.000 range [-1,1]
[INFO] [1784730324.039798]: Lane error: 0.000 range [-1,1]
[INFO] [1784730325.067863]: Lane error: 0.471 range [-1,1]
[INFO] [1784730325.980513]: [control_intersection] Sequenz fertig -> turn_done
[INFO] [1784730326.070107]: [switch] Turning fertig -> LANE
[INFO] [1784730326.070415]: Lane error: 0.115 range [-1,1]
[INFO] [1784730326.100048]: [switch] -> Lane
[INFO] [1784730326.132702]: [detect_apriltag] Neue Kante – Tag-Gedaechtnis zurueckgesetzt
[INFO] [1784730327.120421]: Lane error: 0.045 range [-1,1]
[INFO] [1784730328.138020]: Lane error: -0.004 range [-1,1]
[INFO] [1784730329.173356]: Lane error: 0.025 range [-1,1]
[INFO] [1784730330.201690]: Lane error: 0.175 range [-1,1]
[INFO] [1784730331.209855]: Lane error: 0.286 range [-1,1]
[INFO] [1784730332.236173]: Lane error: -0.031 range [-1,1]
[INFO] [1784730333.268094]: Lane error: 0.220 range [-1,1]
[INFO] [1784730334.270161]: Lane error: 0.080 range [-1,1]
[INFO] [1784730335.304463]: Lane error: 0.135 range [-1,1]
[INFO] [1784730336.336516]: Lane error: -0.111 range [-1,1]
[INFO] [1784730337.367845]: Lane error: 0.040 range [-1,1]
[INFO] [1784730338.402150]: Lane error: 0.144 range [-1,1]
[INFO] [1784730339.430575]: Lane error: 0.060 range [-1,1]
[WARN] [1784730339.599716]: [apriltag] Tag 3 verworfen (hamming=1 - nur mit Bitfehler-Korrektur dekodiert)
[INFO] [1784730339.835168]: [switch] Kreuzung (Linie+Graph) -> STOPPING | erlaubte Richtungen: ['right', 'left']
[INFO] [1784730339.849997]: [switch] -> Stopping
[INFO] [1784730340.473377]: Lane error: 0.270 range [-1,1]
[INFO] [1784730341.497868]: Lane error: 0.270 range [-1,1]
[INFO] [1784730341.870018]: [switch] Richtung: straight (aus Planung; Graph erlaubt: ['right', 'left']) -> TURNING
[INFO] [1784730341.893093]: [switch] -> Turning
[INFO] [1784730341.893407]: [graph_state] C --4(straight)--> B
[INFO] [1784730341.893619]: [control_intersection] Starte Sequenz: straight
[WARN] [1784730342.364434]: [apriltag] Tag 3 verworfen (hamming=1 - nur mit Bitfehler-Korrektur dekodiert)
[INFO] [1784730342.504958]: Lane error: 0.321 range [-1,1]
[INFO] [1784730343.532446]: Lane error: 0.555 range [-1,1]
[INFO] [1784730344.480462]: [control_intersection] Sequenz fertig -> turn_done
[INFO] [1784730344.564258]: Lane error: 0.650 range [-1,1]
[INFO] [1784730344.570098]: [switch] Turning fertig -> LANE
[INFO] [1784730344.582467]: [switch] -> Lane
[INFO] [1784730344.608341]: [detect_apriltag] Neue Kante – Tag-Gedaechtnis zurueckgesetzt
[INFO] [1784730345.599278]: Lane error: 0.094 range [-1,1]
[INFO] [1784730346.601808]: Lane error: 0.046 range [-1,1]
[INFO] [1784730347.628237]: Lane error: -0.180 range [-1,1]
[INFO] [1784730348.628566]: Lane error: 0.020 range [-1,1]
[INFO] [1784730349.636005]: Lane error: 0.145 range [-1,1]
[INFO] [1784730350.318234]: [path_planner] Tor 8 abgefahren (Tag erneut bestaetigt) (3/5)
[INFO] [1784730350.670137]: Lane error: 0.259 range [-1,1]
[INFO] [1784730351.679139]: Lane error: 0.157 range [-1,1]
[INFO] [1784730352.695425]: Lane error: 0.169 range [-1,1]
[WARN] [1784730353.340297]: [apriltag] Tag 3 verworfen (hamming=1 - nur mit Bitfehler-Korrektur dekodiert)
[INFO] [1784730353.695997]: Lane error: 0.079 range [-1,1]
[INFO] [1784730354.538444]: [switch] Kreuzung (Linie+Graph) -> STOPPING | erlaubte Richtungen: ['straight', 'left']
[INFO] [1784730354.560654]: [switch] -> Stopping
[INFO] [1784730354.697029]: Lane error: 0.044 range [-1,1]
[WARN] [1784730355.401815]: [apriltag] Tag 3 verworfen (hamming=1 - nur mit Bitfehler-Korrektur dekodiert)
[INFO] [1784730355.754547]: Lane error: 0.042 range [-1,1]
[WARN] [1784730356.570112]: [switch] Noch keine next_direction von der Planung - bleibe in STOPPING
[INFO] [1784730356.763791]: Lane error: 0.040 range [-1,1]
[WARN] [1784730357.428847]: [apriltag] Tag 3 verworfen (hamming=1 - nur mit Bitfehler-Korrektur dekodiert)
[WARN] [1784730357.670167]: [switch] Noch keine next_direction von der Planung - bleibe in STOPPING
[INFO] [1784730357.764591]: Lane error: 0.042 range [-1,1]
[WARN] [1784730358.670176]: [switch] Noch keine next_direction von der Planung - bleibe in STOPPING
[INFO] [1784730358.766092]: Lane error: 0.042 range [-1,1]
[WARN] [1784730359.428903]: [apriltag] Tag 3 verworfen (hamming=1 - nur mit Bitfehler-Korrektur dekodiert)
[WARN] [1784730359.770171]: [switch] Noch keine next_direction von der Planung - bleibe in STOPPING
[INFO] [1784730359.795660]: Lane error: 0.042 range [-1,1]
[WARN] [1784730360.770183]: [switch] Noch keine next_direction von der Planung - bleibe in STOPPING
[INFO] [1784730360.795730]: Lane error: 0.042 range [-1,1]
[WARN] [1784730361.461857]: [apriltag] Tag 3 verworfen (hamming=1 - nur mit Bitfehler-Korrektur dekodiert)
[WARN] [1784730361.770184]: [switch] Noch keine next_direction von der Planung - bleibe in STOPPING
[INFO] [1784730361.796215]: Lane error: 0.042 range [-1,1]
[WARN] [1784730362.770203]: [switch] Noch keine next_direction von der Planung - bleibe in STOPPING
[INFO] [1784730362.797092]: Lane error: 0.042 range [-1,1]
[WARN] [1784730363.495853]: [apriltag] Tag 3 verworfen (hamming=1 - nur mit Bitfehler-Korrektur dekodiert)
[INFO] [1784730363.798378]: Lane error: 0.042 range [-1,1]
[WARN] [1784730363.870564]: [switch] Noch keine next_direction von der Planung - bleibe in STOPPING
[INFO] [1784730364.836520]: Lane error: 0.042 range [-1,1]
[WARN] [1784730364.970486]: [switch] Noch keine next_direction von der Planung - bleibe in STOPPING
[WARN] [1784730365.542763]: [apriltag] Tag 3 verworfen (hamming=1 - nur mit Bitfehler-Korrektur dekodiert)
[INFO] [1784730365.863801]: Lane error: 0.042 range [-1,1]
[WARN] [1784730366.070271]: [switch] Noch keine next_direction von der Planung - bleibe in STOPPING
[INFO] [1784730366.868826]: Lane error: 0.041 range [-1,1]
[WARN] [1784730367.170469]: [switch] Noch keine next_direction von der Planung - bleibe in STOPPING
[WARN] [1784730367.564651]: [apriltag] Tag 3 verworfen (hamming=1 - nur mit Bitfehler-Korrektur dekodiert)
[INFO] [1784730367.899651]: Lane error: 0.041 range [-1,1]
[WARN] [1784730368.270186]: [switch] Noch keine next_direction von der Planung - bleibe in STOPPING
[INFO] [1784730368.926700]: Lane error: 0.042 range [-1,1]
[WARN] [1784730369.270235]: [switch] Noch keine next_direction von der Planung - bleibe in STOPPING
[WARN] [1784730369.594490]: [apriltag] Tag 3 verworfen (hamming=1 - nur mit Bitfehler-Korrektur dekodiert)
[INFO] [1784730369.926793]: Lane error: 0.042 range [-1,1]
[WARN] [1784730370.370170]: [switch] Noch keine next_direction von der Planung - bleibe in STOPPING
[INFO] [1784730370.928179]: Lane error: 0.042 range [-1,1]
[WARN] [1784730371.370190]: [switch] Noch keine next_direction von der Planung - bleibe in STOPPING
[WARN] [1784730371.628280]: [apriltag] Tag 3 verworfen (hamming=1 - nur mit Bitfehler-Korrektur dekodiert)
[INFO] [1784730371.960597]: Lane error: 0.042 range [-1,1]
[WARN] [1784730372.370253]: [switch] Noch keine next_direction von der Planung - bleibe in STOPPING
[INFO] [1784730372.993766]: Lane error: 0.041 range [-1,1]
[WARN] [1784730373.370180]: [switch] Noch keine next_direction von der Planung - bleibe in STOPPING
[WARN] [1784730373.630516]: [apriltag] Tag 3 verworfen (hamming=1 - nur mit Bitfehler-Korrektur dekodiert)
[INFO] [1784730374.027956]: Lane error: 0.040 range [-1,1]
[WARN] [1784730374.470246]: [switch] Noch keine next_direction von der Planung - bleibe in STOPPING
[INFO] [1784730375.062251]: Lane error: 0.041 range [-1,1]
[WARN] [1784730375.570184]: [switch] Noch keine next_direction von der Planung - bleibe in STOPPING
[WARN] [1784730375.658844]: [apriltag] Tag 3 verworfen (hamming=1 - nur mit Bitfehler-Korrektur dekodiert)
[INFO] [1784730376.094418]: Lane error: 0.042 range [-1,1]
[WARN] [1784730376.670164]: [switch] Noch keine next_direction von der Planung - bleibe in STOPPING
[INFO] [1784730377.098470]: Lane error: 0.042 range [-1,1]
