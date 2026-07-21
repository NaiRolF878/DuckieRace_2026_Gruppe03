#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────────────────────
# graph_state_node.py  (Challenge 4 – Mapping & Path Finding)
#
# Verwaltet den kompletten Graphen-Zustand zur Laufzeit. Laedt mapping_node.json
# DIREKT (nicht ueber util.init_parameters - andere JSON-Struktur ohne
# "parameters"-Block).
#
# Tag-ID <-> Wort-Uebersetzung (Erweiterung ueber das reine Nachschlagen hinaus):
#   Der an einer Kreuzung sichtbare AprilTag (1-4) markiert die EINFAHRT-Einmuendung
#   (die Einmuendung, ueber die der Bot gerade ankommt). Die Graph-Keys in
#   mapping_node.json referenzieren dagegen die AUSFAHRT-Einmuendung. Da die
#   Einmuendungs-Geometrie fest ist (Tag1&3 gegenueber, Tag2 rechts von Tag1,
#   Tag4 links von Tag1 - siehe Konzept), lassen sich Einfahrt-Tag + gewaehlte
#   Wort-Richtung (left/right/straight) IMMER in eine Ausfahrt-Tag-ID umrechnen:
#     Ausfahrt = Einfahrt + 2  -> straight
#     Ausfahrt = Einfahrt + 1  -> right
#     Ausfahrt = Einfahrt - 1  -> left      (jeweils mod 4, Tags 1-4)
#   Diese Node macht daher zusaetzlich zum reinen Graph-Nachschlagen auch diese
#   Umrechnung (in beide Richtungen) und veroeffentlicht sie als
#   /graph/exit_directions, damit explore_control_node und path_planner_node
#   ihre Tag-ID-basierten Entscheidungen (aus dem Graphen) in die Wort-Richtung
#   uebersetzen koennen, die switch_control_node/control_intersection_node
#   erwarten (left/right/straight).
#
# Der Graph-Uebergang (current_node -> naechster Knoten, visited_edges) wird
# ausgeloest, sobald /intersection/phase auf "Turning" wechselt - das ist der
# Moment, in dem die Richtung fuer DIESE Kreuzung garantiert frisch gewaehlt
# wurde (cbStopLine setzt die Richtung unmittelbar vor Stopping, Stopping geht
# danach in Turning ueber). So wird verhindert, dass eine noch alte
# /intersection/direction-Nachricht aus der letzten Kreuzung faelschlich
# fuer die naechste Kreuzung verwendet wird.
#
# current_edge (fuer die Tor-Zuordnung in cbGateId) folgt dagegen dem
# tatsaechlichen physischen Befahren, nicht der logischen Graph-Entscheidung:
# es wird erst beim Wechsel Turning->Lane aktiv (der Bot faehrt jetzt wirklich
# die Kante ab) und wieder geleert beim Wechsel Lane->Stopping (naechste
# Kreuzung erreicht, Kante vorbei). Ohne diese Trennung wuerde current_edge
# waehrend der gesamten Entscheidungsphase an der naechsten Kreuzung noch auf
# die VORHERIGE Kante zeigen - ein dort faelschlich erkanntes Tor wuerde dann
# der falschen, bereits verlassenen Kante zugeordnet.
# ─────────────────────────────────────────────────────────────────────────────

import json
import os
import rospy
from std_msgs.msg import String, Int32, Bool


class GraphStateNode:
    # Ausfahrt-Tag = Einfahrt-Tag + Offset (mod 4, Tags 1-4)
    _OFFSET_FOR_DIR = {"straight": 2, "right": 1, "left": 3}
    _DIR_FOR_OFFSET = {2: "straight", 1: "right", 3: "left"}

    def __init__(self, node_name):
        rospy.init_node(node_name)
        self._vehicle_name = os.environ['VEHICLE_NAME']

        self._load_map()
        self._load_tag_directions()

        self.current_node     = self.mapping_start_node
        self.current_edge     = None   # {"from": node, "tag": "2"} oder None - NUR waehrend
                                        # der Bot die Kante tatsaechlich befaehrt (Lane-Phase
                                        # nach abgeschlossenem Turning, siehe cbPhase)
        self._pending_edge    = None   # von _advance_graph gesetzt, wird erst beim
                                        # Turning->Lane-Wechsel zu current_edge (s.u.)
        self.visited_edges    = []     # [[node, tag], ...] normalisiert
        # {"5": {"node":.., "tag":..}, ...} - vorbelegt aus mapping_node.json
        # (Feld "gate_map"), damit ein von Hand dort eingetragener/korrigierter
        # Eintrag nicht durch eine neue Live-Erkennung ueberschrieben wird
        # (siehe cbGateId: "nicht ueberschreiben falls bereits vorhanden").
        self.gate_map         = self._load_gate_map_from_config()

        self.current_entry_tag = None  # zuletzt LIVE gesehener Kreuzungs-Tag (1-4)
        # Rein aus dem Graph vorhergesagter Eingangs-Tag fuer current_node -
        # von _advance_graph() gesetzt, sobald die Abbiegeentscheidung an der
        # VORHERIGEN Kreuzung feststeht (also lange bevor der Bot ueberhaupt
        # an dieser Kreuzung ankommt). Dient als Fallback fuer current_entry_tag,
        # wenn die Kamera den Tag diesmal gar nicht lesen kann (siehe
        # _effective_entry_tag) - wir kennen die Einfahrt-Geometrie durch die
        # eigene Kartenverfolgung ohnehin deterministisch, unabhaengig davon,
        # ob die Kamera sie gerade bestaetigen kann.
        self.predicted_entry_tag = None
        self._last_direction    = ""   # zuletzt von switch_control gewaehltes Wort
        self.phase               = "Lane"
        self._last_phase         = "Lane"

        rospy.Subscriber(f'/{self._vehicle_name}/detect/apriltag/id',
                         Int32, self.cbAprilTagId, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/intersection/direction',
                         String, self.cbDirection, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/intersection/phase',
                         String, self.cbPhase, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/detect/gate/id',
                         Int32, self.cbGateId, queue_size=1)
        # Notfall-Korrektur: mapping_node.json von Hand editiert (Tor-ID
        # getauscht/entfernt) - auf diesem Trigger wird NUR gate_map neu von
        # der Platte gelesen, current_node/visited_edges bleiben unberuehrt
        # (kein Neustart der Exploration noetig).
        rospy.Subscriber(f'/{self._vehicle_name}/graph/reload_gate_map',
                         Bool, self.cbReloadGateMap, queue_size=1)
        # Erkundung neu starten (z.B. weil ein Tor uebersehen wurde): nur
        # visited_edges leeren, damit explore_control_node wieder jede Kante
        # als unbesucht ansieht - current_node/current_edge (physische
        # Position) und gate_map (schon gefundene Tore) bleiben erhalten.
        rospy.Subscriber(f'/{self._vehicle_name}/graph/reset_exploration',
                         Bool, self.cbResetExploration, queue_size=1)

        self.pub_current_node = rospy.Publisher(
            f'/{self._vehicle_name}/graph/current_node', String, queue_size=1)
        self.pub_current_edge = rospy.Publisher(
            f'/{self._vehicle_name}/graph/current_edge', String, queue_size=1)
        self.pub_visited_edges = rospy.Publisher(
            f'/{self._vehicle_name}/graph/visited_edges', String, queue_size=1)
        self.pub_gate_map = rospy.Publisher(
            f'/{self._vehicle_name}/graph/gate_map', String, queue_size=1)
        self.pub_exit_directions = rospy.Publisher(
            f'/{self._vehicle_name}/graph/exit_directions', String, queue_size=1)
        # Vom Graph vorhergesagte erlaubte Richtungen (aus predicted_entry_tag
        # + tag_directions) - Fallback-Quelle fuer switch_control_node, falls
        # dessen eigene Live-Erkennung an dieser Kreuzung ausbleibt/steckenbleibt.
        self.pub_allowed_directions = rospy.Publisher(
            f'/{self._vehicle_name}/graph/allowed_directions', String, queue_size=1)

        rospy.loginfo(f"[{node_name}] Bereit. Start-Knoten: {self.current_node}")

    # ── Config laden ─────────────────────────────────────────────────────────

    def _mapping_config_path(self):
        return os.path.join(os.path.dirname(__file__), "../config/mapping_node.json")

    def _load_map(self):
        path = self._mapping_config_path()
        try:
            with open(path, 'r') as f:
                config = json.load(f)
            self.graph               = config["graph"]
            self.mapping_start_node  = config["mapping_start_node"]
            self.delivery_start_node = config.get("delivery_start_node", self.mapping_start_node)
            self.path_planning       = config.get("path_planning", {})
        except Exception as e:
            rospy.logerr(f"[graph_state] mapping_node.json konnte nicht geladen werden: {e}")
            raise

    def _load_gate_map_from_config(self):
        # Liest NUR das gate_map-Feld frisch von der Platte - fuer den
        # Notfall-Reload (cbReloadGateMap) bewusst getrennt von _load_map(),
        # damit graph/mapping_start_node/path_planning dabei unangetastet
        # bleiben.
        path = self._mapping_config_path()
        try:
            with open(path, 'r') as f:
                config = json.load(f)
            raw = config.get("gate_map", {})
        except Exception as e:
            rospy.logwarn(f"[graph_state] gate_map konnte nicht geladen werden: {e}")
            return {}
        gate_map = {}
        for gate_id, entry in raw.items():
            if not isinstance(entry, dict) or "node" not in entry or "tag" not in entry:
                rospy.logwarn(f"[graph_state] Ungueltiger gate_map-Eintrag fuer Tor "
                              f"{gate_id} ignoriert: {entry}")
                continue
            gate_map[str(gate_id)] = {"node": entry["node"], "tag": str(entry["tag"])}
        return gate_map

    def _save_gate_map(self):
        # Neu entdecktes Tor in mapping_node.json zurueckschreiben, damit die
        # Datei jederzeit den aktuell bekannten Stand zeigt - Grundlage fuer
        # die manuelle Notfall-Korrektur (von Hand editieren, dann per
        # /graph/reload_gate_map neu einlesen, siehe cbReloadGateMap).
        path = self._mapping_config_path()
        try:
            with open(path, 'r') as f:
                config = json.load(f)
            config["gate_map"] = self.gate_map
            with open(path, 'w') as f:
                json.dump(config, f, indent=2)
        except Exception as e:
            rospy.logwarn(f"[graph_state] gate_map konnte nicht gespeichert werden: {e}")

    def _load_tag_directions(self):
        # Gleiche Datei/gleicher Schluessel wie detect_apriltag_node._load_tag_config
        # (keine zweite Kopie der Zuordnung pflegen) - Tag-ID -> erlaubte
        # Wort-Richtungen ist eine feste Geometrie-Eigenschaft der Kreuzung,
        # unabhaengig davon ob sie live erkannt oder aus dem Graph
        # vorhergesagt wurde.
        path = os.path.join(os.path.dirname(__file__), "../config/detect_apriltag_node.json")
        with open(path, 'r') as f:
            config = json.load(f)
        raw = config.get("tag_directions", {})
        self.tag_directions = {int(k): v for k, v in raw.items()}

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def cbAprilTagId(self, msg):
        # Nur akzeptieren, waehrend der Bot tatsaechlich auf dem Weg zur naechsten
        # Kreuzung ist (Lane-Phase). Waehrend Stopping/Turning kann eine eingehende
        # Tag-ID nur noch ein Rest der gerade verlassenen/aktuellen Kreuzung sein -
        # die soll current_entry_tag der NAECHSTEN Kreuzung nicht kontaminieren.
        if self.phase == "Lane" and msg.data != -1 and 1 <= msg.data <= 4:
            self.current_entry_tag = msg.data

    def cbDirection(self, msg):
        self._last_direction = msg.data

    def cbGateId(self, msg):
        gate_id = msg.data
        if gate_id == -1 or self.current_edge is None:
            return
        key = str(gate_id)
        if key in self.gate_map:
            return  # nicht ueberschreiben falls bereits vorhanden
        # Format wie spezifiziert: {"node": .., "tag": ..} - current_edge nutzt
        # "from" statt "node" fuer denselben Sachverhalt, NICHT einfach kopieren
        # (path_planner_node/debug_graph_node erwarten explizit "node").
        self.gate_map[key] = {"node": self.current_edge["from"], "tag": self.current_edge["tag"]}
        rospy.loginfo(f"[graph_state] Tor {gate_id} -> Kante {self.gate_map[key]}")
        self._save_gate_map()

    def cbReloadGateMap(self, msg):
        # Notfall-Korrektur: mapping_node.json wurde von Hand editiert (Tor-ID
        # getauscht/entfernt) - komplett durch den neu eingelesenen Stand
        # ersetzen (nicht nur mergen), damit z.B. ein entferntes Tor auch
        # wirklich verschwindet. current_node/visited_edges bleiben unberuehrt.
        if not msg.data:
            return
        new_map = self._load_gate_map_from_config()
        rospy.loginfo(f"[graph_state] gate_map neu geladen: {self.gate_map} -> {new_map}")
        self.gate_map = new_map

    def cbResetExploration(self, msg):
        if not msg.data:
            return
        rospy.loginfo(f"[graph_state] Erkundung zurueckgesetzt: "
                      f"{len(self.visited_edges)} Kante(n) wieder als unbesucht markiert "
                      f"(Tor-Zuordnung bleibt erhalten)")
        self.visited_edges = []

    def cbPhase(self, msg):
        phase = msg.data
        self.phase = phase
        if phase == "Turning" and self._last_phase != "Turning":
            # Kreuzung wird verlassen: Graph-Zustand (current_node/visited_edges)
            # ist deterministisch aus der eigenen Karte bekannt -> sofort aktualisieren.
            # Die Kante selbst wird aber erst "aktiv" (current_edge), sobald das
            # Abbiegen fertig ist und der Bot tatsaechlich auf ihr faehrt (s.u.).
            self._advance_graph()
        elif phase == "Lane" and self._last_phase == "Turning":
            # Abbiegen abgeschlossen: Bot faehrt jetzt wirklich die neue Kante ab
            # -> Tor-Erkennung (cbGateId) darf ihr ab jetzt zugeordnet werden.
            self.current_edge = self._pending_edge
            self._pending_edge = None
        elif phase == "Stopping" and self._last_phase == "Lane":
            # Naechste Kreuzung erreicht, Bot steht: die zuletzt befahrene Kante
            # ist vorbei -> current_edge leeren, damit ein hier faelschlich
            # erkanntes Tor nicht der alten Kante zugeordnet wird.
            self.current_edge = None
        self._last_phase = phase

    # ── Tag-ID <-> Wort ───────────────────────────────────────────────────────

    def _exit_tag_for(self, entry_tag, direction):
        offset = self._OFFSET_FOR_DIR.get(direction)
        if offset is None:
            return None
        return ((entry_tag - 1 + offset) % 4) + 1

    def _word_for_exit(self, entry_tag, exit_tag):
        if entry_tag is None:
            return None
        offset = (exit_tag - entry_tag) % 4
        return self._DIR_FOR_OFFSET.get(offset)

    def _effective_entry_tag(self):
        # Live-Lesung bevorzugen (echte Kamera-Bestaetigung), aber NIE
        # komplett blockieren, wenn sie diesmal ausbleibt: current_node wird
        # ohnehin ausschliesslich aus der eigenen Kartenverfolgung bestimmt
        # (kein Tag identifiziert je einen Knoten, siehe Kopfkommentar) - der
        # Eingangs-Tag fuer den aktuellen Knoten ist daher bereits durch die
        # VORHERIGE Abbiegeentscheidung deterministisch bekannt (predicted_
        # entry_tag), unabhaengig davon ob die Kamera ihn diesmal bestaetigen
        # kann. Ohne diesen Fallback bleibt exit_directions leer -> next_direction
        # bleibt dauerhaft "" -> die FSM haengt fest (siehe Diskussion).
        return self.current_entry_tag if self.current_entry_tag is not None \
            else self.predicted_entry_tag

    def _compute_exit_directions(self):
        # Fuer jede moegliche Ausfahrt am current_node: passendes Wort, damit
        # explore_control_node/path_planner_node ihre Tag-ID-Entscheidung in
        # left/right/straight uebersetzen koennen.
        entry = self._effective_entry_tag()
        result = {}
        for exit_tag_str in self.graph.get(self.current_node, {}).keys():
            word = self._word_for_exit(entry, int(exit_tag_str))
            if word is not None:
                result[exit_tag_str] = word
        return result

    def _predicted_allowed_directions(self):
        # Rein aus dem Graph vorhergesagte erlaubte Richtungen fuer
        # current_node - bewusst UNABHAENGIG von jeder Live-Kamera-Lesung
        # (auch unabhaengig von current_entry_tag), damit switch_control_node
        # eine wirklich zweite, eigenstaendige Quelle hat, wenn seine eigene
        # Live-Erkennung steckenbleibt/ausbleibt.
        if self.predicted_entry_tag is None:
            return []
        return self.tag_directions.get(self.predicted_entry_tag, [])

    # ── Graph-Uebergang ───────────────────────────────────────────────────────

    def _mark_visited(self, node_a, tag_a, node_b, tag_b):
        endpoint_a = [node_a, tag_a]
        endpoint_b = [node_b, tag_b]
        if node_a != node_b:
            normalized = endpoint_a if node_a <= node_b else endpoint_b
        else:
            # Selbstschleife (z.B. Wendeschleife/Sackgassen-Loop zurueck zum
            # selben Knoten): der Knotenvergleich kann hier nicht disambiguieren
            # (beide Enden sind derselbe Knoten) - stattdessen nach Tag
            # normalisieren, damit BEIDE Fahrtrichtungen der Schleife als EINE
            # Kante gelten (wie bei jeder anderen symmetrischen Kante auch).
            normalized = endpoint_a if tag_a <= tag_b else endpoint_b
        if normalized not in self.visited_edges:
            self.visited_edges.append(normalized)

    def _advance_graph(self):
        # Effektiven (live oder vorhergesagten) Tag verwenden, NICHT nur
        # current_entry_tag: sonst wuerde ein per Graph-Fallback erfolgreich
        # gefahrener Abbiegevorgang (siehe switch_control_node) hier trotzdem
        # verworfen ("Graph-Update uebersprungen") und die Karte liefe der
        # physischen Position des Bots hinterher.
        entry     = self._effective_entry_tag()
        direction = self._last_direction

        if entry is None or not direction or direction == "unknown":
            rospy.logwarn(f"[graph_state] Turning ohne gueltigen Tag/Richtung "
                          f"(tag={entry}, dir='{direction}') - Graph-Update uebersprungen")
            return

        exit_tag = self._exit_tag_for(entry, direction)
        if exit_tag is None:
            rospy.logwarn(f"[graph_state] Unbekannte Richtung '{direction}'")
            return

        target = self.graph.get(self.current_node, {}).get(str(exit_tag))
        if target is None:
            rospy.logwarn(f"[graph_state] Kein Graph-Eintrag fuer "
                          f"{self.current_node}[{exit_tag}]")
            return

        neighbor_node, neighbor_tag = target
        self._mark_visited(self.current_node, str(exit_tag), str(neighbor_node), str(neighbor_tag))
        # Noch nicht current_edge setzen - das passiert erst beim Turning->Lane-Wechsel
        # in cbPhase, sobald der Bot die Kante wirklich befaehrt (siehe dortiger Kommentar).
        self._pending_edge = {"from": self.current_node, "tag": str(exit_tag)}
        rospy.loginfo(f"[graph_state] {self.current_node} --{exit_tag}({direction})--> {neighbor_node}")
        self.current_node = neighbor_node
        # Neue Kreuzung: alter Eingangs-Tag gilt nicht mehr, bis ein neuer erkannt wird.
        self.current_entry_tag = None
        # Eingangs-Tag der NEUEN Kreuzung ist durch diese Abbiegeentscheidung
        # bereits deterministisch bekannt, lange bevor der Bot dort ankommt
        # und unabhaengig davon, ob die Kamera ihn dort lesen kann.
        self.predicted_entry_tag = int(neighbor_tag)

    # ── Hauptschleife ─────────────────────────────────────────────────────────

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.pub_current_node.publish(String(data=self.current_node))
            self.pub_current_edge.publish(
                String(data=json.dumps(self.current_edge) if self.current_edge else ""))
            self.pub_visited_edges.publish(String(data=json.dumps(self.visited_edges)))
            self.pub_gate_map.publish(String(data=json.dumps(self.gate_map)))
            self.pub_exit_directions.publish(String(data=json.dumps(self._compute_exit_directions())))
            self.pub_allowed_directions.publish(
                String(data=",".join(self._predicted_allowed_directions())))
            rate.sleep()


if __name__ == '__main__':
    node = GraphStateNode('graph_state_node')
    node.run()
    rospy.spin()
