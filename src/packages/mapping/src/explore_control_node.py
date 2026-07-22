#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────────────────────
# explore_control_node.py  (Challenge 4 – Mapping & Path Finding)
#
# Steuert Phase 1 (Mapping) mittels DFS ueber alle Graphkanten.
#
# Laedt mapping_node.json DIREKT (wie graph_state_node) fuer die vollstaendige,
# statische Graph-Topologie (welche Ausfahrten gibt es an einem Knoten). Der
# LAUFENDE Zustand (current_node, visited_edges, exit_directions) kommt dagegen
# ausschliesslich per Topic von graph_state_node - single source of truth fuer
# den Laufzeit-Zustand.
#
# Tag-ID -> Wort: Diese Node entscheidet in Tag-IDs (Graph-Keys). Da
# switch_control_node/control_intersection_node weiterhin Wort-Richtungen
# (left/right/straight) erwarten, wird jede gewaehlte Tag-ID ueber
# /graph/exit_directions (von graph_state_node, siehe dort) in ein Wort
# uebersetzt, bevor sie auf next_direction publiziert wird. Bis die Uebersetzung
# verfuegbar ist (der Eingangs-Tag der naechsten Kreuzung noch nicht sichtbar),
# wird "" publiziert - switch_control_node bleibt dann einfach in STOPPING.
#
# Design-Entscheidung: die naechste Ausfahrt wird NICHT einmalig bei Ankunft
# entschieden und zwischengespeichert, sondern JEDEN Tick frisch aus
# current_node/visited_edges/exit_directions neu berechnet (fuer die kleinen
# Graphen dieser Challenge voellig unkritisch). Das macht die Logik robust
# gegenueber der Zeitverzoegerung zwischen "current_node hat sich geaendert"
# und "exit_directions ist fuer den neuen Knoten aktualisiert" - mit einer
# einmalig zwischengespeicherten Entscheidung koennte genau in diesem Fenster
# eine falsche/veraltete Ausfahrt gewaehlt und dauerhaft festgehalten werden.
# ─────────────────────────────────────────────────────────────────────────────

import json
import os
from collections import deque
import rospy
from std_msgs.msg import String, Bool


class ExploreControlNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        self._vehicle_name = os.environ['VEHICLE_NAME']

        self._load_map()
        self._total_edges = self._count_total_edges()
        self._adjacency   = self._build_adjacency()

        self.current_node     = None
        self.visited_edges    = []
        self.exit_directions  = {}
        self.phase            = "exploration"
        self.exploration_done = False
        # FSM-Phase des Bots (Lane/Stopping/Turning) - noetig, um "Erkundung
        # fertig" erst zu melden, wenn der Bot die letzte Kante WIRKLICH
        # fertig gefahren hat und steht, nicht schon wenn visited_edges die
        # Gesamtzahl erreicht (das passiert bereits beim START der letzten
        # Abbiegung, siehe graph_state_node._advance_graph/_mark_visited -
        # der Bot faehrt zu diesem Zeitpunkt noch).
        self.intersection_phase = ""

        rospy.Subscriber(f'/{self._vehicle_name}/graph/current_node',
                         String, self.cbCurrentNode, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/graph/visited_edges',
                         String, self.cbVisitedEdges, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/graph/exit_directions',
                         String, self.cbExitDirections, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/navigation/phase',
                         String, self.cbPhase, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/intersection/phase',
                         String, self.cbIntersectionPhase, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/graph/reset_exploration',
                         Bool, self.cbResetExploration, queue_size=1)

        self.pub_next_direction = rospy.Publisher(
            f'/{self._vehicle_name}/navigation/next_direction', String, queue_size=1)
        self.pub_exploration_done = rospy.Publisher(
            f'/{self._vehicle_name}/navigation/exploration_done', Bool, queue_size=1)
        self.pub_phase = rospy.Publisher(
            f'/{self._vehicle_name}/navigation/phase', String, queue_size=1)

        rospy.loginfo(f"[{node_name}] Bereit. {self._total_edges} Kanten zu erkunden.")

    # ── Config laden ─────────────────────────────────────────────────────────

    def _load_map(self):
        path = os.path.join(os.path.dirname(__file__), "../config/mapping_node.json")
        with open(path, 'r') as f:
            config = json.load(f)
        self.graph = config["graph"]

    def _normalize(self, node, exit_tag):
        neighbor, neighbor_tag = self.graph[node][exit_tag]
        neighbor_tag = str(neighbor_tag)
        if node != neighbor:
            return (node, exit_tag) if node <= neighbor else (neighbor, neighbor_tag)
        # Selbstschleife: Knotenvergleich disambiguiert nicht (beide Enden
        # sind derselbe Knoten) - nach Tag normalisieren, damit beide
        # Fahrtrichtungen der Schleife als EINE Kante zaehlen
        # (siehe graph_state_node._mark_visited, muss hier konsistent sein).
        return (node, exit_tag) if exit_tag <= neighbor_tag else (neighbor, neighbor_tag)

    def _count_total_edges(self):
        seen = set()
        for node, exits in self.graph.items():
            for exit_tag in exits.keys():
                seen.add(self._normalize(node, exit_tag))
        return len(seen)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def cbVisitedEdges(self, msg):
        try:
            self.visited_edges = json.loads(msg.data)
        except (ValueError, json.JSONDecodeError):
            rospy.logwarn("[explore_control] Ungueltiges visited_edges-JSON")

    def cbExitDirections(self, msg):
        try:
            self.exit_directions = json.loads(msg.data)
        except (ValueError, json.JSONDecodeError):
            rospy.logwarn("[explore_control] Ungueltiges exit_directions-JSON")

    def cbPhase(self, msg):
        self.phase = msg.data

    def cbIntersectionPhase(self, msg):
        self.intersection_phase = msg.data

    def cbCurrentNode(self, msg):
        self.current_node = msg.data

    def cbResetExploration(self, msg):
        if not msg.data:
            return
        # visited_edges hier direkt leeren (nicht nur auf graph_state_node's
        # naechsten Publish warten) - vermeidet ein kurzes Zeitfenster, in dem
        # der Abschluss-Check unten faelschlich sofort wieder "fertig" meldet,
        # weil visited_edges noch den alten (vollen) Stand zeigt.
        self.visited_edges    = []
        self.exploration_done = False
        self.phase             = "exploration"
        rospy.loginfo("[explore_control] Erkundung neu gestartet")

    # ── DFS-Logik ─────────────────────────────────────────────────────────────

    def _is_visited(self, node, exit_tag):
        return list(self._normalize(node, exit_tag)) in self.visited_edges

    def _first_unvisited_exit(self, node):
        # Rein graph-basiert (fuer die BFS-Zielsuche beim Backtrack - dort ist der
        # Eingangs-Tag bei hypothetischen Knoten ohnehin nicht bekannt).
        for exit_tag in sorted(self.graph.get(node, {}).keys()):
            if not self._is_visited(node, exit_tag):
                return exit_tag
        return None

    def _first_actionable_exit(self, node):
        # Wie _first_unvisited_exit, aber zusaetzlich nur Ausfahrten, fuer die
        # graph_state_node aktuell ein gueltiges Wort liefert. Ohne diesen Filter
        # koennte am current_node eine Ausfahrt gewaehlt werden, deren Tag-ID
        # zufaellig mit dem AKTUELLEN Eingangs-Tag uebereinstimmt (Offset 0) -
        # das waere ein U-Turn zurueck durch dieselbe Einmuendung, fuer den es
        # keine Wort-Richtung gibt. Nur an node == current_node sinnvoll, da
        # exit_directions ausschliesslich fuer den current_node berechnet wird.
        for exit_tag in sorted(self.graph.get(node, {}).keys()):
            if self._is_visited(node, exit_tag):
                continue
            if exit_tag not in self.exit_directions:
                continue
            return exit_tag
        return None

    def _build_adjacency(self):
        # Gerichtete Schritte (node -> [(exit_tag, neighbor), ...]) ueber ALLE
        # Kanten des Graphen - die Topologie steht komplett in
        # mapping_node.json, wird also nicht erst durchs Abfahren "entdeckt".
        # Eine Einschraenkung auf nur bereits befahrene Kanten (frueherer
        # Stand) konnte eine Sackgasse erzeugen: fuehrte der einzige Weg zu
        # unbesuchtem Gebiet ueber eine noch unbesuchte Kante, fand die Suche
        # keinen Pfad -> next_direction blieb dauerhaft leer, Bot haengt fuer
        # immer in STOPPING (siehe fehler.md). Eine noch unbesuchte Kante als
        # Zwischenschritt zu nutzen ist kein Risiko, sondern zaehlt direkt als
        # Fortschritt.
        adjacency = {}
        for node, exits in self.graph.items():
            for exit_tag, (neighbor, _neighbor_tag) in exits.items():
                adjacency.setdefault(node, []).append((exit_tag, neighbor))
        return adjacency

    def _find_backtrack_path(self, start_node):
        # BFS ueber den vollen Graphen zum naechsten Knoten mit unbesuchten
        # Ausgaengen.
        adjacency = self._adjacency
        seen_nodes = {start_node}
        queue = deque([(start_node, [])])
        while queue:
            node, path = queue.popleft()
            if path and self._first_unvisited_exit(node) is not None:
                return path
            for exit_tag, neighbor in adjacency.get(node, []):
                if neighbor not in seen_nodes:
                    seen_nodes.add(neighbor)
                    queue.append((neighbor, path + [exit_tag]))
        return None

    def _decide_next_exit(self):
        # Liefert die als naechstes anzustrebende Ausfahrt-Tag-ID (String) am
        # current_node, oder None wenn (noch) keine Entscheidung moeglich ist.
        node = self.current_node
        if node is None:
            return None

        actionable = self._first_actionable_exit(node)
        if actionable is not None:
            return actionable

        # Keine direkt nutzbare Ausfahrt -> ein Schritt Richtung naechstem
        # Knoten mit unbesuchten Ausgaengen (nur erster Hop; wird bei der
        # naechsten Ankunft automatisch neu berechnet).
        path_tags = self._find_backtrack_path(node)
        if not path_tags:
            return None
        return path_tags[0]

    # ── Hauptschleife ─────────────────────────────────────────────────────────

    def run(self):
        # Phase-Ownership: /navigation/phase wird NUR publiziert, waehrend diese
        # Node aktiv ist (Exploration laeuft), plus ein letztes Mal beim Wechsel
        # zu "waiting". Danach wird die Node auf diesem Topic still, damit
        # path_planner_node (Phase 2/3: "planning"/"delivery") die alleinige
        # Kontrolle uebernehmen kann, ohne dass hier weiterhin "waiting"
        # dagegen publiziert wird.
        rate = rospy.Rate(10)
        just_finished = False
        while not rospy.is_shutdown():
            # intersection_phase == "Stopping" zusaetzlich zur Kantenanzahl:
            # visited_edges erreicht die Gesamtzahl bereits beim START der
            # letzten Abbiegung (graph_state_node._advance_graph markiert
            # sofort bei turn_start, nicht erst nach der Fahrt) - ohne diese
            # Bedingung wuerde "Erkundung abgeschlossen" (Popup, exploration_done)
            # schon melden, waehrend der Bot die letzte Kante noch faehrt/
            # abbiegt, nicht wenn er wirklich an der naechsten Kreuzung steht.
            if not self.exploration_done and len(self.visited_edges) >= self._total_edges \
                    and self.current_node is not None \
                    and self.intersection_phase == "Stopping":
                self.exploration_done = True
                self.phase = "waiting"
                just_finished = True
                rospy.loginfo("[explore_control] Exploration abgeschlossen - alle Kanten besucht, Bot steht")

            if not self.exploration_done:
                exit_tag = self._decide_next_exit()
                word = self.exit_directions.get(exit_tag, "") if exit_tag is not None else ""
                self.pub_next_direction.publish(String(data=word))
                self.pub_phase.publish(String(data=self.phase))
            elif just_finished:
                # next_direction explizit leeren (nicht nur verstummen) -
                # sonst bleibt die Richtung der ALLERLETZTEN Abbiegung bei
                # switch_control_node haengen. Passt sie zufaellig zu den
                # erlaubten Richtungen an der naechsten Kreuzung, wuerde der
                # Bot ungewollt weiterfahren, obwohl die Erkundung fertig ist
                # und noch niemand "Delivery starten" gedrueckt hat.
                self.pub_next_direction.publish(String(data=""))
                self.pub_phase.publish(String(data=self.phase))
                just_finished = False

            self.pub_exploration_done.publish(Bool(data=self.exploration_done))
            rate.sleep()


if __name__ == '__main__':
    node = ExploreControlNode('explore_control_node')
    node.run()
    rospy.spin()
