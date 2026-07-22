#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────────────────────
# path_planner_node.py  (Challenge 4 – Mapping & Path Finding)
#
# Phase 2 (Planung) und Phase 3 (Delivery).
#
# Laedt mapping_node.json DIREKT (wie graph_state_node/explore_control_node) fuer
# die vollstaendige Graph-Topologie. Dijkstra (nur heapq/collections/itertools,
# Kantengewicht 1) berechnet Kuerzeste-Wege-Distanzen zwischen dem
# delivery_start_node und allen Tor-Positionen (aus gate_map).
#
# Die REIHENFOLGE, in der die Tore abgeliefert werden, wird NUR dann selbst
# optimiert (Permutation/Greedy, siehe _plan_optimal/_plan_nearest_neighbor),
# wenn keine externe Vorgabe existiert. Ist eine vorgegebene Reihenfolge
# gesetzt (config path_planning.gate_order, live ueberschreibbar per
# /navigation/gate_order vom debug_graph_node-Dashboard), wird sie
# uebernommen statt neu berechnet - Dijkstra bestimmt dann nur noch den
# kuerzesten WEG zwischen den vorgegebenen Stationen, nicht mehr deren
# Reihenfolge (siehe _plan_fixed_order).
#
# Ein Tor gilt als "Position" wie folgt: gate_map[g] = {"node": N, "tag": T}
# bedeutet, das Tor liegt auf der Kante die man von N aus ueber Tag T befaehrt.
# Um das Tor abzuliefern, muss der Bot also (a) nach N navigieren und (b) dort
# exakt Tag T nehmen - das ist immer genau 1 Schritt mehr als die
# Dijkstra-Distanz zu N. Nach dem Abliefern steht der Bot am anderen Ende
# dieser Kante (graph[N][T][0]).
#
# Tag-ID -> Wort: wie explore_control_node - Uebersetzung ueber
# /graph/exit_directions (von graph_state_node). Die naechste Ausfahrt wird,
# wie dort, JEDEN Tick frisch aus dem aktuellen Zustand neu berechnet (robust
# gegen Zeitverzoegerungen), NICHT einmalig zwischengespeichert.
#
# Phase-Ownership: /navigation/phase wird nur waehrend aktiver Delivery
# publiziert (Phase 3). Die "Planung" (Phase 2) laeuft im Hintergrund bereits
# waehrend phase=="waiting" (von explore_control_node gesetzt), damit der
# geplante Pfad im Debug-Fenster VOR dem Druecken von "Delivery starten"
# bereits sichtbar ist (delivery_progress wird immer publiziert).
# ─────────────────────────────────────────────────────────────────────────────

import heapq
import itertools
import json
import os
import traceback
import rospy
from std_msgs.msg import String, Bool, Int32


class PathPlannerNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        self._vehicle_name = os.environ['VEHICLE_NAME']

        self._load_map()

        # Vorgegebene Tor-Reihenfolge (leer = keine Vorgabe, Reihenfolge wird
        # selbst optimiert). Aus der Config vorbelegt, per Dashboard live
        # ueberschreibbar (cbGateOrder).
        self.fixed_gate_order = list(self.path_planning.get("gate_order", []))
        # Tore aus fixed_gate_order, die noch nicht gefunden wurden - vom
        # Dashboard angezeigt, damit klar ist, WORAUF gewartet wird, statt
        # dass "Delivery starten" wirkungslos bleibt (siehe _plan_fixed_order).
        self.missing_gates = []

        self.gate_map          = {}
        self.current_node      = None
        self.current_edge      = None
        # Tag-Bestaetigung fuer das aktuell dran befindliche Tor (remaining[0])
        # auf der aktuell befahrenen Kante - siehe cbGateDetected/_check_delivered.
        self._gate_confirmed_this_edge = False
        self.exit_directions   = {}
        # Sicherer Default bis die erste echte /navigation/phase-Nachricht eintrifft:
        # "exploration" (wie explore_control_node selbst startet), NICHT "waiting" -
        # sonst koennte die Planungssperre (phase != "exploration") in einem sehr
        # kurzen Fenster direkt nach dem Start faelschlich durchlaessig sein.
        self.phase             = "exploration"
        self.start_delivery_requested = False

        self.planned_order = []   # Liste von Gate-IDs (String), geplante Reihenfolge
        self.remaining     = []
        self.delivered     = []
        self.delivery_active = False
        self._last_planned_keys = None
        # Bot wird zwischen Erkundung und Abfahrt von Hand an delivery_start_node
        # neu hingestellt (Dashboard-Button "Bot versetzt", siehe graph_state_node.
        # cbBotRelocated) - erst NACH diesem Signal darf geplant/losgefahren
        # werden, sonst wuerde die Route noch ab der alten Erkundungs-Endposition
        # berechnet bzw. die Abfahrt sofort mit einer veralteten Position starten.
        self.bot_relocated_confirmed = False

        rospy.Subscriber(f'/{self._vehicle_name}/graph/gate_map',
                         String, self.cbGateMap, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/graph/current_node',
                         String, self.cbCurrentNode, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/graph/current_edge',
                         String, self.cbCurrentEdge, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/graph/exit_directions',
                         String, self.cbExitDirections, queue_size=1)
        # Fuer die Abliefer-Bestaetigung: dasselbe Topic, das graph_state_node
        # fuer den Aufbau der gate_map nutzt (siehe cbGateDetected).
        rospy.Subscriber(f'/{self._vehicle_name}/detect/gate/id',
                         Int32, self.cbGateDetected, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/navigation/start_delivery',
                         Bool, self.cbStartDelivery, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/navigation/phase',
                         String, self.cbPhase, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/navigation/gate_order',
                         String, self.cbGateOrder, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/graph/bot_relocated',
                         Bool, self.cbBotRelocated, queue_size=1)

        self.pub_next_direction = rospy.Publisher(
            f'/{self._vehicle_name}/navigation/next_direction', String, queue_size=1)
        self.pub_phase = rospy.Publisher(
            f'/{self._vehicle_name}/navigation/phase', String, queue_size=1)
        self.pub_delivery_progress = rospy.Publisher(
            f'/{self._vehicle_name}/navigation/delivery_progress', String, queue_size=1)

        rospy.loginfo(f"[{node_name}] Bereit. Delivery-Start: {self.delivery_start_node}")

    # ── Config laden ─────────────────────────────────────────────────────────

    def _load_map(self):
        path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../config/mapping_node.json"))
        # Absoluten Pfad mitloggen: bei Catkin-Installs/Devel-Space kann die
        # tatsaechlich geladene Datei von der abweichen, die von Hand editiert
        # wird (z.B. eine installierte Kopie statt der Quelle) - dann wuerde
        # eine Aenderung an delivery_start_node nie ankommen, obwohl die
        # editierte Datei korrekt aussieht.
        rospy.loginfo(f"[path_planner] Lade mapping_node.json von: {path}")
        with open(path, 'r') as f:
            config = json.load(f)
        self.graph               = config["graph"]
        self.delivery_start_node = config.get("delivery_start_node", config["mapping_start_node"])
        self.path_planning       = config.get("path_planning", {})

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def cbGateMap(self, msg):
        try:
            self.gate_map = json.loads(msg.data) if msg.data else {}
        except (ValueError, json.JSONDecodeError):
            rospy.logwarn("[path_planner] Ungueltiges gate_map-JSON")

    def cbCurrentNode(self, msg):
        self.current_node = msg.data

    def cbCurrentEdge(self, msg):
        try:
            new_edge = json.loads(msg.data) if msg.data else None
        except (ValueError, json.JSONDecodeError):
            return
        if new_edge is None and self.current_edge is not None:
            # Kante gerade fertig befahren (Bot steht an der naechsten
            # Kreuzung) - Fallback: falls das aktuelle Ziel-Tor auf genau
            # dieser Kante lag und sein Tag waehrend der Fahrt NICHT per
            # cbGateDetected bestaetigt wurde (Erkennung z.B. durch
            # Lichtverhaeltnisse/Verdeckung fehlgeschlagen), jetzt trotzdem
            # als abgefahren werten statt dauerhaft auf eine Bestaetigung zu
            # warten, die nie kommt.
            self._check_delivered_fallback(self.current_edge)
        self.current_edge = new_edge
        self._gate_confirmed_this_edge = False

    def cbGateDetected(self, msg):
        # Primaerer Bestaetigungsweg: das Ziel-Tor (remaining[0]) wird per
        # AprilTag WAEHREND der Fahrt ueber seine Kante erneut gesehen - erst
        # DANN gilt es als abgefahren, nicht schon weil die Kante "irgendwie"
        # zur Position passt (das war der Bug: _check_delivered pruefte
        # frueher ALLE verbleibenden Tore gegen current_edge, wodurch auch ein
        # spaeter in der Reihenfolge faelliges Tor faelschlich sofort
        # abgehakt wurde, sobald seine Kante nur als Durchgangsstrecke
        # befahren wurde - das hat u.a. die vorgegebene Reihenfolge
        # durcheinandergebracht und zwei Tore auf derselben Kante gleichzeitig
        # "geliefert").
        gate_id = msg.data
        if gate_id == -1 or not self.remaining or self.current_edge is None:
            return
        target_gate = self.remaining[0]
        if str(gate_id) != str(target_gate):
            return
        try:
            # Beide Zustellmoeglichkeiten pruefen (Original + Gegenrichtung,
            # siehe _gate_delivery_options) - der Bot kann ueber jede von
            # beiden ankommen.
            options = self._gate_delivery_options(target_gate)
        except KeyError:
            rospy.logerr(f"[path_planner] Tor {target_gate} nicht (mehr) in gate_map "
                         f"gefunden - ueberspringe Bestaetigung diesen Tick.")
            return
        for node, tag in options:
            if self.current_edge.get("from") == node and self.current_edge.get("tag") == tag:
                self._gate_confirmed_this_edge = True
                self._mark_delivered(target_gate, via="Tag erneut bestaetigt")
                return

    def _mark_delivered(self, gate_id, via):
        if gate_id not in self.remaining:
            return
        self.remaining.remove(gate_id)
        self.delivered.append(gate_id)
        rospy.loginfo(f"[path_planner] Tor {gate_id} abgefahren ({via}) "
                      f"({len(self.delivered)}/{len(self.planned_order)})")

    def _check_delivered_fallback(self, finished_edge):
        if self._gate_confirmed_this_edge or not self.remaining:
            return
        target_gate = self.remaining[0]
        try:
            options = self._gate_delivery_options(target_gate)
        except KeyError:
            rospy.logerr(f"[path_planner] Tor {target_gate} nicht (mehr) in gate_map "
                         f"gefunden - ueberspringe Fallback-Pruefung diesen Tick.")
            return
        for node, tag in options:
            if finished_edge.get("from") == node and finished_edge.get("tag") == tag:
                rospy.logwarn(f"[path_planner] Tor {target_gate} waehrend der Fahrt "
                              f"nicht per Tag bestaetigt - werte Kante trotzdem als "
                              f"abgefahren (Fallback).")
                self._mark_delivered(target_gate, via="Fallback: Kante befahren, Tag nicht gesehen")
                return

    def cbExitDirections(self, msg):
        try:
            self.exit_directions = json.loads(msg.data)
        except (ValueError, json.JSONDecodeError):
            rospy.logwarn("[path_planner] Ungueltiges exit_directions-JSON")

    def cbStartDelivery(self, msg):
        if msg.data:
            self.start_delivery_requested = True

    def cbPhase(self, msg):
        if not self.delivery_active:
            self.phase = msg.data

    def cbGateOrder(self, msg):
        try:
            new_order = json.loads(msg.data) if msg.data else []
        except (ValueError, json.JSONDecodeError):
            rospy.logwarn("[path_planner] Ungueltiges gate_order-JSON")
            return
        if new_order != self.fixed_gate_order:
            self.fixed_gate_order = new_order
            # Neuplanung erzwingen, auch wenn sich die Menge der gefundenen
            # Tore seit der letzten Planung nicht geaendert hat.
            self._last_planned_keys = None
            rospy.loginfo(f"[path_planner] Neue vorgegebene Reihenfolge: {new_order}")

    def cbBotRelocated(self, msg):
        if not msg.data:
            return
        # delivery_start_node wurde sonst nur EINMAL beim Node-Start aus
        # mapping_node.json gelesen (_load_map) - eine nachtraegliche
        # Bearbeitung der Datei (z.B. Bot wird diesmal an einem anderen
        # Knoten hingestellt) kaeme sonst nie an, obwohl "Bot versetzt"
        # genau dafuer der richtige Zeitpunkt waere.
        try:
            with open(os.path.join(os.path.dirname(__file__),
                                    "../config/mapping_node.json"), 'r') as f:
                config = json.load(f)
            self.delivery_start_node = config.get(
                "delivery_start_node", config["mapping_start_node"])
        except Exception as e:
            rospy.logwarn(f"[path_planner] delivery_start_node konnte nicht neu "
                          f"geladen werden, nutze zuletzt bekannten Stand: {e}")
        self.bot_relocated_confirmed = True
        # Neuplanung erzwingen: current_node (graph_state_node) ist ab jetzt
        # per Reset auf delivery_start_node gesetzt, eine vorher (waehrend
        # phase=="waiting", aber noch VOR der Neupositionierung) berechnete
        # planned_order koennte von der alten Erkundungs-Endposition ausgehen.
        self._last_planned_keys = None
        rospy.loginfo(f"[path_planner] Bot-Neupositionierung bestaetigt - Route "
                      f"wird ab delivery_start_node={self.delivery_start_node} (neu) geplant")

    # ── Dijkstra (nur heapq/collections/itertools) ──────────────────────────────

    def _dijkstra(self, start):
        dist = {start: 0}
        prev = {}
        pq = [(0, start)]
        done = set()
        while pq:
            d, node = heapq.heappop(pq)
            if node in done:
                continue
            done.add(node)
            for tag, (neighbor, _neighbor_tag) in self.graph.get(node, {}).items():
                nd = d + 1
                if nd < dist.get(neighbor, float('inf')):
                    dist[neighbor] = nd
                    prev[neighbor] = (node, tag)
                    heapq.heappush(pq, (nd, neighbor))
        return dist, prev

    def _dijkstra_excluding_start_exit(self, start, forbidden_tag):
        # Wie _dijkstra, aber OHNE die triviale Distanz 0 fuer start selbst:
        # normales Dijkstra wuerde bei einem Ziel direkt an start immer 0
        # (bzw. +1 fuer den letzten Schritt) liefern, selbst wenn die EINZIGE
        # direkte Kante dorthin forbidden_tag waere (eine Wende, fuer die es
        # kein Wort/keine Ausfuehrung gibt, siehe _gate_distance/
        # _decide_next_tag). Hier zaehlt nur ein echter Umweg ueber die
        # UEBRIGEN Ausfahrten von start.
        dist = {}
        prev = {}
        pq = []
        for tag, (neighbor, _neighbor_tag) in self.graph.get(start, {}).items():
            if tag == forbidden_tag:
                continue
            if 1 < dist.get(neighbor, float('inf')):
                dist[neighbor] = 1
                prev[neighbor] = (start, tag)
                heapq.heappush(pq, (1, neighbor))
        done = set()
        while pq:
            d, node = heapq.heappop(pq)
            if node in done:
                continue
            done.add(node)
            for tag, (neighbor, _neighbor_tag) in self.graph.get(node, {}).items():
                nd = d + 1
                if nd < dist.get(neighbor, float('inf')):
                    dist[neighbor] = nd
                    prev[neighbor] = (node, tag)
                    heapq.heappush(pq, (nd, neighbor))
        return dist, prev

    def _shortest_path_tags(self, start, goal):
        # Liste der Tag-IDs (String) fuer jede Kreuzung von start bis goal.
        # None wenn goal nicht erreichbar, [] wenn start == goal.
        if start == goal:
            return []
        dist, prev = self._dijkstra(start)
        if goal not in dist:
            return None
        tags = []
        node = goal
        while node != start:
            p_node, tag = prev[node]
            tags.append(tag)
            node = p_node
        tags.reverse()
        return tags

    def _gate_entry_and_exit(self, gate_id):
        entry = self.gate_map[gate_id]
        node, tag = entry["node"], entry["tag"]
        exit_node = self.graph[node][tag][0]
        return node, tag, exit_node

    def _gate_delivery_options(self, gate_id):
        # Der Tor-Tag ist von BEIDEN Fahrtrichtungen der Kante aus sichtbar
        # (vor Ort bestaetigt, 2026-07-22) - gate_map speichert aber nur die
        # bei der Erkundung ZUERST gesehene Richtung. Die Graph-Topologie ist
        # durchgehend symmetrisch (N--T-->M bedeutet immer auch M--T'-->N
        # fuer ein festes T', siehe mapping_node.json), daher ist die exakte
        # Gegenrichtung derselben Kante eine ebenso gueltige, oft naehere
        # Zustellmoeglichkeit fuer dasselbe Tor. Liefert alle bekannten
        # Zustellmoeglichkeiten als (entry_node, tag)-Paare.
        node, tag = self.gate_map[gate_id]["node"], self.gate_map[gate_id]["tag"]
        options = [(node, tag)]
        exit_node, mirror_tag = self.graph[node][tag]
        mirror_target = self.graph.get(exit_node, {}).get(mirror_tag)
        if mirror_target and mirror_target[0] == node and (exit_node, mirror_tag) != (node, tag):
            options.append((exit_node, mirror_tag))
        return options

    def _forbidden_first_tag(self):
        # Der aktuelle Einfahrt-Tag ist als Ausfahrt an current_node verboten
        # (reine Wende, kein Wort/keine Ausfuehrung, Offset 0 in
        # graph_state_node._word_for_exit) - erkennbar daran, dass GENAU
        # dieser eine Tag im ansonsten vollstaendigen exit_directions fehlt.
        # Nicht eindeutig bestimmbar (0 oder >1 fehlende Tags) -> None,
        # dann greift keine Einschraenkung (sicherer Fallback statt einer
        # falschen Annahme).
        if self.current_node is None:
            return None
        all_tags = set(self.graph.get(self.current_node, {}).keys())
        missing = all_tags - set(self.exit_directions.keys())
        if len(missing) == 1:
            return next(iter(missing))
        return None

    def _route_to_gate_edge(self, from_node, entry_node, tag):
        # Liefert (erster_abzweig_tag, gesamt_distanz) um GENAU dieses
        # (entry_node, tag)-Zustellziel von from_node aus zu erreichen, oder
        # (None, None) wenn nicht erreichbar.
        #
        # WICHTIG: die "verbotene Wende"-Einschraenkung gilt fuer JEDE Route
        # ab der tatsaechlichen aktuellen Position, nicht nur wenn das Ziel
        # zufaellig direkt am current_node liegt - auch bei einem ENTFERNTEN
        # Ziel kann der kuerzeste Weg als allerersten Schritt genau die Kante
        # nehmen, ueber die gerade eingefahren wurde (z.B. kuerzester Weg
        # B->C ist 1 Schritt genau ueber den Einfahrt-Tag von B) - das ist
        # ebenso eine nicht ausfuehrbare Wende und muss hier genauso
        # ausgeschlossen werden wie im direkten Sonderfall.
        forbidden = self._forbidden_first_tag() if from_node == self.current_node else None
        if from_node == entry_node and tag != forbidden:
            return tag, 1
        if forbidden is not None:
            dist, prev = self._dijkstra_excluding_start_exit(from_node, forbidden)
        else:
            dist, prev = self._dijkstra(from_node)
        if entry_node not in dist:
            return None, None
        node, path_tags = entry_node, []
        for _ in range(dist[entry_node]):
            p_node, t = prev[node]
            path_tags.append(t)
            node = p_node
        if not path_tags:
            return None, None
        path_tags.reverse()
        return path_tags[0], dist[entry_node] + 1

    def _gate_distance(self, from_node, gate_id):
        best = None
        for entry_node, tag in self._gate_delivery_options(gate_id):
            _first_tag, d = self._route_to_gate_edge(from_node, entry_node, tag)
            if d is not None and (best is None or d < best):
                best = d
        return best

    # ── Reihenfolge-Planung ──────────────────────────────────────────────────

    def _plan_optimal(self, start_node, gate_ids):
        best_order, best_dist = None, None
        for perm in itertools.permutations(gate_ids):
            total, pos, feasible = 0, start_node, True
            for g in perm:
                d = self._gate_distance(pos, g)
                if d is None:
                    feasible = False
                    break
                total += d
                _, _, pos = self._gate_entry_and_exit(g)
            if feasible and (best_dist is None or total < best_dist):
                best_dist, best_order = total, perm
        return list(best_order) if best_order is not None else list(gate_ids)

    def _plan_nearest_neighbor(self, start_node, gate_ids):
        remaining, pos, order = list(gate_ids), start_node, []
        while remaining:
            best_g, best_d = None, None
            for g in remaining:
                d = self._gate_distance(pos, g)
                if d is not None and (best_d is None or d < best_d):
                    best_g, best_d = g, d
            if best_g is None:
                order.extend(remaining)  # Rest nicht erreichbar - unveraendert anhaengen
                break
            order.append(best_g)
            remaining.remove(best_g)
            _, _, pos = self._gate_entry_and_exit(best_g)
        return order

    def _plan_fixed_order(self, gate_ids):
        # Reihenfolge wird extern vorgegeben (Config gate_order bzw. live
        # per /navigation/gate_order vom Dashboard), NICHT mehr selbst
        # optimiert. Solange nicht ALLE vorgegebenen Tore tatsaechlich
        # gefunden wurden, wird bewusst NOCH KEINE Route geplant (leere
        # Liste) - der Bot soll erst losfahren, wenn die komplette Vorgabe
        # erfuellbar ist, statt mit einer unvollstaendigen/umsortierten Route
        # zu starten. run() versucht es bei jedem neu gefundenen Tor erneut
        # (siehe current_keys-Vergleich).
        self.missing_gates = [g for g in self.fixed_gate_order if g not in gate_ids]
        if self.missing_gates:
            rospy.logwarn_throttle(5.0,
                f"[path_planner] Warte auf Tor(e) {self.missing_gates} aus der "
                f"vorgegebenen Reihenfolge - noch nicht gefunden, Route wird "
                f"noch nicht geplant")
            return []
        extra = [g for g in gate_ids if g not in self.fixed_gate_order]
        return list(self.fixed_gate_order) + extra

    def _compute_order(self, start_node, gate_ids):
        self.missing_gates = []
        if self.fixed_gate_order:
            return self._plan_fixed_order(gate_ids)
        mode = self.path_planning.get("mode", "nearest_neighbor")
        if mode == "optimal" and len(gate_ids) > 10:
            fallback = self.path_planning.get("fallback", "nearest_neighbor")
            rospy.logwarn(f"[path_planner] {len(gate_ids)} Tore > 10 -> "
                          f"wechsle von 'optimal' auf '{fallback}'")
            mode = fallback
        if mode == "optimal":
            return self._plan_optimal(start_node, gate_ids)
        return self._plan_nearest_neighbor(start_node, gate_ids)

    # ── Delivery-Ausfuehrung ──────────────────────────────────────────────────
    # (Abhak-Logik: cbGateDetected/_check_delivered_fallback weiter oben bei
    # den Callbacks, direkt neben cbCurrentEdge mit dem sie zusammenspielen.)

    def _decide_next_tag(self):
        if not self.remaining or self.current_node is None:
            return None
        target_gate = self.remaining[0]
        best_tag, best_dist = None, None
        for entry_node, tag in self._gate_delivery_options(target_gate):
            first_tag, d = self._route_to_gate_edge(self.current_node, entry_node, tag)
            if first_tag is not None and (best_dist is None or d < best_dist):
                best_tag, best_dist = first_tag, d
        return best_tag

    # ── Hauptschleife ─────────────────────────────────────────────────────────

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                self._tick()
            except Exception:
                # Niemals den ganzen Node lautlos sterben lassen (z.B. bei
                # einer unerwarteten KeyError/IndexError in der Planung) -
                # sonst bleibt naechste_direction fuer immer auf dem letzten
                # Stand haengen (meist ""), ohne dass irgendwo im Log
                # ersichtlich wird, WARUM der Bot nicht mehr weiterfaehrt.
                # Lieber den Fehler laut melden und im naechsten Tick erneut
                # versuchen.
                rospy.logerr(f"[path_planner] Unerwarteter Fehler im Haupt-Tick:\n"
                             f"{traceback.format_exc()}")
            rate.sleep()

    def _tick(self):
        if not self.delivery_active and self.phase != "exploration":
            # Route wird ERST geplant, nachdem "Bot versetzt" bestaetigt
            # wurde (siehe cbBotRelocated/graph_state_node.cbBotRelocated):
            # der Bot wird zwischen Erkundung und Abfahrt von Hand an
            # delivery_start_node neu hingestellt, ohne dass die Software
            # das sonst mitbekommt - eine vorher (schon waehrend
            # phase=="waiting") berechnete Reihenfolge ginge faelschlich
            # noch von der alten Erkundungs-Endposition aus.
            if self.bot_relocated_confirmed:
                current_keys = frozenset(self.gate_map.keys())
                if current_keys and current_keys != self._last_planned_keys:
                    self.planned_order = self._compute_order(
                        self.delivery_start_node, list(current_keys))
                    self.remaining = list(self.planned_order)
                    self.delivered = []
                    self._last_planned_keys = current_keys
                    rospy.loginfo(f"[path_planner] Geplante Reihenfolge: {self.planned_order}")

                if self.start_delivery_requested and self.planned_order:
                    self.delivery_active = True
                    self.phase = "delivery"
                    rospy.loginfo("[path_planner] Delivery gestartet")
            elif self.start_delivery_requested:
                rospy.logwarn_throttle(5.0,
                    "[path_planner] 'Delivery starten' gedrueckt, aber 'Bot "
                    "versetzt' wurde noch nicht bestaetigt - warte darauf, "
                    "um nicht von der alten Erkundungs-Position aus loszufahren.")

        if self.delivery_active:
            exit_tag = self._decide_next_tag()
            word = self.exit_directions.get(exit_tag, "") if exit_tag is not None else ""
            if not word and self.remaining:
                # Sichtbar machen, WORAN next_direction gerade haengt, statt
                # stillschweigend "" zu publizieren (siehe Kopfkommentar zu
                # run()) - haeufigster Grund: exit_directions noch nicht fuer
                # den aktuellen current_node aktualisiert (kurze Verzoegerung
                # nach einer Abbiegung, sollte sich von selbst loesen).
                rospy.logwarn_throttle(2.0,
                    f"[path_planner] Keine next_direction berechenbar - Ziel-Tor "
                    f"{self.remaining[0]}, current_node={self.current_node}, "
                    f"exit_directions={self.exit_directions}")
            self.pub_next_direction.publish(String(data=word))
            self.pub_phase.publish(String(data=self.phase))

        self.pub_delivery_progress.publish(String(data=json.dumps({
            "done": self.delivered,
            "remaining": self.remaining,
            "planned_order": self.planned_order,
            "missing_gates": self.missing_gates,
            "bot_relocated_confirmed": self.bot_relocated_confirmed,
        })))


if __name__ == '__main__':
    node = PathPlannerNode('path_planner_node')
    node.run()
    rospy.spin()
