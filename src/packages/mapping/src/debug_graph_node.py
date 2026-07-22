#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────────────────────
# debug_graph_node.py  (Challenge 4 – Mapping & Path Finding)
#
# tkinter-Dashboard mit Echtzeit-Visualisierung: erstellte Karte (abgefahrene
# Kanten), gewaehlter Delivery-Pfad, aktuelle Bot-Position.
#
# Laedt mapping_node.json DIREKT fuer die statische Graph-Topologie und das
# Layout (node_positions bzw. automatisches Kreislayout). Der Delivery-Pfad
# (Ebene 3) wird aus der geplanten Tor-Reihenfolge (delivery_progress) mit
# einer lokalen Dijkstra-Berechnung in eine Knotenfolge uebersetzt - rein fuer
# die Visualisierung, unabhaengig von path_planner_node's eigener Planung.
#
# ROS-Callbacks aktualisieren AUSSCHLIESSLICH State-Variablen. Das Zeichnen
# passiert ausschliesslich in update_canvas() im Hauptthread (root.after).
# ─────────────────────────────────────────────────────────────────────────────

import heapq
import json
import math
import os
import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
import cv2
import rospy
from std_msgs.msg import String, Bool


class DebugGraphNode:
    GATE_COLORS = {
        5: "#FF00FF", 6: "#00FFFF", 7: "#FF8800", 8: "#FFFF00", 9: "#FF0000",
        10: "#AA00FF", 11: "#88FF00", 12: "#FF44AA", 13: "#00FFAA",
    }

    NODE_RADIUS = 20
    SELF_LOOP_RADIUS = 18   # Radius des kleinen Loop-Kreises fuer Selbstschleifen
    SELF_LOOP_GAP = 4       # Abstand zwischen Knoten-Kreis und Loop-Kreis

    def __init__(self, node_name):
        rospy.init_node(node_name)
        self._vehicle_name = os.environ['VEHICLE_NAME']

        self._load_map()
        self.edges = self._build_edges()
        self.node_positions = self._compute_node_positions()

        # ── State (wird NUR von ROS-Callbacks geschrieben) ─────────────────────
        self.current_node      = ""
        self.current_edge      = None
        self.visited_edges     = []
        self.gate_map          = {}
        self.phase              = "exploration"
        self.exploration_done   = False
        self.delivery_progress  = {"done": [], "remaining": [], "planned_order": [],
                                    "bot_relocated_confirmed": False}
        # FSM-Phase (Lane/Stopping/Turning) + gewaehlte Richtung - fuer die
        # Live-Anzeige "biegt gerade wohin ab" beim Bot-Symbol.
        self.intersection_phase = "Lane"
        self.intersection_dir   = ""

        self._last_drawn_planned_order = None
        self._exploration_done_notified = False

        rospy.Subscriber(f'/{self._vehicle_name}/graph/current_node',
                         String, self.cbCurrentNode, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/graph/current_edge',
                         String, self.cbCurrentEdge, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/graph/visited_edges',
                         String, self.cbVisitedEdges, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/graph/gate_map',
                         String, self.cbGateMap, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/navigation/phase',
                         String, self.cbPhase, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/navigation/exploration_done',
                         Bool, self.cbExplorationDone, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/navigation/delivery_progress',
                         String, self.cbDeliveryProgress, queue_size=1)
        rospy.Subscriber(f'/{self._vehicle_name}/intersection/phase',
                         String, self.cbIntersectionPhase, queue_size=1)
        # turn_start statt des race-anfaelligen /intersection/direction (siehe
        # graph_state_node/control_intersection_node) - liefert die Richtung
        # atomar mit dem Sequenzstart, damit "biegt: X" nie eine veraltete
        # Richtung der vorherigen Abbiegung anzeigt.
        rospy.Subscriber(f'/{self._vehicle_name}/intersection/turn_start',
                         String, self.cbIntersectionDirection, queue_size=1)

        self.pub_start_delivery = rospy.Publisher(
            f'/{self._vehicle_name}/navigation/start_delivery', Bool, queue_size=1)
        # latch=True: path_planner_node soll die zuletzt eingetragene Vorgabe
        # auch dann sehen, wenn es NACH diesem Publish-Aufruf startet/neu
        # verbindet (Reihenfolge in der GUI eingetragen, bevor path_planner
        # laeuft) - zusaetzlich zur Persistierung in mapping_node.json.
        self.pub_gate_order = rospy.Publisher(
            f'/{self._vehicle_name}/navigation/gate_order', String, queue_size=1, latch=True)
        # Notfall-Korrektur: mapping_node.json (Feld "gate_map") von Hand
        # editiert (Tor-ID getauscht/entfernt) - dieser Button laesst
        # graph_state_node sie neu einlesen, OHNE current_node/visited_edges
        # zu verlieren (kein Neustart der Exploration noetig).
        self.pub_reload_gate_map = rospy.Publisher(
            f'/{self._vehicle_name}/graph/reload_gate_map', Bool, queue_size=1)
        # Erkundung neu starten (z.B. Tor uebersehen) - setzt visited_edges
        # zurueck, current_node und gate_map bleiben erhalten.
        self.pub_reset_exploration = rospy.Publisher(
            f'/{self._vehicle_name}/graph/reset_exploration', Bool, queue_size=1)
        # Bot wird zwischen Erkundung und Abfahrt von Hand an delivery_start_node
        # neu hingestellt (z.B. um einen eulerischen Kantenzug bei der Erkundung
        # sicherzustellen) - dieser Button bestaetigt das gegenueber graph_state_
        # node/path_planner_node, die sonst nichts davon mitbekommen wuerden
        # (current_node bliebe sonst auf dem Stand vom Ende der Erkundung).
        self.pub_bot_relocated = rospy.Publisher(
            f'/{self._vehicle_name}/graph/bot_relocated', Bool, queue_size=1)

        self._build_gui()
        rospy.loginfo(f"[{node_name}] Bereit.")

    # ── Config / Graph laden ──────────────────────────────────────────────────

    def _load_map(self):
        path = os.path.join(os.path.dirname(__file__), "../config/mapping_node.json")
        with open(path, 'r') as f:
            config = json.load(f)
        self.graph               = config["graph"]
        self.delivery_start_node = config.get("delivery_start_node", config["mapping_start_node"])
        self._gate_order_cfg     = config.get("path_planning", {}).get("gate_order", [])
        self._node_positions_cfg = config.get("debug_layout", {}).get("node_positions", {})
        # Wegpunkte je Knotenpaar (Schluessel "A-C", alphabetisch sortiert),
        # damit Kanten dem echten, gebogenen Streckenverlauf folgen statt
        # einer geraden Linie. Mehrere Kanten zwischen denselben zwei Knoten
        # teilen sich dieselben Wegpunkte (rein visuell, keine Odometrie-Daten).
        self._edge_waypoints_cfg = config.get("debug_layout", {}).get("edge_waypoints", {})

    def _build_edges(self):
        seen = set()
        edges = []
        for node, exits in self.graph.items():
            for tag, (neighbor, neighbor_tag) in exits.items():
                neighbor_tag = str(neighbor_tag)
                if node != neighbor:
                    if node <= neighbor:
                        key = (node, tag)
                        a_node, a_tag, b_node, b_tag = node, tag, neighbor, neighbor_tag
                    else:
                        key = (neighbor, neighbor_tag)
                        a_node, a_tag, b_node, b_tag = neighbor, neighbor_tag, node, tag
                else:
                    # Selbstschleife (z.B. Wendeschleife): Knotenvergleich
                    # disambiguiert nicht - nach Tag normalisieren, damit beide
                    # Richtungen als EINE Kante gezeichnet werden (konsistent
                    # mit graph_state_node/explore_control_node).
                    if tag <= neighbor_tag:
                        key = (node, tag)
                        a_node, a_tag, b_node, b_tag = node, tag, neighbor, neighbor_tag
                    else:
                        key = (neighbor, neighbor_tag)
                        a_node, a_tag, b_node, b_tag = neighbor, neighbor_tag, node, tag
                if key in seen:
                    continue
                seen.add(key)
                edges.append({"node_a": a_node, "tag_a": a_tag, "node_b": b_node, "tag_b": b_tag})
        return edges

    def _compute_node_positions(self):
        # Kreislayout immer als Basis berechnen (nicht nur wenn node_positions
        # komplett leer ist). Konfigurierte Positionen ueberschreiben dann nur
        # die genannten Knoten - fehlt in mapping_node.json versehentlich ein
        # einzelner Knoten, faellt er auf seine Kreislayout-Position zurueck
        # statt beim Start mit KeyError abzustuerzen.
        nodes = sorted(self.graph.keys())
        n = len(nodes)
        cx, cy, r = 450, 300, 220
        positions = {}
        for i, node in enumerate(nodes):
            angle = math.radians(360.0 * i / n) if n else 0.0
            positions[node] = (cx + r * math.sin(angle), cy - r * math.cos(angle))
        for node, pos in self._node_positions_cfg.items():
            if node in positions:
                positions[node] = tuple(pos)
        return positions

    def _any_edge_for_pair(self, node_a, node_b):
        # Fuer Stellen, an denen nur eine Knoten-Paarung (kein konkretes Tag)
        # bekannt ist (z.B. der visualisierte Delivery-Pfad) - liefert
        # irgendeine der ggf. mehreren parallelen Kanten zwischen den beiden.
        pair = tuple(sorted((node_a, node_b)))
        for edge in self.edges:
            if edge["node_a"] == edge["node_b"]:
                continue
            if tuple(sorted((edge["node_a"], edge["node_b"]))) == pair:
                return edge
        return None

    def _edge_waypoints(self, edge):
        # Wegpunkte sind pro KONKRETER Kante hinterlegt (Schluessel z.B. "A3-C1"),
        # nicht pro Knotenpaar: gibt es zwischen zwei Knoten mehrere Kanten
        # (unterschiedliche Tags), bleibt in der Regel nur EINE davon gebogen -
        # das allein trennt sie schon optisch von der geraden Gegenkante,
        # ganz ohne zusaetzlichen Versatz.
        key = f'{edge["node_a"]}{edge["tag_a"]}-{edge["node_b"]}{edge["tag_b"]}'
        return [tuple(p) for p in self._edge_waypoints_cfg.get(key, [])]

    def _edge_control_points(self, edge):
        # Volle Kontrollpunktliste dieser Kante: [Start, Wegpunkt(e)..., Ende].
        # Ohne konfigurierte Wegpunkte nur Start/Ende (= gerade Linie).
        node_a, node_b = edge["node_a"], edge["node_b"]
        return [self.node_positions[node_a]] + self._edge_waypoints(edge) + [self.node_positions[node_b]]

    def _edge_line_points(self, edge):
        # Flache Punktliste [x1,y1, wp1x,wp1y, ..., x2,y2] fuer create_line().
        pts = []
        for x, y in self._edge_control_points(edge):
            pts.extend([x, y])
        return pts

    def _point_on_edge(self, edge, t):
        # Punkt bei Parameter t in [0,1] auf der tatsaechlich gezeichneten
        # (ggf. gebogenen) Kante - fuer die Tag-Label-Platzierung, damit ein
        # Label auch bei gebogenen Kanten auf der Linie sitzt statt auf der
        # gedachten Geraden. Gerade Linie: lineare Interpolation. Ein
        # Wegpunkt (Regelfall hier): quadratische Bezier-Kurve, entspricht
        # ungefaehr tkinters smooth=True. Mehr als ein Wegpunkt: stueckweise
        # linear ueber die Kontrollpunkte (grobe Naeherung, aktuell ungenutzt).
        pts = self._edge_control_points(edge)
        if len(pts) == 2:
            (x0, y0), (x1, y1) = pts
            return x0 + (x1 - x0) * t, y0 + (y1 - y0) * t
        if len(pts) == 3:
            (x0, y0), (x1, y1), (x2, y2) = pts
            mt = 1.0 - t
            return (mt * mt * x0 + 2 * mt * t * x1 + t * t * x2,
                    mt * mt * y0 + 2 * mt * t * y1 + t * t * y2)
        seg_count = len(pts) - 1
        pos = min(max(t, 0.0), 1.0) * seg_count
        idx = min(int(pos), seg_count - 1)
        local_t = pos - idx
        x0, y0 = pts[idx]
        x1, y1 = pts[idx + 1]
        return x0 + (x1 - x0) * local_t, y0 + (y1 - y0) * local_t

    def _edge_anchor_point(self, edge):
        # Punkt zum Platzieren von Tor-Markern auf der Kante: Mitte der
        # tatsaechlichen (ggf. gebogenen) Linie.
        return self._point_on_edge(edge, 0.5)

    def _self_loop_bbox(self, node):
        # Bounding-Box eines kleinen Kreises oberhalb des Knotens, der eine
        # Selbstschleife (Kante mit node_a == node_b) darstellt - eine normale
        # Linie waere hier eine Strecke der Laenge 0 (unsichtbar).
        x, y = self.node_positions[node]
        cy = y - self.NODE_RADIUS - self.SELF_LOOP_GAP - self.SELF_LOOP_RADIUS
        r = self.SELF_LOOP_RADIUS
        return (x - r, cy - r, x + r, cy + r)

    def _self_loop_anchor(self, node):
        # Punkt auf dem Loop-Kreis, an dem z.B. ein Tor-Symbol platziert wird.
        x, y = self.node_positions[node]
        cy = y - self.NODE_RADIUS - self.SELF_LOOP_GAP - self.SELF_LOOP_RADIUS
        return (x, cy - self.SELF_LOOP_RADIUS)

    def _find_edge_for(self, node, tag):
        for edge in self.edges:
            if (edge["node_a"] == node and edge["tag_a"] == tag) or \
               (edge["node_b"] == node and edge["tag_b"] == tag):
                return edge
        return None

    # ── Lokale Dijkstra (nur fuer Ebene-3-Visualisierung) ───────────────────────

    def _dijkstra_nodes(self, start, goal):
        if start == goal:
            return [start]
        dist = {start: 0}
        prev = {}
        pq = [(0, start)]
        done = set()
        while pq:
            d, node = heapq.heappop(pq)
            if node in done:
                continue
            done.add(node)
            if node == goal:
                break
            for tag, (neighbor, _nt) in self.graph.get(node, {}).items():
                nd = d + 1
                if nd < dist.get(neighbor, float('inf')):
                    dist[neighbor] = nd
                    prev[neighbor] = node
                    heapq.heappush(pq, (nd, neighbor))
        if goal not in dist:
            return None
        path = [goal]
        while path[-1] != start:
            path.append(prev[path[-1]])
        path.reverse()
        return path

    def _build_delivery_route_nodes(self, planned_order):
        if not planned_order:
            return []
        route = [self.delivery_start_node]
        pos = self.delivery_start_node
        for gate_id in planned_order:
            info = self.gate_map.get(gate_id)
            if info is None:
                continue
            entry, tag = info["node"], info["tag"]
            segment = self._dijkstra_nodes(pos, entry)
            if segment is None:
                continue
            route.extend(segment[1:])
            exit_node = self.graph.get(entry, {}).get(tag, [None])[0]
            if exit_node is None:
                continue
            route.append(exit_node)
            pos = exit_node
        return route

    # ── ROS-Callbacks (nur State aktualisieren, NIE tkinter aufrufen) ──────────

    def cbCurrentNode(self, msg):
        self.current_node = msg.data

    def cbCurrentEdge(self, msg):
        try:
            self.current_edge = json.loads(msg.data) if msg.data else None
        except (ValueError, json.JSONDecodeError):
            self.current_edge = None

    def cbVisitedEdges(self, msg):
        try:
            self.visited_edges = json.loads(msg.data)
        except (ValueError, json.JSONDecodeError):
            pass

    def cbGateMap(self, msg):
        try:
            self.gate_map = json.loads(msg.data) if msg.data else {}
        except (ValueError, json.JSONDecodeError):
            pass

    def cbPhase(self, msg):
        self.phase = msg.data

    def cbExplorationDone(self, msg):
        self.exploration_done = msg.data

    def cbDeliveryProgress(self, msg):
        try:
            self.delivery_progress = json.loads(msg.data)
        except (ValueError, json.JSONDecodeError):
            pass

    def cbIntersectionPhase(self, msg):
        self.intersection_phase = msg.data

    def cbIntersectionDirection(self, msg):
        self.intersection_dir = msg.data

    # ── GUI-Aufbau ────────────────────────────────────────────────────────────

    def _build_gui(self):
        self.root = tk.Tk()
        self.root.title("Duckie Graph Dashboard")
        self.root.geometry("1200x650")
        self.root.protocol("WM_DELETE_WINDOW", self.shutdown)

        self.canvas = tk.Canvas(self.root, width=900, height=600, bg="white",
                                 highlightthickness=0)
        self.canvas.pack(side="left", fill="y")

        # Scrollbares Panel: mittlerweile deutlich mehr Widgets (Tor-Reihenfolge,
        # Notfall-Buttons, Status) als in die feste Fensterhoehe passen -
        # "Gefundene Tore"/"Abgefahrene Tore" wachsen ausserdem dynamisch mit
        # jedem neuen Tor. Ohne Scrollbar waeren untere Buttons (z.B. "Delivery
        # starten") schlicht unsichtbar/unklickbar, nicht defekt.
        panel_container = tk.Frame(self.root, width=280)
        panel_container.pack(side="right", fill="both", expand=True)
        panel_canvas = tk.Canvas(panel_container, highlightthickness=0)
        panel_scrollbar = tk.Scrollbar(panel_container, orient="vertical",
                                        command=panel_canvas.yview)
        self.panel = tk.Frame(panel_canvas, width=280)
        self.panel.bind(
            "<Configure>",
            lambda e: panel_canvas.configure(scrollregion=panel_canvas.bbox("all")))
        panel_canvas.create_window((0, 0), window=self.panel, anchor="nw", width=280)
        panel_canvas.configure(yscrollcommand=panel_scrollbar.set)
        panel_canvas.pack(side="left", fill="both", expand=True)
        panel_scrollbar.pack(side="right", fill="y")
        panel_canvas.bind_all(
            "<MouseWheel>", lambda e: panel_canvas.yview_scroll(int(-1 * (e.delta / 120)), "units"))

        self.lbl_phase = tk.Label(self.panel, anchor="w", justify="left")
        self.lbl_phase.pack(fill="x", padx=10, pady=(10, 2))
        self.lbl_position = tk.Label(self.panel, anchor="w", justify="left")
        self.lbl_position.pack(fill="x", padx=10, pady=2)
        self.lbl_edge = tk.Label(self.panel, anchor="w", justify="left", wraplength=260)
        self.lbl_edge.pack(fill="x", padx=10, pady=2)
        self.lbl_progress = tk.Label(self.panel, anchor="w", justify="left", wraplength=260)
        self.lbl_progress.pack(fill="x", padx=10, pady=2)
        self.lbl_gates = tk.Label(self.panel, anchor="w", justify="left", wraplength=260)
        self.lbl_gates.pack(fill="x", padx=10, pady=2)
        self.lbl_planned = tk.Label(self.panel, anchor="w", justify="left", wraplength=260)
        self.lbl_planned.pack(fill="x", padx=10, pady=2)

        ttk.Separator(self.panel, orient="horizontal").pack(fill="x", padx=10, pady=10)

        # Sichtbare Klick-Bestaetigung: die Buttons unten loesen nur einen
        # rospy.loginfo im Terminal aus, das GUI selbst zeigte bisher keine
        # Rueckmeldung - wirkte dadurch so, als wuerde der Klick nichts tun.
        self.lbl_action_feedback = tk.Label(self.panel, anchor="w", justify="left",
                                             wraplength=260, fg="#1a7a1a")
        self.lbl_action_feedback.pack(fill="x", padx=10, pady=(0, 6))
        self._feedback_after_id = None

        tk.Label(self.panel, text="Vorgegebene Tor-Reihenfolge (z.B. 5,9,3):",
                 anchor="w", justify="left", wraplength=260).pack(fill="x", padx=10)
        self.entry_gate_order = tk.Entry(self.panel)
        self.entry_gate_order.insert(0, ",".join(self._gate_order_cfg))
        self.entry_gate_order.pack(fill="x", padx=10, pady=(2, 4))
        tk.Button(self.panel, text="Reihenfolge übernehmen",
                  command=self._on_apply_gate_order).pack(fill="x", padx=10, pady=(0, 10))

        tk.Label(self.panel,
                 text="Notfall: Tor-Zuordnung in mapping_node.json (Feld "
                      "\"gate_map\") von Hand tauschen/entfernen, dann:",
                 anchor="w", justify="left", wraplength=260).pack(fill="x", padx=10)
        tk.Button(self.panel, text="Tor-Zuordnung neu laden",
                  command=self._on_reload_gate_map).pack(fill="x", padx=10, pady=(2, 10))

        tk.Label(self.panel,
                 text="Erkundung wiederholen (z.B. Tor übersehen) - Karte "
                      "bleibt, alle Kanten gelten wieder als unbesucht:",
                 anchor="w", justify="left", wraplength=260).pack(fill="x", padx=10)
        tk.Button(self.panel, text="Erkundung neu starten",
                  command=self._on_reset_exploration).pack(fill="x", padx=10, pady=(2, 10))

        ttk.Separator(self.panel, orient="horizontal").pack(fill="x", padx=10, pady=10)

        tk.Label(self.panel,
                 text="Bot jetzt von Hand an delivery_start_node hingestellt "
                      "(erst DANACH klicken - sonst plant/faehrt der Bot noch "
                      "ab der alten Erkundungs-Position):",
                 anchor="w", justify="left", wraplength=260).pack(fill="x", padx=10)
        tk.Button(self.panel, text="Bot versetzt", bg="#DDAA00",
                  command=self._on_bot_relocated).pack(fill="x", padx=10, pady=(2, 10))

        ttk.Separator(self.panel, orient="horizontal").pack(fill="x", padx=10, pady=10)

        self.lbl_ready = tk.Label(self.panel, anchor="w", justify="left", wraplength=260)
        self.lbl_ready.pack(fill="x", padx=10, pady=(0, 4))

        btn_frame = tk.Frame(self.panel, width=200, height=50)
        btn_frame.pack(padx=10, pady=(0, 10))
        btn_frame.pack_propagate(False)
        self.btn_start_delivery = tk.Button(
            btn_frame, text="Delivery starten", bg="#44AA44", fg="white",
            state="disabled", command=self._on_start_delivery_click)
        self.btn_start_delivery.pack(fill="both", expand=True)

        self.lbl_delivered = tk.Label(self.panel, anchor="w", justify="left", wraplength=260)
        self.lbl_delivered.pack(fill="x", padx=10, pady=2)

        self._draw_static_graph()
        self.root.after(200, self.update_canvas)

    def _show_action_feedback(self, text):
        if self._feedback_after_id is not None:
            self.root.after_cancel(self._feedback_after_id)
        self.lbl_action_feedback.config(text=f"✓ {text}")
        self._feedback_after_id = self.root.after(
            3000, lambda: self.lbl_action_feedback.config(text=""))

    def _on_start_delivery_click(self):
        self.pub_start_delivery.publish(Bool(data=True))
        rospy.loginfo("[debug_graph] 'Delivery starten' gedrueckt")
        self._show_action_feedback("Delivery gestartet")

    def _on_reload_gate_map(self):
        self.pub_reload_gate_map.publish(Bool(data=True))
        rospy.loginfo("[debug_graph] 'Tor-Zuordnung neu laden' gedrueckt")
        self._show_action_feedback("Tor-Zuordnung neu geladen")

    def _on_reset_exploration(self):
        self.pub_reset_exploration.publish(Bool(data=True))
        rospy.loginfo("[debug_graph] 'Erkundung neu starten' gedrueckt")
        self._show_action_feedback("Erkundung zurueckgesetzt")

    def _on_bot_relocated(self):
        self.pub_bot_relocated.publish(Bool(data=True))
        rospy.loginfo("[debug_graph] 'Bot versetzt' gedrueckt")
        self._show_action_feedback("Bot-Neupositionierung bestaetigt")

    def _on_apply_gate_order(self):
        # Eingabe wie "5, 9, 3" -> ["5","9","3"]; leeres Feld -> [] (keine
        # Vorgabe, path_planner_node optimiert die Reihenfolge wieder selbst).
        raw = self.entry_gate_order.get()
        order = [g.strip() for g in raw.replace(";", ",").split(",") if g.strip()]
        self._gate_order_cfg = order
        self._save_gate_order(order)
        self.pub_gate_order.publish(String(data=json.dumps(order)))
        rospy.loginfo(f"[debug_graph] Vorgegebene Tor-Reihenfolge uebernommen: {order}")
        self._show_action_feedback(f"Reihenfolge uebernommen: {order}" if order
                                    else "Reihenfolge geloescht (Auto-Optimierung aktiv)")

    def _save_gate_order(self, order):
        # In mapping_node.json zurueckschreiben, damit path_planner_node die
        # Vorgabe auch nach einem Neustart (ohne laufendes Dashboard) sofort
        # aus der Config kennt, statt nur ueber das Live-Topic.
        path = os.path.join(os.path.dirname(__file__), "../config/mapping_node.json")
        try:
            with open(path, 'r') as f:
                config = json.load(f)
            config.setdefault("path_planning", {})["gate_order"] = order
            with open(path, 'w') as f:
                json.dump(config, f, indent=2)
        except Exception as e:
            rospy.logwarn(f"[debug_graph] Konnte gate_order nicht speichern: {e}")

    # ── Canvas: Ebene 1 (statisch, einmalig) ────────────────────────────────────

    def _draw_edge_tag_label(self, x, y, text):
        # Weisses "Badge" hinter der Tag-Zahl, damit sie sich von der grauen
        # Linie darunter und von benachbarten Labels (parallele Kanten) klar
        # abhebt, statt direkt auf der Linie zu "verschwimmen".
        r = 9
        self.canvas.create_oval(x - r, y - r, x + r, y + r,
                                 fill="white", outline="#AAAAAA")
        self.canvas.create_text(x, y, text=text, fill="#222222",
                                 font=("Arial", 9, "bold"))

    def _draw_static_graph(self):
        for edge in self.edges:
            if edge["node_a"] == edge["node_b"]:
                self.canvas.create_oval(*self._self_loop_bbox(edge["node_a"]),
                                         outline="#555555", width=2)
                ax, ay = self._self_loop_anchor(edge["node_a"])
                label = f'{edge["tag_a"]}/{edge["tag_b"]}'
                self.canvas.create_rectangle(ax - 14, ay - 19, ax + 14, ay - 1,
                                              fill="white", outline="#AAAAAA")
                self.canvas.create_text(ax, ay - 10, text=label, fill="#222222",
                    font=("Arial", 8, "bold"))
                continue
            pts = self._edge_line_points(edge)
            self.canvas.create_line(*pts, fill="#555555", width=2, smooth=True)
            # Tag-Nummer nahe jedem Kantenende: zeigt, ueber welchen Eingangs-Tag
            # man den jeweils GEGENUEBERLIEGENDEN Knoten erreicht. Liegt auf der
            # tatsaechlichen (ggf. gebogenen) Linie, nicht auf der Geraden.
            ax, ay = self._point_on_edge(edge, 0.22)
            bx, by = self._point_on_edge(edge, 0.78)
            self._draw_edge_tag_label(ax, ay, edge["tag_a"])
            self._draw_edge_tag_label(bx, by, edge["tag_b"])
        for node, (x, y) in self.node_positions.items():
            self.canvas.create_oval(x - 20, y - 20, x + 20, y + 20,
                                     fill="#666666", outline="")
            self.canvas.create_text(x, y, text=node, fill="white")

    # ── Canvas: Ebene 2/3 + dynamische Elemente (bei jedem Update neu) ─────────

    def _redraw_visited(self):
        self.canvas.delete("visited")
        visited_set = {tuple(e) for e in self.visited_edges}
        for edge in self.edges:
            if (edge["node_a"], edge["tag_a"]) not in visited_set:
                continue
            if edge["node_a"] == edge["node_b"]:
                self.canvas.create_oval(*self._self_loop_bbox(edge["node_a"]),
                                         outline="#00CC44", width=3, tags="visited")
                continue
            pts = self._edge_line_points(edge)
            self.canvas.create_line(*pts, fill="#00CC44", width=3, smooth=True,
                                     tags="visited")

    def _redraw_planned_path(self):
        self.canvas.delete("planned")
        planned_order = self.delivery_progress.get("planned_order", [])
        route = self._build_delivery_route_nodes(planned_order)
        for i in range(len(route) - 1):
            a, b = route[i], route[i + 1]
            if a not in self.node_positions or b not in self.node_positions:
                continue
            if a == b:
                # Route fuehrt ueber eine Selbstschleife (Wendeschleife)
                self.canvas.create_oval(*self._self_loop_bbox(a),
                                         outline="#4488FF", width=3, dash=(8, 4),
                                         tags="planned")
                continue
            # Delivery-Route kennt nur die Knotenfolge, kein konkretes Tag
            # (Ebene-3-Visualisierung, siehe Kopfkommentar) - nimmt daher
            # irgendeine der ggf. mehreren parallelen Kanten zwischen a und b.
            edge = self._any_edge_for_pair(a, b)
            pts = self._edge_line_points(edge) if edge else [
                *self.node_positions[a], *self.node_positions[b]]
            self.canvas.create_line(*pts, fill="#4488FF", width=3, smooth=True,
                                     dash=(8, 4), arrow=tk.LAST, tags="planned")

    def _redraw_gates(self):
        self.canvas.delete("gates")
        delivered_set = set(self.delivery_progress.get("done", []))
        for gate_id, info in self.gate_map.items():
            edge = self._find_edge_for(info.get("node"), info.get("tag"))
            if edge is None:
                continue
            if edge["node_a"] == edge["node_b"]:
                mx, my = self._self_loop_anchor(edge["node_a"])
            else:
                mx, my = self._edge_anchor_point(edge)
            try:
                color = self.GATE_COLORS.get(int(gate_id), "#FFFFFF")
            except ValueError:
                color = "#FFFFFF"
            self.canvas.create_rectangle(mx - 6, my - 10, mx + 6, my + 10,
                                          fill=color, outline="black", tags="gates")
            self.canvas.create_text(mx, my + 18, text=gate_id, fill="black", tags="gates")
            if gate_id in delivered_set:
                self.canvas.create_text(mx, my - 16, text="✓", fill="black",
                                         font=("Arial", 12, "bold"), tags="gates")

    def _redraw_bot(self):
        self.canvas.delete("bot")
        if self.current_node in self.node_positions:
            x, y = self.node_positions[self.current_node]
            self.canvas.create_oval(x - 24, y - 24, x + 24, y + 24,
                                     fill="#FFCC00", outline="", tags="bot")
            self.canvas.create_text(x, y, text=self.current_node, fill="black", tags="bot")
            if self.intersection_phase == "Turning" and self.intersection_dir:
                self.canvas.create_text(
                    x, y - 36, text=f"biegt: {self.intersection_dir.upper()}",
                    fill="#CC00CC", font=("Arial", 10, "bold"), tags="bot")

    # ── Status-Panel ─────────────────────────────────────────────────────────

    def _update_labels(self):
        self.lbl_phase.config(text=f"Phase: {self.phase}")
        self.lbl_position.config(text=f"Bot-Position: {self.current_node or '-'}")

        edge_txt = "-"
        if self.current_edge:
            frm = self.current_edge.get("from")
            tag = self.current_edge.get("tag")
            neighbor = self.graph.get(frm, {}).get(tag, [None])[0]
            if neighbor:
                edge_txt = f"{frm} → {neighbor} (Tag {tag})"
        self.lbl_edge.config(text=f"Aktuelle Kante: {edge_txt}")

        self.lbl_progress.config(
            text=f"Karten-Fortschritt: {len(self.visited_edges)} / {len(self.edges)} Kanten besucht")

        if self.gate_map:
            gate_lines = [f"  {gid}: {info.get('node')} / Tag {info.get('tag')}"
                          for gid, info in sorted(self.gate_map.items())]
        else:
            gate_lines = ["  -"]
        self.lbl_gates.config(text="Gefundene Tore:\n" + "\n".join(gate_lines))

        planned = self.delivery_progress.get("planned_order", [])
        self.lbl_planned.config(
            text="Geplante Reihenfolge:\n  " + (", ".join(planned) if planned else "-"))

        done = self.delivery_progress.get("done", [])
        if done:
            delivered_txt = "\n".join(f"  ✓ {g}" for g in done)
        else:
            delivered_txt = "  -"
        self.lbl_delivered.config(text="Abgefahrene Tore:\n" + delivered_txt)

        # Bereit zur Abfahrt heisst nicht nur "Exploration fertig" (alle
        # Kanten befahren), sondern bei vorgegebener Reihenfolge zusaetzlich
        # "path_planner_node konnte tatsaechlich eine Route planen" - fehlt
        # dort noch ein vorgegebenes Tor (missing_gates), bleibt der Button
        # bewusst deaktiviert statt wirkungslos klickbar zu sein.
        missing = self.delivery_progress.get("missing_gates", [])
        bot_relocated = self.delivery_progress.get("bot_relocated_confirmed", False)
        ready = self.exploration_done and bot_relocated and bool(planned)
        if not self.exploration_done:
            ready_txt = "Status: Erkunde Karte..."
        elif not bot_relocated:
            ready_txt = "Status: Warte auf 'Bot versetzt' (Bot an delivery_start_node stellen, dann klicken)"
        elif missing:
            ready_txt = "Status: Warte auf Tor(e) " + ", ".join(missing)
        elif ready:
            ready_txt = "Status: Alle Tore gefunden und gemappt ✓"
        else:
            ready_txt = "Status: Keine Tore gefunden"
        self.lbl_ready.config(text=ready_txt, fg="#008800" if ready else "#AA0000")

        self.btn_start_delivery.config(state="normal" if ready else "disabled")

    # ── Haupt-Update (Main-Thread, 5 Hz) ─────────────────────────────────────

    def update_canvas(self):
        self._redraw_visited()

        planned_order = self.delivery_progress.get("planned_order", [])
        if planned_order != self._last_drawn_planned_order:
            self._redraw_planned_path()
            self._last_drawn_planned_order = list(planned_order)

        self._redraw_gates()
        self._redraw_bot()
        self._update_labels()

        # Deutliche, einmalige Meldung beim Uebergang False->True (nicht bei
        # jedem Tick) - der Bot steht zu diesem Zeitpunkt bereits an der
        # naechsten Kreuzung (next_direction wurde geleert, siehe
        # explore_control_node) und wartet auf "Delivery starten".
        if self.exploration_done and not self._exploration_done_notified:
            self._exploration_done_notified = True
            messagebox.showinfo(
                "Erkundung abgeschlossen",
                "Alle Kanten wurden abgefahren. Der Bot steht und wartet.\n\n"
                "Bitte gefundene Tore/Reihenfolge pruefen und anschliessend "
                "\"Delivery starten\" klicken.")
        elif not self.exploration_done:
            self._exploration_done_notified = False

        self.root.after(200, self.update_canvas)

    def shutdown(self):
        rospy.signal_shutdown("GUI geschlossen")
        cv2.destroyAllWindows()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


if __name__ == '__main__':
    node = DebugGraphNode('debug_graph_node')
    node.run()
