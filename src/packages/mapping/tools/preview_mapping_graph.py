#!/usr/bin/env python3
"""Standalone Vorschau fuer eine (ggf. von Hand editierte) mapping_node.json.

Zeichnet Knoten, Kanten, Tor-Zuordnungen und Wegpunkte GENAUSO wie
debug_graph_node.py, aber ohne ROS/rospy - laesst sich also direkt vor Ort
mit reinem Python starten, um eine Aenderung an mapping_node.json (neue
Kreuzung, verschobenes Tor, neue Wegpunkte) kurz visuell zu pruefen, bevor
der ganze Stack hochgefahren wird.

Nutzung:
    python3 preview_mapping_graph.py [pfad/zu/mapping_node.json]

Ohne Argument wird die mapping_node.json aus config/ im mapping-Package
verwendet. Ein Screenshot wird zusaetzlich als PNG neben der json-Datei
abgelegt (z.B. mapping_node_preview.png), fuer den Fall dass kein/nur ein
langsames X-Forwarding zur Verfuegung steht.
"""
import json
import math
import os
import sys
import tkinter as tk

if sys.platform == "win32":
    # Ohne DPI-Awareness rechnet Tkinter (winfo_*) in logischen Pixeln,
    # ImageGrab aber in physischen Bildschirm-Pixeln - bei Windows-
    # Anzeigeskalierung (z.B. 125%) landet der Screenshot dadurch verschoben
    # und zu gross (u.a. mit sichtbarer Taskleiste). Fix: Prozess als
    # DPI-aware markieren, dann nutzen beide dieselbe Pixel-Basis.
    try:
        import ctypes
        ctypes.windll.shcore.SetProcessDpiAwareness(1)
    except Exception:
        pass

NODE_RADIUS = 20
SELF_LOOP_RADIUS = 18
SELF_LOOP_GAP = 4
GATE_COLORS = {
    5: "#FF00FF", 6: "#00FFFF", 7: "#FF8800", 8: "#FFFF00", 9: "#FF0000",
    10: "#AA00FF", 11: "#88FF00", 12: "#FF44AA", 13: "#00FFAA",
}


def default_config_path():
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(here, "..", "config", "mapping_node.json")


def load_config(path):
    with open(path, 'r') as f:
        return json.load(f)


def build_edges(graph):
    seen = set()
    edges = []
    for node, exits in graph.items():
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


def compute_node_positions(graph, node_positions_cfg):
    nodes = sorted(graph.keys())
    n = len(nodes)
    cx, cy, r = 450, 300, 220
    positions = {}
    for i, node in enumerate(nodes):
        angle = math.radians(360.0 * i / n) if n else 0.0
        positions[node] = (cx + r * math.sin(angle), cy - r * math.cos(angle))
    for node, pos in node_positions_cfg.items():
        if node in positions:
            positions[node] = tuple(pos)
    return positions


def validate(graph, gate_map, node_positions_cfg):
    """Prueft Referenzen, die debug_graph_node.py stillschweigend ignorieren
    wuerde (fehlender Knoten faellt z.B. einfach auf Kreislayout zurueck) -
    hier soll das aber laut gemeldet werden, damit Tippfehler beim
    Von-Hand-Editieren sofort auffallen."""
    problems = []
    nodes = set(graph.keys())
    for node, exits in graph.items():
        for tag, (neighbor, _neighbor_tag) in exits.items():
            if neighbor not in nodes:
                problems.append(
                    f"Knoten {node}, Tag {tag}: Nachbar '{neighbor}' ist kein Knoten in \"graph\".")
    for node in node_positions_cfg:
        if node not in nodes:
            problems.append(
                f"debug_layout.node_positions nennt Knoten '{node}', der nicht in \"graph\" existiert.")
    for gate_id, info in gate_map.items():
        node, tag = info.get("node"), info.get("tag")
        if node not in nodes:
            problems.append(f"gate_map[{gate_id}]: Knoten '{node}' existiert nicht in \"graph\".")
            continue
        if tag not in graph.get(node, {}):
            problems.append(f"gate_map[{gate_id}]: Tag '{tag}' existiert nicht an Knoten '{node}'.")
    return problems


def draw_edge_tag_label(canvas, x, y, text):
    r = 9
    canvas.create_oval(x - r, y - r, x + r, y + r, fill="white", outline="#AAAAAA")
    canvas.create_text(x, y, text=text, fill="#222222", font=("Arial", 8, "bold"))


def point_on_edge(control_points, t):
    pts = control_points
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


def self_loop_bbox(node_positions, node):
    x, y = node_positions[node]
    cy = y - NODE_RADIUS - SELF_LOOP_GAP - SELF_LOOP_RADIUS
    r = SELF_LOOP_RADIUS
    return (x - r, cy - r, x + r, cy + r)


def self_loop_anchor(node_positions, node):
    x, y = node_positions[node]
    cy = y - NODE_RADIUS - SELF_LOOP_GAP - SELF_LOOP_RADIUS
    return (x, cy - SELF_LOOP_RADIUS)


def find_edge_for(edges, node, tag):
    for edge in edges:
        if (edge["node_a"] == node and edge["tag_a"] == tag) or \
           (edge["node_b"] == node and edge["tag_b"] == tag):
            return edge
    return None


def draw(canvas, graph, edges, node_positions, edge_waypoints_cfg, gate_map):
    for edge in edges:
        # Kante referenziert einen Knoten, der nicht in "graph" existiert
        # (Tippfehler beim Editieren) - bereits als Warnung gemeldet, hier nur
        # ueberspringen statt abzustuerzen, damit der Rest des Graphen trotzdem
        # angezeigt wird.
        if edge["node_a"] not in node_positions or edge["node_b"] not in node_positions:
            continue
        if edge["node_a"] == edge["node_b"]:
            canvas.create_oval(*self_loop_bbox(node_positions, edge["node_a"]),
                                outline="#555555", width=2)
            ax, ay = self_loop_anchor(node_positions, edge["node_a"])
            label = f'{edge["tag_a"]}/{edge["tag_b"]}'
            canvas.create_rectangle(ax - 14, ay - 19, ax + 14, ay - 1,
                                     fill="white", outline="#AAAAAA")
            canvas.create_text(ax, ay - 10, text=label, fill="#222222",
                                font=("Arial", 8, "bold"))
            continue
        key = f'{edge["node_a"]}{edge["tag_a"]}-{edge["node_b"]}{edge["tag_b"]}'
        waypoints = [tuple(p) for p in edge_waypoints_cfg.get(key, [])]
        control_points = [node_positions[edge["node_a"]]] + waypoints + [node_positions[edge["node_b"]]]
        pts = []
        for x, y in control_points:
            pts.extend([x, y])
        canvas.create_line(*pts, fill="#555555", width=2, smooth=True)
        ax, ay = point_on_edge(control_points, 0.22)
        bx, by = point_on_edge(control_points, 0.78)
        draw_edge_tag_label(canvas, ax, ay, edge["tag_a"])
        draw_edge_tag_label(canvas, bx, by, edge["tag_b"])

    for node, (x, y) in node_positions.items():
        canvas.create_oval(x - NODE_RADIUS, y - NODE_RADIUS, x + NODE_RADIUS, y + NODE_RADIUS,
                            fill="#666666", outline="")
        canvas.create_text(x, y, text=node, fill="white")

    for gate_id, info in gate_map.items():
        edge = find_edge_for(edges, info.get("node"), info.get("tag"))
        if edge is None:
            continue
        if edge["node_a"] == edge["node_b"]:
            mx, my = self_loop_anchor(node_positions, edge["node_a"])
        else:
            key = f'{edge["node_a"]}{edge["tag_a"]}-{edge["node_b"]}{edge["tag_b"]}'
            waypoints = [tuple(p) for p in edge_waypoints_cfg.get(key, [])]
            control_points = [node_positions[edge["node_a"]]] + waypoints + [node_positions[edge["node_b"]]]
            mx, my = point_on_edge(control_points, 0.5)
        try:
            color = GATE_COLORS.get(int(gate_id), "#FFFFFF")
        except ValueError:
            color = "#FFFFFF"
        canvas.create_rectangle(mx - 6, my - 10, mx + 6, my + 10,
                                 fill=color, outline="black")
        canvas.create_text(mx, my + 18, text=gate_id, fill="black")


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else default_config_path()
    path = os.path.abspath(path)
    try:
        config = load_config(path)
    except (OSError, json.JSONDecodeError) as e:
        print(f"FEHLER beim Laden von {path}:\n  {e}")
        sys.exit(1)

    graph = config.get("graph", {})
    gate_map = config.get("gate_map", {})
    node_positions_cfg = config.get("debug_layout", {}).get("node_positions", {})
    edge_waypoints_cfg = config.get("debug_layout", {}).get("edge_waypoints", {})

    problems = validate(graph, gate_map, node_positions_cfg)
    print(f"Geladen: {path}")
    print(f"  Knoten: {sorted(graph.keys())}")
    print(f"  Tore in gate_map: {sorted(gate_map.keys()) if gate_map else '(keine)'}")
    if problems:
        print("WARNUNGEN:")
        for p in problems:
            print(f"  - {p}")
    else:
        print("  Keine Auffaelligkeiten gefunden.")

    edges = build_edges(graph)
    node_positions = compute_node_positions(graph, node_positions_cfg)

    root = tk.Tk()
    root.title(f"mapping_node.json Vorschau - {os.path.basename(path)}")
    # Feste Position: verhindert, dass das Fenster (je nach Fenstermanager-
    # Default) so weit unten/rechts landet, dass Taskleiste o.ae. in den
    # Screenshot hineinragt.
    root.geometry("900x600+50+50")
    canvas = tk.Canvas(root, width=900, height=600, bg="white", highlightthickness=0)
    canvas.pack()
    if problems:
        canvas.create_text(450, 15, text=f"{len(problems)} Warnung(en) - siehe Terminal",
                            fill="#CC0000", font=("Arial", 11, "bold"))

    draw(canvas, graph, edges, node_positions, edge_waypoints_cfg, gate_map)
    root.update()

    screenshot_path = os.path.splitext(path)[0] + "_preview.png"
    try:
        from PIL import ImageGrab
        x0 = root.winfo_rootx() + canvas.winfo_x()
        y0 = root.winfo_rooty() + canvas.winfo_y()
        x1 = x0 + canvas.winfo_width()
        y1 = y0 + canvas.winfo_height()
        ImageGrab.grab(bbox=(x0, y0, x1, y1)).save(screenshot_path)
        print(f"Screenshot gespeichert: {screenshot_path}")
    except Exception as e:
        print(f"Screenshot konnte nicht gespeichert werden ({e}) - Fenster bleibt trotzdem offen.")

    root.mainloop()


if __name__ == "__main__":
    main()
